/*
 *  Linux Network driver over P4bus (hardware virtualisation)
 * 
 *  Copyright (C) 2014 SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

/* ------------------------- FILE INCLUSION -------------------------------- */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/interrupt.h>

#include "compat.h"
#include "p4bus.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF				"P4BUS_vmnet: "
#define DRIVER_NAME			"vmnet"
#define DRIVER_COMPAT		"p4bus,vmnet"

/* todo: add module parameter for that */
#ifdef MODULE
#define MAC_ADDR_PREFIX				1234
#else
#define MAC_ADDR_PREFIX				CONFIG_P4BUS_VMNET_MAC_ADDR_PREFIX 	/* from Kernel configuration */
#endif

/* following IOCTL_CMD.. macros are inspired from drv_net_api.h (see drvApi) */
#define IOCTL_CMD_GET_MAC_ADDR		3

/* new IOCTL */
#define DRV_IOC_CLASS_MASK     0x3FUL
#define DRV_IOC_CLASS_SHIFT    10UL
#define DRV_IOC_ID_MASK        0x3FFUL
#define DRV_IOC_ID_SHIFT       0UL
#define DRV_IOC_GROUP_MASK     0x7UL
#define DRV_IOC_GROUP_SHIFT    7UL
#define DRV_IOC_NUM_MASK       0x7FUL
#define DRV_IOC_NUM_SHIFT      0UL

#define DRV_IOC_MK_ID(class, group, num) \
    ((((class) & DRV_IOC_CLASS_MASK) << DRV_IOC_CLASS_SHIFT) | \
    (((group) & DRV_IOC_GROUP_MASK) << DRV_IOC_GROUP_SHIFT) | \
    (((num) & DRV_IOC_NUM_MASK) << DRV_IOC_NUM_SHIFT))

#define NEW_IOCTL_CMD_GET_MAC_ADDR DRV_IOC_MK_ID(8, 0x0u, 0x2u)


#ifndef FALSE
#define	FALSE				0
#endif
#ifndef TRUE
#define	TRUE				1
#endif

#define CLOSED				0
#define OPEN				1

#define MTU_DEFAULT_SIZE	1500


//#define DEBUG
#ifdef DEBUG
#define DBG(args...)	do { \
							printk(KERN_ERR PREF "(%s:%d): ", __func__,__LINE__); \
							printk(args); \
						} while (0)
#else
#define DBG(args...)	do {} while(0)
#endif

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

typedef struct p4bus_vmnet_deviceinfo_s
{
	/* device related stuff*/
	struct net_device *netdev;

	/* netdevice stat info */
	struct net_device_stats stats;

    /* IORINGS opration */
    uint32_t    rx_ioring_id;
    p4bus_ioring_t rx_ioring_op;
    spinlock_t rx_lock;

    uint32_t    tx_ioring_id;
    p4bus_ioring_t tx_ioring_op;
    spinlock_t tx_lock;

	/* p4bus device */
    p4bus_device_info_t *p4bus_device;

    /* platform device */
    struct platform_device *pdev;

	/* wait queue (used for open/close/ioctl) */
	wait_queue_head_t wait_op;

    /* flags */
	volatile uint32_t isRunning;

}p4bus_vmnet_deviceinfo_t;

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int p4bus_vmnet_probe(struct platform_device *pdev);

static void netdev_init(struct net_device *dev);

static int p4bus_vmnet_open(struct net_device *netdev);

static int p4bus_vmnet_close(struct net_device *netdev);

static int p4bus_vmnet_get_mac_addr(p4bus_vmnet_deviceinfo_t *currDev, uint64_t ioctlnum);

static int p4bus_vmnet_tx(struct sk_buff *skb, struct net_device *netdev);

static int p4bus_vmnet_validate_addr(struct net_device *dev);

static int p4bus_vmnet_set_mac_address(struct net_device *dev, void *addr);

static int p4bus_vmnet_remove(struct platform_device *pdev);

static void p4bus_vmnet_rx(p4bus_vmnet_deviceinfo_t *currDev, int from_irq);

static irqreturn_t p4bus_vmnet_interrupt(int irq, void *priv);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */
static const struct of_device_id p4bus_vmnet_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, p4bus_vmnet_match);


static const struct net_device_ops p4bus_vmnet_netdev_ops = {
	.ndo_open				= p4bus_vmnet_open,
	.ndo_stop				= p4bus_vmnet_close,
	.ndo_start_xmit			= p4bus_vmnet_tx,
	.ndo_validate_addr		= p4bus_vmnet_validate_addr,
	.ndo_set_mac_address	= p4bus_vmnet_set_mac_address,
};

static struct header_ops p4bus_vmnet_eth_header_ops ____cacheline_aligned = {
	.create         = eth_header,
	.parse          = eth_header_parse,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,1,0)
	.rebuild        = eth_rebuild_header,
#endif
	.cache          = eth_header_cache,
	.cache_update   = eth_header_cache_update,
};

static struct platform_driver p4bus_vmnet_driver = {
	.driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = p4bus_vmnet_match,
    },
	.probe  = p4bus_vmnet_probe,
	.remove = p4bus_vmnet_remove,
};

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

module_platform_driver(p4bus_vmnet_driver);

MODULE_ALIAS("platform:vmnet");
MODULE_AUTHOR("Benjamin Reynaud <bre@sysgo.fr>");
MODULE_DESCRIPTION("Network driver on PikeOS Virtualization bus");
MODULE_LICENSE("GPL");

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/**
 *  @purpose
 *    p4bus-vmnet IRQ handler. An IRQ is raised when an operation
 *    from the ioring has been processed.
 *
 *  @param irq  IRQ number
 *  @param priv pointer on a private structure
 *
 *  @returns 
 */
static irqreturn_t p4bus_vmnet_interrupt(int irq, void *priv)
{
    p4bus_operation_t *curr_op;
    p4bus_vmnet_deviceinfo_t *currDev = priv;
    int refill_rx = 0;
    int signal_rx = 0;
    struct platform_device *pdev = currDev->pdev;

    /* check for direct operations */
	if (waitqueue_active(&(currDev->wait_op)))
	{
		wake_up_interruptible(&(currDev->wait_op));
	}

	/* handle receives */
    do {
        /* Check IORING */
    	spin_lock(&currDev->rx_lock);
        curr_op = p4bus_ioring_get_done(&(currDev->rx_ioring_op));
        spin_unlock(&currDev->rx_lock);
        if (curr_op != NULL)
        {
			struct sk_buff *skb = (struct sk_buff *)(unsigned long)curr_op->priv_guest;
			/* if device not running, just clean */
			if (currDev->isRunning && curr_op->retcode == P4BUS_E_OK &&
					curr_op->size > 0)
			{
				/* we can push the packet */
				if (skb == NULL)
				{
					currDev->stats.rx_dropped++;
					dev_err(&(pdev->dev), PREF "%s receive packet with no skb\n",__func__);
				}
				else
				{
					skb_put(skb, curr_op->size);
					curr_op->priv_guest = 0;
					skb->dev = currDev->netdev;
					skb->protocol = eth_type_trans(skb, currDev->netdev);
					skb->ip_summed = CHECKSUM_UNNECESSARY;
					currDev->netdev->stats.rx_packets++;
					currDev->netdev->stats.rx_bytes += curr_op->size;
					netif_rx(skb);
				}
			}

			if (curr_op->flags & OP_FLAGS_SLEEP)
				signal_rx = 1;
			/* op is free */
			p4bus_ioring_push_free(curr_op);

			/* something received so rx can be refilled */
			refill_rx = 1;
        }
    } while (curr_op != NULL);

    /* push buffers in rx ring if we received stuff */
    if (currDev->isRunning && refill_rx)
    {
    	p4bus_vmnet_rx(currDev, TRUE);

    	if (signal_rx)
    		p4bus_command_signal_ioring(currDev->rx_ioring_id);
    }

    /* handle done for transmit */

	/* normal case, we are running */
	if (p4bus_ioring_has_done(&(currDev->tx_ioring_op)) &&
			currDev->isRunning)
	{
		netif_wake_queue(currDev->netdev);
	}
	else if (!currDev->isRunning)
	{
		spin_lock(&currDev->tx_lock);
		/* free the skb */
		do {
			curr_op = p4bus_ioring_get_done(&(currDev->tx_ioring_op));
			if (curr_op != NULL) {
				dev_kfree_skb_irq((struct sk_buff *)(unsigned long)curr_op->priv_guest);
				curr_op->priv_guest = 0;
			}
		} while (curr_op != NULL);
		spin_unlock(&currDev->tx_lock);
	}

    return IRQ_HANDLED;
}

/**
 *  @purpose
 *    transmits a packet to the host
 *
 *  @param skb   	socket buffer, contains data to transfert
 *  @param netdev   network device concerned by the operation
 *
 *  @returns
 *
 *  @retval -EIO  		if operation failed
 *  @retval 0		 	if no error
 */
static int p4bus_vmnet_tx(struct sk_buff *skb, struct net_device *netdev)
{
	p4bus_vmnet_deviceinfo_t *priv = netdev_priv(netdev);
    p4bus_operation_t *curr_op;
    unsigned long flags;
    int signal = 1;

#if 0
	/* enable this to look the data */
	for (int i=0 ; i< skb->len; i++)
	{
		printk(" %02x",skb->data[i]&0xff);
		printk("\n");
	}
#endif

	/* free done operations */
	while (p4bus_ioring_has_done(&(priv->tx_ioring_op)))
	{
		spin_lock_irqsave(&(priv->tx_lock), flags);
		curr_op = p4bus_ioring_get_done(&(priv->tx_ioring_op));
		spin_unlock_irqrestore(&(priv->tx_lock), flags);
		if (curr_op != NULL)
		{
			consume_skb((struct sk_buff *)(unsigned long)curr_op->priv_guest);
			curr_op->priv_guest = 0;

			/* if device not running, just clean */
			if (!priv->isRunning)
			{
				/* device not running just ignore */
			}
			else if (curr_op->retcode != P4BUS_E_OK)
			{
				dev_err(&(priv->pdev->dev), PREF "%s Error during WRITE operation %d\n",priv->p4bus_device->name,curr_op->retcode);
			}
			else
			{
				/* update stats */
				netdev->stats.tx_packets++;
				netdev->stats.tx_bytes += curr_op->size;
			}

			signal = ((curr_op->flags & OP_FLAGS_SLEEP) != 0);

			/* op is free */
			p4bus_ioring_push_free(curr_op);
		}
	}

	/* request a FREE operation in the TX ioring */
	spin_lock_irqsave(&(priv->tx_lock), flags);
	curr_op = p4bus_ioring_get_free(&(priv->tx_ioring_op));
	if (curr_op != NULL)
	{
		/* check if we will need to signal or not */
		if (!p4bus_ioring_has_free(&(priv->tx_ioring_op)))
		{
			/* ask to be signal */
			curr_op->flags = OP_FLAGS_SIGNAL;

			/* stop the transmit queue */
			netif_stop_queue(priv->netdev);
		}
		else
		{
			/* no need to signal us */
			curr_op->flags = 0;
		}
	}
	spin_unlock_irqrestore(&(priv->tx_lock), flags);

	/* we should never be out of operations if we are here */
	if (curr_op == NULL) {
		dev_err(&(priv->pdev->dev), PREF "%s Transmit call with a FULL ioring !!!!\n",priv->p4bus_device->name);
		/* stop the transmit queue */
		netif_stop_queue(priv->netdev);
		/* signal the host */
		p4bus_command_signal_ioring(priv->tx_ioring_id);
		return -EBUSY;
	}

	/* configure the operation */
	curr_op->addr = virt_to_phys(skb->data);
	curr_op->size = skb->len;
	curr_op->priv_guest = (unsigned long)skb;

	/* push the operation */
	p4bus_ioring_push_ready(curr_op);

	/*
	 * signal if the queue was empty of if last packet done
	 * packet said that the host will now sleep
	 *
	 * If operations where done in the meantime the host
	 * could also have gone to sleep
	 */
	if (signal || p4bus_ioring_has_done(&(priv->tx_ioring_op)))
		p4bus_command_signal_ioring(priv->tx_ioring_id);

    return 0;
}


/**
 *  @purpose
 *    received a packet from the host
 *
 *  @param currDev   	information structure on the current network device
 *  @param signal_host  indicates if we need to signal the host to process the op
 *
 *  @returns 0		 	if no error
 */
static void p4bus_vmnet_rx(p4bus_vmnet_deviceinfo_t *currDev, int from_irq)
{
	int32_t ret = 0;
    p4bus_operation_t *curr_op;
    struct sk_buff *skb;
    unsigned long flags;

    while (p4bus_ioring_has_free(&(currDev->rx_ioring_op))) {
    	curr_op = NULL;
    	skb = NULL;

		skb = dev_alloc_skb(currDev->p4bus_device->size);
		if (!skb)
		{
			dev_err(&(currDev->pdev->dev), PREF " %s Cannot allocate skb buffer %d\n",__func__, ret);
			return;
		}

		if (from_irq)
		{
			spin_lock(&currDev->rx_lock);
			curr_op = p4bus_ioring_get_free(&(currDev->rx_ioring_op));
			spin_unlock(&currDev->rx_lock);
			if (curr_op == NULL)
				dev_kfree_skb_irq(skb);
		}
		else
		{
			spin_lock_irqsave(&currDev->rx_lock, flags);
			curr_op = p4bus_ioring_get_free(&(currDev->rx_ioring_op));
			spin_unlock_irqrestore(&currDev->rx_lock, flags);
			if (curr_op == NULL)
				kfree_skb(skb);
		}

    	if (curr_op != NULL)
    	{
    		/* Set the buffer address */
			curr_op->addr = virt_to_phys(skb->data);
			curr_op->priv_guest = (uint64_t)(unsigned long)skb;

			/* configure the operation */
			curr_op->flags = OP_FLAGS_SIGNAL;
			curr_op->size = currDev->p4bus_device->size;

			/* push the operation */
			p4bus_ioring_push_ready(curr_op);
    	}
	}
}

/**
*  @purpose
*    network device initialisation function
*
*  @param dev   	net_device concerned by the operation
*
*  @returns
*/
static void netdev_init(struct net_device *dev)
{
	dev->netdev_ops = &p4bus_vmnet_netdev_ops;

	dev->header_ops         = &p4bus_vmnet_eth_header_ops;
	dev->type               = ARPHRD_ETHER;
	dev->hard_header_len    = ETH_HLEN;
	dev->mtu                = ETH_DATA_LEN;
	dev->addr_len           = ETH_ALEN;
	dev->tx_queue_len       = 1000;
	dev->flags              = IFF_BROADCAST | IFF_MULTICAST ;
	dev->priv_flags         |= IFF_TX_SKB_SHARING;
	memset(dev->broadcast, 0xFF, ETH_ALEN);
}


/**
 *  @purpose
 *    request and init memory resources for the device.
 * 		register the device on the P4 bus
 *
 *  @param pdev   platform device concerned
 *
 *  @returns 
 *  @retval -EINVAL		if an error occured during the device registration on the P4bus
 *  @retval -ENOMEM		if allocation of the managed device structure or
 *                 		ethernet device structure failed
 *  @retval -ENOMEM		if rx buffer could not be allocated
 *  @retval -EIO   		if device could not be registered as netdev
 *  @retval 0      		if no error
 */
static int p4bus_vmnet_probe(struct platform_device *pdev)
{
	p4bus_vmnet_deviceinfo_t *currDev = NULL;
	p4bus_device_info_t *p4bus_device = NULL;
	struct net_device *netdev_tmp;
	int ret;
	int i;

	printk(KERN_ERR PREF "probe %s.\n", pdev->name);

	/* check the P4BUS status */
	if(!p4bus_detected())
	{
		dev_err(&pdev->dev, PREF "no p4bus detected.\n");
		return -ENODEV;
	}

	/* device managed memory allocation */
	p4bus_device = devm_kzalloc(&pdev->dev, sizeof(p4bus_device_info_t), GFP_KERNEL);
	if (p4bus_device == NULL)
	{
		dev_err(&pdev->dev, PREF "could not allocate memory\n");
		return -ENOMEM;
	}

	/* register device on p4bus an fill the dev_info structure */
	ret = p4bus_get_devinfo(pdev, p4bus_device);
	if (ret)
	{
		dev_err(&pdev->dev, PREF  "could not get p4bus device info\n");
		devm_kfree(&pdev->dev, p4bus_device);
		return -EINVAL;
	}

	/* check if access flags required are allowed */
	if ( (((p4bus_device->opmask & (1<<P4BUS_OP_WRITE)) == 0) &&
			((p4bus_device->opmask & (1<<P4BUS_OP_NB_WRITE)) == 0)) )
	{
		dev_err(&pdev->dev, PREF "%s Error no WRITE permissions set in the host driver \n",__func__);
		devm_kfree(&pdev->dev, p4bus_device);
		return -EINVAL;
	}

	/* check if access flags required are allowed */
	if ( (((p4bus_device->opmask & (1<<P4BUS_OP_READ)) == 0) &&
			((p4bus_device->opmask & (1<<P4BUS_OP_NB_READ)) == 0)) )
	{
		dev_err(&pdev->dev, PREF "%s Error no READ permissions set in the host driver \n",__func__);
		devm_kfree(&pdev->dev, p4bus_device);
		return -EINVAL;
	}

	/* allocate private structure */
	netdev_tmp = alloc_etherdev(sizeof(p4bus_vmnet_deviceinfo_t));
	if (netdev_tmp == NULL)
	{
		dev_err(&pdev->dev, PREF "could not allocate net_device structure for %s\n",p4bus_device->name);
		devm_kfree(&pdev->dev, p4bus_device);
		return -ENOMEM;
	}

	SET_NETDEV_DEV(netdev_tmp, &pdev->dev);

	netdev_init(netdev_tmp);

    /* private data initialisation */
    currDev = netdev_priv(netdev_tmp);
    currDev->netdev = netdev_tmp;
    currDev->p4bus_device = p4bus_device;
    currDev->pdev = pdev;
    currDev->isRunning = FALSE;

    currDev->netdev->mtu = MTU_DEFAULT_SIZE;

    /* request the end of operation IRQ */
    if (devm_request_irq(&pdev->dev, p4bus_device->interrupt, p4bus_vmnet_interrupt,
                                IRQF_SHARED, dev_name(&pdev->dev), currDev) != 0)
	{
    	dev_err(&pdev->dev, PREF "cannot attach to the interrupt\n");
    	ret = -EIO;
    	goto err1;
	}

    /* create IORING */
    p4bus_ioring_init(&(currDev->rx_ioring_op));

    if (p4bus_command_create_ioring(&(currDev->rx_ioring_op), &(currDev->rx_ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create RX ioring\n");
        ret = -ENOMEM;
        goto err2;
    }
    else
    {
    	for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    	{
    		currDev->rx_ioring_op.operations[i].devid = p4bus_device->p4bus_devid;
    		currDev->rx_ioring_op.operations[i].type = P4BUS_OP_READ;
    		currDev->rx_ioring_op.operations[i].filedesc = 0;
    		currDev->rx_ioring_op.operations[i].param1 = 0;
    		currDev->rx_ioring_op.operations[i].param2 = 0;
    	}
    }

	p4bus_ioring_init(&(currDev->tx_ioring_op));

    if (p4bus_command_create_ioring(&(currDev->tx_ioring_op), &(currDev->tx_ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create TX ioring\n");
        ret = -ENOMEM;
        goto err3;
    }
    else
    {
       	for (i = 0; i < P4BUS_IORING_DEPTH; i++)
		{
			currDev->tx_ioring_op.operations[i].devid = p4bus_device->p4bus_devid;
			currDev->tx_ioring_op.operations[i].type = P4BUS_OP_WRITE;
			currDev->tx_ioring_op.operations[i].filedesc = 0;
			currDev->tx_ioring_op.operations[i].param1 = 0;
			currDev->tx_ioring_op.operations[i].param2 = 0;
		}
    }

    spin_lock_init(&(currDev->rx_lock));
    spin_lock_init(&(currDev->tx_lock));

	/* init wait queue */
	init_waitqueue_head(&(currDev->wait_op));

	/* register net device to kernel */
	ret = register_netdev(currDev->netdev);
	if (ret < 0) /* -EIO if error */
	{
		dev_err(&pdev->dev, PREF "could not register the net_device to the kernel\n");
		ret = -EIO;
        goto err4;
	}

	printk(KERN_ERR PREF "%s is registered as %s (irq %d)\n",p4bus_device->name, netdev_tmp->name, p4bus_device->interrupt);

	platform_set_drvdata(pdev, currDev);

	return 0;

err4:
    /* destroy the TX IORING */
    p4bus_command_destroy_ioring(currDev->tx_ioring_id);

err3:
    /* destroy the RX IORING */
    p4bus_command_destroy_ioring(currDev->rx_ioring_id);

err2:
    /* free IRQ */
    devm_free_irq(&pdev->dev, p4bus_device->interrupt, currDev);

err1:
	/* free resources */

	/* unregister network device from kernel */
	unregister_netdev(currDev->netdev);
    free_netdev(currDev->netdev);
    devm_kfree(&pdev->dev, p4bus_device);
    return ret;
}

/**
 *  @purpose
 * 		unregister the device on the P4 bus and release resources used by it
 *
 *  @param pdev_arg   platform device concerned
 *
 *  @returns 
 * 
 *  @retval 0		 	if no error
 */
static int p4bus_vmnet_remove(struct platform_device *pdev)
{
	p4bus_vmnet_deviceinfo_t *currDev = NULL;

	currDev = platform_get_drvdata(pdev);

	/* we inform the upper layers to stop TX */
	netif_stop_queue(currDev->netdev);

	/* unregister network device from kernel */
	unregister_netdev(currDev->netdev);

	/* destroy iorings */
	p4bus_command_destroy_ioring(currDev->tx_ioring_id);

	p4bus_command_destroy_ioring(currDev->rx_ioring_id);

	/* free the irq */
	devm_free_irq(&pdev->dev, currDev->p4bus_device->interrupt, currDev);

	/* free the data */
	devm_kfree(&pdev->dev, currDev->p4bus_device);

	/* free the netdev */
	free_netdev(currDev->netdev);

	printk(KERN_ERR PREF "%s removed.\n", pdev->name);

	return 0;
}


static int p4bus_vmnet_open(struct net_device *net_dev)
{
	int32_t ret = 0;
	p4bus_vmnet_deviceinfo_t *currDev = netdev_priv(net_dev);
    p4bus_device_info_t *dev_info = currDev->p4bus_device;

	if(!currDev)
	{
		printk( PREF " %s: Error with the net_device structure \n",__func__);
	}

	/* check if open operation is required */
	if ((dev_info->opmask & (1<<P4BUS_OP_OPEN)) != 0)
	{
		p4bus_operation_t curr_op;

		/* configure the operation */
		curr_op.type = P4BUS_OP_OPEN;
		curr_op.flags = OP_FLAGS_SIGNAL;
		curr_op.devid = currDev->p4bus_device->p4bus_devid;
		curr_op.filedesc = 0;
		curr_op.addr = 0;
		curr_op.size = 0;
		curr_op.param1 = P4BUS_FLAGS_OPEN_RW;
		curr_op.status = P4BUS_OPERATION_READY;

		/* execute the operation */
		ret = p4bus_command_execute_operation(&curr_op, NULL);
		if (ret != 0)
		{
			dev_err(&(currDev->pdev->dev), PREF "Error during OPEN operation on ethernet\n");
			return -EIO;
		}

		wait_event_interruptible(currDev->wait_op, (curr_op.status == P4BUS_OPERATION_DONE));

		if (curr_op.retcode != P4BUS_E_OK)
		{
			dev_err(&(currDev->pdev->dev), PREF "Cannot OPEN the network device\n");
			return -EACCES;
		}
	}

	ret = -EINVAL;

	if ((dev_info->opmask & (1<<P4BUS_OP_IOCTL)) != 0)
	{
		/* to get MAC using old style IOCTL */
		ret = p4bus_vmnet_get_mac_addr(currDev, IOCTL_CMD_GET_MAC_ADDR);
		if (ret != 0)
		{
			/* try to get MAC using new IOCTL */
			ret = p4bus_vmnet_get_mac_addr(currDev, NEW_IOCTL_CMD_GET_MAC_ADDR);
		}

		if (ret == 0)
		{
			printk( PREF "%s automatic MAC addr attribution\n",dev_info->name);
		}
	}

	if (ret != 0)
	{
		/* build a mac address */
		currDev->netdev->dev_addr[5] = (uint8_t) (((uint16_t)MAC_ADDR_PREFIX) & 0xFF);
		currDev->netdev->dev_addr[4] = (uint8_t) (((uint16_t)MAC_ADDR_PREFIX) >> 8) & 0xFF;
		currDev->netdev->dev_addr[3] = (uint8_t) (((uint32_t)dev_info->type_devid))         & 0xFF ;
		currDev->netdev->dev_addr[2] = (uint8_t) (((uint32_t)dev_info->type_devid) >> 8)    & 0xFF ;
		currDev->netdev->dev_addr[1] = (uint8_t) (((uint32_t)dev_info->type_devid) >> 16)   & 0xFF ;
		currDev->netdev->dev_addr[0] = (uint8_t) (((((uint32_t)dev_info->type_devid) >> 24) & 0xFC) | 0x2); /* 0x2 = unicast and locally administrated */
		printk( PREF "%s Cannot request MAC addr, it has been statically attributed  %x:%x:%x:%x:%x:%x \n",
				dev_info->name, currDev->netdev->dev_addr[0], currDev->netdev->dev_addr[1],
				currDev->netdev->dev_addr[2], currDev->netdev->dev_addr[3],
				currDev->netdev->dev_addr[4], currDev->netdev->dev_addr[5]);

	}
	currDev->isRunning = TRUE;

    /* initialize the rx ioring */
    p4bus_vmnet_rx(currDev, FALSE);

    /* wake up read queue */
    p4bus_command_signal_ioring(currDev->rx_ioring_id);

    /* ready to write */
    netif_start_queue(currDev->netdev);

    return 0;
}

/**
 *  @purpose
 *    stop TX, release resources and close the device
 *
 *  @param inode   	inode attached to this driver
 *  @param file 	file structure handled by this driver
 * 
 *  @returns
 * 
 *  @retval -EBUSY   	if device is busy
 *  @retval -EACCES  	if access is not allowed
 *  @retval -EIO  		if operation failed
 *  @retval 0		 	if no error
 */
static int p4bus_vmnet_close(struct net_device *net_dev)
{
	int32_t ret = 0;

	p4bus_vmnet_deviceinfo_t *currDev = netdev_priv(net_dev);
	if(!currDev)
	{	
		printk(PREF " %s: Error with the net_device structure \n",__func__);
	}
	/* stop TX process */
	netif_stop_queue(currDev->netdev);

	/* stop RX process */
	currDev->isRunning = FALSE;

	/* check if close operation is allowed */
	if ((currDev->p4bus_device->opmask & (1<<P4BUS_OP_CLOSE)) != 0)
	{
		p4bus_operation_t curr_op;

		/* configure the operation */
		curr_op.type = P4BUS_OP_CLOSE;
		curr_op.flags = OP_FLAGS_SIGNAL;
		curr_op.devid = currDev->p4bus_device->p4bus_devid;
		curr_op.filedesc = 0;
		curr_op.addr = 0;
		curr_op.size = 0;
		curr_op.param1 = P4BUS_FLAGS_OPEN_RW;
		curr_op.status = P4BUS_OPERATION_READY;

		/* execute the operation */
		ret = p4bus_command_execute_operation(&curr_op, NULL);
		if (ret != 0)
		{
			dev_err(&(currDev->pdev->dev), PREF "Error during CLOSE operation on ethernet\n");
			return -EIO;
		}

		wait_event_interruptible(currDev->wait_op, (curr_op.status == P4BUS_OPERATION_DONE));

		if (curr_op.retcode != P4BUS_E_OK)
		{
			dev_err(&(currDev->pdev->dev), PREF "CLOSE operation returned an error\n");
			return -EACCES;
		}
	}

	return ret;
}

static int p4bus_vmnet_get_mac_addr(p4bus_vmnet_deviceinfo_t *currDev, uint64_t ioctlnum)
{
	p4bus_operation_t curr_op;
	int ret;

	/* configure the operation */
    curr_op.type = P4BUS_OP_IOCTL;
    curr_op.flags = OP_FLAGS_SIGNAL;
    curr_op.devid = currDev->p4bus_device->p4bus_devid;
    curr_op.filedesc = 0;
    curr_op.param1 = ioctlnum;
    curr_op.param2 = P4BUS_SET_IOCTL_SIZE(0,6);
    curr_op.addr = virt_to_phys(currDev->netdev->dev_addr);
    curr_op.size = (sizeof(uint8_t)*6);
    curr_op.status = P4BUS_OPERATION_READY;

	/* execute the operation */
	ret = p4bus_command_execute_operation(&curr_op, NULL);
	if (ret != 0)
	{
		dev_err(&(currDev->pdev->dev), PREF "Error during IOCTL operation on ethernet\n");
		return -EIO;
	}

    /* wait the result */
    wait_event_interruptible(currDev->wait_op, (curr_op.status == P4BUS_OPERATION_DONE));

    if(curr_op.retcode != P4BUS_E_OK)
    {
    	return -EINVAL;
    }

    return 0;
}

/**
 *  @purpose
 *    check if the network device address is valid
 *
 *  @param dev   	network device concerned
 * 
 *  @returns
 * 
 *  @retval -EADDRNOTAVAIL  	if addr is not a valid ethernet address
 *  @retval 0		 			if no error
 */
static int p4bus_vmnet_validate_addr(struct net_device *dev)
{
	int32_t ret = 0;
	DBG(" return OK (not available with vmnet device)");
	return ret;
}

/**
 *  @purpose
 *    change network device mac address
 *
 *  @param dev   	network device concerned
 *  @param addr   	addr on a buffer containing the mac address to set
 * 
 *  @returns
 * 
 *  @retval -EADDRNOTAVAIL;  	if addr is not a valid ethernet address
 *  @retval -EBUSY;			  	if device is busy and operation failed
 *  @retval 0		 			if no error
 */
static int p4bus_vmnet_set_mac_address(struct net_device *dev, void *addr)
{
	int32_t ret = 0;
	DBG(" return OK (not available with vmnet device)");
	return ret; 
}

