/*
 *  Copyright (C) 2014 SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

/* ------------------------- FILE INCLUSION -------------------------------- */

#include <linux/module.h>
#include <linux/console.h>
#include <linux/proc_fs.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>

#include "compat.h"
#include "p4bus.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF				"P4BUS_vmtty: "
#define DRIVER_NAME			"vmtty"
#define DRIVER_COMPAT		"p4bus,vmtty"

#define PORT_P4BUS			30

#define DEV_NOT_INIT 	0x0
#define DEV_STARTUP 	0x1
#define DEV_SHUTDOWN 	0x2

#undef DEBUG
#ifdef DEBUG
#define DBG(args...)	do { \
							printk(KERN_ERR PREF "(%s:%d): ", __func__,__LINE__); \
							printk(args); \
						} while (0)
#else
#define DBG(args...)	do {} while(0)
#endif

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

typedef struct p4bus_vmtty_deviceinfo_s
{
	/* our tty port */
	struct tty_port port;
	
	volatile int active;

    /* platform device */
    struct platform_device *pdev;

	/* p4bus device */
	p4bus_device_info_t *p4bus_dev;

	/* maximum packet size */
	int32_t max_size;

	int id;

    /* RX IORING */
    spinlock_t rx_lock;
    uint32_t    rx_ioring_id;
    p4bus_ioring_t rx_ioring_op;

    /* TX IORING */
    spinlock_t tx_lock;
    uint32_t    tx_ioring_id;
    p4bus_ioring_t tx_ioring_op;

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE
    /* Kernel console IORING */
    spinlock_t console_lock;
	uint32_t    console_ioring_id;
	p4bus_ioring_t console_ioring_op;
	/* kernel console */
	struct console console;
#endif
} p4bus_vmtty_deviceinfo_t;

#ifdef CONFIG_SYSFS
static struct kobject* kobj_master;
#endif

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *p4bus_vmtty_proc_entry;
#endif

static p4bus_vmtty_deviceinfo_t *p4bus_vmtty_devices_list[P4BUS_MAX_DEVICES];

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE
static struct tty_driver *p4bus_vmtty_console_device(struct console *c, int *index);
static void p4bus_vmtty_console_write(struct console *c, const char *s, unsigned count);
static int p4bus_vmtty_console_setup(struct console *currCons, char *options);
#endif /* CONFIG_P4BUS_VMTTY_CONSOLE */

static irqreturn_t p4bus_vmtty_interrupt(int irq, void *priv);

static int p4bus_vmtty_activate(struct tty_port *port, struct tty_struct *tty);

static void p4bus_vmtty_shutdown(struct tty_port *port);

static int p4bus_vmtty_write(struct tty_struct * tty, const unsigned char *buf, int count);

static int p4bus_vmtty_write_room(struct tty_struct *tty);

static int p4bus_vmtty_open(struct tty_struct *tty, struct file *filp);

static void p4bus_vmtty_close(struct tty_struct *tty, struct file *filp);

static void __exit p4bus_vmtty_exit(void);

static int __init p4bus_vmtty_init(void);

static int p4bus_vmtty_probe(struct platform_device *pdev);

static int p4bus_vmtty_remove(struct platform_device *pdev);

#ifdef CONFIG_PROC_FS
static int p4bus_vmtty_proc_open(struct inode *inode, struct file *file);
static int p4bus_vmtty_proc_show(struct seq_file *m, void *v);
#endif
/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

#if defined(CONFIG_OF)
static const struct of_device_id p4bus_vmtty_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, p4bus_vmtty_match);

#else
#define p4bus_vmtty_match NULL
#endif

static struct platform_driver p4bus_vmtty_platform_driver = {
	.probe      = p4bus_vmtty_probe,
    .remove     = p4bus_vmtty_remove,
    .driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = p4bus_vmtty_match,
    },
};


#ifdef CONFIG_PROC_FS
static const struct file_operations p4bus_vmtty_proc_ops = {
	.open		= p4bus_vmtty_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static struct tty_port_operations p4bus_vmtty_port_ops = {
    .activate = p4bus_vmtty_activate,
    .shutdown = p4bus_vmtty_shutdown
};


static struct tty_operations p4bus_vmtty_operations = {
    .write = p4bus_vmtty_write,
    .write_room = p4bus_vmtty_write_room,
	.open = p4bus_vmtty_open,
	.close = p4bus_vmtty_close,
};

static struct tty_driver *p4bus_vmtty_driver;


/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

module_init(p4bus_vmtty_init);
module_exit(p4bus_vmtty_exit);

MODULE_ALIAS("platform:vmtty");
MODULE_AUTHOR("Benjamin Reynaud <bre@sysgo.fr>");
MODULE_DESCRIPTION("TTY/console driver over p4bus (hardware virtualisation)");
MODULE_LICENSE("GPL");

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */
#ifdef CONFIG_SYSFS
/**
 *  @purpose
 *    format guest_name for device attribute 
 *
 *  @param dev   	device concerned
 *  @param attr 	device attributes
 *  @param buf  	formated output for the attribute
 *
 *  @returns size written in buf
 */
static ssize_t guestname_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	p4bus_vmtty_deviceinfo_t* currDev = (p4bus_vmtty_deviceinfo_t*)dev_get_drvdata(dev);
	return sprintf(buf, "%s", currDev->p4bus_dev->name);
}

/**
 *  @purpose
 *    format guest_type for device attribute 
 *
 *  @param dev   	device concerned
 *  @param attr 	device attributes
 *  @param buf  	formated output for the attribute
 *
 *  @returns size written in buf
 */
static ssize_t guesttype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", DRIVER_NAME);
}


DEVICE_ATTR(guestname, S_IRUGO, guestname_show, NULL);
DEVICE_ATTR(guesttype, S_IRUGO, guesttype_show, NULL);
#endif

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE

static struct tty_driver *p4bus_vmtty_console_device(struct console *c, int *index)
{
	*index = c->index;
	return p4bus_vmtty_driver;
}

/**
 *  @purpose
 *    set uart options for console device
 *
 *  @param currCons	current console device
 *  @param options	options coming from the kernel command line
 *
 *  @returns 
 *  @retval 0		 	if no error
 *  @retval -EFAULT	 	if an error occured
 */
static int p4bus_vmtty_console_setup(struct console *currCons, char *options)
{
	return 0;
}

/**
 *  @purpose
 *    filled tx buffer with the chars coming from the console device
 *
 *  @param currCons 	current console device
 *  @param buff			buffer containing data to write
 *  @param count  		size of data
 *
 *  @returns
 */
static void p4bus_vmtty_console_write(struct console *currCons, const char *buff, unsigned count)
{
	p4bus_vmtty_deviceinfo_t *currDev = NULL;

	unsigned long flags;
	int locked = 1;
	int done = 0;
	p4bus_operation_t *curr_op;
	char *dest;

	currDev = p4bus_vmtty_devices_list[currCons->index];

	if (currDev == NULL)
	{
		dev_err(&(currDev->pdev->dev), PREF " %s Error vmtty device is not available\n",__func__);
		return;
	}

	if (oops_in_progress)
		locked = spin_trylock_irqsave(&currDev->console_lock, flags);
	else
		spin_lock_irqsave(&currDev->console_lock, flags);

	while (done < count)
	{
		/* we poll for operation to be done so not possible that
		 * there is no free operation
		 */
		curr_op = p4bus_ioring_get_free(&(currDev->console_ioring_op));
		if (curr_op == NULL)
		{
			dev_err(&(currDev->pdev->dev), PREF " %s Error no space in ioring\n",__func__);
			return;
		}

		dest = (char *)(unsigned long)curr_op->priv_guest;

		if ((count - done) < (currDev->max_size - 1))
		{
			memcpy(dest, &buff[done], (count - done));
			curr_op->size = (count - done) + 1;
			dest[count - done] = '\0';
			done = count;
		}
		else
		{
			memcpy(dest, &buff[done], currDev->max_size - 1);
			curr_op->size = currDev->max_size;
			dest[currDev->max_size - 1] = '\0';
			done += currDev->max_size - 1;
		}

		curr_op->flags = OP_FLAGS_POLL;

		/* push the operation */
		p4bus_ioring_push_ready(curr_op);

		p4bus_command_signal_ioring(currDev->console_ioring_id);

		/* poll for operation to be done */
		while (curr_op->status != P4BUS_OPERATION_DONE)
		{
			barrier();
		}

		/* free the operation */
		do {
			curr_op = p4bus_ioring_get_done(&(currDev->console_ioring_op));
			if (curr_op != NULL)
				p4bus_ioring_push_free(curr_op);
		} while (curr_op != NULL);
	}
	
	if (locked)
		spin_unlock_irqrestore(&currDev->console_lock, flags);
}
#endif

/**
 *  @purpose
 *    break controle handling (not supported)
 *
 *  @param port   		current uart port
 *  @param break_state 	break state
 *
 *  @returns
 */
static irqreturn_t p4bus_vmtty_interrupt(int irq, void *priv)
{
    p4bus_operation_t *curr_op;
    p4bus_vmtty_deviceinfo_t *currDev = priv;

    do{
        /* Check IORINGS */
    	spin_lock(&(currDev->rx_lock));
        curr_op = p4bus_ioring_get_done(&(currDev->rx_ioring_op));
        spin_unlock(&(currDev->rx_lock));
        if (curr_op != NULL) {
        	int signal = 0;
        	if (curr_op->type == P4BUS_OP_READ &&
        			curr_op->retcode == P4BUS_E_OK &&
					currDev->active) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
				tty_insert_flip_string(&(currDev->port), (const void *)(unsigned long)curr_op->priv_guest, curr_op->size);
				tty_flip_buffer_push(&(currDev->port));
#else
				tty_insert_flip_string(currDev->port.tty, (const void *)(unsigned long)curr_op->priv_guest, curr_op->size);
				tty_flip_buffer_push(currDev->port.tty);
#endif

        	} else {
        		if (curr_op->type != P4BUS_OP_READ)
        			dev_err(&(currDev->pdev->dev), "Wrong op type in rx ring\n");
        		else if (curr_op->retcode != P4BUS_E_OK)
        			dev_err(&(currDev->pdev->dev), "Wrong ret code in rx ring: 0x%x\n", curr_op->retcode);
        	}
        	signal = (curr_op->flags & OP_FLAGS_SLEEP);
        	p4bus_ioring_push_free(curr_op);

        	if (currDev->active)
        	{
				spin_lock(&(currDev->rx_lock));
				curr_op = p4bus_ioring_get_free(&(currDev->rx_ioring_op));
				spin_unlock(&(currDev->rx_lock));

				if (curr_op != NULL)
				{
					curr_op->size = currDev->max_size;
					curr_op->flags = OP_FLAGS_SIGNAL;

					p4bus_ioring_push_ready(curr_op);
				}

				if (signal)
					p4bus_command_signal_ioring(currDev->rx_ioring_id);
        	}
        } else {

        	spin_lock(&(currDev->tx_lock));
        	curr_op = p4bus_ioring_get_done(&(currDev->tx_ioring_op));
        	spin_unlock(&(currDev->tx_lock));

        	if (curr_op != NULL) {
        		if (curr_op->type == P4BUS_OP_WRITE &&
        				curr_op->retcode == P4BUS_E_OK) {
        			p4bus_ioring_push_free(curr_op);
        		} else {
        			p4bus_ioring_push_free(curr_op);
        			dev_err(&(currDev->pdev->dev), PREF "Wrong operation in trasmit ring\n");
        		}
        		if (currDev->active)
        			tty_wakeup(currDev->port.tty);
        	}
        }
    }while(curr_op != NULL);

    return IRQ_HANDLED;
}

/**
 *  @purpose
 *    This routine is called when we can start sending/receiving.
 *
 */
static int p4bus_vmtty_activate(struct tty_port *port, struct tty_struct *tty)
{
	p4bus_vmtty_deviceinfo_t *currDev = container_of(port, p4bus_vmtty_deviceinfo_t,
	                                    port);
	p4bus_operation_t *curr_op;
	unsigned long flags;

	currDev->active = 1;
	if ((currDev->p4bus_dev->opmask & (1<<P4BUS_OP_READ)) != 0)
	{
		spin_lock_irqsave(&(currDev->rx_lock), flags);
		do {
			curr_op = p4bus_ioring_get_free(&(currDev->rx_ioring_op));
			if (curr_op != NULL)
			{
				curr_op->size = currDev->max_size;
				curr_op->flags = OP_FLAGS_SIGNAL;
				p4bus_ioring_push_ready(curr_op);
			}
		} while (curr_op != NULL);
		spin_unlock_irqrestore(&(currDev->rx_lock), flags);

		/* signal reception */
		p4bus_command_signal_ioring(currDev->rx_ioring_id);
	}
	return 0;
}

/**
 *  @purpose
 *    This routine is called when we should stop send/receive.
 *
 */
static void p4bus_vmtty_shutdown(struct tty_port *port)
{
	p4bus_vmtty_deviceinfo_t *currDev = container_of(port, p4bus_vmtty_deviceinfo_t,
		                                    port);
	currDev->active = 0;
}

/**
 *  @purpose
 *    This routine is called when a particular tty device is opened.
 *
 *  Mandatory
 */
static int p4bus_vmtty_open(struct tty_struct *tty, struct file *filp)
{
	p4bus_vmtty_deviceinfo_t *currDev = p4bus_vmtty_devices_list[tty->index];
    return tty_port_open(&currDev->port, tty, filp);
}

/**
 *  @purpose
 *    This routine is called when a particular tty device is closed.
 *
 *  Mandatory
 */
static void p4bus_vmtty_close(struct tty_struct *tty, struct file *filp)
{
    tty_port_close(tty->port, tty, filp);
}

/**
 *  @purpose
 *    write characters on the console
 *
 *  @returns: number of characters written
 */
static int p4bus_vmtty_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int done = 0;
	p4bus_operation_t *curr_op;
	char *dest;
	p4bus_vmtty_deviceinfo_t *currDev = p4bus_vmtty_devices_list[tty->index];

	curr_op = p4bus_ioring_get_free(&(currDev->tx_ioring_op));
	if (curr_op == NULL)
	{
		dev_err(&(currDev->pdev->dev), PREF " %s Error no space in ioring\n",__func__);
		return 0;
	}

	dest = (char *)(unsigned long)curr_op->priv_guest;

	if (count + 1 >= currDev->max_size)
		done = currDev->max_size - 1;
	else
		done = count;

	memcpy(dest, buf, done);
	curr_op->size = done + 1;
	dest[done] = '\0';
	curr_op->flags = OP_FLAGS_SIGNAL;

	/* push the operation */
	p4bus_ioring_push_ready(curr_op);

	p4bus_command_signal_ioring(currDev->tx_ioring_id);

	return done;
}

/**
 *  @purpose
 *    Called to know how many characters might be written on
 *    the tty
 *
 *  @returns: number of characters written
 */
static int p4bus_vmtty_write_room(struct tty_struct *tty)
{
	p4bus_vmtty_deviceinfo_t *currDev = p4bus_vmtty_devices_list[tty->index];

	if (p4bus_ioring_has_free(&(currDev->tx_ioring_op)))
		return currDev->max_size;

	return 0;
}

/**
 *  @purpose
 *    init module routine. 
 * 		register an uart driver and a platform driver 
 *
 *  @param 
 *
 *  @returns
 * 
 *  @retval	0	 		if no error
 *  @retval -EFAULT	 	if an error occured during uart driver registration
 *  @retval -ENOMEM 	if an error occured during platform driver registration
 */
static int __init p4bus_vmtty_init(void)
{
	int32_t ret = 0;

	printk(KERN_ERR PREF "init.\n");
		
	for (ret = 0; ret < P4BUS_MAX_DEVICES; ret++)
	{
		p4bus_vmtty_devices_list[ret] = NULL;
	}

	p4bus_vmtty_driver = alloc_tty_driver(P4BUS_MAX_DEVICES);
	if (!p4bus_vmtty_driver)
	    return -ENOMEM;

    p4bus_vmtty_driver->driver_name = DRIVER_NAME;
    p4bus_vmtty_driver->name = DRIVER_NAME;
    p4bus_vmtty_driver->major = 0;
    p4bus_vmtty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    p4bus_vmtty_driver->subtype = SERIAL_TYPE_NORMAL;
    p4bus_vmtty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
    p4bus_vmtty_driver->init_termios = tty_std_termios;
    p4bus_vmtty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;

    tty_set_operations(p4bus_vmtty_driver, &p4bus_vmtty_operations);

    ret = tty_register_driver(p4bus_vmtty_driver);
    if (ret) {
    	put_tty_driver(p4bus_vmtty_driver);
    	return ret;
    }
	
#ifdef CONFIG_PROC_FS
	/* register proc entry */
	p4bus_vmtty_proc_entry = proc_create(DRIVER_NAME, 0600, 0, &p4bus_vmtty_proc_ops);
#endif

#ifdef CONFIG_SYSFS
	kobj_master = kobject_create_and_add(DRIVER_NAME, NULL);
#endif

	/* register platform_driver */
	ret = platform_driver_register(&p4bus_vmtty_platform_driver);
	if (ret)
	{
		printk(KERN_ERR PREF " FATAL ERROR in %s\n", __func__);
		tty_unregister_driver(p4bus_vmtty_driver);
		put_tty_driver(p4bus_vmtty_driver);
		return -EIO;
	}

	return ret;
}

/**
 *  @purpose
 *    exit module rountine
 *
 *  @param
 *
 *  @returns
 */
static void __exit p4bus_vmtty_exit(void)
{
	/* register platform_driver */
	platform_driver_unregister(&p4bus_vmtty_platform_driver);

	tty_unregister_driver(p4bus_vmtty_driver);

	put_tty_driver(p4bus_vmtty_driver);

#ifdef CONFIG_SYSFS
	kobject_put(kobj_master);
#endif

#ifdef CONFIG_PROC_FS
	remove_proc_entry(DRIVER_NAME, 0);
#endif

	printk(KERN_ERR PREF "exit.\n");
}

/**
 *  @purpose
 *    remove the p4bus device from the p4bus.
 * 		remove the uart port 
 *
 *  @param pdev   		platform device
 *
 *  @returns
 * 	@retval -EFAULT		if a memory error occured
 *  @retval 0			if no error
 */
static int p4bus_vmtty_remove(struct platform_device *pdev)
{
	int32_t ret = 0;
	int i;
	p4bus_vmtty_deviceinfo_t *currDev = NULL;
    p4bus_operation_t curr_op;

    currDev = dev_get_drvdata(&(pdev->dev));
    if(!currDev)
    {
        dev_err(&(pdev->dev), PREF " %s cannot remove the vmtty device\n",__func__);
        return -EFAULT;
    }


	/* check if open operation is allowed */
	if ((currDev->p4bus_dev->opmask & (1<<P4BUS_OP_CLOSE)) != 0)
	{
        /* configure the operation */
        curr_op.type = P4BUS_OP_CLOSE;
        curr_op.flags = OP_FLAGS_POLL;
        curr_op.devid = currDev->p4bus_dev->p4bus_devid;
        curr_op.filedesc = 0;
        curr_op.param1 = 0;
        curr_op.addr = 0;
        curr_op.size = 0;
        curr_op.status = P4BUS_OPERATION_READY;

        /* execute the operation */
        ret = p4bus_command_execute_operation(&curr_op, NULL);
        if (ret != 0)
        {
        	/* error closing tty */
        	dev_err(&(pdev->dev), PREF "error closing vmtty device\n");
        	return -EIO;
        }

		/* poll operation to be done */
		do {

			barrier();
		} while (curr_op.status != P4BUS_OPERATION_DONE);
	}

	tty_unregister_device(p4bus_vmtty_driver, currDev->id);
	p4bus_vmtty_devices_list[currDev->id] = NULL;

	/* free irq */
	devm_free_irq(&pdev->dev, currDev->p4bus_dev->interrupt, currDev);

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE

	unregister_console(&currDev->console);

	/* free console ioring */
	p4bus_command_destroy_ioring(currDev->console_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->console_ioring_op.operations[i].priv_guest);
    }
#endif

	/* free tx ioring */
    p4bus_command_destroy_ioring(currDev->tx_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->tx_ioring_op.operations[i].priv_guest);
    }

    /* free rx ioring */
    p4bus_command_destroy_ioring(currDev->rx_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->rx_ioring_op.operations[i].priv_guest);
    }

#ifdef CONFIG_SYSFS
	device_remove_file(&pdev->dev, &dev_attr_guestname);
	device_remove_file(&pdev->dev, &dev_attr_guesttype);

    sysfs_remove_link(kobj_master, currDev->p4bus_dev->name);
#endif

	devm_kfree(&pdev->dev, currDev);
	
	printk(KERN_ERR PREF "%s removed.\n", pdev->name);

	return ret;
}

/**
 *  @purpose
 *    register a platform device as a p4bus device on the p4bus.
 * 		register an uart port for the p4bus device
 *
 *  @param pdev   		platform device
 *
 *  @returns 	
 * 
 *	@retval -EFAULT		if a memory error occured
 *	@retval -EBUSY		if the device is busy when we try to open it
 *	@retval -EINVAL		if the device configuration is bad (buffer size)
 *  @retval 0			if no error
 */
static int p4bus_vmtty_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	p4bus_vmtty_deviceinfo_t* currDev;
    p4bus_operation_t curr_op;
    p4bus_device_info_t *dev_info;
    int i = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
    struct device *dev;
#endif

    printk(KERN_ERR PREF "probe %s.\n", pdev->name);

    /* check the P4BUS status */
    if(!p4bus_detected())
    {
    	dev_err(&pdev->dev, PREF "no p4bus detected.\n");
        return -ENODEV;
    }

	/* device managed memory allocation */
	currDev = devm_kzalloc(&pdev->dev, sizeof(p4bus_vmtty_deviceinfo_t), GFP_KERNEL);
	if (currDev == NULL)
	{
		dev_err(&pdev->dev, PREF "Cannot allocate memory for the device\n");
		return -ENOMEM;
	}

	currDev->p4bus_dev = devm_kzalloc(&pdev->dev, sizeof(p4bus_device_info_t), GFP_KERNEL);
	if (currDev == NULL)
	{
		dev_err(&pdev->dev, PREF "Cannot allocate memory for the device info\n");
		return -ENOMEM;
	}


	/* vmtty interface initialisation */
	currDev->pdev = pdev;

	platform_device_add_data(pdev, currDev, sizeof(p4bus_vmtty_deviceinfo_t));

	/* register device on p4bus */
    ret = p4bus_get_devinfo(pdev, currDev->p4bus_dev);
	if (ret < 0)
	{
		dev_err(&pdev->dev, PREF "could not register p4bus device\n");
		devm_kfree(&pdev->dev, currDev);
		ret = -EFAULT;
		goto err0;
	}

    /* dev_info is now filled */
    dev_info = currDev->p4bus_dev;

	if ((dev_info->opmask & (1<<P4BUS_OP_WRITE)) == 0)
	{
		/* Device cannot receive chars */
		dev_err(&pdev->dev, PREF "Cannot handle a device with no write: 0x%x\n",
				dev_info->opmask);
		ret = -EINVAL;
		goto err0;
	}

	if (dev_info->size<=0)
	{
		dev_err(&pdev->dev, PREF "Device with 0 size cannot be used\n");
		ret = -EINVAL;
		goto err0;
	}

	currDev->max_size = dev_info->size;
	currDev->id = dev_info->type_devid;

    /* IORING initialization */
    p4bus_ioring_init(&(currDev->rx_ioring_op));
    if (p4bus_command_create_ioring(&(currDev->rx_ioring_op), &(currDev->rx_ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create RX ioring\n");
        ret = -ENOMEM;
        goto err0;
    }

    spin_lock_init(&currDev->rx_lock);
	/* initialize the private field in the operations ioring */
	for (i = 0; i < P4BUS_IORING_DEPTH; i++)
	{
		char *buff = devm_kzalloc(&(currDev->pdev->dev), currDev->max_size, GFP_KERNEL);
		if (buff == NULL)
		{
			dev_err(&pdev->dev, PREF "Cannot allocate rx buffers\n");
			p4bus_command_destroy_ioring(currDev->rx_ioring_id);
			ret = -ENOMEM;
			goto err0;
		}

		/* preset constant ioring fields */
		currDev->rx_ioring_op.operations[i].type = P4BUS_OP_READ;
		currDev->rx_ioring_op.operations[i].devid = dev_info->p4bus_devid;
		currDev->rx_ioring_op.operations[i].filedesc = 0;
		currDev->rx_ioring_op.operations[i].addr = virt_to_phys(buff);
		currDev->rx_ioring_op.operations[i].param1 = 0;
		currDev->rx_ioring_op.operations[i].param2 = 0;
		currDev->rx_ioring_op.operations[i].flags = OP_FLAGS_SIGNAL;
		currDev->rx_ioring_op.operations[i].size = currDev->max_size;
		currDev->rx_ioring_op.operations[i].priv_guest = (unsigned long)buff;
	}

    p4bus_ioring_init(&(currDev->tx_ioring_op));
    if (p4bus_command_create_ioring(&(currDev->tx_ioring_op), &(currDev->tx_ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create TX ioring\n");
        ret = -ENOMEM;
        goto err1;
    }

    spin_lock_init(&currDev->tx_lock);

    /* initialize the private field in the operations ioring */
	for (i = 0; i < P4BUS_IORING_DEPTH; i++)
	{
		char *buff = devm_kzalloc(&(currDev->pdev->dev), currDev->max_size, GFP_KERNEL);
		if (buff == NULL)
		{
			dev_err(&pdev->dev, PREF "Cannot allocate rx buffers\n");
			p4bus_command_destroy_ioring(currDev->tx_ioring_id);
			ret = -ENOMEM;
			goto err1;
		}

		/* preset constant ioring fields */
		currDev->tx_ioring_op.operations[i].type = P4BUS_OP_WRITE;
		currDev->tx_ioring_op.operations[i].devid = dev_info->p4bus_devid;
		currDev->tx_ioring_op.operations[i].filedesc = 0;
		currDev->tx_ioring_op.operations[i].addr = virt_to_phys(buff);
		currDev->tx_ioring_op.operations[i].param1 = 0;
		currDev->tx_ioring_op.operations[i].param2 = 0;
		currDev->tx_ioring_op.operations[i].priv_guest = (unsigned long)buff;
	}

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE
    p4bus_ioring_init(&(currDev->console_ioring_op));
    if (p4bus_command_create_ioring(&(currDev->console_ioring_op), &(currDev->console_ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create console ioring\n");
        ret = -ENOMEM;
        goto err2;
    }

    spin_lock_init(&currDev->console_lock);
	/* initialize the private field in the operations ioring */
	for (i = 0; i < P4BUS_IORING_DEPTH; i++)
	{
		char *buff = devm_kzalloc(&(currDev->pdev->dev), currDev->max_size, GFP_KERNEL);
		if (buff == NULL)
		{
			dev_err(&pdev->dev, PREF "Cannot allocate console buffers\n");
			p4bus_command_destroy_ioring(currDev->console_ioring_id);
			ret = -ENOMEM;
			goto err2;
		}

		/* preset constant ioring fields */
		currDev->console_ioring_op.operations[i].type = P4BUS_OP_WRITE;
		currDev->console_ioring_op.operations[i].devid = dev_info->p4bus_devid;
		currDev->console_ioring_op.operations[i].filedesc = 0;
		currDev->console_ioring_op.operations[i].addr = virt_to_phys(buff);
		currDev->console_ioring_op.operations[i].param1 = 0;
		currDev->console_ioring_op.operations[i].param2 = 0;
		currDev->console_ioring_op.operations[i].priv_guest = (unsigned long)buff;
	}
#endif


	/* if device needs an open do it now */
	if ((dev_info->opmask & (1<<P4BUS_OP_OPEN)) != 0)
	{
        /* configure the operation */
        curr_op.type = P4BUS_OP_OPEN;
        curr_op.flags = OP_FLAGS_POLL;
        curr_op.devid = currDev->p4bus_dev->p4bus_devid;
        curr_op.filedesc = 0;
        curr_op.addr = 0;
        curr_op.size = 0;
        curr_op.param1 = P4BUS_FLAGS_OPEN_RW;
        curr_op.status = P4BUS_OPERATION_READY;

        /* push the operation */
        ret = p4bus_command_execute_operation(&curr_op, NULL);

        if (ret != 0)
        {
        	dev_err(&(currDev->pdev->dev), PREF "%s error executing OPEN operation\n",__func__);
        	ret = -EIO;
        	goto err3;
        }

		/* poll for open end */
		do {

			barrier();
		} while (curr_op.status != P4BUS_OPERATION_DONE);

		if (curr_op.retcode != P4BUS_E_OK)
		{
			/* error during open */
			dev_err(&pdev->dev, PREF "Device OPEN operation return an error %d\n",ret);
			ret = -EIO;
            goto err3;
		}
	}

	 /* request the end of operation IRQ */
	if (devm_request_irq(&pdev->dev, dev_info->interrupt, p4bus_vmtty_interrupt,
								IRQF_SHARED, dev_name(&pdev->dev), currDev) != 0)
	{
		dev_err(&pdev->dev, PREF "cannot attach to the interrupt\n");
		ret = -EIO;
		goto err3;
	}

	/* add device to vmtty list */
	p4bus_vmtty_devices_list[currDev->id] = currDev;

	tty_port_init(&currDev->port);
	currDev->port.ops = &p4bus_vmtty_port_ops;
	currDev->active = 0;
	/* add tty device */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	tty_register_device(p4bus_vmtty_driver, currDev->id, &pdev->dev);
#else
	dev = tty_port_register_device(&currDev->port,
			p4bus_vmtty_driver, currDev->id, &pdev->dev);
	if (IS_ERR(dev))
	{
		dev_err(&pdev->dev, PREF "Error registering tty device\n");
		goto err4;
	}
#endif

#ifdef CONFIG_P4BUS_VMTTY_CONSOLE
	strcpy(currDev->console.name, DRIVER_NAME);
	currDev->console.write = p4bus_vmtty_console_write;
	currDev->console.setup = p4bus_vmtty_console_setup;
	currDev->console.device = p4bus_vmtty_console_device;
	currDev->console.flags = CON_PRINTBUFFER;
	currDev->console.index = currDev->id;
	currDev->console.data = currDev;

	register_console(&currDev->console);
#endif

	dev_set_drvdata(&pdev->dev, currDev);

#ifdef CONFIG_SYSFS
	device_create_file(&pdev->dev, &dev_attr_guestname);
	device_create_file(&pdev->dev, &dev_attr_guesttype);

	if(sysfs_create_link(kobj_master, &pdev->dev.kobj , dev_info->name) != 0)
	{
		printk("ERROR during sysfs_create_link\n");
	}
#endif
	DBG("%p 0x%x 0x%x 0x%x %s\n", currDev->uart.cons, 
			currDev->uart.cons->flags, CON_ENABLED, 
			currDev->uart.type, currDev->uart.cons->name);

	return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
err4:
	devm_free_irq(&pdev->dev, dev_info->interrupt, currDev);
#endif

err3:
#ifdef CONFIG_P4BUS_VMTTY_CONSOLE
	p4bus_command_destroy_ioring(currDev->console_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->console_ioring_op.operations[i].priv_guest);
    }
err2:
#endif

    p4bus_command_destroy_ioring(currDev->tx_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->tx_ioring_op.operations[i].priv_guest);
    }

err1:
    p4bus_command_destroy_ioring(currDev->rx_ioring_id);
    for (i = 0; i < P4BUS_IORING_DEPTH; i++)
    {
    	devm_kfree(&pdev->dev, (void *)(unsigned long)currDev->rx_ioring_op.operations[i].priv_guest);
    }

err0:
	devm_kfree(&pdev->dev, currDev);

    return ret;
}

#ifdef CONFIG_PROC_FS
/**
 *  @purpose
 *    prints proc informations regarding this driver
 *
 *  @param m	seq file in which all informations will be write
 *
 *  @returns
 */
static int p4bus_vmtty_proc_show(struct seq_file *m, void *v)
{
	p4bus_vmtty_deviceinfo_t *currDev = NULL;
	int32_t i=0;
			
	seq_printf(m,"# <minor> <guest_name>\n");
			
	/* remove entry resource from list */
	for (i = 0; i < P4BUS_MAX_DEVICES; i++)
	{
		if (p4bus_vmtty_devices_list[i] != NULL)
		{
			currDev = p4bus_vmtty_devices_list[i];
			seq_printf(m,"%d %d %s\n",i , currDev->p4bus_dev->p4bus_devid, currDev->p4bus_dev->name);
		}
	}

	return 0;
}

/**
 *  @purpose
 *    open a proc file 
 *
 *  @param inode	node (not used)
 *  @param file 	file to open
 *
 *  @returns 0 if no error
 */
static int p4bus_vmtty_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, p4bus_vmtty_proc_show, NULL);
}
#endif
