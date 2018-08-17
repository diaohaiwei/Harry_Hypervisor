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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <asm/cacheflush.h>
#include <linux/pm_runtime.h>

#include "compat.h"
#include "p4bus.h"


/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"P4BUS: "
#define DRIVER_NAME		"p4bus"
#define DRIVER_COMPAT	"vmm,p4bus"

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int p4bus_probe(struct platform_device *pdev);

static int p4bus_remove(struct platform_device *pdev);

static int __init p4bus_init(void);

static void __exit p4bus_exit(void);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

static int p4bus_status = 0;

static const struct of_device_id p4bus_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, p4bus_match);


static struct platform_driver p4bus_driver = {
	.driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = p4bus_match,
    },
	.probe  = p4bus_probe,
	.remove = p4bus_remove,
};

#ifdef CONFIG_VMM_ENUMERATE
typedef struct probed_p4bus_device_s {
    struct list_head list;
    struct platform_device *pdev;
    char name[P4BUS_NAMELEN];
    struct resource resources[2];
}probed_p4bus_device_t;

LIST_HEAD(p4bus_devices_list);
#endif

/**
 * Our list of bus drivers
 */

module_init(p4bus_init);
module_exit(p4bus_exit);
MODULE_ALIAS("p4bus");
MODULE_AUTHOR("Bertrand Marquis (bma@sysgo.com)");
MODULE_DESCRIPTION("PikeOS virtualization bus");
MODULE_LICENSE("GPL");

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */
int p4bus_detected(void)
{
	return p4bus_status;
}
EXPORT_SYMBOL(p4bus_detected);

/**
 *  @purpose
 *    retrieve the p4bus protocol version and the 
 *    number of devices registered on it.
 */
int p4bus_command_version(int* version, int* num_devices)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;
    
    msg[0] = P4BUS_COMMAND_INFO_VERSION;

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    if(rc == P4BUS_E_OK)
    {
        /* P4BUS version */
        *version = msg[0];
        /* Number of devices */
        *num_devices = msg[1];
    }
    return rc;
}
EXPORT_SYMBOL(p4bus_command_version);

/**
 *  @purpose
 *    Instantiate an ioring and return the ioring ID
 */
int p4bus_command_create_ioring(p4bus_ioring_t* ioring, int* id)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;

    msg[0] = P4BUS_COMMAND_CREATE_IORING;
    msg[1] = virt_to_phys(ioring);

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    if(rc == P4BUS_E_OK)
    {
        /* ioring ID */
        *id = msg[0];
    }

    return rc;
}
EXPORT_SYMBOL(p4bus_command_create_ioring);

/**
 *  @purpose
 *    Destroy an ioring for a given ID
 */
int p4bus_command_destroy_ioring(int id)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;

    msg[0] = P4BUS_COMMAND_DESTROY_IORING;
    msg[1] = id;

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    return rc;
}
EXPORT_SYMBOL(p4bus_command_destroy_ioring);

/**
 *  @purpose
 *    Signal that an ioring needs to be processed
 */
int p4bus_command_signal_ioring(int id)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;

    msg[0] = P4BUS_COMMAND_SIGNAL_IORING;
    msg[1] = id;

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    return rc;
}
EXPORT_SYMBOL(p4bus_command_signal_ioring);

int p4bus_command_cancel_operation(int id)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;

    msg[0] = P4BUS_COMMAND_CANCEL_OPERATION;
    msg[1] = id;

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    return rc;
}
EXPORT_SYMBOL(p4bus_command_cancel_operation);

int p4bus_command_execute_operation(p4bus_operation_t *op, int *id)
{
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    int rc;

    msg[0] = P4BUS_COMMAND_EXECUTE_OPERATION;
    msg[1] = (unsigned long)virt_to_phys(op);

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    if(rc == P4BUS_E_OK && id != NULL)
	{
		/* ioring ID */
		*id = msg[0];
	}

    return rc;
}
EXPORT_SYMBOL(p4bus_command_execute_operation);

/**
 *  @purpose
 *    (optional) Short description (purpose) of the function.
 *    Will be overwritten by the purpose in the header file.
 */
int p4bus_get_devinfo(struct platform_device *pdev, p4bus_device_info_t* device_info)
{
	unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
	int rc;
	struct resource *regs;
	struct resource *irq;

	if (device_info == NULL ||
			pdev == NULL)
	{
		return -EINVAL;
	}

	/* Get device id */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (regs == NULL)
	{
		dev_err(&pdev->dev, PREF "No register resource\n");
		return -EINVAL;
	}

	/* get the rest of the information using the devinfo command */

    msg[0] = P4BUS_COMMAND_INFO_DEVICE;
    msg[1] = virt_to_phys(device_info);
    msg[2] = regs->start;

    rc = vmm_send_message(VMM_DEV_P4BUS, msg);

    if (rc != P4BUS_E_OK)
    {
    	return -ENOENT;
    }

    /* Get the Interrupt */
    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (irq != NULL)
    {
    	device_info->interrupt = irq->start;
    }
    else
    {
    	dev_err(&pdev->dev, PREF "No interrupt resource\n");
    	return -EINVAL;
    }

	return 0;
}
EXPORT_SYMBOL(p4bus_get_devinfo);

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/**
 *  @purpose
 *    module init routine, register driver to the kernel
 *
 *  @param 
 *
 *  @returns 0 if no error
 */

static int __init p4bus_init(void)
{
	int err;

	printk(KERN_ERR PREF "init.\n");

    /* register the p4bus driver to the kernel */
    err = platform_driver_register(&p4bus_driver);
    if (err)
    {
        printk(KERN_ERR PREF " %s Cannot register the p4bus driver as platform driver \n", __func__);
        return -ENODEV;
    }

    return 0;
}

/**
 *  @purpose
 *    module exit routine, unregister driver from the kernel
 *
 *  @param 
 *
 *  @returns 
 */
static void __exit p4bus_exit(void)
{
	platform_driver_unregister(&p4bus_driver);

	printk(KERN_ERR PREF "exit.\n");
}


/**
 *  @purpose
 *    request and init memory resources for the device.
 * 		register the device as vmm device
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
static int p4bus_probe(struct platform_device *pdev)
{
    int ret;
    int version, num_dev;
#ifdef CONFIG_VMM_ENUMERATE
    probed_p4bus_device_t *p4bus_devices_resources;
    unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
    p4bus_device_info_t device_info;
    struct irq_domain *domain;
    struct device_node *node;
    unsigned int virq;
    unsigned char chip_found = 0;
    struct irq_data *data;
    int i = 0;
    int j = 0;
#else
    struct device_node *np = pdev->dev.of_node;
#endif

    printk(KERN_ERR PREF "probe %s.\n", pdev->name);

    /* check that vmm is on and ok */
    if (!vmm_detected())
    {
    	dev_err(&pdev->dev, PREF "VMM has not been detected, P4BUS turned off.\n");
    	return -ENODEV;
    }

    /* check the P4BUS protocol version */
    if(p4bus_command_version(&version, &num_dev))
    {
    	dev_err(&pdev->dev, PREF "Cannot get the P4BUS API version, P4BUS turned off.\n");
        return -ENODEV;
    }

    /* Check the version */
    if(version != P4BUS_API_VERSION)
    {
    	dev_err(&pdev->dev, PREF "Wrong P4BUS API version: detected 0x%x, wanted 0x%x, P4BUS turned off.\n",
    			version, P4BUS_API_VERSION);
    	return -ENODEV;
    }

    p4bus_status = 1;

#ifdef CONFIG_VMM_ENUMERATE
    /* Search a interrupt controller in order to use his domain */
    node = of_find_node_with_property(NULL, "interrupt-controller");
    if (node == NULL)
    {
        dev_err(&pdev->dev, PREF "unable to find node for interrupt\n");
        return -EINVAL;
    }
    else 
    {
        if ((of_device_is_compatible(node, "arm,gic-400") 
        ||   of_device_is_compatible(node, "arm,gic-500") 
        ||   of_device_is_compatible(node, "arm,cortex-a15-gic")) == 0)
        {
            dev_err(&pdev->dev, PREF "unable to find node for interrupt\n");
            return -EINVAL;
        }
    }

    /* Find domain of the controller */
    domain = irq_find_host(node);
    if (domain == NULL)
    {
        dev_err(&pdev->dev, PREF "unable to find domain for interrupt\n");
        return -EINVAL;
    }

    for (i = 0; i < num_dev; i++)
    {
        /* allocate the resources */
        p4bus_devices_resources = kzalloc(sizeof(probed_p4bus_device_t), GFP_KERNEL);

        msg[0] = P4BUS_COMMAND_INFO_DEVICE;
        msg[1] = virt_to_phys(&device_info);
        msg[2] = i;

        ret = vmm_send_message(VMM_DEV_P4BUS, msg);

        if (ret == P4BUS_E_OK)
        {
            sprintf(p4bus_devices_resources->name, "%s", device_info.type);

            /* Create mapping between hw and vit irq */
            virq = irq_create_mapping(domain, (device_info.interrupt));
            if (virq == 0)
            {
                dev_err(&pdev->dev, PREF "unable to map interrupt\n");
                return -EINVAL;
            }

            chip_found = 0;
            /* Search default chip (associated with hwirq < 32) */
            for (j = 0; (j < 255) && (chip_found == 0); j++) 
            {
                data = irq_get_irq_data(j);
                if (data != NULL) 
                {
                    if ((data->chip != NULL) && (data->hwirq < 32) && data->chip_data != NULL)
                    {
                        chip_found = 1;
                    }
                }
            }

            /* the chip needs to be attached to the IRQ */
            if (chip_found == 0) 
            {
                dev_err(&pdev->dev, PREF "unable to find chip for interrupt\n");
            }

            ret = irq_set_chip(virq, data->chip);
            if (ret)
            {
                dev_err(&pdev->dev, PREF "unable to set chip for interrupt\n");
            }

            if(irq_set_chip_data(virq, data->chip_data) != 0)
            {
                dev_err(&pdev->dev, PREF "unable to set chip_data for interrupt\n");
            }

            irq_set_status_flags(virq, IRQ_LEVEL | IRQ_NOPROBE);
            irq_set_chip_and_handler(virq, data->chip, handle_fasteoi_irq);

            /* Create platform device with the rights informations (devid and compatible) */
            p4bus_devices_resources->resources[0].name = "reg";
            p4bus_devices_resources->resources[0].start = device_info.p4bus_devid;
            p4bus_devices_resources->resources[0].end = device_info.p4bus_devid;
            p4bus_devices_resources->resources[0].flags = IORESOURCE_MEM;

            p4bus_devices_resources->resources[1].name = "interrupts";
            p4bus_devices_resources->resources[1].start = virq;
            p4bus_devices_resources->resources[1].end = virq;
            p4bus_devices_resources->resources[1].flags = IORESOURCE_IRQ;

            /* instantiate a p4bus device manually */
			p4bus_devices_resources->pdev = platform_device_register_simple(
					p4bus_devices_resources->name, device_info.p4bus_devid,
					p4bus_devices_resources->resources, 2);

            /* add the devices to the list */
            list_add_tail(&p4bus_devices_resources->list, &p4bus_devices_list);
        }
    }
#else
    /* Validate our child nodes */
	if (np) {
        /* child nodes will be probed */
		ret = of_platform_populate(np, NULL, NULL, &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev,
					PREF "fail to add resources for bus child\n");
			return -EINVAL;
		}
	}
#endif

	pm_runtime_enable(&pdev->dev);

    return 0;
}

static int p4bus_remove(struct platform_device *pdev)
{
#ifdef CONFIG_VMM_ENUMERATE
    probed_p4bus_device_t *p4bus_devices_resources;

	/* remove all resources */
	if (!list_empty(&p4bus_devices_list))
	{
		list_for_each_entry(p4bus_devices_resources, &p4bus_devices_list, list)
		{
            platform_device_del(p4bus_devices_resources->pdev);
            list_del( &(p4bus_devices_resources->list) );
            kfree(p4bus_devices_resources);
		}
	}
#endif

	p4bus_status = 0;

    pm_runtime_disable(&pdev->dev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

    return 0;
}

