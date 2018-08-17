/*
 *  Virtual Monitor drivers
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
#include "vmm.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"VMM: "
#define DRIVER_NAME		"vmm"
#define DRIVER_COMPAT	"pikeos,vmm"

#ifndef CONFIG_OF
#error We need CONFIG_OF
#endif

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

#ifndef MODULE
static int __init vmm_early_dtb(void);

static int __init vmm_early_param(char *buf);
#endif

static int __init vmm_early_init(void);

static int vmm_probe(struct platform_device *pdev);

static int vmm_remove(struct platform_device *pdev);

static int __init vmm_init(void);

static void __exit vmm_exit(void);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

static int vmm_status = 0;
EXPORT_SYMBOL(vmm_status);

#ifndef MODULE
/**
 * our init must be done before SMP
 */
early_initcall(vmm_early_dtb);

early_param("vmm", vmm_early_param);
#endif

static const struct of_device_id vmm_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, vmm_match);

static const struct of_device_id vmm_driver_match[] = {
    {.compatible = "vmm,p4bus", },
	{.compatible = "vmm,bypass", },
	{.compatible = "vmm,vmapi", },
	{.compatible = "vmm,earlycon", },
    {},
};


static struct platform_driver vmm_driver = {
    .probe      = vmm_probe,
    .remove     = vmm_remove,
    .driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = vmm_match,
    },
};

module_init(vmm_init);
module_exit(vmm_exit);
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Bertrand Marquis (bma@sysgo.com)");
MODULE_DESCRIPTION("Virtual Monitor Bus");
MODULE_LICENSE("GPL");

struct platform_device vmm_dev = {
	.name = DRIVER_NAME,
	.id = 0,
	.num_resources = 0,
};

#ifdef CONFIG_VMM_ENUMERATE
struct platform_device p4bus_dev = {
	.name = "p4bus",
	.id = 0,
	.num_resources = 0,
};
#endif

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

int vmm_detected(void)
{
	return vmm_status;
}
EXPORT_SYMBOL(vmm_detected);

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/**
 *  @purpose
 *    register the platform device
 * 
 *  @param	pdev	platform_device porbed
 */
static int vmm_probe(struct platform_device *pdev)
{
#ifndef CONFIG_VMM_ENUMERATE
	struct device_node *np = pdev->dev.of_node;
#endif
    int ret;

    printk(KERN_ERR PREF "probe %s.\n", pdev->name);

#ifdef MODULE
	vmm_early_init();
#endif

    /* VMM protocol error */
    if(!vmm_detected())
    {
    	dev_err(&pdev->dev, PREF "VMM has not been detected, " DRIVER_NAME "turned off.\n");
        return -ENODEV;
    }


#ifndef CONFIG_VMM_ENUMERATE
    /* validate our child nodes */
	if (np) {
		ret = of_platform_populate(np, vmm_driver_match, NULL, &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev,
					PREF "fail to add resources for bus child\n");
			return -EINVAL;
		}
	}
#else
	/* Create platform device with the rights informations (devid and compatible) */

	/* instantiate a p4bus device manually */
	ret = platform_device_register(&p4bus_dev);
	if (ret) {
		dev_err(&pdev->dev,
				PREF "error registering p4bus platform device\n");
		return -EINVAL;
	}
#endif

	pm_runtime_enable(&pdev->dev);

	return 0;
}

/**
 *  @purpose
 *    remove the platform device
 * 		all its children will be removed
 * 
 *  @param	pdev	platform_device to remove
 * 
 *  @return 0
 */
static int vmm_remove(struct platform_device *pdev)
{

#ifdef CONFIG_VMM_ENUMERATE
    platform_device_unregister(&p4bus_dev);
#endif

	vmm_status = 0;

	pm_runtime_disable(&pdev->dev);

	printk(KERN_ERR PREF "%s removed.\n", pdev->name);

	return 0;
}

#ifndef MODULE
static int __init vmm_early_dtb(void)
{
	struct device_node *node = NULL;

	if (vmm_detected())
	{
		/* init already done, everything ok*/
		return 0;
	}

	/* get informations from the DTB */
	node = of_find_compatible_node(NULL, NULL, DRIVER_COMPAT);
	if (node != NULL)
	{
		return vmm_early_init();
	}

	pr_err(PREF " No PikeOS HWVIRT support detected\n");
	pr_err(PREF " Support turn off.\n");
	return 1;
}

static int __init vmm_early_param(char *buf)
{
	if (vmm_detected())
	{
		/* init already done, everything ok*/
		return 0;
	}

	if (strcmp(buf, "hvc") == 0)
	{
		return vmm_early_init();
	}
	pr_err(PREF "Invalid command line argument: %s\n", buf);

	return 1;
}
#endif

/**
 *  @purpose
 *    Do early initialization.
 *  @returns
 *    Nothing
 *
 *  @Design
 *    Function is early_init to be called before
 *    starting other core. It must not be called
 *    before DTB can be used.
 */
static int __init vmm_early_init(void)
{
	unsigned long message[VMM_MESSAGE_SIZE] = {0,0,0};

    /* get the VMM version */
	message[0] = VMM_INFO_VERSION;
	if (vmm_send_message(VMM_DEV_INFO, message) != 0)
	{
		pr_err(PREF "Cannot get version information\n");
		pr_err( PREF " Support turn off.\n");
		return -1;
	}

	if (message[0] != VMM_API_VERSION)
	{
		pr_err( PREF " Wrong VMM version: 0x%x waited 0x%x\n",
				(unsigned int)message[0], (unsigned int)VMM_API_VERSION);
		pr_err( PREF " Support turn off.\n");
		return 0;
	}

	vmm_status = 1;
	pr_err(PREF " PikeOS HWVIRT Detected, vmm api version 0x%x, PikeOS version 0x%x\n",
			(unsigned int)message[0], (unsigned int)message[1]);

	return 0;
}


static int __init vmm_init(void)
{
	int ret;

	printk(KERN_ERR PREF "init.\n");

    /* register the VMM driver */
	ret = platform_driver_register(&vmm_driver);
	if (ret != 0)
	{
		 printk(KERN_ERR PREF " %s Cannot register the vmm driver as platform driver \n", __func__);
		 return -ENODEV;
	}

#ifdef CONFIG_VMM_ENUMERATE
    /* if device will not be probed by the device tree */
	if (ret == 0 && vmm_status)
	{
        /* register the VMM device manually */
		ret = platform_device_register(&vmm_dev);
	}
#endif



	return ret;
}

static void __exit vmm_exit(void)
{
	platform_driver_unregister(&vmm_driver);

	printk(KERN_ERR PREF "exit.\n");
}
