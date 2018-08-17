/*
 *  Early printk on PikeOS system console
 *
 *  Author: Alex Zuepke <azu@sysgo.de>
 *
 *  Copyright (C) 2014 SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "compat.h"
#include "vmm.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"EARLYCON: "
#define DRIVER_NAME		"earlycon"
#define DRIVER_COMPAT	"vmm,earlycon"

#define BUFF_SIZE		256

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int vmm_earlycon_probe(struct platform_device *pdev);

static int vmm_earlycon_remove(struct platform_device *pdev);

static void vmm_earlycon_write(struct console *c, const char *s, unsigned count);

#ifndef MODULE
static int __init vmm_earlycon_command_line(char *buf);
#endif

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

static const struct of_device_id vmm_earlycon_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, vmm_earlycon_match);


static struct platform_driver vmm_earlycon_driver = {
	.driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = vmm_earlycon_match,
    },
	.probe  = vmm_earlycon_probe,
	.remove = vmm_earlycon_remove,
};

/**
 * console structure describing our device
 */
static struct console vmm_earlycon = {
	name:		"vmm_earlycon",
	write:		vmm_earlycon_write,
	flags:		CON_BOOT,
	index:		-1,
};

/**
 * our init status
 */
static int vmm_earlycon_initialized = 0;

/**
 * Our print buffer
 */
static char *vmm_earlycon_buff = NULL;

#ifndef MODULE
/**
 * A static buffer for very early console, otherwise we allocate the buffer
 */
static char vmm_earlycon_static_buff[BUFF_SIZE] __attribute__ ((aligned (64)));
#endif

/**
 * lock to protect concurrent use of our buffer
 */
static DEFINE_SPINLOCK(earlycon_lock);

module_platform_driver(vmm_earlycon_driver);

MODULE_ALIAS("earlycon");
MODULE_AUTHOR("Bertrand Marquis (bma@sysgo.com)");
MODULE_DESCRIPTION("PikeOS virtualization earlycon");
MODULE_LICENSE("GPL");

#ifndef MODULE
early_param("earlyprintk", vmm_earlycon_command_line);
#endif

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/**
 *  @purpose
 *    write buffer on our early console device 
 *
 *  @param c 		console concerned
 *  @param s 		buffer to write
 *  @param count 	buffer size
 *
 */
static void vmm_earlycon_write(struct console *c, const char *s, unsigned count)
{
	unsigned long sent;
	unsigned long args[VMM_DATA_SIZE] = {0,0,0};
	unsigned long flags;

	if (!vmm_earlycon_initialized)
		return;

	spin_lock_irqsave(&earlycon_lock, flags);

	sent = 0;
	while (sent < count) {
		unsigned i;
		/* build single chunk to send, extend \n to \r\n
		 * i is index in tmp
		 */
		i = 0;
		while ((i < (BUFF_SIZE - 1)) && (sent < count)) {
			if (s[sent] == '\n')
			{
				vmm_earlycon_buff[i] = '\r';
				i++;
			}
			vmm_earlycon_buff[i] = s[sent];
			i++;
			sent++;
		}

		/* messages might get lost, but we don't care during early boot */
		if (i > 0) {
			vmm_earlycon_buff[i] = '\0';

			args[0] = i;
			vmm_send_message(VMM_DEV_EARLYCON, args);
		}
	}

	spin_unlock_irqrestore(&earlycon_lock, flags);
}

#ifndef MODULE
/**
 *  @purpose
 *    This function parses the early console parameter. Use earlyprintk=hvc
 *    to activate the earlyconsole over the PikeOS HWVIRT. When only one
 *    platform has been configured, this one is used for the early console. In
 *    this is case specifying "earlyprintk" is enough.
 *
 *  @param buf is used to specify the PikeOS platform
 *
 *  @retval 0 if no error
 *  @retval 1 if an error occured
 */
static int __init vmm_earlycon_command_line(char *buf)
{
	unsigned long args[VMM_DATA_SIZE] = {0,0,0};

	if (vmm_earlycon_initialized)
		return 1;

	vmm_earlycon_buff = vmm_earlycon_static_buff;

	args[0] = 0;
	args[1] = (unsigned long)virt_to_phys(vmm_earlycon_buff);

	if (vmm_send_message(VMM_DEV_EARLYCON, args) == 0)
	{
		register_console(&vmm_earlycon);

		vmm_earlycon_initialized = 1;

		printk(KERN_ERR PREF "registered\n");

		return 0;
	}

	printk(KERN_ERR PREF "cannot register\n");
	return 1;
}
#endif

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
static int vmm_earlycon_probe(struct platform_device *pdev)
{
	unsigned long args[VMM_DATA_SIZE] = {0,0,0};

	printk(KERN_ERR PREF "probe %s.\n", pdev->name);

    /* check that vmm is on and ok */
    if (!vmm_detected())
    {
    	dev_err(&pdev->dev, PREF "VMM has not been detected, " DRIVER_NAME "turned off.\n");
    	return -ENODEV;
    }

	if (!vmm_earlycon_initialized)
	{
		vmm_earlycon_buff = kzalloc(BUFF_SIZE * sizeof(char), GFP_KERNEL);
		if (ZERO_OR_NULL_PTR(vmm_earlycon_buff))
		{
			dev_err(&pdev->dev, PREF "Cannot allocate a buffer for earlycon.\n");
			return -ENOMEM;
		}

		args[0] = 0;
		args[1] = (unsigned long)virt_to_phys(vmm_earlycon_buff);

		if (vmm_send_message(VMM_DEV_EARLYCON, args) == 0)
		{
			register_console(&vmm_earlycon);

			vmm_earlycon_initialized = 1;

			printk(KERN_ERR PREF "registered\n");
		}
		else
		{
			printk(KERN_ERR PREF "cannot register\n");
		}
	}

	pm_runtime_enable(&pdev->dev);

    return 0;
}

static int vmm_earlycon_remove(struct platform_device *pdev)
{

    pm_runtime_disable(&pdev->dev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

    return 0;
}
