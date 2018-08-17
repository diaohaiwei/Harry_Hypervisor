/*
 *  PikeOS VMM Bypass
 *
 *  Author: Bertrand Marquis <bma@sysgo.com>
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
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#include "vmm.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"BYPASS: "
#define DRIVER_NAME		"bypass"
#define DRIVER_COMPAT	"vmm,bypass"

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int vmm_bypass_probe(struct platform_device *pdev);

static int vmm_bypass_remove(struct platform_device *pdev);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

static int bypass_status = 0;

static const struct of_device_id vmm_bypass_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, vmm_bypass_match);


static struct platform_driver vmm_bypass_driver = {
	.driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = vmm_bypass_match,
    },
	.probe  = vmm_bypass_probe,
	.remove = vmm_bypass_remove,
};

/**
 * Our list of bus drivers
 */
module_platform_driver(vmm_bypass_driver);

MODULE_ALIAS("bypass");
MODULE_AUTHOR("Bertrand Marquis (bma@sysgo.com)");
MODULE_DESCRIPTION("PikeOS virtualization bypass");
MODULE_LICENSE("GPL");

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

int vmm_bypass_detected(void)
{
	return bypass_status;
}
EXPORT_SYMBOL(vmm_bypass_detected);

uint8_t  vmm_bypass_read8(unsigned long addr)
{
	unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
	int ret;

	if (!bypass_status)
		return 0;

	msg[0] = VMM_BYPASS_READ | VMM_BYPASS_8BIT;
	msg[1] = addr;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret == 0)
	{
		return (uint8_t)msg[2];
	}
	else
	{
		printk(KERN_ERR "vmm-bypass: Read error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
		return 0;
	}
}
EXPORT_SYMBOL(vmm_bypass_read8);

uint16_t  vmm_bypass_read16(unsigned long addr)
{
	unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
	int ret;

	if (!bypass_status)
		return 0;

	msg[0] = VMM_BYPASS_READ | VMM_BYPASS_16BIT;
	msg[1] = addr;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret == 0)
	{
		return (uint16_t)msg[2];
	}
	else
	{
		printk(KERN_ERR "vmm-bypass: Read error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
		return 0;
	}
}
EXPORT_SYMBOL(vmm_bypass_read16);

uint32_t  vmm_bypass_read32(unsigned long addr)
{
	unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
	int ret;

	if (!bypass_status)
		return 0;

	msg[0] = VMM_BYPASS_READ | VMM_BYPASS_32BIT;
	msg[1] = addr;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret == 0)
	{
		return (uint32_t)msg[2];
	}
	else
	{
		printk(KERN_ERR "vmm-bypass: Read error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
		return 0;
	}
}
EXPORT_SYMBOL(vmm_bypass_read32);

uint64_t  vmm_bypass_read64(unsigned long addr)
{
	unsigned long msg[VMM_DATA_SIZE] = {0,0,0};
	int ret;

	if (!bypass_status)
		return 0;

	msg[0] = VMM_BYPASS_READ | VMM_BYPASS_64BIT;
	msg[1] = addr;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret == 0)
	{
		return (uint64_t)msg[2];
	}
	else
	{
		printk(KERN_ERR "vmm-bypass: Read error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
		return 0;
	}
}
EXPORT_SYMBOL(vmm_bypass_read64);

void vmm_bypass_write8(unsigned long addr, uint8_t value)
{
	unsigned long msg[VMM_DATA_SIZE];
	int ret;

	if (!bypass_status)
		return;

	msg[0] = VMM_BYPASS_WRITE|VMM_BYPASS_8BIT;
	msg[1] = addr;
	msg[2] = value;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret != 0)
	{
		printk(KERN_ERR "vmm-bypass: Write error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
	}
}
EXPORT_SYMBOL(vmm_bypass_write8);

void vmm_bypass_write16(unsigned long addr, uint16_t value)
{
	unsigned long msg[VMM_DATA_SIZE];
	int ret;

	if (!bypass_status)
		return;

	msg[0] = VMM_BYPASS_WRITE|VMM_BYPASS_16BIT;
	msg[1] = addr;
	msg[2] = value;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret != 0)
	{
		printk(KERN_ERR "vmm-bypass: Write error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
	}
}
EXPORT_SYMBOL(vmm_bypass_write16);

void vmm_bypass_write32(unsigned long addr, uint32_t value)
{
	unsigned long msg[VMM_DATA_SIZE];
	int ret;

	if (!bypass_status)
		return;

	msg[0] = VMM_BYPASS_WRITE|VMM_BYPASS_32BIT;
	msg[1] = addr;
	msg[2] = value;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret != 0)
	{
		printk(KERN_ERR "vmm-bypass: Write error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
	}
}
EXPORT_SYMBOL(vmm_bypass_write32);

void vmm_bypass_write64(unsigned long addr, uint64_t value)
{
	unsigned long msg[VMM_DATA_SIZE];
	int ret;

	if (!bypass_status)
		return;

	msg[0] = VMM_BYPASS_WRITE|VMM_BYPASS_64BIT;
	msg[1] = addr;
	msg[2] = (unsigned long)value;

	ret = vmm_send_message(VMM_DEV_BYPASS, msg);

	if (ret != 0)
	{
		printk(KERN_ERR "vmm-bypass: Write error at 0x%lx(0x%lx)\n",
				addr, (unsigned long)(virt_to_phys((void *)addr)));
	}
}
EXPORT_SYMBOL(vmm_bypass_write64);

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/**
 *  @purpose
 *    module init routine, register driver to the kernel
 *
 *  @param
 *
 *  @returns 0 if no error
 */

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
static int vmm_bypass_probe(struct platform_device *pdev)
{
	printk(KERN_ERR PREF "probe %s.\n", pdev->name);

	/* check that vmm is on and ok */
	if (!vmm_detected())
	{
		dev_err(&pdev->dev, PREF "VMM has not been detected, " DRIVER_NAME "turned off.\n");
		return -ENODEV;
	}

    bypass_status = 1;

	pm_runtime_enable(&pdev->dev);

    return 0;
}

static int vmm_bypass_remove(struct platform_device *pdev)
{

	bypass_status = 0;

    pm_runtime_disable(&pdev->dev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

    return 0;
}

