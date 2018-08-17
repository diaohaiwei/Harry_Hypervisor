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
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>

#include "compat.h"
#include "vmm.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF			"VMAPI: "
#define DRIVER_NAME		"vmapi"
#define DRIVER_COMPAT	"vmm,vmapi"

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int vmm_vmapi_probe(struct platform_device *pdev);

static int vmm_vmapi_remove(struct platform_device *pdev);

#ifdef CONFIG_PROC_FS
static ssize_t vmm_vmapi_proc_fops_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos);
static int vmm_vmapi_proc_show(struct seq_file *m, void *v);
static int vmm_vmapi_proc_open(struct inode *inode, struct file *file);
#endif

static ssize_t vmm_vmapi_sys_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t vmm_vmapi_sys_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int vmm_vmapi_command(char *buffer, int ret);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

static const struct of_device_id vmm_vmapi_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, vmm_vmapi_match);


static struct platform_driver vmm_vmapi_driver = {
	.driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = vmm_vmapi_match,
    },
	.probe  = vmm_vmapi_probe,
	.remove = vmm_vmapi_remove,
};

#ifdef CONFIG_PROC_FS

static const struct file_operations vmm_vmapi_proc_ops = {
	.open		= vmm_vmapi_proc_open,
	.write		= vmm_vmapi_proc_fops_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release 	= single_release,
};

static struct proc_dir_entry *proc_entry = NULL;

#endif

enum vmm_vmapi_command_type {
	COMMAND_NOARGS,
	COMMAND_NUMERIC_ARG,
	COMMAND_STRING_ARG,
};

struct vmm_vmapi_command {
	const char *name;
	const char *desc;
	unsigned int id;
	enum vmm_vmapi_command_type type;
};

static const struct vmm_vmapi_command commands[] = {
	{
			.name = "reboot-partition",
			.desc = "Reboot a partition, needs a partition ID as argument",
			.id = VMM_VMAPI_REBOOT_PARTITION,
			.type = COMMAND_NUMERIC_ARG,
	},
	{
			.name = "stop-partition",
			.desc = "Stop a partition, needs a partition ID as argument",
			.id = VMM_VMAPI_STOP_PARTITION,
			.type = COMMAND_NUMERIC_ARG,
	},
	{
			.name = "reboot-target",
			.desc = "Reboot the complete target",
			.id = VMM_VMAPI_REBOOT_TARGET,
			.type = COMMAND_NOARGS,
	},
	{
			.name = "stop-target",
			.desc = "Stop the complete target",
			.id = VMM_VMAPI_STOP_TARGET,
			.type = COMMAND_NOARGS,
	},
	{
			.name = "set-sched-scheme",
			.desc ="Set the scheduling scheme, needs a schme name as argument",
			.id = VMM_VMAPI_SET_TIME_SCHED,
			.type = COMMAND_STRING_ARG,
	}
};

static DEVICE_ATTR(command, (S_IRUGO | S_IWUSR), vmm_vmapi_sys_show, vmm_vmapi_sys_store);

/**
 * Our list of bus drivers
 */

module_platform_driver(vmm_vmapi_driver);

MODULE_ALIAS("vmapi");
MODULE_AUTHOR("Bertrand Marquis (bma@sysgo.com)");
MODULE_DESCRIPTION("PikeOS virtualization VMAPI");
MODULE_LICENSE("GPL");

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */




/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */


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
static int vmm_vmapi_probe(struct platform_device *pdev)
{
	printk(KERN_ERR PREF "probe %s.\n", pdev->name);

    /* check that vmm is on and ok */
    if (!vmm_detected())
    {
    	dev_err(&pdev->dev, PREF "VMM has not been detected, Bypass turned off.\n");
    	return -ENODEV;
    }

#ifdef CONFIG_PROC_FS
    proc_entry = proc_create_data(DRIVER_NAME, 0600, 0, &vmm_vmapi_proc_ops, NULL);
    if (!proc_entry)
	{
		dev_err(&pdev->dev, PREF " cannot create a proc entry " DRIVER_NAME "\n");
		return -ENOMEM;
	}
#endif

	device_create_file(&pdev->dev, &dev_attr_command);

	if(sysfs_create_link(NULL, &pdev->dev.kobj , DRIVER_NAME) != 0)
	{
		dev_err(&pdev->dev, PREF " cannot create a sys entry %s\n", DRIVER_NAME);
		return -ENOMEM;
	}

	pm_runtime_enable(&pdev->dev);

    return 0;
}

static int vmm_vmapi_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PROC_FS
	if (proc_entry)
	{
		remove_proc_entry(DRIVER_NAME, 0);
		proc_entry = NULL;
	}
#endif

	device_remove_file(&pdev->dev,&dev_attr_command );

	sysfs_remove_link(NULL, DRIVER_NAME);

    pm_runtime_disable(&pdev->dev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

    return 0;
}

static int vmm_vmapi_command(char *buffer, int ret)
{
	char command[64];
	char str_arg[64];
	int num_arg;
	unsigned int i;
	unsigned long msg[3] = {0,0,0};

	num_arg = sscanf(buffer, "%s %s", command, str_arg);
	if (num_arg == 0)
		return -EINVAL;

	for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++)
	{
		if (strcmp(commands[i].name, command) == 0)
		{
			switch (commands[i].type) {
				case COMMAND_NUMERIC_ARG:
					if (num_arg != 2)
						return -EINVAL;
					msg[1] = simple_strtoul(str_arg, NULL, 0);
					break;
				case COMMAND_STRING_ARG:
					if (num_arg != 2)
						return -EINVAL;
					msg[1] = virt_to_phys(str_arg);
					break;
				default:
					break;
			}
			msg[0] = commands[i].id;
			ret = vmm_send_message(VMM_DEV_VMAPI, msg);
			if (ret != 0)
			{
				return -EIO;
			}
			return ret;
		}
	}

	return -EINVAL;
}

#ifdef CONFIG_PROC_FS
/**
 *  @purpose
 *    open a proc file
 *
 *  @param inode	node (not used)
 *  @param file 	file to open
 *
 *  @returns 0 if no error
 */
static int vmm_vmapi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vmm_vmapi_proc_show, PDE_DATA(inode) );
}

/**
 *  @purpose
 * 		print informations from a proc entry
 *
 *  @param page   	page addr
 *  @param start   	start addr for the read operation (page + offset)
 *  @param off   	offset for the read operation
 *  @param count   	read size
 *  @param eof   	Boolean indicates the End Of File is occured during the operation
 *  @param data   	information structure on the current device
 *
 *  @returns
 *
 *  @retval len		 	read size
 */
static int vmm_vmapi_proc_show(struct seq_file *m, void *v)
{
	unsigned int i;
	seq_printf(m,"Available commands:\n");

	for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++)
	{
		seq_printf(m, "%-18s: %s\n", commands[i].name, commands[i].desc);
	}
	return 0;
}

/**
 *  @purpose
 * 		send the cmd from proc entry to the host
 *
 *  @param file   	file concerned by the operation
 *  @param buffer   data to send
 *  @param count   	data size
 *  @param data   	information structure on the current device
 *
 *  @returns
 *
 *  @retval -EFAULT		if an error occured concerning the host operation
 *  @retval -EINVAL		if an error occured
 *  @retval 0		 	if no error
 */
static ssize_t vmm_vmapi_proc_fops_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	char tmpbuf[256];
	int ret = 0;

	if (!size)
		return -EINVAL;

	ret = size;

	if (size > 256)
		size = 256;

	/* fetch from userspace and append zero */
	if (copy_from_user(tmpbuf, buf, size))
		return -EFAULT;

	tmpbuf[size] = 0;

	return vmm_vmapi_command(tmpbuf, ret);
}

#endif

/**
 *  @purpose
 *    format command for device attribute
 *
 *  @param dev   	device concerned
 *  @param attr 	device attributes
 *  @param buf  	formated output for the attribute
 *
 *  @returns size written in buf
 */
static ssize_t vmm_vmapi_sys_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int i;
	ssize_t buff_size = 0;

	buff_size = sprintf(buf, "Available commands:\n");

	for (i = 0; i < sizeof(commands)/sizeof(commands[0]); i++)
	{
		buff_size += sprintf(&(buf[buff_size]), "%-18s: %s\n", commands[i].name, commands[i].desc);
	}

	return buff_size;
}

static ssize_t vmm_vmapi_sys_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char tmpbuf[256];
	int ret = 0;

	if (!count)
		return -EINVAL;

	ret = count;

	if (count > 256)
		count = 256;

	/* fetch from userspace and append zero */
	strncpy(tmpbuf,buf,ret);

	tmpbuf[count] = 0;

	return vmm_vmapi_command(tmpbuf, ret);
}
