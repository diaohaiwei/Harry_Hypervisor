/*
 *  Copyright (C) 2017 SYSGO AG
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2 of the
 *  License.
 */

/* ------------------------- FILE INCLUSION -------------------------------- */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <asm/mman.h>
#include <asm/pgtable.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

#include <linux/sysfs.h>
#include <linux/kobject.h>

#include "compat.h"
#include "p4bus.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF					"P4BUS_vmchar: "
#define DRIVER_NAME				"vmchar"
#define DRIVER_COMPAT			"p4bus,vmchar"

#define CLASS_NAME				"vmchar"

#define DYN_MAJOR				0
#define DEVICE_NAME_MAXSIZE		256

#define OP_ID_INVALID           0xffffffff

/* operation thread errors */
#define OP_THD_ERROR             0xdeadface
#define OP_THD_BUSY              0xfacedead

#define NOT_USED				0
#define IN_USE					1

#define FALSE                   0
#define TRUE                    1

#define PRESENT					1
#define NOT_PRESENT				0

#define MAX_FILE_DESC			128
#define FILE_DESC_UNUSED		0xffffffff

/* 3 iorings per filedescriptor */
#define NUM_IORING              0x3
/* used for READ */
#define IORING_RD               0x0
/* used for WRITE */
#define IORING_WR               0x1
/* used for all other operations */
#define IORING_3RD              0x2

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

typedef enum p4bus_op_flags_e {
	P4BUS_OP_NOK 	= 0,
	P4BUS_OP_OK 	= 1,
}p4bus_op_flags_t;


typedef struct vmchar_user_fd_s {
    /* pointer on the vmchar device */
    struct p4bus_vmchar_deviceinfo_s *vmchar_dev;

    /* file related flags */
    uint32_t    curr_fd;
    volatile uint32_t    isOpen;
}vmchar_user_fd_t;

typedef struct p4bus_vmchar_deviceinfo_s {
    /* platform device */
    struct platform_device *pdev;

    /* p4bus device informations */
	p4bus_device_info_t *p4bus_dev;

	struct list_head list;

	dev_t device;

    /* USER filedescriptor */
	vmchar_user_fd_t user_fd[MAX_FILE_DESC];

    /* wait_queue for the results */
	wait_queue_head_t wait_result;
    /* wait_queue for "exec_operation" availability */
	wait_queue_head_t wait_op;

    /* files related index */
	uint32_t fd_list[MAX_FILE_DESC];
	uint32_t nb_fd_used;

}p4bus_vmchar_deviceinfo_t;

LIST_HEAD(vmchar_char_entry);

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */
static int __init p4bus_vmchar_init(void);

static ssize_t p4bus_vmchar_read(struct file *file, char __user *buffer, size_t count, loff_t * ppos);

static ssize_t p4bus_vmchar_write(struct file *file, const char __user *buffer, size_t count, loff_t * ppos);

static int p4bus_vmchar_mmap (struct file *fd, struct vm_area_struct *vma);

static unsigned int p4bus_vmchar_poll(struct file *file, poll_table * wait);

static long p4bus_vmchar_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int p4bus_vmchar_open(struct inode *inode, struct file *file);

static int p4bus_vmchar_close(struct inode *inode, struct file *file);

static loff_t p4bus_vmchar_llseek(struct file *file, loff_t offset, int whence);

static int p4bus_vmchar_remove(struct platform_device *pdev); 

static int p4bus_vmchar_probe(struct platform_device *pdev);

static irqreturn_t p4bus_vmchar_interrupt(int irq, void *priv);

static int p4bus_vmchar_execute_op(p4bus_operation_t *curr_op, 
                                    uint32_t *operation_id, 
                                    int *err);

#ifdef CONFIG_PROC_FS
static int p4bus_vmchar_proc_open(struct inode *inode, struct file *file);

static int p4bus_vmchar_proc_show(struct seq_file *m, void *v);
#endif
/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

#if defined(CONFIG_OF)
/**
 * Our match table for the hwvirt p4bus
 */
static const struct of_device_id p4bus_vmchar_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, p4bus_vmchar_match);

#else
#define p4bus_vmchar_match NULL
#endif

static struct platform_driver p4bus_vmchar_driver = {
	.probe      = p4bus_vmchar_probe,
    .remove     = p4bus_vmchar_remove,
    .driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = p4bus_vmchar_match,
    },
};

static struct file_operations p4bus_vmchar_ops = {
      owner:			THIS_MODULE,
      read:				p4bus_vmchar_read,
      write:			p4bus_vmchar_write,
      mmap:				p4bus_vmchar_mmap,
      poll:				p4bus_vmchar_poll,
      unlocked_ioctl:	p4bus_vmchar_ioctl,
      open:				p4bus_vmchar_open,
      release:			p4bus_vmchar_close,
      llseek:			p4bus_vmchar_llseek,
};

#ifdef CONFIG_PROC_FS
static const struct file_operations p4bus_vmchar_proc_ops = {
	.open		= p4bus_vmchar_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static struct class *p4bus_vmchar_class = NULL;

static int32_t major_dyn_allocated;

static struct kobject* kobj_master;

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *p4bus_vmchar_proc_entry;
#endif

MODULE_ALIAS("platform:vmchar");
MODULE_AUTHOR("Reynaud Benjamin (bre@sysgo.fr)");
MODULE_DESCRIPTION("Char driver on PikeOS Virtualization bus");
MODULE_LICENSE("GPL");

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */




/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */
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
	p4bus_vmchar_deviceinfo_t *currDev = (p4bus_vmchar_deviceinfo_t*)dev_get_drvdata(dev);
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
    p4bus_vmchar_deviceinfo_t *currDev = (p4bus_vmchar_deviceinfo_t*)dev_get_drvdata(dev);
	return sprintf(buf, "%s", currDev->p4bus_dev->type);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static struct device_attribute p4bus_vmchar_attribs[] = {
	__ATTR_RO(guestname),
	__ATTR_RO(guesttype),
	__ATTR_NULL
};
#else
static DEVICE_ATTR_RO(guestname);
static DEVICE_ATTR_RO(guesttype);

static struct attribute *p4bus_vmchar_attrs[] = {
	&dev_attr_guestname.attr,
	&dev_attr_guesttype.attr,
	NULL,
};
ATTRIBUTE_GROUPS(p4bus_vmchar);
#endif

/**
 *  @purpose
 *    p4bus-vmchar IRQ handler. An IRQ is raised when an operation
 *    from the ioring has been processed.
 *
 *  @param irq  IRQ number
 *  @param priv pointer on a private structure
 *
 *  @returns 
 */
static irqreturn_t p4bus_vmchar_interrupt(int irq, void *priv)
{
    p4bus_vmchar_deviceinfo_t* currDev = priv;

    /* do we have someone to wakeup */
    if( waitqueue_active(&(currDev->wait_result)))
    {
        /* result queue */
        wake_up_interruptible(&(currDev->wait_result));
    }

    if (waitqueue_active(&(currDev->wait_op)))
    {
        /* free op queue */
        wake_up_interruptible(&(currDev->wait_op));
    }

    return IRQ_HANDLED;
}

/**
 *  @purpose
 *    Try to execute a p4bus operation
 *
 *  @param curr_op        pointer on the operation to execute
 *  @param operation_id   operation ID returned if succsess
 * 	@param err            pointer for error code return
 *
 *  @returns
 *  @retval FALSE(0x0)    if the caller must wait for an operation thread
 *                        availability.
 *  @retval TRUE(0x1)     if the caller does not have to wait
 */
static int p4bus_vmchar_execute_op(p4bus_operation_t *curr_op, 
                                    uint32_t *operation_id, 
                                    int *err)
{
    int32_t ret = 0;

    ret = p4bus_command_execute_operation(curr_op, operation_id);
    if(ret == P4BUS_E_INVAL_ARG)
    {
        /* Error executing the operation */
        *err = OP_THD_ERROR;
        /* don't wait */
        return TRUE;
    }
    else if (ret == P4BUS_E_BUSY)
    {
        *err = OP_THD_BUSY;
        /* don't wait */
        return TRUE;
    }
    return *operation_id != OP_ID_INVALID;
}



/**
 *  @purpose
 *    Char driver read function, read data on p4bus
 *
 *  @param file   file handled by the char driver
 *  @param buffer pointer to an user allocated buffer, will contain the read result
 *  @param count  size to read
 * 	@param ppos	  offset
 *
 *  @returns read size
 */
static ssize_t p4bus_vmchar_read(struct file *file, char __user *buffer, size_t count, loff_t * ppos)
{
	ssize_t rd_size = 0;
	int32_t fd;
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_operation_t curr_op;
	vmchar_user_fd_t* user_fd;
	char* bufferTmp;
	p4bus_op_id_t op_type;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = (vmchar_user_fd_t *) (file->private_data);
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;

    if(user_fd == NULL)
        return -ENODEV;

	if(user_fd->isOpen == FALSE)
		return -EBADF;

	/* read operation type */
	if(file->f_flags & O_NONBLOCK)
		op_type = P4BUS_OP_NB_READ;
	else
		op_type = P4BUS_OP_READ;

	/* check read operation permission */
	if ((currDev->p4bus_dev->opmask & (1<<op_type)) == 0)
		return -EACCES;

	bufferTmp = kzalloc(count, GFP_KERNEL);	
	if (!bufferTmp)
	{
		dev_err(&(currDev->pdev->dev), PREF "could not allocate memory\n");
		return -ENOMEM;
	}

    /* configure the operation */
    curr_op.type = op_type;
    curr_op.flags = OP_FLAGS_SIGNAL;
    curr_op.devid = currDev->p4bus_dev->p4bus_devid;
    curr_op.filedesc = fd;
    curr_op.param1 = 0;
    curr_op.param2 = 0;
    curr_op.addr = virt_to_phys(bufferTmp);
    curr_op.size = count;
    curr_op.status = P4BUS_OPERATION_READY;

    /* execute the operation */
    do {
        if(wait_event_interruptible(currDev->wait_op, 
                p4bus_vmchar_execute_op(&curr_op, 
                                &operation_id, 
                                &exec_err) ) == -ERESTARTSYS)
        {
            /* Signal catched */
            kfree(bufferTmp);
            return -ERESTARTSYS;
        }

        if (exec_err == OP_THD_BUSY)
        {
            if(op_type != P4BUS_OP_NB_READ)
                dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the READ operation\n");
                
            /* cannot execute the operation for NB_READ */
            kfree(bufferTmp);
            return -EAGAIN;
        }
        else if (exec_err == OP_THD_ERROR)
        {
            dev_err(&(currDev->pdev->dev), PREF "Error during READ operation on vmchar\n");
            kfree(bufferTmp);
            return -EIO;
        }
    }while(operation_id == OP_ID_INVALID);

    /* wait for a result */
    if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
    {
        /* catch the kill signal */
        p4bus_command_cancel_operation(operation_id);
        kfree(bufferTmp);
        return -ERESTARTSYS;
    }

    if (curr_op.retcode == P4BUS_E_OK)
    {
        rd_size = curr_op.size;
        *ppos += rd_size;
        if(rd_size == 0 && op_type == P4BUS_OP_NB_READ)
        {
            /* this case means that no data were available on the host side */
            rd_size = -EAGAIN;
        }
    }
    else
    {
        dev_err(&(currDev->pdev->dev), PREF " %s Error READ operation returns an error: 0x%x\n",__func__, curr_op.retcode);
        if(curr_op.retcode >= P4BUS_E_P4)
        {
            rd_size = -EINVAL;
        }
        else
        {
            rd_size = -EIO;
        }
    }

    if (rd_size > 0)
    {
        if(copy_to_user(buffer, bufferTmp, rd_size))
        {
            dev_err(&(currDev->pdev->dev), PREF "%s copy_to_user ERROR\n",__func__);
            rd_size = -ENOMEM;
        }
    }
    kfree(bufferTmp);
	return rd_size;
}

/**
 *  @purpose
 *    Char driver write function, write data on p4bus
 *
 *  @param file   file handled by the char driver
 *  @param buffer data to write
 *  @param count  buffer size
 * 	@param ppos	  offset
 *
 *  @returns size written
 */
static ssize_t p4bus_vmchar_write(struct file *file, const char __user *buffer, size_t count, loff_t * ppos)
{
	ssize_t wr_size = 0;
	char* bufferTmp;
	int32_t fd;
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_operation_t curr_op;
	vmchar_user_fd_t* user_fd;
    p4bus_op_id_t op_type;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = (vmchar_user_fd_t *) (file->private_data);
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;

    if(user_fd == NULL)
        return -ENODEV;

	if(user_fd->isOpen == FALSE)
		return -EBADF;

	/* write operation type */
	if(file->f_flags & O_NONBLOCK)
		op_type = P4BUS_OP_NB_WRITE;
	else
		op_type = P4BUS_OP_WRITE;

	/* check write operation permission */
	if ((currDev->p4bus_dev->opmask & (1<<op_type)) == 0)
		return -EACCES;

    /* allocate a buffer */
	bufferTmp = kzalloc(count, GFP_KERNEL);
	if (!bufferTmp)
	{
		dev_err(&(currDev->pdev->dev), PREF "could not allocate memory\n");
		return -ENOMEM;
	}

	if(copy_from_user(bufferTmp, buffer, count))
	{
		dev_err(&(currDev->pdev->dev), PREF "%s copy_from_user ERROR\n",__func__);
        kfree(bufferTmp);
		return -ENOMEM;
	}

    /* configure the operation */
    curr_op.type = op_type;
    curr_op.flags = OP_FLAGS_SIGNAL;
    curr_op.devid = currDev->p4bus_dev->p4bus_devid;
    curr_op.filedesc = fd;
    curr_op.param1 = 0;
    curr_op.param2 = 0;
    curr_op.addr = virt_to_phys(bufferTmp);
    curr_op.size = count;
    curr_op.status = P4BUS_OPERATION_READY;


    /* execute the operation */
    do {
        if(wait_event_interruptible(currDev->wait_op, 
                p4bus_vmchar_execute_op(&curr_op, 
                                &operation_id, 
                                &exec_err) ) == -ERESTARTSYS)
        {
            /* Signal catched */
            kfree(bufferTmp);
            return -ERESTARTSYS;
        }

        if (exec_err == OP_THD_BUSY)
        {
            if(op_type != P4BUS_OP_NB_WRITE)
                dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the WRITE operation\n");
                
            /* cannot execute the operation for NB_WRITE */
            kfree(bufferTmp);
            return -EAGAIN;
        }
        else if (exec_err == OP_THD_ERROR)
        {
            dev_err(&(currDev->pdev->dev), PREF "Error during WRITE operation on vmchar\n");
            kfree(bufferTmp);
            return -EIO;
        }
    }while(operation_id == OP_ID_INVALID);

    /* wait for a result */
    if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
    {
        /* catch the kill signal */
        p4bus_command_cancel_operation(operation_id);
        kfree(bufferTmp);
        return -ERESTARTSYS;
    }

    if (curr_op.retcode == P4BUS_E_OK)
    {
        wr_size = curr_op.size;
        *ppos += wr_size;
        if(wr_size == 0 && op_type == P4BUS_OP_NB_WRITE)
        {
            /* this case means that no data was available on the host side */
            wr_size = -EAGAIN;
        }
        else if (wr_size == 0 && op_type == P4BUS_OP_WRITE)
            wr_size = -EFBIG;
    }
    else
    {
        dev_err(&(currDev->pdev->dev), PREF " %s Error WRITE operation returns an error: 0x%x\n",__func__, curr_op.retcode);
        if(curr_op.retcode >= P4BUS_E_P4)
        {
            wr_size = -EINVAL;
        }
        else
        {
            wr_size = -EIO;
        }
    }
    kfree(bufferTmp);
	return wr_size;
}

/**
 *  @purpose
 *    mmap function
 *
 *  @param fd   File handled by the char driver
 *  @param vma  The virtual memory area into which the page range is being mapped.
 *
 *  @returns
 */
static int p4bus_vmchar_mmap (struct file *filedesc, struct vm_area_struct *vma)
{
	p4bus_vmchar_deviceinfo_t *currDev;
	vmchar_user_fd_t* user_fd;
    p4bus_operation_t curr_op;
	int flags = 0;
	unsigned long size = 0;
	unsigned long phys_addr;
	unsigned long current_mmap_size = 0;
	unsigned long mmapped_size = 0;
	unsigned long inblock_offset;
	unsigned long current_offset;
	uint32_t fd;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = (vmchar_user_fd_t *) (filedesc->private_data);
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;

    if(user_fd == NULL)
        return -ENODEV;

	if(user_fd->isOpen == FALSE)
		return -EBADF;

	/* check access rigths */
	if ((currDev->p4bus_dev->opmask & (1<<P4BUS_OP_MMAP)) == 0)
		 return -EACCES;

	/* flags handling */
	flags = P4BUS_FLAGS_MMAP_READ;
	
	if((vma->vm_flags & VM_WRITE) != 0)
		flags |= P4BUS_FLAGS_MMAP_WRITE;
	if((vma->vm_flags & VM_READ) != 0)
		flags |= P4BUS_FLAGS_MMAP_READ;
	if((vma->vm_flags & VM_EXEC) != 0)
	{
		dev_err(&(currDev->pdev->dev), PREF " %s no access rights for PROT_EXEC\n",__func__);
		return -EACCES;	
	}

	/* mmap size */
	size = vma->vm_end - vma->vm_start;
	current_offset = vma->vm_pgoff * PAGE_SIZE;


	while(mmapped_size < size)
	{
		/* get the offset inside a block */
		inblock_offset = current_offset & ~MAPPING_GRANULARITY_MASK;

		/* mmap recquirement is bigger than a block size, we have to cut */
		if((inblock_offset + (size - mmapped_size)) > MAPPING_GRANULARITY) 
		{
			/* size to mmap */
			current_mmap_size = MAPPING_GRANULARITY - inblock_offset;
		}
		else /* we can mmap directly */
		{
			current_mmap_size = size - mmapped_size;
		}

        /* configure the operation */
        curr_op.type = P4BUS_OP_MMAP;
        curr_op.flags = OP_FLAGS_SIGNAL;
        curr_op.devid = currDev->p4bus_dev->p4bus_devid;
        curr_op.filedesc = fd;
        curr_op.addr = 0;
        curr_op.size = current_mmap_size;
        curr_op.param1 = current_offset;
        curr_op.param2 = flags;
        curr_op.status = P4BUS_OPERATION_READY;

        operation_id = OP_ID_INVALID;
        exec_err = 0;

        /* execute the operation */
        do {
            if(wait_event_interruptible(currDev->wait_op, 
                    p4bus_vmchar_execute_op(&curr_op, 
                                    &operation_id, 
                                    &exec_err) ) == -ERESTARTSYS)
            {
                /* Signal catched */
                return -ERESTARTSYS;
            }

            if (exec_err == OP_THD_BUSY)
            {
                /* operation thread busy */
                dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the MMAP operation\n");
                return -EAGAIN;
            }
            else if (exec_err == OP_THD_ERROR)
            {
                dev_err(&(currDev->pdev->dev), PREF "Error during MMAP operation on vmchar\n");
                return -EIO;
            }
        }while(operation_id == OP_ID_INVALID);

        /* wait for a result */
        if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
        {
            /* catch the kill signal */
            p4bus_command_cancel_operation(operation_id);
            return -ERESTARTSYS;
        }

        if (curr_op.retcode == P4BUS_E_OK)
        {
            phys_addr = (unsigned long)curr_op.addr;

            if (phys_addr & (PAGE_SIZE - 1))
            {
                return -ENOSYS;
            }

            if (remap_pfn_range(vma, vma->vm_start + mmapped_size,
                      phys_addr >> PAGE_SHIFT, current_mmap_size, vma->vm_page_prot))
            {
                return -EAGAIN;
            }

            mmapped_size += current_mmap_size;
            current_offset += current_mmap_size;
        }
        else
        {
            dev_err(&(currDev->pdev->dev), PREF " %s Error MMAP operation returns an error: 0x%x\n",__func__, curr_op.retcode);
            if(curr_op.retcode >= P4BUS_E_P4)
            {
                return -EINVAL;
            }
            else
            {
                return -EIO;
            }
        }
    }

	return 0;
}
/**
 *  @purpose
 *    poll function (not supported)
 *
 *  @param file   file handled by the char driver
 *  @param wait   poll table
 *
 *  @returns
 */
static unsigned int p4bus_vmchar_poll(struct file *file, poll_table * wait)
{
	u_int32_t ret = 0;
	return ret;
}

/**
 *  @purpose
 *    send specific commands to the device
 *
 *  @param file   	file handled by the char driver
 *  @param cmd 		command
 *  @param arg  	undefined argument
 *
 *  @returns
 * 
 *  @retval -EBUSY   if operation is busy
 *  @retval -EACCES  if operation is not allowed
 *  @retval -EIO  	 for other error 
 *  @retval 0		 if no error
 */
static long p4bus_vmchar_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	p4bus_vmchar_deviceinfo_t *currDev;
	vmchar_user_fd_t* user_fd;
    p4bus_operation_t curr_op;
    p4bus_op_id_t op_type;
	unsigned long* bufferTmp;
	uint32_t size_in = 0;
	uint32_t size_out = 0;
	uint32_t fd;
	long ret = 0;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = (vmchar_user_fd_t *) (file->private_data);
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;

    if(user_fd == NULL)
        return -ENODEV;

	if(user_fd->isOpen == FALSE)
		return -EBADF;

	if (_IOC_NR(cmd) == P4BUS_IOCTL_STAT)
		op_type = P4BUS_OP_STAT;
	else
		op_type = P4BUS_OP_IOCTL;

	/* check read operation permission */
	if ((currDev->p4bus_dev->opmask & (1<<op_type)) == 0)
		return -EACCES;


	switch ( _IOC_NR(cmd))
	{
		case P4BUS_IOCTL_STAT:

            bufferTmp = kzalloc(PIKEOS_STAT_STRUCT_SIZE, GFP_KERNEL);	
            if (!bufferTmp)
            {
                dev_err(&(currDev->pdev->dev), PREF "could not allocate memory\n");
                return -ENOMEM;
            }

            if(copy_from_user( bufferTmp, (void __user *)arg , PIKEOS_STAT_STRUCT_SIZE ))
            {
                kfree(bufferTmp);
                return -EIO;
            }

            /* configure the operation */
            curr_op.type = P4BUS_OP_STAT;
            curr_op.flags = OP_FLAGS_SIGNAL;
            curr_op.devid = currDev->p4bus_dev->p4bus_devid;
            curr_op.filedesc = fd;
            curr_op.addr = virt_to_phys(bufferTmp);
            curr_op.size = PIKEOS_STAT_STRUCT_SIZE;
            curr_op.param1 = 0;
            curr_op.param2 = 0;
            curr_op.status = P4BUS_OPERATION_READY;

            /* execute the operation */
            do {
                if(wait_event_interruptible(currDev->wait_op, 
                        p4bus_vmchar_execute_op(&curr_op, 
                                        &operation_id, 
                                        &exec_err) ) == -ERESTARTSYS)
                {
                    /* Signal catched */
                    kfree(bufferTmp);
                    return -ERESTARTSYS;
                }

                if (exec_err == OP_THD_BUSY)
                {
                    /* operation thread busy */
                    dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the IOCTL operation\n");
                    kfree(bufferTmp);
                    return -EAGAIN;
                }
                else if (exec_err == OP_THD_ERROR)
                {
                    dev_err(&(currDev->pdev->dev), PREF "Error during IOCTL operation on vmchar\n");
                    kfree(bufferTmp);
                    return -EIO;
                }
            }while(operation_id == OP_ID_INVALID);

            /* wait for a result */
            if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
            {
                /* catch the kill signal */
                p4bus_command_cancel_operation(operation_id);
                kfree(bufferTmp);
                return -ERESTARTSYS;
            }

            if (curr_op.retcode == P4BUS_E_OK)
            {
                /* No Error, copy the buffer */
                if(copy_to_user((void __user *)arg, bufferTmp,  PIKEOS_STAT_STRUCT_SIZE ))
                    ret = -ENOMEM;
            }
            else
            {
                dev_err(&(currDev->pdev->dev), PREF " %s Error IOCTL operation returns an error: 0x%x\n",__func__, curr_op.retcode);
                if(curr_op.retcode >= P4BUS_E_P4)
                {
                    ret = -EINVAL;
                }
                else
                {
                    ret = -EIO;
                }
            }
            kfree(bufferTmp);
            break;
		
		case P4BUS_IOCTL_NATIVE:

            bufferTmp = kzalloc(PIKEOS_IOCTL_STRUCT_SIZE, GFP_KERNEL);
            if (!bufferTmp)
            {
                return -ENOMEM;
            }
            if (copy_from_user(bufferTmp, (void __user *)arg, PIKEOS_IOCTL_STRUCT_SIZE))
            {
                dev_err(&(currDev->pdev->dev), PREF " copy from user error\n");
                kfree(bufferTmp);
                return  -EINVAL;
            }


            size_in = bufferTmp[PIKEOS_IOCTL_SIZE_IN_FIELD];
            size_out = bufferTmp[PIKEOS_IOCTL_SIZE_OUT_FIELD];

            /* configure the operation */
            curr_op.type = P4BUS_OP_IOCTL;
            curr_op.flags = OP_FLAGS_SIGNAL;
            curr_op.devid = currDev->p4bus_dev->p4bus_devid;
            curr_op.filedesc = fd;
            curr_op.addr = virt_to_phys(&bufferTmp[PIKEOS_IOCTL_DATA_FIELD]);
            curr_op.size = max(size_in, size_out);
            curr_op.param1 = bufferTmp[PIKEOS_IOCTL_CMD_FIELD];
            curr_op.param2 = (( size_in & PIKEOS_IOCTL_SIZE_MASK) << 16) | (( size_out & PIKEOS_IOCTL_SIZE_MASK));
            curr_op.status = P4BUS_OPERATION_READY;


            /* check if all fields are valid */
            if(	( curr_op.param1	& ~PIKEOS_IOCTL_CMD_MASK ) 	||
                ( size_in 	        & ~PIKEOS_IOCTL_SIZE_MASK ) ||
                ( size_out 	        & ~PIKEOS_IOCTL_SIZE_MASK )  	)
            {
                dev_err(&(currDev->pdev->dev), PREF " %s Error: the parameters format for P4BUS_IOCTL_NATIVE is not valid\n",__func__);
                kfree(bufferTmp);
                return -EINVAL;
            }

            /* execute the operation */
            do {
                if(wait_event_interruptible(currDev->wait_op, 
                        p4bus_vmchar_execute_op(&curr_op, 
                                        &operation_id, 
                                        &exec_err) ) == -ERESTARTSYS)
                {
                    /* Signal catched */
                    kfree(bufferTmp);
                    return -ERESTARTSYS;
                }

                if (exec_err == OP_THD_BUSY)
                {
                    /* operation thread busy */
                    dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the P4BUS_IOCTL_NATIVE operation\n");
                    kfree(bufferTmp);
                    return -EAGAIN;
                }
                else if (exec_err == OP_THD_ERROR)
                {
                    dev_err(&(currDev->pdev->dev), PREF "Error during P4BUS_IOCTL_NATIVE operation on vmchar\n");
                    kfree(bufferTmp);
                    return -EIO;
                }
            }while(operation_id == OP_ID_INVALID);

            /* wait for a result */
            if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
            {
                /* catch the kill signal */
                p4bus_command_cancel_operation(operation_id);
                kfree(bufferTmp);
                return -ERESTARTSYS;
            }

            if (curr_op.retcode == P4BUS_E_OK)
            {
                if (copy_to_user((void __user *)arg, bufferTmp, PIKEOS_IOCTL_STRUCT_SIZE))
                {
                    dev_err(&(currDev->pdev->dev), PREF " copy to user error\n");
                    ret = -EINVAL;
                }
            }
            else
            {
                if(curr_op.retcode >= P4BUS_E_P4)
                {
                    ret = -EINVAL;
                }
                else
                {
                    ret = -EIO;
                }
            }
            kfree(bufferTmp);
            break;

		default:
				dev_err(&(currDev->pdev->dev), PREF " Invalid IOCTL command\n");
				ret = -EINVAL;
			break;
	}

	return ret;
}

/**
 *  @purpose
 *    check access permissions and open char device
 *
 *  @param inode   	inode attached to this driver
 *  @param file 	file structure handled by this driver
 * 
 *  @returns
 * 
 *  @retval -EBUSY   if device is busy
 *  @retval -EACCES  if access is not allowed
 *  @retval -EIO  if operation failed
 *  @retval 0		 if no error
 */
static int p4bus_vmchar_open(struct inode *inode, struct file *file)
{
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_device_info_t *dev_info;
    p4bus_operation_t curr_op;
    vmchar_user_fd_t *user_fd;
	uint32_t minor = iminor(inode);
	int32_t flags = 0;
	uint32_t fd = 0;
    int32_t ret = 0;
	int32_t res = 0;
	uint32_t i;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	if (!list_empty(&vmchar_char_entry))
	{
		list_for_each_entry(currDev, &vmchar_char_entry, list)
		{
			if(currDev->p4bus_dev->type_devid == minor)
			{
				res = 1;
				break;
			}
		}
	}

	if(res)
	{
		if(currDev->nb_fd_used < MAX_FILE_DESC)
		{
            /* check for a free FD */
			/* TODO: protect fd_list with a spinlock */
			for(i = 0; i < MAX_FILE_DESC ; i++)
			{
				if(currDev->fd_list[i] == FILE_DESC_UNUSED)
				{
					currDev->fd_list[i] = i;
					fd = currDev->fd_list[i];
					break;
				}
			}

            /* set FD as used */
            currDev->nb_fd_used++;

            /* initialize the USER operation structure */
            user_fd = &(currDev->user_fd[fd]);
            user_fd->curr_fd = fd;
            user_fd->vmchar_dev = currDev;

            /* set the file private DATA pointer */
			file->private_data = user_fd;
		}
		else
		{
			/* this file is already used by MAX_FILE_DESC processes */
			return -EMFILE;
		}
	}
	else
		return -ENODEV;


    dev_info = currDev->p4bus_dev;

	/* check if access flags required are allowed */
	if( (((file->f_flags & O_ACCMODE) == O_WRONLY) || ((file->f_flags & O_ACCMODE) == O_RDWR) ) && 	/* WRITE access is needed 	&& 		*/
				(file->f_flags & O_NONBLOCK) &&														/* NONBLOCKING is needed 	&& 		*/
				((dev_info->opmask & (1<<P4BUS_OP_NB_WRITE)) == 0) )							/* NONBLOCKING WRITE is not allowed	*/
		return -EACCES;
	
	if( (((file->f_flags & O_ACCMODE) == O_RDONLY) || ((file->f_flags & O_ACCMODE) == O_RDWR) ) && 	/* READ access is needed 	&& 		*/
				(file->f_flags & O_NONBLOCK) &&														/* NONBLOCKING is needed 	&& 		*/
				((dev_info->opmask & (1<<P4BUS_OP_NB_READ)) == 0))							/* NONBLOCKING READ is not allowed	*/
		return -EACCES;
		
	/* flags requested for the OPEN function */
	if((file->f_flags & O_ACCMODE) == O_RDWR)  	/* READ and WRITE access are needed */
	{
		if((dev_info->opmask & (1<<P4BUS_OP_READ)) == 0 || (dev_info->opmask & (1<<P4BUS_OP_WRITE)) == 0 ) 	/* READ is not allowed */
		{
			return -EACCES;
		}
		else
		{
			flags = P4BUS_FLAGS_OPEN_RW;
		}
	}
	else if((file->f_flags & O_ACCMODE) == O_RDONLY)  	/* READ access is needed */
	{
		if((dev_info->opmask & (1<<P4BUS_OP_READ)) == 0) 	/* READ is not allowed */
		{
			return -EACCES;
		}
		else
		{
			flags = P4BUS_FLAGS_OPEN_READ;
		}
	}
	else if((file->f_flags & O_ACCMODE) == O_WRONLY)  	/* WRITE access is needed */
	{
		if((dev_info->opmask & (1<<P4BUS_OP_WRITE)) == 0) 	/* WRITE is not allowed */
		{
			return -EACCES;
		}
		else
		{
			flags = P4BUS_FLAGS_OPEN_WRITE;
		}
	}

	user_fd->isOpen = TRUE;
	if(((dev_info->opmask & (1<<P4BUS_OP_MMAP)) != 0))
		flags |= P4BUS_FLAGS_OPEN_MAP;
 
 
	/* check if open operation is allowed */
	if ((dev_info->opmask & (1<<P4BUS_OP_OPEN)) != 0)
	{
        /* configure the operation */
        curr_op.type = P4BUS_OP_OPEN;
        curr_op.flags = OP_FLAGS_SIGNAL;
        curr_op.devid = currDev->p4bus_dev->p4bus_devid;
        curr_op.filedesc = fd;
        curr_op.addr = 0;
        curr_op.param1 = flags;
        curr_op.param2 = 0;
        curr_op.status = P4BUS_OPERATION_READY;

        /* push the operation */
        do {
            if(wait_event_interruptible(currDev->wait_op, 
                    p4bus_vmchar_execute_op(&curr_op, 
                                    &operation_id, 
                                    &exec_err) ) == -ERESTARTSYS)
            {
                /* Signal catched */
                return -ERESTARTSYS;
            }

            if (exec_err == OP_THD_BUSY)
            {
                /* operation thread busy */
                dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the OPEN operation\n");
                return -EAGAIN;
            }
            else if (exec_err == OP_THD_ERROR)
            {
                dev_err(&(currDev->pdev->dev), PREF "Error during OPEN operation on vmchar\n");
                return -EIO;
            }
        }while(operation_id == OP_ID_INVALID);

        /* wait for a result */
        if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
        {
            /* catch the kill signal */
            p4bus_command_cancel_operation(operation_id);
            return -ERESTARTSYS;
        }

        if (curr_op.retcode != P4BUS_E_OK)
        {
            dev_err(&(currDev->pdev->dev), PREF " %s Error OPEN operation returns an error: 0x%x\n",__func__, curr_op.retcode);
            user_fd->isOpen = FALSE;
            if(curr_op.retcode >= P4BUS_E_P4)
            {
                ret = -EINVAL;
            }
            else
            {
                ret = -EIO;
            }
        }
        else
        {
            user_fd->isOpen = TRUE;
        }
	}

    return ret;
}

/**
 *  @purpose
 *    release resources and close the device
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
static int p4bus_vmchar_close(struct inode *inode, struct file *file)
{
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_device_info_t *dev_info;
    p4bus_operation_t curr_op;
    vmchar_user_fd_t *user_fd;
    int32_t ret = 0;
	uint32_t fd;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = file->private_data;
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;
    dev_info = currDev->p4bus_dev;

	DBG("Close called\n");

	if(currDev == NULL)
		ret = -EIO;

	if(ret == 0 && user_fd->isOpen == FALSE)
		ret = -EBADF;

	/* check if close operation is allowed */
	if (ret == 0 && (dev_info->opmask & (1<<P4BUS_OP_CLOSE)) != 0)
	{

        /* configure the operation */
        curr_op.type = P4BUS_OP_CLOSE;
        curr_op.flags = OP_FLAGS_SIGNAL;
        curr_op.devid = dev_info->p4bus_devid;
        curr_op.filedesc = fd;
        curr_op.addr = 0;
        curr_op.size = 0;
        curr_op.param1 = 0;
        curr_op.param2 = 0;
        curr_op.status = P4BUS_OPERATION_READY;

        /* execute the operation */
        do {
            if(wait_event_interruptible(currDev->wait_op, 
                    p4bus_vmchar_execute_op(&curr_op, 
                                    &operation_id, 
                                    &exec_err) ) == -ERESTARTSYS)
            {
                /* Signal catched */
                return -ERESTARTSYS;
            }

            if (exec_err == OP_THD_BUSY)
            {
                /* cannot execute the operation for NB_READ */
                dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the CLOSE operation\n");
                return -EAGAIN;
            }
            else if (exec_err == OP_THD_ERROR)
            {
                dev_err(&(currDev->pdev->dev), PREF "Error during CLOSE operation on vmchar\n");
                return -EIO;
            }
        }while(operation_id == OP_ID_INVALID);

        /* wait for a result */
        if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
        {
            /* catch the kill signal */
            p4bus_command_cancel_operation(operation_id);
            return -ERESTARTSYS;
        }

        if (curr_op.retcode == P4BUS_E_OK)
        {
        	user_fd->isOpen = FALSE;
        }
        else
        {
            dev_err(&(currDev->pdev->dev), PREF " %s Error CLOSE operation returns an error: 0x%x\n",__func__, curr_op.retcode);
            return -EIO;
        }
	}
	else if (ret == 0)
	{
        /* close operation not allowed, just simulate it */
		user_fd->isOpen = FALSE;
		ret = 0;
	}

    /* free the current fd */
    currDev->fd_list[user_fd->curr_fd] = FILE_DESC_UNUSED;
    currDev->nb_fd_used--;

	DBG("Close return %d\n", ret);

	return ret;
}

/**
 *  @purpose
 *    reposition the read/write file offset
 *
 *  @param file   	file concerned by the reposition
 *  @param offset   offset value to apply
 *  @param whence   SEEK_SET, SEEK_CUR, or SEEK_END
 *
 *  @returns
 */
static loff_t p4bus_vmchar_llseek(struct file *file, loff_t offset, int whence)
{
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_device_info_t *dev_info;
    p4bus_operation_t curr_op;
    vmchar_user_fd_t *user_fd;
	loff_t new_offset = 0;
	uint32_t fd;
	int origin = 0;
    uint32_t operation_id = OP_ID_INVALID;
    uint32_t exec_err = 0;

	user_fd = (vmchar_user_fd_t *) (file->private_data);
	currDev = user_fd->vmchar_dev;
	fd = user_fd->curr_fd;
    dev_info = currDev->p4bus_dev;

    if(user_fd == NULL)
        return -ENODEV;

	if(user_fd->isOpen == FALSE)
		return -EBADF;

	/* check if lseek operation is allowed */
	if ((dev_info->opmask & (1<<P4BUS_OP_LSEEK)) == 0)
		return -EACCES;

	/* check whence */
	switch(whence)
	{
		case SEEK_SET:
			origin = P4BUS_FLAGS_SEEK_SET;
		break;

		case SEEK_CUR:
			origin = P4BUS_FLAGS_SEEK_CUR;
		break;

		case SEEK_END:
			origin = P4BUS_FLAGS_SEEK_END;
		break;

		default:
			return -EINVAL;
		break;
	}

    /* configure the operation */
    curr_op.type = P4BUS_OP_LSEEK;
    curr_op.flags = OP_FLAGS_SIGNAL;
    curr_op.devid = dev_info->p4bus_devid;
    curr_op.filedesc = fd;
    curr_op.addr = 0;
    curr_op.size = 0;
    curr_op.param1 = offset;
    curr_op.param2 = origin;
    curr_op.status = P4BUS_OPERATION_READY;

    /* execute the operation */
    do {
        if(wait_event_interruptible(currDev->wait_op, 
                p4bus_vmchar_execute_op(&curr_op, 
                                &operation_id, 
                                &exec_err) ) == -ERESTARTSYS)
        {
            /* Signal catched */
            return -ERESTARTSYS;
        }

        if (exec_err == OP_THD_BUSY)
        {
            /* operation thread busy */
            dev_err(&(currDev->pdev->dev), PREF "Error no operation threads left to execute the LSEEK operation\n");
            return -EAGAIN;
        }
        else if (exec_err == OP_THD_ERROR)
        {
            dev_err(&(currDev->pdev->dev), PREF "Error during LSEEK operation on vmchar\n");
            return -EIO;
        }
    }while(operation_id == OP_ID_INVALID);

    /* wait for a result */
    if (wait_event_interruptible(currDev->wait_result, (curr_op.status == P4BUS_OPERATION_DONE)) == -ERESTARTSYS)
    {
        /* catch the kill signal */
        p4bus_command_cancel_operation(operation_id);
        return -ERESTARTSYS;
    }

    if (curr_op.retcode == P4BUS_E_OK)
    {
        /* no error, return the new offset */
        new_offset = curr_op.addr;
        return new_offset;
    }
    else
    {
        dev_err(&(currDev->pdev->dev), PREF " %s Error LSEEK operation return an error: 0x%x\n",__func__,curr_op.retcode);
        return -EACCES;
    }
}

/**
 *  @purpose
 *    Remove device from P4 bus and kernel and release memory resources
 *
 *  @param pdev   platform device concerned
 *
 *  @returns
 */
static int p4bus_vmchar_remove(struct platform_device *pdev)
{
	p4bus_vmchar_deviceinfo_t *currDev;

	currDev = dev_get_drvdata(&(pdev->dev));
	if (!currDev)
	{
		dev_err(&pdev->dev, PREF "Invalid vmchar device\n");
		return -EINVAL;
	}

	/* remove current entry from the list */
	list_del( &(currDev->list) );

	/* destroy the device */
	device_destroy(p4bus_vmchar_class, currDev->device);

	sysfs_remove_link(kobj_master, currDev->p4bus_dev->name);

	/* free entry resource */
    devm_free_irq(&pdev->dev, currDev->p4bus_dev->interrupt, currDev);

    devm_kfree(&pdev->dev, currDev->p4bus_dev);

    devm_kfree(&pdev->dev, currDev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

	return 0;
}

/**
 *  @purpose
 *    request and init memory resources for the device.
 * 		register the device on the P4 bus
 *
 *  @param pdev   platform device concerned
 *
 *  @returns 0 if no error
 */
static int p4bus_vmchar_probe(struct platform_device *pdev)
{

	char device_name[DEVICE_NAME_MAXSIZE];
	struct device *mydev;
	int ret = 0;
	p4bus_vmchar_deviceinfo_t *currDev;
    p4bus_device_info_t *p4bus_device = NULL;
	int i = 0;

	printk(KERN_ERR PREF "probe %s.\n", pdev->name);

	/* check the P4BUS status */
	if(!p4bus_detected())
	{
		dev_err(&pdev->dev, PREF "no p4bus detected.\n");
		return -ENODEV;
	}

	currDev = devm_kzalloc(&pdev->dev, sizeof(p4bus_vmchar_deviceinfo_t), GFP_KERNEL);
	if (!currDev)
	{
		dev_err(&pdev->dev, PREF " could not allocate memory\n");
		return -ENOMEM;
	}

    p4bus_device = devm_kzalloc(&pdev->dev, sizeof(p4bus_device_info_t), GFP_KERNEL);
	if (!p4bus_device)
	{
		dev_err(&pdev->dev, PREF " could not allocate memory\n");
		return -ENOMEM;
	}

    currDev->p4bus_dev = p4bus_device;

	/* register device on p4bus an fill the dev_info structure */
	ret = p4bus_get_devinfo(pdev, p4bus_device);
	if (ret)
	{
		dev_err(&pdev->dev, PREF  "could not get p4bus device info\n");
		devm_kfree(&pdev->dev, p4bus_device);
        devm_kfree(&pdev->dev, currDev);
		return -EINVAL;
	}

	/* Current Device init */
	currDev->pdev = pdev;
	currDev->nb_fd_used = 0;

    /* FD table initialization */
	for(i = 0; i < MAX_FILE_DESC; i++)
	{
		currDev->fd_list[i] = FILE_DESC_UNUSED;
	}

    /* request the end of operation IRQ */
    if (devm_request_irq(&pdev->dev, p4bus_device->interrupt, p4bus_vmchar_interrupt,
                                IRQF_SHARED, dev_name(&pdev->dev), currDev) != 0)
	{
    	dev_err(&pdev->dev, PREF "cannot attach to the interrupt\n");
    	ret = -EIO;
    	goto err0;
	}

    /* init device wait_queues */
    init_waitqueue_head(&(currDev->wait_result));
    init_waitqueue_head(&(currDev->wait_op));

	INIT_LIST_HEAD(&currDev->list);

	/* create the device */
	sprintf(device_name,"%s%u",CLASS_NAME,currDev->p4bus_dev->type_devid);
	currDev->device = MKDEV(major_dyn_allocated, currDev->p4bus_dev->type_devid);

	mydev = device_create(p4bus_vmchar_class, NULL, currDev->device, currDev, device_name);
	if (IS_ERR(mydev))
	{
		ret = PTR_ERR(mydev);
		dev_err(&pdev->dev, PREF " failed to create device '%s'  err= %d \n", device_name, ret);
        goto err1;
	}
	else
	{
#ifdef CONFIG_SYSFS
		if(sysfs_create_link(kobj_master, &mydev->kobj , currDev->p4bus_dev->name) != 0)
		{
			printk("ERROR during sysfs_create_link for %s\n",currDev->p4bus_dev->name);
		}
#endif
        list_add_tail(&currDev->list, &vmchar_char_entry);
	}
	
	dev_set_drvdata(&(pdev->dev), currDev);

	return ret;

    /* ERROR handling */
err1:
    devm_free_irq(&pdev->dev, p4bus_device->interrupt, currDev);
err0:
    devm_kfree(&pdev->dev, p4bus_device);
    devm_kfree(&pdev->dev, currDev);
    return ret;
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
static int p4bus_vmchar_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, p4bus_vmchar_proc_show, NULL);
}
#endif

#ifdef CONFIG_PROC_FS
/**
 *  @purpose
 *    prints proc informations regarding this driver
 *
 *  @param m	seq file in which all informations will be write
 *
 *  @returns
 */
static int p4bus_vmchar_proc_show(struct seq_file *m, void *v)
{
	p4bus_vmchar_deviceinfo_t *currDev = NULL;
	int32_t i=0;

	seq_printf(m,"# <minor> <guest_name>\n");

	if (!list_empty(&vmchar_char_entry))
	{
		list_for_each_entry(currDev, &vmchar_char_entry, list)
		{
			seq_printf(m,"%d %d %s\n",i , currDev->p4bus_dev->type_devid, currDev->p4bus_dev->name);
			i++;
		}
	}
	return 0;
}
#endif

/**
 *  @purpose
 *    module init routine, create class and register driver to the kernel
 *
 *  @param 
 *
 *  @returns
 */
static int __init p4bus_vmchar_init(void)
{
	int ret = 0;

	printk(KERN_ERR PREF "init.\n");

	/* register driver */
	major_dyn_allocated = register_chrdev(DYN_MAJOR, DRIVER_NAME, &p4bus_vmchar_ops);
	if (major_dyn_allocated < 0) {
		printk(KERN_ERR PREF
		       " in module __init: could not register major number, error %d\n", ret);
	}

	p4bus_vmchar_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(p4bus_vmchar_class)) {
		printk(KERN_ERR PREF "in module __init: failed to register the device class '%s'\n", CLASS_NAME);
		return PTR_ERR(p4bus_vmchar_class);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	p4bus_vmchar_class->dev_attrs = p4bus_vmchar_attribs;
#else
	p4bus_vmchar_class->dev_groups = p4bus_vmchar_groups;
#endif

#ifdef CONFIG_PROC_FS
	/* register proc entry */
	p4bus_vmchar_proc_entry = proc_create(CLASS_NAME, 0600, 0, &p4bus_vmchar_proc_ops);
#endif

	kobj_master = kobject_create_and_add(CLASS_NAME, NULL);
	if (IS_ERR(kobj_master)) {
		printk(KERN_ERR PREF "in module __init: failed to create a kobject '%s'\n", CLASS_NAME);
		return PTR_ERR(kobj_master);
	}

    /* register the vmchar driver */
	ret = platform_driver_register(&p4bus_vmchar_driver);
	if (ret != 0)
	{
		 printk(KERN_ERR PREF " %s Cannot register the vmm driver as platform driver \n", __func__);
		 return -ENODEV;
	}

	return ret;
}

/**
 *  @purpose
 *    module exit routine
 *
 *  @param 
 *
 *  @returns
 */
static void __exit p4bus_vmchar_exit(void)
{
	platform_driver_unregister(&p4bus_vmchar_driver);

	/* free driver resources */
	unregister_chrdev(major_dyn_allocated, DRIVER_NAME);
	
#ifdef CONFIG_PROC_FS
	remove_proc_entry(CLASS_NAME, 0);
#endif
	
    kobject_put(kobj_master);

	class_destroy(p4bus_vmchar_class);

	printk(KERN_ERR PREF "exit.\n");
}

module_init(p4bus_vmchar_init);
module_exit(p4bus_vmchar_exit);
