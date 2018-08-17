/*
 *  Linux Block device driver over P4bus (hardware virtualisation)
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
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/interrupt.h>

#include "compat.h"
#include "p4bus.h"

/* ------------------- CONSTANT / MACRO DEFINITIONS ------------------------ */

#define PREF                    "P4BUS_vmblock: "
#define DRIVER_NAME             "vmblock"
#define DRIVER_COMPAT           "p4bus,vmblock"

#define VMBLOCK_MAJOR           0           /* dynamically allocated */
#define VMBLOCK_MINOR_NBR       16          /* number of minor (partitions) */

#ifndef FALSE
#define FALSE                   0
#endif
#ifndef TRUE
#define TRUE                    1
#endif

/*
 * We can tweak our hardware sector size, but the kernel talks to us
 * in terms of small sectors (= 512).
 */
#define KERNEL_SECTOR_SIZE      512

//#define DEBUG
#ifdef DEBUG
#define DBG(args...) do { \
                        printk(KERN_ERR PREF "(%s:%d): ", __func__,__LINE__); \
                        printk(args); \
                        } while (0)
#else
#define DBG(args...) do {} while (0)
#endif

/* ------------------------ TYPE DECLARATIONS ------------------------------ */

typedef struct vmblock_dev_s {
    struct platform_device *pdev;       /* Platform device pointer */
    uint32_t major;                     /* Major number */
    p4bus_device_info_t *p4bus_dev;     /* P4BUS device pointer */
    uint32_t p4bus_nb_sector;           /* P4BUS number of sector */
    uint32_t p4bus_sector_size;         /* P4BUS sector size */
    uint32_t real_device_size;          /* Roundup (on sector size) shm size */
    struct gendisk *disk;               /* Disk pointer */
    int media_change;                   /* Flag media change */
    wait_queue_head_t wait_op;          /* Operation wait */
    spinlock_t lock;                    /* Lock for mutual exclusion */
    uint32_t ioring_id;                 /* IO ring id */
    p4bus_ioring_t ioring_op;           /* IO ring operation */
    uint32_t active_counter;            /* IO ring active counter */
    struct tasklet_struct tasklet;      /* Tasklet */
} vmblock_dev_t;

/* -------------------- LOCAL FUNCTION DECLARATIONS ------------------------ */

static int p4bus_vmblock_probe(struct platform_device *pdev);
static int p4bus_vmblock_remove(struct platform_device *pdev); 

static int p4bus_vmblock_open(struct block_device *bdev, fmode_t mode);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int p4bus_vmblock_release(struct gendisk *disk, fmode_t mode);
#else
static void p4bus_vmblock_release(struct gendisk *disk, fmode_t mode);
#endif
static int p4bus_vmblock_ioctl(struct block_device *bdev, fmode_t mode,
                                unsigned int cmd, unsigned long arg);
static int p4bus_vmblock_media_changed(struct gendisk *gd);
static int p4bus_vmblock_revalidate(struct gendisk *gd);
static int p4bus_vmblock_getgeo(struct block_device *bdev, struct hd_geometry *geo);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
static blk_qc_t vmblock_make_request(struct request_queue *q, struct bio *bio);
#else
static void vmblock_make_request(struct request_queue *q, struct bio *bio);
#endif
static inline int vmblock_xfer_bio(vmblock_dev_t *dev, struct bio *bio);
static irqreturn_t vmblock_end_operation_handler(int irq, void *priv);
static inline int p4bus_vmblock_do_op(vmblock_dev_t *dev, unsigned int op, 
    unsigned long param, void *buffer, unsigned long size, unsigned long flags);
static void p4bus_vmblock_tasklet(unsigned long data);

/* ----------------------- OBJECT DECLARATIONS ----------------------------- */

/**
 * Our match table for the hwvirt p4bus
 */
static const struct of_device_id p4bus_vmblock_match[] = {
    {.compatible = DRIVER_COMPAT, },
    {},
};
MODULE_DEVICE_TABLE(of, p4bus_vmblock_match);

static struct platform_driver p4bus_vmblock_driver = {
    .probe      = p4bus_vmblock_probe,
    .remove     = p4bus_vmblock_remove,
    .driver     = {
        .name           = DRIVER_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = p4bus_vmblock_match,
    },
};

/*
 * The device operations structure.
 */
static const struct block_device_operations vmblock_ops = {
    .open               = p4bus_vmblock_open,
    .release            = p4bus_vmblock_release,
    .ioctl              = p4bus_vmblock_ioctl,
    .getgeo             = p4bus_vmblock_getgeo,
    .media_changed      = p4bus_vmblock_media_changed,
    .revalidate_disk    = p4bus_vmblock_revalidate,
    .owner              = THIS_MODULE,
};

/* ------------------ GLOBAL FUNCTION DEFINITIONS -------------------------- */

module_platform_driver(p4bus_vmblock_driver);

MODULE_ALIAS("platform:vmblock");
MODULE_AUTHOR("Jerome Poncin <jer@sysgo.com>");
MODULE_DESCRIPTION("Block device driver over PikeOS Virtualization bus");
MODULE_LICENSE("GPL");

/* -------------------------- LOCAL FUNCTION DEFINITIONS ------------------- */

/*
 * irqreturn_t vmblock_end_operation_handler(int irq, void *priv)
 * 
 * Check end of vmblock operation and wake up driver
 */
static irqreturn_t vmblock_end_operation_handler(int irq, void *priv)
{
    vmblock_dev_t *dev;

    if (priv == NULL)
        return -EFAULT;

    dev = (vmblock_dev_t *)priv;

    spin_lock(&dev->lock);
    if (p4bus_ioring_has_done(&(dev->ioring_op)) == TRUE)
    {
        spin_unlock(&dev->lock);
        tasklet_schedule(&dev->tasklet);
    }
    else
    {
        spin_unlock(&dev->lock);
        if (waitqueue_active(&(dev->wait_op)) == TRUE)
            wake_up_interruptible(&(dev->wait_op));
    }

    return IRQ_HANDLED;
}

/*
 * void vmblock_make_request(struct request_queue *q, struct bio *bio)
 * 
 * Make vmblock request
 */
static inline int vmblock_xfer_bio(vmblock_dev_t *dev, struct bio *bio)
{
    unsigned long param, flags, size;
    char *buffer;
    int ret;
    unsigned int op;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
    sector_t sector = bio->bi_sector;
    struct bio_vec *bvec;
    int iter;
#else
    sector_t sector = bio->bi_iter.bi_sector;
    struct bio_vec bvec;
    struct bvec_iter iter;
#endif

    /* Do each segment independently */
    bio_for_each_segment(bvec, bio, iter)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
        buffer = page_address(bvec->bv_page);
#else
        buffer = page_address(bvec.bv_page);
#endif
        if (buffer == NULL)
            return -ENOMEM;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
        buffer += bvec->bv_offset;
#else
        buffer += bvec.bv_offset;
#endif

        param = sector * KERNEL_SECTOR_SIZE;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
        size = (unsigned long)bvec->bv_len;
        /* Test end of write block */
        if (iter == bvec->bv_len)
#else
        size = (unsigned long)bvec.bv_len;
        /* Test end of write block */
        if (iter.bi_size == bvec.bv_len)
#endif
        {
            flags = OP_FLAGS_SIGNAL;
        }
        else
        {
            flags = 0;
        }

        /* Lets do the request */
        if (bio_data_dir(bio) > 0)
            op = P4BUS_OP_WRITE;
        else
            op = P4BUS_OP_READ;

        ret = p4bus_vmblock_do_op(dev, op, param, buffer, size, flags);
        if (ret != 0)
            return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
        sector += (bvec->bv_len >> 9u);
#else
        sector += (bvec.bv_len >> 9u);
#endif
    }

    return 0;
}

/*
 * void vmblock_make_request(struct request_queue *q, struct bio *bio)
 * 
 * Make vmblock request
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
static blk_qc_t vmblock_make_request(struct request_queue *q, struct bio *bio)
#else
static void vmblock_make_request(struct request_queue *q, struct bio *bio)
#endif
{
    int ret;
    vmblock_dev_t *dev;
    unsigned long irq_flags, active_counter;

    if (q == NULL)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
        return BLK_QC_T_NONE;
#else
        return;
#endif

    dev = q->queuedata;

    if ((dev == NULL) || (bio == NULL))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
        return BLK_QC_T_NONE;
#else
        return;
#endif

    spin_lock_irqsave(&dev->lock, irq_flags);
    dev->active_counter++;
    spin_unlock_irqrestore(&dev->lock, irq_flags);

    ret = vmblock_xfer_bio(dev, bio);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,3,0)
    bio_endio(bio);
#else
    bio_endio(bio, ret);
#endif

    spin_lock_irqsave(&dev->lock, irq_flags);
    dev->active_counter--;
    active_counter = dev->active_counter;
    spin_unlock_irqrestore(&dev->lock, irq_flags);

    if (active_counter == 0)
    {
        if (waitqueue_active(&(dev->wait_op)) == TRUE)
            wake_up_interruptible(&(dev->wait_op));
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0)
    return BLK_QC_T_NONE;
#endif
}

/*
 * int p4bus_vmblock_do_op(vmblock_dev_t *dev, unsigned long param, void *buffer, 
 * unsigned long size, unsigned long flags)
 * 
 * vmblock operation
 */
static inline int p4bus_vmblock_do_op(vmblock_dev_t *dev, unsigned int op,
    unsigned long param, void *buffer, unsigned long size, unsigned long flags)
{
    int ret;
    p4bus_operation_t *curr_op;
    unsigned long irq_flags;
    unsigned int wait_flag;

    if (flags == OP_FLAGS_SIGNAL)
        wait_flag = TRUE;
    else
        wait_flag = FALSE;

    spin_lock_irqsave(&dev->lock, irq_flags);

    /* Get IO ring opereration */
    do
    {
        curr_op = p4bus_ioring_get_free(&(dev->ioring_op));
        if (curr_op == NULL)
        {
            spin_unlock_irqrestore(&dev->lock, irq_flags);

            /* Signal the host */
            p4bus_command_signal_ioring(dev->ioring_id);

            /* Wait IO ring done */
            ret = wait_event_interruptible(dev->wait_op,
                            (p4bus_ioring_has_free(&(dev->ioring_op)) == TRUE));
            if (ret != 0)
            {
                dev_err(&(dev->pdev->dev), PREF "%s Error during Wait done ioring (ret 0x%08X)\n", 
                        dev->p4bus_dev->name, ret);

                return -EACCES;
            }

            spin_lock_irqsave(&dev->lock, irq_flags);
        }
    } while (curr_op == NULL);

    if (p4bus_ioring_has_free(&dev->ioring_op) == FALSE)
        flags = OP_FLAGS_SIGNAL;

    /* Configure the operation */
    curr_op->retcode = P4BUS_E_OK;
    curr_op->param2 = 1; 
    curr_op->type = op;
    curr_op->devid = dev->p4bus_dev->p4bus_devid;
    curr_op->param1 = param;
    curr_op->addr = virt_to_phys(buffer);
    curr_op->priv_guest = (unsigned long)buffer;
    curr_op->size = size;
    curr_op->flags = flags;

    /* Push the operation */
    p4bus_ioring_push_ready(curr_op);

    spin_unlock_irqrestore(&dev->lock, irq_flags);

    DBG("OP %i 0x%lx, 0x%llx, %llu\n", op, param, curr_op->addr, curr_op->size);

    if (flags == OP_FLAGS_SIGNAL)
    {
        /* Signal the host */
        p4bus_command_signal_ioring(dev->ioring_id);

        /* Wait only at end of request */
        if (wait_flag == TRUE)
        {
            ret = wait_event_interruptible(dev->wait_op, 
                                    (curr_op->status != P4BUS_OPERATION_READY));
            if ((curr_op->retcode != P4BUS_E_OK) || (ret != 0)) 
            {
                dev_err(&(dev->pdev->dev), PREF "%s Error during Wait operation (ret 0x%08X, err 0x%08X)\n", 
                        dev->p4bus_dev->name, ret, curr_op->retcode);

                return -EACCES;
            }
        }
    }

    return ret;
}

/*
 * void p4bus_vmblock_tasklet(unsigned long data)
 * 
 * vmblock tasklet (free operations done) 
 */
static void p4bus_vmblock_tasklet(unsigned long data)
{
    p4bus_operation_t *curr_op;
    unsigned long irq_flags;
    vmblock_dev_t *dev = (vmblock_dev_t *)data;

    /* For all 'done' operations */
    do
    {
        /* Check IO ring */
        spin_lock_irqsave(&dev->lock, irq_flags);
        curr_op = p4bus_ioring_get_done(&(dev->ioring_op));
        spin_unlock_irqrestore(&dev->lock, irq_flags);

        if (curr_op != NULL)
        {
            if (curr_op->retcode != P4BUS_E_OK)
            {
                dev_err(&(dev->pdev->dev), PREF "%s Error during operation (ret 0x%08X)\n", 
                        dev->p4bus_dev->name, curr_op->retcode);
            }

            /* Operation is free */
            p4bus_ioring_push_free(curr_op);
        }
    } while (curr_op != NULL);

    if (waitqueue_active(&(dev->wait_op)) == TRUE)
        wake_up_interruptible(&(dev->wait_op));
}

/*
 * int p4bus_vmblock_open(struct block_device *bdev, fmode_t mode)
 * 
 * Open vmblock operation
 */
static int p4bus_vmblock_open(struct block_device *bdev, fmode_t mode)
{
    int ret = 0;
    unsigned long fs_flags = P4BUS_FLAGS_OPEN_READ;
    vmblock_dev_t *dev;
    p4bus_operation_t op;

    DBG("OPEN\n");

    if (bdev == NULL)
        return -EFAULT;

    if (bdev->bd_disk == NULL)
        return -EFAULT;

    dev = (vmblock_dev_t *)bdev->bd_disk->private_data;

    if (dev == NULL)
        return -EFAULT;

    if (mode & FMODE_WRITE)
    {
        DBG("MODE WRITE\n");
        fs_flags |= P4BUS_FLAGS_OPEN_WRITE;
    }

    if (mode & FMODE_EXEC)
    {
        DBG("MODE EXEC\n");
        fs_flags |= P4BUS_FLAGS_OPEN_EXEC;
    }

    op.retcode = P4BUS_E_OK;
    op.param2 = 0; 
    op.type = P4BUS_OP_OPEN;
    op.flags = OP_FLAGS_SIGNAL;
    op.filedesc = 0;
    op.devid = dev->p4bus_dev->p4bus_devid;
    op.addr = 0;
    op.size = 0;  
    op.param1 = fs_flags;
    op.priv_guest = 0;

    ret = p4bus_command_execute_operation(&op, NULL);
    if (ret != P4BUS_E_OK)
    {
        dev_err(&(dev->pdev->dev), PREF "%s Error during OPEN operation (ret 0x%08X, err 0x%08X)\n", 
                dev->p4bus_dev->name, ret, op.retcode);
    }
    else
    {
        ret = wait_event_interruptible(dev->wait_op, 
                                       (op.status == P4BUS_OPERATION_DONE));
        if ((op.retcode != P4BUS_E_OK) || (ret != 0)) 
        {
            dev_err(&(dev->pdev->dev), PREF "%s Error during Wait OPEN operation (ret 0x%08X, err 0x%08X)\n", 
                    dev->p4bus_dev->name, ret, op.retcode);

            return -EACCES;
        }

        tasklet_enable(&dev->tasklet);
    }

    return ret;
}

/*
 * void p4bus_vmblock_release(struct gendisk *disk, fmode_t mode)
 * 
 * Close vmblock operation
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int p4bus_vmblock_release(struct gendisk *disk, fmode_t mode)
#else
static void p4bus_vmblock_release(struct gendisk *disk, fmode_t mode)
#endif
{
    int ret;
    vmblock_dev_t *dev;
    p4bus_operation_t op;
    uint32_t active_counter;
    unsigned long irq_flags;

    DBG("CLOSE\n");

    if (disk == NULL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
        return -EINVAL;
#else
        return;
#endif

    dev = (vmblock_dev_t *)disk->private_data;

    if (dev == NULL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
        return  -EINVAL;
#else
        return;
#endif

    op.retcode = P4BUS_E_OK;
    op.param2 = 0; 
    op.type = P4BUS_OP_CLOSE;
    op.flags = OP_FLAGS_SIGNAL;
    op.filedesc = 0;
    op.devid = dev->p4bus_dev->p4bus_devid;
    op.addr = 0;
    op.size = 0;  
    op.param1 = 0;
    op.priv_guest = 0;

    spin_lock_irqsave(&dev->lock, irq_flags);

    /* Wait end of all operations */
    do
    {
        active_counter = dev->active_counter;
        if (active_counter != 0)
        {
            spin_unlock_irqrestore(&dev->lock, irq_flags);

            /* Wait end of IO ring */
            ret = wait_event_interruptible(dev->wait_op, 
                                            (dev->active_counter == 0));
            if (ret != 0)
            {
                dev_err(&(dev->pdev->dev), PREF "%s Error during Wait end of operation (ret 0x%08X)\n", 
                        dev->p4bus_dev->name, ret);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
                return -EACCES;
#else
                return;
#endif
            }

            spin_lock_irqsave(&dev->lock, irq_flags);
        }
    } while (active_counter != 0);
    
    spin_unlock_irqrestore(&dev->lock, irq_flags);

    ret = p4bus_command_execute_operation(&op, NULL);
    if (ret != P4BUS_E_OK)
    {
        dev_err(&(dev->pdev->dev), PREF "%s Error during CLOSE operation (ret 0x%08X, err 0x%08X)\n", 
                dev->p4bus_dev->name, ret, op.retcode);
    }
    else
    {
        ret = wait_event_interruptible(dev->wait_op, 
                                       (op.status == P4BUS_OPERATION_DONE));
        if ((op.retcode != P4BUS_E_OK) || (ret != 0)) 
        {
            dev_err(&(dev->pdev->dev), PREF "%s Error during Wait CLOSE operation (ret 0x%08X, err 0x%08X)\n", 
                    dev->p4bus_dev->name, ret, op.retcode);
        }

        tasklet_disable(&dev->tasklet);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    return ret;
#else

#endif
}

/*
 * int p4bus_vmblock_ioctl(struct block_device *bdev, fmode_t mode,
 *                                unsigned int cmd, unsigned long arg)
 * 
 * IOCTL vmblock operation (get geometry)
 */
static int p4bus_vmblock_ioctl(struct block_device *bdev, fmode_t mode,
                                unsigned int cmd, unsigned long arg)
{
    struct hd_geometry geo;

    if (bdev == NULL)
        return -EFAULT;

    switch(cmd)
    {
        case HDIO_GETGEO:
            if (p4bus_vmblock_getgeo(bdev, &geo))
                return -EFAULT;

            if (copy_to_user((void __user *)arg, &geo, sizeof(geo)))
                return -EFAULT;

            return 0;

        default:
            DBG("IOCTL cmd 0x%X unmanaged\n", cmd);
            break;
    }

    return -ENOTTY; /* unknown command */
}

/*
 * int p4bus_vmblock_getgeo(struct block_device *bdev, struct hd_geometry *geo)
 * 
 * Get geometry vmblock
 */
static int p4bus_vmblock_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
    vmblock_dev_t *dev;

    if ((bdev == NULL) || (geo == NULL))
        return -EFAULT;

    if (bdev->bd_disk == NULL)
        return -EFAULT;

    dev = (vmblock_dev_t *)bdev->bd_disk->private_data;

    if (dev == NULL)
        return -EFAULT;

    /*
     * Get geometry: since we are a virtual device, we have to make
     * up something plausible. So we claim 16 sectors, four heads,
     * and calculate the corresponding number of cylinders. We set the
     * start of data at sector four.
     */
    geo->cylinders = (dev->real_device_size & (~0x3f)) >> 6;
    geo->heads = 4;
    geo->sectors = 16;
    geo->start = 4;

    return 0;
}

/*
 * int p4bus_vmblock_media_changed(struct gendisk *gd)
 * 
 * Media changed
 */
static int p4bus_vmblock_media_changed(struct gendisk *gd)
{
    vmblock_dev_t *dev;

    if (gd == NULL)
        return 0;

    dev = (vmblock_dev_t *)gd->private_data;

    if (dev == NULL)
        return 0;

    return dev->media_change;
}

/*
 * int p4bus_vmblock_revalidate(struct gendisk *gd)
 * 
 * Revalidate media
 */
static int p4bus_vmblock_revalidate(struct gendisk *gd)
{
    vmblock_dev_t *dev;

    if (gd == NULL)
        return 0;

    dev = (vmblock_dev_t *)gd->private_data;

    if (dev == NULL)
        return 0;

    dev->media_change = 0;

    return 0;
}

/*
 * int p4bus_vmblock_probe(struct platform_device *pdev)
 * 
 * vmblock probe
 */
static int p4bus_vmblock_probe(struct platform_device *pdev)
{
    int ret;
    struct gendisk *disk;
    struct request_queue *queue;
    vmblock_dev_t *dev;
    void *private_data;
    unsigned int j;

    if (pdev == NULL)
        return -EFAULT;

    printk(KERN_ERR PREF "probing %s\n", pdev->name);

    /* Check the P4BUS status */
    if(!p4bus_detected())
    {
        dev_err(&pdev->dev, PREF "no p4bus detected.\n");
        return -ENODEV;
    }

    private_data = devm_kzalloc(&pdev->dev, sizeof(vmblock_dev_t), GFP_KERNEL);
    if (private_data == NULL)
    {
        dev_err(&pdev->dev, PREF "could not allocate memory\n");
        return -ENOMEM;
    }

    dev = (vmblock_dev_t *)private_data;

    dev->p4bus_dev = devm_kzalloc(&pdev->dev, sizeof(p4bus_device_info_t), GFP_KERNEL);
    if (dev->p4bus_dev == NULL)
    {
        devm_kfree(&pdev->dev, dev);
        dev_err(&pdev->dev, PREF "Cannot allocate memory for the device info\n");
        return -ENOMEM;
    }

    dev->pdev = pdev;

    /* Register device on p4bus */
    ret = p4bus_get_devinfo(pdev, dev->p4bus_dev);
    if (ret < 0)
    {
        dev_err(&pdev->dev, PREF "could not register p4bus device\n");
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EFAULT;
    }

    /* Check if access flags required are allowed */
    if ((((dev->p4bus_dev->opmask & (1 << P4BUS_OP_WRITE)) == 0)
    &&  ((dev->p4bus_dev->opmask & (1 << P4BUS_OP_NB_WRITE)) == 0)))
    {
        dev_err(&pdev->dev, PREF "%s Error no WRITE permissions set in the host driver \n",__func__);
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EINVAL;
    }

    /* Check if access flags required are allowed */
    if ((((dev->p4bus_dev->opmask & (1 << P4BUS_OP_READ)) == 0)
    &&  ((dev->p4bus_dev->opmask & (1 << P4BUS_OP_NB_READ)) == 0)))
    {
        dev_err(&pdev->dev, PREF "%s Error no READ permissions set in the host driver \n",__func__);
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EINVAL;
    }

    if (dev->p4bus_dev->size <= 0)
    {
        dev_err(&pdev->dev, PREF "Device with 0 size cannot be used\n");
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EINVAL;
    }

    /* Set the block size depending of the Host Side device */
    dev->p4bus_sector_size = PAGE_SIZE;
    dev->p4bus_nb_sector = dev->p4bus_dev->size / dev->p4bus_sector_size;
    dev->real_device_size = dev->p4bus_sector_size * dev->p4bus_nb_sector;
    if ((dev->p4bus_dev->size % dev->p4bus_sector_size) != 0)
    {
        dev_err(&pdev->dev, PREF "SHM is not aligned on sector_size (%u)\n",
                dev->p4bus_sector_size);
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EINVAL;
    }

    ret = register_blkdev(VMBLOCK_MAJOR, dev->p4bus_dev->name);
    if (ret <= 0)
    {
        dev_err(&pdev->dev, PREF "register_blkdev failed. Error : %i\n", ret);
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return ret;
    }

    dev->major = ret;

    p4bus_ioring_init(&(dev->ioring_op));

    if (p4bus_command_create_ioring(&(dev->ioring_op), &(dev->ioring_id)))
    {
        dev_err(&pdev->dev, PREF " cannot create ioring\n");
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -ENOMEM;
    }
    else
    {
        for (j = 0; j < P4BUS_IORING_DEPTH; j++)
        {
            memset(&dev->ioring_op.operations[j], 0, sizeof(dev->ioring_op.operations[j]));
        }
    }

    /* Initialize spinlock */
    spin_lock_init(&dev->lock);

    /* Initialize wait queue */
    init_waitqueue_head(&(dev->wait_op));

    /* Initialize tasklet */
    tasklet_init(&dev->tasklet, p4bus_vmblock_tasklet, (unsigned long)dev);
    tasklet_disable(&dev->tasklet);

    dev->active_counter = 0;

    /* Request the end of operation IRQ */
    if (devm_request_irq(&pdev->dev, dev->p4bus_dev->interrupt, vmblock_end_operation_handler,
                         IRQF_SHARED, dev_name(&pdev->dev), dev) != 0)
    {
        dev_err(&pdev->dev, PREF "cannot attach to the interrupt\n");
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -EIO;
    }

    queue = blk_alloc_queue(GFP_KERNEL);
    if (queue == NULL)
        return -ENOMEM;

    blk_queue_make_request(queue, vmblock_make_request);
    blk_queue_max_segment_size(queue, dev->p4bus_sector_size);
    blk_queue_logical_block_size(queue, dev->p4bus_sector_size);
    blk_queue_io_min(queue, PAGE_SIZE);
    blk_queue_io_opt(queue, (PAGE_SIZE * P4BUS_IORING_DEPTH));
    blk_queue_bounce_limit(queue, BLK_BOUNCE_ANY);

    queue_flag_set_unlocked(QUEUE_FLAG_VIRT, queue);
    queue_flag_clear_unlocked(QUEUE_FLAG_ADD_RANDOM, queue);
    queue_flag_clear_unlocked(QUEUE_FLAG_DISCARD, queue);

    queue->queuedata = dev;

    disk = alloc_disk(VMBLOCK_MINOR_NBR);
    if (disk == NULL)
    {
        dev_err(&pdev->dev, PREF "cannot allocate disk\n");
        devm_kfree(&pdev->dev, dev->p4bus_dev);
        devm_kfree(&pdev->dev, dev);
        return -ENOMEM;
    }

    dev->disk = disk;

    disk->major = dev->major;
    disk->first_minor = dev->p4bus_dev->p4bus_devid;
    disk->fops = &vmblock_ops;
    disk->queue = queue;
    disk->private_data = private_data;
    snprintf(disk->disk_name, sizeof(disk->disk_name), dev->p4bus_dev->name);

    /* Set the capacity in term of KERNEL_SECTOR_SIZE */
    set_capacity(disk, (dev->real_device_size / KERNEL_SECTOR_SIZE));

    add_disk(disk);

    platform_set_drvdata(pdev, dev);

    return 0;
}

/*
 * int p4bus_vmblock_remove(struct platform_device *pdev)
 * 
 * vmblock remove
 */
static int p4bus_vmblock_remove(struct platform_device *pdev)
{
    struct gendisk *disk;
    struct request_queue *queue;
    vmblock_dev_t *dev;

    if (pdev == NULL)
        return -EFAULT;

    dev = (vmblock_dev_t *)platform_get_drvdata(pdev);
    if (dev == NULL)
    {
        dev_err(&pdev->dev, PREF "cannot remove device on p4bus\n");
        return -EFAULT;
    }

    disk = dev->disk;   
    queue = disk->queue;

    del_gendisk(disk);
    blk_cleanup_queue(queue);
    put_disk(disk);

    unregister_blkdev(dev->major, dev->p4bus_dev->name);

    /* Destroy iorings */
    p4bus_command_destroy_ioring(dev->ioring_id);

    /* Free the irq */
    devm_free_irq(&pdev->dev, dev->p4bus_dev->interrupt, dev);

    tasklet_kill(&dev->tasklet);

    /* Free memory */
    devm_kfree(&pdev->dev, dev->p4bus_dev);
    devm_kfree(&pdev->dev, dev);

    put_device(&pdev->dev);

    printk(KERN_ERR PREF "%s removed.\n", pdev->name);

    return 0;
}
