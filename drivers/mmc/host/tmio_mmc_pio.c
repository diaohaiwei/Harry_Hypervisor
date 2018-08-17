/*
 * linux/drivers/mmc/host/tmio_mmc_pio.c
 *
 * Copyright (C) 2016 Sang Engineering, Wolfram Sang
 * Copyright (C) 2015-2017 Renesas Electronics Corporation
 * Copyright (C) 2011 Guennadi Liakhovetski
 * Copyright (C) 2007 Ian Molton
 * Copyright (C) 2004 Ian Molton
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for the MMC / SD / SDIO IP found in:
 *
 * TC6393XB, TC6391XB, TC6387XB, T7L66XB, ASIC3, SH-Mobile SoCs
 *
 * This driver draws mainly on scattered spec sheets, Reverse engineering
 * of the toshiba e800  SD driver and some parts of the 2.4 ASIC3 driver (4 bit
 * support). (Further 4 bit support from a later datasheet).
 *
 * TODO:
 *   Investigate using a workqueue for PIO transfers
 *   Eliminate FIXMEs
 *   SDIO support
 *   Better Power management
 *   Handle MMC errors better
 *   double buffer support
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mfd/tmio.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "tmio_mmc.h"

void tmio_mmc_enable_mmc_irqs(struct tmio_mmc_host *host, u32 i)
{
	host->sdcard_irq_mask &= ~(i & TMIO_MASK_IRQ);
	sd_ctrl_write32_as_16_and_16(host, CTL_IRQ_MASK, host->sdcard_irq_mask);
}

void tmio_mmc_disable_mmc_irqs(struct tmio_mmc_host *host, u32 i)
{
	host->sdcard_irq_mask |= (i & TMIO_MASK_IRQ);
	sd_ctrl_write32_as_16_and_16(host, CTL_IRQ_MASK, host->sdcard_irq_mask);
}

void tmio_set_transtate(struct tmio_mmc_host *host, unsigned int state)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&host->trans_lock, flags);
	host->trans_state |= state;

	if (host->trans_state == (TMIO_TRANSTATE_DEND | TMIO_TRANSTATE_AEND)) {
		spin_unlock_irqrestore(&host->trans_lock, flags);
		tasklet_schedule(&host->dma_complete);
	} else {
		spin_unlock_irqrestore(&host->trans_lock, flags);
	}
}

void tmio_clear_transtate(struct tmio_mmc_host *host)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&host->trans_lock, flags);

	host->trans_state = 0;

	spin_unlock_irqrestore(&host->trans_lock, flags);
}

static void tmio_mmc_ack_mmc_irqs(struct tmio_mmc_host *host, u32 i)
{
	sd_ctrl_write32_as_16_and_16(host, CTL_STATUS, ~i);
}

static void tmio_mmc_init_sg(struct tmio_mmc_host *host, struct mmc_data *data)
{
	host->sg_len = data->sg_len;
	host->sg_ptr = data->sg;
	host->sg_orig = data->sg;
	host->sg_off = 0;
}

static int tmio_mmc_next_sg(struct tmio_mmc_host *host)
{
	host->sg_ptr = sg_next(host->sg_ptr);
	host->sg_off = 0;
	return --host->sg_len;
}

#define CMDREQ_TIMEOUT	5000

#ifdef CONFIG_MMC_DEBUG

#define STATUS_TO_TEXT(a, status, i) \
	do { \
		if (status & TMIO_STAT_##a) { \
			if (i++) \
				printk(" | "); \
			printk(#a); \
		} \
	} while (0)

static void pr_debug_status(u32 status)
{
	int i = 0;
	pr_debug("status: %08x = ", status);
	STATUS_TO_TEXT(CARD_REMOVE, status, i);
	STATUS_TO_TEXT(CARD_INSERT, status, i);
	STATUS_TO_TEXT(SIGSTATE, status, i);
	STATUS_TO_TEXT(WRPROTECT, status, i);
	STATUS_TO_TEXT(CARD_REMOVE_A, status, i);
	STATUS_TO_TEXT(CARD_INSERT_A, status, i);
	STATUS_TO_TEXT(SIGSTATE_A, status, i);
	STATUS_TO_TEXT(CMD_IDX_ERR, status, i);
	STATUS_TO_TEXT(STOPBIT_ERR, status, i);
	STATUS_TO_TEXT(ILL_FUNC, status, i);
	STATUS_TO_TEXT(CMD_BUSY, status, i);
	STATUS_TO_TEXT(CMDRESPEND, status, i);
	STATUS_TO_TEXT(DATAEND, status, i);
	STATUS_TO_TEXT(CRCFAIL, status, i);
	STATUS_TO_TEXT(DATATIMEOUT, status, i);
	STATUS_TO_TEXT(CMDTIMEOUT, status, i);
	STATUS_TO_TEXT(RXOVERFLOW, status, i);
	STATUS_TO_TEXT(TXUNDERRUN, status, i);
	STATUS_TO_TEXT(RXRDY, status, i);
	STATUS_TO_TEXT(TXRQ, status, i);
	STATUS_TO_TEXT(ILL_ACCESS, status, i);
	printk("\n");
}

#else
#define pr_debug_status(s)  do { } while (0)
#endif

static void tmio_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	unsigned int timeout = 1000;
	unsigned long flags = 0;

	if (host->sequencer_enabled)
		spin_lock_irqsave(&host->lock, flags);
	else
		__acquire(&host->lock);

	if (enable && !host->sdio_irq_enabled) {
		u16 sdio_status;

		/* Keep device active while SDIO irq is enabled */
		pm_runtime_get_sync(mmc_dev(mmc));

		host->sdio_irq_enabled = true;
		host->sdio_irq_mask = TMIO_SDIO_MASK_ALL &
					~TMIO_SDIO_STAT_IOIRQ;

		/* Clear obsolete interrupts before enabling */
		sdio_status = sd_ctrl_read16(host, CTL_SDIO_STATUS) & ~TMIO_SDIO_MASK_ALL;
		if (host->pdata->flags & TMIO_MMC_SDIO_STATUS_SETBITS)
			sdio_status |= TMIO_SDIO_SETBITS_MASK;
		sd_ctrl_write16(host, CTL_SDIO_STATUS, sdio_status);

		if (host->sequencer_enabled) {
			while (timeout) {
				sd_ctrl_write16(host, CTL_SDIO_IRQ_MASK,
						host->sdio_irq_mask);
				if (sd_ctrl_read16(host, CTL_SDIO_IRQ_MASK) ==
				    host->sdio_irq_mask)
					break;

				udelay(1);
				timeout--;
			}
			if (!timeout)
				dev_warn(&host->pdev->dev,
					 "failed to write CTL_SDIO_IRQ_MASK\n");
		} else {
			sd_ctrl_write16(host, CTL_SDIO_IRQ_MASK,
					host->sdio_irq_mask);
		}
	} else if (!enable && host->sdio_irq_enabled) {
		host->sdio_irq_mask = TMIO_SDIO_MASK_ALL;
		if (host->sequencer_enabled) {
			while (timeout) {
				sd_ctrl_write16(host, CTL_SDIO_IRQ_MASK,
						host->sdio_irq_mask);
				if (sd_ctrl_read16(host, CTL_SDIO_IRQ_MASK) ==
				    host->sdio_irq_mask)
					break;

				udelay(1);
				timeout--;
			}
			if (!timeout)
				dev_warn(&host->pdev->dev,
					 "failed to write CTL_SDIO_IRQ_MASK\n");
		} else {
			sd_ctrl_write16(host, CTL_SDIO_IRQ_MASK,
					host->sdio_irq_mask);
		}
		host->sdio_irq_enabled = false;
		pm_runtime_mark_last_busy(mmc_dev(mmc));
		pm_runtime_put_autosuspend(mmc_dev(mmc));
	}

	if (host->sequencer_enabled)
		spin_unlock_irqrestore(&host->lock, flags);
	else
		__release(&host->lock);
}

static void tmio_mmc_clk_start(struct tmio_mmc_host *host)
{
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, CLK_CTL_SCLKEN |
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));
	if (!(host->pdata->flags & TMIO_MMC_MIN_RCAR2))
		usleep_range(10000, 11000);

	if (host->pdata->flags & TMIO_MMC_HAVE_HIGH_REG) {
		sd_ctrl_write16(host, CTL_CLK_AND_WAIT_CTL, 0x0100);
		msleep(10);
	}
}

static void tmio_mmc_clk_stop(struct tmio_mmc_host *host)
{
	if (host->pdata->flags & TMIO_MMC_HAVE_HIGH_REG) {
		sd_ctrl_write16(host, CTL_CLK_AND_WAIT_CTL, 0x0000);
		msleep(10);
	}

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~CLK_CTL_SCLKEN &
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));
	if (!(host->pdata->flags & TMIO_MMC_MIN_RCAR2))
		usleep_range(10000, 11000);
}

static void tmio_mmc_set_clock(struct tmio_mmc_host *host,
				unsigned int new_clock)
{
	u32 clk = 0, clock;

	if (new_clock == 0) {
		tmio_mmc_clk_stop(host);
		return;
	}
	/*
	 * Both HS400 and HS200/SD104 set 200MHz, but some devices need to
	 * set 400MHz to distinguish the CPG settings in HS400.
	 */
	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400 &&
	    !host->hs400_use_8tap && new_clock == 200000000)
		new_clock = 400000000;

	if (host->clk_update)
		clock = host->clk_update(host, new_clock) / 512;
	else
		clock = host->mmc->f_min;

	for (clk = 0x80000080; new_clock >= (clock << 1); clk >>= 1)
		clock <<= 1;

	/* 1/1 clock is option */
	if ((host->pdata->flags & TMIO_MMC_CLK_ACTUAL) &&
	    ((clk >> 22) & 0x1)) {
		if (!(host->mmc->ios.timing == MMC_TIMING_MMC_HS400))
			clk |= 0xff;
		else
			clk &= ~0xff;
	}

	if (host->set_clk_div)
		host->set_clk_div(host->pdev, (clk >> 22) & 1);

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~CLK_CTL_SCLKEN &
			sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, clk & CLK_CTL_DIV_MASK);
	if (!(host->pdata->flags & TMIO_MMC_MIN_RCAR2))
		msleep(10);

	tmio_mmc_clk_start(host);
}

static void tmio_mmc_reset(struct tmio_mmc_host *host)
{
	/* FIXME - should we set stop clock reg here */
	sd_ctrl_write16(host, CTL_RESET_SD, 0x0000);
	if (host->pdata->flags & TMIO_MMC_HAVE_HIGH_REG)
		sd_ctrl_write16(host, CTL_RESET_SDIO, 0x0000);

	usleep_range(10000, 11000);

	sd_ctrl_write16(host, CTL_RESET_SD, 0x0001);
	if (host->pdata->flags & TMIO_MMC_HAVE_HIGH_REG)
		sd_ctrl_write16(host, CTL_RESET_SDIO, 0x0001);

	usleep_range(10000, 11000);

	if (host->pdata->flags & TMIO_MMC_SDIO_IRQ) {
		sd_ctrl_write16(host, CTL_SDIO_IRQ_MASK, host->sdio_irq_mask);
		sd_ctrl_write16(host, CTL_TRANSACTION_CTL, 0x0001);
	}

}

static void tmio_mmc_reset_work(struct work_struct *work)
{
	struct tmio_mmc_host *host = container_of(work, struct tmio_mmc_host,
						  delayed_reset_work.work);
	struct mmc_request *mrq;
	unsigned long flags;
	u16 clk;

	spin_lock_irqsave(&host->lock, flags);
	mrq = host->mrq;

	/*
	 * is request already finished? Since we use a non-blocking
	 * cancel_delayed_work(), it can happen, that a .set_ios() call preempts
	 * us, so, have to check for IS_ERR(host->mrq)
	 */
	if (IS_ERR_OR_NULL(mrq)
	    || time_is_after_jiffies(host->last_req_ts +
		msecs_to_jiffies(CMDREQ_TIMEOUT))) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	dev_warn(&host->pdev->dev,
		"timeout waiting for hardware interrupt (CMD%u)\n",
		mrq->cmd->opcode);

	if (host->data)
		host->data->error = -ETIMEDOUT;
	else if (host->cmd)
		host->cmd->error = -ETIMEDOUT;
	else
		mrq->cmd->error = -ETIMEDOUT;

	host->cmd = NULL;
	host->data = NULL;
	host->force_pio = false;

	spin_unlock_irqrestore(&host->lock, flags);

	clk = sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL) & 0x0ff;
	tmio_mmc_reset(host);
	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, clk | 0x100);

	/* Ready for new calls */
	host->mrq = NULL;

	tmio_mmc_abort_dma(host);
	mmc_request_done(host->mmc, mrq);
}

/* called with host->lock held, interrupts disabled */
static void tmio_mmc_finish_request(struct tmio_mmc_host *host)
{
	struct mmc_request *mrq;
	unsigned long flags;
	struct mmc_command *cmd = host->cmd;

	spin_lock_irqsave(&host->lock, flags);

	mrq = host->mrq;
	if (IS_ERR_OR_NULL(mrq)) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	host->cmd = NULL;
	host->data = NULL;
	host->force_pio = false;

	cancel_delayed_work(&host->delayed_reset_work);

	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (mrq->cmd->error || (mrq->data && mrq->data->error)) {
		/* clear the interrupt flag register */
		sd_ctrl_write32_as_16_and_16(host, CTL_STATUS, 0);
		tmio_mmc_abort_dma(host);
	}

	if (host->check_scc_error && host->check_scc_error(host))
		mrq->cmd->error = -EILSEQ;

	if (cmd == mrq->sbc) {
		/* finish SET_BLOCK_COUNT request */
		complete(&host->completion);
		return;
	}

	mmc_request_done(host->mmc, mrq);
}

static void tmio_mmc_done_work(struct work_struct *work)
{
	struct tmio_mmc_host *host = container_of(work, struct tmio_mmc_host,
						  done);
	tmio_mmc_finish_request(host);
}

/* These are the bitmasks the tmio chip requires to implement the MMC response
 * types. Note that R1 and R6 are the same in this scheme. */
#define APP_CMD        0x0040
#define RESP_NONE      0x0300
#define RESP_R1        0x0400
#define RESP_R1B       0x0500
#define RESP_R2        0x0600
#define RESP_R3        0x0700
#define DATA_PRESENT   0x0800
#define TRANSFER_READ  0x1000
#define TRANSFER_MULTI 0x2000
#define SECURITY_CMD   0x4000
#define NO_CMD12_ISSUE 0x4000 /* TMIO_MMC_HAVE_CMD12_CTRL */

static int tmio_mmc_start_command(struct tmio_mmc_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = host->data;
	int c = cmd->opcode;
	u32 irq_mask = TMIO_MASK_CMD;

	/* CMD12 is handled by hardware */
	if (!(host->pdata->flags & TMIO_MMC_MIN_RCAR2)) {
		if (cmd->opcode == MMC_STOP_TRANSMISSION && !cmd->arg) {
			sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x001);
			return 0;
		}
	} else if (cmd->opcode == MMC_STOP_TRANSMISSION && !cmd->arg) {
		u32 status = sd_ctrl_read16_and_16_as_32(host, CTL_STATUS);
		bool busy = false;

		if ((host->pdata->flags &
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) ==
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) {
			if (!(status & TMIO_STAT_SCLKDIVEN))
				busy = true;
		} else {
			if (status & TMIO_STAT_CMD_BUSY)
				busy = true;
		}
		if (busy) {
			sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x001);
			return 0;
		}
	}

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE: c |= RESP_NONE; break;
	case MMC_RSP_R1:
	case MMC_RSP_R1_NO_CRC:
			   c |= RESP_R1;   break;
	case MMC_RSP_R1B:  c |= RESP_R1B;  break;
	case MMC_RSP_R2:   c |= RESP_R2;   break;
	case MMC_RSP_R3:   c |= RESP_R3;   break;
	default:
		pr_debug("Unknown response type %d\n", mmc_resp_type(cmd));
		return -EINVAL;
	}

	host->cmd = cmd;

/* FIXME - this seems to be ok commented out but the spec suggest this bit
 *         should be set when issuing app commands.
 *	if(cmd->flags & MMC_FLAG_ACMD)
 *		c |= APP_CMD;
 */
	if (data) {
		c |= DATA_PRESENT;
		if (data->blocks > 1) {
			sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x100);
			c |= TRANSFER_MULTI;

			/*
			 * Disable auto CMD12 at IO_RW_EXTENDED when
			 * multiple block transfer
			 */
			if ((host->pdata->flags & TMIO_MMC_HAVE_CMD12_CTRL) &&
			    ((cmd->opcode == SD_IO_RW_EXTENDED) ||
			     host->mrq->sbc))
				c |= NO_CMD12_ISSUE;
		}
		if (data->flags & MMC_DATA_READ)
			c |= TRANSFER_READ;
	}

	if (!host->native_hotplug)
		irq_mask &= ~(TMIO_STAT_CARD_REMOVE | TMIO_STAT_CARD_INSERT);
	tmio_mmc_enable_mmc_irqs(host, irq_mask);

	/* Fire off the command */
	sd_ctrl_write32_as_16_and_16(host, CTL_ARG_REG, cmd->arg);
	sd_ctrl_write16(host, CTL_SD_CMD, c);

	return 0;
}

static void tmio_mmc_transfer_data(struct tmio_mmc_host *host,
				   unsigned short *buf,
				   unsigned int count)
{
	int is_read = host->data->flags & MMC_DATA_READ;
	u8  *buf8;

	/*
	 * Transfer the data
	 */
	if (is_read)
		sd_ctrl_read16_rep(host, CTL_SD_DATA_PORT, buf, count >> 1);
	else
		sd_ctrl_write16_rep(host, CTL_SD_DATA_PORT, buf, count >> 1);

	/* if count was even number */
	if (!(count & 0x1))
		return;

	/* if count was odd number */
	buf8 = (u8 *)(buf + (count >> 1));

	/*
	 * FIXME
	 *
	 * driver and this function are assuming that
	 * it is used as little endian
	 */
	if (is_read)
		*buf8 = sd_ctrl_read16(host, CTL_SD_DATA_PORT) & 0xff;
	else
		sd_ctrl_write16(host, CTL_SD_DATA_PORT, *buf8);
}

/*
 * This chip always returns (at least?) as much data as you ask for.
 * I'm unsure what happens if you ask for less than a block. This should be
 * looked into to ensure that a funny length read doesn't hose the controller.
 */
static void tmio_mmc_pio_irq(struct tmio_mmc_host *host)
{
	struct mmc_data *data = host->data;
	void *sg_virt;
	unsigned short *buf;
	unsigned int count;
	unsigned long flags;

	if ((host->chan_tx || host->chan_rx) && !host->force_pio) {
		pr_err("PIO IRQ in DMA mode!\n");
		return;
	} else if (!data) {
		pr_debug("Spurious PIO IRQ\n");
		return;
	}

	sg_virt = tmio_mmc_kmap_atomic(host->sg_ptr, &flags);
	buf = (unsigned short *)(sg_virt + host->sg_off);

	count = host->sg_ptr->length - host->sg_off;
	if (count > data->blksz)
		count = data->blksz;

	pr_debug("count: %08x offset: %08x flags %08x\n",
		 count, host->sg_off, data->flags);

	/* Transfer the data */
	tmio_mmc_transfer_data(host, buf, count);

	host->sg_off += count;

	tmio_mmc_kunmap_atomic(host->sg_ptr, &flags, sg_virt);

	if (host->sg_off == host->sg_ptr->length)
		tmio_mmc_next_sg(host);

	return;
}

static void tmio_mmc_check_bounce_buffer(struct tmio_mmc_host *host)
{
	if (host->sg_ptr == &host->bounce_sg) {
		unsigned long flags;
		void *sg_vaddr = tmio_mmc_kmap_atomic(host->sg_orig, &flags);
		memcpy(sg_vaddr, host->bounce_buf, host->bounce_sg.length);
		tmio_mmc_kunmap_atomic(host->sg_orig, &flags, sg_vaddr);
	}
}

/* needs to be called with host->lock held */
void tmio_mmc_do_data_irq(struct tmio_mmc_host *host)
{
	struct mmc_data *data = host->data;
	struct mmc_command *stop;

	host->data = NULL;

	if (!data) {
		dev_warn(&host->pdev->dev, "Spurious data end IRQ\n");
		return;
	}
	stop = data->stop;

	/* FIXME - return correct transfer count on errors */
	if (!data->error)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	pr_debug("Completed data request\n");

	/*
	 * FIXME: other drivers allow an optional stop command of any given type
	 *        which we dont do, as the chip can auto generate them.
	 *        Perhaps we can be smarter about when to use auto CMD12 and
	 *        only issue the auto request when we know this is the desired
	 *        stop command, allowing fallback to the stop command the
	 *        upper layers expect. For now, we do what works.
	 */

	if (data->flags & MMC_DATA_READ) {
		if (host->chan_rx && !host->force_pio)
			tmio_mmc_check_bounce_buffer(host);
		dev_dbg(&host->pdev->dev, "Complete Rx request %p\n",
			host->mrq);
	} else {
		dev_dbg(&host->pdev->dev, "Complete Tx request %p\n",
			host->mrq);
	}

	if (stop) {
		if (stop->opcode == MMC_STOP_TRANSMISSION && !stop->arg)
			sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x000);
		else
			BUG();
	}

	schedule_work(&host->done);
}

static void tmio_mmc_seq_irq(struct tmio_mmc_host *host, unsigned int stat,
			     u32 seq_stat1, u32 seq_stat2)
{
	struct mmc_data *data;
	struct mmc_command *cmd, *sbc;

	spin_lock(&host->lock);
	data = host->data;
	cmd = host->mrq->cmd;
	sbc = host->mrq->sbc;

	if (seq_stat2) {
		pr_debug("sequencer error, CMD%d SD_INFO2=0x%x\n",
			 cmd->opcode, stat >> 16);
		if (stat & TMIO_STAT_CMDTIMEOUT) {
			cmd->error = -ETIMEDOUT;
			if (sbc)
				sbc->error = -ETIMEDOUT;
		} else if ((stat & TMIO_STAT_CRCFAIL &&
			   cmd->flags & MMC_RSP_CRC) ||
			   stat & TMIO_STAT_STOPBIT_ERR ||
			   stat & TMIO_STAT_CMD_IDX_ERR) {
			cmd->error = -EILSEQ;
			if (sbc)
				sbc->error = -EILSEQ;
		}

		if (stat & TMIO_STAT_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		else if (stat & TMIO_STAT_CRCFAIL ||
			 stat & TMIO_STAT_STOPBIT_ERR ||
			 stat & TMIO_STAT_TXUNDERRUN)
			data->error = -EILSEQ;
	}

	if (host->chan_tx && (data->flags & MMC_DATA_WRITE)) {
		u32 status = sd_ctrl_read16_and_16_as_32(host, CTL_STATUS);
		bool done = false;

		/*
		 * Has all data been written out yet? Testing on SuperH showed,
		 * that in most cases the first interrupt comes already with the
		 * BUSY status bit clear, but on some operations, like mount or
		 * in the beginning of a write / sync / umount, there is one
		 * DATAEND interrupt with the BUSY bit set, in this cases
		 * waiting for one more interrupt fixes the problem.
		 */
		if ((host->pdata->flags &
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) ==
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) {
			if (status & TMIO_STAT_SCLKDIVEN)
				done = true;
		} else {
			if (!(status & TMIO_STAT_CMD_BUSY))
				done = true;
		}

		if (!done)
		goto out;
	}

	/* mask sequencer irq */
	tmio_dm_write(host, DM_CM_INFO1_MASK, 0xffffffff);
	tasklet_schedule(&host->seq_complete);

out:
	spin_unlock(&host->lock);
}

static void tmio_mmc_data_irq(struct tmio_mmc_host *host, unsigned int stat)
{
	struct mmc_data *data;
	spin_lock(&host->lock);
	data = host->data;

	if (!data)
		goto out;

	/* The response of automatic CMD12 is stored */
	if (host->pdata->flags & TMIO_MMC_MIN_RCAR2) {
		if ((sd_ctrl_read16(host, CTL_SD_CMD) &
			(TRANSFER_MULTI | NO_CMD12_ISSUE)) == TRANSFER_MULTI) {
			if (data->stop) {
				data->stop->resp[0] =
					sd_ctrl_read16_and_16_as_32(host,
							CTL_RESPONSE);
			}
		}
	}
	if (stat & TMIO_STAT_DATATIMEOUT)
		data->error = -ETIMEDOUT;
	else if (stat & TMIO_STAT_CRCFAIL || stat & TMIO_STAT_STOPBIT_ERR ||
	    stat & TMIO_STAT_TXUNDERRUN)
		data->error = -EILSEQ;
	if (host->chan_tx && (data->flags & MMC_DATA_WRITE) && !host->force_pio) {
		u32 status = sd_ctrl_read16_and_16_as_32(host, CTL_STATUS);
		bool done = false;

		/*
		 * Has all data been written out yet? Testing on SuperH showed,
		 * that in most cases the first interrupt comes already with the
		 * BUSY status bit clear, but on some operations, like mount or
		 * in the beginning of a write / sync / umount, there is one
		 * DATAEND interrupt with the BUSY bit set, in this cases
		 * waiting for one more interrupt fixes the problem.
		 */
		if ((host->pdata->flags &
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) ==
		   (TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_USE_SCLKDIVEN)) {
			if (status & TMIO_STAT_SCLKDIVEN)
				done = true;
		} else {
			if (!(status & TMIO_STAT_CMD_BUSY))
				done = true;
		}

		if (done) {
			tmio_mmc_disable_mmc_irqs(host, TMIO_MASK_DMA);
			if (!data->error)
				tmio_set_transtate(host, TMIO_TRANSTATE_AEND);
			else
				tasklet_schedule(&host->dma_complete);
		}
	} else if (host->chan_rx && (data->flags & MMC_DATA_READ) && !host->force_pio) {
		tmio_mmc_disable_mmc_irqs(host, TMIO_MASK_DMA);
		if (!data->error)
			tmio_set_transtate(host, TMIO_TRANSTATE_AEND);
		else
			tasklet_schedule(&host->dma_complete);
	} else {
		tmio_mmc_do_data_irq(host);
		tmio_mmc_disable_mmc_irqs(host, TMIO_MASK_READOP | TMIO_MASK_WRITEOP);
	}
out:
	spin_unlock(&host->lock);
}

static void tmio_mmc_cmd_irq(struct tmio_mmc_host *host,
	unsigned int stat)
{
	struct mmc_command *cmd = host->cmd;
	int i, addr;

	spin_lock(&host->lock);

	if (!host->cmd) {
		pr_debug("Spurious CMD irq\n");
		goto out;
	}

	/* This controller is sicker than the PXA one. Not only do we need to
	 * drop the top 8 bits of the first response word, we also need to
	 * modify the order of the response for short response command types.
	 */

	for (i = 3, addr = CTL_RESPONSE ; i >= 0 ; i--, addr += 4)
		cmd->resp[i] = sd_ctrl_read16_and_16_as_32(host, addr);

	if (cmd->flags &  MMC_RSP_136) {
		cmd->resp[0] = (cmd->resp[0] << 8) | (cmd->resp[1] >> 24);
		cmd->resp[1] = (cmd->resp[1] << 8) | (cmd->resp[2] >> 24);
		cmd->resp[2] = (cmd->resp[2] << 8) | (cmd->resp[3] >> 24);
		cmd->resp[3] <<= 8;
	} else if (cmd->flags & MMC_RSP_R3) {
		cmd->resp[0] = cmd->resp[3];
	}

	if (stat & TMIO_STAT_CMDTIMEOUT)
		cmd->error = -ETIMEDOUT;
	else if ((stat & TMIO_STAT_CRCFAIL && cmd->flags & MMC_RSP_CRC) ||
		 stat & TMIO_STAT_STOPBIT_ERR ||
		 stat & TMIO_STAT_CMD_IDX_ERR) {
		cmd->error = -EILSEQ;
		if (stat & TMIO_STAT_DATAEND)
			tmio_mmc_ack_mmc_irqs(host, TMIO_STAT_DATAEND);
	}
	/* If there is data to handle we enable data IRQs here, and
	 * we will ultimatley finish the request in the data_end handler.
	 * If theres no data or we encountered an error, finish now.
	 */
	if (host->data && !cmd->error) {
		if (host->data->flags & MMC_DATA_READ) {
			if (host->force_pio || !host->chan_rx)
				tmio_mmc_enable_mmc_irqs(host, TMIO_MASK_READOP);
			else
				tasklet_schedule(&host->dma_issue);
		} else {
			if (host->force_pio || !host->chan_tx)
				tmio_mmc_enable_mmc_irqs(host, TMIO_MASK_WRITEOP);
			else
				tasklet_schedule(&host->dma_issue);
		}
	} else {
		schedule_work(&host->done);
	}

out:
	spin_unlock(&host->lock);
}

static bool __tmio_mmc_card_detect_irq(struct tmio_mmc_host *host,
				      int ireg, int status)
{
	struct mmc_host *mmc = host->mmc;

	/* Card insert / remove attempts */
	if (ireg & (TMIO_STAT_CARD_INSERT | TMIO_STAT_CARD_REMOVE)) {
		tmio_mmc_ack_mmc_irqs(host, TMIO_STAT_CARD_INSERT |
			TMIO_STAT_CARD_REMOVE);
		if ((((ireg & TMIO_STAT_CARD_REMOVE) && mmc->card) ||
		     ((ireg & TMIO_STAT_CARD_INSERT) && !mmc->card)) &&
		    !work_pending(&mmc->detect.work))
			mmc_detect_change(host->mmc, msecs_to_jiffies(100));
		return true;
	}

	return false;
}

static bool __tmio_mmc_sdcard_irq(struct tmio_mmc_host *host,
				 int ireg, int status)
{
	u64 dm_cm_info1;

	if (host->sequencer_enabled) {
		dm_cm_info1 = tmio_dm_read(host, DM_CM_INFO1);
		if (dm_cm_info1 & DM_CM_INFO1_SEQEND) {
			u64 dm_cm_info2;

			dm_cm_info2 = tmio_dm_read(host, DM_CM_INFO2);
			tmio_dm_write(host, DM_CM_INFO1, 0x0);
			tmio_dm_write(host, DM_CM_INFO2, 0x0);
			tmio_mmc_ack_mmc_irqs(host, TMIO_MASK_IRQ &
					      ~(TMIO_STAT_CARD_REMOVE |
						TMIO_STAT_CARD_INSERT));
			tmio_mmc_seq_irq(host, status,
					 dm_cm_info1, dm_cm_info2);
			return true;
		}
	}

	/* Command completion */
	if (ireg & (TMIO_STAT_CMDRESPEND | TMIO_STAT_CMDTIMEOUT)) {
		tmio_mmc_ack_mmc_irqs(host,
			     TMIO_STAT_CMDRESPEND |
			     TMIO_STAT_CMDTIMEOUT);
		tmio_mmc_cmd_irq(host, status);
		return true;
	}

	/* Data transfer */
	if (ireg & (TMIO_STAT_RXRDY | TMIO_STAT_TXRQ)) {
		tmio_mmc_ack_mmc_irqs(host, TMIO_STAT_RXRDY | TMIO_STAT_TXRQ);
		tmio_mmc_pio_irq(host);
		return true;
	}

	/* Data transfer completion */
	if (ireg & TMIO_MASK_DMA) {
		tmio_mmc_ack_mmc_irqs(host, TMIO_MASK_DMA);
		tmio_mmc_data_irq(host, status);
		return true;
	}

	return false;
}

static void __tmio_mmc_sdio_irq(struct tmio_mmc_host *host)
{
	struct mmc_host *mmc = host->mmc;
	struct tmio_mmc_data *pdata = host->pdata;
	unsigned int ireg, status;
	unsigned int sdio_status;

	if (!(pdata->flags & TMIO_MMC_SDIO_IRQ))
		return;

	if (host->sequencer_enabled)
		spin_lock(&host->lock);
	else
		__acquire(&host->lock);

	status = sd_ctrl_read16(host, CTL_SDIO_STATUS);
	ireg = status & TMIO_SDIO_MASK_ALL & ~host->sdio_irq_mask;

	sdio_status = status & ~TMIO_SDIO_MASK_ALL;
	if (pdata->flags & TMIO_MMC_SDIO_STATUS_SETBITS)
		sdio_status |= TMIO_SDIO_SETBITS_MASK;

	sd_ctrl_write16(host, CTL_SDIO_STATUS, sdio_status);

	if (host->sequencer_enabled)
		spin_unlock(&host->lock);
	else
		__release(&host->lock);

	if (mmc->caps & MMC_CAP_SDIO_IRQ && ireg & TMIO_SDIO_STAT_IOIRQ)
		mmc_signal_sdio_irq(mmc);
}

irqreturn_t tmio_mmc_irq(int irq, void *devid)
{
	struct tmio_mmc_host *host = devid;
	unsigned int ireg, status;

	status = sd_ctrl_read16_and_16_as_32(host, CTL_STATUS);
	ireg = status & TMIO_MASK_IRQ & ~host->sdcard_irq_mask;

	pr_debug_status(status);
	pr_debug_status(ireg);

	/* Clear the status except the interrupt status */
	sd_ctrl_write32_as_16_and_16(host, CTL_STATUS, TMIO_MASK_IRQ);

	if (__tmio_mmc_card_detect_irq(host, ireg, status))
		return IRQ_HANDLED;
	if (__tmio_mmc_sdcard_irq(host, ireg, status))
		return IRQ_HANDLED;
	if (__tmio_mmc_dma_irq(host))
		return IRQ_HANDLED;

	__tmio_mmc_sdio_irq(host);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(tmio_mmc_irq);

static int tmio_mmc_start_seq(struct tmio_mmc_host *host,
			      struct mmc_request *mrq)
{
	struct tmio_mmc_data *pdata = host->pdata;
	struct mmc_data *data = mrq->data;

	pr_debug("setup data transfer: blocksize %08x  nr_blocks %d\n",
		 data->blksz, data->blocks);

	if (!host->chan_rx || !host->chan_tx) {
		host->force_pio = true;
		return 0;
	}

	/* Some hardware cannot perform 2 byte requests in 4 bit mode */
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
		int blksz_2bytes = pdata->flags & TMIO_MMC_BLKSZ_2BYTES;

		if (data->blksz < 2 || (data->blksz < 4 && !blksz_2bytes)) {
			pr_err("%s: %d byte block unsupported in 4 bit mode\n",
			       mmc_hostname(host->mmc), data->blksz);
			return -EINVAL;
		}
	}

	tmio_mmc_init_sg(host, data);
	host->cmd = mrq->cmd;
	host->data = data;

	sd_ctrl_write16(host, CTL_STOP_INTERNAL_ACTION, 0x000);

	tmio_mmc_start_sequencer(host);

	if (host->force_pio) {
		host->cmd = NULL;
		host->data = NULL;
	}

	return 0;
}

static void tmio_mmc_set_blklen_and_blkcnt(struct tmio_mmc_host *host,
					   struct mmc_data *data)
{
	host->force_pio = true;
	tmio_mmc_init_sg(host, data);
	host->data = data;

	sd_ctrl_write16(host, CTL_SD_XFER_LEN, data->blksz);
	sd_ctrl_write16(host, CTL_XFER_BLK_COUNT, data->blocks);
}

static int tmio_mmc_start_data(struct tmio_mmc_host *host,
	struct mmc_data *data)
{
	struct tmio_mmc_data *pdata = host->pdata;

	pr_debug("setup data transfer: blocksize %08x  nr_blocks %d\n",
		 data->blksz, data->blocks);

	/* Some hardware cannot perform 2 byte requests in 4/8 bit mode */
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4 ||
	    host->mmc->ios.bus_width == MMC_BUS_WIDTH_8) {
		int blksz_2bytes = pdata->flags & TMIO_MMC_BLKSZ_2BYTES;

		if (data->blksz < 2 || (data->blksz < 4 && !blksz_2bytes)) {
			pr_err("%s: %d byte block unsupported in 4/8 bit mode\n",
			       mmc_hostname(host->mmc), data->blksz);
			return -EINVAL;
		}
	}

	tmio_mmc_init_sg(host, data);
	host->data = data;

	/* Set transfer length / blocksize */
	sd_ctrl_write16(host, CTL_SD_XFER_LEN, data->blksz);
	sd_ctrl_write16(host, CTL_XFER_BLK_COUNT, data->blocks);

	tmio_mmc_start_dma(host, data);

	return 0;
}

static void tmio_mmc_post_req(struct mmc_host *mmc, struct mmc_request *req,
			      int err)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = req->data;
	enum dma_data_direction dir;

	if (!host->chan_tx || !host->chan_rx || !data)
		return;

	if (data->host_cookie == COOKIE_PRE_MAPPED) {
		if (req->data->flags & MMC_DATA_READ)
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_TO_DEVICE;

		dma_unmap_sg(&host->pdev->dev, data->sg, data->sg_len, dir);
		data->host_cookie = COOKIE_UNMAPPED;
	}
}

static void tmio_mmc_pre_req(struct mmc_host *mmc, struct mmc_request *req,
			     bool is_first_req)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct mmc_data *data = req->data;
	enum dma_data_direction dir;
	int ret;

	if (!host->chan_tx || !host->chan_rx || !data)
		return;

	if (data->host_cookie == COOKIE_UNMAPPED) {
		if (req->data->flags & MMC_DATA_READ)
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_TO_DEVICE;

		ret = dma_map_sg(&host->pdev->dev, data->sg, data->sg_len, dir);
		if (ret <= 0)
			dev_err(&host->pdev->dev,
				"%s: dma_map_sg failed\n", __func__);
		else
			data->host_cookie = COOKIE_PRE_MAPPED;
	}
}

static void tmio_mmc_hw_reset(struct mmc_host *mmc)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);

	tmio_mmc_reset(host);

	tmio_mmc_reset_dma(host);

	if (host->hw_reset)
		host->hw_reset(host);
}

static int tmio_mmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	int i, ret = 0;

	if (!host->init_tuning || !host->select_tuning)
		/* Tuning is not supported */
		goto out;

	host->tap_num = host->init_tuning(host);
	if (!host->tap_num)
		/* Tuning is not supported */
		goto out;

	if (host->tap_num * 2 >= sizeof(host->taps) * BITS_PER_BYTE) {
		dev_warn_once(&host->pdev->dev,
		      "Too many taps, skipping tuning. Please consider "
		      "updating size of taps field of tmio_mmc_host\n");
		goto out;
	}

	bitmap_zero(host->taps, host->tap_num * 2);
	bitmap_zero(host->smpcmp, host->tap_num * 2);

	/* Issue CMD19 twice for each tap */
	for (i = 0; i < 2 * host->tap_num; i++) {
		if (host->prepare_tuning)
			host->prepare_tuning(host, i % host->tap_num);

		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (ret == 0)
			set_bit(i, host->taps);
		if (host->compare_scc_data)
			if (!host->compare_scc_data(host))
				set_bit(i, host->smpcmp);

		mdelay(1);
	}

	ret = host->select_tuning(host);

out:
	if (ret < 0) {
		dev_warn(&host->pdev->dev, "Tuning procedure failed\n");
		tmio_mmc_hw_reset(mmc);
	}

	return ret;
}

/* Process requests from the MMC layer */
static void tmio_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&host->lock, flags);

	if (host->mrq) {
		pr_debug("request not null\n");
		if (IS_ERR(host->mrq)) {
			spin_unlock_irqrestore(&host->lock, flags);
			mrq->cmd->error = -EAGAIN;
			mmc_request_done(mmc, mrq);
			return;
		}
	}

	host->last_req_ts = jiffies;
	wmb();
	host->mrq = mrq;

	spin_unlock_irqrestore(&host->lock, flags);

	if (host->sequencer_enabled) {
		if (mrq->data) {
			/* Start SEQ */
			ret = tmio_mmc_start_seq(host, mrq);
			if (ret)
				goto fail;
			else if (!host->force_pio) {
				/*
				 * Successed to start SEQ
				 * Wait SEQ interrupt
				 */
				schedule_delayed_work(
					&host->delayed_reset_work,
					msecs_to_jiffies(CMDREQ_TIMEOUT));
				return;
			}
		}
	}
	if (mrq->sbc) {
		init_completion(&host->completion);
		ret = tmio_mmc_start_command(host, mrq->sbc);
		if (ret)
			goto fail;
		ret = wait_for_completion_timeout(&host->completion,
					msecs_to_jiffies(CMDREQ_TIMEOUT));
		if (ret < 0)
			goto fail;
		if (!ret) {
			ret = -ETIMEDOUT;
			goto fail;
		}
		host->last_req_ts = jiffies;
		host->mrq = mrq;
		if (mrq->cmd->error && host->mmc->card) {
			ret = mrq->cmd->error;
			goto fail;
		}
		if (host->check_scc_error && host->check_scc_error(host)) {
			ret = -EILSEQ;
			goto fail;
		}
	}

	if (mrq->data) {
		if (host->sequencer_enabled) {
			/*
			 * Failed to start SEQ
			 * Set blklen and blkcnt to transfer in PIO mode
			 */
			tmio_mmc_set_blklen_and_blkcnt(host, mrq->data);
		} else {
			ret = tmio_mmc_start_data(host, mrq->data);
			if (ret)
				goto fail;
		}
		if (host->force_pio &&
		    mrq->data->host_cookie == COOKIE_PRE_MAPPED) {
			/* PIO mode, unmap pre_dma_mapped sg */
			enum dma_data_direction dir;

			if (mrq->data->flags & MMC_DATA_READ)
				dir = DMA_FROM_DEVICE;
			else
				dir = DMA_TO_DEVICE;
			dma_unmap_sg(&host->pdev->dev, mrq->data->sg,
				     mrq->data->sg_len, dir);
			mrq->data->host_cookie = COOKIE_UNMAPPED;
		}
	}

	ret = tmio_mmc_start_command(host, mrq->cmd);
	if (!ret) {
		schedule_delayed_work(&host->delayed_reset_work,
				      msecs_to_jiffies(CMDREQ_TIMEOUT));
		return;
	}

fail:
	host->force_pio = false;
	host->mrq = NULL;
	mrq->cmd->error = ret;
	mmc_request_done(mmc, mrq);
}

static int tmio_mmc_clk_enable(struct tmio_mmc_host *host)
{
	if (!host->clk_enable)
		return -ENOTSUPP;

	return host->clk_enable(host);
}

static void tmio_mmc_power_on(struct tmio_mmc_host *host, unsigned short vdd)
{
	struct mmc_host *mmc = host->mmc;
	int ret = 0;

	/* .set_ios() is returning void, so, no chance to report an error */

	if (host->set_pwr)
		host->set_pwr(host->pdev, 1);

	if (!IS_ERR(mmc->supply.vmmc)) {
		ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, vdd);
		/*
		 * Attention: empiric value. With a b43 WiFi SDIO card this
		 * delay proved necessary for reliable card-insertion probing.
		 * 100us were not enough. Is this the same 140us delay, as in
		 * tmio_mmc_set_ios()?
		 */
		udelay(200);
	}
	/*
	 * It seems, VccQ should be switched on after Vcc, this is also what the
	 * omap_hsmmc.c driver does.
	 */
	if (!IS_ERR(mmc->supply.vqmmc) && !ret) {
		ret = regulator_enable(mmc->supply.vqmmc);
		udelay(200);
	}

	if (ret < 0)
		dev_dbg(&host->pdev->dev, "Regulators failed to power up: %d\n",
			ret);
}

static void tmio_mmc_power_off(struct tmio_mmc_host *host)
{
	struct mmc_host *mmc = host->mmc;

	if (!IS_ERR(mmc->supply.vqmmc))
		regulator_disable(mmc->supply.vqmmc);

	if (!IS_ERR(mmc->supply.vmmc))
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

	if (host->set_pwr)
		host->set_pwr(host->pdev, 0);
}

static void tmio_mmc_set_bus_width(struct tmio_mmc_host *host,
				unsigned char bus_width)
{
	u16 reg = sd_ctrl_read16(host, CTL_SD_MEM_CARD_OPT)
				& ~(CARD_OPT_WIDTH | CARD_OPT_WIDTH8);

	/* reg now applies to MMC_BUS_WIDTH_4 */
	if (bus_width == MMC_BUS_WIDTH_1)
		reg |= CARD_OPT_WIDTH;
	else if (bus_width == MMC_BUS_WIDTH_8)
		reg |= CARD_OPT_WIDTH8;

	sd_ctrl_write16(host, CTL_SD_MEM_CARD_OPT, reg);
}

/* Set MMC clock / power.
 * Note: This controller uses a simple divider scheme therefore it cannot
 * run a MMC card at full speed (20MHz). The max clock is 24MHz on SD, but as
 * MMC wont run that fast, it has to be clocked at 12MHz which is the next
 * slowest setting.
 */
static void tmio_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct device *dev = &host->pdev->dev;
	unsigned long flags;

	if (!(ios->timing == MMC_TIMING_UHS_SDR104) &&
	    !(ios->timing == MMC_TIMING_MMC_HS200) &&
	    !(ios->timing == MMC_TIMING_MMC_HS400))
		if (host->disable_scc)
			host->disable_scc(mmc);

	/* reset HS400 mode */
	if (!(ios->timing == MMC_TIMING_MMC_HS400))
		if (host->reset_hs400_mode)
			host->reset_hs400_mode(mmc);

	mutex_lock(&host->ios_lock);

	spin_lock_irqsave(&host->lock, flags);
	if (host->mrq) {
		if (IS_ERR(host->mrq)) {
			dev_dbg(dev,
				"%s.%d: concurrent .set_ios(), clk %u, mode %u\n",
				current->comm, task_pid_nr(current),
				ios->clock, ios->power_mode);
			host->mrq = ERR_PTR(-EINTR);
		} else {
			dev_dbg(dev,
				"%s.%d: CMD%u active since %lu, now %lu!\n",
				current->comm, task_pid_nr(current),
				host->mrq->cmd->opcode, host->last_req_ts, jiffies);
		}
		spin_unlock_irqrestore(&host->lock, flags);

		mutex_unlock(&host->ios_lock);
		return;
	}

	host->mrq = ERR_PTR(-EBUSY);

	spin_unlock_irqrestore(&host->lock, flags);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		tmio_mmc_power_off(host);
		tmio_mmc_clk_stop(host);
		break;
	case MMC_POWER_UP:
		tmio_mmc_power_on(host, ios->vdd);
		tmio_mmc_set_clock(host, ios->clock);
		tmio_mmc_set_bus_width(host, ios->bus_width);
		break;
	case MMC_POWER_ON:
		tmio_mmc_set_clock(host, ios->clock);
		tmio_mmc_set_bus_width(host, ios->bus_width);
		break;
	}

	/* Let things settle. delay taken from winCE driver */
	udelay(140);
	if (PTR_ERR(host->mrq) == -EINTR)
		dev_dbg(&host->pdev->dev,
			"%s.%d: IOS interrupted: clk %u, mode %u",
			current->comm, task_pid_nr(current),
			ios->clock, ios->power_mode);

	/* HS400 Register setting */
	if (ios->timing == MMC_TIMING_MMC_HS400)
		if (host->prepare_hs400_tuning)
			host->prepare_hs400_tuning(mmc, ios);

	host->mrq = NULL;

	host->clk_cache = ios->clock;

	mutex_unlock(&host->ios_lock);
}

static int tmio_mmc_get_ro(struct mmc_host *mmc)
{
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct tmio_mmc_data *pdata = host->pdata;
	int ret = mmc_gpio_get_ro(mmc);
	if (ret >= 0)
		return ret;

	ret = !((pdata->flags & TMIO_MMC_WRPROTECT_DISABLE) ||
		(sd_ctrl_read16_and_16_as_32(host, CTL_STATUS) & TMIO_STAT_WRPROTECT));

	return ret;
}

static int tmio_multi_io_quirk(struct mmc_card *card,
			       unsigned int direction, int blk_size)
{
	struct tmio_mmc_host *host = mmc_priv(card->host);

	if (host->multi_io_quirk)
		return host->multi_io_quirk(card, direction, blk_size);

	return blk_size;
}

static struct mmc_host_ops tmio_mmc_ops = {
	.post_req	= tmio_mmc_post_req,
	.pre_req	= tmio_mmc_pre_req,
	.request	= tmio_mmc_request,
	.set_ios	= tmio_mmc_set_ios,
	.get_ro         = tmio_mmc_get_ro,
	.get_cd		= mmc_gpio_get_cd,
	.enable_sdio_irq = tmio_mmc_enable_sdio_irq,
	.multi_io_quirk	= tmio_multi_io_quirk,
	.hw_reset	= tmio_mmc_hw_reset,
	.execute_tuning = tmio_mmc_execute_tuning,
};

static int tmio_mmc_init_ocr(struct tmio_mmc_host *host)
{
	struct tmio_mmc_data *pdata = host->pdata;
	struct mmc_host *mmc = host->mmc;

	mmc_regulator_get_supply(mmc);

	/* use ocr_mask if no regulator */
	if (!mmc->ocr_avail)
		mmc->ocr_avail =  pdata->ocr_mask;

	/*
	 * try again.
	 * There is possibility that regulator has not been probed
	 */
	if (!mmc->ocr_avail)
		return -EPROBE_DEFER;

	return 0;
}

static void tmio_mmc_of_parse(struct platform_device *pdev,
			      struct tmio_mmc_data *pdata)
{
	const struct device_node *np = pdev->dev.of_node;
	if (!np)
		return;

	if (of_get_property(np, "toshiba,mmc-wrprotect-disable", NULL))
		pdata->flags |= TMIO_MMC_WRPROTECT_DISABLE;
}

struct tmio_mmc_host*
tmio_mmc_host_alloc(struct platform_device *pdev)
{
	struct tmio_mmc_host *host;
	struct mmc_host *mmc;

	mmc = mmc_alloc_host(sizeof(struct tmio_mmc_host), &pdev->dev);
	if (!mmc)
		return NULL;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;

	return host;
}
EXPORT_SYMBOL(tmio_mmc_host_alloc);

void tmio_mmc_host_free(struct tmio_mmc_host *host)
{
	mmc_free_host(host->mmc);
}
EXPORT_SYMBOL(tmio_mmc_host_free);

int tmio_mmc_host_probe(struct tmio_mmc_host *_host,
			struct tmio_mmc_data *pdata)
{
	struct platform_device *pdev = _host->pdev;
	struct mmc_host *mmc = _host->mmc;
	struct resource *res_ctl;
	int ret;
	u32 irq_mask = TMIO_MASK_CMD;

	tmio_mmc_of_parse(pdev, pdata);

	if (!(pdata->flags & TMIO_MMC_HAS_IDLE_WAIT))
		_host->write16_hook = NULL;

	res_ctl = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_ctl)
		return -EINVAL;

	ret = mmc_of_parse(mmc);
	if (ret < 0)
		goto host_free;

	_host->pdata = pdata;
	platform_set_drvdata(pdev, mmc);

	_host->set_pwr = pdata->set_pwr;
	_host->set_clk_div = pdata->set_clk_div;

	ret = tmio_mmc_init_ocr(_host);
	if (ret < 0)
		goto host_free;

	_host->ctl = devm_ioremap(&pdev->dev,
				  res_ctl->start, resource_size(res_ctl));
	if (!_host->ctl) {
		ret = -ENOMEM;
		goto host_free;
	}

	tmio_mmc_ops.card_busy = _host->card_busy;
	tmio_mmc_ops.start_signal_voltage_switch = _host->start_signal_voltage_switch;
	tmio_mmc_ops.select_drive_strength = _host->select_drive_strength;

	mmc->ops = &tmio_mmc_ops;

	mmc->caps |= MMC_CAP_4_BIT_DATA | pdata->capabilities;
	mmc->caps2 |= pdata->capabilities2;
	mmc->max_segs = pdata->max_segs ? pdata->max_segs : 32;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = pdata->max_blk_count ? :
		(PAGE_SIZE / mmc->max_blk_size) * mmc->max_segs;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	_host->native_hotplug = !(pdata->flags & TMIO_MMC_USE_GPIO_CD ||
				  mmc->caps & MMC_CAP_NEEDS_POLL ||
				  !mmc_card_is_removable(mmc) ||
				  mmc->slot.cd_irq >= 0);

	/*
	 * On Gen2+, eMMC with NONREMOVABLE currently fails because native
	 * hotplug gets disabled. It seems RuntimePM related yet we need further
	 * research. Since we are planning a PM overhaul anyway, let's enforce
	 * for now the device being active by enabling native hotplug always.
	 */
	if (pdata->flags & TMIO_MMC_MIN_RCAR2)
		_host->native_hotplug = true;

	if (tmio_mmc_clk_enable(_host) < 0) {
		mmc->f_max = pdata->hclk;
		mmc->f_min = mmc->f_max / 512;
	}

	/*
	 * Check the sanity of mmc->f_min to prevent tmio_mmc_set_clock() from
	 * looping forever...
	 */
	if (mmc->f_min == 0) {
		ret = -EINVAL;
		goto host_free;
	}

	/*
	 * While using internal tmio hardware logic for card detection, we need
	 * to ensure it stays powered for it to work.
	 */
	if (_host->native_hotplug)
		pm_runtime_get_noresume(&pdev->dev);

	_host->sdio_irq_enabled = false;
	if (pdata->flags & TMIO_MMC_SDIO_IRQ)
		_host->sdio_irq_mask = TMIO_SDIO_MASK_ALL;

	tmio_mmc_clk_stop(_host);
	tmio_mmc_reset(_host);
	tmio_mmc_reset_dma(_host);

	sd_ctrl_write32_as_16_and_16(_host, CTL_IRQ_MASK, TMIO_MASK_INIT);
	_host->sdcard_irq_mask = sd_ctrl_read16_and_16_as_32(_host, CTL_IRQ_MASK);
	tmio_mmc_disable_mmc_irqs(_host, TMIO_MASK_ALL);

	/* Unmask the IRQs we want to know about */
	if (!_host->chan_rx)
		irq_mask |= TMIO_MASK_READOP;
	if (!_host->chan_tx)
		irq_mask |= TMIO_MASK_WRITEOP;
	if (!_host->native_hotplug)
		irq_mask &= ~(TMIO_STAT_CARD_REMOVE | TMIO_STAT_CARD_INSERT);

	_host->sdcard_irq_mask &= ~irq_mask;

	spin_lock_init(&_host->lock);
	mutex_init(&_host->ios_lock);

	spin_lock_init(&_host->trans_lock);
	_host->trans_state = 0;

	/* Init delayed work for request timeouts */
	INIT_DELAYED_WORK(&_host->delayed_reset_work, tmio_mmc_reset_work);
	INIT_WORK(&_host->done, tmio_mmc_done_work);

	/* See if we also get DMA */
	tmio_mmc_request_dma(_host, pdata);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = mmc_add_host(mmc);
	if (ret < 0) {
		tmio_mmc_host_remove(_host);
		return ret;
	}

	dev_pm_qos_expose_latency_limit(&pdev->dev, 100);

	if (pdata->flags & TMIO_MMC_USE_GPIO_CD) {
		ret = mmc_gpio_request_cd(mmc, pdata->cd_gpio, 0);
		if (ret < 0) {
			tmio_mmc_host_remove(_host);
			return ret;
		}
		mmc_gpiod_request_cd_irq(mmc);
	}

	return 0;

host_free:

	return ret;
}
EXPORT_SYMBOL(tmio_mmc_host_probe);

void tmio_mmc_host_remove(struct tmio_mmc_host *host)
{
	struct platform_device *pdev = host->pdev;
	struct mmc_host *mmc = host->mmc;

	if (host->pdata->flags & TMIO_MMC_SDIO_IRQ)
		sd_ctrl_write16(host, CTL_TRANSACTION_CTL, 0x0000);

	if (!host->native_hotplug)
		pm_runtime_get_sync(&pdev->dev);

	dev_pm_qos_hide_latency_limit(&pdev->dev);

	mmc_remove_host(mmc);
	cancel_work_sync(&host->done);
	cancel_delayed_work_sync(&host->delayed_reset_work);
	tmio_mmc_release_dma(host);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}
EXPORT_SYMBOL(tmio_mmc_host_remove);

#ifdef CONFIG_PM
int tmio_mmc_host_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct tmio_mmc_host *host = mmc_priv(mmc);

	tmio_mmc_disable_mmc_irqs(host, TMIO_MASK_ALL);

	if (host->clk_cache)
		tmio_mmc_clk_stop(host);

	if (host->clk_disable)
		host->clk_disable(host);

	return 0;
}
EXPORT_SYMBOL(tmio_mmc_host_runtime_suspend);

static bool tmio_mmc_can_retune(struct tmio_mmc_host *host)
{
	return host->tap_num && mmc_can_retune(host->mmc);
}

int tmio_mmc_host_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct tmio_mmc_host *host = mmc_priv(mmc);

	tmio_mmc_clk_enable(host);
	tmio_mmc_hw_reset(mmc);

	if (host->clk_cache)
		tmio_mmc_set_clock(host, host->clk_cache);

	tmio_mmc_enable_dma(host, true);

	if (tmio_mmc_can_retune(host) && host->select_tuning(host))
		dev_warn(&host->pdev->dev, "Tuning selection failed\n");

	return 0;
}
EXPORT_SYMBOL(tmio_mmc_host_runtime_resume);
#endif

MODULE_LICENSE("GPL v2");
