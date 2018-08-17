/*
 * Renesas Clock Pulse Generator / Module Standby and Software Reset
 *
 * Copyright (C) 2015 Glider bvba
 *
 * Based on clk-mstp.c, clk-rcar-gen2.c, and clk-rcar-gen3.c
 *
 * Copyright (C) 2013 Ideas On Board SPRL
 * Copyright (C) 2015 Renesas Electronics Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/renesas.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/soc/renesas/s2ram_ddr_backup.h>

#include <dt-bindings/clock/renesas-cpg-mssr.h>

#include "renesas-cpg-mssr.h"
#include "clk-div6.h"

#ifdef DEBUG
#define WARN_DEBUG(x)	WARN_ON(x)
#else
#define WARN_DEBUG(x)	do { } while (0)
#endif

#ifdef CONFIG_RCAR_DDR_BACKUP
static struct hw_register cpg_ip_regs[] = {
	{"CPGWPCR",	0x904, 32, 0},
	{"CPGWPR",	0x900, 32, 0},
	{"FRQCRB",	0x004, 32, 0},
	{"FRQCRC",	0x0E0, 32, 0},
	{"PLLECR",	0x0D0, 32, 0},
	{"PLL0CR",	0x0D8, 32, 0},
	{"PLL2CR",	0x02C, 32, 0},
	{"PLL4CR",	0x1F4, 32, 0},
	{"PLL0STPCR",	0x0F0, 32, 0},
	{"PLL2STPCR",	0x0F8, 32, 0},
	{"PLL3STPCR",	0x0FC, 32, 0},
	{"PLL4STPCR",	0x1F8, 32, 0},
	{"SD0CKCR",	0x074, 32, 0},
	{"SD1CKCR",	0x078, 32, 0},
	{"SD2CKCR",	0x268, 32, 0},
	{"SD3CKCR",	0x26C, 32, 0},
	{"RPCCKCR",	0x038, 32, 0},
	{"CANFDCKCR",	0x244, 32, 0},
	{"MSOCKCR",	0x014, 32, 0},
	{"HDMICKCR",	0x250, 32, 0},
	{"CSI0CKCR",	0x00C, 32, 0},
	{"CSIREFCKCR",	0X034, 32, 0},
	{"DVFSCR0",	0x058, 32, 0},
	{"DVFSCR1",	0x05C, 32, 0},
	{"FSAPBR",	0x700, 32, 0},
	{"FSCLKCSR",	0x704, 32, 0},
	{"FSCNTCHKH0",	0x710, 32, 0},
	{"FSCNTCHKH1",	0x714, 32, 0},
	{"FSCNTCHKH2",	0x718, 32, 0},
	{"FSCNTCHKH3",	0x71C, 32, 0},
	{"FSCNTCHKH4",	0x720, 32, 0},
	{"FSCNTCHKH5",	0x724, 32, 0},
	{"FSCNTCHKH6",	0x728, 32, 0},
	{"FSCNTCHKL0",	0x730, 32, 0},
	{"FSCNTCHKL1",	0x734, 32, 0},
	{"FSCNTCHKL2",	0x738, 32, 0},
	{"FSCNTCHKL3",	0x73C, 32, 0},
	{"FSCNTCHKL4",	0x740, 32, 0},
	{"FSCNTCHKL5",	0x744, 32, 0},
	{"FSCNTCHKL6",	0x748, 32, 0},
	{"FSSEQCHKCSR",	0xAF4, 32, 0},
};

static struct rcar_ip cpg_ip = {
	.ip_name   = "CPG",
	.reg_count = ARRAY_SIZE(cpg_ip_regs),
	.ip_reg    = cpg_ip_regs,
};
#endif /* CONFIG_RCAR_DDR_BACKUP */

/*
 * Module Standby and Software Reset register offets.
 *
 * If the registers exist, these are valid for SH-Mobile, R-Mobile,
 * R-Car Gen 2, and R-Car Gen 3.
 * These are NOT valid for R-Car Gen1 and RZ/A1!
 */

/*
 * Module Stop Status Register offsets
 */

static const u16 mstpsr[] = {
	0x030, 0x038, 0x040, 0x048, 0x04C, 0x03C, 0x1C0, 0x1C4,
	0x9A0, 0x9A4, 0x9A8, 0x9AC,
};

#define	MSTPSR(i)	mstpsr[i]


/*
 * System Module Stop Control Register offsets
 */

static const u16 smstpcr[] = {
	0x130, 0x134, 0x138, 0x13C, 0x140, 0x144, 0x148, 0x14C,
	0x990, 0x994, 0x998, 0x99C,
};

#define	SMSTPCR(i)	smstpcr[i]


/*
 * Software Reset Register offsets
 */

static const u16 srcr[] = {
	0x0A0, 0x0A8, 0x0B0, 0x0B8, 0x0BC, 0x0C4, 0x1C8, 0x1CC,
	0x920, 0x924, 0x928, 0x92C,
};

#define	SRCR(i)		srcr[i]


/* Realtime Module Stop Control Register offsets */
#define RMSTPCR(i)	(smstpcr[i] - 0x20)

/* Modem Module Stop Control Register offsets (r8a73a4) */
#define MMSTPCR(i)	(smstpcr[i] + 0x20)

/* Software Reset Clearing Register offsets */
#define	SRSTCLR(i)	(0x940 + (i) * 4)


/**
 * Clock Pulse Generator / Module Standby and Software Reset Private Data
 *
 * @dev: CPG/MSSR device
 * @base: CPG/MSSR register block base address
 * @mstp_lock: protects writes to SMSTPCR
 * @clks: Array containing all Core and Module Clocks
 * @num_core_clks: Number of Core Clocks in clks[]
 * @num_mod_clks: Number of Module Clocks in clks[]
 * @last_dt_core_clk: ID of the last Core Clock exported to DT
 */
struct cpg_mssr_priv {
	struct device *dev;
	void __iomem *base;
	spinlock_t mstp_lock;

	struct clk **clks;
	unsigned int num_core_clks;
	unsigned int num_mod_clks;
	unsigned int last_dt_core_clk;
};


/**
 * struct mstp_clock - MSTP gating clock
 * @hw: handle between common and hardware-specific interfaces
 * @index: MSTP clock number
 * @priv: CPG/MSSR private data
 */
struct mstp_clock {
	struct clk_hw hw;
	u32 index;
	struct cpg_mssr_priv *priv;
};

#define to_mstp_clock(_hw) container_of(_hw, struct mstp_clock, hw)

static int cpg_mstp_clock_endisable(struct clk_hw *hw, bool enable)
{
	struct mstp_clock *clock = to_mstp_clock(hw);
	struct cpg_mssr_priv *priv = clock->priv;
	unsigned int reg = clock->index / 32;
	unsigned int bit = clock->index % 32;
	struct device *dev = priv->dev;
	u32 bitmask = BIT(bit);
	unsigned long flags;
	unsigned int i;
	u32 value;

	dev_dbg(dev, "MSTP %u%02u/%pC %s\n", reg, bit, hw->clk,
		enable ? "ON" : "OFF");
	spin_lock_irqsave(&priv->mstp_lock, flags);

	value = readl(priv->base + SMSTPCR(reg));
	if (enable)
		value &= ~bitmask;
	else
		value |= bitmask;
	writel(value, priv->base + SMSTPCR(reg));

	spin_unlock_irqrestore(&priv->mstp_lock, flags);

	if (!enable)
		return 0;

	for (i = 1000; i > 0; --i) {
		if (!(readl(priv->base + MSTPSR(reg)) & bitmask))
			break;
		cpu_relax();
	}

	if (!i) {
		dev_err(dev, "Failed to enable SMSTP %p[%d]\n",
			priv->base + SMSTPCR(reg), bit);
		return -ETIMEDOUT;
	}

	return 0;
}

static int cpg_mstp_clock_enable(struct clk_hw *hw)
{
	return cpg_mstp_clock_endisable(hw, true);
}

static void cpg_mstp_clock_disable(struct clk_hw *hw)
{
	cpg_mstp_clock_endisable(hw, false);
}

static int cpg_mstp_clock_is_enabled(struct clk_hw *hw)
{
	struct mstp_clock *clock = to_mstp_clock(hw);
	struct cpg_mssr_priv *priv = clock->priv;
	u32 value;

	value = readl(priv->base + MSTPSR(clock->index / 32));

	return !(value & BIT(clock->index % 32));
}

static const struct clk_ops cpg_mstp_clock_ops = {
	.enable = cpg_mstp_clock_enable,
	.disable = cpg_mstp_clock_disable,
	.is_enabled = cpg_mstp_clock_is_enabled,
};

static
struct clk *cpg_mssr_clk_src_twocell_get(struct of_phandle_args *clkspec,
					 void *data)
{
	unsigned int clkidx = clkspec->args[1];
	struct cpg_mssr_priv *priv = data;
	struct device *dev = priv->dev;
	unsigned int idx;
	const char *type;
	struct clk *clk;

	switch (clkspec->args[0]) {
	case CPG_CORE:
		type = "core";
		if (clkidx > priv->last_dt_core_clk) {
			dev_err(dev, "Invalid %s clock index %u\n", type,
			       clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[clkidx];
		break;

	case CPG_MOD:
		type = "module";
		idx = MOD_CLK_PACK(clkidx);
		if (clkidx % 100 > 31 || idx >= priv->num_mod_clks) {
			dev_err(dev, "Invalid %s clock index %u\n", type,
				clkidx);
			return ERR_PTR(-EINVAL);
		}
		clk = priv->clks[priv->num_core_clks + idx];
		break;

	default:
		dev_err(dev, "Invalid CPG clock type %u\n", clkspec->args[0]);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(clk))
		dev_err(dev, "Cannot get %s clock %u: %ld", type, clkidx,
		       PTR_ERR(clk));
	else
		dev_dbg(dev, "clock (%u, %u) is %pC at %pCr Hz\n",
			clkspec->args[0], clkspec->args[1], clk, clk);
	return clk;
}

static void __init cpg_mssr_register_core_clk(const struct cpg_core_clk *core,
					      const struct cpg_mssr_info *info,
					      struct cpg_mssr_priv *priv)
{
	struct clk *clk = NULL, *parent;
	struct device *dev = priv->dev;
	unsigned int id = core->id, div = core->div;
	const char *parent_name;

	WARN_DEBUG(id >= priv->num_core_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	if (!core->name) {
		/* Skip NULLified clock */
		return;
	}

	switch (core->type) {
	case CLK_TYPE_IN:
		clk = of_clk_get_by_name(priv->dev->of_node, core->name);
		break;

	case CLK_TYPE_FF:
	case CLK_TYPE_DIV6P1:
	case CLK_TYPE_DIV6_RO:
		WARN_DEBUG(core->parent >= priv->num_core_clks);
		parent = priv->clks[core->parent];
		if (IS_ERR(parent)) {
			clk = parent;
			goto fail;
		}

		parent_name = __clk_get_name(parent);

		if (core->type == CLK_TYPE_DIV6_RO)
			/* Multiply with the DIV6 register value */
			div *= (readl(priv->base + core->offset) & 0x3f) + 1;

		if (core->type == CLK_TYPE_DIV6P1) {
			clk = cpg_div6_register(core->name, 1, &parent_name,
						priv->base + core->offset);
		} else {
			clk = clk_register_fixed_factor(NULL, core->name,
							parent_name, 0,
							core->mult, div);
		}
		break;

	default:
		if (info->cpg_clk_register)
			clk = info->cpg_clk_register(dev, core, info,
						     priv->clks, priv->base);
		else
			dev_err(dev, "%s has unsupported core clock type %u\n",
				core->name, core->type);
		break;
	}

	if (IS_ERR_OR_NULL(clk))
		goto fail;

	dev_dbg(dev, "Core clock %pC at %pCr Hz\n", clk, clk);
	priv->clks[id] = clk;
	return;

fail:
	dev_err(dev, "Failed to register %s clock %s: %ld\n", "core",
		core->name, PTR_ERR(clk));
}

static void __init cpg_mssr_register_mod_clk(const struct mssr_mod_clk *mod,
					     const struct cpg_mssr_info *info,
					     struct cpg_mssr_priv *priv)
{
	struct mstp_clock *clock = NULL;
	struct device *dev = priv->dev;
	unsigned int id = mod->id;
	struct clk_init_data init;
	struct clk *parent, *clk;
	const char *parent_name;
	unsigned int i;

	WARN_DEBUG(id < priv->num_core_clks);
	WARN_DEBUG(id >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(mod->parent >= priv->num_core_clks + priv->num_mod_clks);
	WARN_DEBUG(PTR_ERR(priv->clks[id]) != -ENOENT);

	if (!mod->name) {
		/* Skip NULLified clock */
		return;
	}

	parent = priv->clks[mod->parent];
	if (IS_ERR(parent)) {
		clk = parent;
		goto fail;
	}

	clock = kzalloc(sizeof(*clock), GFP_KERNEL);
	if (!clock) {
		clk = ERR_PTR(-ENOMEM);
		goto fail;
	}

	init.name = mod->name;
	init.ops = &cpg_mstp_clock_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	for (i = 0; i < info->num_crit_mod_clks; i++)
		if (id == info->crit_mod_clks[i]) {
#ifdef CLK_ENABLE_HAND_OFF
			dev_dbg(dev, "MSTP %s setting CLK_ENABLE_HAND_OFF\n",
				mod->name);
			init.flags |= CLK_ENABLE_HAND_OFF;
			break;
#else
			dev_dbg(dev, "Ignoring MSTP %s to prevent disabling\n",
				mod->name);
			kfree(clock);
			return;
#endif
		}

	parent_name = __clk_get_name(parent);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clock->index = id - priv->num_core_clks;
	clock->priv = priv;
	clock->hw.init = &init;

	clk = clk_register(NULL, &clock->hw);
	if (IS_ERR(clk))
		goto fail;

	dev_dbg(dev, "Module clock %pC at %pCr Hz\n", clk, clk);
	priv->clks[id] = clk;
	return;

fail:
	dev_err(dev, "Failed to register %s clock %s: %ld\n", "module",
		mod->name, PTR_ERR(clk));
	kfree(clock);
}

struct cpg_mssr_clk_domain {
	struct generic_pm_domain genpd;
	struct device_node *np;
	unsigned int num_core_pm_clks;
	unsigned int core_pm_clks[0];
};

static struct cpg_mssr_clk_domain *cpg_mssr_clk_domain;

static bool cpg_mssr_is_pm_clk(const struct of_phandle_args *clkspec,
			       struct cpg_mssr_clk_domain *pd)
{
	unsigned int i;

	if (clkspec->np != pd->np || clkspec->args_count != 2)
		return false;

	switch (clkspec->args[0]) {
	case CPG_CORE:
		for (i = 0; i < pd->num_core_pm_clks; i++)
			if (clkspec->args[1] == pd->core_pm_clks[i])
				return true;
		return false;

	case CPG_MOD:
		return true;

	default:
		return false;
	}
}

int cpg_mssr_attach_dev(struct generic_pm_domain *unused, struct device *dev)
{
	struct cpg_mssr_clk_domain *pd = cpg_mssr_clk_domain;
	struct device_node *np = dev->of_node;
	struct of_phandle_args clkspec;
	struct clk *clk;
	int i = 0;
	int error;

	if (!pd) {
		dev_dbg(dev, "CPG/MSSR clock domain not yet available\n");
		return -EPROBE_DEFER;
	}

	while (!of_parse_phandle_with_args(np, "clocks", "#clock-cells", i,
					   &clkspec)) {
		if (cpg_mssr_is_pm_clk(&clkspec, pd))
			goto found;

		of_node_put(clkspec.np);
		i++;
	}

	return 0;

found:
	clk = of_clk_get_from_provider(&clkspec);
	of_node_put(clkspec.np);

	if (IS_ERR(clk))
		return PTR_ERR(clk);

	error = pm_clk_create(dev);
	if (error) {
		dev_err(dev, "pm_clk_create failed %d\n", error);
		goto fail_put;
	}

	error = pm_clk_add_clk(dev, clk);
	if (error) {
		dev_err(dev, "pm_clk_add_clk %pC failed %d\n", clk, error);
		goto fail_destroy;
	}

	return 0;

fail_destroy:
	pm_clk_destroy(dev);
fail_put:
	clk_put(clk);
	return error;
}

void cpg_mssr_detach_dev(struct generic_pm_domain *unused, struct device *dev)
{
	if (!list_empty(&dev->power.subsys_data->clock_list))
		pm_clk_destroy(dev);
}

static int __init cpg_mssr_add_clk_domain(struct device *dev,
					  const unsigned int *core_pm_clks,
					  unsigned int num_core_pm_clks)
{
	struct device_node *np = dev->of_node;
	struct generic_pm_domain *genpd;
	struct cpg_mssr_clk_domain *pd;
	size_t pm_size = num_core_pm_clks * sizeof(core_pm_clks[0]);

	pd = devm_kzalloc(dev, sizeof(*pd) + pm_size, GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->np = np;
	pd->num_core_pm_clks = num_core_pm_clks;
	memcpy(pd->core_pm_clks, core_pm_clks, pm_size);

	genpd = &pd->genpd;
	genpd->name = np->name;
	genpd->flags = GENPD_FLAG_PM_CLK;
	genpd->attach_dev = cpg_mssr_attach_dev;
	genpd->detach_dev = cpg_mssr_detach_dev;
	pm_genpd_init(genpd, &pm_domain_always_on_gov, false);
	cpg_mssr_clk_domain = pd;

	of_genpd_add_provider_simple(np, genpd);
	return 0;
}

static const struct of_device_id cpg_mssr_match[] = {
#ifdef CONFIG_ARCH_R8A7743
	{
		.compatible = "renesas,r8a7743-cpg-mssr",
		.data = &r8a7743_cpg_mssr_info,
	},
#endif
#ifdef CONFIG_ARCH_R8A7745
	{
		.compatible = "renesas,r8a7745-cpg-mssr",
		.data = &r8a7745_cpg_mssr_info,
	},
#endif
#ifdef CONFIG_ARCH_R8A7795
	{
		.compatible = "renesas,r8a7795-cpg-mssr",
		.data = &r8a7795_cpg_mssr_info,
	},
#endif
#ifdef CONFIG_ARCH_R8A7796
	{
		.compatible = "renesas,r8a7796-cpg-mssr",
		.data = &r8a7796_cpg_mssr_info,
	},
#endif
#ifdef CONFIG_ARCH_R8A77965
	{
		.compatible = "renesas,r8a77965-cpg-mssr",
		.data = &r8a77965_cpg_mssr_info,
	},
#endif
	{ /* sentinel */ }
};

static void cpg_mssr_del_clk_provider(void *data)
{
	of_clk_del_provider(data);
}

static int __init cpg_mssr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct cpg_mssr_info *info;
	struct cpg_mssr_priv *priv;
	unsigned int nclks, i;
	struct resource *res;
	struct clk **clks;
	int error;

	info = of_match_node(cpg_mssr_match, np)->data;
	if (info->init) {
		error = info->init(dev);
		if (error)
			return error;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->mstp_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	nclks = info->num_total_core_clks + info->num_hw_mod_clks;
	clks = devm_kmalloc_array(dev, nclks, sizeof(*clks), GFP_KERNEL);
	if (!clks)
		return -ENOMEM;

	priv->clks = clks;
	priv->num_core_clks = info->num_total_core_clks;
	priv->num_mod_clks = info->num_hw_mod_clks;
	priv->last_dt_core_clk = info->last_dt_core_clk;
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < nclks; i++)
		clks[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < info->num_core_clks; i++)
		cpg_mssr_register_core_clk(&info->core_clks[i], info, priv);

	for (i = 0; i < info->num_mod_clks; i++)
		cpg_mssr_register_mod_clk(&info->mod_clks[i], info, priv);

	error = of_clk_add_provider(np, cpg_mssr_clk_src_twocell_get, priv);
	if (error)
		return error;

	error = devm_add_action_or_reset(dev,
					 cpg_mssr_del_clk_provider,
					 np);
	if (error)
		return error;

	error = cpg_mssr_add_clk_domain(dev, info->core_pm_clks,
					info->num_core_pm_clks);
	if (error)
		return error;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cpg_mssr_suspend(struct device *dev)
{
	int ret = 0;
#ifdef CONFIG_RCAR_DDR_BACKUP
	struct cpg_mssr_priv *priv;

	pr_debug("%s\n", __func__);

	priv = dev_get_drvdata(dev);

	if (!cpg_ip.virt_addr)
		cpg_ip.virt_addr = priv->base;

	ret = rcar_handle_registers(&cpg_ip, DO_BACKUP);
#endif /* CONFIG_RCAR_DDR_BACKUP */
	return ret;
}

static int cpg_mssr_resume(struct device *dev)
{
	int ret = 0;
#ifdef CONFIG_RCAR_DDR_BACKUP
	pr_debug("%s\n", __func__);
	ret = rcar_handle_registers(&cpg_ip, DO_RESTORE);
#endif /* CONFIG_RCAR_DDR_BACKUP */
	return ret;
}

static SIMPLE_DEV_PM_OPS(cpg_mssr_pm_ops,
			cpg_mssr_suspend, cpg_mssr_resume);
#define DEV_PM_OPS (&cpg_mssr_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver cpg_mssr_driver = {
	.driver		= {
		.name	= "renesas-cpg-mssr",
		.pm	= DEV_PM_OPS,
		.of_match_table = cpg_mssr_match,
	},
};

static int __init cpg_mssr_init(void)
{
	return platform_driver_probe(&cpg_mssr_driver, cpg_mssr_probe);
}

subsys_initcall(cpg_mssr_init);

void __init cpg_core_nullify_range(struct cpg_core_clk *core_clks,
				   unsigned int num_core_clks,
				   unsigned int first_clk,
				   unsigned int last_clk)
{
	unsigned int i;

	for (i = 0; i < num_core_clks; i++)
		if (core_clks[i].id >= first_clk &&
		    core_clks[i].id <= last_clk)
			core_clks[i].name = NULL;
}

void __init mssr_mod_nullify(struct mssr_mod_clk *mod_clks,
			     unsigned int num_mod_clks,
			     const unsigned int *clks, unsigned int n)
{
	unsigned int i, j;

	for (i = 0, j = 0; i < num_mod_clks && j < n; i++)
		if (mod_clks[i].id == clks[j]) {
			mod_clks[i].name = NULL;
			j++;
		}
}

void __init mssr_mod_reparent(struct mssr_mod_clk *mod_clks,
			      unsigned int num_mod_clks,
			      const struct mssr_mod_reparent *clks,
			      unsigned int n)
{
	unsigned int i, j;

	for (i = 0, j = 0; i < num_mod_clks && j < n; i++)
		if (mod_clks[i].id == clks[j].clk) {
			mod_clks[i].parent = clks[j].parent;
			j++;
		}
}

MODULE_DESCRIPTION("Renesas CPG/MSSR Driver");
MODULE_LICENSE("GPL v2");
