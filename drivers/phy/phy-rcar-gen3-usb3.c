/*
 * Renesas R-Car Gen3 for USB3.0 PHY driver
 *
 * Copyright (C) 2017 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

#define USB30_CLKSET0	0x034
#define USB30_CLKSET1	0x036
#define USB30_SSC_SET	0x038

#define USB30_PHY_ENABLE	0x060

#define USB30_VBUS_EN	0x064

#define CLKSET0_RESERVED		0x05c0
#define CLKSET0_USB_EXTAL		(CLKSET0_RESERVED | 0x0002)

#define CLKSET1_PLL_MULTI_SHIFT		6
#define CLKSET1_PLL_MULTI_USB_EXTAL	(0x64 << CLKSET1_PLL_MULTI_SHIFT)
#define CLKSET1_PHYRESET	BIT(4)
#define CLKSET1_REF_CLKDIV_USB_EXTAL	BIT(3)
#define CLKSET1_RESERVED_1		BIT(1)
#define CLKSET1_REF_CLK_SEL_USB_EXTAL	0
#define CLKSET1_USB_EXTAL	(CLKSET1_PLL_MULTI_USB_EXTAL | \
				 CLKSET1_REF_CLKDIV_USB_EXTAL | \
				 CLKSET1_RESERVED_1 | \
				 CLKSET1_REF_CLK_SEL_USB_EXTAL)

#define SSC_SET_SSC_EN		BIT(12)
#define SSC_SET_RANGE_SHIFT	9
#define SSC_SET_RANGE_4980	(0x0 << SSC_SET_RANGE_SHIFT)
#define SSC_SET_RANGE_4492	(0x1 << SSC_SET_RANGE_SHIFT)
#define SSC_SET_RANGE_4003	(0x2 << SSC_SET_RANGE_SHIFT)

#define PHY_RESET_EN	BIT(4)

#define VBUS_EN_VBUS_EN		BIT(1)

struct rcar_gen3_usb3 {
	void __iomem *base;
	struct phy *phy;
	u32 ssc_range;
	bool usb_extal;
};

static void rcar_gen3_phy_usb3_enable_ssc(struct rcar_gen3_usb3 *r)
{
	u16 val = SSC_SET_SSC_EN;

	switch (r->ssc_range) {
	case 4980:
		val |= SSC_SET_RANGE_4980;
		break;
	case 4492:
		val |= SSC_SET_RANGE_4492;
		break;
	case 4003:
		val |= SSC_SET_RANGE_4003;
		break;
	default:
		dev_err(&r->phy->dev, "%s: unsupported range (%x)\n", __func__,
			r->ssc_range);
		return;
	}

	writew(val, r->base + USB30_SSC_SET);
}

static void rcar_gen3_phy_usb3_select_usb_extal(struct rcar_gen3_usb3 *r)
{
	writew(CLKSET0_USB_EXTAL, r->base + USB30_CLKSET0);
	writew(PHY_RESET_EN, r->base + USB30_PHY_ENABLE);
	writew(CLKSET1_USB_EXTAL | CLKSET1_PHYRESET, r->base + USB30_CLKSET1);
	usleep_range(10, 20);
	writew(CLKSET1_USB_EXTAL, r->base + USB30_CLKSET1);
}

static int rcar_gen3_phy_usb3_init(struct phy *p)
{
	struct rcar_gen3_usb3 *r = phy_get_drvdata(p);

	dev_vdbg(&r->phy->dev, "%s: enter (%d, %d)\n", __func__, r->usb_extal,
		 r->ssc_range);

	if (r->usb_extal)
		rcar_gen3_phy_usb3_select_usb_extal(r);
	if (r->ssc_range)
		rcar_gen3_phy_usb3_enable_ssc(r);

	/* Enables VBUS detection anyway */
	writew(VBUS_EN_VBUS_EN, r->base + USB30_VBUS_EN);

	return 0;
}

static const struct phy_ops rcar_gen3_phy_usb3_ops = {
	.init		= rcar_gen3_phy_usb3_init,
	.owner		= THIS_MODULE,
};

static const struct of_device_id rcar_gen3_phy_usb3_match_table[] = {
	{ .compatible = "renesas,usb3-phy-r8a7795" },
	{ .compatible = "renesas,usb3-phy-r8a7796" },
	{ .compatible = "renesas,rcar-gen3-usb3-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, rcar_gen3_phy_usb3_match_table);

static int rcar_gen3_phy_usb3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rcar_gen3_usb3 *r;
	struct phy_provider *provider;
	struct resource *res;
	int ret = 0;

	if (!dev->of_node) {
		dev_err(dev, "This driver needs device tree\n");
		return -EINVAL;
	}

	r = devm_kzalloc(dev, sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	r->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(r->base))
		return PTR_ERR(r->base);

	/*
	 * devm_phy_create() will call pm_runtime_enable(&phy->dev);
	 * And then, phy-core will manage runtime pm for this device.
	 */
	pm_runtime_enable(dev);

	r->phy = devm_phy_create(dev, NULL, &rcar_gen3_phy_usb3_ops);
	if (IS_ERR(r->phy)) {
		dev_err(dev, "Failed to create USB3 PHY\n");
		ret = PTR_ERR(r->phy);
		goto error;
	}

	if (of_property_read_u32(dev->of_node, "renesas,ssc-range",
				 &r->ssc_range) < 0)
		r->ssc_range = 0;

	r->usb_extal = of_property_read_bool(dev->of_node,
					     "renesas,usb-extal");

	platform_set_drvdata(pdev, r);
	phy_set_drvdata(r->phy, r);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "Failed to register PHY provider\n");
		ret = PTR_ERR(provider);
		goto error;
	}

	return 0;

error:
	pm_runtime_disable(dev);

	return ret;
}

static int rcar_gen3_phy_usb3_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
};

static struct platform_driver rcar_gen3_phy_usb3_driver = {
	.driver = {
		.name		= "phy_rcar_gen3_usb3",
		.of_match_table	= rcar_gen3_phy_usb3_match_table,
	},
	.probe	= rcar_gen3_phy_usb3_probe,
	.remove = rcar_gen3_phy_usb3_remove,
};
module_platform_driver(rcar_gen3_phy_usb3_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car Gen3 USB 3.0 PHY");
MODULE_AUTHOR("Yoshihiro Shimoda <yoshihiro.shimoda.uh@renesas.com>");