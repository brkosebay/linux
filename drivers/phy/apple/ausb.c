// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple AUSB PHY
 *
 * Copyright (C) 2025 Nick Chan
 */


#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/soc/apple/tunable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

static bool runtime_pm;
module_param(runtime_pm, bool, 0444);
MODULE_PARM_DESC(runtime_pm, "Enable DT-selected j71t PHY runtime PM (default: off)");

/* These look similar to ATC's USB2PHY_CTL and USB2PHY_SIG */
#define AUSB_CTL			0x00
#define AUSB_CTL_RESET			BIT(0)
#define AUSB_CTL_PWRDOWN		BIT(2)
#define AUSB_CTL_SIDDQ   		BIT(3)

#define AUSB_SIG			0x04
#define AUSB_SIG_VBUSDET_FORCE_EN	BIT(1)
#define AUSB_SIG_CABLE_CONNECTED	BIT(8)
#define AUSB_SIG_HOST			(7 << 12)

/* Tunable registers */
#define AUSB_CFG0			0x08
#define AUSB_CFG1			0x0c
#define AUSB_TRIM			0x1c

struct ausb_usbphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
	struct phy_provider *phy_provider;
	u32 cfg0_device;
	u32 cfg1_device;
	u32 cfg0_host;
	u32 cfg1_host;
	/* Serialized by the PHY core's phy->mutex. */
	bool host;
	bool powered;
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);
	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

static void ausb_phy_write_tunables(struct ausb_usbphy *aphy)
{
	writel(aphy->host ? aphy->cfg0_host : aphy->cfg0_device,
	       aphy->base + AUSB_CFG0);
	writel(aphy->host ? aphy->cfg1_host : aphy->cfg1_device,
	       aphy->base + AUSB_CFG1);
}

/*
 * Role change on a running PHY, as AppleS5L8960XUSBPhy::enableHostMode does
 * it for both directions: mode tunables, trim word 0, then a reset pulse.
 * iOS does not touch AUSB_SIG here, so neither do we.
 */
static int ausb_phy_usb2_set_mode(struct phy *phy, enum phy_mode mode,
				  int submode)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);
	bool host;

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_HOST_LS:
	case PHY_MODE_USB_HOST_FS:
	case PHY_MODE_USB_HOST_HS:
	case PHY_MODE_USB_HOST_SS:
		host = true;
		break;
	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_DEVICE_LS:
	case PHY_MODE_USB_DEVICE_FS:
	case PHY_MODE_USB_DEVICE_HS:
	case PHY_MODE_USB_DEVICE_SS:
		host = false;
		break;
	default:
		dev_err(aphy->dev, "Unknown mode for usb2 phy: %d\n", mode);
		return -EINVAL;
	}

	/* A second host-side user (EHCI, then OHCI) must not re-pulse reset. */
	if (host == aphy->host)
		return 0;
	aphy->host = host;
	/* A powered-down PHY picks the mode up at its next power-on. */
	if (!aphy->powered)
		return 0;

	ausb_phy_write_tunables(aphy);
	writel(0, aphy->base + AUSB_TRIM);
	set32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	usleep_range(20, 40);
	clear32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	usleep_range(1000, 1500);
	dev_dbg(aphy->dev, "%s mode\n", host ? "host" : "device");

	return 0;
}

static int ausb_phy_usb2_power_on(struct phy *phy)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);

	/* set a sane tunable: device mode unless a host user asked first */
	ausb_phy_write_tunables(aphy);

	set32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(20);
	clear32(aphy->base, AUSB_CTL_PWRDOWN | AUSB_CTL_SIDDQ);
	udelay(20);
	clear32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(1500);
	set32(aphy->base + AUSB_SIG, AUSB_SIG_VBUSDET_FORCE_EN);
	aphy->powered = true;

	return 0;
}

static int ausb_phy_usb2_power_off(struct phy *phy)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);

	set32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(20);
	set32(aphy->base, AUSB_CTL_PWRDOWN | AUSB_CTL_SIDDQ);
	udelay(20);
	clear32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(1500);
	clear32(aphy->base + AUSB_SIG, AUSB_SIG_VBUSDET_FORCE_EN);
	aphy->powered = false;
	/* The device controller never sets a mode: default it for the next user. */
	aphy->host = false;

	return 0;
}


static const struct phy_ops ausb_phy_usb2_ops = {
	.owner = THIS_MODULE,
	.set_mode = ausb_phy_usb2_set_mode,
	.init = ausb_phy_usb2_power_on,
	.exit = ausb_phy_usb2_power_off,
};

/*
 * Generic PHY core holds a parent runtime-PM reference from phy_power_on()
 * through phy_power_off(). DWC2 therefore keeps the PHY domain powered while
 * it owns the link. No cable/traffic heuristic or connected-gadget suspend.
 */
static const struct phy_ops ausb_pm_phy_ops = {
	.set_mode = ausb_phy_usb2_set_mode,
	.owner = THIS_MODULE,
};

static int ausb_runtime_suspend(struct device *dev)
{
	struct ausb_usbphy *aphy = dev_get_drvdata(dev);

	if (device_may_wakeup(dev) || dev->power.wakeup_path)
		return -EBUSY;
	return ausb_phy_usb2_power_off(aphy->phy);
}

static int ausb_runtime_resume(struct device *dev)
{
	struct ausb_usbphy *aphy = dev_get_drvdata(dev);

	/* genpd is on before this callback; replay tunables and reset delays. */
	return ausb_phy_usb2_power_on(aphy->phy);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ausb_pm_ops, ausb_runtime_suspend,
				 ausb_runtime_resume, NULL);

static int ausb_usbphy_probe(struct platform_device *pdev) {
	struct ausb_usbphy *aphy;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	bool use_runtime_pm;
	int ret;

	aphy = devm_kzalloc(&pdev->dev, sizeof(*aphy), GFP_KERNEL);
	if (!aphy)
		return -ENOMEM;

	if (of_property_read_u32(np, "apple,cfg0-device", &aphy->cfg0_device))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg1-device", &aphy->cfg1_device))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg0-host", &aphy->cfg0_host))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg1-host", &aphy->cfg1_host))
		return -ENODEV;

	aphy->dev = dev;

	aphy->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(aphy->base))
		return PTR_ERR(aphy->base);

	platform_set_drvdata(pdev, aphy);
	use_runtime_pm = runtime_pm && of_machine_is_compatible("apple,j71t") &&
		of_property_read_bool(np, "apple,enable-runtime-pm");
	if (use_runtime_pm) {
		/* The first PHY operation must replay setup, even after firmware use. */
		ret = pm_runtime_set_suspended(dev);
		if (ret)
			return ret;
		/* Must precede phy_create(), which inherits the parent's PM policy. */
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
		dev_info(dev, "opt-in runtime PM; controller retains active PHY hold\n");
	}

	aphy->phy = devm_phy_create(aphy->dev, NULL,
				    use_runtime_pm ? &ausb_pm_phy_ops : &ausb_phy_usb2_ops);
	if (IS_ERR(aphy->phy))
		return PTR_ERR(aphy->phy);

	phy_set_drvdata(aphy->phy, aphy);

	aphy->phy_provider = devm_of_phy_provider_register(&pdev->dev,
		of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(aphy->phy_provider);
}

static const struct of_device_id ausb_usbphy_of_match[] = {
	{ .compatible = "apple,ausb-phy" },
	{},
};

static struct platform_driver ausb_usbphy_driver = {
	.probe = ausb_usbphy_probe,
	.driver = {
		.of_match_table = ausb_usbphy_of_match,
		.name = "phy-apple-ausb",
		.pm = pm_ptr(&ausb_pm_ops),
	}

};

module_platform_driver(ausb_usbphy_driver);
