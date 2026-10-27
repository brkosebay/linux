// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Driver for backlight controllers attached via Apple DWI 2-wire interface
 *
 * Copyright (c) 2024 Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>

#define DWI_BL_CTL			0x0
#define DWI_BL_CTL_SEND1		BIT(0)
#define DWI_BL_CTL_SEND2		BIT(4)
#define DWI_BL_CTL_SEND3		BIT(5)
#define DWI_BL_CTL_LE_DATA		BIT(6)
/* Only used on Apple A9 and later */
#define DWI_BL_CTL_SEND4		BIT(12)

#define DWI_BL_CMD			0x4
#define DWI_BL_CMD_TYPE			GENMASK(31, 28)
#define DWI_BL_CMD_TYPE_SET_BRIGHTNESS	0xa
#define DWI_BL_CMD_DATA			GENMASK(10, 0)

#define DWI_BL_CTL_SEND			(DWI_BL_CTL_SEND1 | \
					 DWI_BL_CTL_SEND2 | \
					 DWI_BL_CTL_SEND3 | \
					 DWI_BL_CTL_LE_DATA | \
					 DWI_BL_CTL_SEND4)

#define DWI_BL_MAX_BRIGHTNESS		2047

struct apple_dwi_bl {
	struct device *dev;
	void __iomem *base;
	void __iomem *block;
	/* Serialize block initialization and brightness transactions. */
	struct mutex lock;
};

static int dwi_bl_wait(struct apple_dwi_bl *dwi_bl)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout(dwi_bl->base + DWI_BL_CTL, value,
				 !(value & DWI_BL_CTL_SEND1), 10, 1000);
	if (ret)
		dev_err_ratelimited(dwi_bl->dev, "DWI busy timeout: %#x\n", value);
	return ret;
}

/* AppleS8000DWI::start(), J71t nclk=1, polarity=1, STR=8000, lockout=5. */
static int dwi_bl_init(struct apple_dwi_bl *dwi_bl)
{
	int ret;

	if (!dwi_bl->block)
		return 0;

	mutex_lock(&dwi_bl->lock);
	writel(0x00000001, dwi_bl->block + 0x00);
	writel(0x0002ee00, dwi_bl->block + 0xd0);
	writel(0x00000078, dwi_bl->block + 0x04);
	writel(0, dwi_bl->block + 0x84);
	writel(0x41, dwi_bl->block + 0x80);
	ret = dwi_bl_wait(dwi_bl);
	mutex_unlock(&dwi_bl->lock);
	if (!ret)
		dev_info(dwi_bl->dev, "DWI block initialized (1 ms transaction bound)\n");
	return ret;
}

static int dwi_bl_update_status(struct backlight_device *bl)
{
	struct apple_dwi_bl *dwi_bl = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	u32 cmd = 0;
	int ret;

	cmd |= FIELD_PREP(DWI_BL_CMD_DATA, brightness);
	cmd |= FIELD_PREP(DWI_BL_CMD_TYPE, DWI_BL_CMD_TYPE_SET_BRIGHTNESS);

	mutex_lock(&dwi_bl->lock);
	ret = dwi_bl_wait(dwi_bl);
	if (!ret) {
		writel(cmd, dwi_bl->base + DWI_BL_CMD);
		writel(DWI_BL_CTL_SEND, dwi_bl->base + DWI_BL_CTL);
	}
	mutex_unlock(&dwi_bl->lock);
	return ret;
}

static int dwi_bl_get_brightness(struct backlight_device *bl)
{
	struct apple_dwi_bl *dwi_bl = bl_get_data(bl);

	u32 cmd = readl(dwi_bl->base + DWI_BL_CMD);

	return FIELD_GET(DWI_BL_CMD_DATA, cmd);
}

static const struct backlight_ops dwi_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = dwi_bl_get_brightness,
	.update_status	= dwi_bl_update_status
};

static int dwi_bl_probe(struct platform_device *dev)
{
	struct apple_dwi_bl *dwi_bl;
	struct backlight_device *bl;
	struct backlight_properties props;
	struct resource *res;
	int brightness, ret;

	dwi_bl = devm_kzalloc(&dev->dev, sizeof(*dwi_bl), GFP_KERNEL);
	if (!dwi_bl)
		return -ENOMEM;

	dwi_bl->base = devm_platform_get_and_ioremap_resource(dev, 0, &res);
	if (IS_ERR(dwi_bl->base))
		return PTR_ERR(dwi_bl->base);

	dwi_bl->dev = &dev->dev;
	mutex_init(&dwi_bl->lock);
	if (of_device_is_compatible(dev->dev.of_node, "apple,s8000-dwi-bl")) {
		if (resource_size(res) >= 0xd4) {
			/* The recovered boot words are specific to this board. */
			if (of_machine_is_compatible("apple,j71t"))
				dwi_bl->block = dwi_bl->base;
			dwi_bl->base += 0x80;
		} else {
			dev_warn(&dev->dev, "legacy DWI window: block resume init unavailable\n");
		}
	}
	/* Preserve the boot brightness before the init transaction clears DATA. */
	brightness = FIELD_GET(DWI_BL_CMD_DATA,
			       readl(dwi_bl->base + DWI_BL_CMD));
	ret = dwi_bl_init(dwi_bl);
	if (ret)
		return ret;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = DWI_BL_MAX_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	bl = devm_backlight_device_register(&dev->dev, dev->name, &dev->dev,
					dwi_bl, &dwi_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	platform_set_drvdata(dev, dwi_bl);

	bl->props.brightness = brightness;

	return backlight_update_status(bl);
}

/* The backlight class child restores brightness after this parent resumes. */
static int dwi_bl_resume(struct device *dev)
{
	return dwi_bl_init(dev_get_drvdata(dev));
}

static DEFINE_SIMPLE_DEV_PM_OPS(dwi_bl_pm_ops, NULL, dwi_bl_resume);

static const struct of_device_id dwi_bl_of_match[] = {
	{ .compatible = "apple,dwi-bl" },
	{},
};

MODULE_DEVICE_TABLE(of, dwi_bl_of_match);

static struct platform_driver dwi_bl_driver = {
	.driver		= {
		.name	= "apple-dwi-bl",
		.pm = pm_sleep_ptr(&dwi_bl_pm_ops),
		.of_match_table = dwi_bl_of_match
	},
	.probe		= dwi_bl_probe,
};

module_platform_driver(dwi_bl_driver);

MODULE_DESCRIPTION("Apple DWI Backlight Driver");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
