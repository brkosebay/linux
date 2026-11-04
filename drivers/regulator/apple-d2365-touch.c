// SPDX-License-Identifier: GPL-2.0-only
/* D2365 pmuL(0x21f): enable-only analog touch supply. */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

static const struct regulator_ops d2365_touch_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc d2365_touch_desc = {
	.name = "d2365-touch-analog",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &d2365_touch_ops,
	.enable_reg = 0x031c,
	.enable_mask = BIT(0),
};

static int d2365_touch_probe(struct platform_device *pdev)
{
	struct regulator_config config = {};
	struct regulator_dev *regulator;
	u32 address;
	int ret;

	ret = of_property_read_u32_index(pdev->dev.of_node, "reg", 0, &address);
	if (ret || address != d2365_touch_desc.enable_reg)
		return -EINVAL;
	config.dev = &pdev->dev;
	config.of_node = pdev->dev.of_node;
	config.regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!config.regmap)
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "PMIC register map unavailable\n");
	config.init_data = of_get_regulator_init_data(&pdev->dev, config.of_node,
						    &d2365_touch_desc);
	regulator = devm_regulator_register(&pdev->dev, &d2365_touch_desc, &config);
	return PTR_ERR_OR_ZERO(regulator);
}

static const struct of_device_id d2365_touch_match[] = {
	{ .compatible = "apple,d2365-touch-analog" },
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_touch_match);
static struct platform_driver d2365_touch_driver = {
	.probe = d2365_touch_probe,
	.driver = {
		.name = "apple-d2365-touch",
		.of_match_table = d2365_touch_match,
	},
};
module_platform_driver(d2365_touch_driver);
MODULE_DESCRIPTION("D2365 touchscreen analog power switch");
MODULE_LICENSE("GPL");
