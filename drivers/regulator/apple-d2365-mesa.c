// SPDX-License-Identifier: GPL-2.0-only
/*
 * Enable-only D2365 supplies for the J71t Mesa sensor.
 *
 * The D2365 LDO table maps Mesa's pmuL(0x307) and pmuL(0x30f) functions to
 * registers 0x0312 (3V0) and 0x0319 (1V8), bit zero.  This deliberately
 * accepts only those two volatile enable registers.  It cannot program a
 * voltage, touch another PMIC register, or access persistent storage.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define D2365_MESA_3V0_ENABLE_REG	0x0312
#define D2365_MESA_1V8_ENABLE_REG	0x0319

struct d2365_mesa_regulator {
	struct regulator_desc desc;
};

static const struct regulator_ops d2365_mesa_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int d2365_mesa_probe(struct platform_device *pdev)
{
	struct d2365_mesa_regulator *mesa;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	u32 address;
	int ret;

	ret = of_property_read_u32_index(pdev->dev.of_node, "reg", 0,
					 &address);
	if (ret)
		return ret;

	if (address != D2365_MESA_3V0_ENABLE_REG &&
	    address != D2365_MESA_1V8_ENABLE_REG)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "refusing non-Mesa D2365 register %#x\n",
				     address);

	mesa = devm_kzalloc(&pdev->dev, sizeof(*mesa), GFP_KERNEL);
	if (!mesa)
		return -ENOMEM;

	mesa->desc.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					 "d2365-mesa-%03x", address);
	if (!mesa->desc.name)
		return -ENOMEM;
	mesa->desc.id = address;
	mesa->desc.type = REGULATOR_VOLTAGE;
	mesa->desc.owner = THIS_MODULE;
	mesa->desc.ops = &d2365_mesa_ops;
	mesa->desc.enable_reg = address;
	mesa->desc.enable_mask = BIT(0);

	config.dev = &pdev->dev;
	config.of_node = pdev->dev.of_node;
	config.regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!config.regmap)
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "PMIC register map unavailable\n");
	config.init_data = of_get_regulator_init_data(&pdev->dev,
						      config.of_node,
						      &mesa->desc);

	rdev = devm_regulator_register(&pdev->dev, &mesa->desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
				     "register %#x\n", address);

	dev_info(&pdev->dev,
		 "enable-only Mesa supply at D2365 register %#x bit 0\n",
		 address);
	return 0;
}

static const struct of_device_id d2365_mesa_match[] = {
	{ .compatible = "apple,d2365-mesa-ldo" },
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_mesa_match);

static struct platform_driver d2365_mesa_driver = {
	.probe = d2365_mesa_probe,
	.driver = {
		.name = "apple-d2365-mesa",
		.of_match_table = d2365_mesa_match,
	},
};
module_platform_driver(d2365_mesa_driver);

MODULE_DESCRIPTION("Apple D2365 enable-only Mesa supplies");
MODULE_LICENSE("GPL");
