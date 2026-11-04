// SPDX-License-Identifier: GPL-2.0-only
/* Apple S8000 TCLK function, reconstructed from j71t iOS 16.7.10. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define TOUCH_DISABLE	BIT(31)
#define TOUCH_ENABLE	BIT(19)
#define TOUCH_BUSY	BIT(18)
#define TOUCH_DIV	GENMASK(9, 0)
#define TOUCH_OFFSET	0x78000
#define TOUCH_DIVISOR	(24000000 / 32768)

struct s8000_touch_clock {
	struct clk_hw hw;
	struct regmap *regmap;
	struct device *dev;
};

#define to_touch_clock(hw) container_of(hw, struct s8000_touch_clock, hw)

static int s8000_touch_clock_prepare(struct clk_hw *hw)
{
	struct s8000_touch_clock *clock = to_touch_clock(hw);
	u32 value;
	int ret;

	ret = regmap_update_bits(clock->regmap, TOUCH_OFFSET, TOUCH_DISABLE, 0);
	if (ret)
		return ret;
	udelay(8);
	ret = regmap_update_bits(clock->regmap, TOUCH_OFFSET,
				 TOUCH_DISABLE | TOUCH_ENABLE | TOUCH_DIV,
				 TOUCH_ENABLE | TOUCH_DIVISOR);
	if (!ret)
		ret = regmap_read_poll_timeout(clock->regmap, TOUCH_OFFSET, value,
					       !(value & TOUCH_BUSY), 10, 20000);
	if (ret) {
		regmap_update_bits(clock->regmap, TOUCH_OFFSET,
				   TOUCH_ENABLE | TOUCH_DISABLE, TOUCH_DISABLE);
		dev_err(clock->dev, "touch clock enable failed: %d\n", ret);
	}
	return ret;
}

static void s8000_touch_clock_unprepare(struct clk_hw *hw)
{
	struct s8000_touch_clock *clock = to_touch_clock(hw);
	u32 value;
	int ret;

	ret = regmap_update_bits(clock->regmap, TOUCH_OFFSET, TOUCH_ENABLE, 0);
	if (!ret)
		ret = regmap_read_poll_timeout(clock->regmap, TOUCH_OFFSET, value,
					       !(value & TOUCH_BUSY), 10, 20000);
	if (ret)
		dev_err(clock->dev, "touch clock disable failed: %d\n", ret);
	regmap_update_bits(clock->regmap, TOUCH_OFFSET, TOUCH_DISABLE, TOUCH_DISABLE);
	udelay(100);
}

static unsigned long s8000_touch_clock_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct s8000_touch_clock *clock = to_touch_clock(hw);
	u32 value;

	if (regmap_read(clock->regmap, TOUCH_OFFSET, &value))
		return 0;
	value = FIELD_GET(TOUCH_DIV, value);
	return value ? parent_rate / value : 0;
}

static const struct clk_ops s8000_touch_clock_ops = {
	.prepare = s8000_touch_clock_prepare,
	.unprepare = s8000_touch_clock_unprepare,
	.recalc_rate = s8000_touch_clock_recalc_rate,
};

static int s8000_touch_clock_probe(struct platform_device *pdev)
{
	struct s8000_touch_clock *clock;
	struct clk_init_data init = {};
	const char *parent;
	u32 offset;
	int ret;

	clock = devm_kzalloc(&pdev->dev, sizeof(*clock), GFP_KERNEL);
	if (!clock)
		return -ENOMEM;
	clock->dev = &pdev->dev;
	clock->regmap = syscon_node_to_regmap(pdev->dev.of_node->parent);
	if (IS_ERR(clock->regmap))
		return PTR_ERR(clock->regmap);
	ret = of_property_read_u32_index(pdev->dev.of_node, "reg", 0, &offset);
	if (ret || offset != TOUCH_OFFSET)
		return -EINVAL;
	parent = of_clk_get_parent_name(pdev->dev.of_node, 0);
	if (!parent)
		return -EINVAL;
	init.name = "s8000-touch";
	init.ops = &s8000_touch_clock_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.parent_names = &parent;
	init.num_parents = 1;
	clock->hw.init = &init;
	ret = devm_clk_hw_register(&pdev->dev, &clock->hw);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_simple_get, &clock->hw);
}

static const struct of_device_id s8000_touch_clock_match[] = {
	{ .compatible = "apple,s8000-touch-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, s8000_touch_clock_match);
static struct platform_driver s8000_touch_clock_driver = {
	.probe = s8000_touch_clock_probe,
	.driver = {
		.name = "apple-s8000-touch-clock",
		.of_match_table = s8000_touch_clock_match,
	},
};
module_platform_driver(s8000_touch_clock_driver);
MODULE_DESCRIPTION("Apple S8000 touchscreen clock");
MODULE_LICENSE("GPL");
