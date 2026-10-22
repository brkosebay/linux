// SPDX-License-Identifier: GPL-2.0-only
/* Bounded, explicitly armed J71t D2365 charging experiment. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/firmware.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define J71T_STATUS 0x18a
#define J71T_INPUT_CONTROL 0xa00
#define J71T_INPUT_SUSPEND BIT(1)
#define J71T_INPUT_LIMIT 0xa03
#define J71T_CHARGE_CURRENT 0xa08
#define J71T_MAX_CHARGE_CURRENT 0xa0c
#define J71T_CHARGE_TIMEOUT BIT(4)
#define J71T_CHARGE_END BIT(2)
#define J71T_CHARGE_CODE 50 /* 2500 mA, still bounded by 0xa0c[5:0]. */
#define J71T_REQUEST_UA 2400000

struct j71t_charger {
	struct device *dev;
	struct regmap *map;
	struct power_supply *psy;
	/* Serializes policy changes, register transactions and work lifetime. */
	struct mutex lock;
	struct delayed_work work;
	u8 input_code;
	u8 old_control;
	u8 old_input;
	u8 old_charge;
	bool calibrated;
	bool saved;
	bool active;
	bool stopping;
};

/* AppleD2365PMU 0x6bc07ec high branch, then 0x6bbff6c encoder. */
static int j71t_calibrate(const u8 *data, size_t size, u8 *code)
{
	s64 point[5], slope, gain, result, ma;
	int i;

	if (size != 24 || get_unaligned_le32(data + 20) != 1)
		return -EINVAL;
	for (i = 0; i < 5; i++) {
		point[i] = (s32)get_unaligned_le32(data + i * 4);
		if (point[i] <= 0 || point[i] > (3000LL << 16) ||
		    (i && point[i] <= point[i - 1]))
			return -ERANGE;
	}
	slope = div_s64(point[4] - point[3], 30);
	if (slope < (8LL << 16) || slope > (12LL << 16))
		return -ERANGE;
	gain = div_s64(point[3] - point[4] + (600LL << 16), 300);
	result = ((gain * (300LL << 16)) >> 16) - point[3] + (4125LL << 16);
	ma = result >> 16;
	if (ma < 75 || ma > 2400)
		return -ERANGE;
	*code = div_s64((ma * 8 - 600) & 0xfff8, 100);
	return *code <= 0xba ? 0 : -ERANGE;
}

static int j71t_read(struct j71t_charger *charger, unsigned int reg, u8 *value)
{
	unsigned int raw;
	int ret = regmap_read(charger->map, reg, &raw);

	if (!ret)
		*value = raw;
	return ret;
}

static int j71t_write_checked(struct j71t_charger *charger, unsigned int reg, u8 value)
{
	u8 readback;
	int ret;

	ret = regmap_write(charger->map, reg, value);
	if (ret)
		return ret;
	ret = j71t_read(charger, reg, &readback);
	if (ret)
		return ret;
	return readback == value ? 0 : -EIO;
}

/* Own only bit 1; preserve every other current control-register bit. */
static int j71t_set_suspend(struct j71t_charger *charger, u8 saved)
{
	u8 control;
	int ret;

	ret = j71t_read(charger, J71T_INPUT_CONTROL, &control);
	if (ret)
		return ret;
	control = (control & ~J71T_INPUT_SUSPEND) | (saved & J71T_INPUT_SUSPEND);
	return j71t_write_checked(charger, J71T_INPUT_CONTROL, control);
}

static int j71t_online(struct j71t_charger *charger)
{
	union power_supply_propval value;
	int ret;

	if (!charger->psy)
		return -EAGAIN;
	ret = power_supply_get_property_from_supplier(charger->psy,
						      POWER_SUPPLY_PROP_ONLINE, &value);
	return ret ? ret : !!value.intval;
}

static int j71t_battery_safe(void)
{
	struct power_supply *battery;
	union power_supply_propval value;
	int ret;

	battery = power_supply_get_by_name("battery");
	if (!battery)
		return -ENODEV;
	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_PRESENT, &value);
	if (ret || value.intval != 1) {
		ret = ret ?: -ENODEV;
		goto out;
	}
	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_TEMP, &value);
	if (ret || value.intval < 100 || value.intval > 400) {
		ret = ret ?: -ERANGE;
		goto out;
	}
	ret = power_supply_get_property(battery, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
	if (!ret && (value.intval < 3000000 || value.intval > 4200000))
		ret = -ERANGE;
out:
	power_supply_put(battery);
	return ret;
}

/* Never re-arm on reconnection: a new owner-authorized request is required. */
static int j71t_restore(struct j71t_charger *charger)
{
	int ret, first = 0;

	charger->active = false;
	if (!charger->saved)
		return 0;
	/* Keep input suspended until BOTH saved budgets have been verified. */
	ret = j71t_set_suspend(charger, J71T_INPUT_SUSPEND);
	if (ret)
		return ret;
	ret = j71t_write_checked(charger, J71T_CHARGE_CURRENT, charger->old_charge);
	if (ret && !first)
		first = ret;
	ret = j71t_write_checked(charger, J71T_INPUT_LIMIT, charger->old_input);
	if (ret && !first)
		first = ret;
	if (!first)
		first = j71t_set_suspend(charger, charger->old_control);
	if (first)
		j71t_set_suspend(charger, J71T_INPUT_SUSPEND);
	if (!first)
		charger->saved = false;
	return first;
}

static int j71t_guard(struct j71t_charger *charger)
{
	u8 status, cap;
	int ret = j71t_online(charger);

	if (ret != 1)
		return ret < 0 ? ret : -ENOLINK;
	ret = j71t_battery_safe();
	if (ret)
		return ret;
	ret = j71t_read(charger, J71T_STATUS, &status);
	if (ret)
		return ret;
	if (status & (J71T_CHARGE_TIMEOUT | J71T_CHARGE_END))
		return -EBUSY;
	ret = j71t_read(charger, J71T_MAX_CHARGE_CURRENT, &cap);
	if (ret)
		return ret;
	return (cap & 0x3f) >= J71T_CHARGE_CODE ? 0 : -ERANGE;
}

static int j71t_enable(struct j71t_charger *charger)
{
	int ret;

	if (!charger->calibrated)
		return -ENODATA;
	ret = j71t_guard(charger);
	if (ret)
		return ret;
	if (charger->active)
		return 0;
	if (charger->saved)
		return -EBUSY;
	ret = j71t_read(charger, J71T_INPUT_CONTROL, &charger->old_control);
	if (ret)
		return ret;
	ret = j71t_read(charger, J71T_INPUT_LIMIT, &charger->old_input);
	if (ret)
		return ret;
	ret = j71t_read(charger, J71T_CHARGE_CURRENT, &charger->old_charge);
	if (ret)
		return ret;
	charger->saved = true;
	/* Same three controls as iOS; no voltage/watchdog/IRQ/cap writes. */
	ret = j71t_set_suspend(charger, J71T_INPUT_SUSPEND);
	if (!ret)
		ret = j71t_write_checked(charger, J71T_CHARGE_CURRENT, J71T_CHARGE_CODE);
	if (!ret)
		ret = j71t_write_checked(charger, J71T_INPUT_LIMIT, charger->input_code);
	if (!ret)
		ret = j71t_guard(charger);
	if (!ret)
		ret = j71t_set_suspend(charger, 0);
	if (!ret)
		ret = j71t_guard(charger);
	if (ret) {
		j71t_restore(charger);
		return ret;
	}
	charger->active = true;
	dev_info(charger->dev,
		 "explicit Mac input budget 2400 mA: input 0x%02x, battery limit 2500 mA\n",
		 charger->input_code);
	return 0;
}

static void j71t_policy_work(struct work_struct *work)
{
	struct j71t_charger *charger = container_of(to_delayed_work(work),
							  struct j71t_charger, work);
	u8 input, control, charge;
	int ret = 0;

	mutex_lock(&charger->lock);
	if (charger->active) {
		ret = j71t_guard(charger);
		if (!ret)
			ret = j71t_read(charger, J71T_INPUT_LIMIT, &input);
		if (!ret)
			ret = j71t_read(charger, J71T_INPUT_CONTROL, &control);
		if (!ret)
			ret = j71t_read(charger, J71T_CHARGE_CURRENT, &charge);
		if (!ret && (input != charger->input_code ||
			     (control & J71T_INPUT_SUSPEND) || charge != J71T_CHARGE_CODE))
			ret = -EIO;
		if (ret) {
			dev_warn(charger->dev, "charging request revoked: %d\n", ret);
			charger->active = false;
		}
	}
	if (!charger->active && charger->saved) {
		ret = j71t_restore(charger);
		if (ret)
			dev_err_ratelimited(charger->dev, "charger restore pending: %d\n", ret);
	}
	if (!charger->stopping)
		mod_delayed_work(system_power_efficient_wq, &charger->work,
				 msecs_to_jiffies(charger->saved && !charger->active ? 100 : 1000));
	mutex_unlock(&charger->lock);
	if (charger->psy)
		power_supply_changed(charger->psy);
}

static int j71t_get_property(struct power_supply *psy, enum power_supply_property prop,
			     union power_supply_propval *value)
{
	struct j71t_charger *charger = power_supply_get_drvdata(psy);
	u8 raw;
	int ret, online;

	mutex_lock(&charger->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		online = j71t_online(charger);
		ret = online < 0 ? online : 0;
		value->intval = online == 1;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = j71t_read(charger, J71T_INPUT_LIMIT, &raw);
		if (!ret)
			value->intval = 75000 + raw * 12500;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = j71t_read(charger, J71T_CHARGE_CURRENT, &raw);
		if (!ret)
			value->intval = raw * 50000;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = j71t_read(charger, J71T_STATUS, &raw);
		online = j71t_online(charger);
		if (!ret) {
			value->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			if (online == 1 && !(raw & (J71T_CHARGE_TIMEOUT | J71T_CHARGE_END)) &&
			    (raw & 3))
				value->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&charger->lock);
	return ret;
}

static int j71t_set_property(struct power_supply *psy, enum power_supply_property prop,
			     const union power_supply_propval *value)
{
	struct j71t_charger *charger = power_supply_get_drvdata(psy);
	int ret;

	if (prop != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	    (value->intval != 0 && value->intval != J71T_REQUEST_UA))
		return -EINVAL;
	mutex_lock(&charger->lock);
	if (charger->stopping)
		ret = -ESHUTDOWN;
	else if (!value->intval)
		ret = j71t_restore(charger);
	else
		ret = j71t_enable(charger);
	if (!charger->stopping)
		mod_delayed_work(system_power_efficient_wq, &charger->work, 0);
	mutex_unlock(&charger->lock);
	power_supply_changed(psy);
	return ret;
}

static int j71t_property_writeable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static void j71t_external_power_changed(struct power_supply *psy)
{
	struct j71t_charger *charger = power_supply_get_drvdata(psy);

	mutex_lock(&charger->lock);
	if (!charger->stopping)
		mod_delayed_work(system_power_efficient_wq, &charger->work, 0);
	mutex_unlock(&charger->lock);
}

static const enum power_supply_property j71t_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static const struct power_supply_desc j71t_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = j71t_properties,
	.num_properties = ARRAY_SIZE(j71t_properties),
	.get_property = j71t_get_property,
	.set_property = j71t_set_property,
	.property_is_writeable = j71t_property_writeable,
	.external_power_changed = j71t_external_power_changed,
};

static int j71t_probe(struct platform_device *pdev)
{
	struct power_supply_config config = {};
	const struct firmware *fw;
	struct j71t_charger *charger;
	int ret;

	charger = devm_kzalloc(&pdev->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;
	charger->dev = &pdev->dev;
	charger->map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!charger->map)
		return -ENODEV;
	ret = devm_mutex_init(&pdev->dev, &charger->lock);
	if (ret)
		return ret;
	ret = devm_delayed_work_autocancel(&pdev->dev, &charger->work, j71t_policy_work);
	if (ret)
		return ret;
	ret = request_firmware_direct(&fw, "apple/j71t-CBAT.bin", &pdev->dev);
	if (!ret) {
		ret = j71t_calibrate(fw->data, fw->size, &charger->input_code);
		release_firmware(fw);
		charger->calibrated = !ret;
	}
	if (ret)
		dev_warn(&pdev->dev,
			 "CBAT unavailable/unsupported (%d): charging writes disabled\n", ret);
	config.drv_data = charger;
	config.fwnode = dev_fwnode(&pdev->dev);
	charger->psy = devm_power_supply_register(&pdev->dev, &j71t_desc, &config);
	if (IS_ERR(charger->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->psy),
				     "charger supply registration\n");
	platform_set_drvdata(pdev, charger);
	mod_delayed_work(system_power_efficient_wq, &charger->work, 0);
	dev_info(&pdev->dev, "J71t charger: explicit 2400000-uA request required for measured Mac source\n");
	return 0;
}

static int j71t_quiesce(struct device *dev)
{
	struct j71t_charger *charger = dev_get_drvdata(dev);
	int attempt, ret = 0;

	mutex_lock(&charger->lock);
	charger->stopping = true;
	mutex_unlock(&charger->lock);
	cancel_delayed_work_sync(&charger->work);
	for (attempt = 0; attempt < 3; attempt++) {
		mutex_lock(&charger->lock);
		ret = j71t_restore(charger);
		mutex_unlock(&charger->lock);
		if (!ret)
			break;
		msleep(20);
	}
	if (ret)
		dev_crit(charger->dev, "charger restore failed at shutdown/remove: %d\n", ret);
	return ret;
}

static void j71t_stop(struct platform_device *pdev)
{
	j71t_quiesce(&pdev->dev);
}

static int j71t_resume(struct device *dev)
{
	struct j71t_charger *charger = dev_get_drvdata(dev);

	mutex_lock(&charger->lock);
	charger->stopping = false;
	mod_delayed_work(system_power_efficient_wq, &charger->work, 0);
	mutex_unlock(&charger->lock);
	return 0;
}

static int j71t_suspend(struct device *dev)
{
	int ret = j71t_quiesce(dev);

	/* A failed suspend need not receive resume; keep rollback retries alive. */
	if (ret)
		j71t_resume(dev);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(j71t_pm_ops, j71t_suspend, j71t_resume);

static const struct of_device_id j71t_match[] = {
	{ .compatible = "apple,j71t-d2365-charger" },
	{}
};
MODULE_DEVICE_TABLE(of, j71t_match);

static struct platform_driver j71t_driver = {
	.probe = j71t_probe,
	.remove = j71t_stop,
	.shutdown = j71t_stop,
	.driver = {
		.name = "apple-j71t-charger",
		.of_match_table = j71t_match,
		.pm = pm_sleep_ptr(&j71t_pm_ops),
	},
};
module_platform_driver(j71t_driver);
MODULE_DESCRIPTION("Explicitly armed Apple J71t D2365 charging policy");
MODULE_LICENSE("GPL");
