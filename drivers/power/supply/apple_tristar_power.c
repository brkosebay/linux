// SPDX-License-Identifier: GPL-2.0-only
/* J71t CBTL1610 source detection, reconstructed from the iOS kernelcache. */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/usb/gadget.h>

#define TRISTAR_VBUS 0x0c
#define TRISTAR_REVISION 0x0f
#define TRISTAR_J71T_REVISION 0x8a
#define TRISTAR_DX_CONTROL 0x01
#define D2365_ADC_DONE 0x156
#define D2365_ADC_START 0x4000
#define D2365_ADC_RESULT 0x4001

struct apple_tristar {
	struct i2c_client *client;
	struct power_supply *psy;
	struct regmap *pmic;
	struct device_node *usb_node;
	struct notifier_block usb_notifier;
	atomic_t reset_generation;
	int seen_generation;
	struct delayed_work work;
	/* Serializes register reads and protects the cached source state. */
	struct mutex lock;
	int online;
	int error;
	int current_ua;
	int usb_type;
	int brick_ua;
	unsigned long classify_after;
	u8 saved_route;
	bool restore_pending;
	bool brick_sampled;
	bool adc_poisoned;
	bool stopping;
	bool notifier_registered;
	bool sleeping;
};

static int apple_tristar_read(struct i2c_client *client, u8 reg)
{
	union i2c_smbus_data data;
	int ret;

	/* iOS op=1: selector + STOP, then a separate read START. */
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			       I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE, NULL);
	if (!ret)
		ret = __i2c_smbus_xfer(client->adapter, client->addr, client->flags,
				       I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	return ret ?: data.byte;
}

static int apple_tristar_write_checked(struct apple_tristar *ts, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ts->client, TRISTAR_DX_CONTROL, value);
	if (ret)
		return ret;
	ret = apple_tristar_read(ts->client, TRISTAR_DX_CONTROL);
	return ret < 0 ? ret : ret == value ? 0 : -EIO;
}

static int apple_tristar_restore_route(struct apple_tristar *ts)
{
	int ret;

	if (!ts->restore_pending)
		return 0;
	ret = apple_tristar_write_checked(ts, ts->saved_route);
	if (!ret)
		ts->restore_pending = false;
	return ret;
}

static int apple_tristar_pmic_read(struct apple_tristar *ts, unsigned int reg)
{
	unsigned int value;
	int ret = regmap_read(ts->pmic, reg, &value);

	return ret ?: value;
}

/* Exclusive ADC use. Never re-trigger after uncertain completion. */
static int apple_tristar_adc_mv(struct apple_tristar *ts)
{
	unsigned int raw, q16;
	u8 result[2];
	int status, ret;

	if (ts->adc_poisoned)
		return -EIO;
	/* Observe the completion latch LOW before starting a new sample. */
	ret = read_poll_timeout(apple_tristar_pmic_read, status,
				status < 0 || !(status & 1), 1000, 20000,
				false, ts, D2365_ADC_DONE);
	if (ret || status < 0)
		return ret ?: status;
	ret = regmap_write(ts->pmic, D2365_ADC_START, 1);
	if (ret)
		goto poison;
	ret = read_poll_timeout(apple_tristar_pmic_read, status,
				status < 0 || (status & 1), 1000, 1000000,
				false, ts, D2365_ADC_DONE);
	if (ret || status < 0) {
		ret = ret ?: status;
		goto poison;
	}
	ret = regmap_bulk_read(ts->pmic, D2365_ADC_RESULT, result, sizeof(result));
	if (ret)
		return ret;
	if (result[0] & 0x80)
		return -ENODATA;
	raw = (result[0] & 0x0f) | (result[1] << 4);
	/* Exact positive Q16-volts conversion, then the source's integer mV. */
	q16 = div_u64((40ULL << 16) + raw * 5000ULL * 16, 1000);
	return (q16 * 1000U) >> 16;
poison:
	/* A late completion must never be mistaken for the next pin's sample. */
	ts->adc_poisoned = true;
	return ret;
}

static int apple_tristar_pin_mv(struct apple_tristar *ts, bool dm)
{
	int ret, restore;
	u8 route;

	ret = apple_tristar_read(ts->client, TRISTAR_DX_CONTROL);
	if (ret < 0)
		return ret;
	ts->saved_route = ret;
	if ((ret & 7) == 1)
		route = (ret & 0xf0) | (dm ? 0x0d : 0x0c);
	else if ((ret & 0x70) == 0x10)
		route = (ret & 0x0f) | (dm ? 0xd0 : 0xc0);
	else
		return -EOPNOTSUPP;
	ts->restore_pending = true;
	ret = apple_tristar_write_checked(ts, route);
	if (!ret) {
		usleep_range(5000, 6000);
		ret = apple_tristar_adc_mv(ts);
	}
	restore = apple_tristar_restore_route(ts);
	return restore ?: ret;
}

static int apple_tristar_brick_current(int dp_mv, int dm_mv)
{
	static const int limits[] = { 500, 1000, 1000, 2100, 2400, 2400, 1000, 1000, 1000 };
	int dp, dm;

	if (dp_mv < 0 || dm_mv < 0 || dp_mv > 4000 || dm_mv > 4000)
		return -ERANGE;
	if (dp_mv < 1500 || dm_mv < 1500)
		return 0;
	dp = dp_mv >= 2880 ? 2 : dp_mv >= 2200 ? 1 : 0;
	dm = dm_mv >= 2880 ? 2 : dm_mv >= 2200 ? 1 : 0;
	return limits[3 * dp + dm] * 1000;
}

static void apple_tristar_classify(struct apple_tristar *ts)
{
	struct usb_gadget_current_budget budget;
	int dp, dm, ret;

	if (ts->brick_sampled || ts->adc_poisoned || !ts->online || ts->error ||
	    time_before(jiffies, ts->classify_after))
		return;
	ret = usb_gadget_get_current_budget(ts->usb_node, &budget);
	/* Never divert active USB traffic to the analog monitor. */
	if (ret || budget.connected || budget.host_grant)
		return;
	ts->brick_sampled = true;
	dp = apple_tristar_pin_mv(ts, false);
	if (dp < 0)
		goto fail;
	ret = usb_gadget_get_current_budget(ts->usb_node, &budget);
	if (ret || budget.connected || budget.host_grant)
		return;
	dm = apple_tristar_pin_mv(ts, true);
	if (dm < 0)
		goto fail;
	ret = usb_gadget_get_current_budget(ts->usb_node, &budget);
	if (ret || budget.connected || budget.host_grant)
		return;
	ret = apple_tristar_read(ts->client, TRISTAR_VBUS);
	if (ret < 0 || (ret & 3) != 3 ||
	    ts->seen_generation != atomic_read(&ts->reset_generation))
		return;
	ret = apple_tristar_brick_current(dp, dm);
	if (ret < 0)
		goto fail;
	ts->brick_ua = ret;
	dev_info(&ts->client->dev, "Apple brick signature: D+ %d mV D- %d mV, budget %d mA\n",
		 dp, dm, ret / 1000);
	return;
fail:
	dev_warn(&ts->client->dev, "brick detection unavailable; no elevated analog allowance\n");
}

static int apple_tristar_budget(const struct usb_gadget_current_budget *budget,
				int brick_ua, int *type)
{
	*type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	if (!budget)
		return 100000;
	if (budget->connected) {
		*type = POWER_SUPPLY_USB_TYPE_SDP;
		if (budget->suspended)
			return 0;
		if (budget->host_grant)
			return min(budget->current_ma, 2400U) * 1000;
		if (budget->configured)
			return min(budget->current_ma, 500U) * 1000;
		return min(budget->current_ma, 100U) * 1000;
	}
	/* A proven brick is independent of USB suspend on an unenumerated port. */
	if (brick_ua > 0) {
		*type = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
		return min(brick_ua, 2400000);
	}
	return 500000;
}

static int apple_tristar_update(struct apple_tristar *ts)
{
	struct usb_gadget_current_budget budget;
	int value, error, online, current_ua = 0, type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int generation, old_current, old_type;
	bool changed;

	mutex_lock(&ts->lock);
	if (ts->sleeping || ts->stopping) {
		mutex_unlock(&ts->lock);
		return -EHOSTDOWN;
	}
	old_current = ts->current_ua;
	old_type = ts->usb_type;
	generation = atomic_read(&ts->reset_generation);
	if (generation != ts->seen_generation) {
		ts->seen_generation = generation;
		ts->brick_sampled = false;
		ts->brick_ua = 0;
		ts->classify_after = jiffies + msecs_to_jiffies(1000);
	}
	value = apple_tristar_read(ts->client, TRISTAR_REVISION);
	error = value < 0 ? value : value == TRISTAR_J71T_REVISION ? 0 : -ENODEV;
	if (!error) {
		value = apple_tristar_read(ts->client, TRISTAR_VBUS);
		if (value < 0)
			error = value;
	}
	/* IC getVBUSDetect(strict=1): both low bits, never orientation bits. */
	online = !error && (value & 3) == 3;
	if (!online) {
		ts->brick_sampled = false;
		ts->brick_ua = 0;
		ts->classify_after = jiffies + msecs_to_jiffies(1000);
	}
	if (ts->restore_pending) {
		int restore = apple_tristar_restore_route(ts);

		if (restore)
			error = restore;
	}
	if (online && !error) {
		if (usb_gadget_get_current_budget(ts->usb_node, &budget)) {
			ts->brick_ua = 0;
			current_ua = apple_tristar_budget(NULL, 0, &type);
		} else {
			if (budget.connected)
				ts->brick_ua = 0;
			current_ua = apple_tristar_budget(&budget, ts->brick_ua, &type);
		}
	}
	changed = ts->error != error || ts->online != online;
	changed |= old_current != current_ua || old_type != type;
	ts->error = error;
	ts->online = online;
	ts->current_ua = current_ua;
	ts->usb_type = type;
	mutex_unlock(&ts->lock);
	if (changed && ts->psy)
		power_supply_changed(ts->psy);
	return error;
}

static int
apple_tristar_get_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   union power_supply_propval *value)
{
	struct apple_tristar *ts = power_supply_get_drvdata(psy);
	int ret;

	/* Policy consumers get a live read rather than a stale cached cable. */
	ret = apple_tristar_update(ts);
	mutex_lock(&ts->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		value->intval = ts->online;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		value->intval = ts->current_ua;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		value->intval = ts->usb_type;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		value->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&ts->lock);
	return ret;
}

static void apple_tristar_poll(struct work_struct *work)
{
	struct apple_tristar *ts = container_of(to_delayed_work(work),
						       struct apple_tristar, work);

	apple_tristar_update(ts);
	mutex_lock(&ts->lock);
	if (!ts->stopping && !ts->sleeping)
		apple_tristar_classify(ts);
	mutex_unlock(&ts->lock);
	apple_tristar_update(ts);
	mutex_lock(&ts->lock);
	if (!ts->stopping && !ts->sleeping)
		mod_delayed_work(system_power_efficient_wq, &ts->work, msecs_to_jiffies(250));
	mutex_unlock(&ts->lock);
}

static int apple_tristar_usb_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct apple_tristar *ts = container_of(nb, struct apple_tristar, usb_notifier);
	const struct usb_gadget_current_budget *budget = data;

	if (budget->of_node != ts->usb_node || READ_ONCE(ts->stopping) ||
	    READ_ONCE(ts->sleeping))
		return NOTIFY_DONE;
	if (!budget->connected)
		atomic_inc(&ts->reset_generation);
	mod_delayed_work(system_power_efficient_wq, &ts->work, 0);
	return NOTIFY_OK;
}

static int apple_tristar_quiesce(struct apple_tristar *ts)
{
	int attempt, ret = 0;

	mutex_lock(&ts->lock);
	WRITE_ONCE(ts->sleeping, true);
	mutex_unlock(&ts->lock);
	if (ts->notifier_registered) {
		usb_gadget_unregister_current_notifier(&ts->usb_notifier);
		ts->notifier_registered = false;
	}
	cancel_delayed_work_sync(&ts->work);
	mutex_lock(&ts->lock);
	for (attempt = 0; attempt < 3; attempt++) {
		ret = apple_tristar_restore_route(ts);
		if (!ret)
			break;
		msleep(20);
	}
	ts->brick_ua = 0;
	ts->brick_sampled = false;
	mutex_unlock(&ts->lock);
	if (ret)
		dev_crit(&ts->client->dev, "Tristar route restore failed at quiesce: %d\n", ret);
	return ret;
}

static void apple_tristar_stop(void *data)
{
	struct apple_tristar *ts = data;

	mutex_lock(&ts->lock);
	WRITE_ONCE(ts->stopping, true);
	mutex_unlock(&ts->lock);
	apple_tristar_quiesce(ts);
}

static int apple_tristar_resume(struct device *dev)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);
	int ret;

	ret = usb_gadget_register_current_notifier(&ts->usb_notifier);
	if (ret && ret != -EOPNOTSUPP)
		return ret;
	ts->notifier_registered = !ret;
	mutex_lock(&ts->lock);
	ts->classify_after = jiffies + msecs_to_jiffies(1000);
	WRITE_ONCE(ts->sleeping, false);
	mod_delayed_work(system_power_efficient_wq, &ts->work, 0);
	mutex_unlock(&ts->lock);
	return 0;
}

static int apple_tristar_suspend(struct device *dev)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);
	int ret = apple_tristar_quiesce(ts);

	if (ret)
		apple_tristar_resume(dev);
	return ret;
}

static void apple_tristar_shutdown(struct i2c_client *client)
{
	apple_tristar_stop(i2c_get_clientdata(client));
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_tristar_pm_ops,
			       apple_tristar_suspend, apple_tristar_resume);

static void apple_tristar_put_device(void *data)
{
	put_device(data);
}

static void apple_tristar_put_node(void *data)
{
	of_node_put(data);
}

static const enum power_supply_property apple_tristar_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc apple_tristar_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) | BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
	.properties = apple_tristar_properties,
	.num_properties = ARRAY_SIZE(apple_tristar_properties),
	.get_property = apple_tristar_get_property,
};

static int apple_tristar_probe(struct i2c_client *client)
{
	struct power_supply_config config = {};
	struct apple_tristar *ts;
	struct device_node *node;
	struct i2c_client *pmic_client;
	struct platform_device *usb_device;
	struct device_link *link;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE))
		return -EOPNOTSUPP;
	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->usb_node = of_parse_phandle(client->dev.of_node, "usb-controller", 0);
	if (!ts->usb_node)
		return -EINVAL;
	ret = devm_add_action_or_reset(&client->dev, apple_tristar_put_node, ts->usb_node);
	if (ret)
		return ret;
	node = of_parse_phandle(client->dev.of_node, "apple,pmic", 0);
	if (!node)
		return -EINVAL;
	pmic_client = of_find_i2c_device_by_node(node);
	of_node_put(node);
	if (!pmic_client)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(&client->dev, apple_tristar_put_device, &pmic_client->dev);
	if (ret)
		return ret;
	ts->pmic = dev_get_regmap(&pmic_client->dev, NULL);
	if (!ts->pmic)
		return -EPROBE_DEFER;
	if (!device_link_add(&client->dev, &pmic_client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	usb_device = of_find_device_by_node(ts->usb_node);
	if (!usb_device)
		return -EPROBE_DEFER;
	link = device_link_add(&client->dev, &usb_device->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	ret = link ? 0 : -ENOMEM;
	put_device(&usb_device->dev);
	if (ret)
		return ret;
	atomic_set(&ts->reset_generation, 0);
	/* Allow old iOS work to quiesce; no first-boot ADC idle bit is known. */
	ts->classify_after = jiffies + msecs_to_jiffies(2000);
	ret = devm_mutex_init(&client->dev, &ts->lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&ts->work, apple_tristar_poll);
	ret = apple_tristar_update(ts);
	if (ret)
		return dev_err_probe(&client->dev, ret, "CBTL1610 revision/VBUS read failed\n");
	config.drv_data = ts;
	config.fwnode = dev_fwnode(&client->dev);
	ts->psy = devm_power_supply_register(&client->dev, &apple_tristar_desc, &config);
	if (IS_ERR(ts->psy))
		return dev_err_probe(&client->dev, PTR_ERR(ts->psy), "USB supply registration\n");
	ts->usb_notifier.notifier_call = apple_tristar_usb_event;
	ret = usb_gadget_register_current_notifier(&ts->usb_notifier);
	if (ret && ret != -EOPNOTSUPP)
		return ret;
	ts->notifier_registered = !ret;
	ret = devm_add_action_or_reset(&client->dev, apple_tristar_stop, ts);
	if (ret)
		return ret;
	mod_delayed_work(system_power_efficient_wq, &ts->work, 0);
	dev_info(&client->dev, "CBTL1610 rev 0x8a: guarded Apple source detection\n");
	return 0;
}

static const struct of_device_id apple_tristar_of_match[] = {
	{ .compatible = "apple,j71t-cbtl1610-power" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_tristar_of_match);

static struct i2c_driver apple_tristar_driver = {
	.driver = {
		.name = "apple-tristar-power",
		.of_match_table = apple_tristar_of_match,
		.pm = pm_sleep_ptr(&apple_tristar_pm_ops),
	},
	.probe = apple_tristar_probe,
	.shutdown = apple_tristar_shutdown,
};
module_i2c_driver(apple_tristar_driver);
MODULE_DESCRIPTION("Apple J71t Tristar source-current detection");
MODULE_LICENSE("GPL");
