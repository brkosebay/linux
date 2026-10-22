// SPDX-License-Identifier: GPL-2.0-only
/* Read-only J71t CBTL1610 VBUS detection, reconstructed from AppleTriStar. */
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>

#define TRISTAR_VBUS 0x0c
#define TRISTAR_REVISION 0x0f
#define TRISTAR_J71T_REVISION 0x8a

struct apple_tristar {
	struct i2c_client *client;
	struct power_supply *psy;
	struct delayed_work work;
	/* Serializes register reads and protects the cached source state. */
	struct mutex lock;
	int online;
	int error;
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

static int apple_tristar_update(struct apple_tristar *ts)
{
	int value, error, online;
	bool changed;

	mutex_lock(&ts->lock);
	value = apple_tristar_read(ts->client, TRISTAR_REVISION);
	error = value < 0 ? value : value == TRISTAR_J71T_REVISION ? 0 : -ENODEV;
	if (!error) {
		value = apple_tristar_read(ts->client, TRISTAR_VBUS);
		if (value < 0)
			error = value;
	}
	/* IC getVBUSDetect(strict=1): both low bits, never orientation bits. */
	online = !error && (value & 3) == 3;
	changed = ts->error != error || ts->online != online;
	ts->error = error;
	ts->online = online;
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

	if (prop != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;
	/* Policy consumers get a live read rather than a stale cached cable. */
	ret = apple_tristar_update(ts);
	mutex_lock(&ts->lock);
	value->intval = ts->online;
	mutex_unlock(&ts->lock);
	return ret;
}

static void apple_tristar_poll(struct work_struct *work)
{
	struct apple_tristar *ts = container_of(to_delayed_work(work),
						       struct apple_tristar, work);

	apple_tristar_update(ts);
	mod_delayed_work(system_power_efficient_wq, &ts->work, msecs_to_jiffies(250));
}

static const enum power_supply_property apple_tristar_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc apple_tristar_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = apple_tristar_properties,
	.num_properties = ARRAY_SIZE(apple_tristar_properties),
	.get_property = apple_tristar_get_property,
};

static int apple_tristar_probe(struct i2c_client *client)
{
	struct power_supply_config config = {};
	struct apple_tristar *ts;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE))
		return -EOPNOTSUPP;
	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	ret = devm_mutex_init(&client->dev, &ts->lock);
	if (ret)
		return ret;
	ret = apple_tristar_update(ts);
	if (ret)
		return dev_err_probe(&client->dev, ret, "CBTL1610 revision/VBUS read failed\n");
	config.drv_data = ts;
	config.fwnode = dev_fwnode(&client->dev);
	ts->psy = devm_power_supply_register(&client->dev, &apple_tristar_desc, &config);
	if (IS_ERR(ts->psy))
		return dev_err_probe(&client->dev, PTR_ERR(ts->psy), "USB supply registration\n");
	ret = devm_delayed_work_autocancel(&client->dev, &ts->work, apple_tristar_poll);
	if (ret)
		return ret;
	mod_delayed_work(system_power_efficient_wq, &ts->work, 0);
	dev_info(&client->dev, "CBTL1610 rev 0x8a: read-only VBUS reporting\n");
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
	},
	.probe = apple_tristar_probe,
};
module_i2c_driver(apple_tristar_driver);
MODULE_DESCRIPTION("Read-only Apple J71t Tristar USB presence");
MODULE_LICENSE("GPL");
