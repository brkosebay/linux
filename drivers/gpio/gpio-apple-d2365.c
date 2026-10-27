// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for the Apple/Dialog D2365 PMIC.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define D2365_GPIO_COUNT		24
#define D2365_GPIO_SPLIT		18
#define D2365_GPIO_CFG_0_17	0x900
#define D2365_GPIO_CFG_18_23	0x8bd
#define D2365_GPIO_INPUT0	0x186

/*
 * PMIC events, one bit each. iOS fetches and discards them by reading 30
 * bytes at 0x140 (clear on read); 0x180 is the live status and 0x1c0 the
 * interrupt mask (set = masked), all ones after its probe. GPIO line N is
 * event 0x30 + N.
 */
#define D2365_EVENTS		0x140
#define D2365_IRQ_MASK		0x1c0
#define D2365_EVENT_BYTES	30
#define D2365_EVENT_COUNT	(D2365_EVENT_BYTES * 8)
#define D2365_GPIO_EVENT	0x30
#define D2365_GPIO_IRQ_MASK	(D2365_IRQ_MASK + D2365_GPIO_EVENT / 8)
#define D2365_HALL_LINES	(BIT(0) | BIT(10))
#define D2365_IRQ_BURST		64
#define D2365_IRQ_RETRIES	5

#define D2365_GPIO_VALUE		BIT(0)
#define D2365_GPIO_INPUT		BIT(3)
#define D2365_GPIO_FUNCTION	BIT(4)
#define D2365_GPIO_MODE_MASK	(D2365_GPIO_VALUE | D2365_GPIO_INPUT | \
				 D2365_GPIO_FUNCTION)

struct d2365_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
	struct device *pmu;
	struct irq_domain *domain;
	/* Serializes the masks, the event thread and the suspend gate. */
	struct mutex irq_lock;
	struct delayed_work irq_retry;
	u8 boot_mask[D2365_EVENT_BYTES];
	unsigned long irq_enabled;	/* lines unmasked while awake */
	unsigned long wake_enabled;	/* lines unmasked while asleep */
	int wake_delta;			/* parent wake references to apply */
	unsigned long irq_window;
	unsigned int irq_burst;
	unsigned int retries;
	int parent_irq;
	bool irq_ready;
	bool suspended;
	bool irq_held;
	bool removing;
};

/* Unmask the GPIO events of @lines; every other GPIO event stays masked. */
static int d2365_irq_sync(struct d2365_gpio *gpio, unsigned long lines)
{
	u8 mask[2];

	lines &= D2365_HALL_LINES;
	mask[0] = ~lines & 0xff;
	mask[1] = ~(lines >> 8) & 0xff;
	return regmap_bulk_write(gpio->regmap, D2365_GPIO_IRQ_MASK, mask,
				 sizeof(mask));
}

static void d2365_irq_bus_lock(struct irq_data *data)
{
	struct d2365_gpio *gpio = irq_data_get_irq_chip_data(data);

	mutex_lock(&gpio->irq_lock);
}

static void d2365_irq_bus_sync_unlock(struct irq_data *data)
{
	struct d2365_gpio *gpio = irq_data_get_irq_chip_data(data);
	int delta = gpio->wake_delta;
	int ret;

	gpio->wake_delta = 0;
	/* No bus access between our suspend and resume callbacks. */
	if (!gpio->suspended) {
		ret = d2365_irq_sync(gpio, READ_ONCE(gpio->irq_enabled));
		if (ret)
			dev_err_ratelimited(gpio->chip.parent,
					    "hall IRQ mask write failed: %d\n", ret);
	}
	mutex_unlock(&gpio->irq_lock);

	/*
	 * A cascaded domain has no hierarchical parent_data for
	 * irq_chip_set_wake_parent(). Count the wake references on the shared
	 * AP GPIO here, outside the child descriptor lock, so that
	 * suspend_device_irqs() leaves that line armed.
	 */
	for (; delta > 0; delta--) {
		ret = irq_set_irq_wake(gpio->parent_irq, 1);
		if (ret)
			dev_err(gpio->chip.parent,
				"cannot arm the event IRQ for wake: %d\n", ret);
	}
	for (; delta < 0; delta++)
		irq_set_irq_wake(gpio->parent_irq, 0);
}

static void d2365_irq_mask(struct irq_data *data)
{
	struct d2365_gpio *gpio = irq_data_get_irq_chip_data(data);

	clear_bit(data->hwirq - D2365_GPIO_EVENT, &gpio->irq_enabled);
}

static void d2365_irq_unmask(struct irq_data *data)
{
	struct d2365_gpio *gpio = irq_data_get_irq_chip_data(data);

	set_bit(data->hwirq - D2365_GPIO_EVENT, &gpio->irq_enabled);
}

static int d2365_irq_set_type(struct irq_data *data, unsigned int type)
{
	/* The inherited event configuration reports both transitions. */
	return type == IRQ_TYPE_NONE || type == IRQ_TYPE_EDGE_BOTH ? 0 : -EINVAL;
}

/* Called with the bus lock held; the parent follows in sync_unlock. */
static int d2365_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct d2365_gpio *gpio = irq_data_get_irq_chip_data(data);
	unsigned int line = data->hwirq - D2365_GPIO_EVENT;

	if (on && !test_bit(line, &gpio->wake_enabled)) {
		__set_bit(line, &gpio->wake_enabled);
		gpio->wake_delta++;
	} else if (!on && test_bit(line, &gpio->wake_enabled)) {
		__clear_bit(line, &gpio->wake_enabled);
		gpio->wake_delta--;
	}
	return 0;
}

static const struct irq_chip d2365_irq_chip = {
	.name = "d2365-hall",
	.irq_bus_lock = d2365_irq_bus_lock,
	.irq_bus_sync_unlock = d2365_irq_bus_sync_unlock,
	.irq_mask = d2365_irq_mask,
	.irq_unmask = d2365_irq_unmask,
	.irq_set_type = d2365_irq_set_type,
	.irq_set_wake = d2365_irq_set_wake,
};

static int d2365_irq_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	struct d2365_gpio *gpio = domain->host_data;

	/* Other events need their own evidence and consumer before use. */
	if (hwirq < D2365_GPIO_EVENT ||
	    hwirq >= D2365_GPIO_EVENT + D2365_GPIO_COUNT ||
	    !(BIT(hwirq - D2365_GPIO_EVENT) & D2365_HALL_LINES))
		return -EINVAL;

	irq_set_chip_data(irq, gpio);
	irq_set_chip(irq, &d2365_irq_chip);
	irq_set_nested_thread(irq, 1);
	irq_set_parent(irq, gpio->parent_irq);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops d2365_irq_domain_ops = {
	.map = d2365_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

/* Called with irq_lock held, including from the threaded handler. */
static void d2365_irq_hold(struct d2365_gpio *gpio, bool retry)
{
	if (!gpio->irq_held) {
		disable_irq_nosync(gpio->parent_irq);
		gpio->irq_held = true;
	}
	if (!retry)
		return;
	if (gpio->retries < D2365_IRQ_RETRIES) {
		gpio->retries++;
		mod_delayed_work(system_dfl_wq, &gpio->irq_retry,
				 msecs_to_jiffies(100 << gpio->retries));
	} else {
		dev_err(gpio->chip.parent,
			"hall event IRQ held until the next resume\n");
	}
}

static void d2365_irq_retry(struct work_struct *work)
{
	struct d2365_gpio *gpio = container_of(to_delayed_work(work),
						struct d2365_gpio, irq_retry);

	mutex_lock(&gpio->irq_lock);
	if (!gpio->suspended && !gpio->removing && gpio->irq_held) {
		gpio->irq_held = false;
		enable_irq(gpio->parent_irq);
	}
	mutex_unlock(&gpio->irq_lock);
}

static irqreturn_t d2365_irq_thread(int irq, void *data)
{
	struct d2365_gpio *gpio = data;
	u8 events[D2365_EVENT_BYTES];
	unsigned long pending;
	unsigned int line, child;
	int ret;

	mutex_lock(&gpio->irq_lock);
	if (gpio->removing)
		goto unlock;
	if (gpio->suspended) {
		/*
		 * resume_device_irqs() runs before our I2C ancestors resume.
		 * Leave the event latched and the level IRQ held until
		 * ->resume. A wake event in the window before
		 * suspend_device_irqs() must abort the transition, as the
		 * armed IRQ does afterwards.
		 */
		if (gpio->wake_enabled)
			pm_wakeup_hard_event(gpio->pmu);
		d2365_irq_hold(gpio, false);
		goto unlock;
	}

	/* Bound a stuck line or a bouncing sensor. */
	if (time_after(jiffies, gpio->irq_window + HZ)) {
		gpio->irq_window = jiffies;
		gpio->irq_burst = 0;
	}
	if (++gpio->irq_burst > D2365_IRQ_BURST) {
		ret = -ELOOP;
		goto hold;
	}

	/* Reading the events clears them and releases the line. */
	ret = regmap_bulk_read(gpio->regmap, D2365_EVENTS, events,
			       sizeof(events));
	if (ret)
		goto hold;
	gpio->retries = 0;

	pending = events[D2365_GPIO_EVENT / 8] |
		  events[D2365_GPIO_EVENT / 8 + 1] << 8;
	dev_dbg(gpio->chip.parent, "events %*phN\n", D2365_EVENT_BYTES, events);
	pending &= READ_ONCE(gpio->irq_enabled) & D2365_HALL_LINES;
	/*
	 * Nothing latched for us (a debug read may have cleared it): the
	 * consumers read the line level again, so let them all resync.
	 */
	if (!pending)
		pending = READ_ONCE(gpio->irq_enabled) & D2365_HALL_LINES;
	mutex_unlock(&gpio->irq_lock);

	/* A child may mask its IRQ, which takes the bus lock. */
	for_each_set_bit(line, &pending, D2365_GPIO_COUNT) {
		child = irq_find_mapping(gpio->domain, D2365_GPIO_EVENT + line);
		if (child)
			handle_nested_irq(child);
	}
	return IRQ_HANDLED;

hold:
	dev_err_ratelimited(gpio->chip.parent,
			    "event read failed or storm (%d), holding the IRQ, retry %u/%u\n",
			    ret, gpio->retries + 1, D2365_IRQ_RETRIES);
	d2365_irq_hold(gpio, true);
unlock:
	mutex_unlock(&gpio->irq_lock);
	return IRQ_HANDLED;
}

static int d2365_gpio_suspend(struct device *dev)
{
	struct d2365_gpio *gpio = dev_get_drvdata(dev);
	int ret;

	if (!gpio->irq_ready)
		return 0;

	cancel_delayed_work_sync(&gpio->irq_retry);
	mutex_lock(&gpio->irq_lock);
	/* A failed event path must not keep a closed iPad awake. */
	if (gpio->irq_held)
		dev_warn(dev, "hall event IRQ held, only the buttons can wake this sleep\n");
	/* Only the wake-enabled halls may pull the line while asleep. */
	ret = d2365_irq_sync(gpio, gpio->wake_enabled);
	if (ret)
		dev_warn(dev, "hall wake mask write failed: %d\n", ret);
	gpio->suspended = true;
	mutex_unlock(&gpio->irq_lock);
	/* A thread that passed the gate before it closed has finished now. */
	synchronize_irq(gpio->parent_irq);
	return 0;
}

static int d2365_gpio_resume(struct device *dev)
{
	struct d2365_gpio *gpio = dev_get_drvdata(dev);
	int ret;

	if (!gpio->irq_ready)
		return 0;

	/* Our I2C ancestors have completed their normal resume callbacks. */
	mutex_lock(&gpio->irq_lock);
	gpio->suspended = false;
	gpio->retries = 0;
	ret = d2365_irq_sync(gpio, READ_ONCE(gpio->irq_enabled));
	if (ret)
		dev_warn(dev, "hall IRQ mask write failed: %d\n", ret);
	/* An event that arrived while gated is still latched. */
	if (gpio->irq_held) {
		gpio->irq_held = false;
		enable_irq(gpio->parent_irq);
	}
	mutex_unlock(&gpio->irq_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(d2365_gpio_pm_ops, d2365_gpio_suspend,
				d2365_gpio_resume);

static void d2365_irq_restore_mask(void *data)
{
	struct d2365_gpio *gpio = data;

	regmap_bulk_write(gpio->regmap, D2365_IRQ_MASK, gpio->boot_mask,
			  sizeof(gpio->boot_mask));
}

static void d2365_irq_remove_domain(void *data)
{
	struct d2365_gpio *gpio = data;
	unsigned int line, irq;
	unsigned long lines = D2365_HALL_LINES;

	for_each_set_bit(line, &lines, D2365_GPIO_COUNT) {
		irq = irq_find_mapping(gpio->domain, D2365_GPIO_EVENT + line);
		if (irq)
			irq_dispose_mapping(irq);
	}
	irq_domain_remove(gpio->domain);
}

static void d2365_irq_stop(void *data)
{
	struct d2365_gpio *gpio = data;

	mutex_lock(&gpio->irq_lock);
	gpio->removing = true;
	gpio->irq_ready = false;
	mutex_unlock(&gpio->irq_lock);
	cancel_delayed_work_sync(&gpio->irq_retry);
	free_irq(gpio->parent_irq, gpio);
}

static int d2365_irq_setup(struct d2365_gpio *gpio)
{
	struct device *dev = gpio->chip.parent;
	u8 buf[D2365_EVENT_BYTES];
	int ret;

	if (!device_property_read_bool(gpio->pmu, "interrupt-controller"))
		return -EINVAL;
	gpio->parent_irq = of_irq_get(gpio->pmu->of_node, 0);
	if (gpio->parent_irq < 0)
		return gpio->parent_irq;
	if (!gpio->parent_irq)
		return -ENXIO;

	ret = devm_mutex_init(dev, &gpio->irq_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&gpio->irq_retry, d2365_irq_retry);

	/* What the boot chain armed and latched is the evidence to keep. */
	ret = regmap_bulk_read(gpio->regmap, D2365_IRQ_MASK, gpio->boot_mask,
			       sizeof(gpio->boot_mask));
	if (ret)
		return ret;
	dev_info(dev, "boot event mask %*phN\n", D2365_EVENT_BYTES,
		 gpio->boot_mask);

	/* As iOS does at probe: mask all, unmask only what has a consumer. */
	memset(buf, 0xff, sizeof(buf));
	ret = regmap_bulk_write(gpio->regmap, D2365_IRQ_MASK, buf, sizeof(buf));
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, d2365_irq_restore_mask, gpio);
	if (ret)
		return ret;

	gpio->domain = irq_domain_create_linear(dev_fwnode(gpio->pmu),
						D2365_EVENT_COUNT,
						&d2365_irq_domain_ops, gpio);
	if (!gpio->domain)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, d2365_irq_remove_domain, gpio);
	if (ret)
		return ret;

	/* Drop stale events so the line is released before it goes live. */
	ret = regmap_bulk_read(gpio->regmap, D2365_EVENTS, buf, sizeof(buf));
	if (ret)
		return ret;
	dev_info(dev, "boot events %*phN\n", D2365_EVENT_BYTES, buf);

	ret = request_threaded_irq(gpio->parent_irq, NULL, d2365_irq_thread,
				   IRQF_ONESHOT, "d2365-events", gpio);
	if (ret)
		return ret;
	gpio->irq_ready = true;
	return devm_add_action_or_reset(dev, d2365_irq_stop, gpio);
}

static int d2365_irq_probe(struct d2365_gpio *gpio)
{
	struct device *dev = gpio->chip.parent;
	void *group;
	int ret;

	/* Old DTBs and the polled default never touch the event registers. */
	if (!device_property_read_bool(dev, "apple,enable-hall-irqs"))
		return 0;

	group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;
	ret = d2365_irq_setup(gpio);
	if (ret) {
		gpio->irq_ready = false;
		devres_release_group(dev, group);
		gpio->domain = NULL;
		return ret;
	}
	devres_remove_group(dev, group);
	dev_info(dev, "hall events 0x30/0x3a on IRQ %d\n", gpio->parent_irq);
	return 0;
}

static unsigned int d2365_gpio_config_reg(unsigned int offset)
{
	if (offset < D2365_GPIO_SPLIT)
		return D2365_GPIO_CFG_0_17 + 2 * offset;

	return D2365_GPIO_CFG_18_23 + 6 * offset;
}

static int d2365_gpio_get_direction(struct gpio_chip *chip,
				    unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, d2365_gpio_config_reg(offset), &value);
	if (ret)
		return ret;
	if (value & D2365_GPIO_FUNCTION)
		return -EINVAL;

	return value & D2365_GPIO_INPUT ? GPIO_LINE_DIRECTION_IN :
					  GPIO_LINE_DIRECTION_OUT;
}

static int d2365_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);
	unsigned int reg = d2365_gpio_config_reg(offset);
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, reg, &value);
	if (ret)
		return ret;

	/*
	 * A line the PMIC's event logic owns also feeds its wake logic (the
	 * Smart Cover halls, for one): never take it over, and say why.
	 */
	if (value & D2365_GPIO_FUNCTION) {
		dev_warn(chip->parent,
			 "line %u belongs to a PMIC function (config 0x%02x), not using it as a GPIO input\n",
			 offset, value);
		return -EBUSY;
	}

	dev_info(chip->parent, "line %u config 0x%02x (%s)\n", offset, value,
		 value & D2365_GPIO_INPUT ? "input" : "output, switching to input");

	/* Already an input: reading it must not write anything. */
	if (value & D2365_GPIO_INPUT)
		return 0;

	return regmap_update_bits(gpio->regmap, reg, D2365_GPIO_MODE_MASK,
				  D2365_GPIO_INPUT);
}

static int d2365_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);

	return regmap_update_bits(gpio->regmap,
				  d2365_gpio_config_reg(offset),
				  D2365_GPIO_MODE_MASK,
				  value ? D2365_GPIO_VALUE : 0);
}

static int d2365_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, d2365_gpio_config_reg(offset), &value);
	if (ret)
		return ret;
	if (!(value & D2365_GPIO_INPUT))
		return !!(value & D2365_GPIO_VALUE);

	ret = regmap_read(gpio->regmap, D2365_GPIO_INPUT0 + offset / 8,
			  &value);
	if (ret)
		return ret;

	return !!(value & BIT(offset % 8));
}

static int d2365_gpio_set(struct gpio_chip *chip, unsigned int offset,
			  int value)
{
	struct d2365_gpio *gpio = gpiochip_get_data(chip);

	return regmap_update_bits(gpio->regmap,
				  d2365_gpio_config_reg(offset),
				  D2365_GPIO_VALUE,
				  value ? D2365_GPIO_VALUE : 0);
}

static int d2365_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct d2365_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	platform_set_drvdata(pdev, gpio);

	gpio->regmap = dev_get_regmap(dev->parent, NULL);
	if (!gpio->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");
	gpio->pmu = dev->parent;

	gpio->chip.label = "d2365-pmic-gpio";
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = D2365_GPIO_COUNT;
	gpio->chip.can_sleep = true;
	gpio->chip.get_direction = d2365_gpio_get_direction;
	gpio->chip.direction_input = d2365_gpio_direction_input;
	gpio->chip.direction_output = d2365_gpio_direction_output;
	gpio->chip.get = d2365_gpio_get;
	gpio->chip.set = d2365_gpio_set;

	ret = devm_gpiochip_add_data(dev, &gpio->chip, gpio);
	if (ret)
		return ret;

	/* The GPIOs stay usable without the event path; only defer. */
	ret = d2365_irq_probe(gpio);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(dev, "hall event IRQs unavailable: %d\n", ret);
	return 0;
}

static const struct of_device_id d2365_gpio_of_match[] = {
	{ .compatible = "apple,d2365-pmic-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_gpio_of_match);

static struct platform_driver d2365_gpio_driver = {
	.probe = d2365_gpio_probe,
	.driver = {
		.name = "apple-d2365-pmic-gpio",
		.of_match_table = d2365_gpio_of_match,
		.pm = pm_sleep_ptr(&d2365_gpio_pm_ops),
	},
};
module_platform_driver(d2365_gpio_driver);

MODULE_DESCRIPTION("Apple D2365 PMIC GPIO driver");
MODULE_LICENSE("GPL");
