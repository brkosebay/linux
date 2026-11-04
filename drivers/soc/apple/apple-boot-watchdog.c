// SPDX-License-Identifier: GPL-2.0-only
/* Opt-in j71t boot deadline. Never programs the SoC reset watchdog. */
#include <linux/crash_dump.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/timer.h>

static unsigned int timeout;
static bool initialized;
static bool supported;
static struct timer_list boot_timer;

static void apple_boot_expired(struct timer_list *timer)
{
	panic("j71t boot deadline expired after %u seconds; inspect retained probe breadcrumbs",
	      timeout);
}

static int apple_boot_timeout_set(const char *val, const struct kernel_param *kp)
{
	unsigned int seconds;
	int ret;

	ret = kstrtouint(val, 0, &seconds);
	if (ret)
		return ret;
	if (seconds && (seconds < 30 || seconds > 600))
		return -EINVAL;
	if (initialized) {
		if (seconds && !supported)
			return -EOPNOTSUPP;
		timer_delete_sync(&boot_timer);
	}
	WRITE_ONCE(timeout, seconds);
	if (initialized) {
		if (seconds) {
			pr_warn("j71t boot deadline rearmed for %u seconds by userspace\n",
				seconds);
			mod_timer(&boot_timer, jiffies + seconds * HZ);
		} else {
			pr_info("j71t boot deadline disarmed by userspace\n");
		}
	}
	return 0;
}

static const struct kernel_param_ops apple_boot_timeout_ops = {
	.set = apple_boot_timeout_set,
	.get = param_get_uint,
};
module_param_cb(timeout, &apple_boot_timeout_ops, &timeout, 0600);
MODULE_PARM_DESC(timeout, "j71t boot deadline: 0=off, 30..600 seconds; write 0 to disarm");

static int __init apple_boot_watchdog_init(void)
{
	timer_setup(&boot_timer, apple_boot_expired, 0);
	initialized = true;
	supported = of_machine_is_compatible("apple,j71t") && !is_kdump_kernel();
	if (!supported) {
		timeout = 0;
		return 0;
	}
	if (!timeout)
		return 0;
	pr_warn("j71t boot deadline armed for %u seconds (software timer, no SoC reset)\n",
		timeout);
	mod_timer(&boot_timer, jiffies + timeout * HZ);
	return 0;
}
subsys_initcall(apple_boot_watchdog_init);
