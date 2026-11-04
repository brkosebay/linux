// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A9 Mesa (Touch ID) sensor: AP-side power and SPI status transport.
 *
 * This is a deliberately small, separately testable slice of the Mesa driver.
 * It proves exactly two things on hardware:
 *
 *   1. the two sensor rails can be sequenced through the regulator framework
 *      (1V8 first, then 3V0; reverse on the way down), and
 *   2. one 23-byte full-duplex status poll can be issued and parsed.
 *
 * It does not capture frames, upload a sensor patch, provision the sensor,
 * exchange session keys, touch sensor NVM, calibrate, reset, boost HV or
 * store any key material. No such operation exists in this file, not even as
 * an unused constant: the only bytes ever written to MOSI are the seven-byte
 * status poll below, padded with 0xff to the reply length. The sensor patch
 * path and every state-machine command are out of scope and stay out until
 * their provenance and wire format are proven (docs/sep/workstreams/
 * mesa-driver-re.md, sections 8 and 12).
 *
 * The driver is inert unless loaded with mesa_a9.enable=1. Even then probe
 * only acquires resources: it does not enable a rail, request a transfer or
 * touch the bus. Bring-up goes through /sys/kernel/debug/mesa-a9:
 *
 *   arm        write 1 once per boot, or 0 for a safe disarm
 *   power      write "on" or "off"
 *   status     read one status poll (requires arm and power)
 *   state      read-only snapshot
 *   irq-count  read-only interrupt counter
 *
 * Every mutating operation requires CAP_SYS_RAWIO; status does too, because
 * it is the only operation that transacts with the sensor. The IRQ handler
 * never transacts and never takes the driver mutex.
 *
 * The device tree is deliberately not part of this change. The driver needs
 * only standard SPI-child resources: a "vdd-1v8" and a "vdd-3v0" supply, an
 * optional interrupt whose trigger comes from firmware, and spi-max-frequency.
 * See docs/sep/workstreams/mesa-driver-implementation.md for the gate list.
 */

#include <linux/atomic.h>
#include <linux/capability.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>

/* >>> MESA_A9_STATUS_CORE_BEGIN (host-testable: tools/sep/mesa_test.py) */
/*
 * Host-testable half of the driver: the only sensor frame that exists in this
 * slice, and its parser. tools/sep/mesa_test.py lifts this block, compiles it
 * with a small prelude and runs tools/sep/mesa_status_test.c against it, so
 * the tests drive this code and not a copy of it. Nothing here may use a
 * kernel API, or the block stops compiling on the host.
 */
#define MESA_A9_STATUS_XFER_LEN	23
#define MESA_A9_STATUS_ECHO_LEN	7
#define MESA_A9_STATUS_DATA_LEN	16

/* Response header at rx[7..10]; rx[9] is the one "??" byte. */
#define MESA_A9_STATUS_HDR0	0x80
#define MESA_A9_STATUS_HDR1	0x20
#define MESA_A9_STATUS_HDR3	0x10

/* Status payload offsets, relative to rx[7]. */
#define MESA_A9_STATUS_ST_STATE		7
#define MESA_A9_STATUS_ST_ERROR		8
#define MESA_A9_STATUS_ST_FLAGS9	9
#define MESA_A9_STATUS_ST_FLAGS11	11
#define MESA_A9_STATUS_ST_CONTEXT	12

/*
 * The whole request: tx[0..6] = 80 10 00 07 00 00 00 (opcode 0x10, total
 * length 7 as a little-endian u32, then three zero bytes). Bytes 7..22 are
 * 0xff, as in AppleSamsungSPI PIO: the sensor answers with 16 status bytes
 * after seven echo/dummy bytes.
 */
static const u8 mesa_a9_status_tx[MESA_A9_STATUS_XFER_LEN] = {
	0x80, 0x10, 0x00, 0x07, 0x00, 0x00, 0x00,
	/* AppleSamsungSPI PIO pads the 7-byte TX to 23 clocks with 0xff. */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

struct mesa_a9_status {
	u8 raw[MESA_A9_STATUS_DATA_LEN];	/* rx[7..22], untouched */
	u8 state;				/* raw[7] */
	u8 error;				/* raw[8] */
	u8 flags9;				/* raw[9] */
	u8 flags11;				/* raw[11] */
	u32 context;				/* raw[12..15], LE */
};

static inline bool mesa_a9_status_valid(const u8 *rx, size_t len)
{
	if (len != MESA_A9_STATUS_XFER_LEN)
		return false;

	return rx[MESA_A9_STATUS_ECHO_LEN + 0] == MESA_A9_STATUS_HDR0 &&
	       rx[MESA_A9_STATUS_ECHO_LEN + 1] == MESA_A9_STATUS_HDR1 &&
	       rx[MESA_A9_STATUS_ECHO_LEN + 3] == MESA_A9_STATUS_HDR3;
}

/*
 * Validate first, then copy. On failure the caller's structure is not touched,
 * so a stale status can never be mistaken for a fresh one.
 */
static inline int mesa_a9_status_parse(const u8 *rx, size_t len,
				       struct mesa_a9_status *st)
{
	const u8 *raw;

	if (!mesa_a9_status_valid(rx, len))
		return -EPROTO;

	raw = rx + MESA_A9_STATUS_ECHO_LEN;
	memcpy(st->raw, raw, MESA_A9_STATUS_DATA_LEN);
	st->state = raw[MESA_A9_STATUS_ST_STATE];
	st->error = raw[MESA_A9_STATUS_ST_ERROR];
	st->flags9 = raw[MESA_A9_STATUS_ST_FLAGS9];
	st->flags11 = raw[MESA_A9_STATUS_ST_FLAGS11];
	st->context = (u32)raw[MESA_A9_STATUS_ST_CONTEXT + 0] |
		      (u32)raw[MESA_A9_STATUS_ST_CONTEXT + 1] << 8 |
		      (u32)raw[MESA_A9_STATUS_ST_CONTEXT + 2] << 16 |
		      (u32)raw[MESA_A9_STATUS_ST_CONTEXT + 3] << 24;

	return 0;
}

/* <<< MESA_A9_STATUS_CORE_END */

/* Never ask the bus for more than the ADT's spi-frequency. */
#define MESA_A9_XFER_HZ_MAX	8000000

/* Apple startup allows three sanity checks; this experiment takes only one. */
#define MESA_A9_STATUS_MAX_ATTEMPTS 1

/* Power sequencing delays from AppleMesa_setSensorPower (static analysis). */
#define MESA_A9_ON_1V8_US	1000
#define MESA_A9_ON_3V0_MS	100
#define MESA_A9_OFF_3V0_US	1000
#define MESA_A9_OFF_DEFAULT_MS	1000
#define MESA_A9_OFF_MAX_MS	5000

struct mesa_a9 {
	struct device *dev;
	struct spi_device *spi;
	struct regulator *vdd_1v8;
	struct regulator *vdd_3v0;

	/*
	 * One mutex for power sequencing and the status transfer. The IRQ
	 * handler does not take it, so disable_irq() under this lock cannot
	 * deadlock against the handler.
	 */
	struct mutex lock;

	struct dentry *dbg;

	bool armed;
	bool arm_used;			/* arm is one-way per boot */
	bool powered;
	bool vdd_1v8_on;
	bool vdd_3v0_on;
	bool irq_requested;
	bool irq_enabled;
	bool mode_supported;
	bool configured;		/* spi_setup() done at first transfer */

	u32 bus_hz;			/* DT spi-max-frequency */
	u32 xfer_hz;			/* clamped to MESA_A9_XFER_HZ_MAX */
	u32 power_off_ms;

	atomic_t irq_count;
	unsigned long last_irq;

	unsigned int status_attempts;
	int last_status_err;
	bool last_status_valid;
	u8 last_rx[MESA_A9_STATUS_XFER_LEN];
	struct mesa_a9_status last_status;

	/*
	 * Formatted output of the last status poll. A read that starts at
	 * offset 0 performs exactly one poll and fills this; later chunks of
	 * the same read sequence are served from here, so a short userspace
	 * buffer cannot turn one read into several transfers.
	 */
	char status_text[768];
	size_t status_text_len;
};

/*
 * Every debugfs write and the status read pass through these two helpers.
 * Both run with mesa->lock held. The mutating entry points (arm, power) are
 * additionally write-only debugfs files (0200); state and irq-count are pure
 * memory reads and need no capability.
 */
static int mesa_a9_may_mutate(void)
{
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	return 0;
}

static int mesa_a9_guard(struct mesa_a9 *mesa, bool require_power)
{
	int ret = mesa_a9_may_mutate();

	if (ret)
		return ret;
	if (!mesa->armed)
		return -EACCES;
	if (require_power && !mesa->powered)
		return -ENODEV;

	return 0;
}

static void mesa_a9_hex(char *out, size_t out_size, const u8 *data, size_t len)
{
	size_t i, used = 0;

	if (!out_size)
		return;

	out[0] = '\0';
	for (i = 0; i < len; i++) {
		int n = scnprintf(out + used, out_size - used, "%s%02x",
				  i ? " " : "", data[i]);

		if (!n)
			break;
		used += n;
	}
}

static void mesa_a9_irq_enable_locked(struct mesa_a9 *mesa);
static void mesa_a9_irq_disable_locked(struct mesa_a9 *mesa);

/*
 * Power on: 1V8, 1 ms, 3V0, 100 ms. If 3V0 fails, 1V8 goes back off before
 * the error is returned. The per-rail flags make a partial state explicit and
 * keep regulator_disable() balanced (the regulator core warns on unbalanced
 * disables).
 */
static int mesa_a9_power_on_locked(struct mesa_a9 *mesa)
{
	bool enabled_1v8 = false;
	int ret;

	if (mesa->powered) {
		mesa_a9_irq_enable_locked(mesa);
		return 0;
	}

	if (!mesa->vdd_1v8_on) {
		ret = regulator_enable(mesa->vdd_1v8);
		if (ret) {
			dev_err(mesa->dev, "1v8 enable failed: %d\n", ret);
			return ret;
		}
		mesa->vdd_1v8_on = true;
		enabled_1v8 = true;
	}

	usleep_range(MESA_A9_ON_1V8_US, MESA_A9_ON_1V8_US + 250);

	if (!mesa->vdd_3v0_on) {
		ret = regulator_enable(mesa->vdd_3v0);
		if (ret) {
			dev_err(mesa->dev,
				"3v0 enable failed: %d; rolling 1v8 back off\n",
				ret);
			if (enabled_1v8 && !regulator_disable(mesa->vdd_1v8))
				mesa->vdd_1v8_on = false;
			return ret;
		}
		mesa->vdd_3v0_on = true;
	}

	msleep(MESA_A9_ON_3V0_MS);
	mesa->powered = true;
	/* Keep a potentially floating sensor IRQ disabled until both rails have
	 * settled. Firmware owns its trigger type; this only controls delivery.
	 */
	mesa_a9_irq_enable_locked(mesa);
	dev_info(mesa->dev, "power on: 1v8 then 3v0 (%u ms settle)\n",
		 MESA_A9_ON_3V0_MS);

	return 0;
}

/*
 * Power off: 3V0, 1 ms, 1V8, configured delay (default 1000 ms). Handles a
 * partial state: each rail is only disabled if this driver enabled it.
 */
static void mesa_a9_power_off_locked(struct mesa_a9 *mesa)
{
	bool any = mesa->vdd_1v8_on || mesa->vdd_3v0_on;

	/* Stop new sensor events before removing either rail. */
	mesa_a9_irq_disable_locked(mesa);

	if (mesa->vdd_3v0_on) {
		if (regulator_disable(mesa->vdd_3v0))
			dev_err(mesa->dev, "3v0 disable failed\n");
		else
			mesa->vdd_3v0_on = false;
	}

	if (any)
		usleep_range(MESA_A9_OFF_3V0_US, MESA_A9_OFF_3V0_US + 250);

	if (mesa->vdd_1v8_on) {
		if (regulator_disable(mesa->vdd_1v8))
			dev_err(mesa->dev, "1v8 disable failed\n");
		else
			mesa->vdd_1v8_on = false;
	}

	mesa->powered = false;
	mesa->last_status_valid = false;

	if (any) {
		msleep(mesa->power_off_ms);
		dev_info(mesa->dev, "power off: 3v0 then 1v8 (%u ms)\n",
			 mesa->power_off_ms);
	}
}

static void mesa_a9_irq_enable_locked(struct mesa_a9 *mesa)
{
	if (mesa->irq_requested && !mesa->irq_enabled) {
		enable_irq(mesa->spi->irq);
		mesa->irq_enabled = true;
	}
}

static void mesa_a9_irq_disable_locked(struct mesa_a9 *mesa)
{
	if (mesa->irq_enabled) {
		disable_irq(mesa->spi->irq);
		mesa->irq_enabled = false;
	}
}

/*
 * The handler counts and times the edge. It never powers a rail, never talks
 * to the sensor, never logs per interrupt and never takes mesa->lock, so it
 * cannot deadlock with disable_irq() or with the status path. Trigger type is
 * whatever firmware put on the interrupt; no IRQF_TRIGGER_* flag is passed.
 */
static irqreturn_t mesa_a9_irq(int irq, void *data)
{
	struct mesa_a9 *mesa = data;

	atomic_inc(&mesa->irq_count);
	WRITE_ONCE(mesa->last_irq, jiffies);

	return IRQ_HANDLED;
}

/*
 * Exactly one bounded full-duplex transfer, exactly once per call. Nothing
 * retries: a caller either gets a validated status or the raw bytes and an
 * error. The transfer is 23 bytes at no more than 8 MHz.
 */
static int mesa_a9_status_locked(struct mesa_a9 *mesa)
{
	struct spi_transfer xfer = { };
	int ret;

	if (mesa->status_attempts >= MESA_A9_STATUS_MAX_ATTEMPTS)
		return -EALREADY;

	/* Never expose a previous reply as if it were the current one. */
	memset(mesa->last_rx, 0, sizeof(mesa->last_rx));
	memset(&mesa->last_status, 0, sizeof(mesa->last_status));
	mesa->last_status_valid = false;
	mesa->last_status_err = 0;

	if (!mesa->powered)
		return -ENODEV;
	if (!mesa->mode_supported)
		return -EOPNOTSUPP;

	if (!mesa->configured) {
		mesa->spi->max_speed_hz = mesa->xfer_hz;
		mesa->spi->bits_per_word = 8;
		ret = spi_setup(mesa->spi);
		if (ret)
			return ret;
		mesa->configured = true;
	}

	xfer.tx_buf = mesa_a9_status_tx;
	xfer.rx_buf = mesa->last_rx;
	xfer.len = MESA_A9_STATUS_XFER_LEN;
	xfer.speed_hz = mesa->xfer_hz;
	xfer.bits_per_word = 8;

	mesa->status_attempts++;

	ret = spi_sync_transfer(mesa->spi, &xfer, 1);
	if (ret) {
		mesa->last_status_err = ret;
		mesa->last_status_valid = false;
		dev_err(mesa->dev, "status transfer failed: %d\n", ret);
		return ret;
	}

	ret = mesa_a9_status_parse(mesa->last_rx, sizeof(mesa->last_rx),
				   &mesa->last_status);
	mesa->last_status_err = ret;
	mesa->last_status_valid = !ret;
	if (ret)
		dev_warn(mesa->dev, "status reply failed validation: %d\n", ret);

	return ret;
}

static ssize_t mesa_a9_state_read(struct file *file, char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct mesa_a9 *mesa = file->private_data;
	char buf[640];
	int len;

	mutex_lock(&mesa->lock);
	len = scnprintf(buf, sizeof(buf),
			"mesa-a9: AP-side power and status transport only\n"
			"armed=%u arm_used=%u powered=%u rail_1v8=%u rail_3v0=%u\n"
			"spi_mode=%u mode_supported=%u configured=%u bus_hz=%u xfer_hz=%u\n"
			"power_off_ms=%u\n"
			"status_max_attempts=1 experiment=mesa2\n"
			"irq=%d irq_requested=%u irq_enabled=%u irq_count=%d last_irq_jiffies=%lu\n"
			"status_attempts=%u last_status_err=%d last_status_valid=%u\n",
			(unsigned int)mesa->armed,
			(unsigned int)mesa->arm_used,
			(unsigned int)mesa->powered,
			(unsigned int)mesa->vdd_1v8_on,
			(unsigned int)mesa->vdd_3v0_on,
			(unsigned int)mesa->spi->mode,
			(unsigned int)mesa->mode_supported,
			(unsigned int)mesa->configured,
			mesa->bus_hz, mesa->xfer_hz, mesa->power_off_ms,
			mesa->spi->irq,
			(unsigned int)mesa->irq_requested,
			(unsigned int)mesa->irq_enabled,
			atomic_read(&mesa->irq_count),
			READ_ONCE(mesa->last_irq),
			mesa->status_attempts, mesa->last_status_err,
			(unsigned int)mesa->last_status_valid);
	mutex_unlock(&mesa->lock);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t mesa_a9_irq_count_read(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct mesa_a9 *mesa = file->private_data;
	char buf[32];
	int len;

	len = scnprintf(buf, sizeof(buf), "%d\n",
			atomic_read(&mesa->irq_count));

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

/*
 * One read sequence is one poll. The first read (offset 0) performs exactly
 * one transfer and formats the result; later chunks of the same sequence are
 * served from that text, so a short buffer cannot cause more transfers. The
 * output always includes the raw bytes, including when validation or the
 * transfer failed; only guard failures return errno. This is a transport
 * check, not a Touch ID readiness claim.
 */
static ssize_t mesa_a9_status_read(struct file *file, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct mesa_a9 *mesa = file->private_data;
	char tx[MESA_A9_STATUS_XFER_LEN * 3];
	char rx[MESA_A9_STATUS_XFER_LEN * 3];
	char raw[MESA_A9_STATUS_DATA_LEN * 3];
	ssize_t len;
	int ret;

	/* A zero-length read must not trigger a transfer. */
	if (!count)
		return 0;

	mutex_lock(&mesa->lock);
	if (!*ppos) {
		ret = mesa_a9_guard(mesa, true);
		if (ret) {
			mutex_unlock(&mesa->lock);
			return ret;
		}

		ret = mesa_a9_status_locked(mesa);
		mesa->last_status_err = ret;
		mesa_a9_hex(tx, sizeof(tx), mesa_a9_status_tx,
			    MESA_A9_STATUS_XFER_LEN);
		mesa_a9_hex(rx, sizeof(rx), mesa->last_rx,
			    MESA_A9_STATUS_XFER_LEN);
		mesa_a9_hex(raw, sizeof(raw),
			    mesa->last_rx + MESA_A9_STATUS_ECHO_LEN,
			    MESA_A9_STATUS_DATA_LEN);
		mesa->status_text_len =
			scnprintf(mesa->status_text, sizeof(mesa->status_text),
				  "mesa-a9 status: one full-duplex poll, transport only\n"
				  "spi_error=%d valid=%u attempts=%u\n"
				  "tx=%s\nrx=%s\nraw=%s\n"
				  "state[7]=0x%02x error[8]=0x%02x flags[9]=0x%02x flags[11]=0x%02x context[12..15]=0x%08x\n",
				  ret, (unsigned int)mesa->last_status_valid,
				  mesa->status_attempts, tx, rx, raw,
				  mesa->last_status.state,
				  mesa->last_status.error,
				  mesa->last_status.flags9,
				  mesa->last_status.flags11,
				  mesa->last_status.context);
		/* Every completed status read tears down even on SPI/setup/header
		 * failure or a later copy_to_user fault. The saved text stays raw.
		 */
		mesa_a9_power_off_locked(mesa);
		mesa->armed = false;
	}

	len = simple_read_from_buffer(ubuf, count, ppos, mesa->status_text,
				      mesa->status_text_len);
	mutex_unlock(&mesa->lock);

	return len;
}

/*
 * arm=1 is one-way per boot. arm=0 is a safe disarm: interrupts off, both
 * rails off in reverse order, then the arming bit cleared. A disarmed device
 * cannot be re-armed until the driver is reloaded or the machine reboots.
 */
static ssize_t mesa_a9_arm_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct mesa_a9 *mesa = file->private_data;
	bool on;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &on);
	if (ret)
		return ret;

	mutex_lock(&mesa->lock);
	ret = mesa_a9_may_mutate();
	if (ret)
		goto out;

	if (on) {
		if (mesa->armed) {
			ret = 0;
		} else if (mesa->arm_used) {
			dev_warn(mesa->dev,
				 "arm is one-way per boot; reboot or reload\n");
			ret = -EPERM;
		} else {
			mesa->arm_used = true;
			mesa->armed = true;
			dev_info(mesa->dev,
				 "armed; IRQ remains disabled until power is settled\n");
		}
	} else {
		mesa_a9_irq_disable_locked(mesa);
		mesa_a9_power_off_locked(mesa);
		mesa->armed = false;
		dev_info(mesa->dev,
			 "disarmed; rails off, no further arm this boot\n");
	}
out:
	mutex_unlock(&mesa->lock);

	return ret ? ret : count;
}

static ssize_t mesa_a9_power_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct mesa_a9 *mesa = file->private_data;
	bool on;
	int ret;

	ret = kstrtobool_from_user(ubuf, count, &on);
	if (ret)
		return ret;

	mutex_lock(&mesa->lock);
	ret = mesa_a9_guard(mesa, false);
	if (ret)
		goto out;

	if (on) {
		ret = mesa_a9_power_on_locked(mesa);
	} else {
		mesa_a9_power_off_locked(mesa);
		ret = 0;
	}
out:
	mutex_unlock(&mesa->lock);

	return ret ? ret : count;
}

static const struct file_operations mesa_a9_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = mesa_a9_state_read,
};

static const struct file_operations mesa_a9_irq_count_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = mesa_a9_irq_count_read,
};

static const struct file_operations mesa_a9_status_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = mesa_a9_status_read,
};

static const struct file_operations mesa_a9_arm_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mesa_a9_arm_write,
};

static const struct file_operations mesa_a9_power_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mesa_a9_power_write,
};

static void mesa_a9_debugfs_init(struct mesa_a9 *mesa)
{
	mesa->dbg = debugfs_create_dir("mesa-a9", NULL);
	if (IS_ERR_OR_NULL(mesa->dbg)) {
		mesa->dbg = NULL;
		return;
	}

	debugfs_create_file("state", 0444, mesa->dbg, mesa,
			    &mesa_a9_state_fops);
	debugfs_create_file("arm", 0200, mesa->dbg, mesa, &mesa_a9_arm_fops);
	debugfs_create_file("power", 0200, mesa->dbg, mesa,
			    &mesa_a9_power_fops);
	debugfs_create_file("status", 0400, mesa->dbg, mesa,
			    &mesa_a9_status_fops);
	debugfs_create_file("irq-count", 0444, mesa->dbg, mesa,
			    &mesa_a9_irq_count_fops);
}

/*
 * Probe is a resource acquisition, not a bring-up. No rail is enabled, no
 * byte is clocked and spi_setup() is deferred to the first status poll (the
 * SPI core has already configured the device from firmware at registration).
 */
static int mesa_a9_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mesa_a9 *mesa;
	u32 off_ms = MESA_A9_OFF_DEFAULT_MS;
	int ret;

	mesa = devm_kzalloc(dev, sizeof(*mesa), GFP_KERNEL);
	if (!mesa)
		return -ENOMEM;

	mesa->dev = dev;
	mesa->spi = spi;
	mutex_init(&mesa->lock);
	spi_set_drvdata(spi, mesa);

	/* Named supplies, in the same order the power-on sequence uses. */
	mesa->vdd_1v8 = devm_regulator_get(dev, "vdd-1v8");
	if (IS_ERR(mesa->vdd_1v8))
		return dev_err_probe(dev, PTR_ERR(mesa->vdd_1v8),
				     "vdd-1v8 supply\n");

	mesa->vdd_3v0 = devm_regulator_get(dev, "vdd-3v0");
	if (IS_ERR(mesa->vdd_3v0))
		return dev_err_probe(dev, PTR_ERR(mesa->vdd_3v0),
				     "vdd-3v0 supply\n");

	if (!device_property_read_u32(dev, "apple,power-off-delay-ms",
				      &off_ms) &&
	    (!off_ms || off_ms > MESA_A9_OFF_MAX_MS)) {
		dev_warn(dev, "power-off delay %u ms outside 1..%u; using %u\n",
			 off_ms, MESA_A9_OFF_MAX_MS,
			 MESA_A9_OFF_DEFAULT_MS);
		off_ms = MESA_A9_OFF_DEFAULT_MS;
	}
	mesa->power_off_ms = off_ms;

	mesa->bus_hz = spi->max_speed_hz;
	mesa->xfer_hz = mesa->bus_hz;
	if (!mesa->xfer_hz || mesa->xfer_hz > MESA_A9_XFER_HZ_MAX)
		mesa->xfer_hz = MESA_A9_XFER_HZ_MAX;

	/*
	 * Mode comes from firmware. J71t Mesa uses CPOL=0, CPHA=1; Apple's
	 * additional clock-phase bit is controller-specific. Anything else (CS polarity
	 * inversion, LSB first, 3-wire, loopback, no CS) is refused rather
	 * than silently translated, and no fallback between modes exists.
	 */
	mesa->mode_supported = !(spi->mode & ~(SPI_CPOL | SPI_CPHA));
	if (!mesa->mode_supported)
		dev_warn(dev,
			 "spi mode %#x is not supported; status will refuse\n",
			 spi->mode);

	/*
	 * Optional IRQ. Its trigger is firmware-owned: no IRQF_TRIGGER_* flag
	 * is passed here. It starts disabled and is enabled only after an explicit
	 * power-on has settled both rails.
	 */
	if (spi->irq < 0)
		return dev_err_probe(dev, spi->irq, "IRQ lookup\n");
	if (spi->irq > 0) {
		ret = devm_request_threaded_irq(dev, spi->irq, NULL,
						mesa_a9_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						dev_name(dev), mesa);
		if (ret)
			return dev_err_probe(dev, ret, "IRQ %d request\n",
					     spi->irq);
		mesa->irq_requested = true;
	}

	mesa_a9_debugfs_init(mesa);

	dev_info(dev,
		 "probe: inert (no rail, no transfer); mode=%u supported=%u bus_hz=%u xfer_hz=%u power_off_ms=%u irq=%d\n",
		 spi->mode, (unsigned int)mesa->mode_supported, mesa->bus_hz,
		 mesa->xfer_hz, mesa->power_off_ms, spi->irq);

	return 0;
}

/*
 * Removal and shutdown both end with both rails off. A failed
 * regulator_disable() is retried exactly once; there is no other retry.
 */
static void mesa_a9_remove(struct spi_device *spi)
{
	struct mesa_a9 *mesa = spi_get_drvdata(spi);

	debugfs_remove_recursive(mesa->dbg);
	mutex_lock(&mesa->lock);
	mesa_a9_irq_disable_locked(mesa);
	mesa->armed = false;
	mesa_a9_power_off_locked(mesa);
	if (mesa->vdd_1v8_on || mesa->vdd_3v0_on)
		mesa_a9_power_off_locked(mesa);
	mutex_unlock(&mesa->lock);
}

static void mesa_a9_shutdown(struct spi_device *spi)
{
	struct mesa_a9 *mesa = spi_get_drvdata(spi);

	mutex_lock(&mesa->lock);
	mesa_a9_irq_disable_locked(mesa);
	mesa->armed = false;
	mesa_a9_power_off_locked(mesa);
	if (mesa->vdd_1v8_on || mesa->vdd_3v0_on)
		mesa_a9_power_off_locked(mesa);
	mutex_unlock(&mesa->lock);
}

static const struct of_device_id mesa_a9_of_match[] = {
	{ .compatible = "apple,mesa" },
	{ .compatible = "biosensor,mesa" },	/* the ADT's own name */
	{ }
};
MODULE_DEVICE_TABLE(of, mesa_a9_of_match);

static struct spi_driver mesa_a9_driver = {
	.probe = mesa_a9_probe,
	.remove = mesa_a9_remove,
	.shutdown = mesa_a9_shutdown,
	.driver = {
		.name = "mesa-a9",
		.of_match_table = mesa_a9_of_match,
	},
};

static bool mesa_a9_registered;

/*
 * Off by default: the sensor is unpowered and its DT wiring is not merged, so
 * probing without an explicit opt-in would only produce deferred probes.
 * Enable explicitly with mesa_a9.enable=1 on the kernel command line or on
 * modprobe.
 */
static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Probe the A9 Mesa sensor transport (default: off)");

static int __init mesa_a9_init(void)
{
	int ret;

	if (!enable) {
		pr_info("mesa-a9: disabled; pass mesa_a9.enable=1 to probe\n");
		return 0;
	}

	ret = spi_register_driver(&mesa_a9_driver);
	if (ret)
		return ret;

	mesa_a9_registered = true;
	return 0;
}

static void __exit mesa_a9_exit(void)
{
	if (mesa_a9_registered)
		spi_unregister_driver(&mesa_a9_driver);
}

module_init(mesa_a9_init);
module_exit(mesa_a9_exit);

MODULE_DESCRIPTION("Apple A9 Mesa sensor AP-side power and status transport");
MODULE_LICENSE("GPL");
