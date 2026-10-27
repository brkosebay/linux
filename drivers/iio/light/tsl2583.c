// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Device driver for monitoring ambient light intensity (lux)
 * within the TAOS tsl258x family of devices (tsl2580, tsl2581, tsl2583).
 *
 * Copyright (c) 2011, TAOS Corporation.
 * Copyright (c) 2016-2017 Brian Masney <masneyb@onstation.org>
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/property.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/overflow.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/pm_runtime.h>
#include <linux/unaligned.h>

#include "tsl2583.h"

/* Device Registers and Masks */
#define TSL2583_CNTRL			0x00
#define TSL2583_ALS_TIME		0X01
#define TSL2583_INTERRUPT		0x02
#define TSL2583_GAIN			0x07
#define CT821_ALS_CONFIG		0x0c
#define CT821_GAIN			0x0f
#define TSL2583_REVID			0x11
#define TSL2583_CHIPID			0x12
#define TSL2583_ALS_CHAN0LO		0x14
#define TSL2583_ALS_CHAN0HI		0x15
#define TSL2583_ALS_CHAN1LO		0x16
#define TSL2583_ALS_CHAN1HI		0x17
#define TSL2583_TMR_LO			0x18
#define TSL2583_TMR_HI			0x19

/* tsl2583 cmd reg masks */
#define TSL2583_CMD_REG			0x80
#define TSL2583_CMD_SPL_FN		0x60
#define TSL2583_CMD_ALS_INT_CLR		0x01
#define CT821_CMD_ALS_INT_CLR		0xc1

/* tsl2583 cntrl reg masks */
#define TSL2583_CNTL_ADC_ENBL		0x02
#define TSL2583_CNTL_PWR_OFF		0x00
#define TSL2583_CNTL_PWR_ON		0x01
#define CT821_CNTL_ENABLE		0x10

#define CT821_ALS_CONFIG_ENABLE		0x40

/* tsl2583 status reg masks */
#define TSL2583_STA_ADC_VALID		0x10
#define TSL2583_STA_ADC_INTR		0x20

#define TSL2583_INTERRUPT_DISABLED	0x00

#define TSL2583_CHIP_ID			0x90
#define TSL2583_CHIP_ID_MASK		0xf0

#define TSL2583_POWER_OFF_DELAY_MS	2000

/* Nominal length of one ADC integration cycle on the generic TSL258x parts */
#define TSL2583_INTEGRATION_CYCLE_US	2700

/* The LSCI record layout is versioned; see tsl2583.h for what they share. */
#define APPLE_ALS_CAL_HEADER_SIZE	8

#define CT819_CAL_RECORD_SIZE_V2	0x40
#define CT819_CAL_GAIN_COUNT		4
#define CT819_CAL_RATIO_OFF		0x30
/* AppleEmbeddedLightSensor publishes ALSSensorMax = 5000 for the CT819. */
#define CT819_LUX_MAX			5000

#define CT821_CAL_RECORD_SIZE_V3	0x54
#define CT821_CAL_RECORD_DATA_SIZE_V3	0x52
#define CT821_CAL_GAIN_COUNT		8
#define CT821_CAL_GAIN_COUNT_OFF	0x30
#define CT821_CAL_RATIO_OFF		0x36
#define CT821_INTEGRATION_CYCLE_US	2780

/* Per-device data */
struct tsl2583_als_info {
	u16 als_ch0;
	u16 als_ch1;
	u16 lux;
};

struct tsl2583_lux {
	unsigned int ratio;
	unsigned int ch0;
	unsigned int ch1;
};

static const struct tsl2583_lux tsl2583_default_lux[] = {
	{  9830,  8520, 15729 },
	{ 12452, 10807, 23344 },
	{ 14746,  6383, 11705 },
	{ 17695,  4063,  6554 },
	{     0,     0,     0 }  /* Termination segment */
};

#define TSL2583_MAX_LUX_TABLE_ENTRIES 11

struct tsl2583_settings {
	int als_time;
	int als_gain;
	int als_gain_trim;
	int als_cal_target;

	/*
	 * This structure is intentionally large to accommodate updates via
	 * sysfs. Sized to 11 = max 10 segments + 1 termination segment.
	 * Assumption is that one and only one type of glass used.
	 */
	struct tsl2583_lux als_device_lux[TSL2583_MAX_LUX_TABLE_ENTRIES];
};

struct tsl2583_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
	struct tsl2583_als_info als_cur_info;
	struct tsl2583_settings als_settings;
	int als_time_scale;
	int als_saturation;
	const struct tsl2583_chip_info *info;
	struct apple_als_calibration cal;
	u16 integration_cycles;
	bool needs_reinit;
};

struct gainadj {
	s16 ch0;
	s16 ch1;
	s16 mean;
};

/*
 * Describes one supported part. @gainadj is indexed by the gain selection
 * index programmed into @gain_reg.
 *
 * The Apple variants are TSL258x compatible parts with a factory LSCI
 * calibration record in SysCfg. They select a different lux computation and
 * therefore also different chip-specific register handling:
 *
 * @gain_reg:		register holding the gain selection index
 * @int_clr_data_cmd:	if non-zero, the ALS interrupt is cleared by a byte
 *			write to this command instead of the TSL258x
 *			special function
 * @cntl_enable:	extra CNTRL bits the part needs while measuring
 * @als_config:		part needs its analog frontend programmed explicitly
 *			while the core is powered down
 * @cal_record_size:	size of the supported LSCI calibration record
 * @cal_record_len:	record length field of the supported record
 * @cal_gain_count:	number of gain steps described by that record
 * @cal_gain_count_off:	offset of the gain-count byte, zero if the record has
 *			none
 * @cal_ratio_off:	offset of the gain ratio pair of the second gain step
 * @cal_ratio_scale:	fixed-point scale of the recorded gain ratios
 * @integration_cycle_us: length of one ADC integration cycle
 * @counts_per_cycle:	ADC counts per integration cycle, zero if only the 16 bit
 *			range limits them
 * @lux_max:		largest lux value the calibrated computation reports
 */
struct tsl2583_chip_info {
	const struct gainadj *gainadj;
	unsigned int num_gainadj;
	const struct iio_info *iio_info;
	int default_als_time;
	int default_als_gain;
	bool apple;
	u8 gain_reg;
	u8 int_clr_data_cmd;
	u8 cntl_enable;
	bool als_config;
	u8 cal_version;
	u8 cal_record_size;
	u8 cal_record_len;
	unsigned int cal_gain_count;
	unsigned int cal_gain_count_off;
	unsigned int cal_ratio_off;
	unsigned int cal_ratio_scale;
	unsigned int integration_cycle_us;
	unsigned int counts_per_cycle;
	unsigned int lux_max;
};

static const struct gainadj tsl2583_gainadj[] = {
	{ 1, 1, 1 },
	{ 8, 8, 8 },
	{ 16, 16, 16 },
	{ 107, 115, 111 }
};

/*
 * The CT819 uses the TSL2581 gain register and its four gain steps. The
 * nominal factors below only select the step; the exact per-device gain comes
 * from the LSCI calibration record.
 */
static const struct gainadj ct819_gainadj[] = {
	{ 1, 1, 1 },
	{ 8, 8, 8 },
	{ 16, 16, 16 },
	{ 128, 128, 128 },
};

static const struct gainadj ct821_gainadj[] = {
	{ 1, 1, 1 },
	{ 2, 2, 2 },
	{ 4, 4, 4 },
	{ 8, 8, 8 },
	{ 16, 16, 16 },
	{ 32, 32, 32 },
	{ 64, 64, 64 },
	{ 140, 140, 140 },
};

/*
 * Two things about the CT819 are not settled by the static analysis of the iOS
 * driver (docs/periph/ds-als-hall.md, A1 and A2). Both can be flipped on a
 * running system through /sys/module/tsl2583/parameters/ (or tsl2583.<name>=
 * on the kernel command line) while the answer is measured.
 *
 * apple_no_pwr_bit: iOS writes CONTROL = 0x02 (ADC enable only); Linux writes
 *	POWER | ADC_ENABLE = 0x03. Set this to write what iOS writes. The next
 *	power-up (autosuspend, or a write to integration_time) applies it.
 * apple_int_clear: how the ALS interrupt status is cleared after a sample.
 *	0 the TSL258x special function (command 0xe1), the default
 *	1 a write of 0x40 to command 0x8c, as a sibling iOS class does
 *	2 do not clear at all
 */
static bool apple_no_pwr_bit;
module_param(apple_no_pwr_bit, bool, 0644);
MODULE_PARM_DESC(apple_no_pwr_bit, "Apple ALS: write CONTROL = ADC enable only, as iOS does");

static int apple_int_clear;
module_param(apple_int_clear, int, 0644);
MODULE_PARM_DESC(apple_int_clear, "Apple ALS interrupt clear: 0 special function, 1 write 0x40 to 0x8c, 2 none");

#define APPLE_ALS_INT_CLR_SPL_FN	0
#define APPLE_ALS_INT_CLR_REG_0C	1
#define APPLE_ALS_INT_CLR_NONE		2
#define APPLE_ALS_INT_CLR_REG		(TSL2583_CMD_REG | 0x0c)
#define APPLE_ALS_INT_CLR_REG_VALUE	0x40

static int apple_als_get_lux(struct tsl2583_chip *chip, u16 ch0, u16 ch1);

static int apple_als_gain_to_index(const struct tsl2583_chip_info *info, u8 gain)
{
	unsigned int i;

	for (i = 0; i < info->num_gainadj; i++)
		if (info->gainadj[i].mean == gain)
			return i;

	return -EINVAL;
}

/*
 * The recorded gain ratios are relative to the previous gain step, so the
 * first step has no entry of its own.
 */
static unsigned int apple_als_ratio_offset(const struct tsl2583_chip_info *info,
					   unsigned int gain)
{
	return info->cal_ratio_off + (gain - 1) * 4;
}

static bool apple_als_record_is_supported(const struct tsl2583_chip_info *info,
					  const u8 *record)
{
	if (record[0] || record[3] != info->cal_record_len || record[4] != 1)
		return false;

	if (info->cal_gain_count_off &&
	    record[info->cal_gain_count_off] != info->cal_gain_count)
		return false;

	return true;
}

static int apple_als_load_calibration(struct tsl2583_chip *chip)
{
	const struct tsl2583_chip_info *info = chip->info;
	struct device *dev = &chip->client->dev;
	struct apple_als_calibration *cal = &chip->cal;
	struct nvmem_cell *cell;
	const struct firmware *fw;
	const char *fw_name;
	const u8 *record;
	size_t len;
	u32 checksum = 0;
	u8 *data;
	int gain_index;
	int ret;
	int i;

	if (!device_property_present(dev, "nvmem-cells") &&
	    !device_property_read_string(dev, "firmware-name", &fw_name)) {
		/* Missing calibration must not wait for a userspace helper. */
		ret = request_firmware_direct(&fw, fw_name, dev);
		if (ret)
			return dev_err_probe(dev, ret, "failed to load LSCI file\n");
		len = fw->size;
		data = kmemdup(fw->data, len, GFP_KERNEL);
		release_firmware(fw);
		if (!data)
			return -ENOMEM;
	} else {
		cell = devm_nvmem_cell_get(dev, "calibration");
		if (IS_ERR(cell))
			return dev_err_probe(dev, PTR_ERR(cell),
					     "failed to get LSCI calibration\n");
		data = nvmem_cell_read(cell, &len);
		if (IS_ERR(data))
			return dev_err_probe(dev, PTR_ERR(data),
					     "failed to read LSCI calibration\n");
	}

	if (len < APPLE_ALS_CAL_HEADER_SIZE + info->cal_record_size ||
	    len & 1 || data[1] != info->cal_version || data[2] != 1 ||
	    data[3] != APPLE_ALS_CAL_HEADER_SIZE ||
	    get_unaligned_le16(data + 4) != len) {
		dev_err(dev, "invalid LSCI v%u calibration header\n",
			info->cal_version);
		goto invalid;
	}

	for (i = 0; i < len; i += 2)
		checksum += get_unaligned_le16(data + i);
	if ((u16)checksum != U16_MAX) {
		dev_err(dev, "invalid LSCI calibration checksum\n");
		goto invalid;
	}

	record = data + data[3];
	if (!apple_als_record_is_supported(info, record)) {
		dev_err(dev, "unsupported LSCI v%u calibration record\n",
			info->cal_version);
		goto invalid;
	}
	if (memchr_inv(record + 0x2a, 0, 6)) {
		dev_err(dev, "unsupported LSCI backlight-leakage calibration\n");
		goto invalid;
	}

	cal->lux_fudge = get_unaligned_le16(record + 0x08);
	cal->integration_time = get_unaligned_le16(record + 0x0a);
	gain_index = apple_als_gain_to_index(info, record[0x10]);
	cal->a_coeff1 = get_unaligned_le32(record + 0x14);
	cal->b_coeff1 = get_unaligned_le32(record + 0x18);
	cal->upper_range = get_unaligned_le32(record + 0x1c);
	cal->a_coeff2 = get_unaligned_le32(record + 0x20);
	cal->b_coeff2 = get_unaligned_le32(record + 0x24);
	cal->ch0_dark_counts = record[0x28];
	cal->ch1_dark_counts = record[0x29];

	if (!cal->lux_fudge || cal->integration_time < 50 ||
	    cal->integration_time > 650 || gain_index < 0 ||
	    cal->a_coeff1 <= 0 || cal->b_coeff1 <= 0 ||
	    cal->upper_range <= 0 || cal->a_coeff2 <= 0 ||
	    cal->b_coeff2 <= 0) {
		dev_err(dev, "invalid LSCI calibration values\n");
		goto invalid;
	}
	cal->gain = gain_index;

	cal->gain_factor[0][0] = APPLE_ALS_CAL_SCALE;
	cal->gain_factor[0][1] = APPLE_ALS_CAL_SCALE;
	for (i = 1; i < info->cal_gain_count; i++) {
		const u8 *ratios = record + apple_als_ratio_offset(info, i);
		u16 ch0 = get_unaligned_le16(ratios);
		u16 ch1 = get_unaligned_le16(ratios + 2);
		u32 previous;
		u32 *factor;

		previous = cal->gain_factor[i - 1][0];
		factor = &cal->gain_factor[i][0];
		ret = apple_als_calculate_gain_factor(previous, ch0,
						      info->cal_ratio_scale,
						      factor);
		if (!ret) {
			previous = cal->gain_factor[i - 1][1];
			factor = &cal->gain_factor[i][1];
			ret = apple_als_calculate_gain_factor(previous, ch1,
							      info->cal_ratio_scale,
							      factor);
		}
		if (ret) {
			dev_err(dev, "invalid LSCI gain factors\n");
			goto invalid;
		}
	}

	chip->als_settings.als_time = cal->integration_time;
	chip->als_settings.als_gain = cal->gain;
	dev_info(dev, "loaded LSCI v%u ambient-light calibration\n",
		 info->cal_version);
	kfree(data);

	return 0;

invalid:
	kfree(data);
	return -EINVAL;
}

/*
 * Provides initial operational parameter defaults.
 * These defaults may be changed through the device's sysfs files.
 */
static void tsl2583_defaults(struct tsl2583_chip *chip)
{
	/*
	 * The integration time must be a multiple of 50ms and within the
	 * range [50, 650] ms.
	 */
	chip->als_settings.als_time = chip->info->default_als_time;

	/*
	 * This is an index into the gainadj table. Assume clear glass as the
	 * default.
	 */
	chip->als_settings.als_gain = chip->info->default_als_gain;

	/* Default gain trim to account for aperture effects */
	chip->als_settings.als_gain_trim = 1000;

	/* Known external ALS reading used for calibration */
	chip->als_settings.als_cal_target = 130;

	/* Default lux table. */
	memcpy(chip->als_settings.als_device_lux, tsl2583_default_lux,
	       sizeof(tsl2583_default_lux));
}

/* The Apple part that is a plain TSL2581 apart from its calibration. */
static bool tsl2583_is_ct819(const struct tsl2583_chip *chip)
{
	return chip->info->apple && !chip->info->als_config;
}

static int tsl2583_clear_interrupt(struct tsl2583_chip *chip)
{
	if (chip->info->int_clr_data_cmd)
		return i2c_smbus_write_byte_data(chip->client,
						 chip->info->int_clr_data_cmd, 0);

	if (tsl2583_is_ct819(chip)) {
		switch (READ_ONCE(apple_int_clear)) {
		case APPLE_ALS_INT_CLR_REG_0C:
			return i2c_smbus_write_byte_data(chip->client,
							 APPLE_ALS_INT_CLR_REG,
							 APPLE_ALS_INT_CLR_REG_VALUE);
		case APPLE_ALS_INT_CLR_NONE:
			return 0;
		}
	}

	return i2c_smbus_write_byte(chip->client,
				    TSL2583_CMD_REG | TSL2583_CMD_SPL_FN |
				    TSL2583_CMD_ALS_INT_CLR);
}

/*
 * Reads and calculates current lux value.
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array als_device_lux[]
 * declared above is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the lux.
 */
static int tsl2583_get_lux(struct iio_dev *indio_dev)
{
	u16 ch0, ch1; /* separated ch0/ch1 data from device */
	u32 lux; /* raw lux calculated from device data */
	u64 lux64;
	u32 ratio;
	u8 buf[5];
	struct tsl2583_lux *p;
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int i, ret;

	ret = i2c_smbus_read_byte_data(chip->client, TSL2583_CMD_REG);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to read CMD_REG register\n",
			__func__);
		goto done;
	}

	/* is data new & valid */
	if (!(ret & TSL2583_STA_ADC_VALID)) {
		dev_dbg(&chip->client->dev, "%s: data not valid; returning last value\n",
			__func__);
		ret = chip->als_cur_info.lux; /* return LAST VALUE */
		goto done;
	}

	for (i = 0; i < 4; i++) {
		int reg = TSL2583_CMD_REG | (TSL2583_ALS_CHAN0LO + i);

		ret = i2c_smbus_read_byte_data(chip->client, reg);
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: failed to read register %x\n",
				__func__, reg);
			goto done;
		}
		buf[i] = ret;
	}

	/*
	 * Clear the pending interrupt status bit on the chip to allow the next
	 * integration cycle to start. This has to be done even though this
	 * driver currently does not support interrupts.
	 */
	ret = tsl2583_clear_interrupt(chip);
	if (ret < 0) {
		/*
		 * The CT819's clear is a guess (A2 in ds-als-hall.md): the counts
		 * are already in hand, so keep them and say so once.
		 */
		if (tsl2583_is_ct819(chip)) {
			dev_warn_once(&chip->client->dev,
				      "%s: failed to clear the interrupt bit (%d)\n",
				      __func__, ret);
		} else {
			dev_err(&chip->client->dev,
				"%s: failed to clear the interrupt bit\n", __func__);
			goto done; /* have no data, so return failure */
		}
	}

	/* extract ALS/lux data */
	ch0 = le16_to_cpup((const __le16 *)&buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&buf[2]);

	chip->als_cur_info.als_ch0 = ch0;
	chip->als_cur_info.als_ch1 = ch1;
	if (chip->info->apple) {
		ret = apple_als_get_lux(chip, ch0, ch1);
		if (ret >= 0)
			chip->als_cur_info.lux = ret;
		goto done;
	}

	if ((ch0 >= chip->als_saturation) || (ch1 >= chip->als_saturation))
		goto return_max;

	if (!ch0) {
		/*
		 * The sensor appears to be in total darkness so set the
		 * calculated lux to 0 and return early to avoid a division by
		 * zero below when calculating the ratio.
		 */
		ret = 0;
		chip->als_cur_info.lux = 0;
		goto done;
	}

	/* calculate ratio */
	ratio = (ch1 << 15) / ch0;

	/* convert to unscaled lux using the pointer to the table */
	for (p = (struct tsl2583_lux *)chip->als_settings.als_device_lux;
	     p->ratio != 0 && p->ratio < ratio; p++)
		;

	if (p->ratio == 0) {
		lux = 0;
	} else {
		u32 ch0lux, ch1lux;

		ch0lux = ((ch0 * p->ch0) +
			  (chip->info->gainadj[chip->als_settings.als_gain].ch0 >> 1))
			 / chip->info->gainadj[chip->als_settings.als_gain].ch0;
		ch1lux = ((ch1 * p->ch1) +
			  (chip->info->gainadj[chip->als_settings.als_gain].ch1 >> 1))
			 / chip->info->gainadj[chip->als_settings.als_gain].ch1;

		/* note: lux is 31 bit max at this point */
		if (ch1lux > ch0lux) {
			dev_dbg(&chip->client->dev, "%s: No Data - Returning 0\n",
				__func__);
			ret = 0;
			chip->als_cur_info.lux = 0;
			goto done;
		}

		lux = ch0lux - ch1lux;
	}

	/* adjust for active time scale */
	if (chip->als_time_scale == 0)
		lux = 0;
	else
		lux = (lux + (chip->als_time_scale >> 1)) /
			chip->als_time_scale;

	/*
	 * Adjust for active gain scale.
	 * The tsl2583_default_lux tables above have a factor of 8192 built in,
	 * so we need to shift right.
	 * User-specified gain provides a multiplier.
	 * Apply user-specified gain before shifting right to retain precision.
	 * Use 64 bits to avoid overflow on multiplication.
	 * Then go back to 32 bits before division to avoid using div_u64().
	 */
	lux64 = lux;
	lux64 = lux64 * chip->als_settings.als_gain_trim;
	lux64 >>= 13;
	lux = lux64;
	lux = DIV_ROUND_CLOSEST(lux, 1000);

	if (lux > TSL2583_LUX_CALC_OVER_FLOW) { /* check for overflow */
return_max:
		lux = TSL2583_LUX_CALC_OVER_FLOW;
	}

	/* Update the structure with the latest VALID lux. */
	chip->als_cur_info.lux = lux;
	ret = lux;

done:
	return ret;
}

static int apple_als_get_lux(struct tsl2583_chip *chip, u16 ch0, u16 ch1)
{
	return apple_als_calc_lux(&chip->cal, chip->als_settings.als_gain,
				  chip->integration_cycles,
				  chip->info->integration_cycle_us,
				  chip->info->counts_per_cycle,
				  chip->als_settings.als_gain_trim,
				  chip->info->lux_max, ch0, ch1);
}

/*
 * Obtain single reading and calculate the als_gain_trim (later used
 * to derive actual lux).
 * Return updated gain_trim value.
 */
static int tsl2583_als_calibrate(struct iio_dev *indio_dev)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	unsigned int gain_trim_val;
	int ret;
	int lux_val;

	ret = i2c_smbus_read_byte_data(chip->client,
				       TSL2583_CMD_REG | TSL2583_CNTRL);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read from the CNTRL register\n",
			__func__);
		return ret;
	}

	if ((ret & (TSL2583_CNTL_ADC_ENBL | TSL2583_CNTL_PWR_ON))
			!= (TSL2583_CNTL_ADC_ENBL | TSL2583_CNTL_PWR_ON)) {
		dev_err(&chip->client->dev,
			"%s: Device is not powered on and/or ADC is not enabled\n",
			__func__);
		return -EINVAL;
	} else if ((ret & TSL2583_STA_ADC_VALID) != TSL2583_STA_ADC_VALID) {
		dev_err(&chip->client->dev,
			"%s: The two ADC channels have not completed an integration cycle\n",
			__func__);
		return -ENODATA;
	}

	lux_val = tsl2583_get_lux(indio_dev);
	if (lux_val < 0) {
		dev_err(&chip->client->dev, "%s: failed to get lux\n",
			__func__);
		return lux_val;
	}

	/* Avoid division by zero of lux_value later on */
	if (lux_val == 0) {
		dev_err(&chip->client->dev,
			"%s: lux_val of 0 will produce out of range trim_value\n",
			__func__);
		return -ENODATA;
	}

	gain_trim_val = (unsigned int)(((chip->als_settings.als_cal_target)
			* chip->als_settings.als_gain_trim) / lux_val);
	if ((gain_trim_val < 250) || (gain_trim_val > 4000)) {
		dev_err(&chip->client->dev,
			"%s: trim_val of %d is not within the range [250, 4000]\n",
			__func__, gain_trim_val);
		return -ENODATA;
	}

	chip->als_settings.als_gain_trim = (int)gain_trim_val;

	return 0;
}

static int tsl2583_set_als_time(struct tsl2583_chip *chip)
{
	int als_count, als_time, ret;
	u8 val;

	/* determine als integration register */
	if (chip->info->apple)
		als_count = chip->als_settings.als_time * USEC_PER_MSEC /
			    chip->info->integration_cycle_us;
	else
		als_count = DIV_ROUND_CLOSEST(chip->als_settings.als_time * 100,
					      270);
	if (!als_count)
		als_count = 1; /* ensure at least one cycle */

	/* convert back to time (encompasses overrides) */
	als_time = DIV_ROUND_CLOSEST(als_count * 27, 10);

	val = 256 - als_count;
	ret = i2c_smbus_write_byte_data(chip->client,
					TSL2583_CMD_REG | TSL2583_ALS_TIME,
					val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to set the als time to %d\n",
			__func__, val);
		return ret;
	}

	/* set chip struct re scaling and saturation */
	chip->als_saturation = als_count * 922; /* 90% of full scale */
	chip->als_time_scale = DIV_ROUND_CLOSEST(als_time, 50);
	if (chip->info->apple)
		chip->integration_cycles = als_count;

	return ret;
}

static int tsl2583_set_als_gain(struct tsl2583_chip *chip)
{
	int ret;

	/* Set the gain based on als_settings struct */
	ret = i2c_smbus_write_byte_data(chip->client,
					TSL2583_CMD_REG | chip->info->gain_reg,
					chip->als_settings.als_gain);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to set the gain to %d\n", __func__,
			chip->als_settings.als_gain);

	return ret;
}

static int tsl2583_set_power_state(struct tsl2583_chip *chip, u8 state)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client,
					TSL2583_CMD_REG | TSL2583_CNTRL, state);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to set the power state to %d\n", __func__,
			state);

	return ret;
}

/*
 * iOS never sets the POWER bit of CONTROL on the CT819, so whether the part
 * keeps it is not known (docs/periph/ds-als-hall.md, A1). Read CONTROL back:
 * the ADC enable must stick, the power bit is allowed not to, and what the
 * part answered is logged once.
 */
static int apple_als_check_control(struct tsl2583_chip *chip, u8 written)
{
	const u8 mask = TSL2583_CNTL_ADC_ENBL | TSL2583_CNTL_PWR_ON;
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client,
				       TSL2583_CMD_REG | TSL2583_CNTRL);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to read CONTROL back\n");
		return ret;
	}

	if (!(ret & TSL2583_CNTL_ADC_ENBL)) {
		dev_err(&chip->client->dev,
			"ADC enable did not stick: CONTROL 0x%02x after writing 0x%02x\n",
			ret, written);
		return -EIO;
	}

	if ((ret & mask) != (written & mask))
		dev_info_once(&chip->client->dev,
			      "CONTROL reads 0x%02x after writing 0x%02x (power bit not kept)\n",
			      ret, written);
	else
		dev_dbg(&chip->client->dev, "CONTROL 0x%02x\n", ret);

	return 0;
}

/*
 * Turn the device on.
 * Configuration must be set before calling this function.
 */
static int tsl2583_chip_init_and_power_on(struct iio_dev *indio_dev)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	u8 cntl;
	int ret;

	if (chip->info->apple)
		chip->needs_reinit = true;

	/* Power on the device; ADC off. */
	ret = tsl2583_set_power_state(chip, chip->info->als_config ?
				      TSL2583_CNTL_PWR_OFF : TSL2583_CNTL_PWR_ON);
	if (ret < 0)
		return ret;

	if (chip->info->als_config) {
		/* CT821 requires its analog frontend to be enabled before ADC use. */
		ret = i2c_smbus_write_byte_data(chip->client,
						TSL2583_CMD_REG | CT821_ALS_CONFIG,
						CT821_ALS_CONFIG_ENABLE);
		if (ret < 0)
			return ret;

		ret = i2c_smbus_write_byte_data(chip->client,
						TSL2583_CMD_REG | CT821_ALS_CONFIG,
						CT821_ALS_CONFIG_ENABLE);
		if (ret < 0)
			return ret;
	}

	ret = i2c_smbus_write_byte_data(chip->client,
					TSL2583_CMD_REG | TSL2583_INTERRUPT,
					TSL2583_INTERRUPT_DISABLED);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to disable interrupts\n", __func__);
		return ret;
	}

	ret = tsl2583_set_als_time(chip);
	if (ret < 0)
		return ret;

	ret = tsl2583_set_als_gain(chip);
	if (ret < 0)
		return ret;

	usleep_range(3000, 3500);

	cntl = TSL2583_CNTL_ADC_ENBL | chip->info->cntl_enable;
	if (!tsl2583_is_ct819(chip) || !READ_ONCE(apple_no_pwr_bit))
		cntl |= TSL2583_CNTL_PWR_ON;
	ret = tsl2583_set_power_state(chip, cntl);
	if (ret < 0)
		return ret;

	if (tsl2583_is_ct819(chip)) {
		ret = apple_als_check_control(chip, cntl);
		if (ret < 0)
			return ret;
	}

	if (chip->info->als_config) {
		ret = tsl2583_set_als_time(chip);
		if (ret < 0)
			return ret;

		ret = i2c_smbus_write_byte_data(chip->client,
						TSL2583_CMD_REG | CT821_ALS_CONFIG,
						CT821_ALS_CONFIG_ENABLE);
		if (ret < 0)
			return ret;
	}

	/* Account for the 2.7 ms integration-time granularity. */
	msleep(chip->als_settings.als_time + 3);
	chip->needs_reinit = false;

	return ret;
}

static int apple_als_reconfigure(struct iio_dev *indio_dev, int als_time,
				 int als_gain, bool *force_suspend)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int old_time = chip->als_settings.als_time;
	int old_gain = chip->als_settings.als_gain;
	int power_ret;
	int restore_ret;
	int ret;

	chip->als_settings.als_time = als_time;
	chip->als_settings.als_gain = als_gain;
	ret = tsl2583_chip_init_and_power_on(indio_dev);
	if (!ret)
		return 0;

	chip->als_settings.als_time = old_time;
	chip->als_settings.als_gain = old_gain;
	restore_ret = tsl2583_chip_init_and_power_on(indio_dev);
	if (restore_ret) {
		dev_err(&chip->client->dev,
			"failed to restore configuration: %d\n", restore_ret);
		power_ret = tsl2583_set_power_state(chip, TSL2583_CNTL_PWR_OFF);
		if (power_ret)
			dev_err(&chip->client->dev,
				"failed to power off after restore failure: %d\n",
				power_ret);
		*force_suspend = true;
	}

	return ret;
}

static int tsl2583_reconfigure_gain(struct iio_dev *indio_dev, int gain,
				    bool *force_suspend)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int old_gain;
	int ret;

	if (chip->info->apple)
		return apple_als_reconfigure(indio_dev,
					     chip->als_settings.als_time, gain,
					     force_suspend);

	old_gain = chip->als_settings.als_gain;
	chip->als_settings.als_gain = gain;
	ret = tsl2583_set_als_gain(chip);
	if (ret < 0)
		chip->als_settings.als_gain = old_gain;

	return ret;
}

static int tsl2583_apply_time(struct iio_dev *indio_dev, int als_time,
			      bool *force_suspend)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int old_time;
	int ret;

	if (chip->info->apple)
		return apple_als_reconfigure(indio_dev, als_time,
					     chip->als_settings.als_gain,
					     force_suspend);

	old_time = chip->als_settings.als_time;
	chip->als_settings.als_time = als_time;
	ret = tsl2583_set_als_time(chip);
	if (ret < 0)
		chip->als_settings.als_time = old_time;

	return ret;
}

/* Sysfs Interface Functions */

static ssize_t in_illuminance_input_target_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%d\n", chip->als_settings.als_cal_target);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_input_target_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int value;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);
	chip->als_settings.als_cal_target = value;
	mutex_unlock(&chip->als_mutex);

	return len;
}

static ssize_t in_illuminance_calibrate_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int value, ret;

	if (kstrtoint(buf, 0, &value) || value != 1)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	ret = tsl2583_als_calibrate(indio_dev);
	if (ret < 0)
		goto done;

	ret = len;
done:
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_lux_table_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	unsigned int i;
	int offset = 0;

	for (i = 0; i < ARRAY_SIZE(chip->als_settings.als_device_lux); i++) {
		offset += sprintf(buf + offset, "%u,%u,%u,",
				  chip->als_settings.als_device_lux[i].ratio,
				  chip->als_settings.als_device_lux[i].ch0,
				  chip->als_settings.als_device_lux[i].ch1);
		if (chip->als_settings.als_device_lux[i].ratio == 0) {
			/*
			 * We just printed the first "0" entry.
			 * Now get rid of the extra "," and break.
			 */
			offset--;
			break;
		}
	}

	offset += sprintf(buf + offset, "\n");

	return offset;
}

static ssize_t in_illuminance_lux_table_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	const unsigned int max_ints = TSL2583_MAX_LUX_TABLE_ENTRIES * 3;
	int value[TSL2583_MAX_LUX_TABLE_ENTRIES * 3 + 1];
	int ret = -EINVAL;
	unsigned int n;

	mutex_lock(&chip->als_mutex);

	get_options(buf, ARRAY_SIZE(value), value);

	/*
	 * We now have an array of ints starting at value[1], and
	 * enumerated by value[0].
	 * We expect each group of three ints is one table entry,
	 * and the last table entry is all 0.
	 */
	n = value[0];
	if ((n % 3) || n < 6 || n > max_ints) {
		dev_err(dev,
			"%s: The number of entries in the lux table must be a multiple of 3 and within the range [6, %d]\n",
			__func__, max_ints);
		goto done;
	}
	if ((value[n - 2] | value[n - 1] | value[n]) != 0) {
		dev_err(dev, "%s: The last 3 entries in the lux table must be zeros.\n",
			__func__);
		goto done;
	}

	memcpy(chip->als_settings.als_device_lux, &value[1],
	       value[0] * sizeof(value[1]));

	ret = len;

done:
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static IIO_CONST_ATTR(in_illuminance_calibscale_available, "1 8 16 111");
static IIO_CONST_ATTR_NAMED(in_illuminance_calibscale_available_ct819,
			   in_illuminance_calibscale_available,
			   "1 8 16 128");
static IIO_CONST_ATTR_NAMED(in_illuminance_calibscale_available_ct821,
			   in_illuminance_calibscale_available,
			   "1 2 4 8 16 32 64 140");
static IIO_CONST_ATTR(in_illuminance_integration_time_available,
		      "0.050 0.100 0.150 0.200 0.250 0.300 0.350 0.400 0.450 0.500 0.550 0.600 0.650");
static IIO_DEVICE_ATTR_RW(in_illuminance_input_target, 0);
static IIO_DEVICE_ATTR_WO(in_illuminance_calibrate, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_lux_table, 0);

static struct attribute *sysfs_attrs_ctrl[] = {
	&iio_const_attr_in_illuminance_calibscale_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	&iio_dev_attr_in_illuminance_input_target.dev_attr.attr,
	&iio_dev_attr_in_illuminance_calibrate.dev_attr.attr,
	&iio_dev_attr_in_illuminance_lux_table.dev_attr.attr,
	NULL
};

static struct attribute *ct819_sysfs_attrs_ctrl[] = {
	&iio_const_attr_in_illuminance_calibscale_available_ct819.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	NULL
};

static struct attribute *ct821_sysfs_attrs_ctrl[] = {
	&iio_const_attr_in_illuminance_calibscale_available_ct821.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	NULL
};

static const struct attribute_group tsl2583_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

static const struct attribute_group ct819_attribute_group = {
	.attrs = ct819_sysfs_attrs_ctrl,
};

static const struct attribute_group ct821_attribute_group = {
	.attrs = ct821_sysfs_attrs_ctrl,
};

static const struct iio_chan_spec tsl2583_channels[] = {
	{
		.type = IIO_LIGHT,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_LIGHT,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE) |
				      BIT(IIO_CHAN_INFO_INT_TIME),
	},
};

static int tsl2583_set_pm_runtime_busy(struct tsl2583_chip *chip, bool on)
{
	if (on)
		return pm_runtime_resume_and_get(&chip->client->dev);

	return pm_runtime_put_autosuspend(&chip->client->dev);
}

static void tsl2583_put_pm_after_error(struct tsl2583_chip *chip,
				       bool force_suspend)
{
	int ret;

	if (force_suspend)
		ret = pm_runtime_put_sync_suspend(&chip->client->dev);
	else
		ret = tsl2583_set_pm_runtime_busy(chip, false);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"failed to release runtime PM after I/O error: %d\n", ret);
}

static int tsl2583_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	bool force_suspend = false;
	int ret, pm_ret;

	ret = tsl2583_set_pm_runtime_busy(chip, true);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->als_mutex);

	if (chip->needs_reinit) {
		ret = tsl2583_chip_init_and_power_on(indio_dev);
		if (ret < 0) {
			force_suspend = true;
			goto read_done;
		}
	}

	ret = -EINVAL;
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_LIGHT) {
			ret = tsl2583_get_lux(indio_dev);
			if (ret < 0)
				goto read_done;

			/*
			 * From page 20 of the TSL2581, TSL2583 data
			 * sheet (TAOS134 − MARCH 2011):
			 *
			 * One of the photodiodes (channel 0) is
			 * sensitive to both visible and infrared light,
			 * while the second photodiode (channel 1) is
			 * sensitive primarily to infrared light.
			 */
			if (chan->channel2 == IIO_MOD_LIGHT_BOTH)
				*val = chip->als_cur_info.als_ch0;
			else
				*val = chip->als_cur_info.als_ch1;

			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_PROCESSED:
		if (chan->type == IIO_LIGHT) {
			ret = tsl2583_get_lux(indio_dev);
			if (ret < 0)
				goto read_done;

			*val = ret;
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type == IIO_LIGHT) {
			*val = chip->als_settings.als_gain_trim;
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (chan->type == IIO_LIGHT) {
			*val = chip->info->gainadj[chip->als_settings.als_gain].mean;
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_INT_TIME:
		if (chan->type == IIO_LIGHT) {
			*val = 0;
			*val2 = chip->als_settings.als_time * USEC_PER_MSEC;
			ret = IIO_VAL_INT_PLUS_MICRO;
		}
		break;
	default:
		break;
	}

read_done:
	mutex_unlock(&chip->als_mutex);

	if (ret < 0) {
		tsl2583_put_pm_after_error(chip, force_suspend);
		return ret;
	}

	/*
	 * Preserve the ret variable if the call to
	 * tsl2583_set_pm_runtime_busy() is successful so the reading
	 * (if applicable) is returned to user space.
	 */
	pm_ret = tsl2583_set_pm_runtime_busy(chip, false);
	if (pm_ret < 0)
		return pm_ret;

	return ret;
}

static int tsl2583_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	bool force_suspend = false;
	int ret;

	ret = tsl2583_set_pm_runtime_busy(chip, true);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->als_mutex);

	if (chip->needs_reinit) {
		ret = tsl2583_chip_init_and_power_on(indio_dev);
		if (ret < 0) {
			force_suspend = true;
			goto write_done;
		}
	}

	ret = -EINVAL;
	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type == IIO_LIGHT && val >= 250 && val <= 4000) {
			chip->als_settings.als_gain_trim = val;
			ret = 0;
		}
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (chan->type == IIO_LIGHT) {
			unsigned int i;

			for (i = 0; i < chip->info->num_gainadj; i++) {
				if (chip->info->gainadj[i].mean == val) {
					ret = tsl2583_reconfigure_gain(indio_dev, i,
								       &force_suspend);
					break;
				}
			}
		}
		break;
	case IIO_CHAN_INFO_INT_TIME:
		if (chan->type == IIO_LIGHT && !val &&
		    val2 >= 50 * USEC_PER_MSEC &&
		    val2 <= 650 * USEC_PER_MSEC &&
		    !(val2 % (50 * USEC_PER_MSEC))) {
			val2 /= USEC_PER_MSEC;
			ret = tsl2583_apply_time(indio_dev, val2, &force_suspend);
		}
		break;
	default:
		break;
	}

write_done:
	mutex_unlock(&chip->als_mutex);

	if (ret < 0) {
		tsl2583_put_pm_after_error(chip, force_suspend);
		return ret;
	}

	ret = tsl2583_set_pm_runtime_busy(chip, false);
	if (ret < 0)
		return ret;

	return ret;
}

static const struct iio_info tsl2583_info = {
	.attrs = &tsl2583_attribute_group,
	.read_raw = tsl2583_read_raw,
	.write_raw = tsl2583_write_raw,
};

static const struct iio_info ct819_iio_info = {
	.attrs = &ct819_attribute_group,
	.read_raw = tsl2583_read_raw,
	.write_raw = tsl2583_write_raw,
};

static const struct iio_info ct821_iio_info = {
	.attrs = &ct821_attribute_group,
	.read_raw = tsl2583_read_raw,
	.write_raw = tsl2583_write_raw,
};

static const struct tsl2583_chip_info tsl2583_chip_info = {
	.gainadj = tsl2583_gainadj,
	.num_gainadj = ARRAY_SIZE(tsl2583_gainadj),
	.iio_info = &tsl2583_info,
	.default_als_time = 100,
	.default_als_gain = 0,
	.gain_reg = TSL2583_GAIN,
	.integration_cycle_us = TSL2583_INTEGRATION_CYCLE_US,
};

/*
 * Apart from the LSCI calibration and the lux computation the CT819 behaves
 * like a plain TSL2581: it keeps the gain in the TSL258x gain register,
 * clears the ALS interrupt through the TSL258x special function and needs no
 * extra control or analog frontend programming.
 */
static const struct tsl2583_chip_info ct819_chip_info = {
	.gainadj = ct819_gainadj,
	.num_gainadj = ARRAY_SIZE(ct819_gainadj),
	.iio_info = &ct819_iio_info,
	.default_als_time = 400,
	.default_als_gain = 1,
	.apple = true,
	.gain_reg = TSL2583_GAIN,
	.cal_version = 2,
	.cal_record_size = CT819_CAL_RECORD_SIZE_V2,
	.cal_record_len = CT819_CAL_RECORD_SIZE_V2,
	.cal_gain_count = CT819_CAL_GAIN_COUNT,
	.cal_ratio_off = CT819_CAL_RATIO_OFF,
	.cal_ratio_scale = CT819_CAL_RATIO_SCALE,
	.integration_cycle_us = TSL2583_INTEGRATION_CYCLE_US,
	.counts_per_cycle = CT819_COUNTS_PER_CYCLE,
	.lux_max = CT819_LUX_MAX,
};

static const struct tsl2583_chip_info ct821_chip_info = {
	.gainadj = ct821_gainadj,
	.num_gainadj = ARRAY_SIZE(ct821_gainadj),
	.iio_info = &ct821_iio_info,
	.default_als_time = 500,
	.default_als_gain = 7,
	.apple = true,
	.gain_reg = CT821_GAIN,
	.int_clr_data_cmd = CT821_CMD_ALS_INT_CLR,
	.cntl_enable = CT821_CNTL_ENABLE,
	.als_config = true,
	.cal_version = 3,
	.cal_record_size = CT821_CAL_RECORD_SIZE_V3,
	.cal_record_len = CT821_CAL_RECORD_DATA_SIZE_V3,
	.cal_gain_count = CT821_CAL_GAIN_COUNT,
	.cal_gain_count_off = CT821_CAL_GAIN_COUNT_OFF,
	.cal_ratio_off = CT821_CAL_RATIO_OFF,
	.cal_ratio_scale = CT821_CAL_RATIO_SCALE,
	.integration_cycle_us = CT821_INTEGRATION_CYCLE_US,
	.lux_max = TSL2583_LUX_CALC_OVER_FLOW,
};

static int tsl2583_probe(struct i2c_client *clientp)
{
	int ret;
	struct tsl2583_chip *chip;
	struct iio_dev *indio_dev;

	if (!i2c_check_functionality(clientp->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&clientp->dev, "%s: i2c smbus byte data functionality is unsupported\n",
			__func__);
		return -EOPNOTSUPP;
	}

	indio_dev = devm_iio_device_alloc(&clientp->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->client = clientp;
	chip->info = device_get_match_data(&clientp->dev);
	if (!chip->info)
		chip->info = &tsl2583_chip_info;
	i2c_set_clientdata(clientp, indio_dev);

	mutex_init(&chip->als_mutex);

	ret = i2c_smbus_read_byte_data(clientp,
				       TSL2583_CMD_REG | TSL2583_CHIPID);
	if (ret < 0) {
		dev_err(&clientp->dev,
			"%s: failed to read the chip ID register\n", __func__);
		return ret;
	}

	if ((ret & TSL2583_CHIP_ID_MASK) != TSL2583_CHIP_ID) {
		dev_err(&clientp->dev, "%s: received an unknown chip ID %x\n",
			__func__, ret);
		return -EINVAL;
	}

	indio_dev->info = chip->info->iio_info;
	indio_dev->channels = tsl2583_channels;
	indio_dev->num_channels = ARRAY_SIZE(tsl2583_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = chip->client->name;

	/* Load defaults before exposing the IIO device to userspace. */
	tsl2583_defaults(chip);
	if (chip->info->apple) {
		ret = apple_als_load_calibration(chip);
		if (ret)
			return ret;
	}

	pm_runtime_enable(&clientp->dev);
	pm_runtime_set_autosuspend_delay(&clientp->dev,
					 TSL2583_POWER_OFF_DELAY_MS);
	pm_runtime_use_autosuspend(&clientp->dev);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&clientp->dev, "%s: iio registration failed\n",
			__func__);
		pm_runtime_disable(&clientp->dev);
		return ret;
	}

	dev_info(&clientp->dev, "Light sensor found.\n");

	return 0;
}

static void tsl2583_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct tsl2583_chip *chip = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	tsl2583_set_power_state(chip, TSL2583_CNTL_PWR_OFF);
}

static int tsl2583_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	ret = tsl2583_set_power_state(chip, TSL2583_CNTL_PWR_OFF);

	mutex_unlock(&chip->als_mutex);

	return ret;
}

static int tsl2583_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tsl2583_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	ret = tsl2583_chip_init_and_power_on(indio_dev);

	mutex_unlock(&chip->als_mutex);

	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(tsl2583_pm_ops, tsl2583_suspend,
				 tsl2583_resume, NULL);

static const struct i2c_device_id tsl2583_idtable[] = {
	{ .name = "tsl2580" },
	{ .name = "tsl2581" },
	{ .name = "tsl2583" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tsl2583_idtable);

static const struct of_device_id tsl2583_of_match[] = {
	{ .compatible = "apple,ct819", .data = &ct819_chip_info },
	{ .compatible = "apple,ct821", .data = &ct821_chip_info },
	{ .compatible = "amstaos,tsl2580", .data = &tsl2583_chip_info },
	{ .compatible = "amstaos,tsl2581", .data = &tsl2583_chip_info },
	{ .compatible = "amstaos,tsl2583", .data = &tsl2583_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, tsl2583_of_match);

/* Driver definition */
static struct i2c_driver tsl2583_driver = {
	.driver = {
		.name = "tsl2583",
		.pm = pm_ptr(&tsl2583_pm_ops),
		.of_match_table = tsl2583_of_match,
	},
	.id_table = tsl2583_idtable,
	.probe = tsl2583_probe,
	.remove = tsl2583_remove,
};
module_i2c_driver(tsl2583_driver);

MODULE_AUTHOR("J. August Brenner <jbrenner@taosinc.com>");
MODULE_AUTHOR("Brian Masney <masneyb@onstation.org>");
MODULE_DESCRIPTION("TAOS tsl2583 ambient light sensor driver");
MODULE_LICENSE("GPL");
