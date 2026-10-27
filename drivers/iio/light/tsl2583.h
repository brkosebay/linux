/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _TSL2583_H
#define _TSL2583_H

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/overflow.h>
#include <linux/time64.h>
#include <linux/types.h>

/*
 * Fixed-point scale of the per-gain-step ratios in an Apple LSCI ambient-light
 * calibration record. The record stores the sensitivity of one gain step
 * relative to the previous one, so the factors have to be folded together.
 */
#define CT819_CAL_RATIO_SCALE 256
#define CT821_CAL_RATIO_SCALE 255

static inline int apple_als_calculate_gain_factor(u32 previous, u16 ratio,
						  u32 scale, u32 *next)
{
	u64 value;

	if (!previous || !ratio || !scale)
		return -EINVAL;

	value = DIV_ROUND_CLOSEST_ULL((u64)previous * ratio, scale);
	if (!value || value > U32_MAX)
		return -ERANGE;

	*next = value;

	return 0;
}

/* Largest lux value any of the parts reports */
#define TSL2583_LUX_CALC_OVER_FLOW	65535

/*
 * Apple pairs the TSL258x register interface with its own factory LSCI
 * calibration record. The record layout is versioned; every version shares
 * the same 8 byte header and the same fixed-point scale for the lux
 * coefficients, but differs in the record size and in the number of
 * calibrated gain steps.
 */
#define APPLE_ALS_CAL_SCALE		65536
#define APPLE_ALS_LUX_FUDGE_SCALE	255
#define APPLE_ALS_MAX_GAIN_COUNT	8

/* ADC counts per integration cycle of the CT819 (a TSL2581 clone) */
#define CT819_COUNTS_PER_CYCLE		1024

struct apple_als_calibration {
	u16 lux_fudge;
	u16 integration_time;
	u8 gain;
	s32 a_coeff1;
	s32 b_coeff1;
	s32 upper_range;
	s32 a_coeff2;
	s32 b_coeff2;
	u8 ch0_dark_counts;
	u8 ch1_dark_counts;
	u32 gain_factor[APPLE_ALS_MAX_GAIN_COUNT][2];
};

/*
 * Scale a raw channel reading back to the integration time and gain the
 * factory calibration was taken at.
 */
static inline int apple_als_normalize_counts(const struct apple_als_calibration *cal,
					     unsigned int gain_index, u32 cycles,
					     u32 cycle_us, u16 raw, int channel,
					     u32 *normalized)
{
	u64 value;
	u32 calibration_gain;
	u32 current_gain;
	u32 calibration_cycles;

	if (!cycle_us || gain_index >= APPLE_ALS_MAX_GAIN_COUNT)
		return -EINVAL;

	calibration_cycles = cal->integration_time * USEC_PER_MSEC / cycle_us;
	current_gain = cal->gain_factor[gain_index][channel];
	calibration_gain = cal->gain_factor[cal->gain][channel];
	if (!calibration_cycles || !cycles || !current_gain || !calibration_gain)
		return -EINVAL;

	value = (u64)raw * calibration_cycles * calibration_gain;
	value = DIV_ROUND_CLOSEST_ULL(value, (u64)cycles * current_gain);
	*normalized = min_t(u64, value, U32_MAX);

	return 0;
}

/**
 * apple_als_calc_lux - lux from raw ADC counts and an Apple LSCI record
 * @cal:	parsed calibration record
 * @gain_index:	gain step the counts were taken at
 * @cycles:	integration cycles the counts were taken over
 * @cycle_us:	length of one integration cycle
 * @counts_per_cycle: the ADC's counts per cycle, 0 if only 16 bits limit it
 * @trim:	user gain trim, 1000 is unity
 * @lux_max:	the largest value to report; a saturated channel reports it
 * @ch0:	counts of the visible + infrared channel
 * @ch1:	counts of the infrared channel
 *
 * The counts are first scaled to the record's reference integration time and
 * gain and the dark counts are taken off. Then lux is the two-segment
 * a * ch0 - b * ch1 curve (coefficients in Q16), times fudge / 255. A channel
 * at its ADC limit (@counts_per_cycle per cycle, 16 bits at most) is clipped,
 * so its ratio is meaningless: report @lux_max.
 *
 * Return: lux, or a negative error for a record or setting that can't be used.
 */
static inline int apple_als_calc_lux(const struct apple_als_calibration *cal,
				     unsigned int gain_index, u32 cycles,
				     u32 cycle_us, u32 counts_per_cycle, u32 trim,
				     u32 lux_max, u16 ch0, u16 ch1)
{
	s64 lux_scaled;
	s32 a_coeff;
	s32 b_coeff;
	u64 lux;
	u32 norm_ch0;
	u32 norm_ch1;
	u32 saturation;

	if (!cycles)
		return -EINVAL;

	saturation = U16_MAX;
	if (counts_per_cycle)
		saturation = min_t(u32, saturation, counts_per_cycle * cycles - 1);
	if (ch0 >= saturation || ch1 >= saturation)
		return lux_max;

	if (apple_als_normalize_counts(cal, gain_index, cycles, cycle_us, ch0, 0,
				       &norm_ch0) ||
	    apple_als_normalize_counts(cal, gain_index, cycles, cycle_us, ch1, 1,
				       &norm_ch1))
		return -EINVAL;
	norm_ch0 = max_t(u32, norm_ch0, cal->ch0_dark_counts) -
		   cal->ch0_dark_counts;
	norm_ch1 = max_t(u32, norm_ch1, cal->ch1_dark_counts) -
		   cal->ch1_dark_counts;

	if (!norm_ch0)
		return 0;

	if ((u64)norm_ch1 * APPLE_ALS_CAL_SCALE <=
	    (u64)norm_ch0 * cal->upper_range) {
		a_coeff = cal->a_coeff1;
		b_coeff = cal->b_coeff1;
	} else {
		a_coeff = cal->a_coeff2;
		b_coeff = cal->b_coeff2;
	}

	lux_scaled = (s64)norm_ch0 * a_coeff - (s64)norm_ch1 * b_coeff;
	if (lux_scaled <= 0)
		return 0;

	if (check_mul_overflow((u64)lux_scaled, (u64)cal->lux_fudge, &lux))
		return lux_max;
	lux = div_u64(lux, APPLE_ALS_CAL_SCALE * APPLE_ALS_LUX_FUDGE_SCALE);
	if (check_mul_overflow(lux, (u64)trim, &lux))
		return lux_max;
	lux = div_u64(lux, 1000);

	return min_t(u64, lux, lux_max);
}

#endif /* _TSL2583_H */
