// SPDX-License-Identifier: GPL-2.0-or-later

#include <kunit/test.h>
#include <linux/module.h>

#include "tsl2583.h"

static void ct821_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 510,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 131072U, factor);
}

static void ct821_gain_factor_zero_test(struct kunit *test)
{
	u32 factor = 1;

	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(65536, 0,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, -EINVAL,
			apple_als_calculate_gain_factor(0, 255,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1U, factor);
}

static void ct821_gain_factor_overflow_test(struct kunit *test)
{
	u32 factor;

	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
	KUNIT_ASSERT_EQ(test, 1U << 24, factor);
	KUNIT_EXPECT_EQ(test, -ERANGE,
			apple_als_calculate_gain_factor(factor, 0xff00,
							CT821_CAL_RATIO_SCALE,
							&factor));
}

static void ct819_gain_factor_valid_test(struct kunit *test)
{
	u32 factor;

	/*
	 * The gain ratios of the factory record of an iPad 7 fold into the
	 * nominal 8x, 16x and 128x gain steps of the part.
	 */
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(65536, 2010,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 514560U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 521,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 1047210U, factor);
	KUNIT_ASSERT_EQ(test, 0,
			apple_als_calculate_gain_factor(factor, 1862,
							CT819_CAL_RATIO_SCALE,
							&factor));
	KUNIT_EXPECT_EQ(test, 7616816U, factor);
}

/*
 * A calibration with round numbers: 8x gain and 400 ms (148 cycles of
 * 2.7 ms) as the reference, a1/b1 = 1/2 up to a ch1/ch0 ratio of 0.4, a2/b2 =
 * 0.8/1.2 above it, fudge 204/255 = 0.8, and the iPad 7 gain ratios as above.
 */
static const struct apple_als_calibration ct819_test_cal = {
	.lux_fudge = 204,
	.integration_time = 400,
	.gain = 1,
	.a_coeff1 = 65536,
	.b_coeff1 = 131072,
	.upper_range = 26214,
	.a_coeff2 = 52429,
	.b_coeff2 = 78643,
	.gain_factor = {
		{ 65536, 65536 },
		{ 514560, 514560 },
		{ 1047210, 1047210 },
		{ 7616816, 7616816 },
	},
};

#define CT819_TEST_CYCLE_US	2700
#define CT819_TEST_REF_CYCLES	148
#define CT819_TEST_UNITY_TRIM	1000

static int ct819_test_lux(u32 cycles, unsigned int gain, u32 lux_max,
			  u16 ch0, u16 ch1)
{
	return apple_als_calc_lux(&ct819_test_cal, gain, cycles,
				  CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
				  CT819_TEST_UNITY_TRIM, lux_max, ch0, ch1);
}

static void ct819_lux_curve_test(struct kunit *test)
{
	/* ch1/ch0 = 0.1 uses the first segment: (1000 * 1 - 100 * 2) * 0.8 */
	KUNIT_EXPECT_EQ(test, 640,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 100));
	/* ch1/ch0 = 0.5 is past the limit: (1000 * 0.8 - 500 * 1.2) * 0.8 */
	KUNIT_EXPECT_EQ(test, 160,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 500));
	/* Mostly infrared light cannot go negative */
	KUNIT_EXPECT_EQ(test, 0,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 100, 1000));
	KUNIT_EXPECT_EQ(test, 0,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 0, 0));
}

static void ct819_lux_integration_time_test(struct kunit *test)
{
	/* The same light in 50 ms (18 cycles) gives 18/148 of the counts */
	KUNIT_EXPECT_EQ(test, 947,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1480, 148));
	KUNIT_EXPECT_EQ(test, 947,
			ct819_test_lux(18, 1, 5000, 180, 18));
}

static void ct819_lux_gain_test(struct kunit *test)
{
	/* 16x sees 1047210 / 514560 times the counts of 8x */
	KUNIT_EXPECT_EQ(test, 640,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 2, 5000, 2035, 204));
}

static void ct819_lux_saturation_test(struct kunit *test)
{
	/* 1024 counts per cycle, 16 bits at most */
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 65535, 100));
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 1000, 65535));
	KUNIT_EXPECT_EQ(test, 1000000, ct819_test_lux(18, 1, 1000000, 18431, 0));
	KUNIT_EXPECT_NE(test, 1000000, ct819_test_lux(18, 1, 1000000, 18430, 0));
	/* One count short of the limit still computes, then clamps */
	KUNIT_EXPECT_EQ(test, 48000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 65535, 60000, 0));
	KUNIT_EXPECT_EQ(test, 5000,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 1, 5000, 60000, 0));
}

static void apple_lux_16bit_limit_test(struct kunit *test)
{
	/* Without a per-cycle limit only the 16 bit range clips (the CT821) */
	KUNIT_EXPECT_EQ(test, 65535,
			apple_als_calc_lux(&ct819_test_cal, 1, 18, CT819_TEST_CYCLE_US,
					   0, CT819_TEST_UNITY_TRIM, 65535, 18431, 0));
	KUNIT_EXPECT_NE(test, 1000000,
			apple_als_calc_lux(&ct819_test_cal, 1, 18, CT819_TEST_CYCLE_US,
					   0, CT819_TEST_UNITY_TRIM, 1000000, 18431, 0));
}

static void ct819_lux_invalid_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, -EINVAL, ct819_test_lux(0, 1, 5000, 100, 10));
	KUNIT_EXPECT_EQ(test, -EINVAL,
			ct819_test_lux(CT819_TEST_REF_CYCLES, 8, 5000, 100, 10));
}

static void ct819_lux_dark_counts_test(struct kunit *test)
{
	struct apple_als_calibration cal = ct819_test_cal;

	cal.ch0_dark_counts = 4;
	cal.ch1_dark_counts = 2;
	/* (1004 - 4) and (102 - 2) are the counts of the first test */
	KUNIT_EXPECT_EQ(test, 640,
			apple_als_calc_lux(&cal, 1, CT819_TEST_REF_CYCLES,
					   CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
					   1000, 5000, 1004, 102));
	/* The dark counts of an unlit sensor leave nothing to report */
	KUNIT_EXPECT_EQ(test, 0,
			apple_als_calc_lux(&cal, 1, CT819_TEST_REF_CYCLES,
					   CT819_TEST_CYCLE_US, CT819_COUNTS_PER_CYCLE,
					   1000, 5000, 3, 2));
}

static struct kunit_case ct819_gain_factor_test_cases[] = {
	KUNIT_CASE(ct819_gain_factor_valid_test),
	{}
};

static struct kunit_suite ct819_gain_factor_test_suite = {
	.name = "ct819-gain-factor",
	.test_cases = ct819_gain_factor_test_cases,
};

static struct kunit_case ct819_lux_test_cases[] = {
	KUNIT_CASE(ct819_lux_curve_test),
	KUNIT_CASE(ct819_lux_integration_time_test),
	KUNIT_CASE(ct819_lux_gain_test),
	KUNIT_CASE(ct819_lux_saturation_test),
	KUNIT_CASE(apple_lux_16bit_limit_test),
	KUNIT_CASE(ct819_lux_invalid_test),
	KUNIT_CASE(ct819_lux_dark_counts_test),
	{}
};

static struct kunit_suite ct819_lux_test_suite = {
	.name = "ct819-lux",
	.test_cases = ct819_lux_test_cases,
};

static struct kunit_case ct821_gain_factor_test_cases[] = {
	KUNIT_CASE(ct821_gain_factor_valid_test),
	KUNIT_CASE(ct821_gain_factor_zero_test),
	KUNIT_CASE(ct821_gain_factor_overflow_test),
	{}
};

static struct kunit_suite ct821_gain_factor_test_suite = {
	.name = "ct821-gain-factor",
	.test_cases = ct821_gain_factor_test_cases,
};

kunit_test_suites(&ct819_gain_factor_test_suite,
		  &ct819_lux_test_suite,
		  &ct821_gain_factor_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple ALS calibration gain factors and lux computation");
MODULE_LICENSE("GPL");
