/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * GPU overclock record (boot_gpu_oc_mhz): docs/power/ds-gpu-dvfs.md §3-4, §9 R5;
 * docs/gpu/experiments/clock-mechanisms.md §2.  Host test: scripts/gpu/perf60-host/test_oc.c.
 *
 * The hardware applies PSINFO record n (PMGR+0x70000 + 0x10*n, written by LLB 180395fd8 for
 * slots 0-4 once per SoC reset, volatile) when Fender is asked for pstate n.  Word 0 = core-rail
 * DAC code [31:24] | PLL valid [21] | multiplier [20:12] | predivider [8:4] | postdivider-1 [3:0];
 * word 1 [7:0] = buck8 code; word 2 = timing selectors.
 *
 * PLL: VCO = 24 MHz * mult / prediv, f = VCO / postdiv.  Apple's GPU-domain (0x17) descriptors
 * (LLB 0x1803bc718, all three chip sections) run VCOs from 800 to 2160 MHz (2160 = id 0x20 of
 * section 0x2f0001ff, 90/1/3); the live states use 948..1650.  The overclock keeps postdiv 2 (every
 * live state at or above 474 MHz uses 2 or 3) and the VCO inside Apple's range: f <= 1080 MHz.
 * Apple's own descriptors at 804 (67/1/2), 850 (425/6/2), 900 (75/1/2); elsewhere record 4's
 * dividers (prediv 4, postdiv 2: f = 3 MHz * mult) on a 3 MHz grid.
 *
 * Voltage: record 5 keeps record 4's timing word; its core (rail 1, PMIC 0x11, 450 mV + 3.125 mV *
 * code, GPU only) and buck8 (rail 8, PMIC 0x18, 600 mV + 3.125 mV * code) codes default to record
 * 4's (138 = 881.25 mV, 124 = 987.5 mV), never lower, raised only on request in whole 3.125 mV
 * codes,
 * and never above the ceilings: the boot parameters (default state 4 + 50 mV, the CPU overclock's
 * rail-offset cap) bounded by the compiled maxima below (state 4 + 150 mV; raising those needs a
 * rebuild).  The encoding itself reaches 1296 mV on both rails; that is not an operating point.
 */
#ifndef G5P_GPU_OC_H
#define G5P_GPU_OC_H

#include "bits-compat.h"

#define G5P_PSINFO_BASE 0x70000U /* PMGR syscon offset */
#define G5P_PSINFO_REC(n) (G5P_PSINFO_BASE + 0x10U * (n))
#define G5P_OC_STATE 5U
#define G5P_OC_PIN 500U /* boot_gpu_perf for state 5 */
#define G5P_OC_STOCK4 \
	0x8a2f1041U /* record 4 word 0 as LLB writes it on this chip */
#define G5P_OC_STOCK4_BUCK8 124U /* record 4 word 1 [7:0] */
#define G5P_OC_MIN_MHZ 726U /* the first grid step above 723 */
#define G5P_OC_VCO_MAX 2160U /* Apple's highest GPU-domain VCO */
#define G5P_OC_MAX_MHZ (G5P_OC_VCO_MAX / 2) /* postdiv 2: 1080 */
#define G5P_OC_VCO_STOCK_MAX \
	1650U /* the highest VCO of a live state (550 MHz) */
#define G5P_OC_VALID BIT(21)
#define G5P_OC_PLL_MASK 0x001ff1ffU /* mult, prediv, postdiv-1 */
#define G5P_OC_UV_STEP 3125U
#define G5P_OC_CORE_BASE_UV 450000U
#define G5P_OC_BUCK8_BASE_UV 600000U
/* 881250 */
#define G5P_OC_CORE_STOCK_UV \
	(G5P_OC_CORE_BASE_UV + G5P_OC_UV_STEP * (G5P_OC_STOCK4 >> 24))
/* 987500 */
#define G5P_OC_BUCK8_STOCK_UV \
	(G5P_OC_BUCK8_BASE_UV + G5P_OC_UV_STEP * G5P_OC_STOCK4_BUCK8)
/* the boot parameters' default ceiling: P4 + 50 mV */
#define G5P_OC_DEFAULT_HEADROOM_UV 50000U
#define G5P_OC_ABS_HEADROOM_UV \
	150000U /* compiled: P4 + 150 mV, no parameter goes above */

struct g5p_oc_pll {
	u32 mhz, mult, prediv, postdiv;
	bool llb; /* Apple's own descriptor */
};

static const struct g5p_oc_pll g5p_oc_llb[] = {
	{ 804, 67, 1, 2, true }, /* LLB id 0x22, all three GPU sections */
	{ 850, 425, 6, 2, true }, /* LLB id 0x23, the live section */
	{ 900, 75, 1, 2, true }, /* LLB id 0x24, the live section */
};

/*
 * The PLL for a requested MHz: Apple's descriptor at 804/850/900 (the request or its 3 MHz
 * rounding),
 * else the 3 MHz grid (rounded down) with record 4's dividers; false outside 726..1080.
 */
static inline bool g5p_oc_pll(u32 want, struct g5p_oc_pll *p)
{
	u32 mhz = want / 3 * 3;

	for (u32 i = 0; i < ARRAY_SIZE(g5p_oc_llb); i++)
		if (want == g5p_oc_llb[i].mhz || mhz == g5p_oc_llb[i].mhz) {
			*p = g5p_oc_llb[i];
			return true;
		}
	if (mhz < G5P_OC_MIN_MHZ || mhz > G5P_OC_MAX_MHZ)
		return false;
	*p = (struct g5p_oc_pll){ mhz, mhz / 3, 4, 2, false };
	return true;
}

/* A rail voltage as a code: exact multiples of 3.125 mV above the base only. */
static inline bool g5p_oc_code(u32 uv, u32 base, u32 *code)
{
	if (uv < base || (uv - base) % G5P_OC_UV_STEP ||
	    (uv - base) / G5P_OC_UV_STEP > 0xff)
		return false;
	*code = (uv - base) / G5P_OC_UV_STEP;
	return true;
}

/* The voltage codes for a request (0 = state 4's), bounded below by state 4 and above by the
 * ceilings (each already clamped to the compiled maximum by g5p_oc_ceiling()).
 */

/* Record 5's words 0 and 1 from the live record 4 (stock only), a PLL and the voltage codes. */
static inline bool g5p_oc_words(const u32 r4[4], u32 want, u32 core, u32 buck8,
				u32 w[4], struct g5p_oc_pll *p)
{
	if (r4[0] != G5P_OC_STOCK4 || (r4[1] & 0xff) != G5P_OC_STOCK4_BUCK8 ||
	    !g5p_oc_pll(want, p) || core < (G5P_OC_STOCK4 >> 24) ||
	    core > 0xff || buck8 < G5P_OC_STOCK4_BUCK8 || buck8 > 0xff)
		return false;
	w[0] = core << 24 | G5P_OC_VALID | p->mult << 12 | p->prediv << 4 |
	       (p->postdiv - 1);
	w[1] = (r4[1] & ~0xffU) | buck8;
	w[2] = r4[2];
	w[3] = r4[3];
	return true;
}

/* Hz of a record word 0, as iBoot 87001c3ac decodes it (0 if the predivider is 0). */
static inline u64 g5p_oc_hz(u32 w0)
{
	u32 mult = (w0 >> 12) & 0x1ff, prediv = (w0 >> 4) & 0x1f,
	    postdiv = (w0 & 0xf) + 1;

	return prediv ? 24000000ULL * mult / prediv / postdiv : 0;
}

/* A nonzero record 5 that this driver writes: record 4's timing and word-3 words, valid, a PLL
 * g5p_oc_pll() produces, voltage codes between state 4's and the compiled maxima.  Anything else in
 * slot 5 is not ours: never overwritten or cleared.
 */
static inline bool g5p_oc_is_ours(const u32 r4[4], const u32 r5[4])
{
	u32 core = r5[0] >> 24, buck8 = r5[1] & 0xff, w[4], cmax, bmax;
	struct g5p_oc_pll p;
	u64 hz = g5p_oc_hz(r5[0]);

	if (!g5p_oc_code(G5P_OC_CORE_STOCK_UV + G5P_OC_ABS_HEADROOM_UV,
			 G5P_OC_CORE_BASE_UV, &cmax) ||
	    !g5p_oc_code(G5P_OC_BUCK8_STOCK_UV + G5P_OC_ABS_HEADROOM_UV,
			 G5P_OC_BUCK8_BASE_UV, &bmax) ||
	    core > cmax || buck8 > bmax || !hz || hz % 1000000ULL ||
	    !g5p_oc_words(r4, (u32)(hz / 1000000ULL), core, buck8, w, &p))
		return false;
	return w[0] == r5[0] && w[1] == r5[1] && w[2] == r5[2] && w[3] == r5[3];
}

#endif
