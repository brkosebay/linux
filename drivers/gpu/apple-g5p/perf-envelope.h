/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * perf60 clock: the inputs of the firmware's policy-state writer (0x10008218) and of its governor,
 * so the firmware itself moves the GPC from pstate 1 to q = pin / 100.  Both compute
 *	x = min(policy+0x6f0, policy+0x688, ctrl+0x102c)
 * (META CS is unsigned lower; iOS's live policy+0x38 equals this min in every sample), store
 * policy+8 = 0 when x % 100 == 0 (2 when x < 100), and request pstate q = x / 100 (policy+0x28).
 * +0x6f0 and +0x688 are recomputed by the firmware from its coefficient block unless their
 * overrides +0x6f4 / +0x68c are nonzero, and the firmware itself reloads those overrides from
 * the control object on every pass, whatever the record held: +0x6f4 = ctrl+0x1040 (0x1000a1e4,
 * 0x1000b388, 0x1000c568) and +0x68c = ctrl+0x1038 (0x1000a74c, 0x1000b10c, 0x1000c2fc).  iOS
 * leaves both control words 0 and arms the sampler (policy+4 = 16); this boot pins them
 * (ctrl+0x1040 = pin, ctrl+0x1038 = 400) and leaves the sampler off (policy+4 = 0), so there is
 * no idle parking at pstate 0.  The pstate table and limits are iOS's (live j71t capture,
 * 2026-11-01): meta+0x710/0x714 = 4/5, MHz {0, 340, 474, 550, 723}, ctrl+0x1028/0x1030 =
 * 3348/400; the floor ctrl+0x102c is the pin.  RAM only, before release.
 */
#ifndef APPLE_G5P_PERF_ENVELOPE_H
#define APPLE_G5P_PERF_ENVELOPE_H

#include "bits-compat.h"
#define G5P_PERF_POLICY 0x4aU
#define G5P_PERF_META 0x4dU
#define G5P_PERF_CTL 0x5eU
#define G5P_PERF_TOP 400U

static const u32 g5p_perf_mhz[] = { 0, 340, 474, 550, 723 };

static inline bool g5p_perf_valid(u32 pin)
{
	return pin <= G5P_PERF_TOP && !(pin % 100);
}

static inline void g5p_perf_install(const struct g5p_boot_layout *p, u8 *root,
				    u8 *shared, u32 pin)
{
	u8 *policy = g5p_object(p, root, shared, G5P_PERF_POLICY);
	u8 *meta = g5p_object(p, root, shared, G5P_PERF_META);
	u8 *ctl = g5p_object(p, root, shared, G5P_PERF_CTL);
	u32 i;

	g5p_put32(meta + 0x710, ARRAY_SIZE(g5p_perf_mhz) - 1);
	g5p_put32(meta + 0x714, ARRAY_SIZE(g5p_perf_mhz));
	for (i = 0; i < ARRAY_SIZE(g5p_perf_mhz); i++)
		g5p_put32(meta + 0x718 + 4 * i, g5p_perf_mhz[i]);
	g5p_put32(ctl + 0x1028, 3348);
	g5p_put32(ctl + 0x102c, pin);
	g5p_put32(ctl + 0x1030, G5P_PERF_TOP);
	g5p_put32(ctl + 0x1038, G5P_PERF_TOP);
	g5p_put32(ctl + 0x1040, pin);
	g5p_put32(policy + 0x688, G5P_PERF_TOP);
	g5p_put32(policy + 0x68c, G5P_PERF_TOP);
	g5p_put32(policy + 0x6f0, pin);
	g5p_put32(policy + 0x6f4, pin);
}

/* boot_gpu_oc_mhz (gpu-oc.h): state 5 = the overclock record; the firmware's table, ceilings and
 * policy limits extended to index 5 so the pin 500 and the power-on request of state 5 are in range
 * (the firmware derives its clock entry priv+0x2c4 = meta+0x718[state] from Fender, 0x1001c838).
 */

/*
 * boot_gpu_perf_seed (perf60-clock.md "Clock-change drain RE" §3): in policy state 0 the
 * firmware
 * never changes pstate while powered; only its boot sampler (0x1000aa80, before ALIVE) moves the
 * GPC, when policy+0x28 != Fender & 0xf.  Seeding the policy record with the firmware's own
 * steady-state output for the pin, +0x28 = +0x2c = q, +0x30 = 0, +8 = 0, makes that sampler
 * request q through its setter (0x1001c9b8 -> 0x1001c88c: pol34 = q, R1+0x1000 = q | 0x10),
 * which is
 * iOS's park -> reboot -> sampler path.  q = pin / 100 for pins 200..400, else 0 (no seed).
 */
static inline u32 g5p_perf_seed_q(u32 pin)
{
	return pin >= 200 && pin <= G5P_PERF_TOP && !(pin % 100) ? pin / 100 :
								   0;
}

/* Seed 2 adds what the sampler needs to get past its first-call init: 0x1000aa80 at 0x1000abb0
 * takes policy+0 == 0 as "never sampled" and only initialises (0x1000ae88: +0 = 1, its time words)
 * without reaching the target compare at 0x1000ada4 or the setter; its sampling path divides by
 * policy+4 (0x10022f88 at 0x1000ac28).  The boot call at 0x10000b54 is its only caller, so a fresh
 * record never moves Fender.  iOS's record persists across firmware reboots: +0 = 1 and +4 = 16 in
 * every sample of the 2026-11-01 capture (idle and load).  Seed 2 writes those two as iOS has them.
 * Seed 2 on hardware (2026-11-03): no ALIVE, Fender 0.  With +0 = 1 the sampler takes its sampling
 * path when (now - policy+0x8f8) / 24000 / policy+4 >= 1 (0x1000ac10..ac5c): +0x8f8 is the host's
 * map-time stamp, ~3 s old at release, so it runs the load governor (0x1000afa4 -> 0x1000b050, busy
 * accumulators policy+0x62c..0x68c, all 0 on a fresh record) and lands on pstate 0.  Seed 3
 * (boot.inc
 * g5p_boot_release) restamps +0x8f8 right before the release stream: < 1 period elapsed, the load
 * update is skipped (0x1000ac64 -> 0x1000acb0 -> 0x1000ada4) and the compare against +0x28 = q
 * reaches the setter (0x1000ae68 -> 0x1001c9b8).
 */
static inline void g5p_perf_seed(const struct g5p_boot_layout *p, u8 *root,
				 u8 *shared, u32 q, bool ios_record)
{
	u8 *policy = g5p_object(p, root, shared, G5P_PERF_POLICY);

	g5p_put64(policy + 0x28, (u64)q << 32 | q);
	g5p_put32(policy + 0x30, 0);
	g5p_put32(policy + 8, 0);
	if (ios_record) {
		g5p_put32(policy + 0, 1);
		g5p_put32(policy + 4, 16);
	}
}

#endif
