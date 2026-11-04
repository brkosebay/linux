/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Latency123: firmware words a powered iOS GPU has and our bootstrap leaves zero
 * (docs/gpu/firmware-re/wake-latency-re.md sections 3.4 and 4.2).
 *
 * Arm A.  The firmware copies meta+0x6c0..0x758 into its private block once, at init
 * (MINIM 0x100009ac -> 0x10003ca4), so these words only act from the boot image:
 *   meta+0x6d4 = 1           per-tick cache maintenance 0x1001b638 (688/1720/e230/e138)
 *   meta+0x710/0x714         pstate table max index / count
 *   meta+0x718..             pstate MHz; the tick period is table[Fender+0x1000 & 0xf]
 *   policy+0x4 = 16          governor sample period, ms (the governor reads it live)
 * 1 = capped: table { 0, 340 }, max index 1.  The firmware's own pstate setter
 *     (0x1001c88c, Fender+0x1000 = index | 0x10) can then only ask for pstate 1, the state
 *     the host already requests, so the clock does not move.
 * 2 = full: table { 0, 340, 474, 550, 723 }, max index 4, plus the limits README rank 2
 *     pairs with it (control+0x1028/0x102c/0x1030 = 3348/400/400, policy+0x7cc/0x6fc =
 *     400).  The firmware may then raise the GPU clock itself; nothing on Linux programs
 *     a GPU voltage table, so this arm runs after the capped one.
 *
 * Arm B.  ctl+0x04 is the per-engine tick-service mask (iOS 7).  Every tick re-copies
 * ctl+0x00..0x20 (0x10001328), so the word is live; the boot image gets the value the
 * parameter holds at map and the render path re-applies it before each job.
 *
 * RAM only.  Temporary A/B code: the winning words become unconditional init writes.
 */
#ifndef APPLE_G5P_LATENCY123_H
#define APPLE_G5P_LATENCY123_H

#include "bits-compat.h"
#define G5P_L123_POLICY 0x4aU
#define G5P_L123_META 0x4dU
#define G5P_L123_CTL 0x5eU
#define G5P_L123_TICK_MASK 7U
#define G5P_L123_CAPPED 1U
#define G5P_L123_FULL 2U

static const u32 g5p_l123_mhz[] = { 0, 340, 474, 550, 723 };

static inline void g5p_l123_arm_a(const struct g5p_boot_layout *p, u8 *root,
				  u8 *shared, u32 arm)
{
	u8 *meta = g5p_object(p, root, shared, G5P_L123_META);
	u8 *policy = g5p_object(p, root, shared, G5P_L123_POLICY);
	u8 *ctl = g5p_object(p, root, shared, G5P_L123_CTL);
	u32 count = arm == G5P_L123_FULL ? ARRAY_SIZE(g5p_l123_mhz) : 2;
	u32 i;

	g5p_put32(meta + 0x6d4, 1);
	g5p_put32(meta + 0x710, count - 1);
	g5p_put32(meta + 0x714, count);
	for (i = 0; i < count; i++)
		g5p_put32(meta + 0x718 + 4 * i, g5p_l123_mhz[i]);
	g5p_put32(policy + 4, 16);
	if (arm != G5P_L123_FULL)
		return;
	g5p_put32(ctl + 0x1028, 3348);
	g5p_put32(ctl + 0x102c, 400);
	g5p_put32(ctl + 0x1030, 400);
	g5p_put32(policy + 0x7cc, 400);
	g5p_put32(policy + 0x6fc, 400);
}

#endif
