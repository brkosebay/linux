/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The GPU register licence (docs/gpu/experiments/mmio-access.md §4), pure so the host test
 * (scripts/gpu/perf60-host/test_mmio.c) drives the same code.  iOS's
 * AGXMetaFirmware_isGPURegisterAccessible (fffffff006ca10ec) licenses an R0 access when PMGR gfx
 * is on and the wrapper is up (fender+0xc), R1+0 bit 29 is clear (the firmware's power-down /
 * clock-change routine 0x1001cec8 raises it before the GPC goes to 0) and the GPC pstate nibble is
 * 1..meta+0x710.  We are stricter: R1+0x1000 bit 4 (a transition in flight) must be clear and
 * R1+0 == 0 (a dead wrapper, Apple's ca05d4 vt+0x70 branch) refuses too.
 *
 * Order, and nothing else touches the bus: the RAM terms (no MMIO when any fails), then R1+0, then
 * R1+0x1000, then at most G5P_MMIO_MAX_OPS R0 accesses, then R1+0x1000 and R1+0 again at release
 * to account a sample whose licence changed under it (stale: data loss, not exclusion).
 * A refusal never waits and never wakes (no R1+0x14 write): the caller decides whether to retry.
 */
#ifndef G5P_MMIO_LICENCE_H
#define G5P_MMIO_LICENCE_H

#include "bits-compat.h"

#define G5P_MMIO_MAX_OPS 8U /* R0 accesses per ticket */
#define G5P_MMIO_R1_CTRL \
	0x0U /* R1+0: bit 29 = firmware power-down in progress */
#define G5P_MMIO_R1_GPC \
	0x1000U /* R1+0x1000: bit 4 transition, nibble = pstate */
#define G5P_MMIO_BIT29 BIT(29)

enum g5p_mmio_why {
	G5P_MMIO_OK,
	G5P_MMIO_OFF, /* RAM: power epoch not readable (no MMIO done) -> -ENODEV */
	G5P_MMIO_BUSY, /* the lock was held (try mode, IRQ census) -> -EAGAIN, no MMIO */
	G5P_MMIO_DEAD, /* R1+0 == 0 */
	G5P_MMIO_FWOFF, /* R1+0 bit 29 */
	G5P_MMIO_TRANS, /* R1+0x1000 bit 4 */
	G5P_MMIO_PARKED, /* nibble 0 */
	G5P_MMIO_OVERMAX, /* nibble > max pstate */
	G5P_MMIO_PARKEPOCH, /* R0 in a park epoch (g5p_mmio_r0_regime_ok) */
	G5P_MMIO_NR,
};

static const char *const g5p_mmio_why_names[G5P_MMIO_NR] = {
	"read",	       "skip-off",     "skip-busy",
	"skip-dead",   "skip-bit29",   "skip-transition",
	"skip-parked", "skip-overmax", "skip-park-epoch",
};

/* The RAM preconditions (§4.2 step 2).  b->failed is deliberately not one: the fault read runs
 * after it latches, as Apple's does.
 */
struct g5p_mmio_ram {
	bool powered, fender_ready, mmio_off, have_boot, released, stopped;
};

static inline bool g5p_mmio_ram_ok(const struct g5p_mmio_ram *r)
{
	return r->powered && r->fender_ready && !r->mmio_off && r->have_boot &&
	       r->released && !r->stopped;
}

static inline enum g5p_mmio_why g5p_mmio_check_r1_0(u32 r1_0)
{
	if (!r1_0)
		return G5P_MMIO_DEAD;
	if (r1_0 & G5P_MMIO_BIT29)
		return G5P_MMIO_FWOFF;
	return G5P_MMIO_OK;
}

static inline enum g5p_mmio_why g5p_mmio_check_gpc(u32 gpc, u32 max)
{
	if (gpc & 0x10)
		return G5P_MMIO_TRANS;
	if (!(gpc & 0xf))
		return G5P_MMIO_PARKED;
	if ((gpc & 0xf) > max)
		return G5P_MMIO_OVERMAX;
	return G5P_MMIO_OK;
}

/*
 * R0 only where the firmware cannot park under it.  The governor's park does not raise R1+0
 * bit 29 (mmio-access.md §2.3: only the power-down routine 0x1001cec8 does, a few calls before it
 * drops the GPC), so against a park the licence's R1 checks are a race: a check that passes can be
 * followed by the GPC going to 0 before the R0 load.  The never-park envelope (boot_gpu_perf >=
 * 100, latched at map: the policy writer's x >= 100, q >= 1, the governor never selects pstate 0)
 * has no such window; at pin 400/500 the 3D-completion and clock samples saw bit 29 0 times, while
 * the park tuple's rung 5 (2026-11-03) saw it on 749 of 983 3D completions and then froze the SoC.
 * iOS reads R0 only at a fault event, never at idle.  `override` = mmio_r0_park (DFU-class rungs
 * only).
 */
static inline bool g5p_mmio_r0_regime_ok(u32 gpu_perf, bool override)
{
	return override || gpu_perf >= 100;
}

/*
 * meta+0x710 as written into the boot image; 0 (no table) means the firmware stays at pstate 1.
 */
static inline u32 g5p_mmio_max_pstate(u32 meta710)
{
	return meta710 < 1 ? 1 : meta710 > 15 ? 15 : meta710;
}

typedef u32 (*g5p_mmio_rd_t)(void *ctx, u32 off);

/* §4.2 steps 2-4 under the caller's lock: no bus access unless the RAM terms pass, R1+0 before
 * R1+0x1000, nothing after a refusal.  *r1_0 / *gpc stay U32_MAX when not read.
 */
static inline enum g5p_mmio_why g5p_mmio_licence(const struct g5p_mmio_ram *ram,
						 u32 max, g5p_mmio_rd_t rd,
						 void *ctx, u32 *r1_0, u32 *gpc)
{
	enum g5p_mmio_why why;

	*r1_0 = 0xffffffffU;
	*gpc = 0xffffffffU;
	if (!g5p_mmio_ram_ok(ram))
		return G5P_MMIO_OFF;
	*r1_0 = rd(ctx, G5P_MMIO_R1_CTRL);
	why = g5p_mmio_check_r1_0(*r1_0);
	if (why)
		return why;
	*gpc = rd(ctx, G5P_MMIO_R1_GPC);
	return g5p_mmio_check_gpc(*gpc, max);
}

/* An R0 op is allowed while ops < G5P_MMIO_MAX_OPS; the counter counts refusals too. */
static inline bool g5p_mmio_op_take(u32 *ops)
{
	return (*ops)++ < G5P_MMIO_MAX_OPS;
}

/* §4.3 release: the licence fields (nibble, bit 4, bit 29) unchanged and still licensing. */
static inline bool g5p_mmio_stale(u32 r1_0, u32 gpc, u32 r1_0_end, u32 gpc_end,
				  u32 max)
{
	if ((gpc & 0x1f) != (gpc_end & 0x1f) ||
	    (r1_0 & G5P_MMIO_BIT29) != (r1_0_end & G5P_MMIO_BIT29))
		return true;
	return g5p_mmio_check_r1_0(r1_0_end) ||
	       g5p_mmio_check_gpc(gpc_end, max);
}

/* mmio_selftest rung 3/5 expectations: CORE_REVISION and CORE_ID as read at bring-up. */
#define G5P_MMIO_EXPECT_20 0x00090f00U
#define G5P_MMIO_EXPECT_18 0x011b0602U

#endif
