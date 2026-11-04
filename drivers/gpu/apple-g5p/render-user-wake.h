/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 publish-time MTS wake (wake110).  Pure functions over plain values; the one
 * register sequence goes through a callback table (the fender.h pattern), so the host
 * suite compiles this file as it is and records every access in order.
 *
 * Question (docs/gpu/experiments/wake110-apple-mts-dispatch.md): is the 0.5-33 s TA/3D
 * start latency caused by the missing Apple submit kick?  Apple's TA and 3D submit
 * wrappers (iPad6,11 kernelcache 0xfffffff006ca00a0 / 0xfffffff006ca01f8) do, every time:
 * ring entry, dmb ish, Fender vt+0x80 (R1+0x14 = 7, read back), Fender vt+0x18
 * (R1+0x1000 & 0xf), then R0+0xb00 = 0x10 (TA, DM0|COUNTED) or 0x11 (3D, DM1|COUNTED).
 * The MTS store is reached from every arm; when the GPC nibble is 0 Apple first runs a
 * power arm (up to forcePowerCycleAndReboot).  Linux V3 publishes job N >= 2's TA and
 * every 3D with no kick at all.
 *
 * render_user_wake (latched per context): 0 off (default: no call has any effect),
 * 1 the A/B, 2 every eligible phase is a factor (the candidate production fix, for use
 * only after the A/B proved it).  The TA of job 1 is never a phase of this file (the r74g
 * grace and its fenced fallback bell own it).  In the A/B each engine's phases are paired
 * in publish order, (0,1), (2,3), ...; exactly one of each pair is the factor, which one
 * is fixed by render_user_wake_seed before the context exists, and bit 31 of the seed
 * inverts the whole schedule (the mirror stage).  The verdict recomputes the arm of every
 * receipt from the seed.
 *
 * At the publish, right after the producer store and its barrier, both arms take the same
 * lock and read the same RAM state and run the same guards; the control stops there (no
 * register).  The factor then runs the Apple order: R1+0x14 = 7, read back (any bit
 * outside 7: stop), R1+0x1000 (state & 0x1f != 1, the GPC is not active: stop, never
 * repaired, never a power arm), dma_wmb(), R0+0xb00 = 0x10 | engine.  At most one attempt
 * per phase.  Never after a timeout or fault (the terminal latch's sticky `failed`), never
 * with the boot failed or stopped, never unpowered, never with an r74f bell in flight,
 * never once the phase shows any sign of retirement, never on a ring that is not exactly
 * this publish.  A consumer already at the target does not cancel (Apple never checks
 * it); it is recorded.  The natural TA completion (terminal kind COMPLETE, the epoch's
 * first note) is not a failure: the factor's sequence is the one licensed exception to the
 * r96 "no register after the TA complete" envelope, default off.
 */
#ifndef G5P_RENDER_USER_WAKE_H
#define G5P_RENDER_USER_WAKE_H

#ifndef G5P_GPC_ON
#define G5P_GPC_ON(raw) (((raw) & 0x1f) == 1) /* core.inc overrides it */
#endif

#define G5P_WK_OFF 0U
#define G5P_WK_AB 1U
#define G5P_WK_ALWAYS 2U
#define G5P_WK_MIRROR 0x80000000U /* seed bit 31: every arm inverted */

#define G5P_WK_R1_FENDER 0x14U /* Fender vt+0x80: write 7, read back */
#define G5P_WK_R1_GPC 0x1000U /* GPC state: & 0x1f == 1 is active */
#define G5P_WK_R0_MTS 0xb00U /* MTS_SCHEDULE, write only */
#define G5P_WK_MTS_TA 0x10U /* DM0 | COUNTED */
#define G5P_WK_MTS_3D 0x11U /* DM1 | COUNTED */

/* wake111: the publish-time power evidence, one bit per RAM fact, sampled under the event lock
 * and printed per phase (power=TT/DD).  Every required bit must be set.  reset_transition is
 * the permanent post-release slave-port lockout (set right before the release, never
 * cleared), not a live reset state: Wake110 vetoed on it, so every phase skipped.  It is
 * recorded (SLAVE_LOCK) and is neither required nor a veto; the slave-port ban is unchanged.
 */
#define G5P_WK_PWR_CORE 0x01U /* g->powered */
#define G5P_WK_PWR_FENDER 0x02U /* g->fender_ready */
#define G5P_WK_PWR_RELEASED 0x04U /* b->released */
#define G5P_WK_PWR_ALIVE 0x08U /* b->alive */
#define G5P_WK_PWR_CONTROL 0x10U /* b->control_complete */
#define G5P_WK_PWR_DS108 \
	0x20U /* DS108 claim proof, current generation, no power abort */
#define G5P_WK_PWR_SLAVE_LOCK 0x40U /* g->reset_transition: evidence only */
#define G5P_WK_PWR_REQUIRED                                          \
	(G5P_WK_PWR_CORE | G5P_WK_PWR_FENDER | G5P_WK_PWR_RELEASED | \
	 G5P_WK_PWR_ALIVE | G5P_WK_PWR_CONTROL | G5P_WK_PWR_DS108)

enum g5p_wk_arm {
	G5P_WK_ARM_NONE, /* wake off */
	G5P_WK_ARM_EXCLUDED, /* the TA of job 1 (r74g) */
	G5P_WK_ARM_CONTROL,
	G5P_WK_ARM_FACTOR,
	G5P_WK_ARMS,
};

enum g5p_wk_did {
	G5P_WK_DID_NONE, /* off, or never reached */
	G5P_WK_DID_RANG, /* factor: fence, GPC active, 0xb00 written */
	G5P_WK_DID_SHAM, /* control: every guard passed, no register */
	G5P_WK_DID_EXCLUDED,
	G5P_WK_SKIP_TWICE, /* this phase was already decided */
	G5P_WK_SKIP_LICENSE, /* a timeout or fault was noted (terminal failed) */
	G5P_WK_SKIP_FAULT, /* boot failed, event failed or stopped */
	G5P_WK_SKIP_POWER, /* one of G5P_WK_PWR_REQUIRED is absent */
	G5P_WK_SKIP_R74F, /* an r74f bell is published, open or firing */
	G5P_WK_SKIP_RING, /* the ring is not exactly this publish */
	G5P_WK_SKIP_RETIRED, /* a stamp == seq or a completion event since the publish */
	G5P_WK_SKIP_FENCE, /* R1+0x14 read back outside 7: no further register */
	G5P_WK_SKIP_GPC, /* GPC not active: no 0xb00 */
	G5P_WK_DIDS,
};

static const char *const g5p_wk_arm_names[G5P_WK_ARMS] = {
	"none",
	"excluded",
	"control",
	"factor",
};

static const char *const g5p_wk_did_names[G5P_WK_DIDS] = {
	"none",		"rang",	      "sham",	    "excluded",	 "skip-twice",
	"skip-license", "skip-fault", "skip-power", "skip-r74f", "skip-ring",
	"skip-retired", "skip-fence", "skip-gpc",
};

/* Everything the decision reads, sampled under the event lock right after the producer store. */
struct g5p_wk_in {
	u32 mode, seed, seq, e;
	bool attempted; /* this phase already went through g5p_wk_decide() */
	bool term_failed; /* g5p_ru_term_failed() */
	bool boot_failed; /* b->failed || b->event_failed || b->stopped */
	u32 power; /* g5p_wk_power_mask(); the slave lock is evidence, not a veto */
	bool r74f_pending; /* an r74f bell in PUBLISHED, OPEN or FIRING */
	u32 start, target, prod,
		cons; /* ring: producer before this publish, our target, now */
	u32 stamp[2]; /* this engine's stamp words */
	u32 type1; /* this engine's completion events decoded since the phase armed */
};

/* The register access of the factor, in order.  The kernel binds them to writel/readl on the
 * Fender window and the RGX window and to dma_wmb(); the host suite records them.
 */
struct g5p_wk_io {
	void *ctx;
	void (*w1)(void *ctx, u32 reg, u32 value);
	u32 (*r1)(void *ctx, u32 reg);
	void (*wbarrier)(void *ctx);
	void (*w0)(void *ctx, u32 reg, u32 value);
};

static inline bool g5p_wk_mode_valid(u32 mode)
{
	return mode <= G5P_WK_ALWAYS;
}

static inline u32 g5p_wk_mix(u32 x)
{
	x ^= x >> 16;
	x *= 0x7feb352dU;
	x ^= x >> 15;
	x *= 0x846ca68bU;
	x ^= x >> 16;
	return x;
}

/* The phase's index among its engine's eligible phases: TA of job seq >= 2 -> seq - 2,
 * 3D of job seq >= 1 -> seq - 1.
 */
static inline u32 g5p_wk_index(u32 seq, u32 e)
{
	return e ? seq - 1 : seq - 2;
}

static inline u32 g5p_wk_arm(u32 mode, u32 seed, u32 seq, u32 e)
{
	u32 i, flip;

	if (!mode || !g5p_wk_mode_valid(mode) || !seq || e > 1)
		return G5P_WK_ARM_NONE;
	if (!e && seq == 1)
		return G5P_WK_ARM_EXCLUDED;
	if (mode == G5P_WK_ALWAYS)
		return G5P_WK_ARM_FACTOR;
	i = g5p_wk_index(seq, e);
	flip = g5p_wk_mix(g5p_wk_mix(seed & ~G5P_WK_MIRROR) + (e << 24) +
			  (i >> 1)) &
	       1;
	flip ^= !!(seed & G5P_WK_MIRROR);
	return ((i & 1) ^ flip) ? G5P_WK_ARM_FACTOR : G5P_WK_ARM_CONTROL;
}

static inline u32 g5p_wk_mts(u32 e)
{
	return e ? G5P_WK_MTS_3D : G5P_WK_MTS_TA;
}

/* wake111: the evidence word, one bit per fact (the caller samples each fact once). */
static inline u32 g5p_wk_power_mask(bool powered, bool fender_ready,
				    bool released, bool alive,
				    bool control_complete, bool ds108,
				    bool slave_lock)
{
	return (powered ? G5P_WK_PWR_CORE : 0) |
	       (fender_ready ? G5P_WK_PWR_FENDER : 0) |
	       (released ? G5P_WK_PWR_RELEASED : 0) |
	       (alive ? G5P_WK_PWR_ALIVE : 0) |
	       (control_complete ? G5P_WK_PWR_CONTROL : 0) |
	       (ds108 ? G5P_WK_PWR_DS108 : 0) |
	       (slave_lock ? G5P_WK_PWR_SLAVE_LOCK : 0);
}

static inline bool g5p_wk_power_ready(u32 power)
{
	return (power & G5P_WK_PWR_REQUIRED) == G5P_WK_PWR_REQUIRED;
}

/* The RAM guards, common to both arms, in priority order.  0 = pass. */
static inline u32 g5p_wk_guard(const struct g5p_wk_in *in)
{
	if (in->attempted)
		return G5P_WK_SKIP_TWICE;
	if (in->term_failed)
		return G5P_WK_SKIP_LICENSE;
	if (in->boot_failed)
		return G5P_WK_SKIP_FAULT;
	if (!g5p_wk_power_ready(in->power))
		return G5P_WK_SKIP_POWER;
	if (in->r74f_pending)
		return G5P_WK_SKIP_R74F;
	if (in->start > 255 || in->target > 255 || in->prod != in->target ||
	    in->cons > 255 || ((in->target - in->start) & 255) != 1 ||
	    ((in->cons - in->start) & 255) > 1)
		return G5P_WK_SKIP_RING;
	if (in->stamp[0] == in->seq || in->stamp[1] == in->seq || in->type1)
		return G5P_WK_SKIP_RETIRED;
	return 0;
}

/* The factor's register sequence (Apple's order).  The caller set *fence and *gpc to U32_MAX
 * (not read).
 */
static inline u32 g5p_wk_ring(const struct g5p_wk_io *io, u32 e, u32 *fence,
			      u32 *gpc)
{
	io->w1(io->ctx, G5P_WK_R1_FENDER, 7);
	*fence = io->r1(io->ctx, G5P_WK_R1_FENDER);
	if (*fence & ~7U)
		return G5P_WK_SKIP_FENCE;
	*gpc = io->r1(io->ctx, G5P_WK_R1_GPC);
	if (!G5P_GPC_ON(*gpc))
		return G5P_WK_SKIP_GPC;
	/* the publish's RAM stores before the MTS write (Apple: dmb ish before vt+0x80) */
	io->wbarrier(io->ctx);
	io->w0(io->ctx, G5P_WK_R0_MTS, g5p_wk_mts(e));
	return G5P_WK_DID_RANG;
}

/* One phase's decision and, for a factor that passed every guard, its register sequence
 * (the control arm never touches io).
 */
static inline u32 g5p_wk_decide(const struct g5p_wk_in *in,
				const struct g5p_wk_io *io, u32 *fence,
				u32 *gpc)
{
	u32 arm = g5p_wk_arm(in->mode, in->seed, in->seq, in->e), skip;

	*fence = U32_MAX;
	*gpc = U32_MAX;
	if (arm == G5P_WK_ARM_NONE)
		return G5P_WK_DID_NONE;
	if (arm == G5P_WK_ARM_EXCLUDED)
		return G5P_WK_DID_EXCLUDED;
	skip = g5p_wk_guard(in);
	if (skip)
		return skip;
	if (arm == G5P_WK_ARM_CONTROL)
		return G5P_WK_DID_SHAM;
	return g5p_wk_ring(io, in->e, fence, gpc);
}

/* A decided phase never decides again (the caller stores this before it unlocks). */
static inline bool g5p_wk_attempted(u32 did)
{
	return did != G5P_WK_DID_NONE && did != G5P_WK_DID_EXCLUDED;
}

#endif
