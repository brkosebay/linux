/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 RAM-touch latency A/B (RC4a).  Pure functions, no kernel services beyond the
 * integer types, so the host suite compiles this file as it is.  Include render-user-wait.h first.
 *
 * Question (docs/gpu/experiments/latency101-device-analysis.md section 9): is the host's bulk read
 * of the receipt pages what releases a stalled phase (the +6 ms atom after the job-1 receipt)?
 *
 * render_user_touch_ms = T, default 0 = off.  On, every phase of every job records the first wait
 * sample at or after publish + T that left the phase undecided (still pending, deadline not
 * reached).  Odd sequences are the touch arm: right after that sample the host reads the
 * receipt's read set once (RAM only, folded into a sum).  Even sequences are the control arm: the
 * same instant is recorded and nothing is read.  Neither arm changes a sample, the sleep, the
 * deadline, a latch or the result, and neither prints before the job ends.
 *
 * Off: g5p_ru_touch_start() leaves the record zero and g5p_ru_touch_due() is never true, so the
 * wait loop runs the latency101 instructions unchanged.
 */
#ifndef G5P_RENDER_USER_TOUCH_H
#define G5P_RENDER_USER_TOUCH_H

#define G5P_RU_TOUCH_MAX_MS \
	20000U /* never at or past the smallest soft bound */

struct g5p_ru_touch {
	u64 at_ns; /* publish + T; 0 = off for this phase */
	u64 seen_ns; /* the undecided sample at or after at_ns that took it; 0 = not reached */
	u64 done_ns; /* end of the read (touch arm), seen_ns (control arm) */
	u32 bytes; /* bytes read: 0 on the control arm */
	u32 sum; /* fold of the words read: the loads cannot be elided */
	bool sweep; /* this phase is in the touch arm */
};

/* 0 (off), or 1..G5P_RU_TOUCH_MAX_MS with latency101 on (hard_ms != 0) and strictly before the
 * effective soft bound (g5p_ru_lat_soft()), so the instant always falls in the plain wait.
 */
static inline bool g5p_ru_touch_valid(u32 touch_ms, u32 soft_ms, u32 hard_ms)
{
	if (!touch_ms)
		return true;
	return hard_ms && touch_ms <= G5P_RU_TOUCH_MAX_MS && touch_ms < soft_ms;
}

/* Odd sequences are the touch arm, even ones the control. */
static inline bool g5p_ru_touch_arm(u32 seq)
{
	return seq & 1;
}

static inline void g5p_ru_touch_start(struct g5p_ru_touch *t, u64 start_ns,
				      u32 touch_ms, u32 seq)
{
	*t = (struct g5p_ru_touch){ 0 };
	if (!touch_ms)
		return;
	t->at_ns = start_ns + (u64)touch_ms * 1000000ULL;
	t->sweep = g5p_ru_touch_arm(seq);
}

/*
 * Asked after g5p_ru_wait_step() of the sample taken at now_ns: true once, for the first sample
 * at
 * or after the instant whose step left the phase undecided and not expired.  A completion, a fault
 * or the deadline in the same sample wins, and then the instant is never taken.
 */
static inline bool g5p_ru_touch_due(const struct g5p_ru_touch *t, u64 now_ns,
				    u32 result, bool expired)
{
	return t->at_ns && !t->seen_ns && !result && !expired &&
	       now_ns >= t->at_ns;
}

/* The instant was taken at seen_ns (a sample time, never 0); done_ns is when the read ended. */
static inline void g5p_ru_touch_mark(struct g5p_ru_touch *t, u64 seen_ns,
				     u64 done_ns, u32 bytes, u32 sum)
{
	t->seen_ns = seen_ns;
	t->done_ns = done_ns < seen_ns ? seen_ns : done_ns;
	t->bytes = bytes;
	t->sum = sum;
}

/* Receipt helper: to - from in microseconds, -1 when an end is missing or the order is wrong. */
static inline s64 g5p_ru_touch_us(u64 from_ns, u64 to_ns)
{
	if (!from_ns || !to_ns || to_ns < from_ns)
		return -1;
	return (s64)((to_ns - from_ns) / 1000);
}

/* Receipt: how long after the instant (the end of the read) the phase's deciding sample saw it
 * complete; -1 when the instant was not taken or the phase did not complete.
 */
static inline s64 g5p_ru_touch_after_us(const struct g5p_ru_touch *t,
					u32 result, u64 end_ns)
{
	if (!t->seen_ns || result != G5P_RU_WAIT_COMPLETE)
		return -1;
	return g5p_ru_touch_us(t->done_ns, end_ns);
}

#endif
