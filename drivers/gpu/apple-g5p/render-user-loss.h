/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Per-context fault isolation (docs/gpu/multi-client-design.md "Stage 2/3 as built", fault
 * isolation).  Pure decisions, host-tested (scripts/gpu/multiclient2-host/test_render_loss.c).
 *
 * The hardware gives no way to abandon a job: the firmware has no abort (iOS ends a
 * non-progressing engine with a firmware stop, a platform power cycle and a new firmware boot,
 * kext 6c9e070, firmware-re README "Fault recovery"), the TA and 3D channels are FIFO, and the
 * one TA and one 3D engine serve every context.  So one context's job can be *lost* (its client
 * gets an error, its context is dead) without ending the device only while the firmware is
 * alive, and only if the stuck phase still finishes: until then no other job may be published
 * behind it (it would wait behind the same engine and be lost in turn).
 *
 *   loss     at the end of a phase that did not complete:
 *              NONE     it completed;
 *              CONTEXT  a phase timeout on a job other than the epoch's first, with the firmware
 *                       alive (its event stream never went silent, no fault event, no protocol
 *                       error) and isolation on: the job and its context are lost, the device is
 *                       not; the channel is blocked behind the stuck phase;
 *              DEVICE   anything else: the epoch is terminal, as before.
 *   drain    while blocked, the scheduler runs no job and waits, RAM only, for the stuck phase:
 *              a late TA gets its 3D published (the channel and PM state then stay consistent),
 *              a late 3D ends it; the channel is then clear and every held job runs.  A silent
 *              event stream, a fault, a refused 3D or the drain bound make the epoch terminal.
 */
#ifndef G5P_RENDER_USER_LOSS_H
#define G5P_RENDER_USER_LOSS_H

enum g5p_ru_loss {
	G5P_RU_LOSS_NONE,
	G5P_RU_LOSS_CONTEXT,
	G5P_RU_LOSS_DEVICE,
};

struct g5p_ru_loss_in {
	u32 result; /* G5P_RU_WAIT_* of the phase */
	bool first; /* the epoch's first job (its attach and bell) */
	bool isolate; /* render_user_drain_ms != 0 */
	bool silent; /* the event stream stopped: the firmware is dead */
	bool fault; /* a fault event, a failed boot, a protocol or drain error */
};

static inline int g5p_ru_loss(const struct g5p_ru_loss_in *in)
{
	if (in->result == G5P_RU_WAIT_COMPLETE)
		return G5P_RU_LOSS_NONE;
	if (in->result != G5P_RU_WAIT_TIMEOUT || in->first || !in->isolate ||
	    in->silent || in->fault)
		return G5P_RU_LOSS_DEVICE;
	return G5P_RU_LOSS_CONTEXT;
}

/* The drain of a blocked channel. */
enum g5p_ru_drain_state {
	G5P_RU_DRAIN_TA, /* waiting for the stuck TA */
	G5P_RU_DRAIN_TD, /* waiting for the stuck job's 3D */
	G5P_RU_DRAIN_CLEAR, /* done: the channel is clear */
	G5P_RU_DRAIN_DEAD, /* the epoch is terminal */
};

/* What the drain does next. */
enum g5p_ru_drain_action {
	G5P_RU_DRAIN_WAIT, /* sample again */
	G5P_RU_DRAIN_PUBLISH_TD, /* publish the stuck job's 3D, wait for it */
	G5P_RU_DRAIN_UNBLOCK, /* release the held jobs */
	G5P_RU_DRAIN_TERMINAL, /* note the epoch terminal, release the held jobs (they fail) */
};

/* One step from the result of the phase wait (G5P_RU_WAIT_*) in state *st.  A refused 3D
 * publish is reported by the caller as G5P_RU_WAIT_FAULT on the TD state.
 */
static inline int g5p_ru_drain_step(u32 *st, u32 result)
{
	if (*st >= G5P_RU_DRAIN_CLEAR)
		return *st == G5P_RU_DRAIN_CLEAR ? G5P_RU_DRAIN_UNBLOCK :
						   G5P_RU_DRAIN_TERMINAL;
	if (result == G5P_RU_WAIT_PENDING)
		return G5P_RU_DRAIN_WAIT;
	if (result != G5P_RU_WAIT_COMPLETE) {
		*st = G5P_RU_DRAIN_DEAD;
		return G5P_RU_DRAIN_TERMINAL;
	}
	if (*st == G5P_RU_DRAIN_TA) {
		*st = G5P_RU_DRAIN_TD;
		return G5P_RU_DRAIN_PUBLISH_TD;
	}
	*st = G5P_RU_DRAIN_CLEAR;
	return G5P_RU_DRAIN_UNBLOCK;
}

/*
 * The drain bound must leave the held clients' own wait intact.  A client waits for its job up to
 * the job_timeout_ms GET_RENDER_INFO reports plus a margin (the ICD: + 3 s, so 2 * hard + obs + 3 s
 * = 125 s with latency101 on, 33 s with it off), and a held job has already waited out the stuck
 * phase (up to the hard bound, or render_user_wait_ms) before the drain starts, then runs after
 * it.  Valid: 0 (isolation off), or from 1000 up to hard + obs - 10000 with latency101 on (hard
 * 60000, obs 2000: 52000), and up to 8000 with it off.  An invalid value at the time of a loss
 * turns isolation off for it.
 */
static inline bool g5p_ru_drain_valid(u32 drain_ms, u32 hard_ms, u32 obs_ms)
{
	if (!drain_ms)
		return true;
	if (drain_ms < 1000)
		return false;
	if (!hard_ms)
		return drain_ms <= 8000;
	return hard_ms + obs_ms >= 11000 &&
	       drain_ms <= hard_ms + obs_ms - 10000;
}



static inline const char *g5p_ru_loss_name(int loss)
{
	return loss == G5P_RU_LOSS_NONE ? "none" :
	       loss == G5P_RU_LOSS_CONTEXT ? "context" : "device";
}

#endif
