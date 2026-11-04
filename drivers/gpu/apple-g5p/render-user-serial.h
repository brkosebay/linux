/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * TYPED_V3 serial jobs of every render context (a compositor and its clients: one TA+3D job
 * per frame each), on the device's one TA/3D channel pair.
 * Pure functions, host-tested (scripts/gpu/multiclient1-host/test_render_admit.c).
 *
 * Semantics:
 *   - jobs of all contexts run strictly one after another: the queue is FIFO, each job is
 *     TA publish -> TA complete -> 3D publish -> 3D complete before the next job starts, and
 *     there is never a TA(N+1) in flight with a 3D(N);
 *   - the epoch's first job brings the device up: PM, channels and its context's DC3 attach
 *     with the r74f/r74g bells (one-shot).  Every later context binds its own slot pair at its
 *     first job with a DC3 pair published in RAM only (render-user-bind.h), no doorbell;
 *   - every job after the first only rebuilds the PM pool, builds its pages for the device's
 *     sequence number, publishes with NO doorbell and waits RAM-only;
 *   - the first job of a context that does not complete (timeout, fault, protocol error)
 *     marks that context bad: every job still queued behind it fails with -EIO without
 *     touching the hardware, and every later submit is refused (-EIO);
 *   - TYPED_V3 never mixes with the other profiles, on a context or on the device;
 *   - after any timeout or fault (render-user-terminal.h `failed`) no job starts at all, and
 *     once the epoch is quiet the device cannot be brought up (that rings a doorbell); binding
 *     another context is RAM only and stays allowed;
 *   - the sequence number is the device's, 31 bit; the channel watermark grid wraps
 *     (g5p_ru_endpoint).
 */
#ifndef G5P_RENDER_USER_SERIAL_H
#define G5P_RENDER_USER_SERIAL_H

enum g5p_ru_v3_refusal {
	G5P_RU_V3_OK,
	G5P_RU_V3_NO_TUPLE, /* boot tuple (r74g + r87b self-drain, no r74h/r80) absent */
	G5P_RU_V3_MIXED, /* another profile ran on this context or brought the device up */
	G5P_RU_V3_OVERFLOW, /* sequence space exhausted */
	G5P_RU_V3_SLOTS, /* not two distinct slots in 7..255 */
	G5P_RU_V3_VARIANT, /* r22/r25 variant, 3d_after_ta, timestamps or pm_policy */
	G5P_RU_V3_BOUND, /* per-phase bound outside 20000..30000 ms */
	G5P_RU_V3_STATE, /* executor only: bind state and job counts disagree */
	G5P_RU_V3_TERMINAL, /* terminal epoch: timeout/fault, or quiet and the device down */
	G5P_RU_V3_REFUSALS,
};

static const char *const g5p_ru_v3_why[G5P_RU_V3_REFUSALS] = {
	"",
	"transport is not ready for TYPED_V3 rendering",
	"another profile ran on this context or brought the device up",
	"sequence space exhausted",
	"slots are not two distinct slots in 7..255",
	"an r22/r25 variant, 3d_after_ta, timestamps or pm_policy is latched",
	"per-phase bound outside 20000..30000 ms",
	"bind state and job counts disagree",
	"terminal epoch (timeout or fault, or quiet and the device not up): no hardware access",
};

/* Everything the admission needs, as plain values. */
struct g5p_ru_v3_state {
	bool tuple; /* boot tuple present */
	bool v3_accepted; /* a V3 job was accepted on this context */
	bool bound; /* this context's slot pair was published */
	u32 jobs; /* jobs that started on this context */
	bool device_up; /* the epoch's first job built PM and channels */
	bool device_v3; /* ... through the TYPED_V3 attach */
	u32 sequence; /* the device's last job that started (0 = none) */
	u32 slot_ta, slot_3d;
	bool variant;
	u32 wait_ms;
	bool quiet; /* a terminal note was made: mmio_ok is false */
	bool terminal_failed; /* ... and it was, or a later one is, a timeout or fault */
};

/* Any two distinct user slots: unbinds leave holes, so a pair need not be adjacent. */
static inline bool g5p_ru_v3_slots_valid(u32 slot_ta, u32 slot_3d)
{
	return slot_ta >= 7 && slot_3d >= 7 && slot_ta <= 255 &&
	       slot_3d <= 255 && slot_ta != slot_3d;
}

#define G5P_RU_V3_SEQ_MAX 0x7fffffffU

/* At submit time and again in the executor (`executing`): a refusal here touched nothing. */
static inline int g5p_ru_v3_admit(const struct g5p_ru_v3_state *s,
				  bool executing)
{
	if (!s->tuple)
		return G5P_RU_V3_NO_TUPLE;
	/* After any timeout or fault nothing new starts (closes the window between a failing job's
	 * unlock and the context being marked bad).  Once quiet, the device cannot be brought up,
	 * because that rings a doorbell; a job on an up device publishes with no doorbell and
	 * touches no register, and so does the bind of a new context.
	 */
	if (s->terminal_failed || (s->quiet && !s->device_up))
		return G5P_RU_V3_TERMINAL;
	/* A job of another profile ran on this context, or another profile brought the device
	 * up: only V3 may follow V3.
	 */
	if ((s->jobs && !s->v3_accepted) || (s->device_up && !s->device_v3))
		return G5P_RU_V3_MIXED;
	if (s->sequence >= G5P_RU_V3_SEQ_MAX)
		return G5P_RU_V3_OVERFLOW;
	if (!g5p_ru_v3_slots_valid(s->slot_ta, s->slot_3d))
		return G5P_RU_V3_SLOTS;
	if (s->variant)
		return G5P_RU_V3_VARIANT;
	if (!g5p_ru_v3_bound_valid(s->wait_ms))
		return G5P_RU_V3_BOUND;
	/* The executor: the device is up exactly when a job has started on it, and a context is
	 * bound exactly when it has run a job, always on an up device.
	 */
	if (executing && (s->device_up != !!s->sequence ||
			  s->bound != !!s->jobs || (s->bound && !s->device_up)))
		return G5P_RU_V3_STATE;
	return G5P_RU_V3_OK;
}

/* Verbose receipts for each context's first jobs and every failure; a compositor prints no
 * line per frame.
 */
static inline bool g5p_ru_v3_verbose(u32 jobs, int ret)
{
	return jobs <= 8 || ret;
}

#endif
