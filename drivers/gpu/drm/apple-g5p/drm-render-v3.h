/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* TYPED_V3 serial executor: per-job state (receipt data). Types only; code: drm-render-v3.inc. */
#ifndef G5P_DRM_RENDER_V3_H
#define G5P_DRM_RENDER_V3_H

enum g5p_rv3_phase {
	G5P_RV3_IDLE,
	G5P_RV3_ATTACH,
	G5P_RV3_BIND,
	G5P_RV3_TA_PUBLISH,
	G5P_RV3_TA_WAIT,
	G5P_RV3_TD_PUBLISH,
	G5P_RV3_TD_WAIT,
	G5P_RV3_DONE,
	G5P_RV3_PHASES,
};

/* g5p_drm.td_want: the 3D completion-event latch is armed for the endpoint in the low byte. */
#define G5P_RV3_TD_WANT 0x100U

/* One RAM sample of one engine: ring, stamps, channel watermarks, idle words, PM state, and
 * (3D) whether the matching completion event has been latched.
 */
struct g5p_rv3_ev {
	u64 ns;
	u32 prod, cons, stamp[2], ctl[5], idle[6], pm80;
	bool proto, failed, consumed, retired, idle_ok, pm_ok, event;
};

struct g5p_rv3 {
	struct g5p_ru_wait ta, td;
	struct g5p_ru_touch
		touch[2]; /* RC4a A/B (render-user-touch.h); zero when off */
	struct g5p_ru_retire rt; /* the 3D predicate (render-user-retire.h) */
	struct g5p_ru_ta tr; /* the TA predicate (render-user-retire.h) */
	struct g5p_rv3_ev last[2];
	u64 td_event_ns; /* decode time of the matching 3D event; 0 = none */
	u64 ta_event_ns; /* decode time of the first TA event since the publish */
	u64 attach_ns, job_deadline_ns, ta_pub_ns, td_pub_ns, ta_consumed_ns,
		td_consumed_ns;
	u32 phase, grace_end, bell;
	u32 seq; /* the device sequence this job got; 0 = none */
	/* wake110 (render-user-wake.h), per engine: the decision, the power evidence word it saw
	 * (wake111), the two register words it read (U32_MAX = not read), the consumer it saw, the
	 * type-1 event counts at the phase's arm and at the job's end, the lock window and the
	 * first type-1 decode time.  Zero when off.
	 */
	u32 wk_did[2], wk_power[2], wk_fence[2], wk_gpc[2], wk_cons[2],
		wk_t1_pub[2], wk_t1_end[2];
	u64 wk_start_ns[2], wk_done_ns[2], wk_first_ns[2];
	bool touched; /* the DC3 pair was published: the firmware may reference the job's memory */
	/* The bind of this job's context published its DC3 pair and then failed: that context is
	 * lost, the device and the contexts already bound are not.
	 */
	bool bind_failed;
	/* The job was lost alone (render-user-loss.h): its context is dead, the device is not; the
	 * channel stays blocked behind its stuck phase until the scheduler's drain clears it.
	 */
	bool lost;
};

#endif
