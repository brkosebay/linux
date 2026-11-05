// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * TYPED_COMPUTE: compute jobs on TYPED_V3 render contexts.
 *
 * A compute job is a TYPED_V3 job of its render context with one phase on the device's CDM
 * channel instead of a TA and a 3D: it is queued, scheduled, fenced, quarantined and lost
 * exactly like a render job (g5p_render_run(), drm-render-sched.inc, render-user-fault.h).
 * It binds the context's slot pair the RAM-only way if the context has not run a job yet,
 * builds the CDM job (compute-user-layout.h) from the pass the submit validated
 * (compute-user-validate.h), publishes it on the CL ring with a guarded DM2 wake, and waits RAM only
 * for coherent channel watermarks and both completion stamps.  It never brings the device up (that rings the TA bell):
 * an epoch without a render job refuses it before the hardware.
 *
 * Only the licensed publish-time wake touches registers; validation, waits,
 * samples, retirement and recovery decisions are RAM-only.
 */

static u8 *g5p_cu_page(struct g5p_boot *b)
{
	return b->shared[G5P_CU_OFFSET / G5P_GPU_PAGE].cpu +
	       G5P_CU_OFFSET % G5P_GPU_PAGE;
}

static u32 g5p_cu_meta(void)
{
	return g5p_meta_address(G5P_SHARED_VA + G5P_CU_OFFSET);
}

/* The CDM's global object (the boot layout's engine-2 object, R0 group, CPU 0x49 / GPU 0x53). */
static u32 g5p_cu_global_meta(struct g5p_boot *b)
{
	return g5p_meta_address(
		g5p_object_va(&b->layout, 0x51 + G5P_CU_ENGINE));
}

/* The render device's bring-up (g5p_render_device_prepare()), after the TA/3D channel init that
 * clears the 3D page: the CDM channel at its epoch start.  RAM only.
 */
static void g5p_cu_device_init(struct g5p_drm *d)
{
	BUILD_BUG_ON(G5P_CU_OFFSET % G5P_GPU_PAGE < G5P_R22_DEP + 0x280);
	BUILD_BUG_ON(G5P_CU_OFFSET % G5P_GPU_PAGE < G5P_RU_SLOT1_END);
	BUILD_BUG_ON(G5P_CU_OFFSET % G5P_GPU_PAGE + G5P_CU_BYTES >
		     G5P_GPU_PAGE);
	BUILD_BUG_ON(G5P_CU_DEP + G5P_CU_DEP_BYTES > G5P_CU_BYTES);
	BUILD_BUG_ON(G5P_CU_W + G5P_CU_W_BYTES > G5P_CU_SKU);
	g5p_cu_channel_init(g5p_cu_page(d->core->boot), g5p_cu_meta());
	d->rdev->cu_sequence = 0;
	d->rdev->cu_target = 0;
}

/* One RAM sample of the CDM channel for job `seq` (the slots of struct g5p_rv3_ev: stamp[0] =
 * W258 mirror, stamp[1] = W25c, ctl = the five watermarks, idle[0..2] = the CDM global's
 * G724/728/72c).  RAM only.
 */
static void g5p_cu_sample(struct g5p_diag *g, u32 seq, u32 target,
			  struct g5p_rv3_ev *s)
{
	struct g5p_boot *b = g->boot;
	u8 *p = g5p_cu_page(b), *global = g5p_ru_global(b, G5P_CU_ENGINE);
	u32 *ring = (u32 *)g5p_boot_shared(b, 0x68);

	g5p_rv3_fw_touch(b);
	memset(s, 0, sizeof(*s));
	dma_rmb();
	s->ns = ktime_get_ns();
	s->prod = READ_ONCE(ring[4]);
	s->cons = READ_ONCE(ring[0]);
	s->stamp[0] = g5p_le32(p + G5P_CU_START);
	s->stamp[1] = g5p_le32(p + G5P_CU_END);
	for (u32 i = 0; i < ARRAY_SIZE(s->ctl); i++)
		s->ctl[i] = g5p_le32(p + G5P_WORK_CTL + i * 0x10);
	for (u32 i = 0; i < 3; i++)
		s->idle[i] = g5p_le32(global + 0x724 + i * 4);
	s->failed = READ_ONCE(b->failed) || READ_ONCE(b->event_failed);
	s->proto = s->prod != target || s->cons > 255 ||
		   !g5p_cu_stamp_sane(p, seq);
	s->consumed = s->cons == target;
	s->idle_ok = !s->idle[0] && !s->idle[1] && !s->idle[2];
	s->retired = g5p_cu_retired(p, g5p_cu_meta(), seq);
	s->pm_ok = true;
}

#define G5P_CU_SLOW_MS 1000U

/* The CDM phase's RAM-only wait, the TYPED_V3 wait (g5p_rv3_wait()) for engine 2: the silence
 * clock, the fault/protocol rules and the closing sample are the same; complete is the ring
 * consumed plus the retirement predicate (the idle words are recorded, not required: render path).
 */
/* Event serial is sampled before RAM, so an IRQ between sample and sleep
 * cannot be lost. Any firmware event wakes us; only the full RAM retirement
 * predicate may complete the job. No render-only engine-indexed observers.
 */
static void g5p_cu_pause(struct g5p_drm *d, u64 seen)
{
	u32 wait_us;

	if (g5p_ew_plan(READ_ONCE(render_user_event_wake), false, &wait_us) ==
	    G5P_EW_LEGACY) {
		usleep_range(1000, 2000);
		return;
	}
	wait_event_hrtimeout(d->ev_wq,
			     g5p_ew_woken(seen, READ_ONCE(d->event_serial)),
			     us_to_ktime(wait_us));
}

static u32 g5p_cu_wait(struct g5p_drm *d, struct g5p_diag *g, u32 seq,
		       u32 target, struct g5p_ru_wait *w, struct g5p_rv3_ev *s,
		       u64 *consumed_ns, int *err)
{
	bool closing = false, slow = false;

	for (;;) {
		int drain = g5p_boot_drain(g);
		u64 seen = READ_ONCE(d->event_serial);
		bool fault, silent, done;

		g5p_cu_sample(g, seq, target, s);
		silent =
			g5p_ru_wait_grace(w, s->ns, READ_ONCE(g->boot->events));
		g5p_ru_wait_busy(w, s->ns, !s->idle_ok);
		fault = drain || s->failed || s->proto || silent;
		if (drain)
			*err = drain;
		else if (s->proto)
			*err = -EPROTO;
		else if (s->failed || silent)
			*err = -EIO;
		if (s->consumed && !*consumed_ns)
			*consumed_ns = s->ns;
		done = s->consumed && s->retired;
		if (closing)
			g5p_ru_wait_close(w, s->ns, done, fault);
		else
			g5p_ru_wait_step(w, s->ns, done, fault);
		if (w->result)
			return w->result;
		if (!slow && s->ns - w->start_ns >=
				     (u64)G5P_CU_SLOW_MS * NSEC_PER_MSEC) {
			slow = true;
			dev_warn(
				g->dev,
				"compute slow seq=%u ms=%lld ring=%u/%u stamp=%08x/%08x ctl=%08x,%08x,%08x,%08x,%08x idle=%08x,%08x,%08x consumed=%u retired=%u events=%u\n",
				seq, g5p_ru_rt_ms(w->start_ns, s->ns), s->prod,
				s->cons, s->stamp[0], s->stamp[1], s->ctl[0],
				s->ctl[1], s->ctl[2], s->ctl[3], s->ctl[4],
				s->idle[0], s->idle[1], s->idle[2], s->consumed,
				s->retired, READ_ONCE(g->boot->events));
		}
		if (w->expired) {
			closing = true; /* the closing sample, at once */
			continue;
		}
		g5p_cu_pause(d, seen);
	}
}

/* The guarded publish-time wake, identical ordering to TA/3D with DM2.
 * Core lock held; only this helper may touch registers in the compute path.
 * The event lock closes the reset/fault race. Never called from wait/drain.
 */
static int g5p_cu_wake(struct g5p_diag *g, u32 seq, u32 start, u32 target)
{
	const struct g5p_wk_io io = { g, g5p_rv3_wk_w1, g5p_rv3_wk_r1,
				      g5p_rv3_wk_wmb, g5p_rv3_wk_w0 };
	struct g5p_boot *b = g->boot;
	u32 *ring = (u32 *)g5p_boot_shared(b, 0x68);
	u8 *p = g5p_cu_page(b);
	struct g5p_wk_in in;
	unsigned long flags;
	u32 why, fence = U32_MAX, gpc = U32_MAX;

	spin_lock_irqsave(&b->event_lock, flags);
	dma_rmb();
	in = (struct g5p_wk_in){
		.mode = G5P_WK_ALWAYS,
		.e = G5P_CU_ENGINE,
		.seq = seq,
		.term_failed = g5p_ru_term_failed(&g5p_rv3_term),
		.boot_failed = READ_ONCE(b->failed) ||
			       READ_ONCE(b->event_failed) ||
			       READ_ONCE(b->stopped),
		.power = g5p_wk_power_mask(
			READ_ONCE(g->powered), READ_ONCE(g->fender_ready),
			READ_ONCE(b->released), READ_ONCE(b->alive),
			READ_ONCE(b->control_complete), g5p_rv3_wk_ds108(b),
			READ_ONCE(g->reset_transition)),
		.r74f_pending = g5p_rv3_wk_r74f_busy(b->r73b),
		.start = start,
		.target = target,
		.prod = READ_ONCE(ring[4]),
		.cons = READ_ONCE(ring[0]),
		.stamp = { g5p_le32(p + G5P_CU_START),
			   g5p_le32(p + G5P_CU_END) },
	};
	why = g5p_wk_guard(&in);
	if (!why)
		why = g5p_wk_ring(&io, G5P_CU_ENGINE, &fence, &gpc);
	spin_unlock_irqrestore(&b->event_lock, flags);
	dev_dbg(g->dev,
		"compute wake seq=%u did=%u power=%#x fence=%#x gpc=%#x\n", seq,
		why, in.power, fence, gpc);
	return why == G5P_WK_DID_RANG ? 0 : -EHOSTDOWN;
}

/* The CL publish of job `seq`: the global ring entry {type 2, channel, endpoint}, the channel's
 * admission watermark, then the producer, followed by the guarded DM2 wake.  Refused, with nothing written, unless the
 * ring is idle and the channel stands at the previous job's endpoint and stamps.  RAM only.
 */
static int g5p_cu_publish(struct g5p_diag *g, struct g5p_render *r,
			  struct g5p_rv3 *v, u32 seq)
{
	struct g5p_boot *b = g->boot;
	struct g5p_rdev *rd = r->ctx->gpu->rdev;
	u32 *ring = (u32 *)g5p_boot_shared(b, 0x68);
	u8 *p = g5p_cu_page(b);
	u32 start = READ_ONCE(ring[4]), previous = g5p_cu_endpoint(seq - 1);
	u32 endpoint = g5p_cu_endpoint(seq), why;

	why = (u32)(start > 255) | (u32)(READ_ONCE(ring[0]) != start) << 1 |
	      (u32)(g5p_le32(p + G5P_WORK_CTL) != previous) << 2 |
	      (u32)(g5p_le32(p + G5P_WORK_CTL + 0x40) != previous) << 3 |
	      (u32)(g5p_le32(p + G5P_CU_START) != seq - 1 ||
		    g5p_le32(p + G5P_CU_END) != seq - 1)
		      << 4;
	if (why) {
		dev_emerg(
			g->dev,
			"compute refused seq=%u why=%#x ring=%u/%u ctl=%08x/%08x stamp=%08x/%08x: nothing published\n",
			seq, why, start, READ_ONCE(ring[0]),
			g5p_le32(p + G5P_WORK_CTL),
			g5p_le32(p + G5P_WORK_CTL + 0x40),
			g5p_le32(p + G5P_CU_START), g5p_le32(p + G5P_CU_END));
		return -EPROTO;
	}
	for (u32 j = 0; j < 5; j++) {
		u32 at = b->layout.offset[0x69] + start * 20 + j * 4;

		g5p_put32(b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE,
			  j == 0 ? 2 :
			  j == 1 ? g5p_cu_meta() :
			  j == 2 ? endpoint :
				   0);
	}
	rd->cu_target = (start + 1) & 255;
	dma_wmb();
	WRITE_ONCE(*(u32 *)(p + G5P_WORK_CTL + 0x40), endpoint);
	dma_wmb();
	v->ta_pub_ns = ktime_get_ns();
	v->job_deadline_ns =
		v->ta_pub_ns +
		(u64)(r->hard_ms ? 2 * r->hard_ms : G5P_RU_V3_CAP_MS) *
			NSEC_PER_MSEC;
	smp_store_release(&ring[4], rd->cu_target);
	dma_wmb();
	if (g5p_ru_v3_verbose(r->jobs, 0))
		dev_dbg(g->dev,
			"compute publish slot=%u endpoint=%u ring=%u W=%08x sequence=%u kick=dm2\n",
			r->slot[0], endpoint, start, g5p_cu_meta() + G5P_CU_W,
			seq);
	return g5p_cu_wake(g, seq, start, rd->cu_target);
}

/* The CDM phase timed out with the firmware alive: lose the job and its context, block the
 * channel behind it until the scheduler's drain (g5p_cu_drain()) sees it retire.  RAM only.
 */
static int g5p_cu_lose(struct g5p_drm *d, struct g5p_render *r,
		       struct g5p_rv3 *v, u32 seq)
{
	struct g5p_rstuck *s = kzalloc(sizeof(*s), GFP_KERNEL);

	if (!s)
		return -ENOMEM;
	v->lost = true;
	s->r = r;
	s->v = *v;
	s->seq = seq;
	s->engine = G5P_CU_ENGINE;
	s->drain_ms = 45000;
	s->lost_ns = ktime_get_ns();
	kref_get(&r->ctx->ref);
	d->rdev->stuck = s;
	dev_emerg(
		d->core->dev,
		"compute lost seq=%u slot=%u phase_ms=%lld drain_ms=%u: context lost, device kept, channel blocked\n",
		seq, r->slot[0],
		g5p_rv3_us(v->ta.start_ns, v->ta.end_ns) / 1000, s->drain_ms);
	return 0;
}

static void g5p_cu_receipt(struct g5p_diag *g, struct g5p_render *r,
			   struct g5p_rjob *job, int ret)
{
	const struct g5p_rv3 *v = &job->rv3;
	const struct g5p_rv3_ev *s = &v->last[0];

	if (!g5p_ru_v3_verbose(r->jobs, ret))
		return;
	dev_dbg(g->dev,
		"compute job=%llu seq=%u ret=%d slot=%u consumed_us=%lld done_us=%lld ring=%u/%u stamp=%08x/%08x ctl=%08x,%08x,%08x,%08x,%08x idle=%08x,%08x,%08x lost=%u bind_failed=%u\n",
		job->id, v->seq, ret, r->slot[0],
		g5p_rv3_us(v->ta_pub_ns, v->ta_consumed_ns),
		g5p_rv3_us(v->ta_pub_ns, v->ta.end_ns), s->prod, s->cons,
		s->stamp[0], s->stamp[1], s->ctl[0], s->ctl[1], s->ctl[2],
		s->ctl[3], s->ctl[4], s->idle[0], s->idle[1], s->idle[2],
		v->lost, v->bind_failed);
}

/* One compute job, core lock held (g5p_render_run() via g5p_rv3_execute()).  Returns 0, -EAGAIN
 * (no epoch: nothing touched), -ETIMEDOUT, or a fault.
 */
static int g5p_cu_execute(struct g5p_drm *d, struct g5p_render *r,
			  struct g5p_rjob *job)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_rv3 *v = &job->rv3;
	struct g5p_ru_v3_state st = g5p_rv3_state(g, r);
	int err = -EIO, ret, why, loss;
	u32 seq, res;

	if (r->stage_done)
		return -ECANCELED;
	why = g5p_ru_v3_admit(&st, true);
	if (why)
		return g5p_rv3_refusal_errno(why);
	why = g5p_cu_admit(READ_ONCE(rd->up), rd->cu_sequence);
	if (why)
		return why == G5P_CU_NO_EPOCH ? -EAGAIN : -EOVERFLOW;
	seq = rd->cu_sequence + 1;
	if (!r->bound) {
		/* RAM only; a failure is this context's alone (v->bind_failed). */
		v->phase = G5P_RV3_BIND;
		ret = rd->spare_count ? g5p_rv3_adopt(d, r, v) :
					g5p_rv3_bind(d, r, v);
		if (ret)
			goto out;
	}
	/* The firmware owns the CL ring and the channel from here on: any failure quarantines. */
	v->touched = true;
	g5p_cu_job(g5p_cu_page(b), g5p_cu_meta(), g5p_cu_global_meta(b),
		   r->slot[0], seq, &job->cu);
	g5p_render_clean_job(g);
	rd->cu_sequence = seq;
	v->seq = seq;
	r->jobs++;
	v->phase = G5P_RV3_TA_PUBLISH;
	ret = g5p_cu_publish(g, r, v, seq);
	if (ret)
		goto out;
	v->phase = G5P_RV3_TA_WAIT;
	g5p_rv3_wait_start(g, r, &v->ta, v->ta_pub_ns, v->job_deadline_ns);
	res = g5p_cu_wait(d, g, seq, rd->cu_target, &v->ta, &v->last[0],
			  &v->ta_consumed_ns, &err);
	if (res != G5P_RU_WAIT_COMPLETE) {
		loss = g5p_rv3_loss(d, r, false, res, &v->ta);
		if (loss == G5P_RU_LOSS_CONTEXT && !g5p_cu_lose(d, r, v, seq)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		if (loss == G5P_RU_LOSS_DEVICE &&
		    g5p_rv3_recoverable(d, false)) {
			g5p_rv3_lose_recover(d, r, v, seq, G5P_CU_ENGINE,
					     v->ta.silent ? "cdm-silent" :
					     res == G5P_RU_WAIT_TIMEOUT ?
							    "cdm-timeout" :
							    "cdm-fault");
			ret = g5p_ru_lost_errno(res, err);
			goto out;
		}
		g5p_rv3_terminal(g,
				 res == G5P_RU_WAIT_TIMEOUT ?
					 G5P_RU_TERM_TIMEOUT :
					 G5P_RU_TERM_FAULT,
				 res == G5P_RU_WAIT_TIMEOUT ? "cdm-timeout" :
							      "cdm-fault");
		ret = res == G5P_RU_WAIT_TIMEOUT ? -ETIMEDOUT : err;
		goto out;
	}
	if (!v->last[0].idle_ok)
		dev_warn(
			g->dev,
			"compute retired seq=%u with idle=%08x,%08x,%08x set\n",
			seq, v->last[0].idle[0], v->last[0].idle[1],
			v->last[0].idle[2]);
	v->phase = G5P_RV3_DONE;
	ret = 0;
out:
	if (ret < 0 && ret != -ECANCELED && v->touched && !v->lost &&
	    !g5p_ru_term_failed(&g5p_rv3_term))
		g5p_rv3_terminal(g, g5p_ru_term_from_ret(ret), "cdm");
	g5p_cu_receipt(g, r, job, ret);
	return ret;
}

/* The scheduler's drain for a lost compute job (g5p_rv3_drain() hands engine 2 here): wait, RAM
 * only and for the drain bound, for the stuck job to retire.  It did: the channel is clear and
 * the held jobs run.  It did not (silent, fault, bound): the device recovers if it may, else the
 * epoch ends.  Returns false, having done nothing, when the core lock is busy.
 */
static bool g5p_cu_drain(struct g5p_drm *d)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_rstuck *s = rd->stuck;
	struct g5p_render *r = s->r;
	struct g5p_rv3 *v = &s->v;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	unsigned long deadline = jiffies + 2 * HZ;
	struct g5p_ru_wait w = {};
	u32 res = G5P_RU_WAIT_FAULT;
	int err = -EIO;
	u64 now;

	while (!mutex_trylock(&g->lock)) {
		if (time_after(jiffies, deadline))
			return false;
		msleep(5);
	}
	WRITE_ONCE(rd->executing, true);
	now = ktime_get_ns();
	g5p_ru_wait_start_lat(&w, now, 0, s->drain_ms,
			      now + (u64)s->drain_ms * NSEC_PER_MSEC,
			      READ_ONCE(b->events));
	if (!READ_ONCE(b->failed) && !READ_ONCE(b->stopped))
		res = g5p_cu_wait(d, g, s->seq, rd->cu_target, &w, &v->last[0],
				  &v->ta_consumed_ns, &err);
	if (res == G5P_RU_WAIT_COMPLETE) {
		r->lost_drained = true;
		dev_emerg(
			g->dev,
			"compute drained seq=%u slot=%u lost_ms=%lld: channel clear, held jobs resume\n",
			s->seq, r->slot[0],
			g5p_rv3_us(s->lost_ns, ktime_get_ns()) / 1000);
	} else {
		bool fault = res == G5P_RU_WAIT_FAULT;

		dev_emerg(
			g->dev,
			"compute drain dead seq=%u why=%s err=%d lost_ms=%lld: the device recovers if it may, else the epoch ends\n",
			s->seq,
			w.silent ? "silent" :
			fault	 ? "fault" :
				   "timeout",
			err, g5p_rv3_us(s->lost_ns, ktime_get_ns()) / 1000);
		if (g5p_rv3_recoverable(d, false))
			WRITE_ONCE(rd->recovering, true);
		else
			g5p_rv3_terminal(g,
					 fault ? G5P_RU_TERM_FAULT :
						 G5P_RU_TERM_TIMEOUT,
					 fault ? "cdm-drain-fault" :
						 "cdm-drain-timeout");
	}
	/* after the ruling: `recovering` or the terminal latch is visible first (g5p_rdev_ruling()) */
	smp_store_release(&rd->executing, false);
	rd->stuck = NULL;
	mutex_unlock(&g->lock);
	g5p_context_put(r->ctx);
	kfree(s);
	return true;
}

static void g5p_cu_pass_from_uapi(struct g5p_cu_pass *p,
				  const struct drm_g5p_compute_pass *u)
{
	*p = (struct g5p_cu_pass){
		.cdm_va = u->cdm_va,
		.cdm_size = u->cdm_size,
		.cdm_end = u->cdm_end,
		.tail = u->state_tail_va,
		.config = { u->config[0], u->config[1] },
		.store = { u->store.code, u->store.data, u->store.control },
		.resume = { u->resume.code, u->resume.data, u->resume.control },
		.term = { u->terminate.code, u->terminate.data,
			  u->terminate.control },
	};
	for (u32 i = 0; i < 4; i++)
		p->state[i] = u->state_va[i];
}

static bool g5p_cu_uapi_clean(const struct drm_g5p_compute_pass *u)
{
	return u->size == sizeof(*u) && !u->flags && !u->store.pad &&
	       !u->resume.pad && !u->terminate.pad &&
	       !memchr_inv(u->reserved, 0, sizeof(u->reserved));
}

/* Acceptance, as g5p_render_submit_admitted() for TYPED_V3 minus the render target: every error
 * precedes the fence replacement and the queue.
 */
static int g5p_cu_submit_admitted(struct drm_device *drm, struct drm_file *file,
				  const struct drm_g5p_submit_compute_typed *a,
				  u64 *job_id)
{
	static const struct g5p_ru_v3_limits lim = {
		G5P_RU_PDS_BASE,
		G5P_RU_CODE_WINDOW,
		G5P_RU_HW_VA_LIMIT,
	};
	struct g5p_drm *d = to_g5p(drm);
	struct g5p_file *f = file->driver_priv;
	struct g5p_boot *b = READ_ONCE(d->core->boot);
	struct drm_g5p_render_resource *res = NULL;
	struct drm_g5p_compute_pass *upass = NULL;
	struct drm_g5p_sync *syncs = NULL;
	struct drm_syncobj *signal = NULL;
	struct dma_fence_chain *chain = NULL;
	struct g5p_ru_range *ranges = NULL;
	struct g5p_cu_pass pass;
	struct g5p_cu_loc loc = {};
	struct g5p_context *c;
	struct g5p_render *r;
	struct g5p_rjob *job;
	u8 tail[4];
	u64 point = 0;
	int ret;

	if (!READ_ONCE(d->rdev->recovering)) {
		if (!b || !READ_ONCE(b->alive) || !READ_ONCE(b->released))
			return -EAGAIN;
		if (READ_ONCE(b->stopped))
			return -ESHUTDOWN;
		if (READ_ONCE(b->failed) && !g5p_rdev_ruling(d->rdev))
			return -EIO;
	}
	res = memdup_array_user(u64_to_user_ptr(a->resources),
				a->resource_count, sizeof(*res));
	if (IS_ERR(res))
		return PTR_ERR(res);
	syncs = memdup_array_user(u64_to_user_ptr(a->syncs), a->sync_count,
				  sizeof(*syncs));
	if (IS_ERR(syncs)) {
		ret = PTR_ERR(syncs);
		syncs = NULL;
		goto out;
	}
	upass = memdup_user(u64_to_user_ptr(a->pass), sizeof(*upass));
	if (IS_ERR(upass)) {
		ret = PTR_ERR(upass);
		upass = NULL;
		goto out;
	}
	if (!g5p_cu_uapi_clean(upass)) {
		ret = -EINVAL;
		goto out;
	}
	g5p_cu_pass_from_uapi(&pass, upass);
	ranges = kvcalloc(a->resource_count, sizeof(*ranges), GFP_KERNEL);
	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (job) {
		job->submit_ns = ktime_get_ns();
		job->objs = kvcalloc(a->resource_count, sizeof(*job->objs),
				     GFP_KERNEL);
	}
	if (!ranges || !job || !job->objs) {
		if (job)
			kvfree(job->objs);
		kfree(job);
		ret = -ENOMEM;
		goto out;
	}
	ret = g5p_render_syncs(file, job, syncs, a->sync_count, &signal, &point,
			       &chain);
	if (ret)
		goto out_job;
	job->fence = kzalloc(sizeof(*job->fence), GFP_KERNEL);
	if (!job->fence) {
		ret = -ENOMEM;
		goto out_job;
	}
	spin_lock_init(&job->fence->lock);
	ret = mutex_lock_interruptible(&d->render_submit_lock);
	if (ret)
		goto out_job;
	mutex_lock(&f->lock);
	c = xa_load(&f->contexts, a->context);
	if (!c || !c->render) {
		ret = c ? -EINVAL : -ENOENT;
		goto unlock;
	}
	r = c->render;
	if (c->bad || r->detached) {
		ret = -EIO;
		goto unlock;
	}
	if (r->fw_window == G5P_RU_FW_FULL && !capable(CAP_SYS_RAWIO)) {
		ret = -EPERM;
		goto unlock;
	}
	ret = g5p_rv3_admit_submit(d, r);
	if (ret)
		goto unlock;
	/* Compute rides an epoch a render job brought up; a recovery in progress is held. */
	if (!READ_ONCE(d->rdev->recovering) &&
	    g5p_cu_admit(READ_ONCE(d->rdev->up),
			 READ_ONCE(d->rdev->cu_sequence))) {
		ret = READ_ONCE(d->rdev->up) ? -EOVERFLOW : -EAGAIN;
		goto unlock;
	}
	if (atomic_read(&r->queued) >= G5P_RU_MAX_QUEUED) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = g5p_render_resolve(file, c->vm, res, a->resource_count, ranges,
				 job->objs, &job->obj_count);
	if (ret)
		goto unlock;
	/* Structure first; the stream's last dword is read only once its range is proven backed. */
	ret = g5p_cu_pass_check(&pass, &lim, ranges, a->resource_count, &loc);
	if (!ret)
		ret = g5p_render_read_va(drm, c->vm,
					 pass.cdm_va + pass.cdm_end - 4, tail,
					 sizeof(tail));
	if (!ret)
		ret = g5p_cu_content(tail);
	if (ret)
		goto unlock;
	g5p_cu_regs_of(&job->cu, &pass);
	ret = g5p_render_result_slot(f, d);
	if (ret)
		goto unlock;
	job->id = atomic64_inc_return(&d->job_sequence);
	dma_fence_init(&job->fence->base, &g5p_fence_ops, &job->fence->lock,
		       r->fence_context, ++r->fence_seqno);
	job->fence->state = DRM_G5P_JOB_PENDING;
	job->fence->error = -EINPROGRESS;
	ret = xa_insert(&f->results, job->id, job->fence, GFP_KERNEL);
	if (!ret) {
		ret = g5p_render_device_claim(d, c);
		if (ret)
			xa_erase(&f->results, job->id);
	}
	if (ret) {
		dma_fence_put(&job->fence->base);
		job->fence = NULL;
		goto unlock;
	}
	/* Accepted from here: every later step is infallible. */
	dma_fence_get(&job->fence->base);
	f->render_results++;
	job->v3 = true;
	job->compute = true;
	r->v3_accepted = true;
	atomic_inc(&r->queued);
	atomic_inc(&d->r8_queued);
	kref_get(&c->ref);
	job->ctx = c;
	job->gpu = d;
	mutex_unlock(&f->lock);
	if (point) {
		drm_syncobj_add_point(signal, chain, &job->fence->base, point);
		chain = NULL;
	} else {
		drm_syncobj_replace_fence(signal, &job->fence->base);
	}
	*job_id = job->id;
	job->deadline = jiffies + msecs_to_jiffies(G5P_RU_INPUT_MS);
	job->setup_error = g5p_render_implicit(job, r->fence_context);
	drm_dev_get(drm);
	g5p_rsched_push(d, r, job);
	mutex_unlock(&d->render_submit_lock);
	job = NULL;
	ret = 0;
	goto out_job;
unlock:
	mutex_unlock(&f->lock);
	mutex_unlock(&d->render_submit_lock);
out_job:
	if (signal)
		drm_syncobj_put(signal);
	dma_fence_chain_free(chain);
	if (job) {
		kfree(job->fence);
		job->fence = NULL;
		g5p_render_job_free(job);
	}
out:
	kvfree(ranges);
	kfree(upass);
	kfree(syncs);
	kfree(res);
	return ret;
}

static_assert(sizeof(struct drm_g5p_compute_pds) == 16);
static_assert(sizeof(struct drm_g5p_compute_pass) == 152);
static_assert(offsetof(struct drm_g5p_compute_pass, state_va) == 24);
static_assert(offsetof(struct drm_g5p_compute_pass, config) == 64);
static_assert(offsetof(struct drm_g5p_compute_pass, store) == 72);
static_assert(offsetof(struct drm_g5p_compute_pass, reserved) == 120);
static_assert(sizeof(struct drm_g5p_submit_compute_typed) == 72);
static_assert(DRM_G5P_CAP_TYPED_COMPUTE == BIT_ULL(30));
static_assert(DRM_G5P_COMPUTE_MAX_STREAM_BYTES == G5P_CU_MAX_STREAM);
static_assert(DRM_G5P_COMPUTE_STREAM_TERMINATE == G5P_CU_TERMINATE);

static int g5p_submit_compute_typed(struct drm_device *drm, void *data,
				    struct drm_file *file)
{
	struct drm_g5p_submit_compute_typed *a = data;
	u64 id = 0;
	int ret;

	if (a->flags || a->pad || a->job_id || a->reserved[0] ||
	    a->reserved[1] || !a->resource_count ||
	    a->resource_count > DRM_G5P_RENDER_MAX_RESOURCES ||
	    !a->sync_count || a->sync_count > DRM_G5P_RENDER_MAX_SYNCS ||
	    a->pass_size != sizeof(struct drm_g5p_compute_pass))
		return -EINVAL;
	if (!g5p_r8_enter())
		return -ESHUTDOWN;
	ret = g5p_cu_submit_admitted(drm, file, a, &id);
	g5p_r8_exit();
	a->job_id = id;
	return ret;
}
