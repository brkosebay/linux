// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The render scheduler (docs/gpu/multi-client-design.md "Stage 2/3 as built"): one worker on
 * the ordered render_wq serves the contexts round-robin (core/render-user-sched.h).  Jobs wait
 * for their inputs on dma-fence callbacks, not in the worker, so one client's unsignalled
 * fence never holds up another client.  Each picked job runs to completion on the one shared
 * channel (one credit), exactly as a work item did before; a context's close item runs after
 * its last job.  Included from drm-render.inc after drm-render-v3.inc.
 *
 * The shape is drm_sched's (entity FIFO, run-list RR, dependency callbacks, entity teardown
 * after the last job), so stage 4 can replace it with drm_sched without a UAPI change.
 */

static struct dma_fence *g5p_rjob_dep(struct g5p_rjob *job, u32 i)
{
	return i < job->input_count ? job->inputs[i] :
				      job->deps[i - job->input_count];
}

/* A signalled input: its error, or that of any fence it wraps (array, chain), else 0. */
static int g5p_render_fence_status(struct dma_fence *fence)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *part;
	u32 unwrapped = 0;
	int ret = dma_fence_get_status(fence);

	if (ret < 0)
		return ret;
	dma_fence_unwrap_for_each(part, &cursor, fence) {
		ret = dma_fence_get_status(part);
		if (ret <= 0 || ++unwrapped > 4096) {
			dma_fence_put(cursor.chain);
			return ret < 0 ? ret : -ETIMEDOUT;
		}
	}
	return 0;
}

/* Fence callback (any context, the fence lock held): the head job's input signalled. */
static void g5p_rjob_dep_cb(struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct g5p_rjob *job = container_of(cb, struct g5p_rjob, cb);

	WRITE_ONCE(job->cb_fired, true);
	mod_delayed_work(job->gpu->render_wq, &job->gpu->rdev->sched_work, 0);
}

/* Scheduler lock held: no callback of this job stays armed. */
static void g5p_rjob_disarm(struct g5p_rjob *job)
{
	if (job->cb_fence)
		dma_fence_remove_callback(job->cb_fence, &job->cb);
	job->cb_fence = NULL;
}

/*
 * Scheduler lock held: may the head job of its context run now?  Walks its explicit inputs,
 * then its implicit dependencies, in order; at the first unsignalled one it arms a callback
 * and reports the job's input deadline.  A failed input, the deadline (G5P_RU_INPUT_MS after
 * the submit, as the old in-worker wait) and closed admission make it ready with an error, so
 * that it fails before the hardware and its fence still signals in order.
 */
static bool g5p_rsched_ready(struct g5p_rs_job *node, void *cookie,
			     unsigned long *deadline)
{
	struct g5p_rjob *job = container_of(node, struct g5p_rjob, node);

	if (job->setup_error || job->dep_error)
		return true;
	if (g5p_r8_admit_closed(&g5p_r8_state.admit)) {
		g5p_rjob_disarm(job);
		job->dep_error = -ESHUTDOWN;
		return true;
	}
	if (job->cb_fence) {
		if (!READ_ONCE(job->cb_fired))
			goto pending;
		job->cb_fence = NULL;
	}
	while (job->dep_next < job->input_count + job->dep_count) {
		struct dma_fence *fence = g5p_rjob_dep(job, job->dep_next);

		if (dma_fence_is_signaled(fence)) {
			job->dep_error = g5p_render_fence_status(fence);
			if (job->dep_error)
				return true;
			job->dep_next++;
			continue;
		}
		WRITE_ONCE(job->cb_fired, false);
		if (dma_fence_add_callback(fence, &job->cb, g5p_rjob_dep_cb))
			continue; /* -ENOENT: signalled meanwhile */
		job->cb_fence = fence;
		goto pending;
	}
	return true;
pending:
	if (time_after_eq(jiffies, job->deadline)) {
		g5p_rjob_disarm(job);
		job->dep_error = -ETIMEDOUT;
		return true;
	}
	*deadline = job->deadline;
	return false;
}

static void g5p_rsched_kick(struct g5p_drm *d)
{
	mod_delayed_work(d->render_wq, &d->rdev->sched_work, 0);
}

/* Submit, render_submit_lock held: an accepted job joins its context's FIFO. */
static void g5p_rsched_push(struct g5p_drm *d, struct g5p_render *r,
			    struct g5p_rjob *job)
{
	unsigned long flags;

	spin_lock_irqsave(&d->rdev->sched_lock, flags);
	g5p_rs_push(&d->rdev->sched, &r->entity, &job->node);
	spin_unlock_irqrestore(&d->rdev->sched_lock, flags);
	g5p_rsched_kick(d);
}

/* The close item joins the context's entity behind its jobs; false if one is queued. */
static bool g5p_rsched_close(struct g5p_drm *d, struct g5p_render *r)
{
	unsigned long flags;
	bool queued;

	spin_lock_irqsave(&d->rdev->sched_lock, flags);
	queued = g5p_rs_close(&d->rdev->sched, &r->entity);
	spin_unlock_irqrestore(&d->rdev->sched_lock, flags);
	if (queued)
		g5p_rsched_kick(d);
	return queued;
}

static void g5p_render_run(struct g5p_rjob *job);
static void g5p_render_close_run(struct g5p_render *r);
static void g5p_render_close_deferred(struct g5p_drm *d);

/*
 * The worker: pick, run to completion, repeat; then sleep until a callback, a submit or the
 * earliest input deadline wakes it.  render_wq is ordered, so this never runs twice at once,
 * and it is the only writer of the device's job state, as each job's work item was.
 */
static void g5p_rsched_work(struct work_struct *work)
{
	struct g5p_rdev *rd = container_of(to_delayed_work(work),
					   struct g5p_rdev, sched_work);
	struct g5p_drm *d = rd->drm;

	/* Each item drops the references it held; the device must outlive this loop. */
	drm_dev_get(&d->drm);
	for (;;) {
		struct g5p_rs_entity *e;
		struct g5p_rs_job *node;
		struct g5p_render *r;
		struct g5p_context *c;
		unsigned long flags, wake = 0;
		bool has_wake;
		int what;

		/* A lost job blocks the channel: hold every job until the drain is decided, and
		 * until a recovery the drain or the executor asked for has run.
		 */
		if (READ_ONCE(rd->stuck) || READ_ONCE(rd->recovering)) {
			if (rd->stuck ? g5p_rv3_drain(d) : g5p_rv3_recover(d))
				continue;
			queue_delayed_work(d->render_wq, &rd->sched_work,
					   msecs_to_jiffies(100));
			break;
		}
		spin_lock_irqsave(&rd->sched_lock, flags);
		what = g5p_rs_pick(&rd->sched, g5p_rsched_ready, d, &e, &node,
				   &wake, &has_wake);
		spin_unlock_irqrestore(&rd->sched_lock, flags);
		if (what == G5P_RS_IDLE) {
			if (has_wake)
				queue_delayed_work(
					d->render_wq, &rd->sched_work,
					max_t(long, (long)(wake - jiffies), 1));
			break;
		}
		if (what == G5P_RS_JOB) {
			g5p_render_run(
				container_of(node, struct g5p_rjob, node));
			g5p_render_close_deferred(d);
			continue;
		}
		r = container_of(e, struct g5p_render, entity);
		c = r->ctx;
		g5p_render_close_run(r);
		spin_lock_irqsave(&rd->sched_lock, flags);
		g5p_rs_closed(&rd->sched, e);
		spin_unlock_irqrestore(&rd->sched_lock, flags);
		/* The close item's own reference (g5p_render_close()); r may go with it. */
		g5p_context_put(c);
		drm_dev_put(&d->drm);
	}
	drm_dev_put(&d->drm);
}
