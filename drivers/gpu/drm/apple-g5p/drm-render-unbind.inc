// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Unbinding firmware slots (docs/gpu/multi-client-design.md §4.5): a closed context's pair
 * (stage 2, core/render-user-close.h) and the render frontend's share of the reboot quiesce
 * (core/render-user-quiesce.h, docs/gpu/experiments/mcfold2-kexec.md).
 * Included from drm-render.inc after drm-render-v3.inc.
 */

/* The RAM words of `slot` and of its unbind record at ring entry `index`. */
static void g5p_render_slot_mem(struct g5p_boot *b, u32 slot, u32 index,
				struct g5p_ru_slot_mem *m)
{
	for (u32 j = 0; j < 5; j++) {
		u32 at = b->layout.offset[0x6d] + (index & 255) * 20 + j * 4;

		m->rec[j] =
			b->shared[at / G5P_GPU_PAGE].cpu + at % G5P_GPU_PAGE;
	}
	m->pa = g5p_boot_shared(b, 0x86) + slot * 8;
	m->host = g5p_boot_shared(b, 0x84) + slot * 4;
	m->cached = g5p_boot_shared(b, 0x85) + slot * 4;
}

/*
 * Unbind `n` firmware slots in one RAM-only DC3 batch: PA and host tag of each to zero, one
 * record {3, slot} per slot, one producer store, then the RAM-only wait of a bind and a check
 * that the firmware's DC3 handler copied the zero tag of every slot.  No doorbell and no
 * register access, so it is safe after a job retired and while the GPU may be idle-gated.
 *
 * The caller holds the core lock (the DC ring has one writer) and has retired every use of
 * these slots: no job of theirs is queued on the firmware or in flight.  On success the
 * firmware holds, per slot, PA 0, host and cached tag 0 and CBASE 0, with the slot's PC level
 * invalidated; the slot may be bound again by a later DC3.  Its translations stay allocated:
 * freeing them is the caller's step, after this returns 0.  A batch that is not consumed in
 * time keeps later binds and unbinds refused until the firmware catches up (dc_pending).
 */
static int g5p_rv3_unbind_slots(struct g5p_drm *d, const u32 *slots, u32 n,
				const char *why)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	struct g5p_ru_slot_sample s = {};
	struct g5p_ru_slot_mem m;
	u64 pub_ns, consumed_ns = 0;
	u32 start, cons, target, bad = 0, verified = 0;
	int ret;

	lockdep_assert_held(&g->lock);
	if (!g5p_ru_unbind_fits(n))
		return -EINVAL;
	for (u32 i = 0; i < n; i++)
		if (slots[i] < G5P_RU_SLOT_MIN || slots[i] > G5P_RU_SLOT_MAX)
			return -EINVAL;
	dma_rmb();
	start = READ_ONCE(ctl[4]);
	cons = READ_ONCE(ctl[0]);
	if (start > 255 || cons != start ||
	    (rd->dc_pending && cons != rd->dc_wait)) {
		dev_emerg(
			g->dev,
			"RENDER-V3 unbind why=%s refused slots=%u dc=%u/%u pending=%u: ring busy; no "
			"hardware touched end=v3\n",
			why, n, cons, start, rd->dc_pending);
		return -EBUSY;
	}
	rd->dc_pending = false;
	for (u32 i = 0; i < n; i++) {
		g5p_render_slot_mem(b, slots[i], start + i, &m);
		g5p_ru_unbind_publish(&m, slots[i]);
	}
	target = (start + n) & 255;
	dma_wmb();
	pub_ns = ktime_get_ns();
	smp_store_release(&ctl[4], target);
	dma_wmb();
	g5p_rv3_dc_kick(g, start, target, why);
	ret = g5p_rv3_dc_wait(g, target, &consumed_ns);
	if (ret == -ETIMEDOUT) {
		rd->dc_pending = true;
		rd->dc_wait = target;
	}
	dma_rmb();
	for (u32 i = 0; i < n; i++) {
		u32 one;

		g5p_render_slot_mem(b, slots[i], start + i, &m);
		g5p_ru_slot_read(&m, READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), &s);
		one = g5p_ru_unbind_check(&s, slots[i], target);
		bad |= one;
		verified += !one;
	}
	if (!ret && bad)
		ret = -EUCLEAN;
	dev_emerg(
		g->dev,
		"RENDER-V3 unbind why=%s slots=%u verified=%u ret=%d consumed_us=%lld "
		"dc=%u->%u/%u bad=%#x first=%u last=%u mode=ram-dc3 end=v3\n",
		why, n, verified, ret, g5p_rv3_us(pub_ns, consumed_ns), start,
		s.cons, s.prod, bad, slots[0], slots[n - 1]);
	return ret;
}

/*
 * render_user_close_park: bind a closed context's just-unbound pair to the park catalogue, one
 * DC3 pair as a context's bind (g5p_render_vm_prepare()), checked the same way.  The catalogue
 * holds the firmware window's entry and the borrowed sink entry, both stable within an epoch, so
 * rewriting them for a later park never clears what an earlier parked pair resolves through.
 * Core lock held, between jobs (the close item), RAM only and no register: as the unbind.
 */
static int g5p_rv3_park(struct g5p_drm *d, struct g5p_render *r)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	struct g5p_ru_bind_sample s = {};
	struct g5p_ru_bind_mem m;
	u64 pub_ns, consumed_ns = 0;
	u32 start, target, entry, sink, bad = 0;
	int ret;

	lockdep_assert_held(&g->lock);
	if (!rd->park.cpu) {
		ret = g5p_vm_alloc_page(d->drm.dev, &rd->park);
		if (ret)
			return ret;
		memset(rd->park.cpu, 0, G5P_GPU_PAGE);
	}
	ret = g5p_render_fw_entry(d, r, &entry);
	if (ret)
		return ret;
	dma_rmb();
	start = READ_ONCE(ctl[4]);
	g5p_render_bind_mem(b, r->slot, start, &m);
	g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), start, &s);
	if (start > 255 || rd->dc_pending || g5p_ru_bind_busy(&s)) {
		dev_emerg(
			g->dev,
			"RENDER-V3 park refused slots=%u/%u dc=%u/%u pending=%u busy=%#x: the pair stays "
			"unbound end=v3\n",
			r->slot[0], r->slot[1], s.cons, start, rd->dc_pending,
			g5p_ru_bind_busy(&s));
		return -EBUSY;
	}
	sink = b->sink_ready ? g5p_le32(b->catalogue.cpu) : 0;
	if (g5p_le32(rd->park.cpu + G5P_RU_FIRMWARE_CI * 4) != entry)
		g5p_put32(rd->park.cpu + G5P_RU_FIRMWARE_CI * 4, entry);
	if (g5p_le32(rd->park.cpu) != sink)
		g5p_put32(rd->park.cpu, sink);
	g5p_table_clean_page(g, &rd->park);
	dsb(sy);
	g5p_ru_bind_publish(&m, r->slot, rd->park.dma, G5P_RU_PARK_TAG);
	target = (start + 2) & 255;
	dma_wmb();
	pub_ns = ktime_get_ns();
	smp_store_release(&ctl[4], target);
	dma_wmb();
	g5p_rv3_dc_kick(g, start, target, "park");
	ret = g5p_rv3_dc_wait(g, target, &consumed_ns);
	if (ret == -ETIMEDOUT) {
		rd->dc_pending = true;
		rd->dc_wait = target;
	}
	dma_rmb();
	g5p_render_bind_mem(b, r->slot, start, &m);
	g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), READ_ONCE(ctl[4]), &s);
	if (!ret) {
		bad = g5p_ru_bind_check(&s, r->slot, rd->park.dma,
					G5P_RU_PARK_TAG, target);
		ret = bad ? -EUCLEAN : 0;
	}
	rd->parks += !ret;
	dev_emerg(
		g->dev,
		"RENDER-V3 park slots=%u/%u ret=%d consumed_us=%lld dc=%u->%u/%u bad=%#x "
		"parks=%u pc=%pad end=v3\n",
		r->slot[0], r->slot[1], ret, g5p_rv3_us(pub_ns, consumed_ns),
		start, s.cons, s.prod, bad, rd->parks, &rd->park.dma);
	return ret;
}

/* Drop what the claim of the device took (g5p_render_device_claim()); the device stays
 * claimed for render.  Returns whether a context reference was dropped.
 */
static bool g5p_render_unclaim(struct g5p_drm *d, struct g5p_render *r)
{
	if (!r->claimant)
		return false;
	r->claimant = false;
	if (d->quarantined_context == r->ctx)
		d->quarantined_context = NULL;
	return true;
}

/*
 * The close item of a TYPED_V3 context (multi-client-design.md "Stage 2/3 as built"): the
 * scheduler runs it after every job of the context, so every use of the pair has retired or
 * failed.  RAM only: the unbind is g5p_rv3_unbind_slots() for this pair under the core lock.
 * After a verified unbind the binding's references go, and the last one frees the VM tables,
 * render targets and mappings (unpinning the BOs) and returns the slots
 * (g5p_render_release()).  Anything the firmware may still hold is retained until reboot.
 */
/* perf60 (docs/gpu/experiments/perf60-clock.md), corrected by the release8 bench (bench-gaps3,
 * render-user-pmset.h): the PM writes a previous PM context back (its free list F, through that
 * context's TA slot) not once at the switch but on the later jobs of the next context, until a
 * third context runs.  A closed context therefore keeps its slots bound and its set and pages alive
 * while it is the last job's context or the previous distinct one; its unbind and release run once
 * it is neither.  At most those two wait.  (The first version held only the last job's context and
 * released it after one other job; 2 held one job longer, by job records: both released it while
 * the PM still wrote to it.)  RAM only: bookkeeping and the close item's references.
 */
/* release8 default */

/*
 * A close item follows every job of this context on the ordered render worker.
 * resident_gone, set under the file lock before queueing it, rejects all later
 * VM_BIND and context creation. Thus the close item owns vm->maps exclusively.
 *
 * Idle VM_BIND UNMAP already drops a user mapping's pages after retirement.
 * Do that here without releasing the PM's write-back route: no unbind, no
 * table pruning, no PM set/pool/aux/RT release, no pm_last/pm_prev change. The
 * PM free list is in the firmware window, never in vm->maps. All user mappings
 * must pass the same range check as VM_BIND before any PTE is touched.
 *
 * No new kick of this detached context can occur. Clean the cleared user PTEs
 * to PoC, but do not publish a tag, invalidate a slot or access R0. The PM can
 * keep using its unchanged translations and pages, including cached ones.
 * Failed/queued/terminal/shutdown epochs keep every BO, as before.
 */
static void g5p_render_close_trim(struct g5p_render *r)
{
	struct g5p_context *c = r->ctx;
	struct g5p_drm *d = c->gpu;
	struct g5p_diag *g = d->core;
	struct g5p_vm *vm = c->vm;
	struct g5p_boot *b;
	struct g5p_ru_close_in in;
	struct g5p_mapping *m, *tmp;
	struct g5p_pt *pt;
	u64 pinned;
	u32 maps, why;

	if (!vm->map_count || !vm->resident_gone || !r->closing ||
	    !r->detached || !mutex_trylock(&g->lock))
		return;
	b = g->boot;
	in = (struct g5p_ru_close_in){
		.retained = r->retained,
		.bound = r->bound,
		/* Match close_run: a successfully drained lost job has no user-memory use left. */
		.bad = READ_ONCE(c->bad) && !r->lost_drained,
		.term_failed = g5p_ru_term_failed(&g5p_rv3_term),
		.dev_failed = !b || READ_ONCE(b->failed),
		.stopped = !b || READ_ONCE(b->stopped),
		.shutdown = g5p_r8_admit_closed(&g5p_r8_state.admit),
		.dc_pending = d->rdev->dc_pending,
		.queued = atomic_read(&r->queued),
	};
	if (g5p_ru_close_plan(&in, &why) != G5P_RU_CLOSE_UNBIND)
		goto out;
	list_for_each_entry(m, &vm->maps, link)
		if (!g5p_user_range_valid(m->node.start, m->node.size) ||
		    (m->node.start < G5P_RU_PRIVATE_VA + G5P_RU_PRIVATE_SIZE &&
		     G5P_RU_PRIVATE_VA < m->node.start + m->node.size))
			goto out;

	pinned = vm->pinned_bytes;
	maps = vm->map_count;
	list_for_each_entry(m, &vm->maps, link)
		g5p_vm_clear(vm, d->drm.dev, m->node.start, m->node.size);
	g5p_table_clean_page(g, &vm->root);
	list_for_each_entry(pt, &vm->tables, link)
		g5p_table_clean_page(g, &pt->page);
	dsb(sy);
	list_for_each_entry_safe(m, tmp, &vm->maps, link) {
		vm->mapped_bytes -= m->node.size;
		drm_mm_remove_node(&m->node);
		list_del(&m->link);
		vm->map_count--;
		g5p_vm_obj_put(vm, m->obj);
		drm_gem_object_put(m->obj);
		kfree(m);
	}
	dev_info(
		g->dev,
		"RENDER-V3 close-trim slots=%u/%u maps=%u unpinned=%lluK remaining=%lluK "
		"device_pinned=%lluK pm=retained end=v3\n",
		r->slot[0], r->slot[1], maps, (pinned - vm->pinned_bytes) >> 10,
		vm->pinned_bytes >> 10, READ_ONCE(d->pinned_bytes) >> 10);
out:
	mutex_unlock(&g->lock);
}

/* render_user_close_defer: may the firmware's PM still write this context's state back? */
static bool g5p_render_pm_holds(struct g5p_rdev *rd, struct g5p_render *r,
				u32 mode)
{
	return mode && g5p_pmset_holds(rd->pm_last, rd->pm_prev, r);
}

static void g5p_render_close_run(struct g5p_render *r)
{
	struct g5p_context *c = r->ctx;
	struct g5p_drm *d = c->gpu;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_ru_close_in in;
	unsigned long deadline = jiffies + 2 * HZ;
	u32 puts = 0, why;
	bool locked = false;
	int plan, ret = 0;
	u32 mode, k;

	mode = (1);
	k = d->rdev->close_deferred[0] ? 1 : 0;
	if (mode && !r->close_defer_run && r->bound &&
	    g5p_render_pm_holds(d->rdev, r, mode) &&
	    k < ARRAY_SIZE(d->rdev->close_deferred)) {
		g5p_render_close_trim(r);
		/* the deferred close's own references, dropped by g5p_render_close_deferred() */
		kref_get(&c->ref);
		drm_dev_get(&d->drm);
		d->rdev->close_deferred[k] = r;
		dev_info(
			g->dev,
			"RENDER-V3 close-defer slots=%u/%u jobs=%u: the PM may still write its state "
			"back; unbind once it has moved past it end=v3\n",
			r->slot[0], r->slot[1], r->jobs);
		dev_info(
			g->dev,
			"RENDER-V3 close-defer2 slots=%u/%u pm=%s pending=%u end=v3\n",
			r->slot[0], r->slot[1],
			d->rdev->pm_last == r ? "last" : "previous", k + 1);
		return;
	}

	for (;;) {
		in = (struct g5p_ru_close_in){
			.retained = r->retained,
			.bound = r->bound,
			/* a lost job that drained left the firmware nothing of this context */
			.bad = READ_ONCE(c->bad) && !r->lost_drained,
			.term_failed = g5p_ru_term_failed(&g5p_rv3_term),
			.dev_failed = !b || READ_ONCE(b->failed),
			.stopped = !b || READ_ONCE(b->stopped),
			.shutdown = g5p_r8_admit_closed(&g5p_r8_state.admit),
			/* as g5p_rv3_unbind_slots() refuses it */
			.dc_pending = b && d->rdev->dc_pending &&
				      READ_ONCE(((u32 *)g5p_boot_shared(
					      b, 0x6c))[0]) != d->rdev->dc_wait,
			.queued = atomic_read(&r->queued),
		};
		plan = g5p_ru_close_plan(&in, &why);
		if (plan != G5P_RU_CLOSE_UNBIND || locked)
			break;
		/* Bounded, as a job takes it: never wait forever behind a command in MMIO. */
		while (!(locked = mutex_trylock(&g->lock)) &&
		       time_before(jiffies, deadline))
			msleep(5);
		if (!locked) {
			plan = G5P_RU_CLOSE_RETAIN;
			why = G5P_RU_CLOSE_LOCK;
			break;
		}
		/* Plan again under the lock: the reboot quiesce or a failure may have run. */
	}
	if (plan == G5P_RU_CLOSE_UNBIND) {
		ret = g5p_rv3_unbind_slots(d, r->slot, 2, "close");
		if (!ret) {
			r->bound = false;
			r->retained = false;
			WRITE_ONCE(c->live, false);
			puts++;
			/* the PM may still write this context's free list back through its TA slot */
			{
				g5p_rv3_park(d, r);
			};
		}
	}
	if (plan == G5P_RU_CLOSE_RELEASE ||
	    (plan == G5P_RU_CLOSE_UNBIND && !ret))
		puts += g5p_render_unclaim(d, r);
	if (locked)
		mutex_unlock(&g->lock);
	dev_info(
		g->dev,
		"RENDER-V3 close slots=%u/%u plan=%s why=%#x ret=%d jobs=%u claimant=%u "
		"retained=%u end=v3\n",
		r->slot[0], r->slot[1], g5p_ru_close_name(plan), why, ret,
		r->jobs, r->claimant, r->retained);
	while (puts--) {
		g5p_context_put(c);
		drm_dev_put(&d->drm);
	}
}

/* After a job: a deferred close runs once the last job was another context's. */
static void g5p_render_close_deferred(struct g5p_drm *d)
{
	struct g5p_rdev *rd = d->rdev;
	u32 mode = (1);

	for (u32 k = 0; k < ARRAY_SIZE(rd->close_deferred); k++) {
		struct g5p_render *r = rd->close_deferred[k];
		struct g5p_context *c;

		/* mode 0 now: a pending close still waits until the PM has moved past it */
		if (!r || g5p_render_pm_holds(rd, r, mode ?: 1))
			continue;
		rd->close_deferred[k] = NULL;
		c = r->ctx;
		r->close_defer_run = true;
		dev_info(
			d->core->dev,
			"RENDER-V3 close-deferred slots=%u/%u: neither the PM's last nor its previous "
			"context end=v3\n",
			r->slot[0], r->slot[1]);
		g5p_render_close_run(r);
		g5p_context_put(c);
		drm_dev_put(&d->drm);
	}
	/* keep a pending close in [0]: close_run fills [1] only behind it */
	if (!rd->close_deferred[0] && rd->close_deferred[1]) {
		rd->close_deferred[0] = rd->close_deferred[1];
		rd->close_deferred[1] = NULL;
	}
}

/*
 * The handle of a TYPED_V3 context is gone (DESTROY_CONTEXT, or its file closed): stop VM and
 * render-target edits and queue the close item behind the context's jobs (g5p_rsched_close()).
 * File lock held, or the file is closing.  A context that never submitted is released by its
 * last reference.
 */
static void g5p_render_close(struct g5p_context *c)
{
	struct g5p_render *r = c->render;

	if (!r->v3_accepted || r->closing)
		return;
	r->closing = true;
	r->detached = true;
	c->vm->resident_gone = true;
	/* The close item's references, dropped by the scheduler once it ran. */
	kref_get(&c->ref);
	drm_dev_get(&c->gpu->drm);
	if (!g5p_rsched_close(c->gpu, r)) {
		g5p_context_put(c);
		drm_dev_put(&c->gpu->drm);
	}
}

/* Reboot quiesce, RAM only and lock free: the TYPED_V3 latch and whether its executor runs.
 * The executor is RAM-only once the latch is quiet, unless the wake110 experiment (doorbells
 * after retirement, off in every desktop tuple) is configured.  Called after admission closed;
 * the scheduler is kicked so that jobs waiting on an input fail now, not at their deadline.
 */
static void g5p_kq_drm_state(void *cookie, struct g5p_ru_kq_in *in)
{
	struct g5p_drm *d = cookie;

	/* Admission is closed by now: queued jobs blocked on an input fail at once. */
	g5p_rsched_kick(d);

	/* A channel blocked behind a lost job is a wedged epoch for the quiesce. */
	in->term_failed = g5p_ru_term_failed(&g5p_rv3_term) ||
			  READ_ONCE(d->rdev->stuck) ||
			  READ_ONCE(d->rdev->recovering);
	in->ram_only = READ_ONCE(d->rdev->v3) &&
		       !g5p_ru_mmio_ok(&g5p_rv3_term) && !(2);
	in->executing = READ_ONCE(d->rdev->executing);
}

/* Reboot quiesce, core lock held: every held user slot, the first claimant's pair included.
 * Nothing is freed and no context changes state: admission is closed and the boot state is
 * frozen right after, so no job runs again on this boot.
 */
static int g5p_kq_drm_detach(void *cookie, const u32 *slots, u32 n)
{
	return g5p_rv3_unbind_slots(cookie, slots, n, "kexec");
}

static const struct g5p_kq_frontend g5p_kq_drm = {
	.state = g5p_kq_drm_state,
	.detach = g5p_kq_drm_detach,
};
