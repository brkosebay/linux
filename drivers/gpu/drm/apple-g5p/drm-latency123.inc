/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Latency123: runtime A/B knobs for the V3 render path, all default off and all writable
 * through /sys/module/a9_gpu_drm/parameters on a live boot.  Included from drm-render.inc
 * before drm-render-v3.inc.  The executor is serial and holds the core lock, so the state
 * below needs no lock of its own.
 *
 *   fw_tick_mask (boot.inc)  ctl+0x04, re-applied before every job (RAM)
 *   slc_evict_us             while a job is in flight, one pass over slc_evict_mib MiB of
 *                            cacheable kernel memory at most every N us (the waiter's
 *                            1-2 ms sample cadence bounds it from below); slc_evict_ops:
 *                            1 read pass, 2 write pass, 4 Normal-NC read of the firmware's
 *                            root+shared pages, 8 TLBI VMALLE1IS, 16 DSB SY + SEV
 *   cpu_sync                 1 dma_sync every job BO (device before, CPU after), 2 clean
 *                            root+shared to PoC before the publish, 4 clean+invalidate the
 *                            polled pages before every sample
 *   lat123_log               one RENDER-L123 line per job (times in ms from the submit ioctl)
 *   pstate_probe             1 PMGR gfx word at job start/end (always-on block), 2 also one
 *                            read of Fender +0x1000 at the first sample that shows a fresh
 *                            engine busy rise (engine running, so the block is powered)
 *
 * No other register access.  Temporary: none of this is the fix.
 */

#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#define G5P_L123_OP_READ BIT(0)
#define G5P_L123_OP_WRITE BIT(1)
#define G5P_L123_OP_FW_READ BIT(2)
#define G5P_L123_OP_TLBI BIT(3)
#define G5P_L123_OP_SEV BIT(4)
#define G5P_L123_SYNC_BO BIT(0)
#define G5P_L123_SYNC_REGIONS BIT(1)
#define G5P_L123_SYNC_POLL BIT(2)
#define G5P_L123_LINE 64

static struct {
	void *buf;
	size_t size;
	u64 sink;
	/* per job */
	u64 exec_ns, next_ns, evict_ns, ps_ns;
	u32 us, mib, ops, sync, passes, tick, ps;
	u32 pmgr[2];
	bool seen[2], idle[2], probed;
	/* render_user_event_wake: pauses ended by an event, the timeout, a post-event poll, legacy */
	u32 ew[4];
	u64 ew_ns;
} g5p_l123;

/* perf60-fillrate.md §11.3 cut 1 (render-user-evwait.h): the phase wait's pause between RAM samples. */
static unsigned int render_user_event_wake = 2000; /* release8 default */
module_param(render_user_event_wake, uint, 0600);
MODULE_PARM_DESC(
	render_user_event_wake,
	"perf60 default0 (release8: sleep 1-2 ms between completion samples); N: sleep "
	"until the next firmware event (the IRQ thread's decode wakes the waiter), at "
	"most N us (100..20000) as the safety net, and poll every 100 us once the "
	"phase's own completion event is latched; read live");

/* A word of a firmware object; objects may straddle the region's 16 KiB pages. */
static u8 *g5p_l123_word(struct g5p_boot *b, u32 slot, u32 field)
{
	u32 off = b->layout.offset[slot] + field;
	struct g5p_dma_page *page = b->layout.group[slot] ? b->shared : b->root;

	return page[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE;
}

static void g5p_l123_pages(struct g5p_dma_page *page, u32 count,
			   void (*op)(unsigned long start, unsigned long end))
{
	for (u32 i = 0; i < count; i++)
		if (page[i].cpu)
			op((unsigned long)page[i].cpu,
			   (unsigned long)page[i].cpu + G5P_GPU_PAGE);
}

static void g5p_l123_page_of(const void *p,
			     void (*op)(unsigned long start, unsigned long end))
{
	unsigned long start = (unsigned long)p &
			      ~(unsigned long)(G5P_GPU_PAGE - 1);

	op(start, start + G5P_GPU_PAGE);
}

/* cpu_sync 4: the pages the waiter samples (rings, TA/3D/PM pages, the idle words). */
static void g5p_l123_poll_pages(struct g5p_boot *b, u32 e)
{
	g5p_l123_page_of(g5p_boot_shared(b, 0x60), dcache_clean_inval_poc);
	g5p_l123_page_of(g5p_boot_shared(b, 0x64), dcache_clean_inval_poc);
	g5p_l123_page_of(g5p_ru_page(b, 0), dcache_clean_inval_poc);
	g5p_l123_page_of(g5p_ru_page(b, 1), dcache_clean_inval_poc);
	g5p_l123_page_of(g5p_ru_pm_cpu(b), dcache_clean_inval_poc);
	g5p_l123_page_of(g5p_ru_global(b, e) + 0x724, dcache_clean_inval_poc);
	dsb(sy);
}

static void g5p_l123_bos(struct g5p_drm *d, struct g5p_rjob *job, bool to_cpu)
{
	for (u32 i = 0; i < job->obj_count; i++) {
		struct sg_table *sgt =
			to_drm_gem_shmem_obj(job->objs[i].obj)->sgt;

		if (!sgt)
			continue;
		if (to_cpu)
			dma_sync_sgtable_for_cpu(d->drm.dev, sgt,
						 DMA_BIDIRECTIONAL);
		else
			dma_sync_sgtable_for_device(d->drm.dev, sgt,
						    DMA_BIDIRECTIONAL);
	}
}

static u32 g5p_l123_pmgr(struct g5p_diag *g)
{
	u32 raw = U32_MAX;

	if (!((0) & 1) || !g->pmgr || regmap_read(g->pmgr, 0x80340, &raw))
		return U32_MAX;
	return raw;
}

/* At the executor's entry: this job's knob values and counters. */
static void g5p_l123_job_reset(struct g5p_diag *g)
{
	g5p_l123.exec_ns = ktime_get_ns();
	g5p_l123.next_ns = 0;
	g5p_l123.evict_ns = 0;
	g5p_l123.ps_ns = 0;
	g5p_l123.passes = 0;
	g5p_l123.ps = U32_MAX;
	g5p_l123.probed = false;
	memset(g5p_l123.ew, 0, sizeof(g5p_l123.ew));
	g5p_l123.ew_ns = 0;
	g5p_l123.seen[0] = false;
	g5p_l123.seen[1] = false;
	g5p_l123.us = (0);
	g5p_l123.mib = clamp((16), 1U, 64U);
	g5p_l123.ops = (1);
	g5p_l123.sync = (0);
	g5p_l123.pmgr[0] = g5p_l123_pmgr(g);
	g5p_l123.pmgr[1] = U32_MAX;
}

/* After the job's records are written, before they are cleaned and published.  May sleep. */
static void g5p_l123_job_prepare(struct g5p_drm *d, struct g5p_rjob *job)
{
	struct g5p_boot *b = d->core->boot;
	u8 *tick = g5p_l123_word(b, G5P_L123_CTL, 4);
	u32 mask = (0);
	size_t size = (size_t)g5p_l123.mib << 20;

	if (mask <= G5P_L123_TICK_MASK && g5p_le32(tick) != mask) {
		g5p_put32(tick, mask);
		dma_wmb();
	}
	g5p_l123.tick = g5p_le32(tick);

	if (!g5p_l123.us ||
	    !(g5p_l123.ops & (G5P_L123_OP_READ | G5P_L123_OP_WRITE))) {
		vfree(g5p_l123.buf);
		g5p_l123.buf = NULL;
		g5p_l123.size = 0;
	} else if (g5p_l123.size != size) {
		vfree(g5p_l123.buf);
		g5p_l123.buf = vmalloc(size);
		g5p_l123.size = g5p_l123.buf ? size : 0;
		if (g5p_l123.buf)
			memset(g5p_l123.buf, 0x5a, size);
	}

	if (g5p_l123.sync & G5P_L123_SYNC_BO)
		g5p_l123_bos(d, job, false);
	if (g5p_l123.sync & G5P_L123_SYNC_REGIONS) {
		g5p_l123_pages(b->root, ARRAY_SIZE(b->root), dcache_clean_poc);
		g5p_l123_pages(b->shared, ARRAY_SIZE(b->shared),
			       dcache_clean_poc);
		dsb(sy);
	}
	if (g5p_l123.sync & G5P_L123_SYNC_POLL)
		g5p_l123_poll_pages(b, 0);
}

/* One streaming pass and the other per-pass operations. */
static void g5p_l123_evict(struct g5p_boot *b)
{
	u64 t0 = ktime_get_ns(), sum = 0;
	u32 ops = g5p_l123.ops;

	if ((ops & G5P_L123_OP_READ) && g5p_l123.buf) {
		u64 *p = g5p_l123.buf;

		for (size_t i = 0; i < g5p_l123.size / 8;
		     i += G5P_L123_LINE / 8)
			sum += READ_ONCE(p[i]);
	}
	if ((ops & G5P_L123_OP_WRITE) && g5p_l123.buf) {
		u64 *p = g5p_l123.buf;

		for (size_t i = 0; i < g5p_l123.size / 8;
		     i += G5P_L123_LINE / 8)
			WRITE_ONCE(p[i], i);
	}
	if (ops & G5P_L123_OP_FW_READ) {
		struct g5p_dma_page *regions[] = { b->root, b->shared };
		u32 counts[] = { ARRAY_SIZE(b->root), ARRAY_SIZE(b->shared) };

		for (u32 r = 0; r < ARRAY_SIZE(regions); r++)
			for (u32 i = 0; i < counts[r]; i++)
				for (u32 at = 0;
				     regions[r][i].cpu && at < G5P_GPU_PAGE;
				     at += G5P_L123_LINE)
					sum += READ_ONCE(*(
						const u32 *)(regions[r][i].cpu +
							     at));
	}
	if (ops & G5P_L123_OP_TLBI)
		flush_tlb_all();
	if (ops & G5P_L123_OP_SEV) {
		dsb(sy);
		sev();
	}
	WRITE_ONCE(g5p_l123.sink, g5p_l123.sink + sum);
	g5p_l123.passes++;
	g5p_l123.evict_ns += ktime_get_ns() - t0;
}

/* Replaces the waiter's sleep between two RAM samples of engine e.  seen: d->event_serial as
 * read after the drain and before this sample (render-user-evwait.h).
 */
static void g5p_l123_pause(struct g5p_drm *d, struct g5p_diag *g, u32 e,
			   const struct g5p_rv3_ev *s, u64 seen)
{
	u64 now = s->ns, t0;
	enum g5p_ew_act act;
	u32 wait_us;

	if (((0) & 2) && !g5p_l123.probed && g5p_l123.seen[e] &&
	    g5p_l123.idle[e] && !s->idle_ok && !s->failed && !s->proto &&
	    !g5p_ru_term_failed(&g5p_rv3_term) && !READ_ONCE(g->boot->failed)) {
		struct g5p_mmio_ticket t;

		/* an R1-only ticket of the register licence (mmio.inc), never a bare read */
		if (!g5p_mmio_hold(g, &t, G5P_MMIO_CENSUS))
			g5p_mmio_release(g, &t);
		g5p_l123.ps = t.gpc;
		g5p_l123.ps_ns = ktime_get_ns();
		g5p_l123.probed = true;
	}
	g5p_l123.seen[e] = true;
	g5p_l123.idle[e] = s->idle_ok;
	if (g5p_l123.us && now >= g5p_l123.next_ns) {
		g5p_l123_evict(g->boot);
		g5p_l123.next_ns = now + (u64)g5p_l123.us * NSEC_PER_USEC;
	}
	act = g5p_ew_plan(READ_ONCE(render_user_event_wake), s->event,
			  &wait_us);
	t0 = ktime_get_ns();
	if (act == G5P_EW_LEGACY) {
		usleep_range(1000, 2000);
		g5p_l123.ew[3]++;
	} else if (act == G5P_EW_SHORT) {
		usleep_range(wait_us, wait_us + wait_us / 2);
		g5p_l123.ew[2]++;
	} else if (wait_event_hrtimeout(
			   d->ev_wq,
			   g5p_ew_woken(seen, READ_ONCE(d->event_serial)),
			   us_to_ktime(wait_us))) {
		g5p_l123.ew[1]++; /* -ETIME: the safety net */
	} else {
		g5p_l123.ew[0]++;
	}
	g5p_l123.ew_ns += ktime_get_ns() - t0;
	if (g5p_l123.sync & G5P_L123_SYNC_POLL)
		g5p_l123_poll_pages(g->boot, e);
}

/* After the receipt.  One line per job while lat123_log is set. */
static void g5p_l123_job_end(struct g5p_drm *d, struct g5p_rjob *job, u32 seq,
			     int ret)
{
	struct g5p_diag *g = d->core;

	if (!ret && (g5p_l123.sync & G5P_L123_SYNC_BO))
		g5p_l123_bos(d, job, true);
	g5p_l123.pmgr[1] = g5p_l123_pmgr(g);
	{
		return;
	}
}
