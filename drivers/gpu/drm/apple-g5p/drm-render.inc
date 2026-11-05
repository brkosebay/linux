/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Userspace TA/3D render: render contexts, render targets and SUBMIT_RENDER.
 * Builds on the r22 constructors through render-user-layout.h (byte-equal for
 * the round16 inputs). Default off.  The first accepted job claims the device
 * for render; TYPED_V3 contexts then share it, each with its own VM and slot
 * pair (docs/gpu/multi-client-design.md), the other profiles stay with that
 * first context.  Jobs of all contexts are serialized; no reset/recovery: any
 * hardware failure retains everything.
 */
#include "../../apple-g5p/render-user-layout.h"
#include "../../apple-g5p/render-user-wait.h"
#include "../../apple-g5p/render-user-touch.h"
#include "../../apple-g5p/render-user-retire.h"
#include "../../apple-g5p/render-user-wake.h"
#include "../../apple-g5p/render-user-serial.h"
#include "../../apple-g5p/render-user-terminal.h"
#include "../../apple-g5p/render-user-bind.h"
#include "../../apple-g5p/render-user-close.h"
#include "../../apple-g5p/render-user-sched.h"
#include "../../apple-g5p/render-user-window.h"
#include "../../apple-g5p/render-user-msaa.h"
#include "../../apple-g5p/render-user-loss.h"
#include "../../apple-g5p/render-user-fault.h"
#include "../../apple-g5p/render-user-mmu.h"
#include "../../apple-g5p/render-user-faultreg.h"
#include "render-user-validate.h"
#include "drm-render-v3.h"
#include "../../apple-g5p/compute-user-layout.h"
#include "compute-user-validate.h"
#include "drm-compute-v1.h"

/* The terminal state of the epoch (one GPU): from the first terminal note on, no register access. */
static struct g5p_ru_terminal g5p_rv3_term;

/* Defaults of the latched knobs. GET_RENDER_INFO reports VARIANT_ACTIVE when
 * any knob differs, so the declarations below and that comparison share them.
 */
#define G5P_RU_DEF_CACHE_VARIANT 3
#define G5P_RU_DEF_MMU_VARIANT 1
/* 20 s: the natural TA completion took 12.006 s in the r96 boots a 12 s cap turned into
 * timeouts (render-user-wait.h); it is also the minimum of a TYPED_V3 job's per-phase bound.
 */
#define G5P_RU_DEF_WAIT_MS G5P_RU_V3_WAIT_MS
#define G5P_RU_DEF_PROFILE1_VDM_CONTROL 0x280

static unsigned int render_user_wait_ms = 20000;
module_param(render_user_wait_ms, uint, 0600);
MODULE_PARM_DESC(
	render_user_wait_ms,
	"Latched per-phase TA/3D bound, 3000..30000 ms (default 20000; TYPED_V3 needs "
	"20000..30000, absolute job cap 30000)");
/* latency101 (default off): host-side TYPED_V3 wait policy; nothing the firmware sees depends on it. */
static unsigned int render_user_hard_ms = 60000;
static unsigned int render_user_soft_ms = 45000;
static unsigned int render_user_obs_ms = 2000;
module_param(render_user_hard_ms, uint, 0600);
module_param(render_user_soft_ms, uint, 0600);
module_param(render_user_obs_ms, uint, 0600);
MODULE_PARM_DESC(
	render_user_hard_ms,
	"Latched TYPED_V3 hard per-phase bound soft..60000 ms, the job bound is twice "
	"it; 0 = off, render_user_wait_ms and the 30000 job cap apply (default 0)");
MODULE_PARM_DESC(
	render_user_soft_ms,
	"Latched TYPED_V3 soft per-phase bound 20000..hard ms, 0 = 45000; needs "
	"render_user_hard_ms");
MODULE_PARM_DESC(
	render_user_obs_ms,
	"Latched RAM-only observation window after a TYPED_V3 timeout, 0..5000 ms "
	"(default 2000; needs render_user_hard_ms)");
/* RC4a (default off): RAM-only touch A/B of the TYPED_V3 wait (render-user-touch.h). */

/* repeat109: the TYPED_V3 3D completion without the idle words (render-user-retire.h). */

/* wake110 (default off): Apple's submit kick at the TYPED_V3 publish (render-user-wake.h). */
/* release8 default */

/* TEMPORARY (multi-client stage 1): the fallback for a firmware that never drains a DC3
 * published after a job retired.  Remove once the device has settled that question.
 */

/* Stage 2 fault isolation (render-user-loss.h): how long the scheduler holds every other job
 * while it waits, RAM only, for a lost job's stuck phase.  Read at the loss.
 */

/* Stage 2 in-boot recovery (core recover.inc): a job that stopped the firmware loses only its
 * context, and the GPU is power-cycled and its firmware booted again while the other contexts'
 * jobs are held (each leaves the old epoch's firmware pages allocated); 0 = such a job ends the
 * epoch, as before.  How many: render_user_recover_streak.  Off by default; the release tuple turns
 * it on (faultiso F1-F4, 2026-11-02: two recoveries in one boot on hardware).
 */

/* render-user-recover.h: Apple restarts the GPU on every recovery packet.  Nonzero: recover every
 * time, but end the epoch after this many recoveries in a row with no job completing between
 * them.  0 = render_user_recover counts recoveries per boot, as before (a boot's second fault then
 * ends the epoch, however long the GPU ran in between).
 */

/* Fault-isolation test (host side only: the firmware runs the job as any other).  A context
 * created while this is N gives up on its N-th job's TA after 1 ms instead of its bound, so the
 * host loses that job the way it loses a stuck one, while the firmware completes it late: the
 * context is lost and the channel drained behind it.  0 = off (default).
 */

/* Fault-isolation test with a real GPU fault (faultiso).  A context created by a CAP_SYS_RAWIO
 * process while this is N reserves one page of its VM that is never mapped
 * (G5P_RU_TEST_FAULT_VA), and its N-th job's TA fetches its control stream from that page
 * instead of the validated stream: the TA reads an unmapped VA of the context's own VM.  What
 * the firmware does then (a fault event, a silent stream, or a stuck engine) is what the test
 * records; each ends that job and loses its context.  0 = off (default).
 */

/* render_user_test_fault_job's page: the last GiB below the hardware VA limit, far from every
 * kernel-private range (low sink, aux, PDS window, firmware window).
 */
#define G5P_RU_TEST_FAULT_VA (G5P_RU_HW_VA_LIMIT - 0x40000000ULL)

/* Stage 3 (render-user-window.h): which firmware memory a render VM maps.  Latched per context
 * at creation.  The kill switch 0 restores the stage 1 mapping and then needs CAP_SYS_RAWIO.
 */

/* The latency101 knobs are valid when all are 0 (off), or hard is in [soft, 60000], soft (0 =
 * min(45000, hard)) in [20000, hard], obs <= 5000 and the dialect is TYPED_V3.  The RC4a touch
 * is 0, or 1..20000 below the effective soft bound with latency101 on.  The 3D idle settle is
 * 0..5000.  The wake110 mode is 0, or 1..2 with latency101 on and the touch off (one A/B per
 * context).  Anything else refuses the context.
 */
static bool g5p_ru_lat_params_valid(void)
{
	u32 hard = READ_ONCE(render_user_hard_ms),
	    soft = READ_ONCE(render_user_soft_ms);

	return g5p_ru_lat_valid(soft, hard, READ_ONCE(render_user_obs_ms)) &&
	       g5p_ru_touch_valid((0), g5p_ru_lat_soft(soft, hard), hard) &&
	       g5p_ru_rt_settle_valid((G5P_RU_RT_DEF_SETTLE_MS)) &&
	       g5p_wk_mode_valid((2)) && (!(2) || (hard && !(0))) &&
	       (!hard || (1));
}
/* r25 frame-capable selections, same values as the fixed render_*_variant. */

#define G5P_RU_MAX_QUEUED 16
#define G5P_RU_MAX_TARGETS 32
#define G5P_RU_RESULTS 1024
#define G5P_RU_INPUT_MS 10000

struct g5p_rt {
	struct list_head link;
	struct g5p_context *ctx;
	struct g5p_ru_rt geometry;
	struct g5p_ru_rt_sizes sizes;
	struct drm_mm_node node;
	struct g5p_dma_page *pages;
	u32 width, height, page_count;
	u32 samples; /* 1 or 4 (CREATE_RENDER_TARGET); 0 = 1 */
	bool destroyed;
};

/* A pair job 1 bound to a kernel-owned catalogue (render_user_spare_pairs, TEMPORARY). */
struct g5p_rspare {
	u32 slot[2]; /* TA, 3D */
	struct g5p_dma_page root; /* the catalogue CBASE points at */
	bool ok, used;
};

/*
 * Device-wide render state (docs/gpu/multi-client-design.md §4.1): the one TA/3D channel pair
 * and the one PM record with its pool and aux pages, shared by every render context.  Jobs of
 * all contexts run one at a time on render_wq with the core lock held, and only they write
 * up, v3, sequence, pm_serial and the spares.  `lock` guards the pool and the aux pages, which
 * render-target creation grows while another context's job may run.  The scheduler's worker
 * is the only executor: it picks the jobs of all contexts round-robin.
 */
/* The kernel-built firmware window shared by every render VM (render-user-window.h): built
 * once from the firmware's tables, never edited, kept until reboot.
 */
struct g5p_rwin {
	bool built;
	u32 pages, missing, leaves;
	struct g5p_dma_page dir, leaf[G5P_RU_FW_PAGES];
	u32 leaf_di[G5P_RU_FW_PAGES];
};

struct g5p_rdev {
	struct mutex lock;
	bool up, v3; /* PM and channels built; by the TYPED_V3 attach */
	bool dc_pending; /* a bind's DC3 pair was not consumed in time ... */
	u32 dc_wait; /* ... and this is the consumer it waits for */
	u32 sequence, pm_serial;
	u32 cu_sequence, cu_target;	/* TYPED_COMPUTE: CDM sequence, CL producer target */
	u32 blocks;
	/* perf60, latched at the epoch's first TYPED_V3 attach: 2 = jobs alternate two record slots
	 * (render-user-layout.h), else 1; pm_once = the PM pool is built at that attach only.
	 */
	u32 job_slots;
	bool pm_once;
	/* perf60 stage 4, latched there too: render_user_pm_ctx (0 off, 1 per-context PM records,
	 * 2 also per-context PM IDs) and the boot-reserved sets in use (core lock).
	 */
	u32 pm_ctx, pm_set_next;
	bool pm_set_used[G5P_PMSET_MAX];
	/* the context of the last job the executor ran and the previous distinct one, the PM's
	 * write-back targets (render-user-pmset.h; compared, never dereferenced), and the closed
	 * contexts whose unbind waits until they are neither (render_user_close_defer)
	 */
	struct g5p_render *pm_last, *pm_prev, *close_deferred[2];
	struct g5p_dma_page *physical;
	struct g5p_dma_page aux[G5P_RU_AUX_PAGES];
	struct g5p_rspare *spares;
	u32 spare_count;
	/* render_user_close_park: the park catalogue a closed pair is bound to (allocated at the
	 * first park, kept for the device's life; rebuilt at every park)
	 */
	struct g5p_dma_page park;
	u32 parks;
	bool executing; /* a TYPED_V3 job is in g5p_rv3_execute() (reboot quiesce) */
	/* The scheduler (drm-render-sched.inc): its run list under sched_lock, and the one
	 * worker on render_wq that runs every job and close item.
	 */
	struct g5p_drm *drm;
	spinlock_t sched_lock;
	struct g5p_rs sched;
	struct delayed_work sched_work;
	/* boot_gpu_oc_mhz: the thermal stop's poll (system_wq, so a busy render_wq never starves it) */
	struct delayed_work oc_work;
	unsigned long oc_armed;
	u32 oc_ticks, oc_jobs;
	struct g5p_rwin
		window; /* written once, by the executor, core lock held */
	struct g5p_rstuck
		*stuck; /* a lost job blocks the channel (drain), worker only */
	/* A lost job stopped the firmware: the worker recovers before any pick.  Set by the
	 * executor, cleared by the worker; submissions and the claim pass meanwhile.
	 */
	bool recovering;
	u32 recoveries;
	u32 recover_streak; /* recoveries since a job completed (core lock) */
	struct list_head
		contexts; /* every render context (lock), for the recovery */
};

/* render-user-fault.h's ruling window on the device state: a failure noted while a TYPED_V3 job or
 * a drain runs, or a recovery is pending, waits for the executor.  RAM only.
 */
static bool g5p_rdev_ruling(struct g5p_rdev *rd)
{
	bool executing =
		smp_load_acquire(&rd->executing); /* before `recovering` */

	return g5p_ru_ruling(executing, READ_ONCE(rd->recovering));
}

/* A render context.  Its firmware state is the slot pair (slot[], bound), owned here the way
 * an Asahi queue owns its firmware context; the channels and the PM record are the device's.
 */
struct g5p_render {
	struct g5p_context *ctx;
	u32 slot[2]; /* engine0 TA, engine1 normal3D */
	u32 jobs; /* jobs that started on this context */
	u32 dc_target; /* DC ring consumer after this context's binding */
	struct drm_mm private_mm;
	struct drm_mm_node aux_node, low_node, firmware_node, private_node;
	struct drm_mm_node
		fault_node; /* render_user_test_fault_job's unmapped page */
	struct list_head targets;
	struct g5p_rt *profile1_rt;
	u32 target_count;
	u64 fence_context, fence_seqno;
	atomic_t queued;
	/* bound: the slot pair is published (for a RAM-only bind: consumed and checked).
	 * retained: the binding holds a context and device reference until its unbind.
	 * closing: the handle is gone; the close item (g5p_render_close_run()) is queued.
	 * live_maps: a job retired on the binding, so VM_BIND MAP may run beside queued jobs.
	 */
	bool claimant, bound, retained, aux_mapped, detached, stage_done,
		unhooked;
	bool closing, live_maps;
	bool lost_drained; /* a lost job of it drained: the firmware holds no use of it */
	struct g5p_rs_entity
		entity; /* its place in the scheduler (drm-render-sched.inc) */
	struct list_head rdev_link; /* in rdev->contexts */
	/* Mappings unmapped beside queued jobs, freed as those retire (g5p_render_reap()). */
	spinlock_t zlock;
	struct list_head zombies;
	/* Latched at creation; mirrors whatever qualifies the fixed path. */
	u32 stage, cache_variant, mmu_variant, timestamp_variant, wait_ms;
	u32 soft_ms, hard_ms, obs_ms; /* latency101; hard_ms == 0: off */
	u32 touch_ms; /* RC4a; 0: off (always 0 when hard_ms is) */
	u32 settle_ms; /* repeat109 minimum coherent run before the 3D event completes */
	u32 stall_job; /* render_user_test_stall_job; 0: off */
	u32 fault_job; /* render_user_test_fault_job; 0: off */
	u32 wake, wake_seed; /* wake110 (render-user-wake.h); wake 0: off */
	u32 wake_rang; /* wake110: 0xb00 writes of this context */
	u32 pm_policy, vdm_control;
	u32 fw_window; /* render_user_fw_window, G5P_RU_FW_* */
	bool after_ta, cache_repeat;
	struct g5p_r25_variants v; /* pm, memory, rt<=1, slots; others 0 */
	u32 target[2], status[2], selector[2];
	bool submitted[2], consumed[2], retired[2];
	/* TYPED_V3 (drm-render-v3.inc): serial jobs, the context is retained from its binding on. */
	bool v3_accepted;
	/* perf60 render_user_pm_switch 2: this context's own PM state (S + aux) after its last job */
	u8 *pm_save;

	/* perf60 stage 4 (render_user_pm_ctx): its own PM set (-1: the device PM), pool pages,
	 * aux pages mapped at G5P_RU_PRIVATE_VA in its VM, and the epoch they were built for.
	 */
	int pm_set;
	u32 pm_blocks, pm_epoch;
	struct g5p_dma_page *pm_pool;
	struct g5p_dma_page pm_aux[G5P_RU_AUX_PAGES];
	bool pm_aux_mapped;
	bool pm_first; /* it brought the epoch up (attach): the device aux is its own */
	bool close_defer_run; /* render_user_close_defer: its deferred close is running */
};

/* The lost job the channel waits behind (render-user-loss.h): its context (a reference held),
 * its per-job state as the executor left it, and the drain bound read at the loss.
 */
struct g5p_rstuck {
	struct g5p_render *r;
	struct g5p_rv3 v;
	u32 seq, engine, drain_ms;
	u64 lost_ns;
};

/* A listed BO that needs implicit fences and/or kernel cache maintenance. */
struct g5p_robj {
	struct drm_gem_object *obj; /* reference held by the job */
	u32 access;
	/* ACCESS_CACHE window (render_user_sync_range bit 1: the resource's range; else whole BO) */
	u64 lo, hi;
};

struct g5p_rjob {
	struct g5p_rs_job node; /* in its context's FIFO until picked */
	struct g5p_drm *gpu;
	struct g5p_context *ctx;
	struct g5p_fence *fence;
	struct dma_fence *inputs[G5P_MAX_SYNCS - 1];
	u32 input_count;
	struct g5p_robj *objs;
	u32 obj_count;
	struct dma_fence **deps; /* implicit dependencies from dma_resv */
	u32 dep_count;
	int setup_error; /* post-acceptance setup failure: fail without hardware */
	/* Inputs then deps, in order: dep_next is the first not yet seen signalled, cb_fence the
	 * one whose callback is armed (scheduler lock), dep_error a failed input or the deadline.
	 */
	u32 dep_next;
	struct dma_fence *cb_fence;
	struct dma_fence_cb cb;
	bool cb_fired;
	int dep_error;
	u64 id;
	unsigned long deadline;
	u8 ta_w[G5P_RU_TA_W_BYTES], td_w[G5P_RU_3D_W_BYTES];
	bool v3;
	bool compute;			/* TYPED_COMPUTE (drm-compute-v1.inc): a CDM job */
	struct g5p_cu_regs cu;
	struct g5p_rv3 rv3;
	u64 submit_ns; /* latency123: the submit ioctl */
};

static u8 *g5p_ru_page(struct g5p_boot *b, u32 engine)
{
	return b->shared[(engine ? G5P_R22_3D : G5P_R22_TA) / G5P_GPU_PAGE].cpu;
}

static u32 g5p_ru_meta(u32 engine)
{
	return g5p_meta_address(G5P_SHARED_VA +
				(engine ? G5P_R22_3D : G5P_R22_TA));
}

static u8 *g5p_ru_pm_cpu(struct g5p_boot *b)
{
	return b->shared[G5P_R22_PM / G5P_GPU_PAGE].cpu;
}

static u8 *g5p_ru_global(struct g5p_boot *b, u32 engine)
{
	u32 off = b->layout.offset[0x47 + engine];

	return b->root[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE;
}

static void g5p_ru_free_page(struct device *dev, struct g5p_dma_page *p)
{
	if (p->cpu)
		dma_free_coherent(dev, G5P_GPU_PAGE, p->cpu, p->dma);
	p->cpu = NULL;
}

/* Probe: the device render state and its scheduler.  Its pages are allocated on demand and
 * kept until reboot.
 */
static void g5p_rsched_work(struct work_struct *work);
static void g5p_oc_work(struct work_struct *work);

static int g5p_render_init(struct g5p_drm *d)
{
	d->rdev = drmm_kzalloc(&d->drm, sizeof(*d->rdev), GFP_KERNEL);
	if (!d->rdev)
		return -ENOMEM;
	d->rdev->drm = d;
	INIT_LIST_HEAD(&d->rdev->contexts);
	spin_lock_init(&d->rdev->sched_lock);
	g5p_rs_init(&d->rdev->sched);
	INIT_DELAYED_WORK(&d->rdev->sched_work, g5p_rsched_work);
	INIT_DELAYED_WORK(&d->rdev->oc_work, g5p_oc_work);
	return drmm_mutex_init(&d->drm, &d->rdev->lock);
}

/* rdev->lock held: the aux pages (once) and a PM pool of at least `blocks`.  The pool never
 * shrinks; every page added reaches PoC like the bring-up pool, whether or not the device's
 * bring-up has cleaned the pool yet.  The pool, list and F are rebuilt from it before every
 * job, so growth takes effect at the next job, and the job in flight keeps its pages.
 */
static int g5p_rdev_grow(struct g5p_drm *d, u32 blocks)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_dma_page *physical;
	u32 have = rd->blocks * 8, i;
	int ret = 0;

	for (i = 0; !ret && i < G5P_RU_AUX_PAGES; i++)
		if (!rd->aux[i].cpu)
			ret = g5p_vm_alloc_page(d->drm.dev, &rd->aux[i]);
	if (ret || blocks <= rd->blocks)
		return ret;
	if (blocks > G5P_RU_MAX_BLOCKS)
		return -EINVAL;
	physical = kvcalloc(blocks * 8, sizeof(*physical), GFP_KERNEL);
	if (!physical)
		return -ENOMEM;
	if (have)
		memcpy(physical, rd->physical, have * sizeof(*physical));
	for (i = have; !ret && i < blocks * 8; i++) {
		ret = g5p_vm_alloc_page(d->drm.dev, &physical[i]);
		if (!ret)
			g5p_table_clean_page(d->core, &physical[i]);
	}
	if (ret) {
		while (i-- > have)
			g5p_ru_free_page(d->drm.dev, &physical[i]);
		kvfree(physical);
		return ret;
	}
	kvfree(rd->physical);
	rd->physical = physical;
	rd->blocks = blocks;
	return 0;
}

/* rdev->lock held: the pool's page addresses, for the PM builders. */
static u64 *g5p_rdev_addresses(struct g5p_rdev *rd)
{
	u64 *physical =
		kvmalloc_array(rd->blocks * 8, sizeof(*physical), GFP_KERNEL);

	for (u32 i = 0; physical && i < rd->blocks * 8; i++)
		physical[i] = rd->physical[i].dma;
	return physical;
}

/* From the moment the firmware may hold a context's slot pair, the context and everything it
 * owns stay alive whatever its files do, until its close item has unbound the pair
 * (g5p_render_close_run()); a pair that cannot be unbound keeps them until reboot.
 */
static void g5p_render_retain(struct g5p_render *r)
{
	if (r->retained)
		return;
	kref_get(&r->ctx->ref);
	drm_dev_get(&r->ctx->gpu->drm);
	r->retained = true;
}

/* The RAM words of the binding of `slot` through the DC records at start and start + 1. */
static void g5p_render_bind_mem(struct g5p_boot *b, const u32 *slot, u32 start,
				struct g5p_ru_bind_mem *m)
{
	for (u32 i = 0; i < 2; i++)
		for (u32 j = 0; j < 5; j++) {
			u32 at = b->layout.offset[0x6d] +
				 ((start + i) & 255) * 20 + j * 4;

			m->rec[i][j] = b->shared[at / G5P_GPU_PAGE].cpu +
				       at % G5P_GPU_PAGE;
		}
	for (u32 e = 0; e < 2; e++) {
		m->pa[e] = g5p_boot_shared(b, 0x86) + slot[e] * 8;
		m->host[e] = g5p_boot_shared(b, 0x84) + slot[e] * 4;
		m->cached[e] = g5p_boot_shared(b, 0x85) + slot[e] * 4;
	}
}

/*
 * Called with the file lock.  Idle: no accepted job is queued or running.  A VM_BIND (`live`)
 * of a context with live_maps may also run beside its queued jobs, the way an Asahi VM_BIND
 * does not wait for the queue: a compositor imports and releases client buffers while its
 * previous frame is in flight.  Its tag reaches the firmware at the next kick of the slots.  A
 * MAP only adds entries for VAs no accepted job references; an UNMAP clears its entries now but
 * keeps the object pinned until the queued jobs have retired (g5p_resident_defer()), so a job
 * that still used the range faults on the hardware instead of reaching freed memory.  Not
 * before a job has retired on the binding: the epoch's first job checks that the tables did not
 * change since its attach, and a bind in flight computes its tag from them.  Render-target
 * edits still need the context idle.
 */
static int g5p_render_vm_idle(struct g5p_context *c, bool live)
{
	struct g5p_render *r = c->render;
	struct g5p_boot *b = c->gpu->core->boot;
	/* During a recovery no epoch may receive a tag. */
	bool busy = (atomic_read(&r->queued) &&
		     !(live && READ_ONCE(r->live_maps))) ||
		    READ_ONCE(c->gpu->rdev->recovering);

	if (c->bad || r->detached || busy) {
		dev_warn_ratelimited(
			c->gpu->drm.dev,
			"G5P VM_BIND busy=render-idle bad=%u detached=%u queued=%d bound=%u\n",
			c->bad, r->detached, atomic_read(&r->queued), r->bound);
		return -EBUSY;
	}
	if (r->bound && (!b || READ_ONCE(b->failed) || READ_ONCE(b->stopped)))
		return g5p_rdev_ruling(c->gpu->rdev) ? -EBUSY : -EIO;
	return 0;
}

/* perf60 MMU invalidation arm, default 0 (render-user-mmu.h): bit 0 every edit publication asks
 * the firmware for a PC-level invalidate, bit 1 a bound context's job after another context's
 * job retags its slots (PC), bit 2 NC-reads every table line before the tag store, bit 3 retags
 * on every job.  RAM only, like the tag path itself.
 */

/* An NC read of every 64-byte line of the VM's tables (g5p_rv3_fw_touch()'s visibility read). */
static void g5p_render_table_touch(struct g5p_vm *vm)
{
	struct g5p_pt *pt;

	for (u32 at = 0; at < G5P_GPU_PAGE; at += 64)
		(void)READ_ONCE(*(const u32 *)(vm->root.cpu + at));
	list_for_each_entry(pt, &vm->tables, link)
		for (u32 at = 0; at < G5P_GPU_PAGE; at += 64)
			(void)READ_ONCE(*(const u32 *)(pt->page.cpu + at));
}

/* File lock held, context idle (or a MAP with live_maps).  After a PTE edit on a bound render
 * VM: tables to PoC, then the packed tag to both of its slots.  RAM only: no register access
 * is safe after retirement, and the firmware reads these words only at this context's next
 * kick, which then invalidates the slots' PC level.
 */
static int g5p_render_invalidate(struct g5p_context *c)
{
	struct g5p_render *r = c->render;
	struct g5p_diag *g = c->gpu->core;
	struct g5p_boot *b = g->boot;
	struct g5p_vm *vm = c->vm;
	struct g5p_pt *pt;
	u32 tag, cached[2], bumps = 0, mode = (0);

	if (!READ_ONCE(r->bound) || r->detached)
		return 0;
	if (!b || c->bad || b->failed || b->stopped)
		return -EIO;
	g5p_table_clean_page(g, &vm->root);
	list_for_each_entry(pt, &vm->tables, link)
		g5p_table_clean_page(g, &pt->page);
	dsb(sy);
	if (mode & G5P_RU_MMU_TOUCH)
		g5p_render_table_touch(vm);
	tag = g5p_vm_generation(&vm->mutation);
	for (u32 e = 0; e < 2; e++)
		cached[e] = g5p_le32(g5p_boot_shared(b, 0x85) + r->slot[e] * 4);
	while (tag == cached[0] || tag == cached[1]) {
		vm->mutation.generation[2]++;
		tag = g5p_vm_generation(&vm->mutation);
		bumps++;
	}
	if (bumps)
		g5p_mmu_record(
			g,
			"RENDER-USER generation collision cached=%08x/%08x bumps=%u tag=%08x\n",
			cached[0], cached[1], bumps, tag);
	if (mode & G5P_RU_MMU_EDIT) {
		/* Advance the VM's own PC counter to the PC field the helper picked. */
		u32 pc = g5p_ru_mmu_pc_tag(tag, cached[0], cached[1]);

		if (pc) {
			vm->mutation.generation[0] +=
				((pc >> 22) - (tag >> 22)) & 0x3ff;
			tag = g5p_vm_generation(&vm->mutation);
		}
	}
	for (u32 e = 0; e < 2; e++)
		g5p_put32(g5p_boot_shared(b, 0x84) + r->slot[e] * 4, tag);
	dma_wmb();
	if (mode)
		g5p_mmu_record(
			g,
			"RENDER-USER tag=%08x slots=%u/%u invalidate=firmware-on-kick mmu_inval=%x "
			"cached=%08x/%08x\n",
			tag, r->slot[0], r->slot[1], mode, cached[0],
			cached[1]);
	else
		g5p_mmu_record(
			g,
			"RENDER-USER tag=%08x slots=%u/%u invalidate=firmware-on-kick\n",
			tag, r->slot[0], r->slot[1]);
	return 0;
}

/* DC3 consumption precedes this CBASE check and selected-slot invalidation;
 * the r22 diagnostic sequence with the user catalogue instead of a private PC.
 */

/* CPU-written firmware objects of one job: TA/3D pages, PM control and the
 * list/free pages. Coherent allocations plus explicit PoC, as r22 does.
 */
static void g5p_render_clean_job(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;

	for (u32 off = G5P_R22_TA; off <= G5P_R22_PM; off += G5P_GPU_PAGE)
		g5p_table_clean_page(g, &b->shared[off / G5P_GPU_PAGE]);
	for (u32 i = 0; i < 2; i++)
		g5p_table_clean_page(g, &b->render_meta[i]);
	dsb(sy);
	dma_wmb();
}

/* RAM only: the DC ring is idle and neither slot of this context has a PA or a tag. */
static int g5p_render_vm_ready(struct g5p_boot *b, struct g5p_render *r)
{
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	struct g5p_ru_bind_sample s;
	struct g5p_ru_bind_mem m;
	u32 start = READ_ONCE(ctl[4]);

	g5p_render_bind_mem(b, r->slot, start, &m);
	g5p_ru_bind_read(&m, READ_ONCE(ctl[0]), start, &s);
	return g5p_ru_bind_busy(&s) ? -EBUSY : 0;
}

/* The epoch's first job, core lock held, device not up: the PM record from the device pool
 * and both channels, and every page they reach cleaned to PoC.  Doing this again with the
 * device up would reset the channels and the PM state that every context shares.
 */
static int g5p_render_device_prepare(struct g5p_drm *d,
				     const struct g5p_r25_variants *v)
{
	struct g5p_rdev *rd = d->rdev;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u64 *physical;
	int ret;

	mutex_lock(&rd->lock);
	physical = g5p_rdev_addresses(rd);
	ret = physical ? 0 : -ENOMEM;
	if (!ret)
		ret = g5p_ru_pm(g5p_ru_pm_cpu(b), b->render_meta[0].cpu,
				b->render_meta[1].cpu,
				G5P_SHARED_VA + G5P_SHARED_SIZE,
				G5P_SHARED_VA + G5P_SHARED_SIZE + G5P_GPU_PAGE,
				physical, rd->blocks, G5P_RU_PRIVATE_VA);
	if (!ret)
		g5p_ru_pm_variant(g5p_ru_pm_cpu(b), b->render_meta[1].cpu,
				  physical, rd->blocks, v->pm);
	kvfree(physical);
	for (u32 i = 0; !ret && i < G5P_RU_AUX_PAGES; i++)
		g5p_table_clean_page(g, &rd->aux[i]);
	for (u32 i = 0; !ret && i < rd->blocks * 8; i++)
		g5p_table_clean_page(g, &rd->physical[i]);
	mutex_unlock(&rd->lock);
	if (ret)
		return ret;
	rd->pm_serial = 0;
	for (u32 e = 0; e < 2; e++)
		g5p_ru_channel_init(g5p_ru_page(b, e), g5p_ru_meta(e));
	g5p_cu_device_init(d);
	g5p_render_clean_job(g);
	return 0;
}

/*
 * Core lock held, firmware up: the root entry of the firmware GiB for a VM in mode `fw_window`.
 * The window is built at its first use from the firmware's leaf entries of the pages it keeps
 * (render-user-window.h), and then shared, unchanged, by every render VM.  RAM only.
 */
static int g5p_render_fw_entry(struct g5p_drm *d, struct g5p_render *r,
			       u32 *entry)
{
	struct g5p_rwin *w = &d->rdev->window;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u8 *leaves[ARRAY_SIZE(b->leaf)];
	int ret = 0;

	*entry = 0;
	if (r->fw_window == G5P_RU_FW_FULL)
		*entry = g5p_le32(b->catalogue.cpu + G5P_RU_FIRMWARE_CI * 4);
	if (r->fw_window != G5P_RU_FW_WINDOW)
		return 0;
	if (w->built) {
		*entry = g5p_catalogue_entry(w->dir.dma);
		return 0;
	}
	for (u32 i = 0; i < b->leaves; i++)
		leaves[i] = b->leaf[i].cpu;
	ret = g5p_vm_alloc_page(d->drm.dev, &w->dir);
	/* perf60 stage 4: the per-context PM sets reserved at boot, after the device PM's pages */
	for (u32 i = 0;
	     !ret && i < G5P_RU_FW_PAGES + b->pm_sets * G5P_PMSET_PAGES; i++) {
		u32 j = i - G5P_RU_FW_PAGES;
		u64 va = i < G5P_RU_FW_PAGES ?
				 g5p_ru_fw_window_va[i] :
				 g5p_pmset_va(j / G5P_PMSET_PAGES,
					      j % G5P_PMSET_PAGES);
		u64 leaf = g5p_ru_fw_leaf(leaves, b->leaf_index, b->leaves, va);
		u32 k = g5p_ru_fw_slot(w->leaf_di, w->leaves, va);

		if (!(leaf & 1)) {
			w->missing++;
			continue;
		}
		if (k == w->leaves) {
			ret = g5p_vm_alloc_page(d->drm.dev, &w->leaf[k]);
			if (ret)
				break;
			w->leaf_di[k] = g5p_ru_fw_di(va);
			g5p_put64(w->dir.cpu + w->leaf_di[k] * 8,
				  g5p_directory_entry(w->leaf[k].dma));
			w->leaves++;
		}
		g5p_put64(w->leaf[k].cpu + g5p_ru_fw_ti(va) * 8, leaf);
		w->pages++;
	}
	if (ret) {
		for (u32 i = 0; i < w->leaves; i++)
			g5p_ru_free_page(d->drm.dev, &w->leaf[i]);
		g5p_ru_free_page(d->drm.dev, &w->dir);
		memset(w, 0, sizeof(*w));
		return ret;
	}
	for (u32 i = 0; i < w->leaves; i++)
		g5p_table_clean_page(g, &w->leaf[i]);
	g5p_table_clean_page(g, &w->dir);
	w->built = true;
	*entry = g5p_catalogue_entry(w->dir.dma);
	dev_info(
		g->dev,
		"RENDER-V3 fw-window pages=%u missing=%u leaves=%u dir=%pad end=v3\n",
		w->pages, w->missing, w->leaves, &w->dir.dma);
	return 0;
}

/* Core lock held, between jobs: the DC3 pair that points this context's slots at its own page
 * catalogue.  Maps the firmware window (g5p_render_fw_entry()) and borrows the sink catalogue
 * entry, cleans the tables and this
 * context's render targets, then writes the PA and tag of both slots and the two records.
 * RAM only, nothing is written when the ring is busy or a slot is in use; the caller stores
 * the producer.
 */
static int g5p_render_vm_prepare(struct g5p_drm *d, struct g5p_render *r,
				 u32 *start_out, u32 *tag_out)
{
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_vm *vm = r->ctx->vm;
	u32 *ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	u32 start = READ_ONCE(ctl[4]), tag, entry;
	struct g5p_ru_bind_mem m;
	struct g5p_pt *pt;
	struct g5p_rt *rt;
	int ret;

	ret = g5p_render_vm_ready(b, r);
	if (!ret)
		ret = g5p_render_fw_entry(d, r, &entry);
	if (ret)
		return ret;
	g5p_vm_entry(&vm->mutation, vm->root.cpu + G5P_RU_FIRMWARE_CI * 4,
		     entry, 0);
	if (b->sink_ready)
		g5p_vm_entry(&vm->mutation, vm->root.cpu,
			     g5p_le32(b->catalogue.cpu), 0);
	g5p_table_clean_page(g, &vm->root);
	list_for_each_entry(pt, &vm->tables, link)
		g5p_table_clean_page(g, &pt->page);
	list_for_each_entry(rt, &r->targets, link)
		for (u32 i = 0; i < rt->page_count; i++)
			g5p_table_clean_page(g, &rt->pages[i]);
	dsb(sy);
	tag = g5p_vm_generation(&vm->mutation);
	/* A zero tag would read as unbound: the unbind check takes a zero cached tag as proof
	 * that the DC3 handler ran.
	 */
	while (!tag && vm->mutation.generations) {
		vm->mutation.generation[2]++;
		tag = g5p_vm_generation(&vm->mutation);
	}
	g5p_render_bind_mem(b, r->slot, start, &m);
	g5p_ru_bind_publish(&m, r->slot, vm->root.dma, tag);
	*start_out = start;
	*tag_out = tag;
	return 0;
}

/* The epoch's first job, core lock held: build the device, then bind this context's pair.
 * Any failure leaves every allocation and binding retained.
 */
static int g5p_render_attach_prepare(struct g5p_drm *d, struct g5p_render *r,
				     u32 *start_out, u32 *tag_out)
{
	int ret = g5p_render_vm_ready(d->core->boot, r);

	if (!ret)
		ret = g5p_render_device_prepare(d, &r->v);
	return ret ?: g5p_render_vm_prepare(d, r, start_out, tag_out);
}

/* The legacy profiles' attach: the epoch's first job, with a doorbell and a host MMU pass. */

/* Pointer contents, channel producer, global producer and kick, each after a
 * barrier. The channel must have retired the previous job's endpoint.
 */

/* Same predicates as g5p_r22_observe() for sequence `seq`. */

#include "drm-render-v3.inc"
#include "drm-render-sched.inc"
#include "drm-render-unbind.inc"

static void g5p_render_job_free(struct g5p_rjob *job)
{
	for (u32 i = 0; i < job->input_count; i++)
		dma_fence_put(job->inputs[i]);
	for (u32 i = 0; i < job->dep_count; i++)
		dma_fence_put(job->deps[i]);
	kfree(job->deps);
	for (u32 i = 0; i < job->obj_count; i++)
		drm_gem_object_put(job->objs[i].obj);
	kvfree(job->objs);
	if (job->fence)
		dma_fence_put(&job->fence->base);
	if (job->ctx)
		g5p_context_put(job->ctx);
	kfree(job);
}

/* CPU cache maintenance on the G5P mapping (for an imported BO, the attachment's sg_table): clean
 * before the job, invalidate after retirement.  Whole BO, or (render_user_sync_range bit 1, latched
 * at submit) the resource's BO range, the same window both ways.
 */
static void g5p_render_cache(struct g5p_drm *d, struct g5p_rjob *job,
			     bool to_cpu)
{
	for (u32 i = 0; i < job->obj_count; i++) {
		struct g5p_robj *o = &job->objs[i];
		struct sg_table *sgt = to_drm_gem_shmem_obj(o->obj)->sgt;

		if (!(o->access & DRM_G5P_RENDER_ACCESS_CACHE) || !sgt)
			continue;
		/* A buffer the job only reads (a sampled client dma-buf or wl_shm udmabuf)
		 * was cleaned before the job and the GPU wrote none of it: no invalidate,
		 * which could discard what the CPU side wrote meanwhile into lines of the
		 * pages the buffer shares (a udmabuf covers whole 16 KiB pages).
		 */
		if (to_cpu && !(o->access & DRM_G5P_RENDER_ACCESS_WRITE))
			continue;
		if (o->hi && (o->lo || o->hi != o->obj->size))
			g5p_sync_sgt_range(d->drm.dev, sgt, o->lo, o->hi,
					   to_cpu);
		else if (to_cpu)
			dma_sync_sgtable_for_cpu(d->drm.dev, sgt,
						 DMA_BIDIRECTIONAL);
		else
			dma_sync_sgtable_for_device(d->drm.dev, sgt,
						    DMA_BIDIRECTIONAL);
	}
}

/* After acceptance, render_submit_lock held, no file lock: one dma_resv at a
 * time, so no ww ordering is needed. Our own context's earlier fences are
 * already ordered by the queue. A failure fails the job before hardware.
 */
static int g5p_render_implicit(struct g5p_rjob *job, u64 own_context)
{
	for (u32 i = 0; i < job->obj_count; i++) {
		struct dma_resv *resv = job->objs[i].obj->resv;
		bool write = job->objs[i].access & DRM_G5P_RENDER_ACCESS_WRITE;
		struct dma_fence **fences = NULL, **grown;
		unsigned int count = 0;
		int ret;

		if (!(job->objs[i].access & DRM_G5P_RENDER_ACCESS_IMPLICIT))
			continue;
		dma_resv_lock(resv, NULL);
		ret = dma_resv_get_fences(resv,
					  write ? DMA_RESV_USAGE_READ :
						  DMA_RESV_USAGE_WRITE,
					  &count, &fences);
		if (!ret)
			ret = dma_resv_reserve_fences(resv, 1);
		if (!ret)
			dma_resv_add_fence(resv, &job->fence->base,
					   write ? DMA_RESV_USAGE_WRITE :
						   DMA_RESV_USAGE_READ);
		dma_resv_unlock(resv);
		grown = ret || !count ?
				NULL :
				krealloc_array(job->deps,
					       job->dep_count + count,
					       sizeof(*grown), GFP_KERNEL);
		if (!ret && count && !grown)
			ret = -ENOMEM;
		if (grown)
			job->deps = grown;
		for (u32 j = 0; j < count; j++) {
			if (!ret && fences[j]->context != own_context)
				job->deps[job->dep_count++] = fences[j];
			else
				dma_fence_put(fences[j]);
		}
		kfree(fences);
		if (ret)
			return ret;
	}
	return 0;
}

/* Free the mappings unmapped beside jobs up to fence seqno `seqno` (all: every one). */
static void g5p_render_reap(struct g5p_render *r, u64 seqno, bool all)
{
	struct g5p_mapping *m, *t;
	unsigned long flags;
	LIST_HEAD(done);

	spin_lock_irqsave(&r->zlock, flags);
	list_for_each_entry_safe(m, t, &r->zombies, link)
		if (all || m->retire <= seqno)
			list_move_tail(&m->link, &done);
	spin_unlock_irqrestore(&r->zlock, flags);
	list_for_each_entry_safe(m, t, &done, link) {
		list_del(&m->link);
		g5p_vm_obj_put(r->ctx->vm, m->obj);
		drm_gem_object_put(m->obj);
		kfree(m);
	}
}

/* One job, picked by the scheduler once its inputs signalled (or failed, or timed out). */
static void g5p_render_run(struct g5p_rjob *job)
{
	struct g5p_drm *d = job->gpu;
	struct g5p_diag *g = d->core;
	struct g5p_render *r = job->ctx->render;
	unsigned long lock_deadline, flags;
	bool hardware = false;
	int ret;

	/* Admission closes only for good (reboot quiesce, FINAL teardown): a job queued before
	 * that fails here, without waiting for its inputs and without competing for the command
	 * lock.
	 */
	if (g5p_r8_admit_closed(&g5p_r8_state.admit))
		ret = -ESHUTDOWN;
	else
		ret = job->setup_error ?: job->dep_error;
	if (!ret && (job->ctx->bad || READ_ONCE(g->boot->failed)))
		ret = -EIO;
	if (!ret) {
		/* Bounded: never queue forever behind a command stuck in MMIO. */
		lock_deadline = jiffies + 2 * HZ;
		while (!mutex_trylock(&g->lock)) {
			if (time_after(jiffies, lock_deadline)) {
				ret = -EBUSY;
				break;
			}
			msleep(5);
		}
	}
	if (!ret) {
		if (g->boot->failed || g->boot->stopped) {
			ret = -EIO;
		} else if (!1 && !g5p_ru_mmio_ok(&g5p_rv3_term)) {
			/* A V3 epoch is terminal: the legacy path would read and write registers. */
			ret = -EIO;
		} else {
			/* RAM-only from the DC3 attach on: no snapshot (it reads R0), no
			 * export, no register access of any kind after a timeout.
			 */
			g5p_render_cache(d, job, false);
			WRITE_ONCE(d->rdev->executing, true);
			ret = g5p_rv3_execute(d, r, job);
			/* after `recovering`, if the executor set it (g5p_rdev_ruling()) */
			smp_store_release(&d->rdev->executing, false);
			hardware = job->rv3.touched || job->rv3.bind_failed;
			if (!ret || ret == -ECANCELED)
				g5p_render_cache(
					d, job,
					true); /* before status and fence */
			if (!ret && r->bound)
				WRITE_ONCE(r->live_maps, true);
		}
		g->last_result = ret;
		mutex_unlock(&g->lock);
	}
	spin_lock_irqsave(&d->status_lock, flags);
	/* A failed device fails every later job; one context's lost job or failed bind does not. */
	if (READ_ONCE(g->boot->failed) || READ_ONCE(g->boot->event_failed))
		ret = ret ?: -EIO;
	job->fence->state = DRM_G5P_JOB_COMPLETE;
	job->fence->error = ret < 0 ? ret : 0;
	d->last_job = job->id;
	d->last_error = job->fence->error;
	d->job_error = job->fence->error;
	d->job_state = DRM_G5P_JOB_COMPLETE;
	if (ret < 0 && ret != -ECANCELED && hardware) {
		/* No reset protocol: quarantine the context and the device. A TYPED_V3
		 * timeout is one exception for the device: nothing is wrong with the
		 * protocol, the firmware may still complete, and the drain must keep
		 * decoding (RAM only) so that a late completion is recorded.  A failed
		 * bind is another: the DC ring is not on the path of the contexts
		 * already bound, so only this context is lost.  A lost job is the last:
		 * the executor already ruled (the drain, or the recovery that replaces
		 * this epoch), and only its context is lost.
		 */
		WRITE_ONCE(job->ctx->bad, true);
		if (g5p_ru_job_fails_device(1, ret, job->rv3.bind_failed,
					    job->rv3.lost))
			WRITE_ONCE(g->boot->failed, true);
	}
	spin_unlock_irqrestore(&d->status_lock, flags);
	if (ret < 0)
		dma_fence_set_error(&job->fence->base, ret);
	atomic_dec(&r->queued);
	atomic_dec(&d->r8_queued);
	dma_fence_signal(&job->fence->base);
	/* A job that may still run on the hardware (a bad context) keeps what it might use. */
	if (!READ_ONCE(job->ctx->bad))
		g5p_render_reap(r, job->fence->base.seqno, false);
	/* a failed job (complete=-N) always prints; a completed one is routine: job_log */
	if (ret)
		dev_info(
			g->dev,
			"RENDER-USER job=%llu seq=%u complete=%d slots=%u/%u\n",
			job->id, job->v3 ? job->rv3.seq : d->rdev->sequence,
			ret, r->slot[0], r->slot[1]);
	else
		g5p_job_printk(
			g->dev,
			"RENDER-USER job=%llu seq=%u complete=%d slots=%u/%u\n",
			job->id, job->v3 ? job->rv3.seq : d->rdev->sequence,
			ret, r->slot[0], r->slot[1]);
	g5p_render_job_free(job);
	drm_dev_put(&d->drm);
}

/* Private PM-aux/RT page with the latched r25 leaf flags (c by default). */
static int g5p_render_map_page(struct g5p_drm *d, struct g5p_render *r, u64 va,
			       struct g5p_dma_page *page)
{
	u8 *pte = g5p_vm_pte(r->ctx->vm, d->drm.dev, va, true);

	if (IS_ERR_OR_NULL(pte))
		return pte ? PTR_ERR(pte) : -ENOMEM;
	if (g5p_le32(pte) || g5p_le32(pte + 4))
		return -EBUSY;
	g5p_vm_entry(&r->ctx->vm->mutation, pte,
		     g5p_leaf_entry(page->dma, g5p_r25_attributes(&r->v, va)),
		     2);
	return 0;
}

/* clear: unmap first (file lock held, VM still hooked). */
static void g5p_rt_release_memory(struct g5p_drm *d, struct g5p_render *r,
				  struct g5p_rt *rt, bool clear)
{
	if (clear && rt->page_count)
		g5p_vm_clear(r->ctx->vm, d->drm.dev, rt->node.start,
			     rt->sizes.main + rt->sizes.secondary_map);
	for (u32 i = 0; i < rt->page_count; i++)
		g5p_ru_free_page(d->drm.dev, &rt->pages[i]);
	kvfree(rt->pages);
	rt->pages = NULL;
	rt->page_count = 0;
	if (drm_mm_node_allocated(&rt->node))
		drm_mm_remove_node(&rt->node);
}

/* File lock held, context idle; RAM only.  Main ring then secondary in one
 * RT_ALIGN node: the first 64x64 target lands exactly on r22's
 * 101020000/101030000.
 */
static int g5p_rt_create(struct g5p_drm *d, struct g5p_render *r,
			 const struct g5p_ru_rt *geometry, u32 width,
			 u32 height, struct g5p_rt **out)
{
	struct g5p_rt *rt;
	u64 bytes;
	int ret;

	if (r->target_count >= G5P_RU_MAX_TARGETS)
		return -ENOSPC;
	rt = kzalloc(sizeof(*rt), GFP_KERNEL);
	if (!rt)
		return -ENOMEM;
	rt->ctx = r->ctx;
	rt->geometry = *geometry;
	rt->width = width;
	rt->height = height;
	ret = g5p_ru_geometry_check(geometry, width, height, &rt->sizes);
	if (ret)
		goto err_free;
	bytes = rt->sizes.main + rt->sizes.secondary_map;
	ret = drm_mm_insert_node_in_range(
		&r->private_mm, &rt->node, bytes, G5P_RU_RT_ALIGN, 0,
		G5P_RU_PRIVATE_VA, G5P_RU_PRIVATE_VA + G5P_RU_PRIVATE_SIZE,
		DRM_MM_INSERT_LOW);
	if (ret)
		goto err_free;
	rt->pages =
		kvcalloc(bytes / G5P_GPU_PAGE, sizeof(*rt->pages), GFP_KERNEL);
	if (!rt->pages) {
		ret = -ENOMEM;
		goto err_node;
	}
	for (; rt->page_count < bytes / G5P_GPU_PAGE; rt->page_count++) {
		struct g5p_dma_page *p = &rt->pages[rt->page_count];

		ret = g5p_vm_alloc_page(d->drm.dev, p);
		if (!ret)
			ret = g5p_render_map_page(
				d, r,
				rt->node.start + rt->page_count * G5P_GPU_PAGE,
				p);
		if (ret) {
			g5p_ru_free_page(d->drm.dev, p);
			goto err_pages;
		}
	}
	if (r->bound) {
		for (u32 i = 0; i < rt->page_count; i++)
			g5p_table_clean_page(d->core, &rt->pages[i]);
		ret = g5p_render_invalidate(r->ctx);
		if (ret) {
			/* Hardware may hold the new tag: retain the target. */
			list_add_tail(&rt->link, &r->targets);
			r->target_count++;
			return ret;
		}
	}
	list_add_tail(&rt->link, &r->targets);
	r->target_count++;
	*out = rt;
	g5p_vmlog_add(G5P_VMLOG_RT_NEW, r->ctx->vm, 0, rt->node.start, bytes,
		      rt->width << 16 | rt->height, rt->samples);
	return 0;
err_pages:
	g5p_rt_release_memory(d, r, rt, true);
	goto err_free;
err_node:
	drm_mm_remove_node(&rt->node);
err_free:
	kfree(rt);
	return ret;
}

/* Detach every VM-side trace: private PTEs and the reserved VA nodes. File
 * lock held, or the file is closing (no concurrent VM ioctl can exist).
 */
static void g5p_render_unhook(struct g5p_context *c)
{
	struct g5p_render *r = c->render;
	struct device *dev = c->gpu->drm.dev;
	struct g5p_rt *rt;

	if (r->unhooked)
		return;
	list_for_each_entry(rt, &r->targets, link)
		if (rt->page_count)
			g5p_vm_clear(c->vm, dev, rt->node.start,
				     rt->sizes.main + rt->sizes.secondary_map);
	if (r->aux_mapped)
		g5p_vm_clear(c->vm, dev, G5P_RU_PRIVATE_VA,
			     G5P_RU_AUX_PAGES * G5P_GPU_PAGE);
	for (u32 i = 0; i < 4; i++) {
		struct drm_mm_node *n = i == 0 ? &r->low_node :
					i == 1 ? &r->firmware_node :
					i == 2 ? &r->private_node :
						 &r->fault_node;

		if (drm_mm_node_allocated(n))
			drm_mm_remove_node(n);
	}
	if (c->vm->resident == c)
		c->vm->resident = NULL;
	r->unhooked = true;
}

/* Last context reference: only reached unbound or after a successful DC3
 * unbind, so no firmware or GPU reference to this memory remains.  The aux
 * pages and the PM pool are the device's.
 */
static void g5p_render_release(struct g5p_context *c)
{
	struct g5p_render *r = c->render;
	struct g5p_drm *d = c->gpu;
	struct g5p_rt *rt, *tmp;
	unsigned long flags;

	g5p_render_unhook(c);
	g5p_render_reap(r, 0, true);
	if (!list_empty(&r->rdev_link)) {
		mutex_lock(&d->rdev->lock);
		list_del_init(&r->rdev_link);
		mutex_unlock(&d->rdev->lock);
	}
	list_for_each_entry_safe(rt, tmp, &r->targets, link) {
		g5p_rt_release_memory(d, r, rt, false);
		list_del(&rt->link);
		kfree(rt);
	}
	if (drm_mm_node_allocated(&r->aux_node))
		drm_mm_remove_node(&r->aux_node);
	drm_mm_takedown(&r->private_mm);
	spin_lock_irqsave(&d->core->slot_lock, flags);
	for (u32 e = 0; e < 2; e++)
		if (r->slot[e])
			__clear_bit(r->slot[e], d->core->user_slots);
	spin_unlock_irqrestore(&d->core->slot_lock, flags);
	{
		u32 aux = 0;

		for (u32 i = 0; i < G5P_RU_AUX_PAGES; i++)
			aux += !!r->pm_aux[i].cpu;
		/* release8 accounting: what this context frees, and the device's live GEM total */
		dev_info(
			d->drm.dev,
			"RENDER-V3 release slots=%u/%u jobs=%u set=%d pool_pages=%u aux_pages=%u "
			"vm_pinned=%lluK live_bos=%u live_bo_bytes=%lluK end=v3\n",
			r->slot[0], r->slot[1], r->jobs, r->pm_set,
			r->pm_pool ? r->pm_blocks * 8 : 0, aux,
			c->vm ? c->vm->pinned_bytes >> 10 : 0,
			READ_ONCE(g5p_bo_count), READ_ONCE(g5p_bo_bytes) >> 10);
	}
	g5p_rv3_pmctx_release(d, r);
	if (d->rdev->pm_last == r)
		d->rdev->pm_last = NULL;
	if (d->rdev->pm_prev == r)
		d->rdev->pm_prev = NULL;
	kvfree(r->pm_save);
	kfree(r);
	c->render = NULL;
}

/* CREATE_CONTEXT type RENDER. Exclusive VM: reserves [64KiB, 1GiB) for the
 * borrowed VA0 sink, the whole firmware GiB and the private RT/PM window.
 */
static int g5p_render_create_context(struct drm_device *drm,
				     struct drm_file *file,
				     struct drm_g5p_create_context *a)
{
	struct g5p_drm *d = to_g5p(drm);
	struct g5p_file *f = file->driver_priv;
	struct g5p_context *c;
	struct g5p_render *r;
	struct g5p_r25_variants v = {
		.pm = (0),
		.memory = (0),
		.rt = (0),
		.slots = (0),
	};
	unsigned long flags;
	u32 id, fw_window;
	int ret;

	BUILD_BUG_ON(G5P_USER_RESERVED_START & ((1ULL << 30) - 1));
	if (a->flags || a->handle)
		return -EINVAL;
	;
	;
	fw_window = (G5P_RU_FW_WINDOW);
	/* Unprivileged unless the VM would map the whole firmware GiB (the kill switch). */
	if (fw_window >= G5P_RU_FW_MODES ||
	    (fw_window == G5P_RU_FW_FULL && !capable(CAP_SYS_RAWIO)))
		return fw_window >= G5P_RU_FW_MODES ? -EINVAL : -EPERM;
	if ((0) > 2 ||
	    ((G5P_RU_DEF_CACHE_VARIANT) != 0 &&
	     (G5P_RU_DEF_CACHE_VARIANT) != 3) ||
	    (G5P_RU_DEF_MMU_VARIANT) > 1 || (0) > 1 || (0) > 1 ||
	    READ_ONCE(render_user_wait_ms) < ((1) ? G5P_RU_V3_WAIT_MS : 3000) ||
	    READ_ONCE(render_user_wait_ms) > G5P_RU_V3_CAP_MS ||
	    !g5p_ru_lat_params_valid() || !(32) || (32) > G5P_RU_MAX_BLOCKS ||
	    ((G5P_RU_DEF_PROFILE1_VDM_CONTROL) & 0xf) ||
	    (G5P_RU_DEF_PROFILE1_VDM_CONTROL) > G5P_RU_MAX_VDM ||
	    !g5p_ru_r25_valid(&v))
		return -EINVAL;
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!c || !r) {
		kfree(c);
		kfree(r);
		return -ENOMEM;
	}
	r->pm_set = -1;
	c->gpu = d;
	c->render = r;
	r->ctx = c;
	INIT_LIST_HEAD(&r->targets);
	INIT_LIST_HEAD(&r->rdev_link);
	g5p_rs_entity_init(&r->entity);
	spin_lock_init(&r->zlock);
	INIT_LIST_HEAD(&r->zombies);
	atomic_set(&r->queued, 0);
	drm_mm_init(&r->private_mm, G5P_RU_PRIVATE_VA, G5P_RU_PRIVATE_SIZE);
	r->stage = (0);
	r->cache_variant = (G5P_RU_DEF_CACHE_VARIANT);
	r->mmu_variant = (G5P_RU_DEF_MMU_VARIANT);
	r->timestamp_variant = (0);
	r->wait_ms = READ_ONCE(render_user_wait_ms);
	r->hard_ms = READ_ONCE(render_user_hard_ms);
	r->soft_ms = r->hard_ms ?
			     g5p_ru_lat_soft(READ_ONCE(render_user_soft_ms),
					     r->hard_ms) :
			     0;
	r->obs_ms = r->hard_ms ? READ_ONCE(render_user_obs_ms) : 0;
	r->touch_ms = r->hard_ms ? (0) : 0;
	r->settle_ms = (G5P_RU_RT_DEF_SETTLE_MS);
	r->stall_job = (0);
	r->fault_job = capable(CAP_SYS_RAWIO) ? (0) : 0;
	r->wake = r->hard_ms && !r->touch_ms ? (2) : 0;
	r->wake = g5p_wk_mode_valid(r->wake) ? r->wake : 0;
	r->wake_seed = (0);
	r->after_ta = (0);
	r->pm_policy = (0);
	r->cache_repeat = (0);
	r->vdm_control = (G5P_RU_DEF_PROFILE1_VDM_CONTROL);
	r->fw_window = fw_window;
	r->v = v;
	r->fence_context = dma_fence_context_alloc(1);
	mutex_lock(&f->lock);
	c->vm = xa_load(&f->vms, a->vm);
	if (!c->vm) {
		ret = -ENOENT;
		goto err_unlock;
	}
	if (c->vm->contexts || c->vm->resident || c->vm->resident_gone ||
	    c->vm->frozen || c->vm->serial_context) {
		ret = -EBUSY;
		goto err_unlock;
	}
	/* Memory mode 4 covers every render leaf, so no BO may be mapped yet. */
	if (v.memory == 4 && c->vm->map_count) {
		ret = -EINVAL;
		goto err_unlock;
	}
	r->low_node.start = G5P_USER_VA_START;
	r->low_node.size = G5P_RU_USER_VA_MIN - G5P_USER_VA_START;
	r->private_node.start = G5P_RU_PRIVATE_VA;
	r->private_node.size = G5P_RU_PRIVATE_SIZE;
	r->firmware_node.start = G5P_USER_RESERVED_END;
	r->firmware_node.size =
		((G5P_RU_FIRMWARE_CI + 1) << 30) - G5P_USER_RESERVED_END;
	if (drm_mm_reserve_node(&c->vm->mm, &r->low_node) ||
	    drm_mm_reserve_node(&c->vm->mm, &r->private_node) ||
	    drm_mm_reserve_node(&c->vm->mm, &r->firmware_node) ||
	    g5p_le32(c->vm->root.cpu + G5P_RU_FIRMWARE_CI * 4) ||
	    g5p_le32(c->vm->root.cpu)) {
		ret = -EINVAL; /* VM already maps a range this context needs */
		goto err_unlock;
	}
	if (r->fault_job) {
		r->fault_node.start = G5P_RU_TEST_FAULT_VA;
		r->fault_node.size = G5P_GPU_PAGE;
		if (drm_mm_reserve_node(&c->vm->mm, &r->fault_node)) {
			dev_warn(
				d->drm.dev,
				"RENDER-V3 test-fault off: va=%#llx already used in this VM end=v3\n",
				G5P_RU_TEST_FAULT_VA);
			r->fault_job = 0;
		}
	}
	spin_lock_irqsave(&d->core->slot_lock, flags);
	if (g5p_ru_slot_pair(d->core->user_slots, v.slots, r->slot)) {
		__set_bit(r->slot[0], d->core->user_slots);
		__set_bit(r->slot[1], d->core->user_slots);
	} else {
		r->slot[0] = 0;
		r->slot[1] = 0;
	}
	spin_unlock_irqrestore(&d->core->slot_lock, flags);
	if (!r->slot[0]) {
		ret = -ENOSPC;
		goto err_unlock;
	}
	/* The device's PM pool (at least this context's latched size) and aux pages, mapped at
	 * the same VA in every render VM, so the PM state each TA reloads is the one the
	 * previous job saved, whichever VM ran it.
	 */
	mutex_lock(&d->rdev->lock);
	ret = g5p_rdev_grow(d, (32));
	mutex_unlock(&d->rdev->lock);
	r->aux_node.start = G5P_RU_PRIVATE_VA;
	r->aux_node.size = G5P_RU_AUX_PAGES * G5P_GPU_PAGE;
	if (!ret)
		ret = drm_mm_reserve_node(&r->private_mm, &r->aux_node);
	for (u32 i = 0; !ret && i < G5P_RU_AUX_PAGES; i++) {
		ret = g5p_render_map_page(d, r,
					  G5P_RU_PRIVATE_VA + i * G5P_GPU_PAGE,
					  &d->rdev->aux[i]);
		r->aux_mapped = true;
	}
	if (ret)
		goto err_unlock;
	ret = xa_alloc(&f->contexts, &id, c, XA_LIMIT(1, G5P_MAX_CONTEXTS),
		       GFP_KERNEL);
	if (ret)
		goto err_unlock;
	kref_init(&c->ref);
	kref_get(&c->vm->ref);
	mutex_lock(&d->rdev->lock);
	list_add_tail(&r->rdev_link, &d->rdev->contexts);
	mutex_unlock(&d->rdev->lock);
	c->vm->contexts++;
	c->vm->serial_context = true;
	c->vm->resident = c;
	c->vm->zero_leaf_flags = v.memory == 4;
	a->handle = id;
	mutex_unlock(&f->lock);
	return 0;
err_unlock:
	if (c->vm)
		g5p_render_release(c);
	else
		kfree(r);
	mutex_unlock(&f->lock);
	kfree(c);
	return ret;
}

/* DESTROY_CONTEXT, file lock held.  A TYPED_V3 context that submitted: its close item, queued
 * behind its jobs, unbinds and releases it (g5p_render_close()).  Unbound: plain release.  A
 * legacy context that brought the device up, bound and idle: zero PA/tag and consume one DC3
 * per slot, as r21 detach does.  Failure or a quarantined legacy context retains everything
 * until reboot.
 */
static int g5p_render_destroy(struct g5p_context *c)
{
	struct g5p_render *r = c->render;
	struct g5p_drm *d = c->gpu;
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	u32 *ctl, start, target;
	unsigned long deadline;
	int ret;

	if (r->v3_accepted) {
		g5p_render_close(c);
		return 0;
	}
	if (atomic_read(&r->queued) || c->bad)
		return -EBUSY;
	if (!r->bound) {
		g5p_render_unhook(c);
		if (r->claimant) {
			/* Claimed but never touched hardware: drop the claim's reference.  The
			 * device stays claimed for render: other contexts may be queued on it.
			 */
			r->claimant = false;
			d->quarantined_context = NULL;
			g5p_context_put(c);
			drm_dev_put(&d->drm);
		}
		return 0;
	}
	/* The legacy unbind rings a doorbell: retained once the epoch is terminal. */
	if (!r->claimant || !g5p_ru_mmio_ok(&g5p_rv3_term))
		return -EBUSY;
	if (!mutex_trylock(&g->lock))
		return -EBUSY;
	ret = -EIO;
	if (b->failed || b->stopped)
		goto out;
	ctl = (u32 *)g5p_boot_shared(b, 0x6c);
	start = READ_ONCE(ctl[4]);
	ret = -EBUSY;
	if (start > 255 || READ_ONCE(ctl[0]) != start)
		goto out;
	for (u32 e = 0; e < 2; e++) {
		u32 index = (start + e) & 255;

		g5p_put64(g5p_boot_shared(b, 0x86) + r->slot[e] * 8, 0);
		g5p_put32(g5p_boot_shared(b, 0x84) + r->slot[e] * 4, 0);
		for (u32 j = 0; j < 5; j++) {
			u32 at = b->layout.offset[0x6d] + index * 20 + j * 4;

			g5p_put32(b->shared[at / G5P_GPU_PAGE].cpu +
					  at % G5P_GPU_PAGE,
				  j == 0 ? 3 :
				  j == 1 ? r->slot[e] :
					   0);
		}
	}
	target = (start + 2) & 255;
	dma_wmb();
	WRITE_ONCE(ctl[4], target);
	dma_wmb();
	ret = g5p_prepare_kick(g, "RENDER-USER-unbind");
	if (ret)
		goto fail;
	writel(0x10, g->rgx + 0xb00);
	deadline = jiffies + 3 * HZ;
	do {
		ret = g5p_boot_drain(g);
		if (ret || b->failed) {
			ret = ret ?: -EIO;
			goto fail;
		}
		dma_rmb();
		if (READ_ONCE(ctl[0]) == target)
			break;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	if (READ_ONCE(ctl[0]) != target) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	r->detached = true;
	r->bound = false;
	r->claimant = false;
	WRITE_ONCE(c->live, false);
	g5p_render_unhook(c);
	/* One render attachment per boot: accepted stays held, so neither a new
	 * render owner nor compute can reuse the shared channel pages.
	 */
	d->quarantined_context = NULL;
	g5p_mmu_record(
		g,
		"RENDER-USER detached slots=%u/%u; resources freed on release\n",
		r->slot[0], r->slot[1]);
	mutex_unlock(&g->lock);
	g5p_context_put(c);
	drm_dev_put(&d->drm);
	return 0;
fail:
	c->bad = true;
	b->failed = true;
out:
	mutex_unlock(&g->lock);
	return ret;
}

static int g5p_render_syncs(struct drm_file *file, struct g5p_rjob *job,
			    const struct drm_g5p_sync *syncs, u32 count,
			    struct drm_syncobj **signal, u64 *point,
			    struct dma_fence_chain **chain)
{
	u32 signal_handle = 0;
	int ret;

	if (!count || count > G5P_MAX_SYNCS)
		return -EINVAL;
	for (u32 i = 0; i < count; i++) {
		for (u32 j = 0; j < i; j++)
			if (syncs[i].handle == syncs[j].handle &&
			    syncs[i].point == syncs[j].point &&
			    syncs[i].flags == syncs[j].flags)
				return -EINVAL;
		if (syncs[i].flags & ~DRM_G5P_SYNC_SIGNAL)
			return -EINVAL;
		if (syncs[i].flags) {
			if (*signal)
				return -EINVAL;
			*signal = drm_syncobj_find(file, syncs[i].handle);
			if (!*signal)
				return -ENOENT;
			*point = syncs[i].point;
			signal_handle = syncs[i].handle;
			continue;
		}
		if (job->input_count == ARRAY_SIZE(job->inputs))
			return -E2BIG;
		ret = drm_syncobj_find_fence(file, syncs[i].handle,
					     syncs[i].point, 0,
					     &job->inputs[job->input_count]);
		if (ret)
			return ret;
		job->input_count++;
	}
	if (!*signal)
		return -EINVAL;
	for (u32 i = 0; i < count; i++)
		if (!syncs[i].flags && syncs[i].handle == signal_handle &&
		    (!*point || !syncs[i].point || syncs[i].point >= *point))
			return -EINVAL;
	if (*point) {
		struct dma_fence *old = drm_syncobj_fence_get(*signal);

		ret = old && (!to_dma_fence_chain(old) ||
			      old->seqno >= *point) ?
			      -EINVAL :
			      0;
		dma_fence_put(old);
		if (ret)
			return ret;
		*chain = dma_fence_chain_alloc();
		if (!*chain)
			return -ENOMEM;
	}
	return 0;
}

/* File lock held. One mapping of the listed BO must contain each range at a
 * consistent BO offset; access is a subset of the BO's immutable roles.
 */
static int g5p_render_resolve(struct drm_file *file, struct g5p_vm *vm,
			      const struct drm_g5p_render_resource *res,
			      u32 count, struct g5p_ru_range *ranges,
			      struct g5p_robj *objs, u32 *obj_count)
{
	u32 known = DRM_G5P_RENDER_ACCESS_READ | DRM_G5P_RENDER_ACCESS_WRITE |
		    DRM_G5P_RENDER_ACCESS_IMPLICIT |
		    DRM_G5P_RENDER_ACCESS_CACHE;

	for (u32 i = 0; i < count; i++) {
		struct drm_gem_object *obj;
		struct g5p_mapping *m, *found = NULL;
		u32 roles, access;

		for (u32 j = 0; j < i; j++)
			if (res[i].handle == res[j].handle)
				return -EINVAL;
		obj = drm_gem_object_lookup(file, res[i].handle);
		if (!obj)
			return -ENOENT;
		roles = to_g5p_bo(obj)->roles;
		{
			access = res[i].kind;
			if (!(access &
			      (G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE)) ||
			    access & ~known ||
			    ((access & G5P_RU_ACCESS_READ) &&
			     !(roles & DRM_G5P_BO_READ)) ||
			    ((access & G5P_RU_ACCESS_WRITE) &&
			     !(roles & DRM_G5P_BO_WRITE))) {
				drm_gem_object_put(obj);
				return -EINVAL;
			}
			/* A foreign buffer always gets cache maintenance and fences. */
			if (drm_gem_is_imported(obj))
				access |= DRM_G5P_RENDER_ACCESS_IMPLICIT |
					  DRM_G5P_RENDER_ACCESS_CACHE;
		}
		list_for_each_entry(m, &vm->maps, link)
			if (m->obj == obj && res[i].va >= m->node.start &&
			    res[i].size &&
			    res[i].va - m->node.start < m->node.size &&
			    res[i].size <= m->node.size - (res[i].va -
							   m->node.start) &&
			    res[i].offset ==
				    m->offset + (res[i].va - m->node.start))
				found = m;
		if (!found) {
			drm_gem_object_put(obj);
			return -EINVAL;
		}
		if (access & (DRM_G5P_RENDER_ACCESS_IMPLICIT |
			      DRM_G5P_RENDER_ACCESS_CACHE)) {
			struct g5p_robj *o = &objs[*obj_count];

			o->obj = obj; /* the job inherits this reference */
			o->access = access;
			o->lo = 0;
			o->hi = obj->size;
			if ((READ_ONCE(render_user_sync_range) & 2) &&
			    !g5p_sync_window(res[i].offset, res[i].size,
					     obj->size, &o->lo, &o->hi)) {
				o->lo = 0;
				o->hi = obj->size;
			}
			(*obj_count)++;
		} else {
			drm_gem_object_put(obj);
		}
		ranges[i].va = res[i].va;
		ranges[i].size = res[i].size;
		ranges[i].access = access &
				   (G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE);
		/* The BO's EXEC role, for TYPED_V3 only; an imported BO never has it. */
		if ((roles & DRM_G5P_BO_EXEC))
			ranges[i].access |= G5P_RU_ACCESS_EXEC;
	}
	return 0;
}

/* LINEAR64_V1 admits only the exact round16 immutable inputs. Writes stay
 * privileged and caller-serialized, as for the r20 first-store dialect.
 */

/* File lock held. Copy `len` bytes at GPU VA `va` out of the BO the VM maps
 * there, through the BO's own pages, as g5p_render_profile1_bytes() does. The
 * mapping is found again by VA and its own (VM-referenced) object is read, so
 * a handle closed or replaced since g5p_render_resolve() cannot redirect the
 * read. Only a native BO has shmem pages: an imported one (never EXEC, so
 * never a TA program) is refused for the stream too.
 */
static int g5p_render_read_va(struct drm_device *drm, struct g5p_vm *vm, u64 va,
			      void *dst, u32 len)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	struct g5p_mapping *m, *found = NULL;
	struct drm_gem_object *obj;
	struct sg_table *sgt;
	u64 off;
	int ret;

	list_for_each_entry(m, &vm->maps, link)
		if (va >= m->node.start && va - m->node.start < m->node.size &&
		    len <= m->node.size - (va - m->node.start))
			found = m;
	if (!found)
		return -ENOENT;
	obj = found->obj;
	off = found->offset + (va - found->node.start);
	if (drm_gem_is_imported(obj) || off > obj->size ||
	    len > obj->size - off)
		return -EINVAL;
	sgt = drm_gem_shmem_get_pages_sgt(to_drm_gem_shmem_obj(obj));
	ret = IS_ERR(sgt) ? PTR_ERR(sgt) : drm_gem_vmap(obj, &map);
	if (ret)
		return ret;
	dma_sync_sgtable_for_cpu(drm->dev, sgt, DMA_BIDIRECTIONAL);
	if (map.is_iomem)
		ret = -EINVAL;
	else
		memcpy(dst, (const u8 *)map.vaddr + off, len);
	dma_sync_sgtable_for_device(drm->dev, sgt, DMA_BIDIRECTIONAL);
	drm_gem_vunmap(obj, &map);
	return ret;
}

static void g5p_render_pass_from_uapi(struct g5p_ru_pass *p,
				      const struct drm_g5p_render_pass *u)
{
	p->vdm_va = u->vdm_va;
	p->vdm_size = u->vdm_size;
	p->vdm_control = u->vdm_control;
	p->scratch_va = u->scratch_va;
	p->scratch_size = u->scratch_size;
	memcpy(p->scratch_ptr, u->scratch_ptr, sizeof(p->scratch_ptr));
	p->pool7_va = u->pool7_va;
	p->pool7_size = u->pool7_size;
	p->pool8_va = u->pool8_va;
	p->pool8_size = u->pool8_size;
	memcpy(p->context_pds, u->context_pds, sizeof(p->context_pds));
	memcpy(p->resume_pds, u->resume_pds, sizeof(p->resume_pds));
	memcpy(p->ta_pds, u->ta_pds, sizeof(p->ta_pds));
	memcpy(p->background, u->background, sizeof(p->background));
	memcpy(p->eot, u->eot, sizeof(p->eot));
	memcpy(p->eot_resume, u->eot_resume, sizeof(p->eot_resume));
	memcpy(p->restore, u->restore, sizeof(p->restore));
}

static void g5p_render_pass2_from_uapi(struct g5p_ru_pass2 *e,
				       const struct drm_g5p_render_pass_ext *u)
{
	*e = (struct g5p_ru_pass2){
		.scissor_va = u->scissor_va,
		.dbias_va = u->dbias_va,
		.background_texture = u->background_texture,
		.scissor_count = u->scissor_count,
		.dbias_count = u->dbias_count,
		.bgobj_depth = u->bgobj_depth,
		.bgobj_vals = u->bgobj_vals,
		.pixel_width = u->pixel_output_width,
		.zls_mask = u->zls_address_mask,
	};
	memcpy(e->zls, u->zls, sizeof(e->zls));
}

static void g5p_render_pass3_from_uapi(struct g5p_ru_pass3 *v,
				       const struct drm_g5p_render_pass_v3 *u)
{
	*v = (struct g5p_ru_pass3){
		.ta_program_va = u->ta_program_va,
		.ta_program_size = u->ta_program_size,
		.vdm_end = u->vdm_end,
		.pixel_width = u->pixel_output_width,
		.pad = u->pad,
	};
}

static_assert(offsetof(struct drm_g5p_render_pass, samples) == 296);
static_assert(offsetof(struct drm_g5p_render_pass, sample_positions) == 304);
static_assert(offsetof(struct drm_g5p_render_pass, reserved) == 312);
static_assert(DRM_G5P_CAP_RENDER_MSAA == BIT_ULL(31));
static_assert(DRM_G5P_RENDER_MSAA_RESOLVE == G5P_RU_MSAA_RESOLVE);

/* CAP_RENDER_MSAA: the target's geometry is kernel-derived (it holds the sample
 * count), and only TYPED_V3 carries the pass fields.
 */
static bool g5p_render_msaa_enabled(void)
{
	return (1) && (1);
}

/* The base pass's MSAA fields: zero unless TYPED_V3 with the capability. */
static int g5p_render_msaa_from_uapi(struct g5p_ru_msaa *m,
				     const struct drm_g5p_render_pass *u,
				     bool v3)
{
	*m = (struct g5p_ru_msaa){
		.samples = u->samples,
		.flags = u->msaa_flags,
		.positions = u->sample_positions,
	};
	if ((m->samples || m->flags || m->positions) &&
	    (!v3 || !g5p_render_msaa_enabled()))
		return -EINVAL;
	return 0;
}

/* Render submit lock held.  The first accepted job claims the device for render until reboot;
 * same exclusions as compute/r22.  Every later job, of any context, passes while the device is
 * neither failed nor stopped, without the core lock (a job may hold it for its whole run).  The
 * claimant keeps the reference d->quarantined_context has always held.  The IRQ-test term is
 * the r74 one (render-fixed.inc, render-r72.inc): the r74 boots refuse the IRQ test, so
 * irq_tested must stay 0 there and must be 1 on every other boot.
 */
static int g5p_render_device_claim(struct g5p_drm *d, struct g5p_context *c)
{
	struct g5p_diag *g = d->core;
	struct g5p_boot *b = g->boot;
	struct g5p_render *r = c->render;
	int ret = -EBUSY;

	if (READ_ONCE(d->render_claimed)) {
		/* A recovery swaps the epoch, and a failure noted mid-job may lead to one: the job
		 * waits for the ruling in the scheduler, which fails it if the epoch ends.
		 */
		if (g5p_rdev_ruling(d->rdev))
			return 0;
		b = READ_ONCE(g->boot);
		return !b || READ_ONCE(b->failed) || READ_ONCE(b->stopped) ?
			       -EIO :
			       0;
	}
	if (!mutex_trylock(&g->lock))
		return -EBUSY;
	if (b->failed || b->stopped || !b->control_complete || b->irq_tested ||
	    b->user_compute_owned || d->quarantined_context ||
	    !b->render_meta[0].cpu || !b->render_meta[1].cpu ||
	    b->layout.used[1] > G5P_R22_TA ||
	    /* recovery108: expose the GPU only with a current-generation ALIVE proof. */
	    g5p_ds_hook_claim(g) || atomic_cmpxchg(&d->accepted, 0, 1))
		goto out;
	b->user_compute_owned = true; /* blocks diagnostic commands, latched */
	r->claimant = true;
	kref_get(&c->ref);
	drm_dev_get(&d->drm);
	d->quarantined_context = c;
	WRITE_ONCE(d->render_claimed, true);
	ret = 0;
out:
	mutex_unlock(&g->lock);
	return ret;
}

/* Evict the oldest completed result beyond the per-file bound. */
static int g5p_render_result_slot(struct g5p_file *f, struct g5p_drm *d)
{
	unsigned long index = 0, flags;
	struct g5p_fence *old;
	bool done;

	if (f->render_results < G5P_RU_RESULTS)
		return 0;
	old = xa_find(&f->results, &index, ULONG_MAX, XA_PRESENT);
	if (!old)
		return 0;
	spin_lock_irqsave(&d->status_lock, flags);
	done = old->state == DRM_G5P_JOB_COMPLETE;
	spin_unlock_irqrestore(&d->status_lock, flags);
	if (!done)
		return -EBUSY;
	xa_erase(&f->results, index);
	dma_fence_put(&old->base);
	f->render_results--;
	return 0;
}

static int g5p_render_submit_admitted(struct drm_device *drm,
				      struct drm_file *file, u32 context,
				      u32 profile, u32 rt_handle, u64 res_ptr,
				      u32 res_count, u64 sync_ptr,
				      u32 sync_count, u64 pass_ptr,
				      u32 pass_size, u64 *job_id)
{
	struct g5p_drm *d = to_g5p(drm);
	struct g5p_file *f = file->driver_priv;
	struct g5p_boot *b = READ_ONCE(d->core->boot);
	struct drm_g5p_render_resource *res = NULL;
	struct drm_g5p_render_pass_v3 *upass = NULL;
	struct drm_g5p_sync *syncs = NULL;
	struct drm_syncobj *signal = NULL;
	struct dma_fence_chain *chain = NULL;
	struct g5p_ru_range *ranges = NULL;
	struct g5p_ru_pass pass;
	struct g5p_ru_pass2 pass2 = {};
	struct g5p_ru_pass3 pass3 = {};
	struct g5p_ru_v3_loc loc = {};
	struct g5p_ru_msaa msaa = {};
	struct g5p_context *c;
	struct g5p_render *r;
	struct g5p_rjob *job;
	struct g5p_rt *rt = NULL;
	const bool v2 = false;
	const bool v3 = true;
	u64 point = 0;
	int ret;

	;
	;
	/* TYPED_V3 is unprivileged (checked per context below); the legacy profiles touch
	 * registers and stay with CAP_SYS_RAWIO.
	 */
	;
	;
	/* During a recovery the epoch is being replaced: accept, the scheduler holds the job. */
	if (!READ_ONCE(d->rdev->recovering)) {
		if (!b || !READ_ONCE(b->alive) || !READ_ONCE(b->released))
			return -EAGAIN;
		if (READ_ONCE(b->stopped))
			return -ESHUTDOWN;
		/* a failure noted mid-job waits for the executor's ruling (g5p_rdev_ruling()) */
		if (READ_ONCE(b->failed) && !g5p_rdev_ruling(d->rdev))
			return -EIO;
	}
	res = memdup_array_user(u64_to_user_ptr(res_ptr), res_count,
				sizeof(*res));
	if (IS_ERR(res))
		return PTR_ERR(res);
	syncs = memdup_array_user(u64_to_user_ptr(sync_ptr), sync_count,
				  sizeof(*syncs));
	if (IS_ERR(syncs)) {
		ret = PTR_ERR(syncs);
		syncs = NULL;
		goto out;
	}
	{
		/* pass_size is the exact V1, V2 or V3 size; ext exists for V2 and V3,
		 * the program fields for V3 only.
		 */
		upass = memdup_user(u64_to_user_ptr(pass_ptr), pass_size);
		if (IS_ERR(upass)) {
			ret = PTR_ERR(upass);
			upass = NULL;
			goto out;
		}
		if (upass->base.size != pass_size || upass->base.flags ||
		    memchr_inv(upass->base.reserved, 0,
			       sizeof(upass->base.reserved)) ||
		    (true && memchr_inv(upass->ext.reserved, 0,
					sizeof(upass->ext.reserved))) ||
		    (v3 &&
		     memchr_inv(upass->reserved, 0, sizeof(upass->reserved)))) {
			ret = -EINVAL;
			goto out;
		}
		g5p_render_pass_from_uapi(&pass, &upass->base);
		ret = g5p_render_msaa_from_uapi(&msaa, &upass->base, v3);
		if (ret)
			goto out;
		{
			g5p_render_pass2_from_uapi(&pass2, &upass->ext);
		};
		{
			g5p_render_pass3_from_uapi(&pass3, upass);
		};
	}
	ranges = kvcalloc(res_count, sizeof(*ranges), GFP_KERNEL);
	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (job) {
		job->submit_ns = ktime_get_ns();
		job->objs = kvcalloc(res_count, sizeof(*job->objs), GFP_KERNEL);
	}
	if (!ranges || !job || !job->objs) {
		if (job)
			kvfree(job->objs);
		kfree(job);
		ret = -ENOMEM;
		goto out;
	}
	ret = g5p_render_syncs(file, job, syncs, sync_count, &signal, &point,
			       &chain);
	if (ret)
		goto out_job;
	job->fence = kzalloc(sizeof(*job->fence), GFP_KERNEL);
	if (!job->fence) {
		ret = -ENOMEM;
		goto out_job;
	}
	spin_lock_init(&job->fence->lock);
	/* Acceptance, implicit fence attach and queueing happen in one order. */
	ret = mutex_lock_interruptible(&d->render_submit_lock);
	if (ret)
		goto out_job;
	mutex_lock(&f->lock);
	c = xa_load(&f->contexts, context);
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
	{
		ret = g5p_rv3_admit_submit(d, r);
		if (ret)
			goto unlock;
	}
	if (atomic_read(&r->queued) >= G5P_RU_MAX_QUEUED) {
		ret = -EBUSY;
		goto unlock;
	}
	ret = g5p_render_resolve(file, c->vm, res, res_count, ranges, job->objs,
				 &job->obj_count);
	if (ret)
		goto unlock;
	{
		rt = xa_load(&f->targets, rt_handle);
		if (!rt || rt->ctx != c || rt->destroyed) {
			ret = -ENOENT;
			goto unlock;
		}
	}
	{
		static const struct g5p_ru_v3_limits lim = {
			G5P_RU_PDS_BASE,
			G5P_RU_CODE_WINDOW,
			G5P_RU_HW_VA_LIMIT,
		};
		u8 head[G5P_RU_PROGRAM_MIN], tail[4];

		/* Structure first; the program head and the stream's last dword are
		 * read only once every range is proven backed, READ and (program) EXEC.
		 */
		ret = g5p_ru_pass3_check(&pass, &pass2, &pass3, &lim, ranges,
					 res_count, &loc);
		if (!ret)
			ret = g5p_render_read_va(drm, c->vm,
						 pass3.ta_program_va, head,
						 sizeof(head));
		if (!ret)
			ret = g5p_render_read_va(
				drm, c->vm, pass.vdm_va + pass3.vdm_end - 4,
				tail, sizeof(tail));
		if (!ret)
			ret = g5p_ru_pass3_content(head, tail);
	}
	/* Every profile: a one-sample pass needs a one-sample target, and back. */
	if (!ret)
		ret = g5p_ru_msaa_check(&msaa, rt->samples ? rt->samples : 1);
	if (ret)
		goto unlock;
	g5p_ru_fill_ta(job->ta_w, &pass);
	{
		g5p_ru_fill_ta_v3(job->ta_w, loc.program_off);
	};
	g5p_ru_fill_3d(job->td_w, &pass);
	/* TYPED_V3 with an all-zero ext is the V1 dialect: the V1 3D words, plus
	 * the output width words (width 2 rewrites the frozen dialect-1 values).
	 */
	if (v2 || loc.dialect2)
		g5p_ru_fill_3d_v2(job->td_w, &pass2);
	else {
		g5p_ru_fill_3d_width(job->td_w, loc.pixel_width);
	};
	/* r25 RT variant 1 moves only TA.W28/3D.W20 one slice into the ring. */
	g5p_ru_rt_fill(job->ta_w, job->td_w, &rt->geometry,
		       rt->node.start + (r->v.rt == 1 ? rt->sizes.slice : 0),
		       rt->node.start + rt->sizes.main);
	{
		g5p_ru_fill_msaa(job->ta_w, job->td_w, &msaa, &rt->geometry);
	};
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
		/* Unpublished, never signalled: still a clean rejection. */
		dma_fence_put(&job->fence->base);
		job->fence = NULL;
		goto unlock;
	}
	/* Accepted from here: every later step is infallible. */
	dma_fence_get(&job->fence->base);
	f->render_results++;
	job->v3 = v3;
	{
		r->v3_accepted = true;
	};
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
		kfree(job->fence); /* only ever an uninitialized fence here */
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

/* recovery108: a submitter is inside the admission window for its whole call; once a
 * FINAL teardown closed admission it is refused before it looks at anything.
 */
static int g5p_render_submit(struct drm_device *drm, struct drm_file *file,
			     u32 context, u32 profile, u32 rt_handle,
			     u64 res_ptr, u32 res_count, u64 sync_ptr,
			     u32 sync_count, u64 pass_ptr, u32 pass_size,
			     u64 *job_id)
{
	int ret;

	if (!g5p_r8_enter())
		return -ESHUTDOWN;
	ret = g5p_render_submit_admitted(drm, file, context, profile, rt_handle,
					 res_ptr, res_count, sync_ptr,
					 sync_count, pass_ptr, pass_size,
					 job_id);
	g5p_r8_exit();
	return ret;
}

/* The layout the profile dispatch (pass_size selects the profile) and
 * GET_RENDER_INFO rely on; a header edit that moves any of it fails here.
 */
static_assert(sizeof(struct drm_g5p_render_pass) == 360);
static_assert(offsetof(struct drm_g5p_render_pass, vdm_control) == 20);
static_assert(sizeof(struct drm_g5p_render_pass_ext) == 128);
static_assert(sizeof(struct drm_g5p_render_pass_v2) == 488);
static_assert(sizeof(struct drm_g5p_render_pass_v3) == 536);
static_assert(offsetof(struct drm_g5p_render_pass_v3, ext) ==
	      sizeof(struct drm_g5p_render_pass));
static_assert(offsetof(struct drm_g5p_render_pass_v3, ta_program_va) == 488);
static_assert(offsetof(struct drm_g5p_render_pass_v3, ta_program_size) == 496);
static_assert(offsetof(struct drm_g5p_render_pass_v3, vdm_end) == 500);
static_assert(offsetof(struct drm_g5p_render_pass_v3, pixel_output_width) ==
	      504);
static_assert(offsetof(struct drm_g5p_render_pass_v3, reserved) == 512);
static_assert(sizeof(struct drm_g5p_submit_render_typed) == 80);
static_assert(sizeof(struct drm_g5p_get_render_info) == 120);
static_assert(offsetof(struct drm_g5p_get_render_info, hw_va_limit) == 104);
static_assert(offsetof(struct drm_g5p_get_render_info, reserved1) == 112);
static_assert(DRM_G5P_CAP_TYPED_RENDER_V3 == BIT_ULL(26));
static_assert(DRM_G5P_CAP_RENDER_MULTI_CONTEXT == BIT_ULL(27));
static_assert(DRM_G5P_CAP_RENDER_PIXEL_OUTPUT_WIDTH == BIT_ULL(29));
static_assert(DRM_G5P_RENDER_PROFILE_TYPED_V3 == 4);

static int g5p_submit_render_typed(struct drm_device *drm, void *data,
				   struct drm_file *file)
{
	struct drm_g5p_submit_render_typed *a = data;
	u64 id = 0;
	int ret;
	if (a->flags || a->pad || a->job_id || a->reserved[0] ||
	    a->reserved[1] || a->profile != DRM_G5P_RENDER_PROFILE_TYPED_V3 ||
	    !a->render_target || !a->resource_count ||
	    a->resource_count > DRM_G5P_RENDER_MAX_RESOURCES ||
	    !a->sync_count || a->sync_count > DRM_G5P_RENDER_MAX_SYNCS ||
	    a->pass_size != sizeof(struct drm_g5p_render_pass_v3))
		return -EINVAL;
	ret = g5p_render_submit(drm, file, a->context,
				DRM_G5P_RENDER_PROFILE_TYPED_V3,
				a->render_target, a->resources,
				a->resource_count, a->syncs, a->sync_count,
				a->pass, a->pass_size, &id);
	a->job_id = id;
	return ret;
}

static void g5p_rt_geometry_to_uapi(struct drm_g5p_rt_geometry *u,
				    const struct g5p_ru_rt *k)
{
	u->header_bytes = k->header;
	u->region_bytes = k->region;
	u->packed_state = k->packed;
	u->state_word = k->state;
	u->padded_dims = k->padded;
	memcpy(u->scale, k->scale, sizeof(u->scale));
	memcpy(u->ta_state, k->ta_state, sizeof(u->ta_state));
	memcpy(u->ta_tiling, k->ta_tiling, sizeof(u->ta_tiling));
	u->ta_tiling_word = k->ta_tiling_word;
	u->tile_count = k->tiles;
	u->secondary_bytes = k->secondary;
	u->secondary_align = k->secondary_align;
}

static int g5p_create_render_target(struct drm_device *drm, void *data,
				    struct drm_file *file)
{
	struct drm_g5p_create_render_target *a = data;
	struct g5p_drm *d = to_g5p(drm);
	struct g5p_file *f = file->driver_priv;
	const struct drm_g5p_rt_geometry *u = &a->geometry;
	struct g5p_ru_rt geometry;
	struct g5p_context *c;
	struct g5p_render *r;
	struct g5p_rt *rt;
	bool user = a->flags & DRM_G5P_RT_USER_GEOMETRY;
	bool kernel = (1);
	u64 positions = a->samples == 4 ? G5P_RU_POSITIONS_4X :
					  G5P_RU_POSITIONS_1X;
	u32 id, blocks = 0;
	int ret;

	;
	;
	if (a->flags & ~DRM_G5P_RT_USER_GEOMETRY || a->handle || a->pad ||
	    a->main_va || a->main_bytes || a->secondary_va ||
	    a->secondary_bytes ||
	    (a->samples != 1 &&
	     (a->samples != 4 || !g5p_render_msaa_enabled())) ||
	    a->layers != 1 || a->pm_blocks || a->pad2 || a->reserved ||
	    u->reserved[0] || u->reserved[1])
		return -EINVAL;
	if (user) {
		{
			return -EOPNOTSUPP;
		}
	} else if (kernel) {
		/* 6ca6d5c in integers; equal to the leaf for every size. */
		if (memchr_inv(u, 0, sizeof(*u)) ||
		    g5p_ru_rt_derive_samples(a->width, a->height, a->samples,
					     positions, &geometry))
			return -EINVAL;
	} else {
		/* Kernel table: only the offline-executed 6ca6d5c result. */
		if (a->width != 64 || a->height != 64 ||
		    memchr_inv(u, 0, sizeof(*u)))
			return -EOPNOTSUPP;
		geometry = g5p_ru_rt64;
	}
	mutex_lock(&f->lock);
	c = xa_load(&f->contexts, a->context);
	if (!c || !c->render) {
		ret = c ? -EINVAL : -ENOENT;
		goto out;
	}
	r = c->render;
	ret = g5p_render_vm_idle(c, false);
	if (ret)
		goto out;
	/* No core lock: another context's job may hold it for its whole run, and nothing here
	 * touches the hardware.  The device pool only grows; it serves every target of every
	 * context.
	 */
	if (kernel) {
		mutex_lock(&d->rdev->lock);
		ret = g5p_rdev_grow(d, g5p_ru_pm_blocks_for(geometry.tiles,
							    d->rdev->blocks));
		blocks = d->rdev->blocks;
		mutex_unlock(&d->rdev->lock);
	}
	if (!ret)
		ret = g5p_rt_create(d, r, &geometry, a->width, a->height, &rt);
	if (ret)
		goto out;
	rt->samples = a->samples;
	ret = xa_alloc(&f->targets, &id, rt,
		       XA_LIMIT(1, G5P_RU_MAX_TARGETS * 8), GFP_KERNEL);
	if (ret) {
		/* Mapped but unpublished to a job: leave it owned by the context. */
		rt->destroyed = true;
		goto out;
	}
	kref_get(&c->ref);
	a->handle = id;
	a->main_va = rt->node.start;
	a->main_bytes = rt->sizes.main;
	a->secondary_va = rt->node.start + rt->sizes.main;
	a->secondary_bytes = rt->sizes.secondary_map;
	if (!user)
		g5p_rt_geometry_to_uapi(&a->geometry, &geometry);
	if (kernel)
		a->pm_blocks = blocks;
out:
	mutex_unlock(&f->lock);
	return ret;
}

static int g5p_destroy_render_target(struct drm_device *drm, void *data,
				     struct drm_file *file)
{
	struct drm_g5p_destroy_render_target *a = data;
	struct g5p_drm *d = to_g5p(drm);
	struct g5p_file *f = file->driver_priv;
	struct g5p_context *c = NULL;
	struct g5p_render *r;
	struct g5p_rt *rt;
	int ret;

	if (a->flags)
		return -EINVAL;
	mutex_lock(&f->lock);
	rt = xa_load(&f->targets, a->handle);
	if (!rt) {
		ret = -ENOENT;
		goto out;
	}
	c = rt->ctx;
	r = c->render;
	ret = r->detached ? 0 : g5p_render_vm_idle(c, false);
	if (ret)
		goto out;
	g5p_vmlog_add(G5P_VMLOG_RT_DEL, c->vm, 0, rt->node.start,
		      rt->sizes.main + rt->sizes.secondary_map,
		      rt->width << 16 | rt->height, rt->samples);
	/* RAM only, no core lock: see g5p_create_render_target(). */
	if (r->bound) {
		g5p_vm_clear(c->vm, drm->dev, rt->node.start,
			     rt->sizes.main + rt->sizes.secondary_map);
		ret = g5p_render_invalidate(c);
		if (ret)
			goto out;
	}
	if (!r->detached) {
		g5p_rt_release_memory(d, r, rt, true);
		list_del(&rt->link);
		r->target_count--;
		if (r->profile1_rt == rt)
			r->profile1_rt = NULL;
		kfree(rt);
	} else {
		rt->destroyed = true;
	}
	xa_erase(&f->targets, a->handle);
out:
	mutex_unlock(&f->lock);
	if (!ret && c)
		g5p_context_put(c);
	return ret;
}

/* True when a context created now would latch a hypothesis knob that differs
 * from its default: GET_RENDER_INFO has no context to report on.
 */
static bool g5p_render_variant_active(void)
{
	return (0) || (G5P_RU_DEF_CACHE_VARIANT) != G5P_RU_DEF_CACHE_VARIANT ||
	       (G5P_RU_DEF_MMU_VARIANT) != G5P_RU_DEF_MMU_VARIANT || (0) ||
	       READ_ONCE(render_user_wait_ms) != G5P_RU_DEF_WAIT_MS || (0) ||
	       (0) || (0) ||
	       (G5P_RU_DEF_PROFILE1_VDM_CONTROL) !=
		       G5P_RU_DEF_PROFILE1_VDM_CONTROL ||
	       (0) || (0) || (0) || (0);
}

static int g5p_get_render_info(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct drm_g5p_get_render_info *a = data;

	if (a->size != sizeof(*a) || a->flags ||
	    memchr_inv(&a->profiles, 0,
		       sizeof(*a) - offsetof(struct drm_g5p_get_render_info,
					     profiles)))
		return -EINVAL;
	;
	;
	if (g5p_render_variant_active())
		a->flags = DRM_G5P_RENDER_INFO_VARIANT_ACTIVE;
	a->profiles = BIT(DRM_G5P_RENDER_PROFILE_TYPED_V3);
	;
	;
	{
		a->profiles |= BIT(DRM_G5P_RENDER_PROFILE_TYPED_V3);
	};
	{
		a->max_render_width = G5P_RU_MAX_DIM;
		a->max_render_height = G5P_RU_MAX_DIM;
		a->max_pm_blocks = G5P_RU_MAX_BLOCKS;
	}
	a->max_render_targets = G5P_RU_MAX_TARGETS;
	a->private_va_start = G5P_RU_PRIVATE_VA;
	a->private_va_end = G5P_RU_PRIVATE_VA + G5P_RU_PRIVATE_SIZE;
	a->user_va_min = G5P_RU_USER_VA_MIN;
	a->usc_base = G5P_RU_PDS_BASE;
	a->pds_base = a->usc_base;
	a->code_window = G5P_RU_CODE_WINDOW;
	a->hw_va_limit = G5P_RU_HW_VA_LIMIT;
	a->pm_blocks = (32);
	a->pm_block_bytes = G5P_RU_BLOCK_BYTES;
	a->max_queued_jobs = G5P_RU_MAX_QUEUED;
	a->max_vdm_bytes = DRM_G5P_RENDER_MAX_VDM_BYTES;
	/* TYPED_V3: what a waiter must outlast is the job's absolute bound (TA publish to the end of the
	 * 3D, render-user-wait.h), not the per-phase module value: a TA that takes its whole phase leaves the
	 * 3D the rest of the job cap.
	 */
	{
		if (READ_ONCE(render_user_hard_ms))
			a->job_timeout_ms = g5p_ru_lat_job_ms(
				READ_ONCE(render_user_hard_ms),
				READ_ONCE(render_user_obs_ms));
		else
			a->job_timeout_ms = G5P_RU_V3_CAP_MS;
	};
	a->retained_results = G5P_RU_RESULTS;
	return 0;
}

/* VM_BIND hooks for the context resident on a VM (serial compute or render). */
static int g5p_resident_idle(struct g5p_context *c, bool live)
{
	return g5p_render_vm_idle(c, live);
}

/* VM_BIND UNMAP, file lock held, PTEs already cleared and the tag published: a render
 * context with queued jobs keeps the mapping's object (its pages and its pin) until those
 * jobs have retired (g5p_render_reap()).  False: free it now.
 */
static bool g5p_resident_defer(struct g5p_context *c, struct g5p_mapping *m)
{
	struct g5p_render *r = c->render;
	unsigned long flags;

	if (!r || !atomic_read(&r->queued))
		return false;
	m->retire = r->fence_seqno;
	spin_lock_irqsave(&r->zlock, flags);
	list_add_tail(&m->link, &r->zombies);
	spin_unlock_irqrestore(&r->zlock, flags);
	return true;
}

static int g5p_resident_invalidate(struct g5p_context *c)
{
	return g5p_render_invalidate(c);
}
