// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Experimental render frontend; default disabled, no implicit firmware boot. */
#include "../../apple-g5p/core.inc"
#include "../../apple-g5p/drm-validate.h"
#include "../../apple-g5p/render-user-evwait.h"
#include "../../apple-g5p/render-user-sync.h"
#include <linux/dma-buf.h>
#include <linux/dma-fence-chain.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/dma-resv.h>
#include <linux/xarray.h>
#include <linux/bitmap.h>
#include <linux/kref.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/seq_file.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_mm.h>
#include <drm/drm_prime.h>
#include <drm/drm_syncobj.h>
#include <uapi/drm/apple_g5p_drm.h>

#define G5P_MAX_VMS 8
#define G5P_MAX_CONTEXTS 32
#define G5P_MAX_MAPS 4096
#define G5P_MAX_PT_PAGES 512
#define G5P_MAX_SYNCS 17

struct g5p_bo {
	struct drm_gem_shmem_object shmem;
	u32 roles;
	u32 vm_pins; /* VMs that map it, any file; counted once in pinned_bytes (pin_lock) */
	/* release8 memory accounting (drm-vm.inc g5p_bo_live): the live list, its creator */
	struct list_head live;
	pid_t pid;
	char comm[TASK_COMM_LEN];
};

struct g5p_pt {
	struct list_head link;
	struct g5p_dma_page page;
	u32 key;
	u32 level;
};

struct g5p_mapping {
	struct list_head link;
	struct drm_mm_node node;
	struct drm_gem_object *obj;
	u64 offset;
	u64 retire; /* unmapped beside queued jobs: freed once this fence seqno has signalled */
};

struct g5p_vm {
	struct g5p_vm_mutation mutation;
	bool bounded, frozen, serial_context;
	bool zero_leaf_flags; /* render context latched r25 memory variant 4 */
	/* Its render context's handle is gone (g5p_render_close()): no VM_BIND, no new
	 * context; the context's close item and release own the tables.
	 */
	bool resident_gone;
	struct g5p_context *resident;
	struct kref ref;
	struct device *dev;
	struct g5p_drm *gpu;
	struct drm_mm mm;
	struct drm_mm_node reserved;
	struct list_head maps;
	struct xarray objs; /* object -> its mappings in this VM (xa_mk_value) */
	struct list_head tables;
	struct g5p_dma_page root;
	u64 mapped_bytes, pinned_bytes;
	u32 table_count, map_count, contexts;
};

struct g5p_drm;
struct g5p_render;
struct g5p_rdev;

struct g5p_context {
	struct g5p_drm *gpu;

	bool live, bad;
	bool serial, pending;

	struct g5p_render *render; /* render contexts only, see drm-render.inc */
	struct kref ref;
	struct g5p_vm *vm;
};

struct g5p_file {
	/* Serializes VM/context handles, mappings and the one-shot submit state. */
	struct mutex lock;
	struct xarray vms;
	struct xarray contexts;
	struct xarray results;
	struct xarray targets; /* render targets; each holds its context */
	u32 render_results;
	bool submitted;
	u64 job_id;
};

struct g5p_drm {
	struct drm_device drm;
	struct g5p_diag *core;
	/* Protects cached IRQ event/status fields; never held for hardware IO. */
	spinlock_t status_lock;
	u32 last_event[11], notifications[4], first_fault[11];
	u64 event_serial, fault_serial;
	/* render_user_event_wake: the phase waiter sleeps here until event_serial moves */
	wait_queue_head_t ev_wq;
	/* TYPED_V3 3D completion-event latch (render-user-retire.h): G5P_RV3_TD_WANT | endpoint
	 * while a 3D phase waits, else 0; td_hit_ns: decode time of the first matching event.
	 */
	u32 td_want;
	u64 td_hit_ns;
	/* The TA's latch: armed while a TA phase is published; ta_hit_ns as td_hit_ns. */
	bool ta_want;
	u64 ta_hit_ns;
	/* wake110: type-1 completion events decoded per engine (word 1 bit 0 TA, bit 1 3D) since
	 * load, and the decode time of the first one since the executor last cleared it.
	 */
	u32 t1_count[2];
	u64 t1_first_ns[2];
	/* perf60 bell gates (drm-render-v3.inc): the previous TYPED_V3 job's end and context. */
	u64 bell_prev_end_ns;
	const void *bell_prev_ctx;
	u64 last_job;
	int last_error, job_error;
	u32 job_state;

	atomic_t accepted;
	atomic64_t job_sequence;
	u64 fence_context;
	/* Attached catalogues survive fd close until proven unbind/reset. */
	struct g5p_context *quarantined_context;
	/* The first accepted render job claimed the device for render, for the rest of the boot. */
	bool render_claimed;
	struct g5p_rdev *rdev; /* device-wide render state, see drm-render.inc */
	struct workqueue_struct
		*render_wq; /* ordered: the render scheduler's worker */
	/* Orders render acceptance, dma_resv fence attach and queueing. */
	struct mutex render_submit_lock;
	/* recovery108 epoch inputs, RAM only: open render-node files, render jobs not yet
	 * signalled.
	 */
	atomic_t open_files, r8_queued;
	/* Memory mapped into any VM, each object once (render_pinned_mib). */
	struct mutex pin_lock;
	u64 pinned_bytes;
};

static inline struct g5p_drm *to_g5p(struct drm_device *drm)
{
	return container_of(drm, struct g5p_drm, drm);
}

static inline struct g5p_bo *to_g5p_bo(struct drm_gem_object *obj)
{
	return container_of(obj, struct g5p_bo, shmem.base);
}

#include "vm-ranges.h"

static int g5p_resident_idle(struct g5p_context *c, bool live);
static bool g5p_resident_defer(struct g5p_context *c, struct g5p_mapping *m);
static int g5p_resident_invalidate(struct g5p_context *c);
static void g5p_render_release(struct g5p_context *c);
static int g5p_render_destroy(struct g5p_context *c);
static void g5p_render_close(struct g5p_context *c);
static int g5p_render_create_context(struct drm_device *drm,
				     struct drm_file *file,
				     struct drm_g5p_create_context *a);
static struct dma_buf *g5p_gem_prime_export(struct drm_gem_object *obj,
					    int flags);

#include "drm-vm.inc"
#include "drm-submit.inc"
#include "drm-render.inc"
#include "drm-prime.inc"
#include "drm-compute-v1.inc"

static int g5p_get_info(struct drm_device *drm, void *data,
			struct drm_file *file)
{
	struct drm_g5p_get_info *a = data;
	u64 caps = DRM_G5P_CAP_BO_CPU | DRM_G5P_CAP_VM_PREPARE |
		   DRM_G5P_CAP_SYNCOBJ | DRM_G5P_CAP_JOB_STATUS |
		   DRM_G5P_CAP_CONTEXT_STATUS | DRM_G5P_CAP_FAULT_INFO;

	if (a->size != sizeof(*a) || a->flags || a->reserved[0] ||
	    a->reserved[1])
		return -EINVAL;
	;
	;
	;
	;
	memset(a, 0, sizeof(*a));
	a->size = sizeof(*a);
	a->version = DRM_G5P_UAPI_VERSION;
	a->gpu_page_shift = 14;
	;
	;
	;
	;
	;
	{
		caps |= DRM_G5P_CAP_RENDER | DRM_G5P_CAP_BOUNDED_RENDER |
			DRM_G5P_CAP_TYPED_RENDER | DRM_G5P_CAP_VM_LIVE |
			DRM_G5P_CAP_CONTEXT_LIVE |
			DRM_G5P_CAP_CODE_BASE_VERIFIED;
		a->code_base = 0x300000000ULL;
	};
	;
	{
		caps |= DRM_G5P_CAP_RENDER_KERNEL_GEOMETRY;
	};
	if ((1) && g5p_render_msaa_enabled())
		caps |= DRM_G5P_CAP_RENDER_MSAA;
	;
	;
	{
		caps |= DRM_G5P_CAP_TYPED_RENDER_V3 |
			DRM_G5P_CAP_RENDER_MULTI_CONTEXT |
			DRM_G5P_CAP_RENDER_PIXEL_OUTPUT_WIDTH;
	};
	{
		caps |= DRM_G5P_CAP_RENDER_UNPRIVILEGED;
	};
	;
	;
	caps |= DRM_G5P_CAP_PRIME;
	caps |= DRM_G5P_CAP_TYPED_COMPUTE;
	a->capabilities = caps;
	a->va_start = G5P_USER_VA_START;
	a->va_end = G5P_USER_VA_END;
	a->reserved_start = G5P_USER_RESERVED_START;
	a->reserved_end = G5P_USER_RESERVED_END;
	a->max_bo_size = G5P_MAX_BO_SIZE;
	a->bvnc[0] = 9;
	a->bvnc[1] = 15;
	a->bvnc[2] = 6;
	a->bvnc[3] = 2;
	return 0;
}

/* The gated dialect uses the common B proven by the r19c numerical store. */
static int g5p_get_exec_info(struct drm_device *drm, void *data,
			     struct drm_file *file)
{
	struct drm_g5p_get_exec_info *a = data;

	if (a->size != sizeof(*a) || a->flags || a->pad ||
	    memchr_inv(a->reserved, 0, sizeof(a->reserved)))
		return -EINVAL;
	memset(a, 0, sizeof(*a));
	a->size = sizeof(*a);
	{
		a->base_scope = DRM_G5P_BASE_VM;
		a->usc_base = 0x300000000ULL;
		a->pds_base = a->usc_base;
		a->window_size = 0x34000;
		a->allocation_alignment = G5P_GPU_PAGE;
		a->usc_offset_bits = 32;
		a->pds_offset_bits = 32;
	}
	return 0;
}

static int g5p_get_job_status(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct drm_g5p_get_job_status *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_drm *d = to_g5p(drm);
	unsigned long flags;
	bool owned;

	if (a->flags || a->pad || !a->job_id || a->reset_generation)
		return -EINVAL;
	mutex_lock(&f->lock);
	if (xa_load(&f->results, a->job_id)) {
		struct g5p_fence *fence = xa_load(&f->results, a->job_id);

		spin_lock_irqsave(&d->status_lock, flags);
		a->state = fence->state;
		a->error = fence->error;
		spin_unlock_irqrestore(&d->status_lock, flags);
		mutex_unlock(&f->lock);
		return 0;
	}
	owned = f->job_id == a->job_id;
	mutex_unlock(&f->lock);
	if (!owned)
		return -ENOENT;
	spin_lock_irqsave(&d->status_lock, flags);
	a->state = d->job_state;
	a->error = d->job_error;
	/* Owner acknowledges terminal status before another fixed submission.
	 * This avoids waking a fence waiter before the device admission reopens.
	 */
	;
	spin_unlock_irqrestore(&d->status_lock, flags);
	return 0;
}

/* c99788 classifies event4/7/8. Preserve opaque fields, never invent a VA. */
static int g5p_get_fault_info(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct drm_g5p_get_fault_info *a = data;
	struct g5p_drm *d = to_g5p(drm);
	unsigned long flags;

	if (a->flags || a->reserved)
		return -EINVAL;
	memset(a, 0, sizeof(*a));
	spin_lock_irqsave(&d->status_lock, flags);
	a->event_serial = d->fault_serial;
	memcpy(a->raw, d->first_fault, sizeof(a->raw));
	spin_unlock_irqrestore(&d->status_lock, flags);
	if (!a->event_serial)
		return 0;
	switch (a->raw[0]) {
	case 4:
		a->kind = DRM_G5P_FAULT_RECOVERY;
		break;
	case 7:
		a->kind = DRM_G5P_FAULT_PAGE_REQUEST;
		break;
	case 8:
		a->kind = DRM_G5P_FAULT_CHANNEL;
		a->valid = DRM_G5P_FAULT_VALID_CHANNEL;
		a->channel_class = a->raw[1];
		a->channel_token = a->raw[3];
		a->channel_stamp = a->raw[4];
		break;
	default:
		a->kind = DRM_G5P_FAULT_UNKNOWN;
		break;
	}
	return 0;
}

static int g5p_get_context_status(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	struct drm_g5p_get_context_status *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_boot *b = READ_ONCE(to_g5p(drm)->core->boot);
	struct g5p_context *c;
	bool failed;
	int ret = 0;

	if (a->flags || a->state || a->map_count || a->mapped_bytes ||
	    a->reserved)
		return -EINVAL;
	mutex_lock(&f->lock);
	c = xa_load(&f->contexts, a->context);
	if (!c) {
		ret = -ENOENT;
		goto out;
	}
	a->state = DRM_G5P_CONTEXT_HOST_ONLY;
	if (READ_ONCE(c->live))
		a->state = DRM_G5P_CONTEXT_LIVE;
	/* a failure noted mid-job is not this context's until the executor rules */
	failed = b && READ_ONCE(b->failed) &&
		 !g5p_rdev_ruling(to_g5p(drm)->rdev);
	if (READ_ONCE(c->bad) || failed || (b && READ_ONCE(b->stopped)))
		a->state = DRM_G5P_CONTEXT_LOST;
	a->map_count = c->vm->map_count;
	a->mapped_bytes = c->vm->mapped_bytes;
out:
	mutex_unlock(&f->lock);
	return ret;
}

static void g5p_drm_event(void *cookie, const u32 words[11])
{
	struct g5p_drm *d = cookie;
	unsigned long flags;

	spin_lock_irqsave(&d->status_lock, flags);
	memcpy(d->last_event, words, sizeof(d->last_event));
	d->event_serial++;
	if (words[0] == 1)
		for (u32 i = 0; i < 4; i++)
			d->notifications[i] |= words[i + 1];
	if (d->td_want && !d->td_hit_ns &&
	    g5p_ru_rt_event_match(words, d->td_want & 0xff))
		WRITE_ONCE(d->td_hit_ns, ktime_get_ns());
	if (d->ta_want && !d->ta_hit_ns && g5p_ru_ta_event_match(words))
		WRITE_ONCE(d->ta_hit_ns, ktime_get_ns());
	for (u32 e = 0; words[0] == 1 && e < 2; e++) {
		if (!(words[1] & BIT(e)))
			continue;
		WRITE_ONCE(d->t1_count[e], d->t1_count[e] + 1);
		if (!d->t1_first_ns[e])
			WRITE_ONCE(d->t1_first_ns[e], ktime_get_ns());
	}
	if (words[0] != 0 && words[0] != 1) {
		d->last_error = -EIO;
		if (!d->fault_serial) {
			d->fault_serial = d->event_serial;
			memcpy(d->first_fault, words, sizeof(d->first_fault));
		}
	}
	spin_unlock_irqrestore(&d->status_lock, flags);
	wake_up(&d->ev_wq);
}

static int g5p_drm_open(struct drm_device *drm, struct drm_file *file)
{
	struct g5p_file *f = kzalloc(sizeof(*f), GFP_KERNEL);

	if (!f)
		return -ENOMEM;
	mutex_init(&f->lock);
	xa_init_flags(&f->vms, XA_FLAGS_ALLOC1);
	xa_init_flags(&f->contexts, XA_FLAGS_ALLOC1);
	xa_init(&f->results);
	xa_init_flags(&f->targets, XA_FLAGS_ALLOC1);
	file->driver_priv = f;
	atomic_inc(&to_g5p(drm)->open_files);
	return 0;
}

static void g5p_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
	struct g5p_file *f = file->driver_priv;
	struct g5p_context *c;
	struct g5p_vm *vm;
	unsigned long id;
	struct g5p_rt *rt;

	atomic_dec(&to_g5p(drm)->open_files);
	/* Render-target handles hold their context; drop them first. */
	xa_for_each(&f->targets, id, rt)
		g5p_context_put(rt->ctx);
	xa_destroy(&f->targets);
	/* Jobs hold contexts, contexts hold VMs, mappings hold GEM objects.
	 * Closing the fd drops handle ownership, never a queued job's references.
	 * Host-only tables may retire at job completion: no live GPU attachment.
	 */
	xa_for_each(&f->contexts, id, c) {
		/* A render context's firmware binding outlives the handle until its close
		 * item, queued behind its jobs, has unbound it.
		 */
		if (c->render)
			g5p_render_close(c);
		g5p_context_put(c);
	}
	xa_destroy(&f->contexts);
	{
		struct g5p_fence *fence;

		xa_for_each(&f->results, id, fence)
			dma_fence_put(&fence->base);
		xa_destroy(&f->results);
	}
	xa_for_each(&f->vms, id, vm)
		g5p_vm_put(vm);
	xa_destroy(&f->vms);
	mutex_destroy(&f->lock);
	kfree(f);
	/* The file's handles are gone (drm_gem_release() ran first): what stays live is held
	 * by another file's handle, a dma-buf, a VM or a job (debugfs g5p_bos names it).
	 */
	dev_info(drm->dev,
		"G5P file close live_bos=%u live_bo_bytes=%lluK device_pinned=%lluK\n",
		READ_ONCE(g5p_bo_count), READ_ONCE(g5p_bo_bytes) >> 10,
		READ_ONCE(to_g5p(drm)->pinned_bytes) >> 10);
}

static const struct drm_ioctl_desc g5p_ioctls[] = {
	DRM_IOCTL_DEF_DRV(G5P_GET_INFO, g5p_get_info, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_CREATE_BO, g5p_create_bo, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_BO_MMAP_OFFSET, g5p_bo_mmap_offset,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_BO_SYNC, g5p_bo_sync, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_CREATE_VM, g5p_create_vm, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_DESTROY_VM, g5p_destroy_vm, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_VM_BIND, g5p_vm_bind, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_CREATE_CONTEXT, g5p_create_context,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_DESTROY_CONTEXT, g5p_destroy_context,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_GET_EXEC_INFO, g5p_get_exec_info,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_GET_JOB_STATUS, g5p_get_job_status,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_GET_CONTEXT_STATUS, g5p_get_context_status,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_GET_FAULT_INFO, g5p_get_fault_info,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_CREATE_RENDER_TARGET, g5p_create_render_target,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_DESTROY_RENDER_TARGET, g5p_destroy_render_target,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_GET_RENDER_INFO, g5p_get_render_info,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_SUBMIT_RENDER_TYPED, g5p_submit_render_typed,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(G5P_SUBMIT_COMPUTE_TYPED, g5p_submit_compute_typed,
			  DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_FOPS(g5p_drm_fops);

static const struct drm_driver g5p_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER | DRIVER_SYNCOBJ |
			   DRIVER_SYNCOBJ_TIMELINE,
	.open = g5p_drm_open,
	.postclose = g5p_drm_postclose,
	.fops = &g5p_drm_fops,
	.ioctls = g5p_ioctls,
	.num_ioctls = ARRAY_SIZE(g5p_ioctls),
	.gem_create_object = g5p_gem_create_object,
	.prime_handle_to_fd = g5p_prime_handle_to_fd,
	.prime_fd_to_handle = g5p_prime_fd_to_handle,
	.gem_prime_import = g5p_gem_prime_import,
	.gem_prime_import_sg_table = g5p_prime_import_sg_table,
	.name = "apple_g5p",
	.desc = "Apple A9 G5P experimental render driver",
	.major = 0,
	.minor = DRM_G5P_UAPI_VERSION,
};

/* recovery108: the frontend's epoch inputs for the retire layer.  RAM only and lock free:
 * the claimed context is compared, never dereferenced (it may be dropped concurrently).
 */
static void g5p_drm_r8_epoch(void *cookie, struct g5p_r8_in *in)
{
	struct g5p_drm *d = cookie;
	int queued = atomic_read(&d->r8_queued);

	in->claimed = READ_ONCE(d->render_claimed) ||
		      READ_ONCE(d->quarantined_context);
	in->held = in->claimed && atomic_read(&d->open_files) > 0;
	in->queued = queued > 0 ? queued : 0;
}

/* The teardown DETACH phase.  No fence is signalled here: with admission closed and the
 * device frozen, the render scheduler fails each queued job exactly once (-ESHUTDOWN,
 * without waiting for its inputs), and an in-flight job ends on its own RAM-only
 * deadline.  The count left to the workers is reported as the explicit abandonment.
 */
static void g5p_drm_td_detach(void *cookie)
{
	struct g5p_drm *d = cookie;
	int queued = atomic_read(&d->r8_queued);

	g5p_r8_note_detach(&g5p_r8_state, queued > 0 ? queued : 0);
	g5p_rsched_kick(d);
}

static int g5p_drm_probe(struct platform_device *pdev)
{
	struct g5p_drm *d;
	struct g5p_diag *g;
	int ret;

	;
	;
	ret = g5p_probe(pdev);
	if (ret)
		return ret;
	g = platform_get_drvdata(pdev);
	d = devm_drm_dev_alloc(&pdev->dev, &g5p_drm_driver, struct g5p_drm,
			       drm);
	if (IS_ERR(d)) {
		ret = PTR_ERR(d);
		goto err_pm;
	}
	d->core = g;
	d->fence_context = dma_fence_context_alloc(1);
	spin_lock_init(&d->status_lock);
	init_waitqueue_head(&d->ev_wq);
	atomic_set(&d->accepted, 0);
	atomic64_set(&d->job_sequence, 0);
	d->render_wq = drmm_alloc_ordered_workqueue(&d->drm, "g5p-render", 0);
	if (IS_ERR_OR_NULL(d->render_wq)) {
		ret = d->render_wq ? PTR_ERR(d->render_wq) : -ENOMEM;
		goto err_pm;
	}
	ret = drmm_mutex_init(&d->drm, &d->render_submit_lock);
	if (!ret)
		ret = drmm_mutex_init(&d->drm, &d->pin_lock);
	if (ret)
		goto err_pm;
	ret = g5p_render_init(d);
	if (ret)
		goto err_pm;
	g->event_cookie = d;
	g->event_notify = g5p_drm_event;
	atomic_set(&d->open_files, 0);
	atomic_set(&d->r8_queued, 0);
	drm_debugfs_add_file(&d->drm, "g5p_bos", g5p_bos_show, NULL);
	drm_debugfs_add_file(&d->drm, "g5p_cache_receipts", g5p_cache_show, NULL);
	ret = drm_dev_register(&d->drm, 0);
	if (ret)
		goto err_pm;
	g5p_r8_register_epoch(g5p_drm_r8_epoch, d);
	g5p_td_register_detach(g5p_drm_td_detach, d);
	g5p_kq_register(&g5p_kq_drm, d);
	dev_info(&pdev->dev,
		"render node registered; no GPU access, COMPUTE/RENDER capabilities absent\n");
	return 0;
err_pm:
	g->event_notify = NULL;
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void g5p_drm_remove(struct platform_device *pdev)
{
	struct g5p_diag *g = platform_get_drvdata(pdev);
	struct g5p_drm *d = g->event_cookie;

	drm_dev_unplug(&d->drm);
	g5p_remove(pdev);
	/* recovery108: the remove-time FINAL above was the callbacks' last use; d is
	 * devres-freed next.
	 */
	g5p_td_register_detach(NULL, NULL);
	g5p_r8_register_epoch(NULL, NULL);
	g5p_kq_register(NULL, NULL);
}

static const struct of_device_id g5p_drm_match[] = {
	{ .compatible = "gpu,s8003" },
	{},
};
MODULE_DEVICE_TABLE(of, g5p_drm_match);

static struct platform_driver g5p_drm_platform = {
	.probe = g5p_drm_probe,
	.remove = g5p_drm_remove,
	.shutdown = g5p_shutdown,
	.driver = {
		.name = "apple-g5p-drm",
		.of_match_table = g5p_drm_match,
		.dev_groups = g5p_groups,
		.suppress_bind_attrs = true,
	},
};

static bool enable;
static bool drm_initialized;
static bool drm_registered;

static int g5p_drm_enable_set(const char *value, const struct kernel_param *kp)
{
	bool requested;
	int ret = kstrtobool(value, &requested);

	if (ret)
		return ret;
	if (!drm_initialized) {
		enable = requested;
		return 0;
	}
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (!requested && drm_registered)
		return -EBUSY;
	if (requested && !drm_registered) {
		ret = g5p_frontend_available();
		if (ret)
			return ret;
		ret = platform_driver_register(&g5p_drm_platform);
		if (ret)
			return ret;
		drm_registered = true;
	}
	enable = requested;
	return 0;
}

static const struct kernel_param_ops g5p_drm_enable_ops = {
	.set = g5p_drm_enable_set,
	.get = param_get_bool,
};
module_param_cb(enable, &g5p_drm_enable_ops, &enable, 0600);
MODULE_PARM_DESC(enable,
	"Explicitly register G5P DRM frontend; conflicts with diagnostic binding, default false");

static int __init g5p_drm_init(void)
{
	drm_initialized = true;
	if (!enable) {
		pr_info("apple-g5p-drm: inert; explicit a9_gpu_drm.enable=1 required\n");
		return 0;
	}
	return g5p_drm_enable_set("1", NULL);
}
module_init(g5p_drm_init);

/* Built-in only until reset can prove all DMA references are quiescent. */
MODULE_DESCRIPTION("Default-disabled Apple G5P render/GEM/VM experimental driver");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_FIRMWARE(G5P_FW_NAME);
