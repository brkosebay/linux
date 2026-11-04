/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* 16 KiB VM preparation; accepted user VMs freeze until reboot. */

/* release8 memory accounting: every live G5P GEM object (native and imported) with the task
 * that created it.  One line per context release and per file close prints the totals;
 * debugfs g5p_bos lists each object (size, references, handles, exported/imported, VM pins,
 * creator) and each open file's handles, so a page held after its client exited is named.
 */
static LIST_HEAD(g5p_bo_live);
static DEFINE_SPINLOCK(g5p_bo_lock);
static u32 g5p_bo_count;
static u64 g5p_bo_bytes;

static void g5p_gem_free(struct drm_gem_object *obj);

static const struct drm_gem_object_funcs g5p_gem_funcs = {
	.free = g5p_gem_free,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.export = g5p_gem_prime_export,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

/* perf60 fault address (render_user_fault_read, drm-render-v3.inc): bit 0 reads the MMU fault
 * banks at a TA/3D fault event, bit 1 also at a silent or timed-out phase; any nonzero value also
 * records every VM edit (MAP, UNMAP, render-target create/destroy) in a device-wide ring, so the
 * fault address can be named.  The ring holds an opaque VM pointer, never dereferenced.
 */

/* faultrec: the bank read and its decode alone at a TA/3D fault event (no per-job PM lanes, no
 * PM snapshots, no VM edit log): one licensed read per fault, at most 32 per boot.
 */

/* perf60-fillrate.md §11.3 cut 5, kernel half (render-user-sync.h): cache maintenance by range. */
static unsigned int render_user_sync_range = 3; /* release8 default */
module_param(render_user_sync_range, uint, 0600);
MODULE_PARM_DESC(
	render_user_sync_range,
	"perf60 default0 (release8: whole-BO maintenance); bit0 BO_SYNC honours "
	"offset/size; bit1 a job's ACCESS_CACHE clean/invalidate covers each resource's "
	"BO range [offset, offset+size), page widened, instead of the whole BO (latched "
	"per job at submit); BO_SYNC read live");

/* CPU cache maintenance of [lo, hi) of a BO's DMA mapping, walked over its DMA segments. */
static void g5p_sync_sgt_range(struct device *dev, struct sg_table *sgt, u64 lo,
			       u64 hi, bool to_cpu)
{
	struct scatterlist *sg;
	u64 pos = 0, start, n;
	unsigned int i;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (pos >= hi)
			break;
		if (g5p_sync_clip(pos, sg_dma_len(sg), lo, hi, &start, &n)) {
			if (to_cpu)
				dma_sync_single_for_cpu(
					dev, sg_dma_address(sg) + start, n,
					DMA_BIDIRECTIONAL);
			else
				dma_sync_single_for_device(
					dev, sg_dma_address(sg) + start, n,
					DMA_BIDIRECTIONAL);
		}
		pos += sg_dma_len(sg);
	}
}

#define G5P_VMLOG_N 1024U
enum { G5P_VMLOG_MAP = 1, G5P_VMLOG_UNMAP, G5P_VMLOG_RT_NEW, G5P_VMLOG_RT_DEL };
static const char *const g5p_vmlog_ops[] = { "?", "map", "unmap", "rt-new",
					     "rt-del" };

struct g5p_vmlog_ent {
	u64 ns, va, size, offset;
	const void *vm;
	u32 op, handle, vm_id;
	pid_t tgid;
	char comm[TASK_COMM_LEN];
};

static struct {
	spinlock_t lock; /* head and the entries */
	u32 head;
	struct g5p_vmlog_ent e[G5P_VMLOG_N];
} g5p_vmlog = { .lock = __SPIN_LOCK_UNLOCKED(g5p_vmlog.lock) };

static void g5p_vmlog_add(u32 op, const void *vm, u32 vm_id, u64 va, u64 size,
			  u32 handle, u64 offset)
{
	{
		return;
	}
}

/* Latency123: leaf attributes of new user mappings (0 = Apple's CC | SLC bypass). */

/* Stage 3: the device-wide pinned-memory budget. */
static unsigned int render_pinned_mib = 1024;
module_param(render_pinned_mib, uint, 0600);
MODULE_PARM_DESC(
	render_pinned_mib,
	"MiB of BO memory all GPU VMs together may map; an object mapped in several VMs "
	"or files (PRIME) counts once (default 1024)");

static u32 g5p_pte_attrs(void)
{
	static const u32 attrs[] = { 0xc, 0x4, 0x8, 0x0 };

	return attrs[min_t(u32, (0), ARRAY_SIZE(attrs) - 1)];
}

static struct drm_gem_object *g5p_gem_create_object(struct drm_device *drm,
						    size_t size)
{
	struct g5p_bo *bo = kzalloc(sizeof(*bo), GFP_KERNEL);

	if (!bo)
		return ERR_PTR(-ENOMEM);
	bo->shmem.base.funcs = &g5p_gem_funcs;
	INIT_LIST_HEAD(&bo->live);
	return &bo->shmem.base;
}

/* A fully created object enters the live list (an object the core frees on a failed init
 * never does).
 */
static void g5p_bo_track(struct drm_gem_object *obj)
{
	struct g5p_bo *bo = to_g5p_bo(obj);
	unsigned long flags;

	bo->pid = current->tgid;
	strscpy(bo->comm, current->comm, sizeof(bo->comm));
	spin_lock_irqsave(&g5p_bo_lock, flags);
	list_add_tail(&bo->live, &g5p_bo_live);
	g5p_bo_count++;
	g5p_bo_bytes += obj->size;
	spin_unlock_irqrestore(&g5p_bo_lock, flags);
}

static void g5p_gem_free(struct drm_gem_object *obj)
{
	struct g5p_bo *bo = to_g5p_bo(obj);
	unsigned long flags;

	spin_lock_irqsave(&g5p_bo_lock, flags);
	if (!list_empty(&bo->live)) {
		list_del_init(&bo->live);
		g5p_bo_count--;
		g5p_bo_bytes -= obj->size;
	}
	spin_unlock_irqrestore(&g5p_bo_lock, flags);
	drm_gem_shmem_object_free(obj);
}

static int g5p_bos_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct drm_device *drm = entry->dev;
	u32 exported = 0, imported = 0, nohandle = 0;
	u64 nohandle_bytes = 0;
	struct drm_file *file;
	struct g5p_bo *bo;
	unsigned long flags;

	spin_lock_irqsave(&g5p_bo_lock, flags);
	list_for_each_entry(bo, &g5p_bo_live, live) {
		struct drm_gem_object *obj = &bo->shmem.base;
		bool imp = drm_gem_is_imported(obj);

		exported += !!READ_ONCE(obj->dma_buf);
		imported += imp;
		if (!READ_ONCE(obj->handle_count)) {
			nohandle++;
			nohandle_bytes += obj->size;
		}
		seq_printf(
			m,
			"bo size=%zuK refs=%u handles=%u exported=%c imported=%c vm_pins=%u by=%s[%d]\n",
			obj->size >> 10, kref_read(&obj->refcount),
			READ_ONCE(obj->handle_count),
			READ_ONCE(obj->dma_buf) ? 'y' : 'n', imp ? 'y' : 'n',
			bo->vm_pins, bo->comm, bo->pid);
	}
	seq_printf(
		m,
		"live=%u bytes=%lluK exported=%u imported=%u no_handle=%u/%lluK device_pinned=%lluK\n",
		g5p_bo_count, g5p_bo_bytes >> 10, exported, imported, nohandle,
		nohandle_bytes >> 10,
		READ_ONCE(to_g5p(drm)->pinned_bytes) >> 10);
	spin_unlock_irqrestore(&g5p_bo_lock, flags);
	mutex_lock(&drm->filelist_mutex);
	list_for_each_entry(file, &drm->filelist, lhead) {
		struct drm_gem_object *obj;
		struct task_struct *task;
		u64 bytes = 0;
		u32 handles = 0;
		struct pid *pid;
		int id;

		spin_lock(&file->table_lock);
		idr_for_each_entry(&file->object_idr, obj, id) {
			handles++;
			bytes += obj->size;
		}
		spin_unlock(&file->table_lock);
		rcu_read_lock();
		pid = rcu_dereference(file->pid);
		task = pid_task(pid, PIDTYPE_TGID);
		seq_printf(m, "file %s[%d] handles=%u bytes=%lluK\n",
			   task ? task->comm : "?", pid_vnr(pid), handles,
			   bytes >> 10);
		rcu_read_unlock();
	}
	mutex_unlock(&drm->filelist_mutex);
	return 0;
}

static int g5p_create_bo(struct drm_device *drm, void *data,
			 struct drm_file *file)
{
	struct drm_g5p_create_bo *a = data;
	struct drm_gem_shmem_object *shmem;
	int ret;

	if (!a->size || a->size > G5P_MAX_BO_SIZE ||
	    (a->size & (G5P_GPU_PAGE - 1)) || a->handle || !a->flags ||
	    (a->flags &
	     ~(DRM_G5P_BO_READ | DRM_G5P_BO_WRITE | DRM_G5P_BO_EXEC)))
		return -EINVAL;
	shmem = drm_gem_shmem_create(drm, a->size);
	if (IS_ERR(shmem))
		return PTR_ERR(shmem);
	/* Match the DMA API's coherent/noncoherent CPU mapping choice. The GPU
	 * cache contract remains an explicit COMPUTE capability prerequisite.
	 */
	shmem->map_wc = !dev_is_dma_coherent(drm->dev);
	to_g5p_bo(&shmem->base)->roles = a->flags;
	g5p_bo_track(&shmem->base);
	ret = drm_gem_handle_create(file, &shmem->base, &a->handle);
	drm_gem_object_put(&shmem->base);
	return ret;
}

static int g5p_bo_mmap_offset(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct drm_g5p_bo_mmap_offset *a = data;
	struct drm_gem_object *obj;
	int ret;

	if (a->flags || a->offset)
		return -EINVAL;
	obj = drm_gem_object_lookup(file, a->handle);
	if (!obj)
		return -ENOENT;
	ret = drm_gem_create_mmap_offset(obj);
	if (!ret)
		a->offset = drm_vma_node_offset_addr(&obj->vma_node);
	drm_gem_object_put(obj);
	return ret;
}

static int g5p_bo_sync(struct drm_device *drm, void *data,
		       struct drm_file *file)
{
	struct drm_g5p_bo_sync *a = data;
	struct drm_gem_object *obj;
	struct sg_table *sgt;
	struct g5p_file *f = file->driver_priv;
	struct g5p_context *ctx;
	unsigned long id;
	int ret = 0;

	if (a->op != DRM_G5P_BO_SYNC_TO_DEVICE &&
	    a->op != DRM_G5P_BO_SYNC_TO_CPU)
		return -EINVAL;
	obj = drm_gem_object_lookup(file, a->handle);
	if (!obj)
		return -ENOENT;
	if (!g5p_bo_range_valid(a->offset, a->size, obj->size)) {
		ret = -EINVAL;
		goto out;
	}
	mutex_lock(&f->lock);
	xa_for_each(&f->contexts, id, ctx)
		if (ctx->serial &&
		    (ctx->bad || smp_load_acquire(&ctx->pending))) {
			mutex_unlock(&f->lock);
			ret = -EBUSY;
			goto out;
		}
	sgt = drm_gem_shmem_get_pages_sgt(to_drm_gem_shmem_obj(obj));
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		mutex_unlock(&f->lock);
		goto out;
	}
	/* render_user_sync_range bit 0: the requested range; else the whole BO (stronger, allowed). */
	if (READ_ONCE(render_user_sync_range) & 1) {
		u64 lo, hi;

		if (g5p_sync_window(a->offset, a->size, obj->size, &lo, &hi))
			g5p_sync_sgt_range(drm->dev, sgt, lo, hi,
					   a->op == DRM_G5P_BO_SYNC_TO_CPU);
	} else if (a->op == DRM_G5P_BO_SYNC_TO_DEVICE) {
		dma_sync_sgtable_for_device(drm->dev, sgt, DMA_BIDIRECTIONAL);
	} else {
		dma_sync_sgtable_for_cpu(drm->dev, sgt, DMA_BIDIRECTIONAL);
	}
	mutex_unlock(&f->lock);
out:
	drm_gem_object_put(obj);
	return ret;
}

static int g5p_vm_alloc_page(struct device *dev, struct g5p_dma_page *p)
{
	p->cpu = dma_alloc_coherent(dev, G5P_GPU_PAGE, &p->dma, GFP_KERNEL);
	if (!p->cpu)
		return -ENOMEM;
	if (!g5p_user_pa_valid(p->dma) || dma_to_phys(dev, p->dma) != p->dma) {
		dma_free_coherent(dev, G5P_GPU_PAGE, p->cpu, p->dma);
		p->cpu = NULL;
		return -ERANGE;
	}
	memset(p->cpu, 0, G5P_GPU_PAGE);
	return 0;
}

static struct g5p_pt *g5p_vm_table(struct g5p_vm *vm, struct device *dev,
				   u32 level, u32 key, bool create)
{
	struct g5p_pt *pt;
	int ret;

	list_for_each_entry(pt, &vm->tables, link)
		if (pt->level == level && pt->key == key)
			return pt;
	if (!create)
		return NULL;
	if (vm->table_count >= G5P_MAX_PT_PAGES)
		return ERR_PTR(-ENOSPC);
	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt)
		return ERR_PTR(-ENOMEM);
	ret = g5p_vm_alloc_page(dev, &pt->page);
	if (ret) {
		kfree(pt);
		return ERR_PTR(ret);
	}
	pt->level = level;
	pt->key = key;
	list_add_tail(&pt->link, &vm->tables);
	vm->table_count++;
	return pt;
}

static u8 *g5p_vm_pte(struct g5p_vm *vm, struct device *dev, u64 va,
		      bool create)
{
	u32 ci = (va >> 30) & 1023, di = (va >> 21) & 511;
	struct g5p_pt *dir, *leaf;

	dir = g5p_vm_table(vm, dev, 1, ci, create);
	if (IS_ERR_OR_NULL(dir))
		return (void *)dir;
	leaf = g5p_vm_table(vm, dev, 2, (ci << 9) | di, create);
	if (IS_ERR_OR_NULL(leaf))
		return (void *)leaf;
	if (create) {
		g5p_vm_entry(&vm->mutation, vm->root.cpu + ci * 4,
			     g5p_catalogue_entry(dir->page.dma), 0);
		g5p_vm_entry(&vm->mutation, dir->page.cpu + di * 8,
			     g5p_directory_entry(leaf->page.dma), 1);
	}
	return leaf->page.cpu + ((va >> 14) & 127) * 8;
}

static void g5p_vm_clear(struct g5p_vm *vm, struct device *dev, u64 va,
			 u64 size)
{
	for (u64 off = 0; off < size; off += G5P_GPU_PAGE) {
		u8 *pte = g5p_vm_pte(vm, dev, va + off, false);

		if (!IS_ERR_OR_NULL(pte))
			g5p_vm_entry(&vm->mutation, pte, 0, 2);
	}
	dma_wmb();
}

static bool g5p_vm_table_empty(struct g5p_pt *pt)
{
	u32 count = pt->level == 2 ? 128 : 512;

	for (u32 i = 0; i < count; i++)
		if (g5p_le32(pt->page.cpu + i * 8) & 1)
			return false;
	return true;
}

static void g5p_vm_prune(struct device *dev, struct g5p_vm *vm)
{
	struct g5p_pt *pt, *tmp, *dir;

	/* These tables are mutable only before firmware attachment.
	 * Remove empty leaves before directories, including failed MAP rollback.
	 */
	for (u32 level = 2; level; level--)
		list_for_each_entry_safe(pt, tmp, &vm->tables, link) {
			if (pt->level != level || !g5p_vm_table_empty(pt))
				continue;
			if (level == 2) {
				dir = g5p_vm_table(vm, dev, 1, pt->key >> 9,
						   false);
				if (dir)
					g5p_vm_entry(&vm->mutation,
						     dir->page.cpu + (pt->key &
								      511) * 8,
						     0, 1);
			} else {
				g5p_vm_entry(&vm->mutation,
					     vm->root.cpu + pt->key * 4, 0, 0);
			}
			dma_wmb();
			list_del(&pt->link);
			dma_free_coherent(dev, G5P_GPU_PAGE, pt->page.cpu,
					  pt->page.dma);
			vm->table_count--;
			kfree(pt);
		}
}

static void g5p_vm_obj_put(struct g5p_vm *vm, struct drm_gem_object *obj);

static void g5p_vm_release(struct kref *ref)
{
	struct g5p_vm *vm = container_of(ref, struct g5p_vm, ref);
	struct device *dev = vm->dev;
	struct g5p_mapping *m, *mtmp;
	struct g5p_pt *pt, *ptmp;
	u64 pinned = vm->pinned_bytes;
	u32 maps = vm->map_count, tables = vm->table_count;

	list_for_each_entry_safe(m, mtmp, &vm->maps, link) {
		drm_mm_remove_node(&m->node);
		g5p_vm_obj_put(vm, m->obj);
		drm_gem_object_put(m->obj);
		list_del(&m->link);
		kfree(m);
	}
	list_for_each_entry_safe(pt, ptmp, &vm->tables, link) {
		dma_free_coherent(dev, G5P_GPU_PAGE, pt->page.cpu,
				  pt->page.dma);
		list_del(&pt->link);
		kfree(pt);
	}
	dma_free_coherent(dev, G5P_GPU_PAGE, vm->root.cpu, vm->root.dma);
	xa_destroy(&vm->objs);
	drm_mm_remove_node(&vm->reserved);
	drm_mm_takedown(&vm->mm);
	/* One line per freed VM: the device's pinned total must fall back after a client exits. */
	dev_info(
		dev,
		"G5P VM release maps=%u tables=%u pinned=%lluK device_pinned=%lluK\n",
		maps, tables, pinned >> 10,
		READ_ONCE(vm->gpu->pinned_bytes) >> 10);
	kfree(vm);
}

static void g5p_vm_put(struct g5p_vm *vm)
{
	kref_put(&vm->ref, g5p_vm_release);
}

static void g5p_context_release(struct kref *ref)
{
	struct g5p_context *c = container_of(ref, struct g5p_context, ref);
	g5p_render_release(c);
	g5p_vm_put(c->vm);
	kfree(c);
}

static void g5p_context_put(struct g5p_context *c)
{
	kref_put(&c->ref, g5p_context_release);
}

static int g5p_create_vm(struct drm_device *drm, void *data,
			 struct drm_file *file)
{
	struct drm_g5p_create_vm *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_vm *vm;
	u32 id;
	int ret;

	if (a->flags || a->handle)
		return -EINVAL;
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;
	kref_init(&vm->ref);
	vm->dev = drm->dev;
	vm->gpu = to_g5p(drm);
	vm->bounded = (0);
	vm->mutation.generations = true;
	INIT_LIST_HEAD(&vm->maps);
	xa_init(&vm->objs);
	INIT_LIST_HEAD(&vm->tables);
	drm_mm_init(&vm->mm, G5P_USER_VA_START,
		    G5P_USER_VA_END - G5P_USER_VA_START);
	vm->reserved.start = G5P_USER_RESERVED_START;
	vm->reserved.size = G5P_USER_RESERVED_END - G5P_USER_RESERVED_START;
	ret = drm_mm_reserve_node(&vm->mm, &vm->reserved);
	if (ret)
		goto err_mm;
	ret = g5p_vm_alloc_page(drm->dev, &vm->root);
	if (ret)
		goto err_reserved;
	vm->table_count = 1;
	mutex_lock(&f->lock);
	ret = xa_alloc(&f->vms, &id, vm, XA_LIMIT(1, G5P_MAX_VMS), GFP_KERNEL);
	mutex_unlock(&f->lock);
	if (ret) {
		g5p_vm_put(vm);
		return ret;
	}
	a->handle = id;
	return 0;
err_reserved:
	drm_mm_remove_node(&vm->reserved);
err_mm:
	drm_mm_takedown(&vm->mm);
	kfree(vm);
	return ret;
}

static int g5p_destroy_vm(struct drm_device *drm, void *data,
			  struct drm_file *file)
{
	struct drm_g5p_destroy_vm *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_vm *vm;
	int ret = 0;

	if (a->flags)
		return -EINVAL;
	mutex_lock(&f->lock);
	vm = xa_load(&f->vms, a->handle);
	if (!vm)
		ret = -ENOENT;
	else if (vm->contexts || f->submitted || vm->frozen)
		ret = -EBUSY;
	else
		xa_erase(&f->vms, a->handle);
	mutex_unlock(&f->lock);
	if (!ret)
		g5p_vm_put(vm);
	return ret;
}

static unsigned long g5p_vm_obj_count(struct g5p_vm *vm,
				      struct drm_gem_object *obj)
{
	void *v = xa_load(&vm->objs, (unsigned long)obj);

	return v ? xa_to_value(v) : 0;
}

/*
 * One more mapping of `obj` in `vm`.  Its first one in this VM counts its size against the
 * VM's pin limit, and its first one in any VM against the device budget (render_pinned_mib).
 * Our own dma-bufs import back as the same GEM object, in any file, so memory shared between
 * clients (a client's buffer in the compositor's VM) counts once.
 */
static int g5p_vm_obj_get(struct g5p_vm *vm, struct drm_gem_object *obj)
{
	struct g5p_drm *d = to_g5p(obj->dev);
	struct g5p_bo *bo = to_g5p_bo(obj);
	u64 limit = (u64)READ_ONCE(render_pinned_mib) << 20;
	unsigned long count;
	int ret = 0;

	mutex_lock(&d->pin_lock);
	count = g5p_vm_obj_count(vm, obj);
	if (!count && obj->size > G5P_MAX_PINNED_BYTES - vm->pinned_bytes)
		ret = -ENOSPC;
	else if (!count && !bo->vm_pins &&
		 obj->size > limit - min(limit, d->pinned_bytes))
		ret = -ENOSPC;
	if (!ret)
		ret = xa_err(xa_store(&vm->objs, (unsigned long)obj,
				      xa_mk_value(count + 1), GFP_KERNEL));
	if (!ret && !count) {
		vm->pinned_bytes += obj->size;
		if (!bo->vm_pins++)
			d->pinned_bytes += obj->size;
	}
	mutex_unlock(&d->pin_lock);
	return ret;
}

/* One mapping of `obj` in `vm` less: g5p_vm_obj_get() undone.  Also called by the render
 * scheduler for an unmap deferred behind queued jobs, hence the lock around the counts.
 */
static void g5p_vm_obj_put(struct g5p_vm *vm, struct drm_gem_object *obj)
{
	struct g5p_drm *d = to_g5p(obj->dev);
	struct g5p_bo *bo = to_g5p_bo(obj);
	unsigned long count;

	mutex_lock(&d->pin_lock);
	count = g5p_vm_obj_count(vm, obj);
	if (count > 1) {
		/* replaces an existing entry: no allocation */
		xa_store(&vm->objs, (unsigned long)obj, xa_mk_value(count - 1),
			 GFP_KERNEL);
	} else if (!WARN_ON(!count)) {
		xa_erase(&vm->objs, (unsigned long)obj);
		vm->pinned_bytes -= obj->size;
		if (!--bo->vm_pins)
			d->pinned_bytes -= obj->size;
	}
	mutex_unlock(&d->pin_lock);
}

static int g5p_vm_map_bo(struct drm_device *drm, struct g5p_vm *vm,
			 struct drm_file *file, struct drm_g5p_vm_bind *a)
{
	struct drm_gem_object *obj;
	struct drm_gem_shmem_object *shmem;
	struct sg_table *sgt;
	struct sg_dma_page_iter iter;
	struct g5p_mapping *m;
	u64 *addresses, done = 0;
	u32 count = 0;
	bool imported;
	int ret;

	if (vm->map_count >= G5P_MAX_MAPS ||
	    a->size > G5P_MAX_VM_BYTES - vm->mapped_bytes)
		return -ENOSPC;
	obj = drm_gem_object_lookup(file, a->bo);
	if (!obj)
		return -ENOENT;
	if (!g5p_bo_range_valid(a->offset, a->size, obj->size)) {
		ret = -EINVAL;
		goto err_obj;
	}
	ret = g5p_vm_obj_get(vm, obj);
	if (ret)
		goto err_obj;
	addresses = kvmalloc_array(obj->size / G5P_GPU_PAGE, sizeof(*addresses),
				   GFP_KERNEL);
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!addresses || !m) {
		ret = -ENOMEM;
		goto err_alloc;
	}
	m->node.start = a->va;
	m->node.size = a->size;
	ret = drm_mm_reserve_node(&vm->mm, &m->node);
	if (ret) {
		ret = -EINVAL;
		goto err_alloc;
	}
	imported = drm_gem_is_imported(obj);
	shmem = to_drm_gem_shmem_obj(obj);
	/* PRIME import mapped this attachment for our device and owns it until
	 * object release.  Re-locking the exporter's reservation here can return
	 * -EBUSY after the first V3 submission even though the render VM is idle.
	 */
	if (imported) {
		sgt = READ_ONCE(shmem->sgt);
		if (!sgt) {
			dev_warn_ratelimited(
				drm->dev,
				"G5P VM_BIND map=prime-no-sgt vm=%u bo=%u bytes=%llu\n",
				a->vm, a->bo, a->size);
			ret = -ENXIO;
			goto err_node;
		}
	} else {
		sgt = drm_gem_shmem_get_pages_sgt(shmem);
	}
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_warn_ratelimited(
			drm->dev,
			"G5P VM_BIND map=sgt-error vm=%u bo=%u imported=%u err=%d\n",
			a->vm, a->bo, imported, ret);
		goto err_node;
	}
	for_each_sgtable_dma_page(sgt, &iter, 0) {
		u64 dma = sg_page_iter_dma_address(&iter);

		if (count >= obj->size / G5P_GPU_PAGE ||
		    !g5p_user_pa_valid(dma) ||
		    dma_to_phys(drm->dev, dma) != dma ||
		    (!imported && dma_to_phys(drm->dev, dma) !=
					  page_to_phys(shmem->pages[count]))) {
			ret = -ERANGE;
			goto err_node;
		}
		addresses[count++] = dma;
	}
	if (count != obj->size / G5P_GPU_PAGE) {
		ret = -EINVAL;
		goto err_node;
	}
	for (; done < a->size; done += G5P_GPU_PAGE) {
		u8 *pte = g5p_vm_pte(vm, drm->dev, a->va + done, true);

		if (IS_ERR_OR_NULL(pte)) {
			ret = pte ? PTR_ERR(pte) : -ENOMEM;
			goto err_clear;
		}
		/* Recovered bypass bits; user permission encodings are not claimed.
		 * A render VM latched to r25 memory variant 4 uses flags 0.
		 */
		g5p_vm_entry(
			&vm->mutation, pte,
			g5p_leaf_entry(
				addresses[(a->offset + done) / G5P_GPU_PAGE],
				vm->zero_leaf_flags ? 0 : g5p_pte_attrs()),
			2);
	}
	dma_wmb();
	m->obj = obj;
	m->offset = a->offset;
	list_add_tail(&m->link, &vm->maps);
	vm->mapped_bytes += a->size;
	vm->map_count++;
	kvfree(addresses);
	return 0;
err_clear:
	g5p_vm_clear(vm, drm->dev, a->va, done);
	if (!vm->resident)
		g5p_vm_prune(drm->dev, vm);
err_node:
	drm_mm_remove_node(&m->node);
err_alloc:
	kvfree(addresses);
	kfree(m);
	g5p_vm_obj_put(vm, obj);
err_obj:
	drm_gem_object_put(obj);
	return ret;
}

static int g5p_vm_bind(struct drm_device *drm, void *data,
		       struct drm_file *file)
{
	struct drm_g5p_vm_bind *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_mapping *m;
	struct g5p_vm *vm;
	int ret = -ENOENT;
	bool resident = false, locked = false;
	u32 generation[3];

	if (a->flags || !g5p_user_range_valid(a->va, a->size) ||
	    (a->op != DRM_G5P_VM_MAP && a->op != DRM_G5P_VM_UNMAP) ||
	    (a->op == DRM_G5P_VM_UNMAP && (a->bo || a->offset)))
		return -EINVAL;
	mutex_lock(&f->lock);
	vm = xa_load(&f->vms, a->vm);
	if (!vm)
		goto out;
	/* Its render context is closing: the close item owns the tables now, and the context
	 * may be freed at any time, so it is not looked at.
	 */
	if (vm->resident_gone) {
		ret = -EBUSY;
		goto out;
	}
	if ((f->submitted || vm->frozen) && !vm->resident) {
		dev_warn_ratelimited(
			drm->dev,
			"G5P VM_BIND busy=no-resident submitted=%u frozen=%u vm=%u op=%u\n",
			f->submitted, vm->frozen, a->vm, a->op);
		ret = -EBUSY;
		goto out;
	}
	if (vm->resident) {
		ret = g5p_resident_idle(vm->resident, true);
		if (ret)
			goto out;
		/* A render context's edit is RAM only and touches its own tables and
		 * slots, which the firmware reads only at that context's next kick;
		 * f->lock plus resident-idle orders it against its submit (a MAP or
		 * UNMAP may pass queued jobs, see g5p_render_vm_idle()).  Other contexts' jobs
		 * may hold the core lock meanwhile, so it is not taken.  Serialized
		 * compute still uses the global mutex.
		 */
		if (!vm->resident->render) {
			mutex_lock(&to_g5p(drm)->core->lock);
			locked = true;
		}
		resident = true;
		memcpy(generation, vm->mutation.generation, sizeof(generation));
	}
	if (a->op == DRM_G5P_VM_MAP) {
		if (vm->bounded && !(vm->resident && vm->resident->render) &&
		    g5p_user_private_overlap(a->va, a->size)) {
			ret = -EINVAL;
			goto out;
		}
		ret = g5p_vm_map_bo(drm, vm, file, a);
		if (!ret)
			g5p_vmlog_add(G5P_VMLOG_MAP, vm, a->vm, a->va, a->size,
				      a->bo, a->offset);
		if (ret == -EBUSY)
			dev_warn_ratelimited(
				drm->dev,
				"G5P VM_BIND busy=map vm=%u bo=%u submitted=%u frozen=%u resident=%u render=%u\n",
				a->vm, a->bo, f->submitted, vm->frozen,
				!!vm->resident,
				!!(vm->resident && vm->resident->render));
		goto out;
	}
	list_for_each_entry(m, &vm->maps, link) {
		if (m->node.start != a->va || m->node.size != a->size) {
			if (m->node.start < a->va + a->size &&
			    a->va < m->node.start + m->node.size) {
				ret = -EINVAL;
				break;
			}
			continue;
		}
		g5p_vmlog_add(G5P_VMLOG_UNMAP, vm, a->vm, a->va, a->size, 0, 0);
		g5p_vm_clear(vm, drm->dev, a->va, a->size);
		if (!vm->resident)
			g5p_vm_prune(drm->dev, vm);
		else {
			ret = g5p_resident_invalidate(vm->resident);
			if (ret)
				goto out;
			memcpy(generation, vm->mutation.generation,
			       sizeof(generation));
		}
		drm_mm_remove_node(&m->node);
		list_del(&m->link);
		vm->map_count--;
		vm->mapped_bytes -= a->size;
		/* Beside queued jobs the PTEs are gone, but the pages stay pinned until
		 * those jobs have retired.
		 */
		if (!vm->resident || !g5p_resident_defer(vm->resident, m)) {
			g5p_vm_obj_put(vm, m->obj);
			drm_gem_object_put(m->obj);
			kfree(m);
		}
		ret = 0;
		break;
	}
out:
	if (resident) {
		int invalidate = 0;

		if (memcmp(generation, vm->mutation.generation,
			   sizeof(generation)))
			invalidate = g5p_resident_invalidate(vm->resident);

		if (invalidate)
			ret = invalidate;
		if (locked)
			mutex_unlock(&to_g5p(drm)->core->lock);
	}
	mutex_unlock(&f->lock);
	return ret;
}

static int g5p_create_context(struct drm_device *drm, void *data,
			      struct drm_file *file)
{
	struct drm_g5p_create_context *a = data;
	if (a->type != DRM_G5P_CONTEXT_RENDER)
		return -EOPNOTSUPP;
	return g5p_render_create_context(drm, file, a);
}

static int g5p_destroy_context(struct drm_device *drm, void *data,
			       struct drm_file *file)
{
	struct drm_g5p_destroy_context *a = data;
	struct g5p_file *f = file->driver_priv;
	struct g5p_context *c;
	int ret = 0;

	if (a->flags)
		return -EINVAL;
	mutex_lock(&f->lock);
	c = xa_load(&f->contexts, a->handle);
	if (!c)
		ret = -ENOENT;
	else
		ret = g5p_render_destroy(c);
	if (!ret && c) {
		c->vm->contexts--;
		xa_erase(&f->contexts, a->handle);
	}
	mutex_unlock(&f->lock);
	if (!ret)
		g5p_context_put(c);
	return ret;
}
