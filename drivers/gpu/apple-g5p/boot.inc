/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Experimental G2 stages. Included after the bounded META slave primitives. */
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <asm/cacheflush.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/vmalloc.h>
#include "boot-layout.h"
#include "render-user-pmset.h"
#include "render-user-recover.h"
#include "fender.h"
#include "readback.h"
#include "meta71c114.h"
#include "latency123.h"
#include "perf-envelope.h"

/* Boot deltas are selected before map; no live render-time reconfiguration. */

/* R46 (diag-fable51 T7): the first SOFT_RESET write of the host reset FIFO
 * also asserts bits 56-59, VBS/VCE/TE3/TILING_CORE in the PBE2_IN_XE
 * layout. Apple's mask 00e7fffffffffc1d is exactly the other layout's
 * MASKFULL, in which those bits do not exist and TE/GPP/UVS/TLA/PM/VDM are
 * already asserted; expected to be a no-op. The unchanged second write
 * releases them. The FIFO forbids a readback here.
 */
#define G5P_BOOT_RESET1 0x00e7fffffffffc1dULL
#define G5P_BOOT_R46_RESET_BITS (0xfULL << 56)

/* R72b: isolate Apple meta+0x6c8 while retaining dc7 transport. */

/* The event ring's stamp events (type 1) dump once per epoch, then a count at each power of two. */

/* R73: mirror Apple's auxiliary firmware-ring returns on IRQ174. */

/* Latency114: the pstate-1 frequency word meta+0x71c (meta71c114.h). */

/* Latency123 (latency123.h): arm A boot words, and the live ctl+0x04 tick-service mask. */

/* perf60 clock (perf-envelope.h): the firmware's own pstate request, pinned before release. */
/* release8 default */
/* GPU overclock (gpu-oc.h, docs/power/ds-gpu-dvfs.md §9 R5): PSINFO record 5 = stock record 4
 * with only the PLL multiplier moved (no voltage change), written before release while the GPC is at
 * state 1, the firmware's table extended to state 5, the release powering on at 5.  Needs
 * boot_gpu_perf 500 and boot_gpu_perf_poweron 1.  A thermal stop (drm-render-v3.inc) reboots the
 * firmware at state 4 when a SoC zone reaches boot_gpu_oc_temp_mc.  Record 5 is volatile PMGR
 * state (LLB rewrites the table only at SoC reset; iBoot reads it once to build the ADT before us)
 * and is cleared on every boot that does not overclock.
 */

static bool
	g5p_oc_tripped; /* the thermal stop fired: every later epoch of this boot runs state 4 */

static const char *const g5p_oc_zone_names[] = { "apple-a9-soc0",
						 "apple-a9-soc1",
						 "apple-a9-tcc2" };

/* The hottest of the three SoC zones (millidegrees); the number of zones read. */
static u32 g5p_oc_temp(int *max_mc, const char **zone)
{
	u32 n = 0;

	*max_mc = INT_MIN;
	*zone = "none";
	for (u32 i = 0; i < ARRAY_SIZE(g5p_oc_zone_names); i++) {
		struct thermal_zone_device *tz =
			thermal_zone_get_zone_by_name(g5p_oc_zone_names[i]);
		int t;

		if (IS_ERR_OR_NULL(tz) || thermal_zone_get_temp(tz, &t))
			continue;
		n++;
		if (t > *max_mc) {
			*max_mc = t;
			*zone = g5p_oc_zone_names[i];
		}
	}
	return n;
}

static int g5p_oc_read(struct g5p_diag *g, u32 n, u32 w[4])
{
	for (u32 i = 0; i < 4; i++)
		if (regmap_read(g->pmgr, G5P_PSINFO_REC(n) + 4 * i, &w[i]))
			return -EIO;
	return 0;
}

/* Write record 5 from the live record 4 and read it back; the GPC is at state 1 (map time).
 * Slot 5 is read first: nonzero and not a record this driver writes -> refused, left untouched.
 */

/* A boot without the overclock clears a record 5 this driver wrote; anything else stays. */
static void g5p_oc_clear(struct g5p_diag *g)
{
	u32 r4[4], r5[4];

	if (!g->pmgr || g5p_oc_read(g, 4, r4) ||
	    g5p_oc_read(g, G5P_OC_STATE, r5) ||
	    !(r5[0] || r5[1] || r5[2] || r5[3]))
		return;
	if (!g5p_oc_is_ours(r4, r5)) {
		dev_info(
			g->dev,
			"GPUPERF oc slot 5 not ours, left: %08x %08x %08x %08x end=gpuperf\n",
			r5[0], r5[1], r5[2], r5[3]);
		return;
	}
	for (u32 i = 0; i < 4; i++)
		regmap_write(g->pmgr, G5P_PSINFO_REC(G5P_OC_STATE) + 4 * i, 0);
	g5p_oc_read(g, G5P_OC_STATE, r5);
	dev_info(
		g->dev,
		"GPUPERF oc record 5 cleared (no overclock this epoch) word0=%08x end=gpuperf\n",
		r5[0]);
}

/* perf60 pin >100, the host's own route: power the GPC on at q (R1+0x1000 = 0x10 | q) in the release,
 * as the GPC1 request the kext and we already make, with q in place of 1; the policy record carries
 * the seed-1 words (+0x28 = +0x2c = q) and stays "never sampled" (+0 = 0), so the boot sampler only
 * initialises; it derives +0x34 and its clock entry priv+0x2c4 from Fender (0x1001c838).
 */
/* release8 default */
/* perf60 pin >100 (perf-envelope.h g5p_perf_seed): the boot sampler applies the pin before ALIVE. */

/* perf60 stage 4 (render-user-pmset.h): firmware-GiB pages for per-context PM records. */
/* release8 default */
/* perf60 A/B (docs/gpu/experiments/perf60-clock.md): the firmware's per-tick full flush alone. */

/* Latched by map, before firmware sees any pointers. Zero preserves round 9. */

/* Explicit decoded VA input only: zero is a valid address when valid=true.
 * Never infer an address or context from revision-dependent e150 bits.
 */
static unsigned long long mmu_fault_va;

static const bool boot_experiment = 1;

struct g5p_dma_page {
	u8 *cpu;
	dma_addr_t dma;
};

struct g5p_boot {
	u32 r27_variant, r27_mode_writes, r46_reset, r50_mode, r50_writes;

	bool table_clean, preinstall_context, compute_boot_vm;
	u32 compute_slot;

	bool compute_numerical, user_compute_owned;

	struct g5p_boot_layout layout;
	struct g5p_dma_page image[G5P_IMAGE_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page root[G5P_ROOT_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page shared[G5P_SHARED_SIZE / G5P_GPU_PAGE];
	struct g5p_dma_page render_meta[2];
	/* perf60 stage 4 (render-user-pmset.h): boot_pm_sets per-context PM sets, 3 pages each */
	u32 pm_sets;
	struct g5p_dma_page pm_set[G5P_PMSET_MAX * G5P_PMSET_PAGES];
	struct g5p_dma_page catalogue, directory, zero, leaf[4];
	struct g5p_dma_page sink_directory, sink_leaf, sink_page;
	u32 setup_variant;
	bool sink_ready, sink_dumped;
	u32 leaf_index[4];
	u32 leaves, pages, mapped, fifo, events, alive_word;
	u32 fender_raw[ARRAY_SIZE(g5p_fender_regs)];
	u32 rgx_raw[ARRAY_SIZE(g5p_rgx_regs)];
	const char *snapshot_stage;
	bool snapshot_valid, rgx_valid;
	u32 rgx_count;
	int fifo_error;
	int irq;
	atomic_t irqs;
	spinlock_t event_lock;
	void __iomem *swirq;
	bool maps_ready, armed, copied, released, alive, failed, irq_ready;
	bool release_ack, release_ack_attempted;
	bool fault_va_valid;
	u64 fault_va;
	bool work_ready, work_submitted, work_complete;
	u32 work_target, control_target;
	u32 dc_variant, work_variant;
	u32 r72_meta6c8;
	u32 meta71c;
	u32 fw_power_words;
	u32 gpu_perf;
	u32 perf_seed; /* boot_gpu_perf_seed: the seeded pstate q, 0 = not seeded */
	u32 perf_seed_mode; /* boot_gpu_perf_seed as latched (1..5) */
	u32 perf_poweron; /* boot_gpu_perf_poweron: the pstate q the release requests, 0 = 1 as before */
	u32 gpu_oc_mhz; /* boot_gpu_oc_mhz as applied (state 5), 0 = no overclock */
	 /* record 5's voltage codes */
	u64 seed_stamp,
		seed_gpc1; /* seed 3/4: the stamp and the GPC1 write (arch timer) */
	u32 meta6d4;
	u32 r78_ctrl0;
	u32 r73_aux_drain;
	u32 r73_aux_entries[6];
	u32 r73_aux_hash[6];
	u32 r73_aux_first[6][3];
	u32 r73_aux_last[6][3];
	struct g5p_r73b *r73b;
	bool irq_capped;
	bool dc_observe;

	bool control_ready, control_submitted, control_complete;
	bool irq_tested, stopped, parked;
	bool control_timed_out, work_timed_out, event_failed;
	bool context_submitted, context_complete, context_timed_out;
	u32 context_target;
	/* The epoch's last recovery packet (render-user-recover.h), under event_lock. */
	struct g5p_fw_recovery fw_recovery;
	u32 fw_recoveries;
	u32 event_n[10]; /* events drained this epoch by type (9: 9 and above) */
};

#include "boot-r73b.inc"

/* The fault-status subset is read-only, including while META reset is held.
 * Never access R0 with GPC off/busy; no slave-port transaction is involved.
 */
static void g5p_fault_snapshot(struct g5p_diag *g, const char *stage);

static int g5p_boot_alloc_page(struct g5p_diag *g, struct g5p_dma_page *p)
{
	p->cpu = dmam_alloc_coherent(g->dev, G5P_GPU_PAGE, &p->dma, GFP_KERNEL);
	if (!p->cpu)
		return -ENOMEM;
	/* There is no GPU bus translation in the recovered GART path. */
	if ((p->dma & (G5P_GPU_PAGE - 1)) || p->dma >= BIT_ULL(40) ||
	    dma_to_phys(g->dev, p->dma) != p->dma) {
		dev_err(g->dev,
			"unsupported DMA translation/alignment at %pad\n",
			&p->dma);
		return -ERANGE;
	}
	memset(p->cpu, 0, G5P_GPU_PAGE);
	g->boot->pages++;
	return 0;
}

/* Deliberately map one16KiB page only, not the iOS all-address fallback.
 * Own distinct directory/leaf/backing; do not alias firmware tables or e160.
 * No live table surgery: this helper runs only during first map construction.
 */
static int g5p_boot_sink_init(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	int ret;

	;
	;
	if (!b->catalogue.cpu || b->maps_ready || b->armed || b->released ||
	    b->sink_ready || b->sink_directory.cpu || b->sink_leaf.cpu ||
	    b->sink_page.cpu)
		return -EPERM;
	if (g5p_le32(b->catalogue.cpu))
		return -EEXIST;
	ret = g5p_boot_alloc_page(g, &b->sink_directory);
	if (ret)
		return ret;
	ret = g5p_boot_alloc_page(g, &b->sink_leaf);
	if (ret)
		return ret;
	ret = g5p_boot_alloc_page(g, &b->sink_page);
	if (ret)
		return ret;
	/* Round12 iOS optional dummy-tree leaf flags0: valid writable PA|1.
	 * Only cat[0]/dir[0]/leaf[0] are installed; every other hole stays absent.
	 */
	g5p_put64(b->sink_leaf.cpu, g5p_leaf_entry(b->sink_page.dma, 0));
	g5p_put64(b->sink_directory.cpu, g5p_directory_entry(b->sink_leaf.dma));
	dma_wmb();
	g5p_put32(b->catalogue.cpu, g5p_catalogue_entry(b->sink_directory.dma));
	dma_wmb();
	b->mapped++;
	b->sink_ready = true;
	g5p_mmu_record(
		g,
		"MMU sink-installed VA=0 bytes=4000 context=0 leaf_flags=0; pre-release "
		"containment experiment\n");
	g5p_blob_printf(
		g,
		"MMU sink PC=%pad cat0=%08x dir=%pad dir0=%016llx leaf=%pad pte0=%016llx "
		"backing=%pad; initially zero\n",
		&b->catalogue.dma, g5p_catalogue_entry(b->sink_directory.dma),
		&b->sink_directory.dma, g5p_directory_entry(b->sink_leaf.dma),
		&b->sink_leaf.dma, g5p_leaf_entry(b->sink_page.dma, 0),
		&b->sink_page.dma);
	return 0;
}

static int g5p_boot_map_page(struct g5p_diag *g, u64 va, u64 pa, u32 attributes)
{
	struct g5p_boot *b = g->boot;
	u32 ci = (va >> 30) & 1023, di = (va >> 21) & 511;
	u32 ti = (va >> 14) & 127, i;
	int ret;

	if ((va | pa) & (G5P_GPU_PAGE - 1))
		return -EINVAL;
	/* This experiment only owns the recovered firmware heap's catalogue slot. */
	if (ci != ((G5P_IMAGE_VA >> 30) & 1023))
		return -ERANGE;
	for (i = 0; i < b->leaves; i++)
		if (b->leaf_index[i] == di)
			break;
	if (i == b->leaves) {
		if (i == ARRAY_SIZE(b->leaf))
			return -ENOSPC;
		ret = g5p_boot_alloc_page(g, &b->leaf[i]);
		if (ret)
			return ret;
		b->leaf_index[i] = di;
		g5p_put64(b->directory.cpu + di * 8,
			  g5p_directory_entry(b->leaf[i].dma));
		b->leaves++;
	}
	if (g5p_le32(b->leaf[i].cpu + ti * 8))
		return -EEXIST;
	g5p_put64(b->leaf[i].cpu + ti * 8, g5p_leaf_entry(pa, attributes));
	g5p_put32(b->catalogue.cpu + ci * 4,
		  g5p_catalogue_entry(b->directory.dma));
	b->mapped++;
	return 0;
}

#include "mmu-attrs.inc"
#include "mmu-tables.inc"

static int g5p_boot_region(struct g5p_diag *g, struct g5p_dma_page *p, u32 size,
			   u64 va)
{
	u32 offset;
	u32 attributes = va == G5P_IMAGE_VA ? 0x10 : 0x14;
	int ret;

	for (offset = 0; offset < size; offset += G5P_GPU_PAGE, p++) {
		ret = g5p_boot_alloc_page(g, p);
		if (ret)
			return ret;
		ret = g5p_boot_map_page(g, va + offset, p->dma, attributes);
		if (ret)
			return ret;
	}
	return 0;
}

static void g5p_boot_copy(struct g5p_dma_page *p, const u8 *data, u32 size)
{
	u32 offset;

	for (offset = 0; offset < size; offset += G5P_GPU_PAGE, p++)
		memcpy(p->cpu, data + offset, G5P_GPU_PAGE);
}

static u8 *g5p_boot_shared(struct g5p_boot *b, u32 slot)
{
	u32 off = b->layout.offset[slot];

	return b->shared[off / G5P_GPU_PAGE].cpu + off % G5P_GPU_PAGE;
}

static int g5p_boot_maps(struct g5p_diag *g)
{
	struct g5p_boot *b;
	u8 *root = NULL, *shared = NULL;
	bool m71c_wrote;
	u32 i, m71c, seed;
	int ret;

	if (!g->loaded || !g->id_checked || PAGE_SIZE != G5P_GPU_PAGE)
		return -EPERM;
	if (g->core_id != 0x011b0602 || g->core_revision != 0x00090f00)
		return -ENODEV;
	if (g->boot)
		return g->boot->maps_ready ? 0 : -EIO;
	b = devm_kzalloc(g->dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	g->boot = b;
	b->r50_mode = (0);
	;
	b->r27_variant = (0);
	;
	b->r46_reset = (0);
	;
	b->r72_meta6c8 = (1);
	b->r78_ctrl0 = (0);
	if (b->r78_ctrl0 && b->r78_ctrl0 != 0x5f) {
		b->failed = true;
		return -EINVAL;
	};
	b->r73_aux_drain = (0);
	if (b->r73_aux_drain > 1 || (b->r73_aux_drain && !1)) {
		b->failed = true;
		return -EINVAL;
	}
	b->release_ack = (0);
	b->table_clean = (1);
	b->compute_boot_vm = (0);
	b->compute_slot = 0 ? 0 : (7);
	b->preinstall_context = (0) || 0;
	;
	b->setup_variant = (1);
	;
	b->fault_va = READ_ONCE(mmu_fault_va);
	b->fault_va_valid = (0);
	;
	b->dc_variant = (7);
	b->work_variant = (0);
	b->dc_observe = (0);
	;
	b->meta71c = (0);
	m71c = g5p_m71c_check(b->meta71c, 7);
	if (m71c != G5P_M71C_OK) {
		dev_err(g->dev,
			"META71C refused requested=%u variant=%u why=%s before_release=1 end=meta71c\n",
			b->meta71c, b->dc_variant, g5p_m71c_whys[m71c]);
		b->failed = true;
		return -EINVAL;
	}
	b->fw_power_words = (0);
	b->gpu_perf = (400);
	b->meta6d4 = (0);
	seed = (0);
	b->gpu_oc_mhz = 0;
	;
	b->perf_poweron =
		(1) ? (0 ? G5P_OC_STATE : g5p_perf_seed_q(b->gpu_perf)) : 0;
	b->perf_seed = seed && !b->perf_poweron ? g5p_perf_seed_q(b->gpu_perf) :
						  0;
	b->perf_seed_mode = b->perf_seed ? seed : 0;
	if (!(g5p_perf_valid(b->gpu_perf) ||
	      (0 && b->gpu_perf == G5P_OC_PIN)) ||
	    (b->gpu_perf && b->fw_power_words) || seed > 5 || (1) > 1 ||
	    (b->gpu_perf > 100 && !(1)) || b->meta6d4 > 1 ||
	    (b->meta6d4 && !b->gpu_perf)) {
		dev_err(g->dev,
			"GPUPERF refused pin=%u power_words=%u gpc_any_pstate=%u meta6d4=%u seed=%u "
			"before_release=1 end=gpuperf\n",
			b->gpu_perf, b->fw_power_words, (1), b->meta6d4, seed);
		b->failed = true;
		return -EINVAL;
	}
	if (b->fw_power_words > G5P_L123_FULL || (0) > G5P_L123_TICK_MASK) {
		dev_err(g->dev,
			"L123 refused power_words=%u tick_mask=%u before_release=1 end=l123\n",
			b->fw_power_words, (0));
		b->failed = true;
		return -EINVAL;
	}
	ret = g5p_r73b_latch(g);
	if (ret) {
		b->failed = true;
		return ret;
	};
	dev_info(
		g->dev,
		"DC matrix latched variant=%u observe=%u (changes require reboot)\n",
		b->dc_variant, b->dc_observe);
	dev_info(
		g->dev,
		"COMPUTE VM latched slot=%u boot_vm=%u preinstall=%u (changes require reboot)\n",
		b->compute_slot, b->compute_boot_vm, b->preinstall_context);
	spin_lock_init(&b->event_lock);
	atomic_set(&b->irqs, 0);
	ret = g5p_plan_layout(&b->layout);
	if (ret)
		goto out;
	root = kvzalloc(G5P_ROOT_SIZE, GFP_KERNEL);
	shared = kvzalloc(G5P_SHARED_SIZE, GFP_KERNEL);
	if (!root || !shared) {
		ret = -ENOMEM;
		goto out;
	}
	ret = g5p_boot_alloc_page(g, &b->catalogue);
	if (ret)
		goto out;
	ret = g5p_boot_alloc_page(g, &b->directory);
	if (ret)
		goto out;
	ret = g5p_boot_alloc_page(g, &b->zero);
	if (ret)
		goto out;
	ret = g5p_boot_sink_init(g);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->image, G5P_IMAGE_SIZE, G5P_IMAGE_VA);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->root, G5P_ROOT_SIZE, G5P_ROOT_VA);
	if (ret)
		goto out;
	ret = g5p_boot_region(g, b->shared, G5P_SHARED_SIZE, G5P_SHARED_VA);
	if (ret)
		goto out;
	/* R22 metadata mappings are installed before release, only behind its
	 * default-off gate. Compute/boot defaults allocate no additional pages.
	 */
	{
		ret = g5p_boot_region(g, b->render_meta, 2 * G5P_GPU_PAGE,
				      G5P_SHARED_VA + G5P_SHARED_SIZE);
		if (ret)
			goto out;
		b->pm_sets = min((8), G5P_PMSET_MAX);
		if (b->pm_sets) {
			ret = g5p_boot_region(g, b->pm_set,
					      b->pm_sets * G5P_PMSET_PAGES *
						      G5P_GPU_PAGE,
					      G5P_PMSET_VA);
			if (ret)
				goto out;
			dev_info(
				g->dev,
				"RENDER-V3 pm-sets boot sets=%u va=%#llx..%#llx before_release=1 end=v3\n",
				b->pm_sets, G5P_PMSET_VA,
				g5p_pmset_va(b->pm_sets, 0));
		}
	}
	ret = 0;
	if (ret)
		goto out;
	/* c80758: Fender (2 pages), AIC timer, AIC software interrupt. */
	for (i = 0; i < 4; i++) {
		u64 pa = i < 2	? 0x201e00000ULL + i * G5P_GPU_PAGE :
			 i == 2 ? 0x20e100000ULL :
				  0x20e104000ULL;

		ret = g5p_boot_map_page(g, G5P_PIO_VA + i * G5P_GPU_PAGE, pa,
					0x18);
		if (ret)
			goto out;
	}
	g5p_make_boot_config(&g->image, &b->layout);
	g5p_make_init(&b->layout, root, shared, &g->image, b->catalogue.dma,
		      arch_timer_read_counter());
	g5p_dc_init_variant(&b->layout, root, shared, 7);
	/* meta+0x784 is the abstime OFFSET the firmware adds to its timebase (0x100221e8 = PIO-1
	 * counter + priv+688, loaded from meta+0x784 at 0x1001ff58); iOS writes
	 * ml_get_abstime_offset().  Linux's timebase is the counter itself: 0.  (make_init's absolute
	 * stamp put the firmware's clock ~uptime ahead of every host stamp in policy+0x8f8.)
	 */
	g5p_put64(g5p_object(&b->layout, root, shared, 0x4d) + 0x784, 0);
	if (g5p_boot_ctrl0(b))
		g5p_put32(g5p_object(&b->layout, root, shared, 0x5e),
			  g5p_boot_ctrl0(b));
	dev_info(
		g->dev,
		"R78 ctrl0 requested=%08x installed=%08x before_release=1 end=r78\n",
		g5p_boot_ctrl0(b),
		g5p_le32(g5p_object(&b->layout, root, shared, 0x5e)));
	;
	;
	{
		g5p_put32(g5p_object(&b->layout, root, shared, 0x4d) + 0x6c8,
			  1);
	};
	m71c = g5p_m71c_install(&b->layout, root, shared, b->meta71c,
				&m71c_wrote);
	dev_info(
		g->dev,
		"META71C requested=%u installed=%08x variant=%u wrote=%u before_release=1 "
		"end=meta71c\n",
		b->meta71c, m71c, b->dc_variant, m71c_wrote);
	if (m71c != b->meta71c) {
		ret = -EIO;
		goto out;
	}
	if (b->fw_power_words)
		g5p_l123_arm_a(&b->layout, root, shared, b->fw_power_words);
	if (b->gpu_perf) {
		u8 *pol = g5p_object(&b->layout, root, shared, G5P_PERF_POLICY);
		u8 *mt = g5p_object(&b->layout, root, shared, G5P_PERF_META);
		u8 *ct = g5p_object(&b->layout, root, shared, G5P_PERF_CTL);

		g5p_perf_install(&b->layout, root, shared, b->gpu_perf);
		{
			g5p_oc_clear(g);
		}
		if (b->meta6d4)
			g5p_put32(mt + 0x6d4, 1);
		if (b->perf_poweron) {
			g5p_perf_seed(&b->layout, root, shared, b->perf_poweron,
				      false);
			dev_info(
				g->dev,
				"GPUPERF poweron q=%u policy28=%u/%u policy0=%u policy4=%u seed=%u ignored "
				"before_release=1 end=gpuperf\n",
				b->perf_poweron, g5p_le32(pol + 0x28),
				g5p_le32(pol + 0x2c), g5p_le32(pol + 0),
				g5p_le32(pol + 4), seed);
		} else {
			dev_info(
				g->dev,
				"GPUPERF poweron ignored pin=%u (pins 200..400 only) before_release=1 end=gpuperf\n",
				b->gpu_perf);
		}
		if (seed && !b->perf_seed && !b->perf_poweron)
			dev_info(
				g->dev,
				"GPUPERF seed ignored pin=%u (seeds pins 200..400 only) before_release=1 "
				"end=gpuperf\n",
				b->gpu_perf);
		if (b->perf_seed) {
			g5p_perf_seed(&b->layout, root, shared, b->perf_seed,
				      seed >= 2);
			/* seed 5: meta+0x784 is the abstime OFFSET the firmware adds to its timebase
			 * (0x100221e8 = PIO-1 counter + priv+688, loaded from meta+0x784 at 0x1001ff58);
			 * iOS writes ml_get_abstime_offset(), make_init the absolute counter, so every
			 * policy+0x8f8 stamp lay ~counter in the firmware's past.  Linux has no offset.
			 */
			if (seed == 5)
				g5p_put64(mt + 0x784, 0);
			dev_info(
				g->dev,
				"GPUPERF seed mode=%u q=%u policy28=%u/%u policy30=%u policy8=%u policy0=%u "
				"policy4=%u meta784=%llx before_release=1 end=gpuperf\n",
				seed, b->perf_seed, g5p_le32(pol + 0x28),
				g5p_le32(pol + 0x2c), g5p_le32(pol + 0x30),
				g5p_le32(pol + 8), g5p_le32(pol + 0),
				g5p_le32(pol + 4),
				(u64)g5p_le32(mt + 0x784) |
					(u64)g5p_le32(mt + 0x788) << 32);
		}
		dev_info(
			g->dev,
			"GPUPERF boot pin=%u table=%u/%u/%u,%u,%u,%u,%u limits=%u/%u/%u ovr=%u/%u "
			"policy688=%u/%u policy6f0=%u/%u policy4=%u policy8=%u/%u q=%u mhz=%u "
			"gpc_any_pstate=%u seed=%u/%u policy0=%u policy28=%u/%u before_release=1 "
			"end=gpuperf\n",
			b->gpu_perf, g5p_le32(mt + 0x710), g5p_le32(mt + 0x714),
			g5p_le32(mt + 0x718), g5p_le32(mt + 0x71c),
			g5p_le32(mt + 0x720), g5p_le32(mt + 0x724),
			g5p_le32(mt + 0x728), g5p_le32(ct + 0x1028),
			g5p_le32(ct + 0x102c), g5p_le32(ct + 0x1030),
			g5p_le32(ct + 0x1038), g5p_le32(ct + 0x1040),
			g5p_le32(pol + 0x688), g5p_le32(pol + 0x68c),
			g5p_le32(pol + 0x6f0), g5p_le32(pol + 0x6f4),
			g5p_le32(pol + 4), g5p_le32(pol + 8),
			g5p_le32(pol + 0xc), b->gpu_perf / 100,
			b->gpu_perf / 100 < ARRAY_SIZE(g5p_perf_mhz) ?
				g5p_perf_mhz[b->gpu_perf / 100] :
				b->gpu_oc_mhz,
			(1), seed, b->perf_seed, g5p_le32(pol + 0),
			g5p_le32(pol + 0x28), g5p_le32(pol + 0x2c));
	}
	g5p_put32(g5p_object(&b->layout, root, shared, G5P_L123_CTL) + 4, (0));
	{
		u8 *meta = g5p_object(&b->layout, root, shared, G5P_L123_META);

		dev_info(
			g->dev,
			"L123 boot power_words=%u meta6d4=%u table=%u/%u/%u,%u,%u,%u,%u policy4=%u "
			"ctl04=%u limits=%u/%u/%u before_release=1 end=l123\n",
			b->fw_power_words, g5p_le32(meta + 0x6d4),
			g5p_le32(meta + 0x710), g5p_le32(meta + 0x714),
			g5p_le32(meta + 0x718), g5p_le32(meta + 0x71c),
			g5p_le32(meta + 0x720), g5p_le32(meta + 0x724),
			g5p_le32(meta + 0x728),
			g5p_le32(g5p_object(&b->layout, root, shared,
					    G5P_L123_POLICY) +
				 4),
			g5p_le32(g5p_object(&b->layout, root, shared,
					    G5P_L123_CTL) +
				 4),
			g5p_le32(g5p_object(&b->layout, root, shared,
					    G5P_L123_CTL) +
				 0x1028),
			g5p_le32(g5p_object(&b->layout, root, shared,
					    G5P_L123_CTL) +
				 0x102c),
			g5p_le32(g5p_object(&b->layout, root, shared,
					    G5P_L123_CTL) +
				 0x1030));
		/* the accessor's licence bound (mmio-access.md §4.2 step 4): meta+0x710 as written */
		WRITE_ONCE(g->mmio_max_pstate,
			   g5p_mmio_max_pstate(g5p_le32(meta + 0x710)));
	};
	;
	g5p_boot_copy(b->image, g->image.data, G5P_IMAGE_SIZE);
	g5p_boot_copy(b->root, root, G5P_ROOT_SIZE);
	g5p_boot_copy(b->shared, shared, G5P_SHARED_SIZE);
	dma_wmb();
	ret = 0;
	if (ret)
		goto out;
	g5p_tables_clean(g, "boot-before-release", NULL, 0);
	b->maps_ready = true;
	;
	dev_info(
		g->dev,
		"G2 maps: %u x 16KiB allocations; %u PTEs, %u leaves, PC=%pad\n",
		b->pages, b->mapped, b->leaves, &b->catalogue.dma);
	dev_info(
		g->dev,
		"G2 init: root=%llx control=%llx event-ctl=%llx event-data=%llx\n",
		g5p_object_va(&b->layout, 0x50),
		g5p_object_va(&b->layout, 0x5f),
		g5p_object_va(&b->layout, 0x78),
		g5p_object_va(&b->layout, 0x7c));
	dev_info(
		g->dev,
		"G2 experimental zero policy fields; fault observation only in map\n");
out:
	kvfree(root);
	kvfree(shared);
	if (ret)
		b->failed = true;
	g5p_ds_hook_maps(g, ret);
	return ret;
}

#include "boot-r74f.inc"
#include "boot-r74g.inc"

static u32 g5p_transport_event_limit(void);

static int g5p_boot_drain(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 *control = (u32 *)g5p_boot_shared(b, 0x70);
	u32 consumer, producer, n;
	unsigned long flags;
	int ret = 0;

	if (READ_ONCE(b->failed))
		return -EIO;
	spin_lock_irqsave(&b->event_lock, flags);
	consumer = READ_ONCE(control[0]);
	producer = READ_ONCE(control[4]);
	if (consumer > 255 || producer > 255) {
		dev_err(g->dev, "G2 invalid event indices %u/%u\n", consumer,
			producer);
		ret = -EPROTO;
		goto out;
	}
	dma_rmb();
	for (n = 0; n < 16 && consumer != producer; n++) {
		u32 words[11], off = b->layout.offset[0x74] + consumer * 44, t,
			       n;
		u8 *src;
		unsigned int i;
		bool want;

		/* Heartbeat116: an R74 epoch bounds ring entries per window
		 * in the IRQ thread instead (heartbeat116.h EVENTS).
		 */
		if (!g5p_hb116_on(b) &&
		    b->events >= g5p_transport_event_limit()) {
			ret = -EOVERFLOW;
			goto out;
		}
		/* Entries may straddle DMA pages. */
		for (i = 0; i < ARRAY_SIZE(words); i++) {
			u32 at = off + i * 4;

			src = b->shared[at / G5P_GPU_PAGE].cpu +
			      at % G5P_GPU_PAGE;
			words[i] = g5p_le32(src);
		}
		/* A healthy meta6c8 setup loop can emit thousands of identical
		 * ALIVEs. Keep its first four packets and the exact aggregate,
		 * without making printk throughput part of the experiment.
		 * R74 also quiets a moved loop: each new word1 prints once.
		 */
		want = !b->r73b || words[0] || b->events < 4 ||
		       (words[1] != 0x8000a120 && g5p_r74_word1(b, words[1]));
		/* Stamp events (type 1, two per job) dump once per epoch, then only count:
		 * at a few jobs per frame they were most of the log and evicted the faults
		 * (release8 bench, 306 k lines).  Every other type dumps as before.
		 */
		t = min_t(u32, words[0], ARRAY_SIZE(b->event_n) - 1);
		n = ++b->event_n[t];
		if (words[0] == 1 && n > 1 && !(0)) {
			if (is_power_of_2(n))
				dev_info(
					g->dev,
					"G2 events type=1 n=%u: stamp events counted, dumped once (boot_event_dump 1: each)\n",
					n);
			want = false;
		}
		/* Alive (0) and stamp (1) events are routine, one or more per job: job_log.  Every other
		 * type (recovery, page request, channel error, fault) prints as before.
		 */
		if (want && words[0] > 1) {
			dev_info(g->dev,
				 "G2 event index=%u type=%u word1=%08x\n",
				 consumer, words[0], words[1]);
			print_hex_dump(KERN_INFO,
				       "G2 event raw: ", DUMP_PREFIX_NONE, 16,
				       4, words, sizeof(words), false);
		} else if (want) {
			g5p_job_printk(g->dev,
				       "G2 event index=%u type=%u word1=%08x\n",
				       consumer, words[0], words[1]);
			g5p_job_hex("G2 event raw: ", words, sizeof(words));
		};
		if (g->event_notify)
			g->event_notify(g->event_cookie, words);
		/* c99788 routes these to recovery/page-request/channel-error paths. */
		if (words[0] != 0 && words[0] != 1) {
			WRITE_ONCE(b->failed, true);
			switch (words[0]) {
			case G5P_FW_EVENT_RECOVERY: {
				/* The kext's fields (render-user-recover.h). */
				struct g5p_fw_recovery rec =
					g5p_fw_recovery_decode(words);

				b->fw_recovery = rec;
				b->fw_recoveries++;
				dev_err(g->dev,
					"G2 recovery event slot=%u flag=%u code=%08x w18=%08x w20=%08x w24=%08x n=%u: "
					"the firmware stopped; iOS restarts the GPU\n",
					rec.slot, rec.flag, rec.code, rec.w18,
					rec.w20, rec.w24, b->fw_recoveries);
				break;
			}
			case 7:
				dev_err(g->dev,
					"G2 page-request event payload=%08x/%08x/%08x; address encoding unresolved\n",
					words[1], words[2], words[3]);
				break;
			case 8:
				dev_err(g->dev,
					"G2 channel-error event kind=%u engine=%u token=%08x stamp=%08x; no "
					"acknowledgment command guessed\n",
					words[1], words[2], words[3], words[4]);
				break;
			}
			dev_err(g->dev,
				"G2 unsupported/fault event=%u; epoch failed, entry retained (the render device "
				"recovers it if it may)\n",
				words[0]);
			ret = -EIO;
			break;
		}
		if (words[0] == 0 && READ_ONCE(b->released) &&
		    !READ_ONCE(b->stopped)) {
			b->alive_word = words[1];
			WRITE_ONCE(b->alive, true);
		}
		b->events++;
		consumer = (consumer + 1) & 255;
		/* c99788 uses a full barrier: finish slot reads before returning it. */
		mb();
		WRITE_ONCE(control[0], consumer);
		/* Return the ALIVE slot before ringing work. No logging or
		 * other MMIO may be inserted between these two operations.
		 */
		if (!words[0] && g5p_r74f_on(b) &&
		    READ_ONCE(b->r73b->owner) == G5P_R73B_OWNER_IRQ &&
		    READ_ONCE(b->released) && !READ_ONCE(b->stopped))
			g5p_r74f_alive(g);
	}
out:
	if (ret) {
		WRITE_ONCE(b->event_failed, true);
		WRITE_ONCE(b->failed, true);
	}
	spin_unlock_irqrestore(&b->event_lock, flags);
	return ret;
}

static irqreturn_t g5p_boot_irq(int irq, void *data)
{
	struct g5p_diag *g = data;
	struct g5p_boot *b = g->boot;
	u32 n = atomic_inc_return(&b->irqs);
	int ret = 0;

	if (b->r73b)
		return g5p_r73b_irq(g, irq, n);
	if (n > g5p_transport_event_limit()) {
		/* Never leave the software source asserted, even when capped. */
		writel(0x4000, b->swirq + 0x94);
		if (!b->irq_capped) {
			b->irq_capped = true;
			WRITE_ONCE(b->event_failed, true);
			WRITE_ONCE(b->failed, true);
			disable_irq_nosync(irq);
			dev_err(g->dev,
				"G2 IRQ174 storm capped; reboot required\n");
		}
		return IRQ_HANDLED;
	}
	/* KPF level6G: Apple uses this AIC ACK before all ring drains. */
	writel(0x4000, b->swirq + 0x94);
	if (READ_ONCE(b->released))
		ret = g5p_boot_drain(g);
	if (ret) {
		WRITE_ONCE(b->event_failed, true);
		WRITE_ONCE(b->failed, true);
	}
	return IRQ_HANDLED;
}

static int g5p_boot_irq_setup(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (b->irq_ready)
		return 0;
	b->swirq = devm_ioremap(g->dev, 0x20e104000ULL, 0x100);
	if (!b->swirq)
		return -ENOMEM;
	b->irq = platform_get_irq(to_platform_device(g->dev), 4);
	if (b->irq < 0)
		return b->irq;
	ret = devm_request_threaded_irq(g->dev, b->irq, NULL, g5p_boot_irq,
					IRQF_ONESHOT, "g5p-meta", g);
	if (ret)
		return ret;
	b->irq_ready = true;
	dev_info(g->dev, "G2 IRQ174 registered as Linux IRQ %d\n", b->irq);
	return 0;
}

/* Cache first, then report: sysfs status never issues GPU MMIO. */
static void g5p_boot_snapshot(struct g5p_diag *g, const char *stage,
			      enum g5p_readback_state mode)
{
	struct g5p_boot *b = g->boot;
	u32 i, state;

	b->snapshot_stage = stage;
	b->snapshot_valid = false;
	b->rgx_valid = false;
	b->rgx_count = 0;
	g5p_power_snapshot(g, stage);
	dev_info(g->dev, "G2 snapshot %s: R1 first\n", stage);
	for (i = 0; i < ARRAY_SIZE(g5p_fender_regs); i++) {
		b->fender_raw[i] = readl(g->window[1] + g5p_fender_regs[i]);
		dev_info(g->dev, "G2 raw R1+%05x=%08x\n", g5p_fender_regs[i],
			 b->fender_raw[i]);
	}
	b->snapshot_valid = true;
	state = b->fender_raw[4];
	/* GPC=1 did not make META a18 safe while bit32 was held. */
	b->rgx_count = g5p_readback_rgx_count(state, mode);
	if (!b->rgx_count) {
		dev_info(g->dev, "G2 R0 snapshot skipped: R1+1000=%08x\n",
			 state);
		return;
	}
	for (i = 0; i < b->rgx_count; i++) {
		dev_info(g->dev, "G2 R R0+%05x\n", g5p_rgx_regs[i]);
		b->rgx_raw[i] = readl(g->rgx + g5p_rgx_regs[i]);
		dev_info(g->dev, "G2 raw R0+%05x=%08x\n", g5p_rgx_regs[i],
			 b->rgx_raw[i]);
	}
	b->rgx_valid = true;
}

static int g5p_fender_write(void *ctx, u32 reg, u32 value)
{
	struct g5p_diag *g = ctx;
	u32 raw;

	/* recovery108: never an R1 access after a FINAL teardown retired the device. */
	if (!g5p_r8_r1_allowed())
		return -ESHUTDOWN;
	dev_info(g->dev, "G2 W R1+%05x=%08x\n", reg, value);
	writel(value, g->window[1] + reg);
	raw = readl(g->window[1] + reg);
	dev_info(g->dev, "G2 raw R1+%05x=%08x (write fence)\n", reg, raw);
	return 0;
}

static int g5p_fender_poll(void *ctx, u32 reg, u32 mask, u32 expected)
{
	struct g5p_diag *g = ctx;
	u32 raw;
	int ret;

	if (!g5p_r8_r1_allowed())
		return -ESHUTDOWN;
	dev_info(g->dev, "G2 poll R1+%05x mask=%08x expected=%08x (100ms)\n",
		 reg, mask, expected);
	ret = readl_poll_timeout(g->window[1] + reg, raw,
				 (raw & mask) == expected, 10, 100000);
	dev_info(g->dev, "G2 poll raw R1+%05x=%08x ret=%d\n", reg, raw, ret);
	return ret;
}

static int g5p_r74e_power_poll(struct g5p_diag *g, u32 expected, u32 timeout_us,
			       u64 *elapsed_ns, u32 *samples)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	u64 start = ktime_get_mono_fast_ns();
	u32 raw = U32_MAX, n = 0;
	int ret;

	for (;;) {
		if (READ_ONCE(b->alive)) {
			ret = -EHOSTDOWN;
			break;
		}
		raw = readl(g->window[1] + 0x1000);
		n++;
		if ((raw & 0x1f) == expected) {
			ret = 0;
			break;
		}
		if (ktime_get_mono_fast_ns() - start >=
		    (u64)timeout_us * 1000) {
			ret = -ETIMEDOUT;
			break;
		}
		udelay(10);
	}
	*elapsed_ns = ktime_get_mono_fast_ns() - start;
	/* wake112: the reads taken; 0 means raw is still the U32_MAX initializer, not a read. */
	*samples = n;
	r->early_power_raw = raw;
	return ret;
}

/* wake112 (r74e112.h): a failed deferred proof is the cold power abort, absorbing. */
static void g5p_r74e_fail(struct g5p_r73b *r, u32 why)
{
	r->early_proof_why = why;
	WRITE_ONCE(r->early_power_abort, true);
	WRITE_ONCE(r->fast_window, false);
	atomic_set_release(&r->early_proof, G5P_PX_FAILED);
}

/* The once-owner takes the deferred proof right before its one R1+0x1000 read.
 * false: that read must not happen, and the proof has failed.
 */
static bool g5p_r74e_take(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	struct g5p_px_pre p;
	u32 why;

	if (atomic_cmpxchg(&r->early_proof, G5P_PX_DEFERRED, G5P_PX_TAKEN) !=
	    G5P_PX_DEFERRED)
		return false;
	p = (struct g5p_px_pre){
		.gen = r->early_proof_gen,
		.gen_now = g5p_ds_gen(),
		.alive = READ_ONCE(b->alive),
		.released = READ_ONCE(b->released),
		.stopped = READ_ONCE(b->stopped),
		.failed = READ_ONCE(b->failed) || READ_ONCE(b->event_failed),
		.r1_allowed = g5p_r8_r1_allowed(),
	};
	r->early_proof_gen_now = p.gen_now;
	why = g5p_px_check(&p);
	if (why)
		g5p_r74e_fail(r, why);
	return !why;
}

/* The owner's read decides: only the state field 1 licenses the release. */
static void g5p_r74e_settle(struct g5p_r73b *r, u32 raw)
{
	if (!g5p_px_on(raw)) {
		g5p_r74e_fail(r, G5P_PXW_STATE);
		return;
	}
	WRITE_ONCE(r->early_release_ready, true);
	atomic_set_release(&r->early_proof, G5P_PX_PROVEN);
}

/* No ALIVE wait ends with the proof still owed: close it, no register access.
 * true: it was still owed.
 */
static bool g5p_r74e_close(struct g5p_diag *g)
{
	struct g5p_r73b *r = g->boot ? g->boot->r73b : NULL;

	if (!r || atomic_cmpxchg(&r->early_proof, G5P_PX_DEFERRED,
				 G5P_PX_TAKEN) != G5P_PX_DEFERRED)
		return false;
	g5p_r74e_fail(r, G5P_PXW_UNTAKEN);
	return true;
}

/* boot_gpu_perf_seed 3/4: the sampler's last-sample time policy+0x8f8 = now.  Seed 3 stamps at the
 * start of the release (flown 2026-11-03: 38 ms before the stream's end, the window was missed);
 * seed 4 stamps as the last host write before the one that lets META run, the GPC1 request
 * R1+0x1000 = 0x11 (writel orders the RAM store before it).
 */
static void g5p_perf_seed_stamp(struct g5p_boot *b, u32 mode)
{
	u32 o;

	if (mode == 4 ? b->perf_seed_mode < 4 : b->perf_seed_mode != mode)
		return; /* the GPC1 stamp serves seeds 4 and 5 */
	o = b->layout.offset[G5P_PERF_POLICY] + 0x8f8;
	b->seed_stamp = arch_timer_read_counter();
	WRITE_ONCE(*(u32 *)(b->root[o / G5P_GPU_PAGE].cpu + o % G5P_GPU_PAGE),
		   cpu_to_le32((u32)b->seed_stamp));
	o += 4;
	WRITE_ONCE(*(u32 *)(b->root[o / G5P_GPU_PAGE].cpu + o % G5P_GPU_PAGE),
		   cpu_to_le32((u32)(b->seed_stamp >> 32)));
	wmb();
}

static int g5p_r74e_fender_start(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	struct g5p_r73b *r = b->r73b;
	enum g5p_pxp latch;
	bool on_ran = false;
	u32 samples =
		0; /* wake112: the reads of the poll that ran last (the printed raw) */
	int ret;

	r->early_power_start_ns = ktime_get_mono_fast_ns();
	ret = g5p_fender_write(g, 0x20000, 0);
	if (!ret)
		ret = g5p_r74e_power_poll(g, 0, 1000, &r->early_off_ns,
					  &samples);
	if (!ret) {
		g5p_perf_seed_stamp(b, 4);
		ret = g5p_fender_write(g, 0x1000,
				       0x10 | (b->perf_poweron ?: 1));
		if (b->seed_stamp)
			b->seed_gpc1 = arch_timer_read_counter();
	}
	if (!ret) {
		on_ran = true;
		/* poweron: the first request above pstate 1, a voltage step; a longer bound */
		ret = g5p_r74e_power_poll(g, b->perf_poweron ?: 1,
					  b->perf_poweron ? 10000 : 1800,
					  &r->early_on_ns, &samples);
		if (b->perf_poweron)
			dev_emerg(
				g->dev,
				"GPUPERF poweron q=%u r1_1000=%08x on_ns=%llu ret=%d end=gpuperf\n",
				b->perf_poweron, r->early_power_raw,
				r->early_on_ns, ret);
	}
	/* wake112: only ALIVE preempting the on-poll before its first read defers; the
	 * first-ALIVE owner owes the proof.  A sampled on-poll (0x11, ...) aborts as before.
	 */
	latch = g5p_px_poll(on_ran, ret, ret == -EHOSTDOWN, samples);
	if (latch == G5P_PXP_DEFER) {
		r->early_defer_ret = ret;
		r->early_proof_gen = g5p_ds_gen();
		ret = 0;
		atomic_set_release(&r->early_proof, G5P_PX_DEFERRED);
	}
	r->early_power_ret = ret;
	if (latch == G5P_PXP_ABORT) {
		WRITE_ONCE(r->early_power_abort, true);
		WRITE_ONCE(r->fast_window, false);
	} else if (latch == G5P_PXP_READY) {
		WRITE_ONCE(r->early_release_ready, true);
	}
	dev_emerg(
		g->dev,
		"R74E power off_ns=%llu on_ns=%llu raw=%08x ret=%d release_ready=%u alive=%u "
		"sampled=%u end=r74e\n",
		r->early_off_ns, r->early_on_ns, r->early_power_raw, ret,
		r->early_release_ready, READ_ONCE(b->alive), samples);
	if (latch == G5P_PXP_DEFER)
		dev_emerg(
			g->dev,
			"R74E112 defer gen=%u on_ret=%d raw=%08x alive=%u end=r74e112\n",
			r->early_proof_gen, r->early_defer_ret,
			r->early_power_raw, READ_ONCE(b->alive));
	return ret;
}

static void g5p_fifo_word(struct g5p_diag *g, u32 word)
{
	struct g5p_boot *b = g->boot;
	u32 raw;

	if (b->fifo_error)
		return;
	if (b->fifo < 0x10000 || b->fifo >= 0x10200) {
		b->fifo_error = -EOVERFLOW;
		return;
	}
	writel(word, g->window[1] + b->fifo);
	raw = readl(g->window[1] + b->fifo);
	if (raw != word) {
		dev_err(g->dev,
			"G2 FIFO verify R1+%05x actual=%08x expected=%08x\n",
			b->fifo, raw, word);
		b->fifo_error = -EIO;
	}
	b->fifo += 4;
}

static void g5p_fifo32(struct g5p_diag *g, u32 reg, u32 value, u32 flags)
{
	dev_info(g->dev, "G2 FIFO R0+%05x=%08x flags=%02x\n", reg, value,
		 flags);
	g5p_fifo_word(g, (reg & 0xff00ffff) | (flags & 0xff) << 16);
	g5p_fifo_word(g, value);
}

static void g5p_fifo64(struct g5p_diag *g, u32 reg, u64 value)
{
	g5p_fifo32(g, reg + 4, value >> 32, 0);
	g5p_fifo32(g, reg, value, 0xff);
}

static int g5p_boot_fifo_dump(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 off, raw, end = min(b->fifo, 0x10200U);

	if (end <= 0x10000)
		return -ENODATA;
	dev_info(g->dev, "G2 FIFO dump R1+10000..%05x (<=128 words)\n",
		 end - 4);
	for (off = 0x10000; off < end; off += 4) {
		raw = readl(g->window[1] + off);
		dev_info(g->dev, "G2 FIFO raw R1+%05x=%08x\n", off, raw);
	}
	return 0;
}

static u64 g5p_snapshot64(struct g5p_boot *b, u32 reg)
{
	u32 i;

	for (i = 0; i + 1 < b->rgx_count; i++)
		if (g5p_rgx_regs[i] == reg && g5p_rgx_regs[i + 1] == reg + 4)
			return (u64)b->rgx_raw[i + 1] << 32 | b->rgx_raw[i];
	return 0;
}

static void g5p_release_observe(void *ctx, const char *stage)
{
	;
	;
}

/* A diagnostic notification ACK only, never engine repair or a retry loop.
 * Snapshot first, require the active wrapper, ACK only200 once, fence and
 * re-sample. A cleared event alone is not evidence the underlying fault healed.
 */
static int g5p_release_ack(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 state, event, after;

	if (!b->release_ack)
		return 0;
	if (!b->released || b->stopped || b->release_ack_attempted)
		return -EPERM;
	state = readl(g->window[1] + 0x1000);
	if (!g5p_gpc_ok(state))
		return -EHOSTDOWN;
	g5p_fault_snapshot(g, "release-ack-before");
	g5p_mmu_progress(g, "release-ack-event", 0x130);
	event = readl(g->rgx + 0x130);
	b->release_ack_attempted = true;
	if (event & 0x200) {
		g5p_mmu_progress(g, "release-ack-write", 0x138);
		writel(0x200, g->rgx + 0x138);
		/* Ordered same-device read fences the single write. */
		g5p_mmu_progress(g, "release-ack-fence", 0x130);
		after = readl(g->rgx + 0x130);
		g5p_mmu_record(
			g,
			"MMU release-ack mask=00000200 before=%08x after=%08x; notification only\n",
			event, after);
	} else {
		g5p_mmu_record(g,
			       "MMU release-ack skipped=no-event before=%08x\n",
			       event);
	}
	g5p_fault_snapshot(g, "release-ack-after");
	return 0;
}

static int g5p_boot_program_stream(struct g5p_diag *g, bool release)
{
	struct g5p_boot *b = g->boot;
	const struct g5p_fender_io io = {
		.ctx = g,
		.write = g5p_fender_write,
		.poll = g5p_fender_poll,
	};
	u64 reset;
	int ret;

	if (b->armed && !release)
		return -EALREADY;
	ret = g5p_boot_irq_setup(g);
	if (ret)
		return ret;
	g5p_boot_snapshot(g, release ? "before-release" : "before-arm",
			  !g->reset_transition && !b->failed ?
				  G5P_READBACK_BASELINE :
				  G5P_READBACK_WRAPPER_ONLY);
	;
	ret = 0;
	if (ret)
		goto out;
	/* Slave-port access remains forbidden for the rest of this boot. */
	g->reset_transition = true;
	/* c9fa10: core request false, GPC 0. Establish an actual state edge;
	 * wake left GPC at 1, so writing 1 again did not prove FIFO execution.
	 * No R0 accesses between this point and the successful active-state poll.
	 */
	dev_info(g->dev, "G2 Fender park: core=0, GPC=0\n");
	ret = g5p_fender_idle(&io);
	if (ret)
		goto out;
	/* c9fc10: core request true before prepareBoot c9feb4. */
	dev_info(g->dev, "G2 Fender prepare: core=20, require GPC=0\n");
	ret = g5p_fender_hold(&io);
	if (ret)
		goto out;
	dev_info(g->dev,
		 "G2 init stream: master boot, release=%u; FIFO reset\n",
		 release);
	ret = g5p_fender_write(g, 0x10, 0);
	if (ret)
		goto out;
	/* setMode(3), after Fender prepare cleared +10, before firmware runs.
	 * Both arm and release execute prepare; never reapply after ALIVE.
	 */
	;
	;
	;
	g5p_fender_write(g, 0, 0x40000000);
	;
	;
	b->fifo = 0x10000;
	b->fifo_error = 0;
	;
	;
	g5p_fifo64(g, 0x100,
		   G5P_BOOT_RESET1 | (0 ? G5P_BOOT_R46_RESET_BITS : 0));
	;
	;
	g5p_fifo64(g, 0x100, 0x00e7fe071ffffc1dULL);
	;
	;
	g5p_fifo64(g, 0x100, 0x100000000ULL);
	;
	;
	g5p_fifo32(g, 0xbf8, 1, 0xff);
	;
	;
	g5p_fifo32(g, 0xe140, 0, 0xff);
	;
	;
	g5p_fifo64(g, 0xe148, b->catalogue.dma >> 12);
	;
	;
	g5p_fifo64(g, 0xe200, 3);
	;
	;
	g5p_fifo64(g, 0xe208, 0xb42dd24b4bd22db4ULL);
	;
	;
	g5p_fifo64(g, 0xe210, 0xb42dd24b4bd22db4ULL);
	;
	;
	g5p_fender_write(g, 0x2010, 0xbf);
	;
	;
	g5p_fender_write(g, 0x2014, 0x233);
	;
	;
	g5p_fifo64(g, 0xe160, b->zero.dma);
	;
	;
	g5p_fifo64(g, 0xbb8, 0);
	;
	;
	if (release) {
		g5p_fifo64(g, 0x100, 0);
		;
	}
	g5p_fifo_word(g, 0xffff0000);
	;
	;
	ret = b->fifo_error;
	if (ret)
		goto out;
	dma_wmb();
	dev_info(
		g->dev,
		"G2 FIFO stored and verified: %u words (not execution proof)\n",
		(b->fifo - 0x10000) / 4);
	if (release) {
		WRITE_ONCE(b->released, true);
		g5p_r73b_crumb(b, G5P_R73B_RELEASE_FIFO, G5P_R73B_NONE, 0);
		ret = 0;
		if (ret)
			goto out;
	}
	/* c9f954: core request false, then GPC 1. */
	dev_info(g->dev, "G2 Fender execute: core=0, GPC=1 (from 0)\n");
	if (release && !(b->r73b && 1)) {
		g5p_perf_seed_stamp(
			b,
			4); /* the non-r74e start: as close as this path gets */
		if (b->seed_stamp)
			b->seed_gpc1 = arch_timer_read_counter();
	}
	ret = release && b->r73b && 1 ? g5p_r74e_fender_start(g) :
	      release ? g5p_fender_start_observed_q(&io, g5p_release_observe,
						    b->perf_poweron ?: 1) :
			g5p_fender_start(&io);
	if (release)
		g5p_r73b_crumb(b, G5P_R73B_RELEASE_GPC, G5P_R73B_NONE, ret);
	if (ret)
		goto out;
	/* r74e: after GPC1 is active, wait for ALIVE without optional MMIO. */
	if (release && b->r73b && 1)
		return 0;
	usleep_range(10, 20);
	g5p_boot_snapshot(g, release ? "after-release" : "after-arm",
			  release ? G5P_READBACK_WRAPPER_ONLY :
				    G5P_READBACK_RESET_HELD);
	;
	/* Only the fault-status subset above is sampled after release. */
	if (release)
		return g5p_release_ack(g);
	if (!b->rgx_valid)
		return -EIO;
	reset = g5p_snapshot64(b, 0x100);
	dev_info(g->dev, "G2 verify reset actual=%016llx expected=%016llx\n",
		 reset, 0x100000000ULL);
	dev_info(
		g->dev,
		"G2 expected catalogue=%llx scratch=%llx SLC=3 (not read: state safety)\n",
		(u64)b->catalogue.dma >> 12, (u64)b->zero.dma);
	if (reset != 0x100000000ULL)
		return -EIO;
	dev_info(
		g->dev,
		"G2 reset-held gate verified; remaining FIFO effects await firmware event\n");
	b->armed = true;
	return 0;
out:
	/* A slow r74e epoch is scoreable without another unsafe MMIO read. */
	if (release && b->r73b && 1 && READ_ONCE(b->r73b->early_power_abort))
		return ret;
	/* Wrapper-only snapshot if GPC remains off/busy. */
	g5p_boot_snapshot(g, "transition-or-FIFO-failed",
			  G5P_READBACK_WRAPPER_ONLY);
	return ret;
}

static int g5p_boot_arm(struct g5p_diag *g)
{
	return g5p_boot_program_stream(g, false);
}

static int g5p_boot_load(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u32 off;

	/* c9f1bc/ca43b0: segment and LDR configuration pairs are supplied in
	 * mapped RAM. Polling MSLVCTRL1 in the reset-held state locks this SoC.
	 */
	for (off = 0; off < G5P_IMAGE_SIZE; off += G5P_GPU_PAGE)
		if (memcmp(b->image[off / G5P_GPU_PAGE].cpu,
			   g->image.data + off, G5P_GPU_PAGE))
			return -EBADMSG;
	dma_wmb();
	b->copied = true;
	;
	dev_info(
		g->dev,
		"G2 RAM load verified bytes=%x segment_pairs=13 LDR_pairs=27; no slave-port MMIO\n",
		G5P_IMAGE_SIZE);
	return 0;
}

static int g5p_boot_release(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	u64 seed_ts = 0;
	int ret;

	if (b->released)
		return -EALREADY;
	/* recovery108: fail closed before the release touches anything. */
	ret = g5p_ds_hook_release_gate(g);
	if (ret)
		return ret;
	g5p_r73b_release_entry(g);
	if (b->r73b && 1) {
		ret = g5p_r73e_prepare(g);
		if (ret)
			return ret;
	}
	dev_info(g->dev,
		 "G2 release: META master boot; no GPU job submitted\n");
	/* Reproduce the complete original stream, including its final reset release.
	 * Do not assume the experimental arm stage preserves a FIFO read cursor.
	 */
	/* boot_gpu_perf_seed 3: the sampler's last-sample time = now, as close to the release as the
	 * host gets (perf-envelope.h); the firmware divides (now - policy+0x8f8) by 24000 * policy+4.
	 */
	if (b->perf_seed_mode >= 3) {
		seed_ts = arch_timer_read_counter();
		g5p_perf_seed_stamp(b, 3);
	}
	ret = g5p_boot_program_stream(g, true);
	if (seed_ts) {
		u64 end = arch_timer_read_counter(),
		    rate = arch_timer_get_rate();

		/* always printed on a seed 3/4 boot, before ALIVE is awaited */
		dev_emerg(
			g->dev,
			"GPUPERF seed mode=%u stamp=%llx release_to_gpc1_us=%llu stamp_to_gpc1_us=%llu "
			"gpc1_to_stream_end_us=%llu stamp_to_stream_end_us=%llu period_us=16000 "
			"before_alive=1 end=gpuperf\n",
			b->perf_seed_mode, b->seed_stamp,
			b->seed_gpc1 ?
				div_u64((b->seed_gpc1 - seed_ts) * 1000000ULL,
					rate) :
				0,
			b->seed_gpc1 ? div_u64((b->seed_gpc1 - b->seed_stamp) *
						       1000000ULL,
					       rate) :
				       0,
			b->seed_gpc1 ?
				div_u64((end - b->seed_gpc1) * 1000000ULL,
					rate) :
				0,
			div_u64((end - b->seed_stamp) * 1000000ULL, rate));
	}
	g5p_r73b_release_exit(g, ret);
	g5p_ds_hook_release(g, ret);
	if (ret)
		return ret;
	if (b->r73b && 1)
		return g5p_r73c_fast_control(g);
	return 0;
}

static int g5p_boot_events_once(struct g5p_diag *g)
{
	struct g5p_boot *b = g->boot;
	unsigned long deadline = jiffies + 3 * HZ;
	u64 submit_wait_start = 0;
	u32 pass = 0, submit_wait_passes = 0;
	int ret;

	do {
		if (READ_ONCE(b->failed))
			return -EIO;
		ret = g5p_r73b_service(g, G5P_R73B_OWNER_EVENTS,
				       g5p_r73b_sampled(++pass));
		if (ret)
			return ret;
		if (READ_ONCE(b->alive)) {
			if (b->r73b && 1) {
				ret = g5p_r73e_alive_submit(
					g, G5P_R73B_OWNER_EVENTS);
				if (ret == -EAGAIN)
					submit_wait_start =
						ktime_get_mono_fast_ns();
				while (ret == -EAGAIN &&
				       ktime_get_mono_fast_ns() -
						       submit_wait_start <
					       100ULL * NSEC_PER_MSEC) {
					usleep_range(50, 100);
					ret = g5p_r73e_alive_submit(
						g, G5P_R73B_OWNER_EVENTS);
					submit_wait_passes++;
				}
				if (submit_wait_passes)
					dev_info(
						g->dev,
						"R73C alive-submit synchronize wait_ns=%llu passes=%u ret=%d submitted=%u end=r73c\n",
						ktime_get_mono_fast_ns() -
							submit_wait_start,
						submit_wait_passes, ret,
						READ_ONCE(
							b->control_submitted));
				if (ret)
					return ret;
			}
			dev_info(
				g->dev,
				"G2 ALIVE type=0 word1=%08x events=%u IRQ174=%d\n",
				b->alive_word, b->events,
				atomic_read(&b->irqs));
			g5p_r41_bridge_firmware(g);
			return 0;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	dev_err(g->dev,
		"G2 no alive event: events=%u IRQ174=%d; preserve baseline\n",
		b->events, atomic_read(&b->irqs));
	return -ETIMEDOUT;
}

/* recovery108: every ALIVE wait (release fast path and the events command) is an ALIVE event. */
static int g5p_boot_events(struct g5p_diag *g)
{
	int ret = g5p_boot_events_once(g);

	/* wake112: no ALIVE wait ends with an R74E proof still owed (r74e112.h). */
	if (g5p_r74e_close(g)) {
		dev_emerg(
			g->dev,
			"R74E112 close gen=%u why=untaken ret=%d end=r74e112\n",
			g->boot->r73b->early_proof_gen, ret);
		if (!ret)
			ret = -EHOSTDOWN;
	}
	g5p_ds_hook_alive(g, ret);
	return ret;
}

static int g5p_boot_command(struct g5p_diag *g, const char *command)
{
	struct g5p_boot *b = g->boot;
	int ret;

	if (b && b->stopped)
		return -ESHUTDOWN;
	if (sysfs_streq(command, "map"))
		return g5p_boot_maps(g);
	/* Read-only diagnostics remain available after a latched boot failure. */
	if (b && g->powered && g->fender_ready && g->id_checked) {
		if (sysfs_streq(command, "snapshot")) {
			g5p_boot_snapshot(g, "explicit",
					  !g->reset_transition && !b->failed ?
						  G5P_READBACK_BASELINE :
						  G5P_READBACK_WRAPPER_ONLY);
			return 0;
		}
		if (sysfs_streq(command, "fifo"))
			return g5p_boot_fifo_dump(g);
	}
	if (!boot_experiment || !b || !b->maps_ready || b->failed)
		return -EPERM;
	if (sysfs_streq(command, "arm")) {
		ret = g5p_boot_arm(g);
	} else if (sysfs_streq(command, "load") && b->armed && !b->released &&
		   !b->copied) {
		ret = g5p_boot_load(g);
		g5p_ds_hook_load(g, ret);
	} else if (sysfs_streq(command, "release") && b->copied) {
		ret = g5p_boot_release(g);
	} else if (sysfs_streq(command, "events") && b->released) {
		ret = g5p_boot_events(g);
		g5p_r73b_events_exit(g, ret);
	} else {
		return -EPERM;
	}
	if (ret && ret != -EALREADY)
		b->failed = true;
	return ret;
}
