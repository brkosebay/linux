/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_USER_LAYOUT_H
#define G5P_RENDER_USER_LAYOUT_H

#include "bits-compat.h"
/* Parameterized forms of the r22 fixed constructors for userspace render.
 * Pure byte builders: no allocation, MMIO, locking or firmware publication.
 * With r22's inputs every builder reproduces render-layout-r22.h exactly;
 * tools/gpu/test_render_user.c checks that equivalence byte for byte.
 */
#include "render-layout-r22.h"
#include "render-variants-r25.h"
#include "render-user-template.h"

/* Kernel-private window in every render VM. r22's aux/RT addresses are the
 * first allocations, so a 64x64 target keeps the fixed path's W anchors.
 * Root entry 0 carries the boot VA0 sink, so user maps start at 1 GiB.
 */
#define G5P_RU_PRIVATE_VA G5P_R22_AUX
#define G5P_RU_PRIVATE_SIZE 0x1000000ULL
#define G5P_RU_USER_VA_MIN 0x40000000ULL
/* Catalogue entry of the firmware GiB, borrowed from the boot tables. */
#define G5P_RU_FIRMWARE_CI ((G5P_IMAGE_VA >> 30) & 1023)
#define G5P_RU_AUX_BYTES 0x10680U
#define G5P_RU_AUX_PAGES 5U
#define G5P_RU_RT_ALIGN 0x10000ULL
#define G5P_RU_MAX_BLOCKS 128U
#define G5P_RU_BLOCK_BYTES 0x20000U
#define G5P_RU_MAX_HEADER 0x80000U
#define G5P_RU_MAX_SECONDARY 0x100000ULL
#define G5P_RU_TA_W_BYTES 0x560U
#define G5P_RU_3D_W_BYTES 0x8c0U
#define G5P_RU_CMD_BYTES 0x40U

/* Kernel mirror of drm_g5p_rt_geometry (firmware round16 R-offsets). */
struct g5p_ru_rt {
	u32 header, region; /* R10, R20 */
	u64 packed; /* Rb0 */
	u32 state, padded; /* Rb8, Rbc */
	u64 scale[3]; /* Rc0, Rc8, Rd0 */
	u64 ta_state[2]; /* Rf0, Rf8 */
	u64 ta_tiling[2]; /* R100, R108 */
	u32 ta_tiling_word, tiles; /* R110, R118 */
	u64 secondary, secondary_align; /* R138, R178 */
};

/* 6ca6d5c executed offline for 64,64,1,8,8 (round16 section 3). */
static const struct g5p_ru_rt g5p_ru_rt64 = {
	.header = 0x500,
	.region = 0x50,
	.packed = 0x88,
	.state = 0,
	.padded = 0x40004,
	.scale = { 0x3b89309c3b89309cULL, 0x3cddb3d93cddb3d9ULL,
		   0x4480000044800000ULL },
	.ta_state = { 0, 0x88 },
	.ta_tiling = { 0x00001001003f003fULL, 0x0010100c0010100cULL },
	.ta_tiling_word = 0x10,
	.tiles = 4,
	.secondary = 0x800,
	.secondary_align = 0x40,
};

#include "render-user-geometry.h"

struct g5p_ru_rt_sizes {
	u64 slice, main, secondary_map;
};

/* 6cb3d68: slice=R10 rounded to 4KiB; main ring is the power-of-two capacity
 * for sixteen slices. 6cb40e0 places the secondary extent at offset 0 of
 * its own mapping. Bounds are Linux allocation policy, not hardware limits.
 */
static inline int g5p_ru_rt_sizes(const struct g5p_ru_rt *rt,
				  struct g5p_ru_rt_sizes *out)
{
	u64 main = 0x1000;

	if (!rt->header || rt->header > G5P_RU_MAX_HEADER || !rt->secondary ||
	    rt->secondary > G5P_RU_MAX_SECONDARY || !rt->secondary_align ||
	    (rt->secondary_align & (rt->secondary_align - 1)) ||
	    rt->secondary_align > G5P_GPU_PAGE || !rt->tiles)
		return -EINVAL;
	out->slice = ((u64)rt->header + 0xfff) & ~0xfffULL;
	while (main < out->slice * 16)
		main <<= 1;
	out->main = main < G5P_GPU_PAGE ? G5P_GPU_PAGE : main;
	out->secondary_map = (rt->secondary + G5P_GPU_PAGE - 1) &
			     ~(u64)(G5P_GPU_PAGE - 1);
	return 0;
}

/* 6cba480/6c7b64c anchors. Native mapping dialect: L=H, Laux=Haux.
 * Equal to g5p_r22_rt() for g5p_ru_rt64 at the r22 addresses. 3D W58 and
 * Wa8 are the importer's literal Rb0 and R20>>2 for the verified geometry.
 */
static inline void g5p_ru_rt_fill(u8 *ta, u8 *td, const struct g5p_ru_rt *rt,
				  u64 main_va, u64 secondary_va)
{
	g5p_put64(ta + 0x28, main_va);
	g5p_put64(ta + 0x30, secondary_va);
	g5p_put64(ta + 0x38, rt->ta_state[0]);
	g5p_put64(ta + 0x40, rt->ta_state[1]);
	g5p_put64(ta + 0x444, secondary_va);
	g5p_put32(ta + 0x44c, rt->secondary);
	g5p_put64(ta + 0x3ac, rt->ta_tiling[0]);
	g5p_put64(ta + 0x3b4, rt->ta_tiling[1]);
	g5p_put32(ta + 0x3bc, rt->ta_tiling_word);
	g5p_put64(td + 0x20, main_va);
	g5p_put64(td + 0x28, rt->packed);
	g5p_put32(td + 0x30, rt->state);
	g5p_put32(td + 0x34, rt->padded);
	g5p_put32(td + 0x38, rt->region);
	g5p_put64(td + 0x3c, rt->scale[0]);
	g5p_put64(td + 0x44, rt->scale[1]);
	g5p_put64(td + 0x4c, rt->scale[2]);
	g5p_put64(td + 0x58, rt->packed);
	g5p_put32(td + 0xa8, rt->region >> 2);
	g5p_put32(td + 0x3a0, rt->padded);
	g5p_put64(td + 0x3d0, rt->scale[0]);
	g5p_put64(td + 0x3d8, rt->scale[1]);
	g5p_put32(td + 0x3e0, rt->scale[2]);
	g5p_put32(td + 0x3e4, rt->scale[0]);
}

/* Validated typed render inputs, already range-checked by the caller. */
struct g5p_ru_pass {
	u64 vdm_va;
	u32 vdm_size, vdm_control;
	u64 scratch_va, scratch_size, scratch_ptr[6];
	u64 pool7_va, pool7_size, pool8_va, pool8_size;
	u32 context_pds[2][6], resume_pds[2][4], ta_pds[4];
	u32 background[8], eot[4], eot_resume[4], restore[4];
};

/* Importer chains 6cba8a0/6c7b100/6c7b34c -> 6ca9710/6ca7ae0 (offline
 * differential map in make-render-user-template.py). Every user VA field
 * and code-relative descriptor lands here; the rest of W is the frozen
 * dialect-1 scalar table or a kernel RT/channel field.
 */
static inline void g5p_ru_fill_ta(u8 *w, const struct g5p_ru_pass *p)
{
	memset(w, 0, G5P_RU_TA_W_BYTES);
	for (u32 i = 0; i < ARRAY_SIZE(g5p_ru_ta_fixed); i++)
		g5p_put32(w + g5p_ru_ta_fixed[i][0], g5p_ru_ta_fixed[i][1]);
	g5p_put64(w + 0x48, p->scratch_ptr[4]);
	g5p_put64(w + 0x50, p->scratch_ptr[5]);
	for (u32 i = 0; i < 6; i++) {
		g5p_put32(w + 0x58 + i * 4, p->context_pds[0][i]);
		g5p_put32(w + 0x70 + i * 4, p->context_pds[1][i]);
	}
	g5p_put64(w + 0x88, p->vdm_control & ~0xfULL);
	g5p_put64(w + 0x90, p->scratch_va & ~7ULL);
	g5p_put64(w + 0x98, p->vdm_va & ~3ULL);
	for (u32 i = 0; i < 4; i++) {
		g5p_put32(w + 0xa0 + i * 4, p->resume_pds[0][i]);
		g5p_put32(w + 0xb0 + i * 4, p->resume_pds[1][i]);
	}
	for (u32 i = 0; i < 3; i++)
		g5p_put32(w + 0xc0 + i * 4, p->ta_pds[i]);
	for (u32 i = 0; i < 4; i++)
		g5p_put64(w + 0x468 + i * 8, p->scratch_ptr[i]);
	g5p_put64(w + 0x488, p->scratch_ptr[4]);
}

static inline void g5p_ru_fill_3d(u8 *w, const struct g5p_ru_pass *p)
{
	memset(w, 0, G5P_RU_3D_W_BYTES);
	for (u32 i = 0; i < ARRAY_SIZE(g5p_ru_3d_fixed); i++)
		g5p_put32(w + g5p_ru_3d_fixed[i][0], g5p_ru_3d_fixed[i][1]);
	g5p_put64(w + 0x60, p->pool8_va & ~3ULL);
	g5p_put64(w + 0x68, p->pool7_va & ~3ULL);
	g5p_put64(w + 0x490, p->pool7_va);
	g5p_put64(w + 0x4a0, p->pool8_va);
	for (u32 i = 0; i < 8; i++)
		g5p_put32(w + 0x5b8 + i * 4, p->background[i]);
	g5p_put32(w + 0xb0, p->background[0]);
	g5p_put32(w + 0xb4, p->background[1]);
	for (u32 i = 4; i < 8; i++)
		g5p_put32(w + 0xc0 + (i - 4) * 4, p->background[i]);
	/* 6ca7ae0 swaps the first EOT qword's halves in its B upload. */
	g5p_put32(w + 0x3b4, p->eot[1]);
	g5p_put32(w + 0x3b8, p->eot[0]);
	g5p_put32(w + 0x3bc, p->eot[2]);
	for (u32 i = 0; i < 3; i++) {
		g5p_put32(w + 0x680 + i * 4, p->eot[i]);
		g5p_put32(w + 0x694 + i * 4, p->eot_resume[i]);
	}
	g5p_put32(w + 0x600, p->restore[0]);
	g5p_put32(w + 0x604, p->restore[1]);
	g5p_put32(w + 0x610, p->restore[2]);
	g5p_put32(w + 0x614, p->restore[3]);
}

/* TYPED_V2 additions, already validated by g5p_ru_pass2_check(). */
struct g5p_ru_pass2 {
	u64 scissor_va, dbias_va, background_texture, zls[6];
	u32 scissor_count, dbias_count, bgobj_depth, bgobj_vals;
	u32 pixel_width, zls_mask;
};

/* Dialect-2 words over g5p_ru_fill_3d() (TYPED_V2 has zero pools). Each
 * register image goes to its B-upload word and to the normal builder's
 * copy (6c7a06c); make-render-user-dialect2.py proves every position by
 * perturbing record 2d8/2e0 (pools), 2f0/310/338/360/388/308 (ZLS), 508
 * (background texture) and 4d0 (clears) through the r22 importer.
 * Pixel output: 6ca7ae0 for D2f8 = width, one sample (D6e0 = 0), D630 =
 * D6b2 = 0, six cores. Widths 2/4/8 keep two or more partitions, so W3c0
 * and W3c4 keep their dialect-1 values; width 2 is dialect 1 exactly.
 */
static inline void g5p_ru_fill_3d_width(u8 *w, u32 width)
{
	/* 6ca7ae0's output-width code table 5942678, by D2f8 - 2. */
	static const u8 width_code[7] = { 0, 0, 0x10, 0, 0, 0, 0x20 };
	u32 count = 0x4000 / (width << 10);
	u32 code = width_code[width - 2];

	g5p_put32(w + 0x3c8, count & 0x1f);
	g5p_put32(w + 0x3cc, code >> 4 | 0xf8);
	for (u32 i = 0; i < 6; i++)
		g5p_put64(w + 0x108 + i * 8, code | (count - 1) << 7 | 0x43);
	g5p_put32(w + 0x6b0, width);
}

static inline void g5p_ru_fill_3d_v2(u8 *w, const struct g5p_ru_pass2 *e)
{
	static const u16 zls_w[6][2] = {
		{ 0x80, 0x618 }, { 0x88, 0x628 }, { 0x90, 0x630 },
		{ 0x98, 0x648 }, { 0xa0, 0x650 }, { 0x138, 0x6d0 },
	};

	g5p_put64(w + 0x60, e->scissor_va & ~3ULL);
	g5p_put64(w + 0x4a0, e->scissor_va);
	g5p_put64(w + 0x68, e->dbias_va & ~3ULL);
	g5p_put64(w + 0x490, e->dbias_va);
	g5p_put64(w + 0xb8, e->background_texture);
	g5p_put64(w + 0x5f0, e->background_texture);
	g5p_put32(w + 0x3a4, e->bgobj_depth);
	g5p_put32(w + 0x6a8, e->bgobj_depth);
	g5p_put32(w + 0x3a8, e->bgobj_vals);
	g5p_put32(w + 0x6ac, e->bgobj_vals);
	for (u32 i = 0; i < 6; i++) {
		g5p_put64(w + zls_w[i][0], e->zls[i]);
		g5p_put64(w + zls_w[i][1], e->zls[i]);
	}
	g5p_ru_fill_3d_width(w, e->pixel_width);
}

/* TYPED_V3 additions, already validated by g5p_ru_pass3_check(). */
struct g5p_ru_pass3 {
	u64 ta_program_va;
	u32 ta_program_size, vdm_end;
	u32 pixel_width, pad; /* V1 dialect output width, 0 = 2 */
};

/* Dialect-3 word over g5p_ru_fill_ta(): W+0x88 (-> R0+0x470) is the exact
 * byte offset of the TA PDS program from pds_base, a plain 64-bit store with
 * no mask. g5p_ru_fill_ta() leaves it zero for a V3 pass (vdm_control is
 * zero), so V1/V2 bytes are untouched; the stream length never reaches W.
 */
static inline void g5p_ru_fill_ta_v3(u8 *w, u64 program_off)
{
	g5p_put64(w + 0x88, program_off);
}

/* 6cb2378 selected state S at control+80: explicit zeros plus the four
 * recovered backing pointers. N=1, PM ID0/record0. aux_va covers 10680.
 */
static inline int g5p_ru_pm_state(u8 *control, u64 aux_va, u64 list_va)
{
	memset(control + 0x80, 0, 0xc0);
	return g5p_pm_record_backing(control + 0x80, 0, 1, aux_va, 0x80,
				     0x10000, 0x600,
				     g5p_meta_address(list_va + 0x40));
}

/* 6cb124c/13f4/14dc pool for `blocks` resident = maximum blocks. Rewrites
 * control+0..7f, the whole list and F; S at control+80 is not touched.
 * physical[] holds blocks*8 16KiB PAs. Validate everything before writing.
 */
static inline int g5p_ru_pm_check(u64 list_va, u64 free_va, const u64 *physical,
				  u32 blocks)
{
	struct g5p_pm_sizes sizes;
	int ret;

	if (!blocks || blocks > G5P_RU_MAX_BLOCKS)
		return -EINVAL;
	ret = g5p_pm_sizes(1, blocks, 0x80, 0x10000, 0x600, &sizes);
	if (ret)
		return ret;
	if ((list_va | free_va) & (G5P_GPU_PAGE - 1) ||
	    list_va >= (1ULL << 40) || free_va >= (1ULL << 40) ||
	    0x80 + blocks * 0x24 > G5P_GPU_PAGE ||
	    blocks * 32 * 4 > G5P_GPU_PAGE)
		return -EINVAL;
	for (u32 i = 0; i < blocks * 8; i++)
		if ((physical[i] & (G5P_GPU_PAGE - 1)) ||
		    physical[i] >= (1ULL << 40))
			return -ERANGE;
	return 0;
}

static inline int g5p_ru_pm_pool(u8 *control, u8 *list, u8 *free, u64 list_va,
				 u64 free_va, const u64 *physical, u32 blocks)
{
	u32 previous = 0;
	int ret = g5p_ru_pm_check(list_va, free_va, physical, blocks);

	if (ret)
		return ret;
	memset(control, 0, 0x80);
	memset(list, 0, G5P_GPU_PAGE);
	memset(free, 0, G5P_GPU_PAGE);
	for (u32 j = 0; j < blocks; j++) {
		u32 at = 0x80 + j * 0x24;

		g5p_put32(list + at, previous);
		for (u32 k = 0; k < 8; k++) {
			u32 ppn = physical[j * 8 + k] >> 12;

			g5p_put32(list + at + 4 + k * 4, ppn);
			for (u32 p = 0; p < 4; p++)
				g5p_put32(free + ((j * 8 + k) * 4 + p) * 4,
					  ppn + p);
		}
		previous = g5p_meta_address(list_va + at);
	}
	g5p_put32(list + 0x60, 1);
	g5p_put32(control + 0x10, previous);
	g5p_put64(control + 0x14, free_va);
	g5p_put32(control + 0x1c, blocks * 32 * 4);
	g5p_put32(control + 0x20, blocks * 32);
	g5p_put32(control + 0x24, blocks * 32 - 1);
	g5p_put32(control + 0x34, G5P_GPU_PAGE);
	g5p_put32(control + 0x40, g5p_meta_address(list_va));
	g5p_put32(control + 0x54, blocks * 32);
	g5p_put32(control + 0x58, blocks * 32);
	return 0;
}

/* Complete fresh PM object: g5p_r22_pm() when blocks=32, aux=G5P_R22_AUX. */
static inline int g5p_ru_pm(u8 *control, u8 *list, u8 *free, u64 list_va,
			    u64 free_va, const u64 *physical, u32 blocks,
			    u64 aux_va)
{
	int ret = g5p_ru_pm_check(list_va, free_va, physical, blocks);

	if (ret)
		return ret;
	if ((aux_va & (G5P_GPU_PAGE - 1)) ||
	    aux_va > (1ULL << 40) - G5P_RU_AUX_PAGES * G5P_GPU_PAGE)
		return -EINVAL;
	memset(control, 0, G5P_GPU_PAGE);
	ret = g5p_ru_pm_pool(control, list, free, list_va, free_va, physical,
			     blocks);
	return ret ?: g5p_ru_pm_state(control, aux_va, list_va);
}

/* The r25 latched selections that can still yield a frame. Context/host/SKU
 * probes and the heap-relative RT dialect are diagnostic-only in the fixed
 * path (pair rejected) and are not offered here; blocks is pm_blocks.
 */
static inline bool g5p_ru_r25_valid(const struct g5p_r25_variants *v)
{
	return g5p_r25_valid(v) && !g5p_r25_diagnostic(v) && !v->blocks;
}

/* r25 render_pm_variant 2/3 on a pool from g5p_ru_pm()/g5p_ru_pm_pool():
 * reverse only F's block order, or report F's whole mapped page as its
 * extent. Equal to g5p_r25_pool() for 32, 64 and 128 blocks.
 */
static inline void g5p_ru_pm_variant(u8 *control, u8 *free, const u64 *physical,
				     u32 blocks, u32 variant)
{
	if (variant == 2)
		for (u32 j = 0; j < blocks; j++)
			for (u32 k = 0; k < 8; k++) {
				u32 ppn = physical[(blocks - 1 - j) * 8 + k] >>
					  12;

				for (u32 p = 0; p < 4; p++)
					g5p_put32(free + ((j * 8 + k) * 4 + p) *
								  4,
						  ppn + p);
			}
	if (variant == 3)
		g5p_put32(control + 0x1c, G5P_GPU_PAGE);
}

/* r25 render_pm_variant 1, after g5p_ru_job() built the TA page: tag6+10
 * names an owned holder with the linked head (outside CH/ring/W/SKU/TQ/
 * dependency); F/count/head start clear and last=ffffffff, so 10005368
 * rebuilds them. Equal to g5p_r25_inputs() for the same pool.
 */

/* Persistent per-channel objects (6c89420 layout): written once at attach.
 * Control indices, TQ and the dependency object stay firmware-owned after.
 */
static inline void g5p_ru_channel_init(u8 *p, u32 meta)
{
	memset(p, 0, G5P_GPU_PAGE);
	g5p_put32(p, meta + G5P_WORK_CTL);
	g5p_put32(p + 4, meta + G5P_WORK_RING);
	g5p_put32(p + 8, meta + G5P_R22_TQ);
	g5p_put32(p + 0xc, 4);
	g5p_put32(p + 0x18, ~0U);
	g5p_put32(p + G5P_WORK_CTL + 0x50, G5P_WORK_SLOTS);
	g5p_put32(p + G5P_R22_TQ + 4, meta + G5P_R22_TQ);
	g5p_put64(p + G5P_R22_DEP + 0x30, 1);
	g5p_put64(p + G5P_R22_DEP + 0x48, ~0ULL);
	g5p_put32(p + G5P_R22_DEP + 0x50, ~0U);
	for (u32 off = 0x70; off < 0x270; off += 0x10)
		g5p_put32(p + G5P_R22_DEP + off + 8, 0xf);
}

/* Pointer-ring positions of job `seq` (1-based): two entries per job. */
static inline u32 g5p_ru_ring_index(u32 seq)
{
	return (2 * (seq - 1)) % G5P_WORK_SLOTS;
}

static inline u32 g5p_ru_endpoint(u32 seq)
{
	return (2 * seq) % G5P_WORK_SLOTS;
}

/* perf60 job slots (render_user_job_slots, drm-render-v3.inc).  A job's command, W record and
 * microsequence ("SKU") live in slot 0 (the r22 offsets) or slot 1, in the channel page's free
 * tail; the firmware reaches both through the job's two pointer-ring entries.  The channel header,
 * control, pointer ring, TQ, stamps and dependency object stay shared.  Slot 1 clears TQ/STAMP/DEP
 * (0x1800..0x1b70) and the r25 holder (G5P_R25_HEAD, 0x3000).
 */
#define G5P_RU_SLOT1 0x1c00U
#define G5P_RU_SLOT1_END \
	(G5P_RU_SLOT1 + G5P_RU_CMD_BYTES + G5P_RU_3D_W_BYTES + 0x180U)

static inline u32 g5p_ru_cmd_at(u32 js)
{
	return js ? G5P_RU_SLOT1 : G5P_WORK_CMD;
}

static inline u32 g5p_ru_w_at(u32 js)
{
	return js ? G5P_RU_SLOT1 + G5P_RU_CMD_BYTES : G5P_R22_W;
}

static inline u32 g5p_ru_sku_at(u32 js, bool td)
{
	if (js)
		return G5P_RU_SLOT1 + G5P_RU_CMD_BYTES + G5P_RU_3D_W_BYTES;
	return td ? G5P_R22_3D_SKU : G5P_R22_TA_SKU;
}

/* Per-job objects for sequence `seq`: W image (fixed+typed+RT), channel W
 * fields, the full timestamped microsequence, the tag6 PM bootstrap (TA) or
 * tag4 TA-finish dependency (3D) and both pointer-ring entries. With seq=1
 * this equals g5p_r22_channel(). Stamp target and sequence are both seq.
 * ta_finish is M(TA stamp word addressed by TA.W530), used by 3D only.
 */
static inline void g5p_ru_job_at(u8 *p, u32 meta, u32 global, u32 pm, u32 slot,
				 bool td, u32 seq, const u8 *image,
				 u32 ta_finish, bool zero_timestamps, u32 js,
				 u32 pmid)
{
	u32 w = meta + g5p_ru_w_at(js), state = pm + 0x80;
	u32 finish = meta + G5P_R22_STAMP + 4;
	u32 sku = g5p_ru_sku_at(js, td), cmd = g5p_ru_cmd_at(js);
	u32 ring = g5p_ru_ring_index(seq);
	u8 *work = p + g5p_ru_w_at(js);
	u32 cursor = sku, starts, finishes;

	memcpy(work, image, td ? G5P_RU_3D_W_BYTES : G5P_RU_TA_W_BYTES);
	memset(p + sku, 0, td ? 0x180 : 0xc0);
	g5p_put32(work, td ? 1 : 0);
	g5p_put32(work + 4, slot);
	g5p_put32(work + (td ? 0x14 : 0xc), meta + G5P_R22_DEP);
	g5p_put32(work + (td ? 0x18 : 0x14), pm);
	if (!td && pmid)
		g5p_put32(work + 0x10,
			  pmid); /* TA W+10: the PM identity (iOS Dcb0) */
	g5p_put32(work + (td ? 0x1c : 0x18), state);
	g5p_put32(work + (td ? 0xc : 0x454), meta + sku);
	g5p_put32(work + (td ? 0x10 : 0x458), td ? 0x180 : 0xc0);
	g5p_put32(work + (td ? 0x888 : 0x52c), finish - 4);
	g5p_put32(work + (td ? 0x88c : 0x530), finish);
	g5p_put32(work + (td ? 0x890 : 0x534), seq);
	g5p_put32(work + (td ? 0x894 : 0x538), td ? 1 : 0);
	g5p_put32(work + (td ? 0x898 : 0x53c), td ? 1 : 0);
	g5p_put32(work + (td ? 0x8a0 : 0x544), seq);
	{
		u32 ta_start[] = {
			0x23,
			w + 0x3a4,
			w + 0x20,
			state,
			global + 0x724,
			meta,
			slot,
			0,
			pmid,
			0,
			0,
			0,
			w + 0x464,
			g5p_le32(work + 0x51c),
			g5p_le32(work + 0x520),
			0,
			seq,
		};
		u32 td_start[] = {
			0x25,
			w + 0x3a0,
			w + 0x58,
			state,
			global + 0x724,
			w + 0x7f0,
			w + 0x7f4,
			w + 0x878,
			meta,
			w,
			slot,
			0,
			pmid,
			g5p_le32(work + 0x800),
			0,
			1,
			w + 0x6d8,
			g5p_le32(work + 0x87c),
			g5p_le32(work + 0x884),
			0,
			seq,
		};
		u32 ta_finish[] = {
			0x24,
			state,
			pm,
			global + 0x724,
			meta,
			slot,
			0,
			w + 0x464,
			g5p_le32(work + 0x51c),
			g5p_le32(work + 0x520),
			seq,
			finish,
			seq,
			(u32)-0x70,
		};
		u32 td_finish[] = {
			0x26,
			seq,
			finish,
			seq,
			state,
			pm,
			1,
			global + 0x724,
			w + 0x878,
			w + 0x7f0,
			meta,
			w,
			slot,
			w + 0x6d8,
			0,
			g5p_le32(work + 0x87c),
			g5p_le32(work + 0x884),
			(u32)-0x104,
		};
		u32 result = w + (td ? 0x8ac : 0x550);
		u32 timestamp = w + (td ? 0x8b4 : 0x558);
		u32 ts0[] = { 0x80000018, result, timestamp, timestamp, seq };
		u32 ts1[] = { 0x18, result, timestamp, timestamp + 4, seq };

		starts = td ? ARRAY_SIZE(td_start) : ARRAY_SIZE(ta_start);
		finishes = td ? ARRAY_SIZE(td_finish) : ARRAY_SIZE(ta_finish);
		for (u32 i = 0; i < starts; i++, cursor += 4)
			g5p_put32(p + cursor, td ? td_start[i] : ta_start[i]);
		for (u32 i = 0; i < ARRAY_SIZE(ts0); i++, cursor += 4)
			g5p_put32(p + cursor, zero_timestamps ? 1 : ts0[i]);
		if (td) {
			for (u32 i = 0; i < ARRAY_SIZE(g5p_r22_wait_words);
			     i++, cursor += 4)
				g5p_put32(p + cursor, g5p_r22_wait_words[i]);
		} else {
			g5p_put32(p + cursor, 0x801);
			cursor += 4;
		}
		for (u32 i = 0; i < ARRAY_SIZE(ts1); i++, cursor += 4)
			g5p_put32(p + cursor, zero_timestamps ? 1 : ts1[i]);
		for (u32 i = 0; i < finishes; i++, cursor += 4)
			g5p_put32(p + cursor, td ? td_finish[i] : ta_finish[i]);
		g5p_put32(p + cursor, 0x40000017);
	}
	memset(p + cmd, 0, G5P_RU_CMD_BYTES);
	g5p_put32(p + cmd, td ? 4 : 6);
	g5p_put32(p + cmd + 4, td ? ta_finish : slot);
	g5p_put32(p + cmd + 8,
		  td ? seq : pmid); /* tag6 {6, STA, PMid, M(control)} */
	g5p_put32(p + cmd + 12, td ? seq & 0x7fffffff : pm);
	g5p_put32(p + G5P_WORK_RING + ring * 4, meta + cmd);
	g5p_put32(p + G5P_WORK_RING + ((ring + 1) % G5P_WORK_SLOTS) * 4, w);
}

/* Retirement of job `seq`: both W pointers still ours, finish word and
 * retirement mirror at seq, all five channel watermarks at its endpoint.
 * g5p_r22_retired() is the seq=1 case.
 */
static inline bool g5p_ru_retired_at(const u8 *p, u32 meta, bool td, u32 seq,
				     u32 js)
{
	const u8 *w = p + g5p_ru_w_at(js);
	u32 endpoint = g5p_ru_endpoint(seq);

	if (g5p_le32(w + (td ? 0x888 : 0x52c)) != meta + G5P_R22_STAMP ||
	    g5p_le32(w + (td ? 0x88c : 0x530)) != meta + G5P_R22_STAMP + 4 ||
	    g5p_le32(w + (td ? 0x890 : 0x534)) != seq ||
	    g5p_le32(p + G5P_R22_STAMP) != seq ||
	    g5p_le32(p + G5P_R22_STAMP + 4) != seq)
		return false;
	for (u32 off = 0; off <= 0x40; off += 0x10)
		if (g5p_le32(p + G5P_WORK_CTL + off) != endpoint)
			return false;
	return true;
}

/* A stamp beyond the published target, or a watermark not on the even
 * two-entry grid, is a protocol error rather than "not yet retired".
 */
static inline bool g5p_ru_stamp_sane(const u8 *p, u32 seq)
{
	for (u32 i = 0; i < 2; i++) {
		s32 ahead = (s32)(g5p_le32(p + G5P_R22_STAMP + i * 4) - seq);

		if (ahead > 0)
			return false;
	}
	for (u32 off = 0; off <= 0x40; off += 0x10)
		if (g5p_le32(p + G5P_WORK_CTL + off) >= G5P_WORK_SLOTS)
			return false;
	return true;
}

#endif
