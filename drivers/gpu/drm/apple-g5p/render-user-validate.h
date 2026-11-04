/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_USER_VALIDATE_H
#define G5P_RENDER_USER_VALIDATE_H

/* Pure TYPED_V1 checks over already-resolved resource ranges. The caller
 * proves each range is inside a mapping of the listed BO, in the submitting
 * context's VM, with BO roles covering `access`. Nothing here is a pointer
 * the kernel dereferences; GPU addresses are only range-checked.
 */
#define G5P_RU_ACCESS_READ 1U
#define G5P_RU_ACCESS_WRITE 2U
#define G5P_RU_MAX_VDM 0x10000U
#define G5P_RU_MAX_SCRATCH (64ULL << 20)

/* Hardware facts GET_RENDER_INFO reports and the TYPED_V3 validator enforces;
 * one definition so the two cannot drift. Code offsets are 32-bit from the
 * common USC/PDS base. TA/3D reads at or above 2^38 stall silently on A10
 * (works at 0x3_, 0x10_ and 0x20_, stalls at 0x40_ and 0x80_, no MMU fault);
 * not flown on A9, so the limit is a statement, not a proof (see
 * docs/gpu/experiments/typed-v3-kernel-implementation.md).
 */
#define G5P_RU_PDS_BASE 0x300000000ULL
#define G5P_RU_CODE_WINDOW 0x100000000ULL /* 2^32 */
#define G5P_RU_HW_VA_LIMIT 0x4000000000ULL /* 2^38 */

/* Not a requested access: the resolver sets it, for the typed profiles only,
 * when the listed BO carries DRM_G5P_BO_EXEC. Only TYPED_V3 reads it; every
 * other check masks on READ/WRITE. It lives in `access` rather than in a new
 * field so positional g5p_ru_range initialisers keep compiling.
 */
#define G5P_RU_ACCESS_EXEC 16U

struct g5p_ru_range {
	u64 va, size;
	u32 access;
};

/* [va, va + size) inside one listed range that grants every access bit. */
static inline bool g5p_ru_in(const struct g5p_ru_range *r, u32 n, u64 va,
			     u64 size, u32 access)
{
	if (!size || va > ~0ULL - size)
		return false;
	for (u32 i = 0; i < n; i++)
		if ((r[i].access & access) == access && va >= r[i].va &&
		    va - r[i].va <= r[i].size &&
		    size <= r[i].size - (va - r[i].va))
			return true;
	return false;
}

/* Everything TYPED_V1 and TYPED_V2 share: all fields except the pools. */
static inline int g5p_ru_pass_check_common(const struct g5p_ru_pass *p,
					   const struct g5p_ru_range *r, u32 n)
{
	u32 rw = G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE;

	if (p->vdm_size < 4 || p->vdm_size > G5P_RU_MAX_VDM ||
	    ((p->vdm_va | p->vdm_size) & 3) ||
	    !g5p_ru_in(r, n, p->vdm_va, p->vdm_size, G5P_RU_ACCESS_READ))
		return -EINVAL;
	/* Copied to TA W88/R0+470; meaning unresolved (r23), so bounded. */
	if ((p->vdm_control & 0xf) || p->vdm_control > G5P_RU_MAX_VDM)
		return -EINVAL;
	if ((p->scratch_va & 0xf) || p->scratch_size < 0x10 ||
	    p->scratch_size > G5P_RU_MAX_SCRATCH ||
	    !g5p_ru_in(r, n, p->scratch_va, p->scratch_size, rw))
		return -EINVAL;
	for (u32 i = 0; i < 6; i++)
		if ((p->scratch_ptr[i] & 3) ||
		    !g5p_ru_in(r, n, p->scratch_ptr[i], 4, rw))
			return -EINVAL;
	/* Three-word descriptors keep their fourth slot zero. */
	if (p->ta_pds[3] || p->eot[3] || p->eot_resume[3])
		return -EINVAL;
	return 0;
}

/* TYPED_V1 and LINEAR64_V1: the two allocator pools are page-aligned RW. */
static inline int g5p_ru_pass_check(const struct g5p_ru_pass *p,
				    const struct g5p_ru_range *r, u32 n)
{
	u32 rw = G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE;
	int ret = g5p_ru_pass_check_common(p, r, n);

	if (ret)
		return ret;
	if (((p->pool7_va | p->pool7_size | p->pool8_va | p->pool8_size) &
	     (G5P_GPU_PAGE - 1)) ||
	    !g5p_ru_in(r, n, p->pool7_va, p->pool7_size, rw) ||
	    !g5p_ru_in(r, n, p->pool8_va, p->pool8_size, rw))
		return -EINVAL;
	return 0;
}

#define G5P_RU_MAX_TABLE 65536U

/* TYPED_V2: the pools are the ISP scissor (8-byte entries) and depth-bias
 * (12-byte) tables, READ and word aligned; the V1 pool fields stay zero.
 * Flagged ZLS words are addresses into listed RW ranges; the rest is opaque.
 */
static inline int g5p_ru_pass2_check(const struct g5p_ru_pass *p,
				     const struct g5p_ru_pass2 *e,
				     const struct g5p_ru_range *r, u32 n)
{
	u32 rw = G5P_RU_ACCESS_READ | G5P_RU_ACCESS_WRITE;
	int ret;

	if (p->pool7_va || p->pool7_size || p->pool8_va || p->pool8_size)
		return -EINVAL;
	ret = g5p_ru_pass_check_common(p, r, n);
	if (ret)
		return ret;
	if (!e->scissor_count || e->scissor_count > G5P_RU_MAX_TABLE ||
	    (e->scissor_va & 3) ||
	    !g5p_ru_in(r, n, e->scissor_va, e->scissor_count * 8ULL,
		       G5P_RU_ACCESS_READ))
		return -EINVAL;
	if (!e->dbias_count || e->dbias_count > G5P_RU_MAX_TABLE ||
	    (e->dbias_va & 3) ||
	    !g5p_ru_in(r, n, e->dbias_va, e->dbias_count * 12ULL,
		       G5P_RU_ACCESS_READ))
		return -EINVAL;
	if ((e->pixel_width != 2 && e->pixel_width != 4 &&
	     e->pixel_width != 8) ||
	    (e->bgobj_vals & ~0x3ffU) || (e->zls_mask & ~0x3fU))
		return -EINVAL;
	for (u32 i = 0; i < 6; i++)
		if ((e->zls_mask & (1U << i)) &&
		    ((e->zls[i] >> 40) ||
		     !g5p_ru_in(r, n, e->zls[i] & ~0xfULL, 16, rw)))
			return -EINVAL;
	return 0;
}

/* TYPED_V3: the TA PDS program (W+0x88 -> R0+0x470) is an explicit reference
 * and the VDM stream length is a separate field that never reaches W.
 * Contract and evidence: docs/gpu/experiments/typed-v3-kernel-implementation.md.
 * Everything here is rejected, never masked. Structural rules need only the
 * resolved ranges; the two content rules take bytes the caller read through
 * the BOs' own pages (g5p_ru_pass3_content()). The rules are a subset of what
 * the userspace lane accepts (tools/mesa-a9 a9c_ctx_lint(), a9scene): nothing
 * here is stricter than a client that passes those lints.
 *   -EINVAL  stale vdm_control, malformed program/stream/descriptor fields,
 *            no terminator, or (V1 dialect) the V1 rules fail
 *   -ERANGE  outside [pds_base, pds_base + code_window) or past hw_va_limit
 *   -ENOENT  no listed range wholly covers the program, stream or task code
 *   -EACCES  missing READ, or the program/task code range's BO lacks EXEC
 *   -ENODATA the first 16 program bytes are all zero
 */
#define G5P_RU_PROGRAM_ALIGN 16U
#define G5P_RU_PROGRAM_MIN 16U

struct g5p_ru_v3_limits {
	u64 pds_base, code_window;
	u64 hw_va_limit; /* 0: no statement */
};

/* Where the content rules read: indexes into the ranges array. */
struct g5p_ru_v3_loc {
	u64 program_off; /* the exact W+0x88 value */
	u32 program_range, vdm_range;
	u32 pixel_width; /* the pass's output width, 2, 4 or 8, either dialect */
	bool dialect2; /* ext was non-zero: V2 3D words; else the V1 dialect */
};

/* TYPED_V3 carries the V2 ext struct. An all-zero ext selects the V1 dialect
 * (pools in base, no tables, V1 3D words): the pass tools/mesa-a9 records
 * today, plus the explicit program. Any non-zero ext field selects V2.
 */
static inline bool g5p_ru_pass2_zero(const struct g5p_ru_pass2 *e)
{
	u64 any = e->scissor_va | e->dbias_va | e->background_texture;

	for (u32 i = 0; i < ARRAY_SIZE(e->zls); i++)
		any |= e->zls[i];
	any |= e->scissor_count | e->dbias_count | e->bgobj_depth |
	       e->bgobj_vals | e->pixel_width | e->zls_mask;
	return !any;
}

/* First listed range that wholly contains [va, va + size). */
static inline bool g5p_ru_cover(const struct g5p_ru_range *r, u32 n, u64 va,
				u64 size, u32 *idx)
{
	if (!size || va > ~0ULL - size)
		return false;
	for (u32 i = 0; i < n; i++)
		if (va >= r[i].va && va - r[i].va <= r[i].size &&
		    size <= r[i].size - (va - r[i].va)) {
			*idx = i;
			return true;
		}
	return false;
}

/* [va, va + size) ends at or below the limit; zero limit makes no statement. */
static inline bool g5p_ru_below(u64 limit, u64 va, u64 size)
{
	return !limit || (size <= limit && va <= limit - size);
}

/* One PDS task reference of a store/resume descriptor: a 32-bit offset from
 * pds_base, 16-byte aligned like every captured value and W+0x88's mask,
 * non-zero, inside the code window and the hardware limit, and starting in a
 * listed READ range (`code`: whose BO also carries EXEC).
 */
static inline int g5p_ru_v3_ref(const struct g5p_ru_v3_limits *lim,
				const struct g5p_ru_range *r, u32 n, u32 off,
				bool code)
{
	u64 va;
	u32 i;

	if (!off || (off % G5P_RU_PROGRAM_ALIGN))
		return -EINVAL;
	if (lim->code_window < G5P_RU_PROGRAM_MIN ||
	    off > lim->code_window - G5P_RU_PROGRAM_MIN ||
	    lim->pds_base > ~0ULL - off)
		return -ERANGE;
	va = lim->pds_base + off;
	if (!g5p_ru_below(lim->hw_va_limit, va, G5P_RU_PROGRAM_MIN))
		return -ERANGE;
	if (!g5p_ru_cover(r, n, va, G5P_RU_PROGRAM_MIN, &i))
		return -ENOENT;
	if (!(r[i].access & G5P_RU_ACCESS_READ) ||
	    (code && !(r[i].access & G5P_RU_ACCESS_EXEC)))
		return -EACCES;
	return 0;
}

/* The TYPED_V3 pixel output width. The V1 dialect takes it from the V3 field
 * (0 = 2, the V1 dialect exactly), the V2 dialect from ext, where the V3 field
 * must stay zero. A width above 2 is several colour targets, and a partial
 * render must then resume with the pass's own EOT: a one-target resume EOT
 * would store only the first target. The EOT itself is not decoded.
 */
static inline int g5p_ru_pass3_width(const struct g5p_ru_pass *p,
				     const struct g5p_ru_pass2 *e,
				     const struct g5p_ru_pass3 *v,
				     bool dialect2, u32 *width)
{
	u32 w = v->pixel_width;

	if (v->pad || (w && w != 2 && w != 4 && w != 8) || (dialect2 && w))
		return -EINVAL;
	if (dialect2)
		w = e->pixel_width; /* 2, 4 or 8: g5p_ru_pass2_check() */
	else if (!w)
		w = 2;
	if (w > 2 && memcmp(p->eot, p->eot_resume, sizeof(p->eot)))
		return -EINVAL;
	*width = w;
	return 0;
}

/* The V3 rules, then the TYPED_V1 rules (all-zero ext) or the TYPED_V2 rules
 * (any ext field set; pools zero), then the output width. base.vdm_control is
 * checked first. On success loc names the ranges to read, the dialect and the
 * width.
 */
static inline int g5p_ru_pass3_check(const struct g5p_ru_pass *p,
				     const struct g5p_ru_pass2 *e,
				     const struct g5p_ru_pass3 *v,
				     const struct g5p_ru_v3_limits *lim,
				     const struct g5p_ru_range *r, u32 n,
				     struct g5p_ru_v3_loc *loc)
{
	u32 pi, vi;
	u64 off;
	int ret;

	if (p->vdm_control)
		return -EINVAL; /* a stream length here is the r36/r88 defect */
	if (v->ta_program_size < G5P_RU_PROGRAM_MIN ||
	    v->ta_program_size > DRM_G5P_RENDER_MAX_PROGRAM_BYTES ||
	    (v->ta_program_size % G5P_RU_PROGRAM_ALIGN) ||
	    (v->ta_program_va % G5P_RU_PROGRAM_ALIGN))
		return -EINVAL;
	if (p->vdm_size < 4 || p->vdm_size > G5P_RU_MAX_VDM ||
	    ((p->vdm_va | p->vdm_size) & 3) || v->vdm_end < 4 ||
	    v->vdm_end > p->vdm_size || (v->vdm_end & 3))
		return -EINVAL;
	if (v->ta_program_va < lim->pds_base)
		return -ERANGE;
	off = v->ta_program_va - lim->pds_base;
	if (lim->code_window < v->ta_program_size ||
	    off > lim->code_window - v->ta_program_size)
		return -ERANGE;
	if (!g5p_ru_below(lim->hw_va_limit, v->ta_program_va,
			  v->ta_program_size) ||
	    !g5p_ru_below(lim->hw_va_limit, p->vdm_va, p->vdm_size))
		return -ERANGE;
	/* Every listed range, not only the two above: on A10 a buffer moved past
	 * the limit stalls the TA without a fault, and all buffers moved together.
	 */
	for (u32 i = 0; i < n; i++)
		if (!g5p_ru_below(lim->hw_va_limit, r[i].va, r[i].size))
			return -ERANGE;
	if (!g5p_ru_cover(r, n, v->ta_program_va, v->ta_program_size, &pi))
		return -ENOENT;
	if (!(r[pi].access & G5P_RU_ACCESS_READ) ||
	    !(r[pi].access & G5P_RU_ACCESS_EXEC))
		return -EACCES;
	if (!g5p_ru_cover(r, n, p->vdm_va, p->vdm_size, &vi))
		return -ENOENT;
	if (!(r[vi].access & G5P_RU_ACCESS_READ))
		return -EACCES;
	/* Context store [0] / load [1]: code at word 2 (READ and EXEC), STREAM_OUT2
	 * at word 5 (READ). Resume tasks: word 2 (READ only: R0+0xf060/0xf070,
	 * named stream-out by the userspace lane and aux program by r92 2.4, so
	 * its role is unresolved and EXEC is not demanded). Words 0, 1, 3 and 4
	 * stay opaque. The spacing (store - program = 0x280, resume - store =
	 * 0x340) and a program that equals a task entry (r63 flew the store
	 * program; the role is not detectable structurally) are not enforced.
	 */
	for (u32 i = 0; i < 2; i++) {
		ret = g5p_ru_v3_ref(lim, r, n, p->context_pds[i][2], true);
		if (!ret)
			ret = g5p_ru_v3_ref(lim, r, n, p->context_pds[i][5],
					    false);
		if (!ret)
			ret = g5p_ru_v3_ref(lim, r, n, p->resume_pds[i][2],
					    false);
		if (ret)
			return ret;
	}
	loc->dialect2 = !g5p_ru_pass2_zero(e);
	ret = loc->dialect2 ? g5p_ru_pass2_check(p, e, r, n) :
			      g5p_ru_pass_check(p, r, n);
	if (!ret)
		ret = g5p_ru_pass3_width(p, e, v, loc->dialect2,
					 &loc->pixel_width);
	if (ret)
		return ret;
	loc->program_off = off;
	loc->program_range = pi;
	loc->vdm_range = vi;
	return 0;
}

/* The first 16 program bytes are not all zero (r88 published 0x370, which
 * pointed at zeros) and the last stream dword is the terminator. `head` is
 * G5P_RU_PROGRAM_MIN bytes, `tail` the 4 stream bytes at vdm_va + vdm_end - 4.
 */
static inline int g5p_ru_pass3_content(const u8 *head, const u8 *tail)
{
	u8 any = 0;
	u32 word = tail[0] | tail[1] << 8 | tail[2] << 16 | (u32)tail[3] << 24;

	for (u32 i = 0; i < G5P_RU_PROGRAM_MIN; i++)
		any |= head[i];
	if (!any)
		return -ENODATA;
	if (word != DRM_G5P_RENDER_STREAM_TERMINATE)
		return -EINVAL;
	return 0;
}

/* AGXRenderTargetMemoryG5 output sanity for userspace geometry: exact one
 * sample and the sizes the kernel will allocate. Opaque fields unchecked.
 */
static inline int g5p_ru_geometry_check(const struct g5p_ru_rt *rt, u32 width,
					u32 height,
					struct g5p_ru_rt_sizes *sizes)
{
	if (!width || !height || width > 16384 || height > 16384)
		return -EINVAL;
	if (rt->tiles > ((width + 31) / 32) * ((height + 31) / 32))
		return -EINVAL;
	return g5p_ru_rt_sizes(rt, sizes);
}

#endif
