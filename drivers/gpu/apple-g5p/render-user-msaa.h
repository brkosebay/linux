/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_USER_MSAA_H
#define G5P_RENDER_USER_MSAA_H

#include "bits-compat.h"
/* TYPED_V3 4x multisampling: the per-job words a pass with a 4-sample render
 * target needs on top of the V1/V2 3D words and the target's geometry
 * (g5p_ru_rt_derive_samples()). Pure byte builders over the kernel's own
 * work records: no allocation, MMIO, locking or firmware publication.
 *
 * Evidence (docs/gpu/experiments/msaa.md): the iOS gles3/gles3b captures
 * 20261101T175626Z, msaa4-resolve, msaa4-store, g3b-a2c and g3b-samplemask,
 * against g3-base, and the importer's 3D/TA fillers 6ca7ae0/6ca9710 executed
 * with the 4-sample descriptor (build/gpu/msaa/filler-msaa.c):
 *   record+0x4d8 0xf -> 0x7   D6a8 -> 3D W3c0[3:0] (ISP_CTL; the draws'
 *                             ISPCTL.upass drops 0xf -> 0x7 with it)
 *   record+0x640 0 -> 1       D6ad -> 3D W3c0 bit 17 and W86c (resolve only;
 *                             bit 17 is ISP_CTL.PROCESS_EMPTY_TILES, which
 *                             Linux's template sets for every pass since
 *                             0e05f993cf, so only W86c is new)
 *   RT Rb8 0 -> 2             D6e0 -> 3D Wd0 (AA mode, CR f30), and the
 *                             pixel-output shift 10 -> 11 in W3c0 [21],
 *                             W3c4, W3c8 and the six W108 words
 *   RT Rb0 / Rf8              3D W58 (CR fa8) / TA W40 (PPP_MULTISAMPLECTL)
 * The rest of the target's geometry reaches W through g5p_ru_rt_fill().
 */
#define G5P_RU_MSAA_RESOLVE 1U /* DRM_G5P_RENDER_MSAA_RESOLVE */

struct g5p_ru_msaa {
	u32 samples, flags; /* 0 = 1 */
	u64 positions; /* 0 = the target's */
};

/* samples 0 or 1 is the one-sample pass and must use a one-sample target;
 * 4 must use a four-sample target. Positions are one byte per sample, so a
 * word with bits above the sample count is malformed; zero takes the
 * target's (the standard pattern). Resolve exists only with four samples.
 * Nothing here is a hardware hazard: positions are a 4-bit grid and the
 * resolve bit changes what the PBE writes, not where.
 */
static inline int g5p_ru_msaa_check(const struct g5p_ru_msaa *m, u32 rt_samples)
{
	u32 s = m->samples ? m->samples : 1;

	if ((s != 1 && s != 4) || s != rt_samples ||
	    m->flags & ~G5P_RU_MSAA_RESOLVE || (m->flags && s == 1) ||
	    m->positions >> (s * 8))
		return -EINVAL;
	return 0;
}

/* After g5p_ru_fill_3d(), the dialect's width words and g5p_ru_rt_fill().
 * A one-sample pass is left as it is, unless it names its position.
 */
static inline void g5p_ru_fill_msaa(u8 *ta, u8 *td, const struct g5p_ru_msaa *m,
				    const struct g5p_ru_rt *rt)
{
	/* 6ca7ae0's output-width code table 5942678, by width - 2. */
	static const u8 width_code[7] = { 0, 0, 0x10, 0, 0, 0, 0x20 };
	u64 positions = m->positions ? m->positions : rt->packed;
	bool resolve = m->flags & G5P_RU_MSAA_RESOLVE;
	u32 width, count, mode, w3c0;

	if (m->samples != 4 && !m->positions)
		return;
	g5p_put64(ta + 0x40, positions);
	g5p_put64(td + 0x58, positions);
	if (m->samples != 4)
		return;
	/* The width the dialect wrote (2, 4 or 8); 6ca7ae0 shifts by 11 with an
	 * AA mode, so each width has half the output partitions.
	 */
	width = g5p_le32(td + 0x6b0);
	count = 0x4000 / (width << 11);
	mode = count > 1 ? 2 : count;
	w3c0 = g5p_le32(td + 0x3c0) & ~(0xfU | BIT(21));
	g5p_put32(td + 0x3c0,
		  w3c0 | 0x7 | (resolve ? BIT(17) : 0) | (mode - 1) << 21);
	g5p_put32(td + 0x3c4, mode * 0x204);
	g5p_put32(td + 0x3c8, count & 0x1f);
	for (u32 i = 0; i < 6; i++)
		g5p_put64(td + 0x108 + i * 8,
			  width_code[width - 2] | (count - 1) << 7 | 0x43);
	g5p_put64(td + 0xd0, rt->state);
	g5p_put32(td + 0x86c, resolve);
}

#endif
