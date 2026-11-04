/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * perf60 MMU invalidation arm (render_user_mmu_inval, drm-render-v3.inc), pure helpers.
 *
 * The only invalidation of a user MMU context besides DC3 is the firmware's per-kick tag compare
 * (10017ac4): diff = host_tag[S] ^ cached_tag[S]; bits 31:22 set -> e138 = (S << 3) | 4 (PC),
 * else bits 21:12 -> | 2 (PD), else bits 11:0 -> | 1 (PT); then cached = host.  One invalidate per
 * kick, the highest changed field (docs/gpu/firmware-re/README.md "generation producer").  A PC
 * field change therefore asks for the firmware's PC-level invalidate of that context, the level
 * Apple's own producer (6cabe70) and DC3 (1000218c) use as the strongest one.
 *
 * g5p_ru_mmu_pc_tag(): a host tag whose PC field differs from both slots' cached PC fields, so
 * the next kick of each slot takes the PC branch.  The PD and PT fields are the given tag's.
 * Never 0 (0 is the unbound tag) and never equal to either cached tag.  Host-tested
 * (scripts/gpu/perf60-host/test_mmu_inval.c).
 */
#ifndef G5P_RENDER_USER_MMU_H
#define G5P_RENDER_USER_MMU_H

#define G5P_RU_MMU_PC_MASK 0xffc00000U
#define G5P_RU_MMU_PC_ONE 0x00400000U

/* render_user_mmu_inval bits */
#define G5P_RU_MMU_EDIT \
	0x1U /* every VM edit publication (and spare adoption) asks for PC */
#define G5P_RU_MMU_SWITCH \
	0x2U /* a bound context's job after another context's job: PC retag */
#define G5P_RU_MMU_TOUCH \
	0x4U /* NC read of every table line before the tag store */
#define G5P_RU_MMU_EVERY \
	0x8U /* every job of a bound context: PC retag (superset of SWITCH) */

static inline u32 g5p_ru_mmu_pc_tag(u32 tag, u32 cached0, u32 cached1)
{
	u32 pc = (tag & G5P_RU_MMU_PC_MASK) + G5P_RU_MMU_PC_ONE;

	/* At most two values are excluded, so three candidates always contain a free one. */
	for (u32 i = 0; i < 3; i++, pc += G5P_RU_MMU_PC_ONE) {
		u32 t = (pc & G5P_RU_MMU_PC_MASK) | (tag & ~G5P_RU_MMU_PC_MASK);

		if (((t ^ cached0) & G5P_RU_MMU_PC_MASK) &&
		    ((t ^ cached1) & G5P_RU_MMU_PC_MASK) && t)
			return t;
	}
	return 0; /* unreachable: kept for the host test */
}

/*
 * Retag on a job: the published host tag, or the cached one if none, moved to a fresh PC field.
 */
static inline bool g5p_ru_mmu_retag_due(u32 mode, bool bound_before,
					bool switched)
{
	if (!bound_before)
		return false; /* this job's DC3 or adoption publishes its own tag */
	return (mode & G5P_RU_MMU_EVERY) ||
	       ((mode & G5P_RU_MMU_SWITCH) && switched);
}

#endif
