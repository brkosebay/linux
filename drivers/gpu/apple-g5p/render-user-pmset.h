/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Per-context PM records (perf60 stage 4, docs/gpu/multi-client-design.md risk R3).
 *
 * iOS acquires a ParamBuffer ID per prepare from a 127-entry pool (6cb9afc) and builds a PM
 * control/state/aux object and its block list per PM (6cb2378); one PM is never shared by two
 * work queues.  Linux built one device PM (control+S at shared+G5P_R22_PM, list and F in the
 * two render_meta pages, aux at G5P_RU_PRIVATE_VA) for every render context; with the GPU
 * never parked, one context's job then started from the PM state another context's job left.
 *
 * boot_pm_sets reserves N sets in the firmware GiB at boot, right after render_meta: set s is
 * three pages {control + S, list, F} at G5P_PMSET_VA + s * 3 pages, mapped like the shared
 * pages (they are firmware objects the firmware and the PM requesters address by meta VA).
 * A render context takes one set, its own pool pages and its own aux pages.  PM ID: 0 stays
 * the device PM's; set s may carry s + 1.
 *
 * Pure helpers, host-tested (scripts/gpu/perf60-host/test_pmset.c).
 */
#ifndef G5P_RENDER_USER_PMSET_H
#define G5P_RENDER_USER_PMSET_H

#define G5P_PMSET_MAX 8U
#define G5P_PMSET_PAGES 3U /* control + S, list, F */
#define G5P_PMSET_VA (G5P_SHARED_VA + G5P_SHARED_SIZE + 2 * G5P_GPU_PAGE)
#define G5P_PMSET_END \
	(G5P_PMSET_VA + G5P_PMSET_MAX * G5P_PMSET_PAGES * G5P_GPU_PAGE)

enum { G5P_PMSET_CONTROL, G5P_PMSET_LIST, G5P_PMSET_FREE };

static inline u64 g5p_pmset_va(u32 set, u32 part)
{
	return G5P_PMSET_VA +
	       ((u64)set * G5P_PMSET_PAGES + part) * G5P_GPU_PAGE;
}

static inline u32 g5p_pmset_id(u32 set)
{
	return set + 1;
}

/*
 * The PM's write-back target (release8 bench-gaps3, 2026-11-03).  After the GPU switches from
 * context A's jobs to context B's, the firmware's PM keeps writing A's free list F back, through
 * A's TA slot (its MMU context), on B's later jobs, until a job of a third context runs: two
 * faults, one on the 2nd and one on the 3rd job of B after A's last, both the first job after A
 * was unbound (page-invalid level 2 in A's MMU context, A's set F page; with A still bound the
 * same writes landed).  So a context's slots and set stay bound while it is the context of the
 * last job or the previous *distinct* one: g5p_pmset_track() at every job end, g5p_pmset_holds()
 * at close.  That bounds when the slots may go, not when the PM stops: two later faults (release8
 * quiet2, seq 858 and 893) wrote the free list of a context two switches back, after this rule had
 * released it.  So a released pair is parked rather than left empty (render_user_close_park,
 * drm-render-v3.inc), and a released set is reused last (g5p_pmset_pick()).
 */
static inline bool g5p_pmset_switched(const void *last, const void *job)
{
	return last != job;
}

static inline bool g5p_pmset_holds(const void *last, const void *prev,
				   const void *ctx)
{
	return ctx && (ctx == last || ctx == prev);
}

/* A free set, round-robin from `next` (a released set is reused last). -1: none free. */
static inline int g5p_pmset_pick(const bool *used, u32 n, u32 next)
{
	for (u32 i = 0; i < n; i++) {
		u32 s = (next + i) % n;

		if (!used[s])
			return s;
	}
	return -1;
}

#endif
