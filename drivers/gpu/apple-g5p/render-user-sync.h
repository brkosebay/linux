/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * CPU cache maintenance by range (perf60-fillrate.md §11.3 cut 5, kernel half).  Host test:
 * scripts/gpu/perf60-host/test_sync_range.c.
 *
 * render_user_sync_range bit 0: BO_SYNC honours offset/size (already validated page aligned by
 * g5p_bo_range_valid) instead of syncing the whole BO.  Bit 1: a job's ACCESS_CACHE maintenance
 * (clean before, invalidate after retirement) covers the resource's BO range [offset, offset +
 * size), the bytes the job can reach through that resource, instead of the whole BO.  The window
 * is widened to whole GPU pages and clamped to the BO.  Off: the whole BO, as release8.
 * The window is walked over the sg_table's DMA segments in order (their lengths concatenate to
 * the BO's bytes); each segment gets the intersection, relative to the segment.
 */
#ifndef G5P_RENDER_USER_SYNC_H
#define G5P_RENDER_USER_SYNC_H

#define G5P_SYNC_ALIGN 0x4000ULL

/*
 * [lo, hi) = [off, off + size) widened to pages, clamped to the BO; false if empty or outside.
 */
static inline bool g5p_sync_window(u64 off, u64 size, u64 bo_size, u64 *lo,
				   u64 *hi)
{
	u64 end;

	if (!size || off >= bo_size)
		return false;
	end = size > bo_size - off ? bo_size : off + size;
	*lo = off & ~(G5P_SYNC_ALIGN - 1);
	*hi = (end + G5P_SYNC_ALIGN - 1) & ~(G5P_SYNC_ALIGN - 1);
	if (*hi > bo_size)
		*hi = bo_size;
	return *hi > *lo;
}

/* The part of segment [pos, pos + len) inside [lo, hi): *start relative to the segment. */
static inline bool g5p_sync_clip(u64 pos, u64 len, u64 lo, u64 hi, u64 *start,
				 u64 *n)
{
	u64 a = pos > lo ? pos : lo, b = pos + len < hi ? pos + len : hi;

	if (b <= a)
		return false;
	*start = a - pos;
	*n = b - a;
	return true;
}

#endif
