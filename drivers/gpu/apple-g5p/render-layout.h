/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_LAYOUT_H
#define G5P_RENDER_LAYOUT_H

#include "boot-layout.h"

/* Offline construction primitives only. No TA/3D publication path exists.
 * 6caf9f8/6cb2378: caller must recover N/S1/S2/S3, not infer them from USC
 * count or target dimensions. Sizes are bounded to u32 allocation extents.
 */
struct g5p_pm_sizes {
	u32 control, page_lists, backing;
};

static inline int g5p_pm_sizes(u32 n, u32 blocks, u64 s1, u64 s2, u64 s3,
			       struct g5p_pm_sizes *out)
{
	u64 limit = 0xffffffffULL;
	u64 a, b;

	if (!n || !blocks || !s1 || !s2 || !s3)
		return -EINVAL;
	if (n > (limit - 0x80) / 0xc0 || blocks > (limit - 0x44) / 0x24 ||
	    s1 > limit / n || s2 > limit / n || s3 > limit)
		return -EOVERFLOW;
	a = s1 * n;
	b = s2 * n;
	if (a > limit - b || a + b > limit - s3)
		return -EOVERFLOW;
	out->control = 0x80 + n * 0xc0;
	out->page_lists = 0x44 + blocks * 0x24;
	out->backing = a + b + s3;
	return 0;
}

/* Only four recovered backing-pointer writes, not a complete PM record.
 * Caller owns a fully initialized0xc0 record and validates GPU mappings.
 */
static inline int g5p_pm_record_backing(u8 *record, u32 index, u32 n, u64 base,
					u64 s1, u64 s2, u64 s3, u32 meta)
{
	struct g5p_pm_sizes sizes;
	int ret;

	if (index >= n)
		return -EINVAL;
	ret = g5p_pm_sizes(n, 1, s1, s2, s3, &sizes);
	if (ret)
		return ret;
	if (base > ~0ULL - sizes.backing)
		return -EOVERFLOW;
	g5p_put64(record + 0xa8, base + index * s1);
	g5p_put64(record + 0xa0, base + n * s1 + index * s2);
	g5p_put64(record + 0xb0, base + n * s1 + n * s2);
	g5p_put32(record + 0xb8, meta);
	return 0;
}

#endif
