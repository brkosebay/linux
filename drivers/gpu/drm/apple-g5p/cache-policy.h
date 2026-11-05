/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* No caller may infer CPU caching from BO size or GPU use alone. */
#ifndef G5P_CACHE_POLICY_H
#define G5P_CACHE_POLICY_H
static inline int g5p_cache_class(int imported, int exported, int wc)
{
	return imported ? 2 : (!exported && wc ? 0 : 1);
}
static inline int g5p_cache_omit(int enabled, int imported, int exported, int wc)
{
	return enabled && g5p_cache_class(imported, exported, wc) == 0;
}
#endif
