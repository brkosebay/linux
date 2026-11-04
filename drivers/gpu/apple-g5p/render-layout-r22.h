/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_RENDER_LAYOUT_R22_H
#define G5P_RENDER_LAYOUT_R22_H

#include "render-layout.h"

#define G5P_R22_TA 0x54000U
#define G5P_R22_3D 0x58000U
#define G5P_R22_PM 0x5c000U
#define G5P_R22_W 0x400U
#define G5P_R22_TA_SKU 0xa00U
#define G5P_R22_3D_SKU 0xd00U
#define G5P_R22_TQ 0x1800U
#define G5P_R22_STAMP 0x1840U
#define G5P_R22_DEP 0x1900U
#define G5P_R22_BLOCKS 32U
#define G5P_R22_PHYS_PAGES (G5P_R22_BLOCKS * 8)
#define G5P_R22_AUX 0x101000000ULL
#define G5P_R22_RT 0x101020000ULL
#define G5P_R22_RT_AUX 0x101030000ULL

/* 6cb124c/13f4/14dc: PA PPNs, GPU F and META links are distinct.
 * Caller provides complete 16KiB control/list/F allocations. Check the whole
 * physical input before writing anything. No synthesized PM catalogue encoder.
 */

/* 6ca6d5c arithmetic oracle + 6cba480/6c7b64c. Native legacy mapping
 * dialect has bit36 clear: L=H, Laux=Haux. No PA substitution or float math.
 */

static const u32 g5p_r22_wait_words[] = {
	0x201,	    0x138204,  0x10,   0x8388204, 0,	  0x104000a, 0x1f,
	0x1f,	    0x8388204, 1,      0x104000a, 0x1f,	  0x1f,	     0xf48008,
	0x10000015, 4,	       0,      0x14,	  0x109f, 0x13000a,  0x4000,
	0x4000,	    0x138204,  0x4000, 0x8388204, 0,	  0x104000a, 8,
	8,	    0x8388204, 1,      0x104000a, 8,	  8,
};

/* Full normal timestamped recipes, independent stamp identities TA0/3D1.
 * k is dependency+4 (zero on the proven fresh initializer), not MMU tag.
 */

/* PM bank0 save writes runtime qwords before the a0/a8/b0 backing anchors. */
static inline bool g5p_r22_pm_saved(const u8 *state)
{
	for (u32 off = 8; off < 0xa0; off += 4)
		if (g5p_le32(state + off))
			return true;
	return false;
}

/* Never trust a firmware pointer until it equals our retained stamp word. */

#endif
