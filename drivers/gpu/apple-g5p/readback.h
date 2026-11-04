/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_READBACK_H
#define G5P_READBACK_H

#include "bits-compat.h"
#ifndef G5P_GPC_ON
#define G5P_GPC_ON(raw) (((raw) & 0x1f) == 1) /* core.inc overrides it */
#endif

enum g5p_readback_state {
	G5P_READBACK_WRAPPER_ONLY,
	G5P_READBACK_BASELINE,
	G5P_READBACK_RESET_HELD,
};

/* Returning wrapper registers in exp03; never a META slave-port register. */
static const u32 g5p_fender_regs[] = {
	0, 8, 0x10, 0x14, 0x1000, 0x1004, 0x2010, 0x2014, 0x20000,
};

/* Reset halves first: the only post-arm R0 readback needed by the gate. */
static const u32 g5p_rgx_regs[] = {
	0x100, 0x104, 0, 4, 8, 0xc, 0x18, 0x20, 0x80, 0x84, 0x88, 0x8c,
};

static inline u32 g5p_readback_rgx_count(u32 gpc, enum g5p_readback_state state)
{
	if (!G5P_GPC_ON(gpc))
		return 0;
	if (state == G5P_READBACK_RESET_HELD)
		return 2;
	return state == G5P_READBACK_BASELINE ? ARRAY_SIZE(g5p_rgx_regs) : 0;
}

static inline bool g5p_slave_access_allowed(bool reset_transition)
{
	return !reset_transition;
}

#endif
