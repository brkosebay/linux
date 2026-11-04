/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_COMPUTE_USER_H
#define G5P_COMPUTE_USER_H

/* Public geometry is fixed across VMs. The private ranges never alias a BO. */

static const u64 g5p_private_va[] = {
	0x300040000ULL, 0x300014000ULL, 0x300028000ULL, 0x300044000ULL, 0,
	0x100080000ULL, 0x100084000ULL, 0x1000a0000ULL, 0x1000a4000ULL,
};

static inline bool g5p_user_private_overlap(u64 va, u64 size)
{
	for (u32 i = 0; i < ARRAY_SIZE(g5p_private_va); i++)
		if (g5p_private_va[i] && va < g5p_private_va[i] + 0x4000 &&
		    g5p_private_va[i] < va + size)
			return true;
	return false;
}

#endif
