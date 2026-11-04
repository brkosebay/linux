/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * perf60 fault address (render_user_fault_read, drm-render-v3.inc): decode of a GPU MMU fault
 * status bank, R0+0xe150/e154 (bank 0) or R0+0xe158/e15c (bank 1), as high32:low32.
 *
 * The firmware's fault path (source 4 -> 1001a494) reads both banks with a 64-bit load, keeps
 * only a class in its companion record and posts event 4 without the status, so the address
 * exists only in these registers.  iOS reads them itself in its fault handler (kext 6ca5c14,
 * accelerator virtual+740) with exactly these fields (docs/gpu/firmware-re/README.md "Complete
 * e150 decode"): bit 0 valid, 2:1 type (0 page invalid, 2 read-only, 3 PM/META protected),
 * 3 read (0 write), 5:4 level (3 = catalogue), 11:6 requester (iOS table 6ff5ed8), 19:12
 * sideband, 27:20 MMU context (the slot), VA = (status >> 24) & 0xfffffffff0 (16-byte precision).
 * Pure, host-tested (scripts/gpu/perf60-host/test_fault_decode.c).
 */
#ifndef G5P_RENDER_USER_FAULTREG_H
#define G5P_RENDER_USER_FAULTREG_H

#include "bits-compat.h"
struct g5p_ru_mmu_fault {
	bool valid, read;
	u8 type, level, requester, sideband, context;
	u64 va;
};

static inline struct g5p_ru_mmu_fault g5p_ru_mmu_fault_decode(u32 lo, u32 hi)
{
	u64 s = (u64)hi << 32 | lo;
	struct g5p_ru_mmu_fault f = {
		.valid = s & 1,
		.type = (s >> 1) & 3,
		.read = (s >> 3) & 1,
		.level = (s >> 4) & 3,
		.requester = (s >> 6) & 0x3f,
		.sideband = (s >> 12) & 0xff,
		.context = (s >> 20) & 0xff,
		.va = (s >> 24) & 0xfffffffff0ULL,
	};

	return f;
}

static const char *const g5p_ru_mmu_fault_types[4] = {
	"page-invalid",
	"type1",
	"read-only",
	"pm-meta-protected",
};

/* Requester names, iOS kext table 6ff5ed8 (16-byte {name, class} entries; read from the j71t
 * 16.7.10 kernelcache, 2026-11-03).  56 entries; a larger index prints its number only.
 */
static const char *const g5p_ru_mmu_requesters[] = {
	"META",
	"TLA",
	"HOST",
	"VDMM",
	"CDM",
	"IPP",
	"PM",
	"TILING",
	"MCU_TEXAS0",
	"PDS_TEXAS0",
	"PBE0_TEXAS0",
	"PBE1_TEXAS0",
	"VDMS_BLACKPEARL0",
	"IPF_BLACKPEARL0",
	"ISP_BLACKPEARL0",
	"TPF_BLACKPEARL0",
	"USCS_BLACKPEARL0",
	"PPP_BLACKPEARL0",
	"VCE_BLACKPEARL0",
	"FBCDC_BLACKPEARL0",
	"MCU_TEXAS1",
	"PDS_TEXAS1",
	"PBE0_TEXAS1",
	"PBE1_TEXAS1",
	"MCU_TEXAS2",
	"PDS_TEXAS2",
	"PBE0_TEXAS2",
	"PBE1_TEXAS2",
	"VDMS_BLACKPEARL1",
	"IPF_BLACKPEARL1",
	"ISP_BLACKPEARL1",
	"TPF_BLACKPEARL1",
	"USCS_BLACKPEARL1",
	"PPP_BLACKPEARL1",
	"VCE_BLACKPEARL1",
	"FBCDC_BLACKPEARL1",
	"MCU_TEXAS3",
	"PDS_TEXAS3",
	"PBE0_TEXAS3",
	"PBE1_TEXAS3",
	"MCU_TEXAS4",
	"PDS_TEXAS4",
	"PBE0_TEXAS4",
	"PBE1_TEXAS4",
	"VDMS_BLACKPEARL2",
	"IPF_BLACKPEARL2",
	"ISP_BLACKPEARL2",
	"TPF_BLACKPEARL2",
	"USCS_BLACKPEARL2",
	"PPP_BLACKPEARL2",
	"VCE_BLACKPEARL2",
	"FBCDC_BLACKPEARL2",
	"MCU_TEXAS5",
	"PDS_TEXAS5",
	"PBE0_TEXAS5",
	"PBE1_TEXAS5",
};

static inline const char *g5p_ru_mmu_requester(u32 requester)
{
	if (requester >= ARRAY_SIZE(g5p_ru_mmu_requesters))
		return "?";
	return g5p_ru_mmu_requesters[requester];
}

/* The range test of the VM edit log: does [va, va + size) hold the fault address? */
static inline bool g5p_ru_mmu_fault_in(u64 fault_va, u64 va, u64 size)
{
	return fault_va >= va && fault_va - va < size;
}

#endif
