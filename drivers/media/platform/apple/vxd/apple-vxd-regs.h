/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD (Imagination MSVDX, "PowerVR video decoder") register map
 *
 * Offsets recovered by mining the iOS 16.7.10 com.apple.driver.AppleD5500
 * kext (iPad6,11 kernelcache, unslid virtual addresses); every offset below
 * is reached through CPVDECCore::ReadReg/WriteReg in that kext. See
 * docs/periph/ds-video-decoder.md. Names follow the public MSVDX layout used
 * by Imagination-derived drivers; offset are facts, names are inferred.
 */
#ifndef __APPLE_VXD_REGS_H__
#define __APPLE_VXD_REGS_H__

#include <linux/bits.h>

/* Core identification (CPVDECCore::GetCoreRev [E]) */
#define VXD_REG_CORE_ID			0x0630
#define  VXD_CORE_ID_MAJOR_SH		16
#define  VXD_CORE_ID_ENT_PIPES		GENMASK(3, 0)
#define  VXD_CORE_ID_PIX_PIPES		GENMASK(7, 4)
#define VXD_REG_CORE_REV		0x0640

/*
 * Clock enables (CPVDECCore_Clocks.cpp SetClocks [E]).  Written with the
 * default value first, read back for a sanity check, then written with the
 * requested value.  A read-back mismatch means the core is not powered or
 * not clocked ("Core register write differs from read").
 */
#define VXD_REG_CR_PVDEC_MAN_CLK_ENABLE	0x0440
#define  VXD_CLK_ENABLE_DEFAULT		0x00000011
#define  VXD_CLK_ENABLE_ALL		0xffffffff
#define  VXD_CLK_ENABLE_FW		0x01010113

/*
 * MTX (META) block.  MTX_CTRL is read-modify-written with bits 0 and 16
 * taken from core flags; MTX_MAN_CLK_ENABLE is written with bit 28 set and
 * polled for equality during the boot sequence.
 */
#define VXD_REG_MTX_CTRL		0x1000
#define  VXD_MTX_CTRL_KEEP_MASK		0xfffefffe
#define VXD_REG_MTX_MAN_CLK_ENABLE	0x1008
/*
 * The same 0x1008 seen from the submit path: the kext writes 0xf00 there
 * before a decode submit that follows a GART change (+0x27a0 rule,
 * 0xfffffff00628af2c [E]).  In Imagination's BUS4 MMU layout (PVDEC,
 * img-vxe-vxd) 0x1000 + 8 is MMU_CONTROL1 and bits 11:8 are MMU_INVALDC,
 * one directory-cache invalidate per context [I].
 */
#define VXD_REG_MMU_CONTROL1		0x1008
#define  VXD_MMU_INVALDC_ALL		0x00000f00
#define  VXD_MTX_CLK_SEL		0x10000000
#define VXD_REG_MTX_MMU_0		0x1010
#define VXD_REG_MTX_MMU_DIR_ADR		0x1020
#define VXD_REG_MTX_MMU_MODE		0x1070
#define  VXD_MTX_MMU_MODE_40BIT		0x10
#define VXD_REG_MTX_MMU_CFG0		0x1080
#define  VXD_MTX_MMU_CFG0_40BIT		0x80	/* (cfg0 & 0xf0) == 0x80 */
#define VXD_REG_MTX_MMU_CFG1		0x1084
#define  VXD_MTX_MMU_CFG1_TILING_CTX	BIT(29)

/*
 * MTX boot block, programmed by LoadFirmwareAndStartCore [E].  BOOT_5 (7)
 * enables the MTX clocks and must be written before the rest of the block;
 * BOOT_1 is written twice (stage 0 then stage 1) to release the MTX from
 * reset; BOOT_FW_ADR holds the low 32 bits of the firmware buffer address.
 */
#define VXD_REG_MTX_BOOT_FW_ADR		0x6a00
#define VXD_REG_MTX_BOOT_1		0x6a04
#define  VXD_MTX_BOOT_1_STAGE0		0x22000000
#define  VXD_MTX_BOOT_1_STAGE1		0x22010000
#define VXD_REG_MTX_BOOT_2		0x6a08
#define  VXD_MTX_BOOT_2_VALUE		0x01000000
#define VXD_REG_MTX_BOOT_3		0x6a0c
#define VXD_REG_MTX_BOOT_4		0x6a14
#define  VXD_MTX_BOOT_4_VALUE		0x00000350
#define VXD_REG_MTX_BOOT_5		0x6a18
#define  VXD_MTX_BOOT_5_ENABLE		0x7

/* VEC block and entropy decoder pipes */
#define VXD_REG_VECT_CFG		0x0200
#define VXD_REG_VECT_CFG2		0x0208
#define  VXD_VECT_CFG2_MSGS_MINUS1	0x000000c7	/* core[0xb4] - 1 */
#define VXD_REG_ENTDEC_0_SIZE		0x0340
#define VXD_REG_ENTDEC_0_CFG		0x0344

/*
 * MTX -> host "message consumed" word: the firmware stores the word count
 * of the message it last took from TO_MTX here (fw 0x1000a90 [E]).
 */
#define VXD_REG_MTX_MSG_CONSUMED	0x0000

/* Host -> MTX doorbell, written 1 after every message push [E]. */
#define VXD_REG_MTX_DOORBELL		0x0088

/*
 * CMD block control.  The reset value 0xffff0000 is written before and
 * after firmware load (known state v1/v2 [E]).
 */
#define VXD_REG_CMD_CTRL		0x0400
#define  VXD_CMD_CTRL_RESET		0xffff0000

/*
 * Host interrupt: status / write-1-to-clear / enable.  The firmware raises
 * bit 20 by writing it to 0x410 when it posts a TO_HOST message with the
 * interrupt flag (fw 0x1000fe4 [E]); the kext ISR reads 0x410, clears with
 * 0x414 = 0x100000 before draining (0xfffffff00629a0ac [E]).  Known state
 * writes 0 to the enable and all-ones to the clear; boot enables bit 20.
 */
#define VXD_REG_IRQ_STATUS		0x0410
#define VXD_REG_IRQ_CLEAR		0x0414
#define VXD_REG_IRQ_ENABLE		0x0418
#define  VXD_IRQ_FW_MSG			BIT(20)

/* DMAC */
#define VXD_REG_MTX_MEM_CFG		0x0500
#define  VXD_MTX_MEM_BANK_COUNT		GENMASK(11, 8)
#define  VXD_MTX_MEM_BANK_SIZE		GENMASK(19, 16)
#define  VXD_MTX_MEM_LAST_BANK		GENMASK(27, 24)
#define VXD_REG_DMAC_STATUS		0x0504
#define  VXD_DMAC_STATUS_FLUSH		0x80000000

/*
 * Pipe select + per-pipe registers.  The block multiplexes several register
 * banks behind CMD_KICK: writing the pipe index (1-based) there selects
 * which pipe the 0x30xx/0x40xx banks talk to.
 */
#define VXD_REG_CMD_KICK		0x0460
#define VXD_REG_PIPE_SET_0		0x3020
#define VXD_REG_PIPE_SET_1		0x3030
#define VXD_REG_PIPE_SET_2		0x3038
#define VXD_REG_PIPE_4004		0x4004
#define VXD_REG_PIPE_CTRL		0x4020
#define VXD_REG_PIPE_CR0		0x4060
#define VXD_REG_PIPE_CR1		0x4068
#define VXD_REG_PIPE_CR2		0x4070
#define VXD_REG_PIPE_CR3		0x4078
#define VXD_REG_PIPE_STATUS		0x40c0
#define VXD_REG_PIPE_STATUS_2		0x40c4

/*
 * Host<->MTX message rings (comms), docs/periph/ds-vxd-decode-protocol.md
 * section 2.  The four indices are confirmed by the kext recovery path:
 *   0x201c TO_HOST_RD_INDEX (host)   0x2020 TO_HOST_WR_INDEX (fw)
 *   0x2028 TO_MTX_RD_INDEX  (fw)     0x2030 TO_MTX_WR_INDEX  (host)
 * FW_READY_MAGIC is polled (0 = the kext clears it, 0xa5a5a5a5 = firmware
 * alive).  After the magic the firmware has published each ring's geometry
 * as (base_bytes << 16) | capacity_words: word i of a ring is the register
 * at VXD_RING_WINDOW + base + 4 * i.  Rounds 1-3 called 0x2018/0x2024
 * "build halves".
 */
#define VXD_REG_COMMS_STATE		0x2000
#define VXD_REG_COMMS_2008		0x2008	/* cleared at ring setup [E] */
#define VXD_REG_FW_PC			0x200c	/* fw stores its PC; host clears */
#define VXD_REG_COMMS_MSG_COUNT		0x2010	/* fw->host messages posted */
#define VXD_REG_FW_READY_MAGIC		0x2014
#define  VXD_FW_READY_MAGIC		0xa5a5a5a5
#define VXD_REG_TO_HOST_RING		0x2018
#define VXD_REG_TO_HOST_RD_INDEX	0x201c
#define VXD_REG_TO_HOST_WR_INDEX	0x2020
#define VXD_REG_TO_MTX_RING		0x2024
#define VXD_REG_TO_MTX_RD_INDEX		0x2028
#define VXD_REG_COMMS_FLAGS		0x202c
#define VXD_REG_TO_MTX_WR_INDEX		0x2030

#define VXD_RING_WINDOW			0x2000
#define  VXD_RING_BASE			GENMASK(31, 16)
#define  VXD_RING_CAP			GENMASK(15, 0)

/*
 * Message header, both directions: (fence << 16) | (type << 8) | nwords,
 * nwords counting the header.  The firmware drops TO_MTX messages longer
 * than 0x14 words (fw 0x1000b80) and a zero-length header wedges its
 * reader (fw 0x1000a4c: it re-publishes the same read index forever).
 * The kext refuses TO_HOST messages of 0xf0 / 4 = 60 words or more.
 */
#define VXD_MSG_NWORDS			GENMASK(6, 0)
#define VXD_MSG_TYPE			GENMASK(15, 8)
#define VXD_MSG_FENCE			GENMASK(31, 16)
#define VXD_MSG_TO_MTX_MAX_WORDS	0x14
#define VXD_MSG_TO_HOST_MAX_WORDS	(0xf0 / 4 - 1)

/*
 * Host -> firmware types: the DEVA dispatcher (fw 0x1000ad4) handles
 * 0x80..0x8b.  0x83..0x85 jump straight back to the reader loop: they are
 * consumed and dropped, which makes them the benign ping.
 */
#define VXD_MSG_INIT			0x80
#define VXD_MSG_DECODE			0x81
#define VXD_MSG_NOP_FIRST		0x83
#define VXD_MSG_NOP_LAST		0x85
#define VXD_MSG_DEBLOCK			0x87
#define VXD_MSG_RENDER_TARGET		0x88

/* Firmware -> host types (ProcessFWMessages 0xfffffff006299bb4 [E]). */
#define VXD_MSG_COMPLETION		0xc0
#define VXD_MSG_CONTIGUITY		0xc6
#define VXD_MSG_PANIC			0xc7

/* 0xc0 word 2: fault class, "after core reset", REATTEMPT_SINGLEPIPE. */
#define VXD_COMPLETION_FAULT		GENMASK(28, 27)
#define VXD_COMPLETION_AFTER_RESET	BIT(28)
#define VXD_COMPLETION_REATTEMPT	BIT(31)

/* 0x80 init message (InitMessageBuffers 0xfffffff006298d40 [E]). */
#define VXD_INIT_MSG_WORDS		5
#define VXD_INIT_PAGES			GENMASK(15, 0)
#define VXD_INIT_SEQ			GENMASK(23, 16)
#define VXD_INIT_SENTINEL		0xffff0000

/*
 * Decode-family message w1 low-half flags (SubmitCommandBufferInternal
 * 0xfffffff00628a8d4 [E]): last packet of a submission | interrupt the
 * host on completion; more packets follow.
 */
#define VXD_MSG_FLAG_LAST		0x4004
#define VXD_MSG_FLAG_MORE		0x2000

/*
 * MSVDX command stream (docs/periph/ds-vxd-cmdstream.md section 2, the
 * emitters at 0xfffffff00629a34c..0xfffffff00629b15c [E]).  These words
 * live in the GART-mapped command buffer and are read by the core's
 * command processor, not by the DEVA firmware.
 */
#define VXD_CMD_REG_RUN			0x10000000	/* | count << 16 | reg */
#define  VXD_CMD_REG_RUN_ONE		0x00010000	/* count += 1 */
#define  VXD_CMD_REG_RUN_COUNT		GENMASK(27, 16)
#define VXD_CMD_REG1			0x000f0000	/* | (flags & 0xf0) << 16 | reg */
#define  VXD_CMD_REG1_FLAGS		GENMASK(7, 4)
#define VXD_CMD_REG_ADDR		GENMASK(15, 0)
#define VXD_CMD_SLICE_BLOCK		0x90000000	/* + 9 zero words */
#define  VXD_CMD_SLICE_BLOCK_WORDS	10
#define VXD_CMD_SLICE_DESC		0xb0000001	/* + 2 words */
#define VXD_CMD_LOAD			0xa0000000	/* | bytes, + device address */
#define  VXD_CMD_LOAD_MAX		GENMASK(27, 0)
#define VXD_CMD_TABLE_XFER		0xf0000000	/* | bytes, + address, + dest */
#define VXD_CMD_END_SLICE		0x60000000
/*
 * AVC additions (ds-vxd-cmdstream.md section 16.3-16.6; Imagination's
 * DXVA control-allocation names from psb_video's dxva_fw_ctrl.h):
 *   0x5 CMD_RENDEC_BLOCK  0x50000000 | flags | count << 16 | reg, values
 *   0xe CMD_DMA           0xe0000000 | type | bytes, device address
 *   0xb CMD_SR_SETUP flag 1 = ENABLE_RBDU_EXTRACTION (the core strips the
 *                         emulation-prevention bytes itself)
 */
#define VXD_CMD_BLOCK			0x50000000
#define  VXD_CMD_BLOCK_COUNT		GENMASK(26, 16)
#define  VXD_CMD_BLOCK_FLAG_REFS	0x08000000	/* on the 0x7100 block */
#define VXD_CMD_DMA			0xe0000000
#define  VXD_CMD_DMA_BYTES		GENMASK(15, 0)
#define  VXD_CMD_DMA_TYPE_VLC		0x00000000
#define VXD_CMD_CHUNK_ALIGN		64		/* zero pad between slices */

/*
 * Host-side packet list (CmdBuf::addCommand 0xfffffff0062982f8 [E]):
 * {magic0, payload bytes, magic1} + payload, back to back.  The submit
 * path walks it from its base, checks both magics (0xfffffff00628ac78) and
 * turns every packet into one mailbox message; the list itself never
 * reaches the device.
 */
#define VXD_PKT_MAGIC0			0x80000007
#define VXD_PKT_MAGIC1			0xabcdefab
#define VXD_PKT_HDR_BYTES		12
#define VXD_PKT_H264_PAYLOAD		0x78
/* Payload offsets (header-relative offset - 0xc). */
#define VXD_PKT_FLAGS			0x04	/* -> w1[15:0] */
#define VXD_PKT_W4			0x08	/* -> w4 */
#define VXD_PKT_CMD_BYTES		0x0c	/* -> w1[31:16] (>> 2, kind 4: >> 3) */
#define VXD_PKT_W2			0x10	/* -> w2 */
#define VXD_PKT_W5			0x24	/* -> w5 */
#define VXD_PKT_W6			0x28	/* -> w6, 7-word messages only */
#define VXD_PKT_KIND			0x4c
#define  VXD_PKT_KIND_PICTURE		1	/* -> 0x81 */
#define  VXD_PKT_KIND_DEBLOCK		3	/* -> 0x87 */
#define  VXD_PKT_KIND_RENDER_TARGET	4	/* -> 0x88 */
#define VXD_PKT_NOT_REF			0x50	/* AVC: host-side bookkeeping */
#define VXD_DECODE_MSG_WORDS		6

#define VXD_MMIO_SIZE			0x100000

#endif /* __APPLE_VXD_REGS_H__ */
