// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - command-stream encoders (R3b, split out for R3c)
 *
 * The H.264 command stream is an MSVDX register program
 * (docs/periph/ds-vxd-cmdstream.md): per slice one chunk of command words
 * in a GART-mapped buffer, padded to 64 bytes, plus one 0x78-byte packet
 * describing that chunk; per picture one more chunk of single-register
 * writes (the kind-4 / mailbox 0x88 packet).
 *
 * What the kext does, byte for byte [E]:
 *
 *   CH264Decoder::HOST_H264DecodeFrame, per slice (0xfffffff0062967cc):
 *     bytes = wptr - base - chunk_start            (payload +0x0c)
 *     zero-fill 64 - (wptr & 63) bytes, advance by that & 0x7c
 *     packet = addCommand(pktlist, 0x80000007, 0x78)   <- a second object
 *     payload +0x10 = chunk_start + cmd->[0x18]
 *   after the slice loop, 0x62b7ab0: one chunk of 0x000f0000 pairs, padded
 *     by 0x40 - (pairs & 7) * 8 bytes, payload +0x0c counting the pad
 *   CmdBuf::addCommand (0xfffffff0062982f8): {magic, size, 0xabcdefab} at
 *     the list's own offset, gate offset + size + 0xc < capacity
 *   SubmitCommandBufferInternal (0xfffffff00628a8d4): walk the packet list
 *     from its base, check both magics, one mailbox message per packet
 *
 * The packet list and the register program are therefore two buffers:
 * the walker starts at the list's base and would reject the first command
 * word (0x90000000) otherwise.  Only the register program is device
 * memory.
 *
 * Register runs (0xfffffff00629b068) are written through a scoped writer
 * that asserts exclusive use of the buffer: consecutive registers merge
 * into one 0x1 header (count += 1) only inside one group, so groups are
 * explicit here (apple_vxd_cmd_group_open/close) and no other command may
 * be emitted while one is open.
 *
 * Every emitter is bounds-checked; a failure sets a sticky error that the
 * caller checks once at the end of a chunk.  Nothing here touches MMIO,
 * the DMA API or a lock, so the file builds unchanged on the host.
 */
#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/string.h>

#include "apple-vxd-cmd.h"

void apple_vxd_cmdbuf_reset(struct apple_vxd_cmdbuf *cb)
{
	cb->used = 0;
	cb->chunk_start = 0;
	cb->group_open = false;
	cb->group_flags = 0;
	cb->run_hdr = SIZE_MAX;
	cb->run_next_reg = 0;
	cb->npairs = 0;
	cb->error = false;
	cb->pkt_used = 0;
	cb->npackets = 0;
}

static bool vxd_cmd_room(struct apple_vxd_cmdbuf *cb, size_t bytes)
{
	if (cb->error)
		return false;
	if (bytes > cb->size - cb->used) {
		cb->error = true;
		return false;
	}
	return true;
}

static void vxd_cmd_put(struct apple_vxd_cmdbuf *cb, u32 word)
{
	cb->vaddr[cb->used / 4] = word;
	cb->used += 4;
}

/* Every non-0x1 command: no open register group, room for @words. */
static bool vxd_cmd_begin(struct apple_vxd_cmdbuf *cb, size_t words)
{
	if (cb->group_open) {
		cb->error = true;
		return false;
	}
	return vxd_cmd_room(cb, words * 4);
}

void apple_vxd_cmd_word(struct apple_vxd_cmdbuf *cb, u32 word)
{
	if (vxd_cmd_begin(cb, 1))
		vxd_cmd_put(cb, word);
}

/* Open a 0x1 register group; @flags is OR-ed into every run header. */
void apple_vxd_cmd_group_open(struct apple_vxd_cmdbuf *cb, u32 flags)
{
	if (cb->group_open) {
		cb->error = true;
		return;
	}
	cb->group_open = true;
	cb->group_flags = flags;
	cb->run_hdr = SIZE_MAX;
}

/*
 * 0x1 form: 0x10010000 | flags | reg, value; a write to the register right
 * after the previous one extends that run instead (0xfffffff00629b068).
 */
void apple_vxd_cmd_reg(struct apple_vxd_cmdbuf *cb, u32 reg, u32 val)
{
	u32 *hdr;

	if (!cb->group_open) {
		cb->error = true;
		return;
	}

	hdr = cb->run_hdr != SIZE_MAX ? &cb->vaddr[cb->run_hdr / 4] : NULL;
	if (hdr && cb->run_next_reg == reg &&
	    FIELD_GET(VXD_CMD_REG_RUN_COUNT, *hdr) < FIELD_MAX(VXD_CMD_REG_RUN_COUNT)) {
		if (!vxd_cmd_room(cb, 4))
			return;
		*hdr += VXD_CMD_REG_RUN_ONE;
	} else {
		if (!vxd_cmd_room(cb, 8))
			return;
		cb->run_hdr = cb->used;
		vxd_cmd_put(cb, (reg & VXD_CMD_REG_ADDR) | cb->group_flags |
				VXD_CMD_REG_RUN | VXD_CMD_REG_RUN_ONE);
	}
	vxd_cmd_put(cb, val);
	cb->run_next_reg = reg + 4;
}

void apple_vxd_cmd_group_close(struct apple_vxd_cmdbuf *cb)
{
	if (!cb->group_open)
		cb->error = true;
	cb->group_open = false;
	cb->run_hdr = SIZE_MAX;
}

/* 0x000f0000 form: one register, (flags & 0xf0) << 16 (0xfffffff00629aa5c). */
void apple_vxd_cmd_reg1(struct apple_vxd_cmdbuf *cb, u32 reg, u32 val,
			u32 flags)
{
	if (!vxd_cmd_begin(cb, 2))
		return;
	vxd_cmd_put(cb, ((flags & VXD_CMD_REG1_FLAGS) << 16) |
			(reg & VXD_CMD_REG_ADDR) | VXD_CMD_REG1);
	vxd_cmd_put(cb, val);
	cb->npairs++;
}

/* 0x9: the 0x28-byte per-slice block, emitted all zero by Apple. */
void apple_vxd_cmd_slice_block(struct apple_vxd_cmdbuf *cb)
{
	unsigned int i;

	if (!vxd_cmd_begin(cb, VXD_CMD_SLICE_BLOCK_WORDS))
		return;
	vxd_cmd_put(cb, VXD_CMD_SLICE_BLOCK);
	for (i = 1; i < VXD_CMD_SLICE_BLOCK_WORDS; i++)
		vxd_cmd_put(cb, 0);
}

/*
 * 0x9 with its payload: the AVC path fills the ten-word CTRL_ALLOC_HEADER
 * (0xfffffff0062c4c98, ds-vxd-cmdstream.md section 16.4).
 */
void apple_vxd_cmd_ctrl_alloc(struct apple_vxd_cmdbuf *cb,
			      const u32 words[VXD_CMD_SLICE_BLOCK_WORDS - 1])
{
	unsigned int i;

	if (!vxd_cmd_begin(cb, VXD_CMD_SLICE_BLOCK_WORDS))
		return;
	vxd_cmd_put(cb, VXD_CMD_SLICE_BLOCK);
	for (i = 0; i < VXD_CMD_SLICE_BLOCK_WORDS - 1; i++)
		vxd_cmd_put(cb, words[i]);
}

/*
 * 0x5: one block of @n consecutive registers from @reg, never merged with
 * a neighbour (0xfffffff00629a568 / 0x629a6c0 / 0x629a81c: the header
 * count is patched when the block is closed, boundaries are kept).
 */
void apple_vxd_cmd_block(struct apple_vxd_cmdbuf *cb, u32 reg, u32 flags,
			 const u32 *vals, unsigned int n)
{
	unsigned int i;

	if (!n || n > FIELD_MAX(VXD_CMD_BLOCK_COUNT) ||
	    (flags & ~VXD_CMD_BLOCK_FLAG_REFS)) {
		cb->error = true;
		return;
	}
	if (!vxd_cmd_begin(cb, 1 + n))
		return;
	vxd_cmd_put(cb, VXD_CMD_BLOCK | flags |
			FIELD_PREP(VXD_CMD_BLOCK_COUNT, n) | (reg & VXD_CMD_REG_ADDR));
	for (i = 0; i < n; i++)
		vxd_cmd_put(cb, vals[i]);
}

/* 0xe: DMA @bytes from @va into the core, @type selects the target. */
void apple_vxd_cmd_dma(struct apple_vxd_cmdbuf *cb, u32 type, u32 bytes,
		       u32 va)
{
	if (bytes & ~VXD_CMD_DMA_BYTES) {
		cb->error = true;
		return;
	}
	if (!vxd_cmd_begin(cb, 2))
		return;
	vxd_cmd_put(cb, VXD_CMD_DMA | type | bytes);
	vxd_cmd_put(cb, va);
}

/* 0xb: {0xb0000001, header bits consumed, slice bytes} (0x62b5b00). */
void apple_vxd_cmd_slice_desc(struct apple_vxd_cmdbuf *cb, u32 w1, u32 w2)
{
	if (!vxd_cmd_begin(cb, 3))
		return;
	vxd_cmd_put(cb, VXD_CMD_SLICE_DESC);
	vxd_cmd_put(cb, w1);
	vxd_cmd_put(cb, w2);
}

/*
 * 0xa: load @bytes from device address @va (0xfffffff00629a444, asserts
 * bytes < 1 << 28).  This is how the slice bitstream reaches the core:
 * @va is the GART VA of the OUTPUT buffer plus the slice offset.
 */
void apple_vxd_cmd_load(struct apple_vxd_cmdbuf *cb, u32 va, u32 bytes)
{
	if (bytes & ~VXD_CMD_LOAD_MAX) {
		cb->error = true;
		return;
	}
	if (!vxd_cmd_begin(cb, 2))
		return;
	vxd_cmd_put(cb, VXD_CMD_LOAD | bytes);
	vxd_cmd_put(cb, va);
}

/* 0xf: install @bytes at @va into destination @dst (0x62b5fcc). */
void apple_vxd_cmd_table_xfer(struct apple_vxd_cmdbuf *cb, u32 bytes,
			      u32 va, u32 dst)
{
	if (bytes & ~VXD_CMD_LOAD_MAX) {
		cb->error = true;
		return;
	}
	if (!vxd_cmd_begin(cb, 3))
		return;
	vxd_cmd_put(cb, VXD_CMD_TABLE_XFER | bytes);
	vxd_cmd_put(cb, va);
	vxd_cmd_put(cb, dst);
}

void apple_vxd_cmd_end_slice(struct apple_vxd_cmdbuf *cb)
{
	apple_vxd_cmd_word(cb, VXD_CMD_END_SLICE);
}

static int vxd_cmd_pad(struct apple_vxd_cmdbuf *cb, size_t pad)
{
	if (!vxd_cmd_room(cb, pad))
		return -ENOSPC;
	memset((u8 *)cb->vaddr + cb->used, 0, pad);
	cb->used += pad;
	cb->chunk_start = cb->used;
	cb->npairs = 0;
	return 0;
}

/*
 * Close the current slice chunk: report its offset and its byte count
 * (payload +0x0c, before padding), then zero-fill 64 - (used & 63) bytes
 * as Apple does (a full 64 when already aligned).  The next chunk starts
 * 64-byte aligned.  Returns -ENOSPC on a sticky error.
 */
int apple_vxd_cmdbuf_chunk_end(struct apple_vxd_cmdbuf *cb, u32 *offset,
			       u32 *bytes)
{
	if (cb->group_open)
		cb->error = true;
	if (cb->error)
		return -ENOSPC;

	*offset = cb->chunk_start;
	*bytes = cb->used - cb->chunk_start;
	return vxd_cmd_pad(cb, VXD_CMD_CHUNK_ALIGN -
			   (cb->used & (VXD_CMD_CHUNK_ALIGN - 1)));
}

/*
 * Close an AVC slice chunk: the AVC emitter records the exact used bytes
 * and starts the next chunk right behind it, with no pad (0x62c4c98,
 * ds-vxd-cmdstream.md section 16.3).
 */
int apple_vxd_cmdbuf_chunk_end_exact(struct apple_vxd_cmdbuf *cb,
				     u32 *offset, u32 *bytes)
{
	if (cb->group_open)
		cb->error = true;
	if (cb->error)
		return -ENOSPC;

	*offset = cb->chunk_start;
	*bytes = cb->used - cb->chunk_start;
	cb->chunk_start = cb->used;
	cb->npairs = 0;
	return 0;
}

/*
 * Close a chunk made only of 0x000f0000 pairs (the per-picture kind-4
 * chunk, 0x62b7ab0): pad by 0x40 - (pairs & 7) * 8 bytes, a full 64 when
 * the pair count is a multiple of eight, and report the byte count
 * including that pad (the kext measures it after the pad; the message
 * sends it as a pair count, >> 3).
 */
int apple_vxd_cmdbuf_pair_chunk_end(struct apple_vxd_cmdbuf *cb, u32 *offset,
				    u32 *bytes)
{
	size_t pad;
	int ret;

	if (cb->group_open || cb->used - cb->chunk_start != cb->npairs * 8)
		cb->error = true;
	if (cb->error)
		return -ENOSPC;

	*offset = cb->chunk_start;
	pad = VXD_CMD_CHUNK_ALIGN - (cb->npairs & 7) * 8;
	ret = vxd_cmd_pad(cb, pad);
	if (!ret)
		*bytes = cb->used - *offset;
	return ret;
}

/*
 * CmdBuf::addCommand: append {magic, size, 0xabcdefab} and a zeroed
 * payload of @size bytes to the host packet list.  Same gate as the kext
 * (offset + size + 0xc < capacity).  Returns the payload or NULL.
 */
u32 *apple_vxd_cmdbuf_add_packet(struct apple_vxd_cmdbuf *cb, u32 magic,
				 u32 size)
{
	u32 *hdr;

	if (!IS_ALIGNED(size, 4) ||
	    (u64)cb->pkt_used + size + VXD_PKT_HDR_BYTES >= cb->pkt_size) {
		cb->error = true;
		return NULL;
	}

	hdr = (u32 *)(cb->pkt + cb->pkt_used);
	hdr[0] = magic;
	hdr[1] = size;
	hdr[2] = VXD_PKT_MAGIC1;
	memset(&hdr[3], 0, size);
	cb->pkt_used += VXD_PKT_HDR_BYTES + size;
	cb->npackets++;
	return &hdr[3];
}

/*
 * The submit path's container check (0xfffffff00628ac78): from the list
 * base, both magics on every packet, size + 0xc steps, landing exactly on
 * the end.  Returns the packet count or -EINVAL.
 */
int apple_vxd_cmdbuf_validate(const struct apple_vxd_cmdbuf *cb)
{
	size_t off = 0;
	int n = 0;

	while (off < cb->pkt_used) {
		const u32 *hdr = (const u32 *)(cb->pkt + off);

		if (cb->pkt_used - off < VXD_PKT_HDR_BYTES ||
		    hdr[0] != VXD_PKT_MAGIC0 || hdr[2] != VXD_PKT_MAGIC1 ||
		    hdr[1] > cb->pkt_used - off - VXD_PKT_HDR_BYTES)
			return -EINVAL;
		off += VXD_PKT_HDR_BYTES + hdr[1];
		n++;
	}
	return n == cb->npackets ? n : -EINVAL;
}
