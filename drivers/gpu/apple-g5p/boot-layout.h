/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_G5P_BOOT_LAYOUT_H
#define APPLE_G5P_BOOT_LAYOUT_H

#include "pack.h"

#define G5P_GPU_PAGE 0x4000U
#define G5P_ROOT_SIZE 0x8000U
#define G5P_SHARED_SIZE 0x60000U
#define G5P_IMAGE_VA 0xe1c0000000ULL
#define G5P_SHARED_VA 0xe1c0200000ULL
#define G5P_ROOT_VA 0xe1c3200000ULL
#define G5P_PIO_VA 0xe1c4200000ULL
#define G5P_PIO_SIZE 0x10000U

/* c95da0 descriptors, sizes at 5940b90; c97b6c packs on 64-byte boundaries. */
struct g5p_boot_layout {
	u32 offset[0x90];
	u8 group[0x90];
	u32 used[2];
};

static inline void g5p_put32(u8 *p, u32 value)
{
	p[0] = value;
	p[1] = value >> 8;
	p[2] = value >> 16;
	p[3] = value >> 24;
}

static inline void g5p_put64(u8 *p, u64 value)
{
	g5p_put32(p, value);
	g5p_put32(p + 4, value >> 32);
}

static inline u32 g5p_meta_address(u64 va)
{
	/* ca0744: root range 8 bypasses META caches; other shared RAM is global. */
	u32 prefix = va >= G5P_ROOT_VA && va < G5P_ROOT_VA + 0x1000000 ?
		     0x40000000 : 0xc0000000;

	return ((u32)va + 0x40000000) | prefix;
}

static inline int g5p_plan_layout(struct g5p_boot_layout *p)
{
	static const struct {
		u8 cpu, gpu, group;
		u32 size;
	} objects[] = {
		{ 0x44, 0x50, 0, 0x1490 }, { 0x47, 0x51, 0, 0x990 },
		{ 0x48, 0x52, 0, 0x8e0 }, { 0x49, 0x53, 0, 0x8c0 },
		{ 0x4a, 0x54, 0, 0x938 }, { 0x4b, 0x55, 0, 0x28 },
		{ 0x4c, 0x56, 0, 8 }, { 0x4d, 0x57, 0, 0x78c },
		{ 0x4e, 0x58, 0, 0x84 },
		{ 0x5e, 0x5f, 1, 0x2058 }, { 0x70, 0x78, 1, 0x20 },
		{ 0x71, 0x79, 1, 0x80 }, { 0x72, 0x7a, 1, 0x20 },
		{ 0x73, 0x7b, 1, 0x20 }, { 0x74, 0x7c, 1, 0x2c00 },
		{ 0x75, 0x7d, 1, 0xb000 }, { 0x4f, 0x59, 1, 0x36000 },
		{ 0x76, 0x7e, 1, 0x2c00 }, { 0x77, 0x7f, 1, 0x2400 },
		{ 0x6c, 0x6e, 1, 0x20 }, { 0x6d, 0x6f, 1, 0x1400 },
		{ 0x60, 0x62, 1, 0x20 }, { 0x64, 0x66, 1, 0x20 },
		{ 0x68, 0x6a, 1, 0x20 }, { 0x61, 0x63, 1, 0x1400 },
		{ 0x65, 0x67, 1, 0x1400 }, { 0x69, 0x6b, 1, 0x1400 },
		/* Separate c95da0 allocations, placed after the recovered group. */
		{ 0x5c, 0x5d, 1, 8 }, { 0x85, 0x88, 1, 0x400 },
		{ 0x84, 0x87, 1, 0x400 }, { 0x86, 0x89, 1, 0x800 },
	};
	unsigned int i;

	memset(p, 0, sizeof(*p));
	for (i = 0; i < ARRAY_SIZE(objects); i++) {
		u32 group = objects[i].group;
		u32 off = (p->used[group] + 63) & ~63U;

		p->offset[objects[i].cpu] = off;
		p->offset[objects[i].gpu] = off;
		p->group[objects[i].cpu] = group;
		p->group[objects[i].gpu] = group;
		p->used[group] = off + objects[i].size;
	}
	return p->used[0] <= G5P_ROOT_SIZE && p->used[1] <= G5P_SHARED_SIZE ? 0 : -EINVAL;
}

static inline u64 g5p_object_va(const struct g5p_boot_layout *p, u32 slot)
{
	return (p->group[slot] ? G5P_SHARED_VA : G5P_ROOT_VA) + p->offset[slot];
}

static inline u8 *g5p_object(const struct g5p_boot_layout *p, u8 *root, u8 *shared, u32 slot)
{
	return (p->group[slot] ? shared : root) + p->offset[slot];
}

/* ca9b34 + cab44c + cac314: 40-bit PA, 16 KiB GPU pages. */
static inline u32 g5p_catalogue_entry(u64 pa)
{
	return (u32)(pa >> 8) | 1;
}

static inline u64 g5p_directory_entry(u64 pa)
{
	/* Geometry bytes 3c..3f are 08 05 05 15: size, entry/PA/VA shifts. */
	return pa | 3;
}

static inline u64 g5p_leaf_entry(u64 pa, u32 attributes)
{
	/* caa6e4 range attributes -> ca9a60: META protect, CC / SLC bypass. */
	return (pa & 0xfffffff000ULL) | (attributes & 0xc) |
	       ((u64)(attributes & 0x10) << 58) | 1;
}

static inline void g5p_make_boot_config(struct g5p_ldr_image *im,
					const struct g5p_boot_layout *p)
{
	static const u32 bases[] = { 0x10000000, 0x20000000, 0x40000000, 0x60001000 };
	static const u32 sizes[] = { 0x18000, 0x4000, 0x4000, 0x4000 };
	static const u32 offsets[] = { 0x4000, 0x1c000, 0, 0x20000 };
	u8 *out = im->data + 0x200;
	unsigned int seg, i;

	for (seg = 0; seg < 4; seg++) {
		u64 va = G5P_IMAGE_VA + offsets[seg];
		u32 reg = 0x04850000 + seg * 16;
		u32 values[] = { bases[seg] | 0xf02, sizes[seg] - 1,
				 (u32)va, (u32)(va >> 32) | 0x300000 };

		if (seg == 2) {
			g5p_put32(out, reg + 12);
			g5p_put32(out + 4, (u32)(va >> 32) | 0x100);
			out += 8;
			continue;
		}
		for (i = 0; i < 4; i++, out += 8) {
			g5p_put32(out, reg + i * 4);
			g5p_put32(out + 4, values[i]);
		}
	}
	for (i = 0; i < im->config_count; i++, out += 8) {
		g5p_put32(out, im->config[i][0]);
		g5p_put32(out + 4, im->config[i][1]);
	}
	g5p_put64(out, 0);
	g5p_put32(out + 8, g5p_meta_address(G5P_IMAGE_VA + 0x3ff8));
	g5p_put32(im->data + 0x3ff8, g5p_meta_address(g5p_object_va(p, 0x50)));
	g5p_put32(im->data + 0x3ffc, g5p_meta_address(g5p_object_va(p, 0x5f)));
}

/* c96be8/c9edcc/ca3ccc. Unknown platform policy fields remain zero. */
static inline void g5p_make_init(const struct g5p_boot_layout *p, u8 *root,
				 u8 *shared, const struct g5p_ldr_image *im,
				 u64 catalogue, u64 timestamp)
{
	static const u8 roots[] = {
		0x62, 0x63, 0x66, 0x67, 0x6a, 0x6b, 0x6e, 0x6f,
		0x78, 0x7c, 0x79, 0x7d, 0x7a, 0x7e, 0x7b, 0x7f,
	};
	static const u8 objects[] = { 0x51, 0x52, 0x53, 0x54, 0x55 };
	static const u8 tail[] = { 0x56, 0x58, 0x57, 0x88, 0x59 };
	static const u64 pio_pa[] = { 0x201e00000ULL, 0x20e100000ULL, 0x20e104000ULL,
				     0x201000000ULL };
	static const u32 pio_size[] = { 0x5000, 0x4000, 0x4000, 0x20000 };
	static const u32 pio_offset[] = { 0, 0x8000, 0xc000, 0 };
	u8 *init = g5p_object(p, root, shared, 0x44);
	u8 *meta = g5p_object(p, root, shared, 0x4d);
	u8 *ctrl = g5p_object(p, root, shared, 0x5e);
	unsigned int i;

	memset(root, 0, G5P_ROOT_SIZE);
	memset(shared, 0, G5P_SHARED_SIZE);
	for (i = 0; i < ARRAY_SIZE(roots); i++)
		g5p_put32(init + i * 4, g5p_meta_address(g5p_object_va(p, roots[i])));
	for (i = 0; i < ARRAY_SIZE(objects); i++)
		g5p_put32(init + 0x64 + i * 4, g5p_meta_address(g5p_object_va(p, objects[i])));
	for (i = 0; i < ARRAY_SIZE(tail); i++)
		g5p_put32(init + 0x44c + i * 4, g5p_meta_address(g5p_object_va(p, tail[i])));
	for (i = 0; i < 3; i++) {
		u8 *channel = g5p_object(p, root, shared, 0x47 + i);
		u32 ptr = g5p_meta_address(g5p_object_va(p, 0x51 + i));

		g5p_put32(channel + 4, ptr);
		g5p_put32(channel + 12, ptr + 8);
	}
	g5p_put32(g5p_object(p, root, shared, 0x48) + 0x778, 0xffffffff);
	g5p_put32(init + 0x480, (u32)(G5P_IMAGE_VA + 0x24000) + 0x40000000);
	g5p_put64(init + 0x484, G5P_IMAGE_VA + 0x24000);
	g5p_put32(init + 0x48c, 0x55dc);
	g5p_put64(meta, 0x600000000ULL);
	g5p_put64(meta + 8, 0xffc00000);
	g5p_put64(meta + 0x10, 0x300000000ULL);
	g5p_put64(meta + 0x18, 0x300000000ULL);
	for (i = 0; i < ARRAY_SIZE(pio_pa); i++) {
		u8 *entry = meta + 0x630 + i * 16;

		g5p_put64(entry, pio_pa[i]);
		g5p_put32(entry + 8, i == 3 ? 0 : g5p_meta_address(G5P_PIO_VA + pio_offset[i]));
		g5p_put32(entry + 12, pio_size[i]);
	}
	g5p_put32(meta + 0x6c0, 1);
	g5p_put32(meta + 0x6c8, 1);
	g5p_put64(meta + 0x6d8, 0x100000000ULL);
	g5p_put32(meta + 0x6e0, 1);
	g5p_put32(meta + 0x6e4, 1);
	/* readChipInfo version 15 -> tables 5942650/5942674 -> 1,5; N=6. */
	g5p_put32(meta + 0x704, 1);
	g5p_put32(meta + 0x708, 5);
	g5p_put32(meta + 0x70c, 6);
	g5p_put32(meta + 0x758, 0x14);
	g5p_put32(meta + 0x75c, 0x4000);
	g5p_put32(meta + 0x760, 0x1020);
	g5p_put32(meta + 0x764, im->t1_pc);
	g5p_put32(meta + 0x768, im->t1_sp);
	g5p_put64(meta + 0x774, G5P_IMAGE_VA);
	g5p_put64(meta + 0x77c, 0x300000);
	g5p_put64(meta + 0x784, timestamp);
	/* Fixed portion of c974f0; dynamic power-policy fields remain zero. */
	g5p_put64(g5p_object(p, root, shared, 0x4a) + 8, 0x400000000ULL);
	g5p_put64(g5p_object(p, root, shared, 0x4a) + 0x10, 0x3f800000);
	g5p_put64(g5p_object(p, root, shared, 0x4a) + 0x28, 0x100000001ULL);
	g5p_put32(g5p_object(p, root, shared, 0x4a) + 0x38, 100);
	g5p_put64(g5p_object(p, root, shared, 0x4a) + 0x8f8, timestamp);
	g5p_put32(ctrl + 0x24, 0x78);
	g5p_put32(ctrl + 0x30, 1);
	g5p_put32(ctrl + 0x1044, g5p_meta_address(g5p_object_va(p, 0x87)));
	g5p_put32(ctrl + 0x1048, g5p_meta_address(g5p_object_va(p, 0x89)));
	g5p_put32(ctrl + 0x104c, 1);
	g5p_put64(g5p_object(p, root, shared, 0x86), catalogue);
}

#endif
