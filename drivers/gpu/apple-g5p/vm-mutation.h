/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_VM_MUTATION_H
#define G5P_VM_MUTATION_H
struct g5p_vm_mutation {
	bool generations;
	u32 generation[3];
};

/* Count actual changed descriptors, including clears, not dispatches. Repeated
 * parent writes of identical bytes do not advance a generation. Packed fields
 * follow iOS6cabf40. Publication is serialized by the existing command lock.
 */
static void g5p_vm_entry(struct g5p_vm_mutation *c, void *at, u64 value,
			 u32 level)
{
	u64 old = g5p_le32(at);

	if (level)
		old |= (u64)g5p_le32(at + 4) << 32;
	if (old == value)
		return;
	if (level)
		g5p_put64(at, value);
	else
		g5p_put32(at, value);
	c->generation[level]++;
}

static u32 g5p_vm_generation(struct g5p_vm_mutation *c)
{
	if (!c->generations)
		return 0;
	return ((c->generation[0] & 0x3ff) << 22) |
	       ((c->generation[1] & 0x3ff) << 12) | (c->generation[2] & 0xfff);
}
#endif
