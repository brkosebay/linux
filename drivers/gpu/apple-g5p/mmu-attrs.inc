/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#include <asm/pgtable.h>
#include <asm/sysreg.h>

/* Read the actual stable kernel mapping of a retained DMA allocation.
 * DMA memory may be a vmalloc PTE or a cached linear-map block.
 */
static u64 g5p_table_cpu_descriptor(unsigned long va)
{
	pgd_t *pgd = pgd_offset_k(va);
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;

	if (pgd_none(READ_ONCE(*pgd)) || pgd_bad(READ_ONCE(*pgd)))
		return 0;
	p4d = p4d_offset(pgd, va);
	if (p4d_none(READ_ONCE(*p4d)) || p4d_bad(READ_ONCE(*p4d)))
		return 0;
	pud = pud_offset(p4d, va);
	if (pud_leaf(READ_ONCE(*pud)))
		return pud_val(READ_ONCE(*pud));
	if (pud_none(READ_ONCE(*pud)) || pud_bad(READ_ONCE(*pud)))
		return 0;
	pmd = pmd_offset(pud, va);
	if (pmd_leaf(READ_ONCE(*pmd)))
		return pmd_val(READ_ONCE(*pmd));
	if (pmd_none(READ_ONCE(*pmd)) || pmd_bad(READ_ONCE(*pmd)))
		return 0;
	return pte_val(READ_ONCE(*pte_offset_kernel(pmd, va)));
}

static void g5p_table_attrs(struct g5p_diag *g, const char *stage,
			    struct g5p_dma_page *p)
{
	u64 desc, mair = read_sysreg(mair_el1);
	u32 index, attr;

	if (!p->cpu)
		return;
	desc = g5p_table_cpu_descriptor((unsigned long)p->cpu);
	index = (desc & PTE_ATTRINDX_MASK) >> 2;
	attr = (mair >> (index * 8)) & 0xff;
	g5p_mmu_record(
		g,
		"MMU allocation %s DMA=%pad coherent=%u cpu_desc=%016llx AttrIndx=%u "
		"MAIR=%016llx attr=%02x table_memtype=%s\n",
		stage, &p->dma, dev_is_dma_coherent(g->dev), desc, index, mair,
		attr,
		!desc	     ? "unknown" :
		attr == 0xff ? "Normal-WB" :
		attr == 0x44 ? "Normal-NC" :
			       "other");
}
