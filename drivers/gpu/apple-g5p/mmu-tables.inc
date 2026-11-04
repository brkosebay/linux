/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Fixed, pre-release aliases only. No firmware patch or live table surgery. */
#define G5P_TABLE_ALIAS_VA (G5P_ROOT_VA + 0x100000ULL)
#define G5P_TABLE_COOKIE 0x3ffcU

static void g5p_table_clean_page(struct g5p_diag *g, struct g5p_dma_page *p)
{
	unsigned long start;

	if (!p->cpu)
		return;
	start = (unsigned long)p->cpu;
	/* Explicit architecture PoC operation, even if DT calls this coherent.
	 * dma_sync_single_for_device is not an API for coherent allocations.
	 */
	dcache_clean_poc(start, start + G5P_GPU_PAGE);
}

static void g5p_tables_clean(struct g5p_diag *g, const char *stage,
			     struct g5p_dma_page *user, u32 count)
{
	struct g5p_boot *b = g->boot;

	g5p_mmu_record(
		g,
		"MMU tables %s clean_poc=%u dma_coherent=%u dma_pgprot=%016llx; coherent DMA "
		"API, extra clean is experimental\n",
		stage, 1, dev_is_dma_coherent(g->dev),
		(u64)pgprot_val(dma_pgprot(g->dev, PAGE_KERNEL, 0)));
	for (u32 i = 0; i < count; i++)
		g5p_table_attrs(g, stage, &user[i]);
	g5p_table_attrs(g, stage, &b->catalogue);
	g5p_table_attrs(g, stage, &b->directory);
	for (u32 i = 0; i < b->leaves; i++)
		g5p_table_attrs(g, stage, &b->leaf[i]);
	;
	;
	for (u32 i = 0; i < count; i++)
		g5p_table_clean_page(g, &user[i]);
	for (u32 i = 0; i < b->leaves; i++)
		g5p_table_clean_page(g, &b->leaf[i]);
	g5p_table_clean_page(g, &b->sink_leaf);
	g5p_table_clean_page(g, &b->sink_directory);
	g5p_table_clean_page(g, &b->directory);
	g5p_table_clean_page(g, &b->catalogue);
	dsb(sy);
	dma_wmb();
	g5p_mmu_record(
		g,
		"MMU tables %s PoC complete user_pages=%u boot_leaves=%u; dsb sy before publication\n",
		stage, count, b->leaves);
}

/* Deliberately separate from g5p_meta_read(): the generic slave guard stays
 * latched after reset. This opt-in reader can address only the preinstalled
 * table aliases and one immutable root control word, never private META RAM.
 */
