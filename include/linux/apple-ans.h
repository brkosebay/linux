/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_APPLE_ANS_H
#define _LINUX_APPLE_ANS_H

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

/* SoC-side view of the endpoint's port (none of it crosses the link) */
struct apple_ans_host_state {
	u32 link_up;
	u32 ltssm;	/* S8003 LTSSM state, 0x11 = L0 */
	u32 port_irq;	/* port interrupt status */
	u32 nvmmu_err;	/* NVMMU error status, 0 = none */
	u32 dart_err;	/* DART error, bit 31 = fault latched */
	u32 dart_addr;	/* DART last fault address */
};

#define APPLE_ANS_NVMMU_MAX_REQS	36
#define APPLE_ANS_NVMMU_MAX_PAGES	256
#define APPLE_ANS_NVMMU_PAGE_SIZE	4096

#if IS_REACHABLE(CONFIG_PCIE_APPLE_H9P)
/*
 * Map @npages 4K pages for FlatDMA slot @tag, or retire the slot when
 * @npages is 0. @to_memory: the device writes host memory (NVMe Read).
 */
int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
			const u64 *pages, unsigned int npages,
			bool to_memory, dma_addr_t *iova);
/* 1 if the endpoint's PCIe link is up, 0 if down, <0 on error */
int apple_ans_link_up(struct device *dev);
int apple_ans_host_state(struct device *dev, struct apple_ans_host_state *st);
void apple_ans_host_dump(struct device *dev, const char *when);
void apple_ans_dart_walk(struct device *dev, const char *what, u64 iova);
#else
static inline int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
				      const u64 *pages, unsigned int npages,
				      bool to_memory, dma_addr_t *iova)
{
	return -EOPNOTSUPP;
}

static inline int apple_ans_link_up(struct device *dev)
{
	return 1;
}

static inline int apple_ans_host_state(struct device *dev,
				       struct apple_ans_host_state *st)
{
	return -EOPNOTSUPP;
}

static inline void apple_ans_host_dump(struct device *dev, const char *when)
{
}

static inline void apple_ans_dart_walk(struct device *dev, const char *what,
				       u64 iova)
{
}
#endif

#endif /* _LINUX_APPLE_ANS_H */
