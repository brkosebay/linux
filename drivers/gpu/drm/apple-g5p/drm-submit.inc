/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Asynchronous privileged probes; payloads and firmware VM are kernel-owned. */
struct g5p_fence {
	struct dma_fence base;
	/* Owned by dma_fence; independent of queue/status locks. */
	spinlock_t lock;
	u32 state;
	int error;
};

static const char *g5p_fence_driver_name(struct dma_fence *f)
{
	return "apple_g5p";
}

static const char *g5p_fence_timeline_name(struct dma_fence *f)
{
	return "g5p-probe";
}

/* Default release uses dma_fence_free()/RCU; base is the first member. */
static const struct dma_fence_ops g5p_fence_ops = {
	.get_driver_name = g5p_fence_driver_name,
	.get_timeline_name = g5p_fence_timeline_name,
};

/* recovery108: the compute/probe submitters share the render path's admission window. */

/* Legacy JOB_COMPUTE remains unsupported: v1 lacks a bounded stream length. */
