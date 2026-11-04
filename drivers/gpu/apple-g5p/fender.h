/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef G5P_FENDER_H
#define G5P_FENDER_H

/* Every poll callback must enforce a deadline and retain its final raw word.
 * Apple 20H350: stop c9fa10, prepare c9fc10, boot c9f954;
 * Fender helpers ca4ccc (GPC state) and ca6544 (core-power request).
 * Hardware interpretation of the 0 -> 1 transition remains experimental.
 */
struct g5p_fender_io {
	void *ctx;
	int (*write)(void *ctx, u32 reg, u32 value);
	int (*poll)(void *ctx, u32 reg, u32 mask, u32 expected);
};

static inline int g5p_fender_idle(const struct g5p_fender_io *io)
{
	int ret;

	ret = io->write(io->ctx, 0x20000, 0);
	if (ret)
		return ret;
	ret = io->poll(io->ctx, 0x1000, 0x10, 0);
	if (ret)
		return ret;
	ret = io->write(io->ctx, 0x1000, 0x10);
	if (ret)
		return ret;
	return io->poll(io->ctx, 0x1000, 0x1f, 0);
}

static inline int g5p_fender_hold(const struct g5p_fender_io *io)
{
	int ret;

	ret = io->write(io->ctx, 0x20000, 0x20);
	if (ret)
		return ret;
	ret = io->poll(io->ctx, 0x20000, 0x20, 0x20);
	if (ret)
		return ret;
	return io->poll(io->ctx, 0x1000, 0x1f, 0);
}

/* q: the pstate the GPC is requested at (R1+0x1000 = 0x10 | q, poll for q); 1 as before,
 * or boot_gpu_perf_poweron's pin (boot.inc).
 */
static inline int
g5p_fender_start_observed_q(const struct g5p_fender_io *io,
			    void (*observe)(void *, const char *), u32 q)
{
	int ret;

	ret = io->write(io->ctx, 0x20000, 0);
	if (ret)
		return ret;
	if (observe)
		observe(io->ctx, "release-core-clear");
	ret = io->poll(io->ctx, 0x1000, 0x1f, 0);
	if (ret)
		return ret;
	if (observe)
		observe(io->ctx, "release-gpc0-verified");
	ret = io->write(io->ctx, 0x1000, 0x10 | q);
	if (ret)
		return ret;
	if (observe)
		observe(io->ctx, "release-gpc1-requested");
	ret = io->poll(io->ctx, 0x1000, 0x1f, q);
	if (!ret && observe)
		observe(io->ctx, "release-gpc1-active");
	return ret;
}

static inline int g5p_fender_start_observed(const struct g5p_fender_io *io,
					    void (*observe)(void *,
							    const char *))
{
	return g5p_fender_start_observed_q(io, observe, 1);
}

static inline int g5p_fender_start(const struct g5p_fender_io *io)
{
	return g5p_fender_start_observed(io, NULL);
}

#endif
