/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* R33 (ds-r33-b knob A): the kexec GFX cycle holds ps_gfx off for ~10 us, and
 * R0+0x418/0x41c (no reset) keeps the previous boot's value through it. At the
 * first Fender wake, before any GPU MMIO and before any GPU IRQ is requested,
 * hold the same PMGR gate (0x80340, gate124) off for boot_gfx_off_ms, then
 * restore target 0xf. The refusal rules are pmgr-pwrstate's own (min, reset,
 * auto). Every exit leaves the gate requested on.
 */
#define G5P_R33_GFX_PS 0x80340U
#define G5P_R33_GFX_OFF_MAX_MS 2000U
#define G5P_R33_GFX_POLL_US 10000U

static inline bool g5p_r33_gfx_on(u32 raw)
{
	return (raw & 0xf) == 0xf && ((raw >> 4) & 0xf) == 0xf;
}

/* Only a plain, fully-on, software-controlled gate may be cycled. */
static inline bool g5p_r33_gfx_cyclable(u32 raw)
{
	return g5p_r33_gfx_on(raw) && !((raw >> 16) & 0xf) && !(raw >> 31) &&
	       !((raw >> 28) & 1);
}

static int g5p_r33_gfx_off(struct device *dev, struct regmap *pmgr, u32 ms)
{
	u32 before, held, v = 0, after = 0;
	u64 t0;
	int ret, on;

	if (!ms)
		return 0;
	if (ms > G5P_R33_GFX_OFF_MAX_MS || !pmgr)
		return -EINVAL;
	ret = regmap_read(pmgr, G5P_R33_GFX_PS, &before);
	if (ret)
		return ret;
	if (!g5p_r33_gfx_cyclable(before)) {
		dev_info(
			dev,
			"POWER gfx-off refused ms=%u before=%08x; pmgr min/reset/auto or not on\n",
			ms, before);
		return -EBUSY;
	}
	ret = regmap_update_bits(pmgr, G5P_R33_GFX_PS, 0xf, 0);
	if (!ret)
		ret = regmap_read_poll_timeout(pmgr, G5P_R33_GFX_PS, v,
					       !((v >> 4) & 0xf), 10,
					       G5P_R33_GFX_POLL_US);
	t0 = ktime_get_ns();
	if (!ret)
		msleep(ms);
	held = (u32)((ktime_get_ns() - t0) / 1000);
	if (!ret)
		ret = regmap_read(pmgr, G5P_R33_GFX_PS, &v);
	/* Never leave the GPU domain off, whatever happened above. */
	on = regmap_update_bits(pmgr, G5P_R33_GFX_PS, 0xf, 0xf);
	if (!on)
		on = regmap_read_poll_timeout(pmgr, G5P_R33_GFX_PS, after,
					      g5p_r33_gfx_on(after), 10,
					      G5P_R33_GFX_POLL_US);
	dev_info(
		dev,
		"POWER gfx-off ms=%u held_us=%u before=%08x end=%08x after=%08x off_ret=%d "
		"on_ret=%d\n",
		ms, held, before, v, after, ret, on);
	if (!ret && (v >> 4) & 0xf)
		ret = -EIO;
	return ret ?: on;
}
