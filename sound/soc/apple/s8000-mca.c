// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple S8000/S8003 (A9) MCA, "mca-version 4", as an ASoC CPU DAI.
 *
 * Register layout and the programming sequence follow iOS 16.7.10's
 * AppleAE2MCA::_setIISConfig_v4 (see docs/audio/playback-path.md in
 * omarchy-ipad5). Also handles the two neighbours iOS drives from the MCA
 * driver: the per-MCA MCLK control word (exported as a clock, the codec's
 * MCLK) and the i2s-switch route (a syscon; route register values from DT).
 *
 * Data path: iOS feeds the MCA through SmartIO DMA (endpoint 0x28 for MCA0
 * TX, 0x29 for MCA0 RX). With "dmas"/"dma-names = tx[, rx]" the generic
 * dmaengine PCM runs on the apple-sio-dma driver; the SmartIO shim record
 * (ADT dma-channels words) is passed as the slave config's
 * peripheral_config. Capture exists only with an "rx" channel. Without DMA
 * (or with nodma=1), a timer-driven playback PCM keeps the stream running
 * and, if "apple,pio-fifo-offset" names a CPU writable FIFO port
 * (unverified, see test plan), feeds it from the MCA's TX-low-water
 * interrupt. Otherwise samples are discarded: clocks, codec and speakers
 * still run.
 *
 * TX and RX share the frame (clocks, slot width, run bit) and the IRQ: the
 * first prepared direction programs the whole block, the other one only
 * resets and programs its own half, and the frame stops with the last.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/soc/apple/sio.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define MCA_VERSION		0x00
#define MCA_FIFO_SIZE		0x04
#define  MCA_FIFO_SIZE_TX	GENMASK(9, 0)
#define  MCA_FIFO_SIZE_RX	GENMASK(25, 16)
#define MCA_SAMPLE		0x08
#define MCA_RESET		0x0c
#define  MCA_RESET_TX		BIT(0)
#define  MCA_RESET_RX		BIT(1)
#define MCA_FRAME		0x10
#define  MCA_FRAME_MASTER	BIT(0)
#define  MCA_FRAME_RUN		BIT(1)
#define  MCA_FRAME_FMT		BIT(3)	/* set unless format flag bit 1 */
#define  MCA_FRAME_DUTY		BIT(6)
#define  MCA_FRAME_MCLK_DIV	GENMASK(11, 8)
#define  MCA_FRAME_SLOTS	GENMASK(15, 12)
#define  MCA_FRAME_WIDTH	GENMASK(23, 20)
#define  MCA_FRAME_HALF		GENMASK(31, 24)
#define MCA_PAD			0x14
#define MCA_IRQ_STATUS		0x1c
#define MCA_IRQ_MASK		0x20
#define  MCA_IRQ_TX_UNDERFLOW	BIT(0)
#define  MCA_IRQ_TX_OVERFLOW	BIT(1)
#define  MCA_IRQ_TX_LOW_WATER	BIT(2)
#define  MCA_IRQ_RX_UNDERFLOW	BIT(3)
#define  MCA_IRQ_RX_OVERFLOW	BIT(4)
#define  MCA_IRQ_RX_HIGH_WATER	BIT(5)
#define  MCA_IRQ_FRAME_ERROR	BIT(6)
#define  MCA_IRQ_BCLK		BIT(7)
#define  MCA_IRQ_NO_BCLK	BIT(8)
#define  MCA_IRQ_FSYNC		BIT(9)
#define  MCA_IRQ_NO_FSYNC	BIT(10)
#define  MCA_IRQ_SHORT_FSYNC	BIT(11)
#define  MCA_IRQ_RATE		BIT(12)
#define  MCA_IRQ_ALL		GENMASK(12, 0)
#define  MCA_IRQ_ERRORS		(MCA_IRQ_TX_UNDERFLOW | MCA_IRQ_TX_OVERFLOW | \
				 MCA_IRQ_RX_UNDERFLOW | MCA_IRQ_RX_OVERFLOW | \
				 MCA_IRQ_FRAME_ERROR)
/*
 * iOS v4 unmasks only the four FIFO error causes (mask 0x1fe4); split per
 * direction so a direction's causes are only open while it is prepared.
 */
#define  MCA_IRQ_TX_CAUSES	(MCA_IRQ_TX_UNDERFLOW | MCA_IRQ_TX_OVERFLOW)
#define  MCA_IRQ_RX_CAUSES	(MCA_IRQ_RX_UNDERFLOW | MCA_IRQ_RX_OVERFLOW)
#define  MCA_IRQ_TX_ALL		GENMASK(2, 0)
#define  MCA_IRQ_RX_ALL		GENMASK(5, 3)
#define MCA_FLUSH		0x24
#define MCA_TX_CFG		0x60
#define MCA_TX_FIFO_CFG		0x64
#define MCA_TX_SLOTS		0x68
#define MCA_TX_SLOTS_HI		0x6c
#define MCA_RX_CFG		0x80
#define MCA_RX_FIFO_CFG		0x84
#define MCA_RX_SLOTS		0x88
#define MCA_RX_SLOTS_HI		0x8c
#define  MCA_XX_CFG_EN		BIT(0)

/* iOS _setIISConfig_v4 values for MCA0 (playback-path.md section 3) */
#define MCA_TX_CFG_I2S		(BIT(8) | 0x33 | 0x4)	/* enable: 0x137 */
#define MCA_RX_CFG_I2S		(BIT(8) | 0x33)		/* enable: 0x133 */
#define MCA_RX_FIFO_CFG_IOS	0x00030004	/* RX depth 4 (mcaKeepDMABusy) */

static bool nodma;
module_param(nodma, bool, 0444);
MODULE_PARM_DESC(nodma, "ignore dmas, use the timer PCM (clock/codec tests)");

/* bring-up overrides, read at every prepare / MCLK enable */
static uint mclk_ctl;
module_param(mclk_ctl, uint, 0644);
MODULE_PARM_DESC(mclk_ctl, "MCLK control word override (0 = from DT)");
static char *route = "";
module_param(route, charp, 0644);
MODULE_PARM_DESC(route, "i2s-switch route override: \"off=val,off=val\" (byte offsets)");

#define MCA_POLL_US		10
#define MCA_TIMEOUT_US		10000

/* i2s-switch v3 reset values (AppleAE2I2SSwitch3 +0x580) */
#define I2S_SW_NODE_DEFAULT	0x00181818
#define I2S_SW_DSPC_DEFAULT	0x18
#define I2S_SW_DSPC_OFFSET	0x38

#define MCA_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000)
#define MCA_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct s8000_mca {
	struct device *dev;
	void __iomem *base;
	void __iomem *mclk_reg;
	struct regmap *sw;
	u32 *sw_route;		/* offset, value pairs */
	unsigned int sw_route_len;

	struct clk_hw mclk_hw;
	u32 mclk_ctl;
	u32 mclk_rate;

	u32 fifo_tx;
	bool use_dma;
	bool capture;		/* SmartIO "rx" channel present */
	struct snd_soc_dai_driver dai_drv;
	struct snd_dmaengine_dai_dma_data dma_tx, dma_rx;
	/* SmartIO shim words from DT: xfer 16-bit, xfer 32-bit, timeout, fifo, trigger */
	u32 sio_dt[5];
	struct apple_sio_dma_config sio_cfg[2];	/* per SNDRV_PCM_STREAM_* */

	/* computed in hw_params, written in prepare */
	u32 sample, frame, tx_cfg, tx_fifo_cfg, tx_slots, rx_slots;

	/*
	 * BIT(SNDRV_PCM_STREAM_*): prepared (prepare .. hw_free) and running
	 * (trigger start .. stop). Under hw_lock with frame RUN and irq_mask.
	 */
	spinlock_t hw_lock;
	unsigned int prep_mask, run_mask;

	/* timer/PIO PCM */
	spinlock_t lock;
	struct snd_pcm_substream *substream;
	struct hrtimer timer;
	ktime_t period;
	snd_pcm_uframes_t pos;
	unsigned int pio_off;	/* 0 = no CPU FIFO port */
	unsigned int pio_burst;
	bool running;
	bool warned_null;

	int irq;
	unsigned long irq_count, err_count, spurious_count;
	u32 last_status;
	u32 irq_mask;		/* current MCA_IRQ_MASK value */
	u32 mclk_cur;		/* last MCLK word written (0 = off) */
	bool mclk_on;		/* MCLK clock enabled: word written */
	bool route_on;		/* i2s-switch route applied */
	/* last stream per direction: a start-up underflow vs recurring ones */
	struct {
		ktime_t start_time;
		s64 first_err_us;	/* first error IRQ after start, -1 = none */
		u32 first_err_status;
		u32 stop_status;	/* raw IRQ_STATUS at stop */
	} last[2];
	bool mclk_forced;	/* debugfs s8000-mca-mclk */
	struct mutex dbg_lock;	/* debugfs s8000-mca-mclk */
	u32 route_ovr[16];	/* parsed "route" parameter */
	unsigned int route_ovr_len;
};

static inline void mca_write(struct s8000_mca *mca, u32 off, u32 val)
{
	writel_relaxed(val, mca->base + off);
}

static inline u32 mca_read(struct s8000_mca *mca, u32 off)
{
	return readl_relaxed(mca->base + off);
}

/* --- MCLK control word as a clock ----------------------------------------- */

static struct s8000_mca *hw_to_mca(struct clk_hw *hw)
{
	return container_of(hw, struct s8000_mca, mclk_hw);
}

static int mca_mclk_prepare(struct clk_hw *hw)
{
	return pm_runtime_resume_and_get(hw_to_mca(hw)->dev);
}

static void mca_mclk_unprepare(struct clk_hw *hw)
{
	pm_runtime_put(hw_to_mca(hw)->dev);
}

static u32 mca_mclk_word(struct s8000_mca *mca)
{
	return READ_ONCE(mclk_ctl) ?: mca->mclk_ctl;
}

static int mca_mclk_enable(struct clk_hw *hw)
{
	struct s8000_mca *mca = hw_to_mca(hw);

	/* AppleAE2MCA _getMCLKClkCon: nclk/2 (0x18) | output enable (0x40) */
	mca->mclk_cur = mca_mclk_word(mca);
	writel(mca->mclk_cur, mca->mclk_reg);
	mca->mclk_on = true;
	return 0;
}

static void mca_mclk_disable(struct clk_hw *hw)
{
	struct s8000_mca *mca = hw_to_mca(hw);

	mca->mclk_on = false;
	if (!mca->mclk_forced) {
		writel(0, mca->mclk_reg);
		mca->mclk_cur = 0;
	}
}

static unsigned long mca_mclk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	return hw_to_mca(hw)->mclk_rate;
}

static const struct clk_ops mca_mclk_ops = {
	.prepare = mca_mclk_prepare,
	.unprepare = mca_mclk_unprepare,
	.enable = mca_mclk_enable,
	.disable = mca_mclk_disable,
	.recalc_rate = mca_mclk_recalc_rate,
};

/* --- i2s-switch ---------------------------------------------------------- */

/* parse the "route" parameter ("off=val,off=val") into route_ovr */
static void mca_parse_route(struct s8000_mca *mca)
{
	char buf[128], *p, *tok;
	unsigned int n = 0;
	u32 off, val;

	strscpy(buf, READ_ONCE(route) ?: "", sizeof(buf));
	p = buf;
	while ((tok = strsep(&p, ",")) && n + 1 < ARRAY_SIZE(mca->route_ovr)) {
		if (sscanf(tok, "%i=%i", &off, &val) != 2 || off >= 0x50 || (off & 3))
			continue;
		mca->route_ovr[n++] = off;
		mca->route_ovr[n++] = val;
	}
	mca->route_ovr_len = n;
}

static void mca_route(struct s8000_mca *mca, bool on)
{
	const u32 *r = mca->sw_route;
	unsigned int i, len = mca->sw_route_len;

	if (!mca->sw)
		return;

	if (on)
		mca_parse_route(mca);
	if (mca->route_ovr_len) {
		r = mca->route_ovr;
		len = mca->route_ovr_len;
	}

	for (i = 0; i + 1 < len; i += 2) {
		u32 off = r[i], val = r[i + 1];

		if (!on)
			val = off < I2S_SW_DSPC_OFFSET ? I2S_SW_NODE_DEFAULT :
			      off == I2S_SW_DSPC_OFFSET ? I2S_SW_DSPC_DEFAULT : 0;
		regmap_write(mca->sw, off, val);
	}
	mca->route_on = on;
}

/* --- DAI ------------------------------------------------------------------ */

static int mca_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* iOS runs MCA0 as the clock consumer of the CS42L81. */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

/* AppleAE2MCA::_getFIFODepth, TX direction */
static u32 mca_tx_depth(struct s8000_mca *mca, unsigned int rate,
			unsigned int channels)
{
	u32 per_ms = round_up(rate * channels / 2000, 4);
	u32 min = (channels * 2 + 3) & 0x3fc;
	u32 depth;

	if (per_ms < min)
		depth = min;
	else
		depth = min_t(u32, per_ms, mca->fifo_tx - 3);

	depth &= ~3;
	return max_t(u32, depth, 4);
}

static int mca_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	struct snd_dmaengine_dai_dma_data *dma =
		substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &mca->dma_tx :
								 &mca->dma_rx;
	unsigned int bits = params_width(params);
	unsigned int slot = params_physical_width(params);
	unsigned int channels = params_channels(params);
	unsigned int slots = 2, frame_bits = slot * slots, depth;
	u32 width_code;

	if (channels != 2)
		return -EINVAL;

	switch (slot) {
	case 16: width_code = 1; break;
	case 32: width_code = 4; break;
	default: return -EINVAL;
	}

	/*
	 * _setIISConfig_v4 for: consumer, MCLK out, I2S, 2 slots. The DAI is
	 * symmetric, so both directions compute the same values for both
	 * halves; prepare writes the half (or the block) it owns.
	 */
	mca->sample = (32 - bits) << 8 | (32 - bits);
	depth = mca_tx_depth(mca, params_rate(params), channels);
	mca->tx_fifo_cfg = depth | (depth - 1) << 16;
	mca->tx_slots = GENMASK(slots - 1, 0);
	mca->rx_slots = GENMASK(slots - 1, 0);
	/* bits 15:8 = data delay (1, I2S); 0x33 base; 0x4 for format 0 */
	mca->tx_cfg = MCA_TX_CFG_I2S;
	mca->frame = FIELD_PREP(MCA_FRAME_HALF, frame_bits / 2 - 1) |
		     FIELD_PREP(MCA_FRAME_WIDTH, width_code) |
		     FIELD_PREP(MCA_FRAME_SLOTS, slots - 1) |
		     FIELD_PREP(MCA_FRAME_MCLK_DIV, 0) |
		     MCA_FRAME_DUTY | MCA_FRAME_FMT;

	dma->addr_width = slot == 16 ? DMA_SLAVE_BUSWIDTH_2_BYTES :
				       DMA_SLAVE_BUSWIDTH_4_BYTES;
	/* informational: the SmartIO limit comes from mca_prepare_slave_config */
	dma->maxburst = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? depth :
			MCA_RX_FIFO_CFG_IOS & 0xffff;

	dev_dbg(mca->dev, "sample %#x frame %#x txcfg %#x txfifo %#x\n",
		mca->sample, mca->frame, mca->tx_cfg, mca->tx_fifo_cfg);
	return 0;
}

static u32 mca_dir_causes(struct s8000_mca *mca, bool tx)
{
	if (!tx)
		return MCA_IRQ_RX_CAUSES;
	return MCA_IRQ_TX_CAUSES | (mca->pio_off ? MCA_IRQ_TX_LOW_WATER : 0);
}

static void mca_write_tx(struct s8000_mca *mca)
{
	mca_write(mca, MCA_TX_FIFO_CFG, mca->tx_fifo_cfg);
	mca_write(mca, MCA_TX_SLOTS, mca->tx_slots);
	mca_write(mca, MCA_TX_SLOTS_HI, 0);
	mca_write(mca, MCA_TX_CFG, mca->tx_cfg & ~MCA_XX_CFG_EN);
}

static void mca_write_rx(struct s8000_mca *mca)
{
	mca_write(mca, MCA_RX_FIFO_CFG, MCA_RX_FIFO_CFG_IOS);
	mca_write(mca, MCA_RX_SLOTS, mca->rx_slots);
	mca_write(mca, MCA_RX_SLOTS_HI, 0);
	mca_write(mca, MCA_RX_CFG, MCA_RX_CFG_I2S & ~MCA_XX_CFG_EN);
}

/* the whole block, as _setIISConfig_v4: only while no direction is prepared */
static int mca_init_block(struct s8000_mca *mca)
{
	unsigned long flags;
	u32 val;
	int ret;

	spin_lock_irqsave(&mca->hw_lock, flags);
	mca->irq_mask = MCA_IRQ_ALL;
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	mca_write(mca, MCA_RESET, MCA_RESET_TX | MCA_RESET_RX);
	ret = readl_relaxed_poll_timeout(mca->base + MCA_RESET, val,
					 !(val & (MCA_RESET_TX | MCA_RESET_RX)),
					 MCA_POLL_US, MCA_TIMEOUT_US);
	if (ret) {
		dev_err(mca->dev, "reset timed out (%#x)\n", val);
		return ret;
	}

	mca_write(mca, MCA_FLUSH, 1);
	ret = readl_relaxed_poll_timeout(mca->base + MCA_FLUSH, val, !(val & 1),
					 MCA_POLL_US, MCA_TIMEOUT_US);
	if (ret) {
		dev_err(mca->dev, "flush timed out (%#x)\n", val);
		return ret;
	}

	mca_write(mca, MCA_IRQ_MASK, MCA_IRQ_ALL);
	mca_write(mca, MCA_IRQ_STATUS, MCA_IRQ_ALL);
	mca_write(mca, MCA_SAMPLE, mca->sample);
	mca_write(mca, MCA_TX_FIFO_CFG, mca->tx_fifo_cfg);
	mca_write(mca, MCA_RX_FIFO_CFG, MCA_RX_FIFO_CFG_IOS);
	mca_write(mca, MCA_PAD, 0);
	mca_write(mca, MCA_TX_SLOTS, mca->tx_slots);
	mca_write(mca, MCA_TX_SLOTS_HI, 0);
	mca_write(mca, MCA_RX_SLOTS, mca->rx_slots);
	mca_write(mca, MCA_RX_SLOTS_HI, 0);
	mca_write(mca, MCA_TX_CFG, mca->tx_cfg & ~MCA_XX_CFG_EN);
	mca_write(mca, MCA_RX_CFG, MCA_RX_CFG_I2S & ~MCA_XX_CFG_EN);
	return 0;
}

/* one direction while the other is prepared or running: leave the frame */
static int mca_init_dir(struct s8000_mca *mca, bool tx)
{
	u32 bit = tx ? MCA_RESET_TX : MCA_RESET_RX, val;
	int ret;

	mca_write(mca, MCA_RESET, bit);
	ret = readl_relaxed_poll_timeout(mca->base + MCA_RESET, val,
					 !(val & bit), MCA_POLL_US,
					 MCA_TIMEOUT_US);
	if (ret) {
		dev_err(mca->dev, "%s reset timed out (%#x)\n",
			tx ? "TX" : "RX", val);
		return ret;
	}

	mca_write(mca, MCA_SAMPLE, mca->sample);
	if (tx)
		mca_write_tx(mca);
	else
		mca_write_rx(mca);
	mca_write(mca, MCA_IRQ_STATUS, tx ? MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL);
	return 0;
}

static int mca_prepare(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	bool tx = dir == SNDRV_PCM_STREAM_PLAYBACK, first;
	unsigned long flags;
	int ret;

	/* ASoC serializes prepare/hw_free of the card's streams (pcm_mutex) */
	spin_lock_irqsave(&mca->hw_lock, flags);
	first = !(mca->prep_mask & ~BIT(dir));
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	ret = first ? mca_init_block(mca) : mca_init_dir(mca, tx);
	if (ret)
		return ret;

	spin_lock_irqsave(&mca->hw_lock, flags);
	mca->prep_mask |= BIT(dir);
	mca->irq_mask &= ~mca_dir_causes(mca, tx);
	mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
	if (first)
		mca_write(mca, MCA_FRAME, mca->frame);
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	if (first)
		mca_route(mca, true);
	return 0;
}

static int mca_hw_free(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	unsigned long flags;
	bool last;

	spin_lock_irqsave(&mca->hw_lock, flags);
	mca->prep_mask &= ~BIT(dir);
	last = !mca->prep_mask;
	if (last) {
		/* back to the idle state runtime resume sets up: everything masked */
		mca->irq_mask = MCA_IRQ_ALL;
		mca_write(mca, MCA_FRAME, 0);
	} else {
		mca->irq_mask |= dir == SNDRV_PCM_STREAM_PLAYBACK ?
				 MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL;
	}
	mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
	spin_unlock_irqrestore(&mca->hw_lock, flags);

	if (last)
		mca_route(mca, false);
	return 0;
}

static int mca_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);
	int dir = substream->stream;
	bool tx = dir == SNDRV_PCM_STREAM_PLAYBACK;
	u32 reset = tx ? MCA_RESET_TX : MCA_RESET_RX, val;
	unsigned long flags;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&mca->hw_lock, flags);
		mca->last[dir].start_time = ktime_get();
		mca->last[dir].first_err_us = -1;
		mca->last[dir].stop_status = 0;
		/* the other direction's pending causes are not ours to clear */
		mca_write(mca, MCA_IRQ_STATUS, !mca->run_mask ? MCA_IRQ_ALL :
			  tx ? MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL);
		if (tx && mca->pio_off) {
			mca->irq_mask &= ~MCA_IRQ_TX_LOW_WATER;
			mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
		}
		if (tx)
			mca_write(mca, MCA_TX_CFG, mca->tx_cfg | MCA_XX_CFG_EN);
		else
			mca_write(mca, MCA_RX_CFG, MCA_RX_CFG_I2S | MCA_XX_CFG_EN);
		mca->run_mask |= BIT(dir);
		mca_write(mca, MCA_FRAME, mca->frame | MCA_FRAME_RUN);
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&mca->hw_lock, flags);
		mca->last[dir].stop_status = mca_read(mca, MCA_IRQ_STATUS);
		if (tx) {
			/* an empty FIFO would keep TX low-water firing until prepare */
			mca->irq_mask |= MCA_IRQ_TX_LOW_WATER;
			mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
			mca_write(mca, MCA_TX_CFG, 0);
		} else {
			mca_write(mca, MCA_RX_CFG, 0);
		}
		mca->run_mask &= ~BIT(dir);
		if (!mca->run_mask)
			mca_write(mca, MCA_FRAME, mca->frame);
		/* TX stops with RESET = 1, RX with RESET = 2 (iOS) */
		mca_write(mca, MCA_RESET, reset);
		ret = readl_relaxed_poll_timeout_atomic(mca->base + MCA_RESET,
							val, !(val & reset), 1,
							1000);
		spin_unlock_irqrestore(&mca->hw_lock, flags);
		if (ret)
			dev_warn(mca->dev, "%s reset timed out\n",
				 tx ? "TX" : "RX");
		return 0;
	}

	return -EINVAL;
}

static int mca_dai_probe(struct snd_soc_dai *dai)
{
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &mca->dma_tx,
				  mca->capture ? &mca->dma_rx : NULL);
	return 0;
}

static const struct snd_soc_dai_ops mca_dai_ops = {
	.probe = mca_dai_probe,
	.set_fmt = mca_set_fmt,
	.hw_params = mca_hw_params,
	.prepare = mca_prepare,
	.hw_free = mca_hw_free,
	.trigger = mca_trigger,
};

/* template; probe drops .capture without a SmartIO "rx" channel */
static const struct snd_soc_dai_driver mca_dai = {
	.name = "s8000-mca",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MCA_RATES,
		.formats = MCA_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MCA_RATES,
		.formats = MCA_FORMATS,
	},
	.ops = &mca_dai_ops,
	/* one frame (clocks, slot width) for both directions */
	.symmetric_rate = 1,
	.symmetric_sample_bits = 1,
};

/* --- timer / PIO PCM (no SIO DMA) ----------------------------------------- */

static const struct snd_pcm_hardware mca_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = MCA_FORMATS,
	.rates = MCA_RATES,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 512 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 128,
};

/* Called with mca->lock held; returns true when a period boundary passed. */
static bool mca_pio_fill(struct s8000_mca *mca)
{
	struct snd_pcm_runtime *rt = mca->substream->runtime;
	unsigned int ch = rt->channels, n = mca->pio_burst / ch, i, c;
	snd_pcm_uframes_t old = mca->pos;

	for (i = 0; i < n; i++) {
		void *frame = rt->dma_area + frames_to_bytes(rt, mca->pos);

		for (c = 0; c < ch; c++) {
			u32 s;

			if (rt->sample_bits == 16)
				s = (u16)((s16 *)frame)[c];
			else
				s = ((u32 *)frame)[c];
			writel_relaxed(s, mca->base + mca->pio_off);
		}
		if (++mca->pos >= rt->buffer_size)
			mca->pos = 0;
	}

	return old / rt->period_size != mca->pos / rt->period_size;
}

static enum hrtimer_restart mca_timer_fn(struct hrtimer *t)
{
	struct s8000_mca *mca = container_of(t, struct s8000_mca, timer);
	struct snd_pcm_substream *ss;
	unsigned long flags;

	spin_lock_irqsave(&mca->lock, flags);
	ss = mca->running ? mca->substream : NULL;
	if (ss) {
		mca->pos += ss->runtime->period_size;
		if (mca->pos >= ss->runtime->buffer_size)
			mca->pos -= ss->runtime->buffer_size;
	}
	spin_unlock_irqrestore(&mca->lock, flags);

	if (!ss)
		return HRTIMER_NORESTART;

	snd_pcm_period_elapsed(ss);
	hrtimer_forward_now(t, mca->period);
	return HRTIMER_RESTART;
}

static int mca_pcm_open(struct snd_soc_component *component,
			struct snd_pcm_substream *substream)
{
	struct s8000_mca *mca = snd_soc_component_get_drvdata(component);

	snd_soc_set_runtime_hwparams(substream, &mca_pcm_hw);
	mca->substream = substream;

	if (!mca->pio_off && !mca->warned_null) {
		dev_warn(mca->dev,
			 "no SIO DMA and no PIO FIFO port: samples are discarded (clocks and codec still run)\n");
		mca->warned_null = true;
	}
	return 0;
}

static int mca_pcm_close(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream)
{
	struct s8000_mca *mca = snd_soc_component_get_drvdata(component);

	hrtimer_cancel(&mca->timer);
	mca->substream = NULL;
	return 0;
}

static int mca_pcm_prepare(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	struct s8000_mca *mca = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *rt = substream->runtime;

	mca->pos = 0;
	mca->period = ns_to_ktime(div_u64((u64)rt->period_size * NSEC_PER_SEC,
					  rt->rate));
	return 0;
}

static int mca_pcm_trigger(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream, int cmd)
{
	struct s8000_mca *mca = snd_soc_component_get_drvdata(component);
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&mca->lock, flags);
		mca->running = true;
		spin_unlock_irqrestore(&mca->lock, flags);
		if (!mca->pio_off)
			hrtimer_start(&mca->timer, mca->period,
				      HRTIMER_MODE_REL_SOFT);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&mca->lock, flags);
		mca->running = false;
		spin_unlock_irqrestore(&mca->lock, flags);
		hrtimer_try_to_cancel(&mca->timer);
		return 0;
	}

	return -EINVAL;
}

static snd_pcm_uframes_t mca_pcm_pointer(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream)
{
	struct s8000_mca *mca = snd_soc_component_get_drvdata(component);

	return READ_ONCE(mca->pos);
}

static int mca_pcm_new(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       0, 0);
	return 0;
}

static const struct snd_soc_component_driver mca_component_pio = {
	.name = "s8000-mca",
	.open = mca_pcm_open,
	.close = mca_pcm_close,
	.prepare = mca_pcm_prepare,
	.trigger = mca_pcm_trigger,
	.pointer = mca_pcm_pointer,
	.pcm_new = mca_pcm_new,
	.legacy_dai_naming = 1,
};

static const struct snd_soc_component_driver mca_component_dma = {
	.name = "s8000-mca",
	.legacy_dai_naming = 1,
};

/* SmartIO queues one command per period; it has 31 tags per endpoint. */
/*
 * SmartIO reports the position per completed period (residue granularity
 * "segment"). The generic dmaengine PCM would flag that as BATCH, but it
 * skips that logic when the driver supplies its own hardware limits, so
 * set it here: PipeWire/PulseAudio need it to keep a period of headroom.
 */
static const struct snd_pcm_hardware mca_dma_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
	.formats = MCA_FORMATS,
	.rates = MCA_RATES,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 512,
	.period_bytes_max = 32 * 1024,
	.periods_min = 2,
	.periods_max = 16,
};

static int mca_prepare_slave_config(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct dma_slave_config *sc)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct s8000_mca *mca = snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));
	struct apple_sio_dma_config *cfg = &mca->sio_cfg[substream->stream];
	unsigned int bytes = params_physical_width(params) / 8;
	unsigned int per_ms;
	int ret;

	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, sc);
	if (ret)
		return ret;

	/*
	 * AppleSmartIODMA shim record as AppleAE2MCA sets it up: transfer
	 * width from the dma-channels entry (16 or 32 bit; the RX entries
	 * have the same words), and the FIFO depth it requests for either
	 * direction: sample width * samples per ms, rounded to 4. The DMA
	 * driver applies SmartIO's validFIFODepth: TX at least trigger + 4,
	 * RX at most the trigger (192 B at 48 kHz, under 0x180).
	 */
	per_ms = round_up(params_rate(params) * params_channels(params) / 1000, 4);
	*cfg = (struct apple_sio_dma_config) {
		.xfer = bytes == 2 ? mca->sio_dt[0] : mca->sio_dt[1],
		.timeout = mca->sio_dt[2],
		.fifo = mca->sio_dt[3],
		.trigger = mca->sio_dt[4],
		.limit = bytes * per_ms,
	};
	sc->peripheral_config = cfg;
	sc->peripheral_size = sizeof(*cfg);
	return 0;
}

static const struct snd_dmaengine_pcm_config mca_dma_config = {
	.prepare_slave_config = mca_prepare_slave_config,
	.pcm_hardware = &mca_dma_hw,
	.prealloc_buffer_size = 128 * 1024,
};

/* --- IRQ ------------------------------------------------------------------ */

static irqreturn_t mca_irq(int irq, void *data)
{
	struct s8000_mca *mca = data;
	struct snd_pcm_substream *elapsed = NULL;
	u32 raw, status;
	int dir;

	spin_lock(&mca->hw_lock);
	raw = mca_read(mca, MCA_IRQ_STATUS);
	/*
	 * Only bits 12:0 are interrupt causes (iOS tests status & 0x1fff and
	 * logs "spurious intr" otherwise). Bit 31 also reads as set while a
	 * cause is pending (seen as 0x80000008), so it must not count as
	 * handled. Otherwise a cause that is masked only in irq_mask, not in
	 * the hardware mask, is "handled" forever. That IRQ storm is the
	 * likely cause of the round-4 hard lockup.
	 */
	status = raw & MCA_IRQ_ALL & ~mca->irq_mask;
	if (!status) {
		/* resync the hardware mask, so a stale open mask cannot storm */
		mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
		mca_write(mca, MCA_IRQ_STATUS, raw & MCA_IRQ_ALL);
		mca->spurious_count++;
		mca->last_status = raw;
		spin_unlock(&mca->hw_lock);
		return IRQ_NONE;
	}

	mca_write(mca, MCA_IRQ_STATUS, status);
	mca->irq_count++;
	mca->last_status = raw;

	if (status & MCA_IRQ_ERRORS) {
		/*
		 * Error bits re-latch every frame while the condition lasts
		 * (e.g. TX underflow without a data path): count once, mask
		 * them until the direction's next prepare instead of storming.
		 */
		mca->err_count++;
		for (dir = 0; dir < 2; dir++) {
			u32 mine = dir == SNDRV_PCM_STREAM_PLAYBACK ?
				   MCA_IRQ_TX_ALL : MCA_IRQ_RX_ALL;

			if (!(status & mine & MCA_IRQ_ERRORS) ||
			    mca->last[dir].first_err_us >= 0)
				continue;
			mca->last[dir].first_err_us =
				ktime_us_delta(ktime_get(),
					       mca->last[dir].start_time);
			mca->last[dir].first_err_status = raw;
		}
		mca->irq_mask |= status & MCA_IRQ_ERRORS;
		mca_write(mca, MCA_IRQ_MASK, mca->irq_mask);
		dev_dbg_ratelimited(mca->dev, "irq status %#x, masked\n", status);
	}
	spin_unlock(&mca->hw_lock);

	if ((status & MCA_IRQ_TX_LOW_WATER) && mca->pio_off) {
		spin_lock(&mca->lock);
		if (mca->running && mca->substream && mca_pio_fill(mca))
			elapsed = mca->substream;
		spin_unlock(&mca->lock);
	}

	/* outside mca->lock: period_elapsed takes the stream lock */
	if (elapsed)
		snd_pcm_period_elapsed(elapsed);

	return IRQ_HANDLED;
}

/* --- runtime PM ----------------------------------------------------------- */

/*
 * The IRQ is only enabled while the block is powered: the handler reads
 * MCA registers, and an MMIO access to a power-gated Apple block can hang
 * the CPU.
 */
static int mca_runtime_suspend(struct device *dev)
{
	struct s8000_mca *mca = dev_get_drvdata(dev);

	disable_irq(mca->irq);
	mca_write(mca, MCA_IRQ_MASK, MCA_IRQ_ALL);
	return 0;
}

static int mca_runtime_resume(struct device *dev)
{
	struct s8000_mca *mca = dev_get_drvdata(dev);

	/*
	 * The power domain was just switched on, so the MCA may be at its
	 * reset state, with causes like no-BCLK open in the hardware mask.
	 * Mask and clear everything before the IRQ is enabled. Prepare
	 * unmasks what a stream needs.
	 */
	mca->irq_mask = MCA_IRQ_ALL;
	mca_write(mca, MCA_IRQ_MASK, MCA_IRQ_ALL);
	mca_write(mca, MCA_IRQ_STATUS, MCA_IRQ_ALL);
	enable_irq(mca->irq);
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mca_pm_ops, mca_runtime_suspend,
				 mca_runtime_resume, NULL);

/* --- debugfs: register snapshot for the device test ----------------------- */

/*
 * No debugfs access changes the power state or touches a suspended block:
 * the MCA registers are read only while the device is already runtime
 * active (stream running, or power/control = on). The MCLK word and the
 * i2s-switch are read back only while the driver itself has them
 * programmed. Their power domain is not known.
 */
static int mca_regs_show(struct seq_file *s, void *unused)
{
	struct s8000_mca *mca = dev_get_drvdata(s->private);
	unsigned int off, val;
	int dir;

	seq_printf(s, "irqs %lu errors %lu spurious %lu last status %#x irq_mask %#x\n",
		   mca->irq_count, mca->err_count, mca->spurious_count,
		   mca->last_status, mca->irq_mask);
	seq_printf(s, "mclk word %#x (last written)%s%s, route %s\n",
		   mca->mclk_cur, mca->mclk_on ? ", clock on" : "",
		   mca->mclk_forced ? ", forced" : "",
		   mca->route_on ? "applied" : "off");
	seq_printf(s, "capture %s, prepared %#x running %#x (bit 0 playback, 1 capture)\n",
		   mca->capture ? "yes" : "no", mca->prep_mask, mca->run_mask);
	for (dir = 0; dir < 2; dir++) {
		const char *name = dir == SNDRV_PCM_STREAM_PLAYBACK ?
				   "playback" : "capture";

		if (mca->last[dir].first_err_us >= 0)
			seq_printf(s, "last %s stream: first error %lld us after start (status %#x), status at stop %#x\n",
				   name, mca->last[dir].first_err_us,
				   mca->last[dir].first_err_status,
				   mca->last[dir].stop_status);
		else
			seq_printf(s, "last %s stream: no error IRQ, status at stop %#x\n",
				   name, mca->last[dir].stop_status);
	}

	if (pm_runtime_get_if_active(mca->dev) <= 0) {
		seq_puts(s, "runtime-suspended: registers not read\n");
		return 0;
	}

	for (off = 0; off < 0x100; off += 4)
		seq_printf(s, "mca+%#05x = %#010x\n", off, mca_read(mca, off));
	if (mca->mclk_on || mca->mclk_forced)
		seq_printf(s, "mclk     = %#010x\n", readl(mca->mclk_reg));
	if (mca->sw && mca->route_on) {
		for (off = 0; off < 0x50; off += 4) {
			regmap_read(mca->sw, off, &val);
			seq_printf(s, "sw+%#04x  = %#010x\n", off, val);
		}
	}

	pm_runtime_put(mca->dev);
	return 0;
}

/*
 * debugfs s8000-mca-mclk: "1"/"0" or a raw word forces MCLK for pad tests.
 * Forcing holds a runtime PM reference, so the MCA domain is on (and its
 * IRQ masked by runtime resume) before the word is written.
 */
static ssize_t mca_mclk_write(struct file *f, const char __user *ubuf,
			      size_t len, loff_t *ppos)
{
	struct s8000_mca *mca = f->private_data;
	u32 val;
	int ret;

	ret = kstrtou32_from_user(ubuf, len, 0, &val);
	if (ret)
		return ret;

	guard(mutex)(&mca->dbg_lock);
	if (val && !mca->mclk_forced) {
		ret = pm_runtime_resume_and_get(mca->dev);
		if (ret)
			return ret;
		mca->mclk_forced = true;
	}
	if (val) {
		mca->mclk_cur = val == 1 ? mca_mclk_word(mca) : val;
		writel(mca->mclk_cur, mca->mclk_reg);
	} else if (mca->mclk_forced) {
		/* back to what the clock wants: its word, or off */
		mca->mclk_cur = mca->mclk_on ? mca_mclk_word(mca) : 0;
		writel(mca->mclk_cur, mca->mclk_reg);
		mca->mclk_forced = false;
		pm_runtime_put(mca->dev);
	}
	return len;
}

/* no MMIO: reports the last written word */
static ssize_t mca_mclk_read(struct file *f, char __user *ubuf, size_t len,
			     loff_t *ppos)
{
	struct s8000_mca *mca = f->private_data;
	char buf[48];
	int n;

	n = scnprintf(buf, sizeof(buf), "forced %d word %#x\n",
		      mca->mclk_forced, mca->mclk_cur);
	return simple_read_from_buffer(ubuf, len, ppos, buf, n);
}

static const struct file_operations mca_mclk_fops = {
	.open = simple_open,
	.read = mca_mclk_read,
	.write = mca_mclk_write,
	.llseek = default_llseek,
};

/* --- probe ---------------------------------------------------------------- */

static int s8000_mca_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct clk_init_data init = {};
	const char *clk_name = "mca-mclk";
	struct s8000_mca *mca;
	u32 ver, fifo;
	int ret, n;

	mca = devm_kzalloc(dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;
	mca->dev = dev;
	spin_lock_init(&mca->lock);
	spin_lock_init(&mca->hw_lock);
	ret = devm_mutex_init(dev, &mca->dbg_lock);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, mca);

	mca->base = devm_platform_ioremap_resource_byname(pdev, "mca");
	if (IS_ERR(mca->base))
		return PTR_ERR(mca->base);
	mca->mclk_reg = devm_platform_ioremap_resource_byname(pdev, "mclk");
	if (IS_ERR(mca->mclk_reg))
		return PTR_ERR(mca->mclk_reg);

	mca->sw = syscon_regmap_lookup_by_phandle_optional(np, "apple,i2s-switch");
	if (IS_ERR(mca->sw))
		return dev_err_probe(dev, PTR_ERR(mca->sw), "no i2s-switch\n");
	n = of_property_count_u32_elems(np, "apple,i2s-switch-route");
	if (mca->sw && n > 0) {
		mca->sw_route = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
		if (!mca->sw_route)
			return -ENOMEM;
		of_property_read_u32_array(np, "apple,i2s-switch-route",
					   mca->sw_route, n);
		mca->sw_route_len = n;
	}

	mca->mclk_ctl = 0x58;
	of_property_read_u32(np, "apple,mclk-control", &mca->mclk_ctl);
	mca->mclk_rate = 12000000;
	of_property_read_u32(np, "apple,mclk-frequency", &mca->mclk_rate);
	of_property_read_u32(np, "apple,pio-fifo-offset", &mca->pio_off);
	mca->pio_burst = 32;
	of_property_read_u32(np, "apple,pio-burst", &mca->pio_burst);
	if (mca->pio_off >= 0x4000 || (mca->pio_off & 3))
		return dev_err_probe(dev, -EINVAL, "bad apple,pio-fifo-offset\n");

	mca->irq_mask = MCA_IRQ_ALL;
	mca->last[SNDRV_PCM_STREAM_PLAYBACK].first_err_us = -1;
	mca->last[SNDRV_PCM_STREAM_CAPTURE].first_err_us = -1;
	mca->irq = platform_get_irq(pdev, 0);
	if (mca->irq < 0)
		return mca->irq;
	/* enabled by runtime resume, once the hardware mask is set */
	ret = devm_request_irq(dev, mca->irq, mca_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), mca);
	if (ret)
		return ret;

	hrtimer_setup(&mca->timer, mca_timer_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_SOFT);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power up failed\n");
	ver = mca_read(mca, MCA_VERSION);
	fifo = mca_read(mca, MCA_FIFO_SIZE);
	pm_runtime_put(dev);

	mca->fifo_tx = FIELD_GET(MCA_FIFO_SIZE_TX, fifo);
	mca->use_dma = of_property_present(np, "dmas") && !nodma;
	/* the generic PCM fails the card if a DAI stream has no channel */
	mca->capture = mca->use_dma &&
		       of_property_match_string(np, "dma-names", "rx") >= 0;
	dev_info(dev, "MCA version %u, FIFO tx %u rx %u, %s%s\n", ver,
		 mca->fifo_tx, (u32)FIELD_GET(MCA_FIFO_SIZE_RX, fifo),
		 mca->use_dma ? "SmartIO DMA" :
		 mca->pio_off ? "PIO (experimental)" : "timer PCM, no data path",
		 mca->capture ? ", capture" : "");
	if (ver != 4 || mca->fifo_tx != 0x40)
		dev_warn(dev, "expected version 4 / FIFO 0x40 (iOS)\n");
	if (mca->fifo_tx < 8)
		mca->fifo_tx = 0x40;

	/* MCLK control word -> codec MCLK */
	of_property_read_string(np, "clock-output-names", &clk_name);
	init.name = clk_name;
	init.ops = &mca_mclk_ops;
	mca->mclk_hw.init = &init;
	ret = devm_clk_hw_register(dev, &mca->mclk_hw);
	if (ret)
		return ret;
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &mca->mclk_hw);
	if (ret)
		return ret;

	/* MCA0 TX in Apple's ADT dma-channels: 0x28 {0x21|0x22, 0, 0x300, 0x180} */
	mca->sio_dt[0] = 0x21;
	mca->sio_dt[1] = 0x22;
	mca->sio_dt[2] = 0;
	mca->sio_dt[3] = 0x300;
	mca->sio_dt[4] = 0x180;
	of_property_read_u32_array(np, "apple,sio-dma-config", mca->sio_dt,
				   ARRAY_SIZE(mca->sio_dt));

	mca->dai_drv = mca_dai;
	if (!mca->capture)
		memset(&mca->dai_drv.capture, 0, sizeof(mca->dai_drv.capture));

	ret = devm_snd_soc_register_component(dev, mca->use_dma ?
					      &mca_component_dma :
					      &mca_component_pio,
					      &mca->dai_drv, 1);
	if (ret)
		return ret;

	if (mca->use_dma) {
		ret = devm_snd_dmaengine_pcm_register(dev, &mca_dma_config, 0);
		if (ret)
			return dev_err_probe(dev, ret, "dmaengine PCM failed\n");
	}

	debugfs_create_devm_seqfile(dev, "s8000-mca-regs", NULL, mca_regs_show);
	debugfs_create_file("s8000-mca-mclk", 0600, NULL, mca, &mca_mclk_fops);
	return 0;
}

static const struct of_device_id s8000_mca_of_match[] = {
	{ .compatible = "apple,s8000-mca" },
	{ }
};
MODULE_DEVICE_TABLE(of, s8000_mca_of_match);

static struct platform_driver s8000_mca_driver = {
	.driver = {
		.name = "s8000-mca",
		.of_match_table = s8000_mca_of_match,
		.pm = pm_ptr(&mca_pm_ops),
	},
	.probe = s8000_mca_probe,
};
module_platform_driver(s8000_mca_driver);

MODULE_DESCRIPTION("Apple S8000 (A9) MCA I2S driver");
MODULE_LICENSE("Dual MIT/GPL");
