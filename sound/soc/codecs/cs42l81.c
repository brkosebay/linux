// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Cirrus Logic CS42L81 (Apple 338S-series) codec: speaker and headphone
 * playback, built-in DMIC capture, headphone jack.
 *
 * The CS42L81 is an Apple-specific Cirrus codec with integrated speaker
 * drivers, used on the A9 iPad 5 (J71t). There is no public datasheet; the
 * register sequences below were reconstructed from iOS 16.7.10's
 * AppleCS42L81Audio driver (see docs/audio/playback-path.md and
 * docs/audio/ds-capture-and-jack.md in omarchy-ipad5). Register names are
 * descriptive guesses.
 *
 * Control port: SPI, 16-bit register address (page << 8 | reg), 8-bit
 * data, framed as   write: 6C page reg 00 val
 *                   read:  6D page reg 00 xx  (value in the 5th byte)
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define CS42L81_SPI_WRITE	0x6c
#define CS42L81_SPI_READ	0x6d

#define CS42L81_DEVID_AB	0x0001	/* reads 0x42 */
#define CS42L81_DEVID_CD	0x0002	/* reads 0xa8 */
#define CS42L81_DEVID_E		0x0003	/* reads 0x1x */
#define CS42L81_REVID		0x0005
#define CS42L81_PWR_CTL1	0x0006
#define CS42L81_PWR_CTL1_PDN		BIT(0)
#define CS42L81_PWR_CTL1_BIT2		BIT(2)
#define CS42L81_PWR_CTL1_FREEZE		BIT(6)	/* set after PDN_DONE */
#define CS42L81_PWR_CTL1_REVA0		BIT(7)
#define CS42L81_PWR_CTL2	0x0007
/*
 * Bits 1/3/5 power the bias of the analog mics mic1/mic3/mic4. iOS sets
 * them only for mics with a "<mic>-bias" or "<mic>-intmic" ADT property,
 * so on J71t (DMICs only) it never touches them.
 */
#define CS42L81_ADC_PWR		0x000d
/* iOS updateRouting: 7 cleared for output, 6 for input, 5 for either */
#define CS42L81_PATH_PWR	0x000e
#define CS42L81_PATH_PWR_DAC_PDN	BIT(7)
#define CS42L81_PATH_PWR_ADC_PDN	BIT(6)
#define CS42L81_PATH_PWR_COMMON_PDN	BIT(5)
#define CS42L81_CLK_CTL		0x000f	/* [3:0] rate code, bit 7 see below */
/*
 * iOS enableAudio writes CLK_CTL[7:6,3:0] = 0x80 when the IIS link's
 * isMaster() is false (codec provides BCLK/LRCK from MCLK, J71t) and 0x0c
 * when the SoC is clock master (AppleCS42L81Audio flag +0x1ec bit 0 =
 * !AppleARMIISDevice::isMaster() = !(iisConfig[0] & 1)).
 */
#define CS42L81_CLK_CTL_CODEC_MASTER	0x80
#define CS42L81_CLK_CTL_CODEC_SLAVE	0x0c
#define CS42L81_ASP_PWR		0x0011	/* bit 7 ASP RX PDN, 6 ASP TX PDN */
#define CS42L81_ASP_PWR_RX_PDN		BIT(7)
#define CS42L81_ASP_PWR_TX_PDN		BIT(6)
#define CS42L81_ASP_CLK		0x0012	/* bit 7 ASP clock master, [3:0] rate */
/* ASP TX slot n: byte offset in the frame, bit 7 = slot unused (no width) */
#define CS42L81_ASP_TX_SLOT(n)	(0x0014 + (n))	/* n = 0..9 */
#define CS42L81_ASP_TX_SLOT_EMPTY	BIT(7)
/* ASP TX byte-lane enable mask, frame byte 0 = bit 0 of 0x0021 */
#define CS42L81_ASP_TX_EN(n)	(0x001e + (n))	/* n = 0..3, MSB first */
#define CS42L81_ASP_RX_SLOT(n)	(0x0022 + (n))	/* n = 0..9 */
#define CS42L81_INT_STATUS1	0x002e
#define CS42L81_INT_STATUS2	0x002f
#define CS42L81_INT_STATUS2_PDN_DONE	BIT(7)
#define CS42L81_INT_STATUS3	0x0033
#define CS42L81_INT_STATUS4	0x007b
#define CS42L81_INT_STATUS5	0x007c
#define CS42L81_INT_STATUS6	0x0106
/* ASP TX slot n source: 0x18 = ADC A, 0x1c = ADC B (0x20/0x24 = DMIC2A/B) */
#define CS42L81_ASP_TX_SRC(n)	(0x0115 + (n))	/* n = 0..9 */
/* ADC channel n source, same codes as ASP_TX_SRC */
#define CS42L81_ADC_SRC(n)	(0x011f + (n))	/* n = 0..1 */
#define CS42L81_SRC_RATE_A	0x0123
#define CS42L81_ASP_TX_PAIR_RATE(p)	(0x0124 + (p))	/* slot pair p = 0..4 */
#define CS42L81_ASP_PAIR_RATE(p)	(0x0129 + (p))	/* channel pair p = 0..4 */
#define CS42L81_ASP_TX_RATE	0x012e
#define CS42L81_ASP_RX_RATE	0x012f
#define CS42L81_ASP_CTL		0x0131
#define CS42L81_OUT_CTL		0x0201
/*
 * ADC input select. iOS starts at 0x04 and, per DMIC route, clears bit 2
 * and toggles 0x40 (dmic1a) / 0x80 (dmic1b); 0xc4 is its no-input value.
 */
#define CS42L81_ADC_ROUTE	0x0203
#define CS42L81_ADC_ROUTE_NONE		0xc4
#define CS42L81_ADC_ROUTE_DMIC1		0xc0
#define CS42L81_ADC_CTL_A	0x0204	/* [1:0] ADC A +20 dB boost (inferred) */
#define CS42L81_ADC_RATE_A	0x0205	/* [3:0] rate, [7:4] analog inputs */
#define CS42L81_ADC_PGA_A	0x0206	/* analog PGA, 0 for DMICs */
#define CS42L81_ADC_PGA_B	0x0207
#define CS42L81_ADC_VOL_A	0x0208	/* signed, 1 dB (inferred), 0x80 mute */
#define CS42L81_ADC_CTL_B	0x020f
#define CS42L81_ADC_CTL_C	0x0210	/* [1:0] ADC B boost (inferred) */
#define CS42L81_ADC_RATE_B	0x0211
#define CS42L81_ADC_VOL_B	0x0212
#define CS42L81_ADC_BOOST		GENMASK(1, 0)
#define CS42L81_ADC_VOL_MUTE		0x80
/* DMIC1 (1A + 1B) control: bit 7 = iOS control 0x188, [6:4] by rate */
#define CS42L81_DMIC1_CTL	0x0215
#define CS42L81_DMIC2_CTL	0x0216
#define CS42L81_HPCP_CTL	0x0219
#define CS42L81_DRV_PDN		0x0220	/* bit 5 HP PDN, bit 3 line PDN */
#define CS42L81_DRV_PDN_HP		BIT(5)
#define CS42L81_DRV_PDN_LINE		BIT(3)
#define CS42L81_DRV_CTL1	0x0221
#define CS42L81_DAC_RATE	0x0222
#define CS42L81_DRV_CTL2	0x0223
#define CS42L81_DRV_CTL3	0x0224
#define CS42L81_SPK_VOL_A	0x0225
#define CS42L81_SPK_VOL_B	0x0226
#define CS42L81_HP_VOL_A	0x0227
#define CS42L81_HP_VOL_B	0x0228
#define CS42L81_LINE_VOL_A	0x0229
#define CS42L81_LINE_VOL_B	0x022a
#define CS42L81_SPK_PDN		0x022e	/* bit 4 speaker B PDN, bit 3 speaker A PDN */
#define CS42L81_SPK_PDN_A		BIT(3)
#define CS42L81_SPK_PDN_B		BIT(4)
#define CS42L81_SPK_RATE	0x022f
#define CS42L81_DRV_CTL4	0x0230
#define CS42L81_SPK_MODE	0x0231	/* [5:4] */
#define CS42L81_SPK_MODE_MASK		GENMASK(5, 4)
#define CS42L81_MAIN_VOL	0x0232
#define CS42L81_INT_STATUS7	0x0528
#define CS42L81_TRIM1		0xc81f
#define CS42L81_TRIM2		0xc85f
#define CS42L81_TRIM3		0xc96f
#define CS42L81_UNLOCK		0x9901

#define CS42L81_VOL_MUTE	0x19	/* digital volume "mute" code */
#define CS42L81_VOL_OFFSET	0x34	/* control 0 = -102 dB */
#define CS42L81_VOL_MAX_CTL	228	/* +12 dB */
#define CS42L81_HP_MUTE		0x40

/* Headphone control: 0.5 dB steps, 0 = -50 dB, 100 = 0 dB, 124 = +12 dB */
#define CS42L81_HP_VOL_0DB	100
#define CS42L81_HP_VOL_MAX	124
#define CS42L81_HP_VOL_DEFAULT	40	/* -30 dB */
/* iOS keeps the HP charge pump off at or below -9 dB and while settling */
#define CS42L81_HP_CP_MIN_DB	(-9)

/* Capture control: 1 dB steps (inferred), 0 = -96 dB, 96 = 0 dB, +32 dB max */
#define CS42L81_ADC_VOL_0DB	96
#define CS42L81_ADC_VOL_MAX	128

#define CS42L81_PDN_POLL_US	10000
#define CS42L81_PDN_TIMEOUT_US	1000000	/* iOS: 100 x 10 ms */

/*
 * Headphone jack (docs/audio/ds-capture-and-jack.md section 3). iOS polls
 * the plug state on a timer; the codec status bit is not known yet, so
 * polling is off by default and, without jack_reg, only logs which status
 * bits change (test T1).
 */
static unsigned int jack_poll_ms;
module_param(jack_poll_ms, uint, 0644);
MODULE_PARM_DESC(jack_poll_ms,
		 "poll the codec status registers every N ms (0 = off, min 100)");
static unsigned int jack_reg;
module_param(jack_reg, uint, 0644);
MODULE_PARM_DESC(jack_reg, "status register with the plug bit (0 = unknown, log only)");
static unsigned int jack_mask;
module_param(jack_mask, uint, 0644);
MODULE_PARM_DESC(jack_mask, "plug bit(s) in jack_reg");
static bool jack_invert;
module_param(jack_invert, bool, 0644);
MODULE_PARM_DESC(jack_invert, "plug bit is set while unplugged");
/* iOS reads control 0x188 with default 1; J71t publishes no such control */
static bool dmic_bit7 = true;
module_param(dmic_bit7, bool, 0644);
MODULE_PARM_DESC(dmic_bit7, "set DMIC1 control (0x0215) bit 7 at capture start");

#define CS42L81_JACK_POLL_MIN_MS	100
#define CS42L81_JACK_DEBOUNCE		2	/* equal polls before a report */

/* latched (read-to-clear) status registers, as iOS enableAudio clears them */
static const u16 cs42l81_status_regs[] = {
	0x002e, 0x002f, 0x0033, 0x007b, 0x007c, 0x0106, 0x0528,
};

/* ASP TX slot 0/1 and ADC channel 0/1 sources: ADC A (DMIC1A), ADC B (DMIC1B) */
static const u8 cs42l81_adc_src[2] = { 0x18, 0x1c };

struct cs42l81 {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *mute_gpio;
	struct gpio_descs *hs_switch;	/* ext_sw1/2, read only */
	u8 *xfer_buf;		/* 2 x 5 bytes, DMA-safe */

	unsigned int rate_code;
	unsigned int slot_bytes;
	unsigned int spk_vol;	/* control value, 0..vol_max */
	unsigned int vol_max;
	bool spk_switch;
	bool spk_on;
	bool core_on;
	bool clk_provider;	/* codec drives BCLK/LRCK (set_fmt) */
	unsigned int dbg_reg;	/* debugfs cs42l81-reg selection */
	u32 spk_mode;		/* 2 = stereo speakers (both outputs) */

	/* headphones */
	unsigned int hp_vol;	/* control value, 0..CS42L81_HP_VOL_MAX */
	unsigned int hp_switch;
	bool hp_on;
	bool hp_settled;	/* 60 ms after the HP driver came up */
	bool hp_cp_on;		/* charge pump */

	/* capture */
	unsigned int adc_vol;	/* control value, 0..CS42L81_ADC_VOL_MAX */
	unsigned int adc_switch;
	bool adc_on;

	/* jack */
	struct snd_soc_component *component;
	struct snd_soc_jack jack;
	struct delayed_work jack_work;
	struct mutex status_lock;	/* read-to-clear status vs PDN_DONE poll */
	struct mutex jack_lock;		/* jack_* below, reports */
	bool jack_ready;
	int jack_forced;	/* -1 = detect, else forced state */
	bool jack_state;	/* last reported */
	bool jack_pending;
	unsigned int jack_stable;
	u8 status_last[ARRAY_SIZE(cs42l81_status_regs)];
	u8 status_seen[ARRAY_SIZE(cs42l81_status_regs)];
};

/* --- SPI regmap bus ------------------------------------------------------ */

static int cs42l81_bus_write(void *context, unsigned int reg, unsigned int val)
{
	struct cs42l81 *cs = context;
	u8 *tx = cs->xfer_buf;

	tx[0] = CS42L81_SPI_WRITE;
	tx[1] = reg >> 8;
	tx[2] = reg & 0x7f;
	tx[3] = 0;
	tx[4] = val;

	return spi_write(cs->spi, tx, 5);
}

static int cs42l81_bus_read(void *context, unsigned int reg, unsigned int *val)
{
	struct cs42l81 *cs = context;
	u8 *tx = cs->xfer_buf, *rx = cs->xfer_buf + 5;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 5,
	};
	int ret;

	tx[0] = CS42L81_SPI_READ;
	tx[1] = reg >> 8;
	tx[2] = reg & 0x7f;
	tx[3] = 0;
	tx[4] = 0;
	memset(rx, 0, 5);

	ret = spi_sync_transfer(cs->spi, &t, 1);
	if (ret)
		return ret;

	*val = rx[4];
	return 0;
}

static const struct regmap_bus cs42l81_bus = {
	.reg_write = cs42l81_bus_write,
	.reg_read = cs42l81_bus_read,
};

static bool cs42l81_valid_reg(struct device *dev, unsigned int reg)
{
	/* The register byte is 7 bits wide on the wire. */
	return !(reg & 0x80);
}

static const struct regmap_config cs42l81_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
	.readable_reg = cs42l81_valid_reg,
	.writeable_reg = cs42l81_valid_reg,
	.cache_type = REGCACHE_NONE,
};

/* --- helpers -------------------------------------------------------------- */

static int cs42l81_rate_code(unsigned int rate)
{
	switch (rate) {
	case 8000:	return 0x1;
	case 11025:	return 0x2;
	case 12000:	return 0x4;
	case 16000:	return 0x5;
	case 22050:	return 0x6;
	case 24000:	return 0x8;
	case 32000:	return 0x9;
	case 44100:	return 0xa;
	case 48000:	return 0xc;
	default:	return -EINVAL;
	}
}

static void cs42l81_set_rate(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int c = cs->rate_code;

	/* iOS setSampleRate (vtable +0x650) */
	regmap_update_bits(map, CS42L81_CLK_CTL, 0x0f, c);
	regmap_write(map, CS42L81_ASP_TX_RATE, c | c << 4);
	regmap_write(map, CS42L81_ASP_RX_RATE, c | c << 4);
	regmap_write(map, CS42L81_SRC_RATE_A, c);
	regmap_write(map, CS42L81_DAC_RATE, c);
	regmap_update_bits(map, CS42L81_ADC_RATE_A, 0x0f, c);
	regmap_update_bits(map, CS42L81_ADC_RATE_B, 0x0f, c);
	regmap_write(map, CS42L81_SPK_RATE, c);
}

static void cs42l81_set_asp(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int c = cs->rate_code, b = cs->slot_bytes, n;

	/* Codec drives BCLK/LRCK (MCA0 is the clock consumer). */
	regmap_write(map, CS42L81_ASP_CLK, 0x80 | c);

	/* ASP RX slot n: byte offset n*b, width b bytes */
	for (n = 0; n < 10; n++)
		regmap_write(map, CS42L81_ASP_RX_SLOT(n),
			     ((n * b) | ((b - 1) << 5)) & 0xff);
	for (n = 0; n < 5; n++)
		regmap_write(map, CS42L81_ASP_PAIR_RATE(n), c | c << 4);
}

static void cs42l81_apply_spk_volume(struct cs42l81 *cs)
{
	unsigned int v = CS42L81_VOL_MUTE;

	if (cs->spk_switch)
		v = (cs->spk_vol + CS42L81_VOL_OFFSET) & 0xff;

	/* Speaker-only state, as iOS: per-speaker volume, main volume muted */
	regmap_write(cs->regmap, CS42L81_SPK_VOL_A, v);
	regmap_write(cs->regmap, CS42L81_SPK_VOL_B, v);
	regmap_write(cs->regmap, CS42L81_MAIN_VOL, CS42L81_VOL_MUTE);
}

/* --- capture: DMIC1A/1B -> ADC A/B -> ASP TX slots 0/1 -------------------- */

/* iOS updateRouting: 0x0215 = control 0x188 << 7 | divider from the rate */
static unsigned int cs42l81_dmic_ctl(struct cs42l81 *cs)
{
	unsigned int div;

	switch (cs->rate_code) {
	case 0x1 ... 0x8:
		div = 0x70;	/* 8..24 kHz */
		break;
	case 0x9:
		div = 0x40;	/* 32 kHz */
		break;
	case 0xa:
	case 0xb:
		div = 0x10;	/* 44.1 kHz */
		break;
	default:
		div = 0;	/* 48 kHz: iOS writes no divider */
		break;
	}
	return (READ_ONCE(dmic_bit7) ? 0x80 : 0) | div;
}

/*
 * iOS DMIC gain (control 0x170, 0x60 = 0 dB): from +12 dB on the ADC
 * +20 dB boost is on and the digital volume carries the rest.
 */
static void cs42l81_apply_adc_volume(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	int db = (int)cs->adc_vol - CS42L81_ADC_VOL_0DB;
	unsigned int boost = 0, vol = CS42L81_ADC_VOL_MUTE;

	if (cs->adc_switch) {
		if (db >= 12) {
			boost = CS42L81_ADC_BOOST;
			db -= 20;
		}
		vol = db & 0xff;
	}

	regmap_update_bits(map, CS42L81_ADC_CTL_A, 0x73, boost);
	regmap_write(map, CS42L81_ADC_PGA_A, 0);
	regmap_write(map, CS42L81_ADC_PGA_B, 0);
	regmap_write(map, CS42L81_ADC_VOL_A, vol);
	regmap_update_bits(map, CS42L81_ADC_CTL_C, 0x43, boost);
	regmap_write(map, CS42L81_ADC_VOL_B, vol);
}

/* iOS updateRouting, input slots: ADC A/B in slots 0/1, slots 2..9 unused */
static void cs42l81_set_asp_tx(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int c = cs->rate_code, b = cs->slot_bytes, n;
	u32 lanes = GENMASK(2 * b - 1, 0);	/* frame bytes of slots 0/1 */

	for (n = 0; n < 10; n++) {
		regmap_write(map, CS42L81_ASP_TX_SLOT(n),
			     n * b | (n < 2 ? 0 : CS42L81_ASP_TX_SLOT_EMPTY));
		regmap_write(map, CS42L81_ASP_TX_PAIR_RATE(n / 2), c | c << 4);
		if (n < 2)
			regmap_write(map, CS42L81_ASP_TX_SRC(n),
				     cs42l81_adc_src[n]);
	}
	/* iOS writes these as one 4-byte burst of the byte-swapped mask */
	for (n = 0; n < 4; n++)
		regmap_write(map, CS42L81_ASP_TX_EN(n),
			     (lanes >> (24 - 8 * n)) & 0xff);
}

/* iOS updateRouting with the input stream active, routes dmic1a + dmic1b */
static void cs42l81_capture_power_up(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int ch;

	cs42l81_set_asp_tx(cs);
	for (ch = 0; ch < 2; ch++) {
		/* digital input: no analog input select bits */
		regmap_update_bits(map, CS42L81_ADC_RATE_A, ch ? 0xc0 : 0x30, 0);
		regmap_write(map, CS42L81_ADC_SRC(ch), cs42l81_adc_src[ch]);
	}
	cs42l81_apply_adc_volume(cs);
	/* CS42L81_ADC_PWR (analog mic bias) stays as it is, as on iOS */
	regmap_write(map, CS42L81_ADC_ROUTE, CS42L81_ADC_ROUTE_DMIC1);
	regmap_write(map, CS42L81_ADC_CTL_B, 0x04);
	regmap_clear_bits(map, CS42L81_ASP_PWR, CS42L81_ASP_PWR_TX_PDN);
	regmap_set_bits(map, CS42L81_ASP_CTL, 0x01);
	regmap_write(map, CS42L81_DMIC1_CTL, cs42l81_dmic_ctl(cs));
	regmap_clear_bits(map, CS42L81_PATH_PWR, CS42L81_PATH_PWR_ADC_PDN |
			  CS42L81_PATH_PWR_COMMON_PDN);
	cs->adc_on = true;
}

/* iOS updateRouting once the input stream is gone; 0x0215 is left as is */
static void cs42l81_capture_power_down(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;

	cs->adc_on = false;
	regmap_write(map, CS42L81_ADC_ROUTE, CS42L81_ADC_ROUTE_NONE);
	regmap_write(map, CS42L81_ADC_CTL_B, 0x04);
	regmap_set_bits(map, CS42L81_ASP_PWR, CS42L81_ASP_PWR_TX_PDN);
	regmap_set_bits(map, CS42L81_PATH_PWR, CS42L81_PATH_PWR_ADC_PDN);
}

/* --- headphones ------------------------------------------------------------ */

static void cs42l81_hp_cp_on(struct cs42l81 *cs)
{
	/* iOS setHeadphoneVolume */
	regmap_write(cs->regmap, CS42L81_TRIM3, 0x0e);
	regmap_write(cs->regmap, CS42L81_HPCP_CTL, 0x01);
	usleep_range(155, 200);		/* iOS IODelay(155) */
	regmap_write(cs->regmap, CS42L81_TRIM3, 0x1e);
	cs->hp_cp_on = true;
}

static void cs42l81_hp_cp_off(struct cs42l81 *cs)
{
	/* iOS updateRouting with the headphone route gone */
	regmap_write(cs->regmap, CS42L81_TRIM3, 0x0e);
	usleep_range(1000, 2000);
	regmap_write(cs->regmap, CS42L81_HPCP_CTL, 0x00);
	cs->hp_cp_on = false;
}

/*
 * iOS setHeadphoneVolume without a line out (J71t): the HP driver gain in
 * 1 dB steps (0x0227, 7-bit signed, 0x40 = mute) plus a 0.5 dB trim in the
 * digital volume (0x0225). Until the driver has settled, and at or below
 * -9 dB, the charge pump stays off and the gain is at most -9 dB.
 */
static void cs42l81_apply_hp_volume(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	int g = (int)cs->hp_vol - CS42L81_HP_VOL_0DB;	/* 0.5 dB */
	int gain = g / 2, trim = g - 2 * gain;

	if (!cs->hp_on)
		return;

	if (!cs->hp_switch) {
		regmap_write(map, CS42L81_HP_VOL_A, CS42L81_HP_MUTE);
		return;
	}

	regmap_write(map, CS42L81_SPK_VOL_A, trim & 0xff);
	/* iOS leaves 0x0226 alone here; keep both channels equal (inferred) */
	regmap_write(map, CS42L81_SPK_VOL_B, trim & 0xff);
	if (gain > CS42L81_HP_CP_MIN_DB) {
		if (!cs->hp_settled)
			gain = CS42L81_HP_CP_MIN_DB;
		else if (!cs->hp_cp_on)
			cs42l81_hp_cp_on(cs);
	}
	regmap_write(map, CS42L81_HP_VOL_A, gain & 0x7f);
}

/* --- power sequences (DAPM) ---------------------------------------------- */

static int cs42l81_core_power_up(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int rev, val;
	int ret;

	ret = clk_prepare_enable(cs->mclk);
	if (ret) {
		dev_err(cs->dev, "could not enable MCLK: %d\n", ret);
		return ret;
	}

	/* iOS enableAudio (vtable +0x600), cold path */
	regmap_write(map, CS42L81_UNLOCK, 0xa5);
	msleep(10);
	regmap_write(map, CS42L81_UNLOCK, 0x00);

	/* read to clear latched status */
	mutex_lock(&cs->status_lock);
	regmap_read(map, CS42L81_INT_STATUS1, &val);
	regmap_read(map, CS42L81_INT_STATUS2, &val);
	regmap_read(map, CS42L81_INT_STATUS3, &val);
	regmap_read(map, CS42L81_INT_STATUS4, &val);
	regmap_read(map, CS42L81_INT_STATUS5, &val);
	regmap_read(map, CS42L81_INT_STATUS6, &val);
	regmap_read(map, CS42L81_INT_STATUS7, &val);
	mutex_unlock(&cs->status_lock);

	ret = regmap_read(map, CS42L81_REVID, &rev);
	if (ret)
		goto err;
	if (rev == 0xa0)
		regmap_set_bits(map, CS42L81_PWR_CTL1, CS42L81_PWR_CTL1_REVA0);

	regmap_clear_bits(map, CS42L81_PWR_CTL2, 0x80);
	regmap_update_bits(map, CS42L81_CLK_CTL, 0xcf,
			   cs->clk_provider ? CS42L81_CLK_CTL_CODEC_MASTER :
					      CS42L81_CLK_CTL_CODEC_SLAVE);
	regmap_clear_bits(map, CS42L81_ASP_CLK, 0xc0);
	regmap_clear_bits(map, CS42L81_ASP_PWR, 0x10);

	regmap_update_bits(map, CS42L81_PWR_CTL1,
			   CS42L81_PWR_CTL1_PDN | CS42L81_PWR_CTL1_BIT2,
			   CS42L81_PWR_CTL1_BIT2);

	regmap_write(map, CS42L81_OUT_CTL, 0xe0);
	regmap_write(map, CS42L81_HPCP_CTL, 0x00);
	regmap_write(map, CS42L81_TRIM1, 0xff);
	regmap_write(map, CS42L81_TRIM2, 0x0f);
	regmap_write(map, CS42L81_TRIM3, 0x0e);
	cs->hp_cp_on = false;
	regmap_set_bits(map, CS42L81_DRV_CTL1, 0x20);
	regmap_write(map, CS42L81_DRV_CTL2, 0x08);
	regmap_write(map, CS42L81_DRV_CTL3, 0x09);
	regmap_set_bits(map, CS42L81_ADC_CTL_A, 0x04);
	regmap_set_bits(map, CS42L81_ADC_CTL_C, 0x04);
	regmap_write(map, CS42L81_DRV_CTL4, 0x0c);

	cs42l81_set_rate(cs);

	/* updateRouting: playback-only state */
	cs42l81_set_asp(cs);
	regmap_update_bits(map, CS42L81_ADC_PWR, 0x3f, 0);
	regmap_write(map, CS42L81_ADC_ROUTE, 0xc4);
	regmap_write(map, CS42L81_ADC_CTL_B, 0x04);
	regmap_write(map, CS42L81_DRV_CTL2, 0x08);
	regmap_write(map, CS42L81_DRV_CTL3, 0x09);
	regmap_update_bits(map, CS42L81_PWR_CTL1, CS42L81_PWR_CTL1_BIT2,
			   CS42L81_PWR_CTL1_BIT2);
	regmap_update_bits(map, CS42L81_ASP_PWR, 0xc0, 0x40);
	regmap_set_bits(map, CS42L81_ASP_CTL, 0x01);
	regmap_clear_bits(map, CS42L81_PATH_PWR, 0xa0);
	regmap_set_bits(map, CS42L81_PATH_PWR, 0x40);

	cs->core_on = true;
	/* latched status after power-up: clock-error bits help the bring-up */
	{
		unsigned int st[4] = {};

		mutex_lock(&cs->status_lock);
		regmap_read(map, CS42L81_INT_STATUS1, &st[0]);
		regmap_read(map, CS42L81_INT_STATUS2, &st[1]);
		regmap_read(map, CS42L81_INT_STATUS3, &st[2]);
		mutex_unlock(&cs->status_lock);
		regmap_read(map, CS42L81_CLK_CTL, &st[3]);
		dev_info(cs->dev,
			 "powered up: rev %#x clk_ctl %#x status 2e=%#x 2f=%#x 33=%#x\n",
			 rev, st[3], st[0], st[1], st[2]);
	}
	return 0;

err:
	clk_disable_unprepare(cs->mclk);
	return ret;
}

static void cs42l81_core_power_down(struct cs42l81 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int val;
	int ret;

	/* iOS disableAudio (vtable +0x608) */
	cs->core_on = false;
	regmap_write(map, CS42L81_HP_VOL_A, CS42L81_HP_MUTE);
	regmap_write(map, CS42L81_HP_VOL_B, CS42L81_HP_MUTE);
	regmap_write(map, CS42L81_LINE_VOL_A, CS42L81_HP_MUTE);
	regmap_write(map, CS42L81_LINE_VOL_B, CS42L81_HP_MUTE);
	regmap_write(map, CS42L81_SPK_VOL_A, CS42L81_VOL_MUTE);
	regmap_write(map, CS42L81_SPK_VOL_B, CS42L81_VOL_MUTE);
	regmap_write(map, CS42L81_MAIN_VOL, CS42L81_VOL_MUTE);
	regmap_write(map, CS42L81_DRV_PDN, 0x28);
	regmap_write(map, CS42L81_SPK_PDN, 0x18);

	/* the jack poll must not consume PDN_DONE (read to clear) */
	mutex_lock(&cs->status_lock);
	regmap_read(map, CS42L81_INT_STATUS2, &val);
	if (val & CS42L81_INT_STATUS2_PDN_DONE)
		dev_dbg(cs->dev, "PDN_DONE already set before power down\n");

	regmap_update_bits(map, CS42L81_PWR_CTL1,
			   CS42L81_PWR_CTL1_PDN | CS42L81_PWR_CTL1_BIT2,
			   CS42L81_PWR_CTL1_PDN | CS42L81_PWR_CTL1_BIT2);

	ret = regmap_read_poll_timeout(map, CS42L81_INT_STATUS2, val,
				       val & CS42L81_INT_STATUS2_PDN_DONE,
				       CS42L81_PDN_POLL_US,
				       CS42L81_PDN_TIMEOUT_US);
	mutex_unlock(&cs->status_lock);
	if (ret)
		dev_warn(cs->dev, "timed out waiting for codec shutdown\n");

	regmap_set_bits(map, CS42L81_PWR_CTL1, CS42L81_PWR_CTL1_FREEZE);
	msleep(10);
	regmap_write(map, CS42L81_UNLOCK, 0xa5);

	clk_disable_unprepare(cs->mclk);
}

static int cs42l81_core_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return cs42l81_core_power_up(cs);
	case SND_SOC_DAPM_POST_PMD:
		cs42l81_core_power_down(cs);
		break;
	}

	return 0;
}

static int cs42l81_spk_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	struct regmap *map = cs->regmap;
	unsigned int pdn = CS42L81_SPK_PDN_A | CS42L81_SPK_PDN_B;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/*
		 * iOS never drives speakers and headphones together (it mutes
		 * the speakers when the headphones take the volume); the jack
		 * pins keep the two exclusive, this only guards a manual mix.
		 */
		if (cs->hp_on) {
			dev_warn(cs->dev, "headphones active, speakers stay off\n");
			break;
		}
		gpiod_set_value_cansleep(cs->mute_gpio, 1);
		regmap_update_bits(map, CS42L81_SPK_MODE, CS42L81_SPK_MODE_MASK,
				   cs->spk_mode == 2 ? 0 : 0x10);
		regmap_write(map, CS42L81_OUT_CTL, 0xe0);
		regmap_update_bits(map, CS42L81_DRV_PDN,
				   CS42L81_DRV_PDN_HP | CS42L81_DRV_PDN_LINE,
				   CS42L81_DRV_PDN_HP | CS42L81_DRV_PDN_LINE);
		regmap_update_bits(map, CS42L81_SPK_PDN, pdn,
				   cs->spk_mode == 2 ? 0 : CS42L81_SPK_PDN_B);
		msleep(50);
		cs->spk_on = true;
		cs42l81_apply_spk_volume(cs);
		/* iOS waits for the volume ramp: (v + 27) / 2 ms */
		msleep((cs->spk_vol + 27) / 2);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (cs->spk_on)
			gpiod_set_value_cansleep(cs->mute_gpio, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (!cs->spk_on)
			break;
		gpiod_set_value_cansleep(cs->mute_gpio, 1);
		cs->spk_on = false;
		regmap_write(map, CS42L81_SPK_VOL_A, CS42L81_VOL_MUTE);
		regmap_write(map, CS42L81_SPK_VOL_B, CS42L81_VOL_MUTE);
		regmap_update_bits(map, CS42L81_SPK_PDN, pdn, pdn);
		msleep(20);
		break;
	}

	return 0;
}

/* iOS updateRouting: speaker route -> headphone route, and back */
static int cs42l81_hp_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	struct regmap *map = cs->regmap;
	unsigned int pdn = CS42L81_SPK_PDN_A | CS42L81_SPK_PDN_B, old = pdn;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (cs->spk_on) {
			dev_warn(cs->dev, "speakers active, headphones stay off\n");
			break;
		}
		gpiod_set_value_cansleep(cs->mute_gpio, 1);
		regmap_write(map, CS42L81_OUT_CTL, 0xe0);
		/* HP driver on, line driver off */
		regmap_update_bits(map, CS42L81_DRV_PDN,
				   CS42L81_DRV_PDN_HP | CS42L81_DRV_PDN_LINE,
				   CS42L81_DRV_PDN_LINE);
		/* speakers off; iOS drops OUT_CTL bit 5 around the change */
		regmap_read(map, CS42L81_SPK_PDN, &old);
		if (~old & pdn) {
			regmap_write(map, CS42L81_OUT_CTL, 0xc0);
			regmap_update_bits(map, CS42L81_SPK_PDN, pdn, pdn);
			regmap_write(map, CS42L81_OUT_CTL, 0xe0);
			msleep(20);
		}
		cs->hp_on = true;
		cs->hp_settled = false;
		cs42l81_apply_hp_volume(cs);	/* at most -9 dB, pump off */
		msleep(60);
		cs->hp_settled = true;
		cs42l81_apply_hp_volume(cs);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (!cs->hp_on)
			break;
		cs->hp_on = false;
		cs->hp_settled = false;
		regmap_write(map, CS42L81_LINE_VOL_A, CS42L81_HP_MUTE);
		regmap_write(map, CS42L81_HP_VOL_A, CS42L81_HP_MUTE);
		regmap_write(map, CS42L81_SPK_VOL_A, CS42L81_VOL_MUTE);
		regmap_write(map, CS42L81_SPK_VOL_B, CS42L81_VOL_MUTE);
		regmap_update_bits(map, CS42L81_DRV_PDN,
				   CS42L81_DRV_PDN_HP | CS42L81_DRV_PDN_LINE,
				   CS42L81_DRV_PDN_HP | CS42L81_DRV_PDN_LINE);
		msleep(20);
		if (cs->hp_cp_on)
			cs42l81_hp_cp_off(cs);
		break;
	}

	return 0;
}

static int cs42l81_adc_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cs42l81_capture_power_up(cs);
		break;
	case SND_SOC_DAPM_POST_PMD:
		cs42l81_capture_power_down(cs);
		break;
	}

	return 0;
}

/* --- controls ------------------------------------------------------------- */

/* 0.5 dB steps; control 0 = -102 dB. Max is limited by spkr-gain-db. */
static const DECLARE_TLV_DB_SCALE(cs42l81_spk_tlv, -10200, 50, 0);

static int cs42l81_spk_vol_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = cs->vol_max;
	return 0;
}

static int cs42l81_spk_vol_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->spk_vol;
	return 0;
}

static int cs42l81_spk_vol_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	long v = ucontrol->value.integer.value[0];

	if (v < 0 || v > cs->vol_max)
		return -EINVAL;
	if (v == cs->spk_vol)
		return 0;

	cs->spk_vol = v;
	if (cs->spk_on)
		cs42l81_apply_spk_volume(cs);
	return 1;
}

static int cs42l81_spk_sw_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->spk_switch;
	return 0;
}

static int cs42l81_spk_sw_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	bool on = !!ucontrol->value.integer.value[0];

	if (on == cs->spk_switch)
		return 0;

	cs->spk_switch = on;
	if (cs->spk_on)
		cs42l81_apply_spk_volume(cs);
	return 1;
}

/* Headphone and capture controls: the soc_mixer_control reg is the field */
enum {
	CS42L81_CTL_HP_VOL,
	CS42L81_CTL_HP_SW,
	CS42L81_CTL_ADC_VOL,
	CS42L81_CTL_ADC_SW,
};

static unsigned int *cs42l81_ctl_field(struct cs42l81 *cs, unsigned int id)
{
	switch (id) {
	case CS42L81_CTL_HP_VOL:
		return &cs->hp_vol;
	case CS42L81_CTL_HP_SW:
		return &cs->hp_switch;
	case CS42L81_CTL_ADC_VOL:
		return &cs->adc_vol;
	default:
		return &cs->adc_switch;
	}
}

static int cs42l81_ctl_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = *cs42l81_ctl_field(cs, mc->reg);
	return 0;
}

static int cs42l81_ctl_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int *field = cs42l81_ctl_field(cs, mc->reg);
	long v = ucontrol->value.integer.value[0];

	if (v < 0 || v > mc->max)
		return -EINVAL;
	if (v == *field)
		return 0;

	*field = v;
	if (mc->reg == CS42L81_CTL_HP_VOL || mc->reg == CS42L81_CTL_HP_SW)
		cs42l81_apply_hp_volume(cs);
	else if (cs->adc_on)
		cs42l81_apply_adc_volume(cs);
	return 1;
}

/* HP: 0.5 dB steps from -50 dB. Capture: 1 dB steps from -96 dB (inferred) */
static const DECLARE_TLV_DB_SCALE(cs42l81_hp_tlv, -5000, 50, 0);
static const DECLARE_TLV_DB_SCALE(cs42l81_adc_tlv, -9600, 100, 0);

static const struct snd_kcontrol_new cs42l81_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Speaker Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.tlv.p = cs42l81_spk_tlv,
		.info = cs42l81_spk_vol_info,
		.get = cs42l81_spk_vol_get,
		.put = cs42l81_spk_vol_put,
	},
	SOC_SINGLE_BOOL_EXT("Speaker Playback Switch", 0,
			    cs42l81_spk_sw_get, cs42l81_spk_sw_put),
	SOC_SINGLE_EXT_TLV("Headphone Playback Volume", CS42L81_CTL_HP_VOL, 0,
			   CS42L81_HP_VOL_MAX, 0, cs42l81_ctl_get,
			   cs42l81_ctl_put, cs42l81_hp_tlv),
	SOC_SINGLE_EXT("Headphone Playback Switch", CS42L81_CTL_HP_SW, 0, 1, 0,
		       cs42l81_ctl_get, cs42l81_ctl_put),
	SOC_SINGLE_EXT_TLV("Capture Volume", CS42L81_CTL_ADC_VOL, 0,
			   CS42L81_ADC_VOL_MAX, 0, cs42l81_ctl_get,
			   cs42l81_ctl_put, cs42l81_adc_tlv),
	SOC_SINGLE_EXT("Capture Switch", CS42L81_CTL_ADC_SW, 0, 1, 0,
		       cs42l81_ctl_get, cs42l81_ctl_put),
};

static const struct snd_soc_dapm_widget cs42l81_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Core", SND_SOC_NOPM, 0, 0, cs42l81_core_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN("ASPRX", "ASP Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("Speaker Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs42l81_spk_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HP Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs42l81_hp_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPKOUT"),
	SND_SOC_DAPM_OUTPUT("HPOUT"),

	/* the two built-in DMICs (ADT dmic1a-intmic, dmic1b-intmic) */
	SND_SOC_DAPM_INPUT("DMIC1A"),
	SND_SOC_DAPM_INPUT("DMIC1B"),
	SND_SOC_DAPM_ADC_E("ADC", NULL, SND_SOC_NOPM, 0, 0, cs42l81_adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("ASPTX", "ASP Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route cs42l81_routes[] = {
	{ "ASPRX", NULL, "Core" },
	{ "Speaker Amp", NULL, "ASPRX" },
	{ "SPKOUT", NULL, "Speaker Amp" },
	{ "HP Amp", NULL, "ASPRX" },
	{ "HPOUT", NULL, "HP Amp" },

	{ "ADC", NULL, "Core" },
	{ "ADC", NULL, "DMIC1A" },
	{ "ADC", NULL, "DMIC1B" },
	{ "ASPTX", NULL, "ADC" },
};

/* --- DAI ------------------------------------------------------------------ */

static int cs42l81_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The codec provides BCLK and LRCK; MCA0 is the consumer (iOS). */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBP_CFP)
		return -EINVAL;
	((struct cs42l81 *)snd_soc_component_get_drvdata(dai->component))->clk_provider = true;
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

static int cs42l81_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct cs42l81 *cs = snd_soc_component_get_drvdata(dai->component);
	int code = cs42l81_rate_code(params_rate(params));
	unsigned int width = params_physical_width(params);

	if (code < 0)
		return code;
	if (width != 16 && width != 32)
		return -EINVAL;

	if (code == cs->rate_code && width / 8 == cs->slot_bytes)
		return 0;

	cs->rate_code = code;
	cs->slot_bytes = width / 8;

	/* DAPM keeps the core up for pmdown_time after a stream: reapply. */
	if (cs->core_on) {
		cs42l81_set_rate(cs);
		cs42l81_set_asp(cs);
	}
	if (cs->adc_on) {
		cs42l81_set_asp_tx(cs);
		regmap_write(cs->regmap, CS42L81_DMIC1_CTL, cs42l81_dmic_ctl(cs));
	}
	return 0;
}

static const struct snd_soc_dai_ops cs42l81_dai_ops = {
	.set_fmt = cs42l81_set_fmt,
	.hw_params = cs42l81_hw_params,
};

static struct snd_soc_dai_driver cs42l81_dai = {
	.name = "cs42l81-asp",
	.playback = {
		.stream_name = "ASP Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "ASP Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &cs42l81_dai_ops,
	/* one ASP clock and one slot width for both directions */
	.symmetric_rate = 1,
	.symmetric_sample_bits = 1,
};

/* --- headphone jack -------------------------------------------------------- */

/*
 * A plug moves playback from SPKOUT to HPOUT. The pins are switched here,
 * not as jack pins, because snd_soc_jack_add_pins() would add a "<pin>
 * Jack" control per pin; userspace looks for "Headphone Jack".
 */
static void cs42l81_jack_report(struct cs42l81 *cs, bool plugged)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(cs->component);

	cs->jack_state = plugged;
	dev_info(cs->dev, "headphones %s\n", plugged ? "plugged" : "unplugged");
	if (plugged) {
		snd_soc_dapm_disable_pin(dapm, "SPKOUT");
		snd_soc_dapm_enable_pin(dapm, "HPOUT");
	} else {
		snd_soc_dapm_disable_pin(dapm, "HPOUT");
		snd_soc_dapm_enable_pin(dapm, "SPKOUT");
	}
	snd_soc_dapm_sync(dapm);
	snd_soc_jack_report(&cs->jack, plugged ? SND_JACK_HEADPHONE : 0,
			    SND_JACK_HEADPHONE);
}

/* hs-switch lines (ADT function-ext_sw1/2) are only read, never driven */
static int cs42l81_hs_switch_str(struct cs42l81 *cs, char *buf, size_t len)
{
	unsigned int i;
	int n = 0;

	for (i = 0; cs->hs_switch && i < cs->hs_switch->ndescs; i++) {
		struct gpio_desc *d = cs->hs_switch->desc[i];

		n += scnprintf(buf + n, len - n, " ext_sw%u %s=%d", i + 1,
			       gpiod_get_direction(d) ? "in" : "out",
			       gpiod_get_raw_value_cansleep(d));
	}
	if (!n && len)
		buf[0] = '\0';
	return n;
}

/*
 * Survey (test T1): log every change of the latched status registers with
 * the hs-switch levels. With jack_reg/jack_mask set, also debounce that bit
 * and report it through the jack, whose pins switch HPOUT and SPKOUT.
 */
static void cs42l81_jack_work(struct work_struct *work)
{
	struct cs42l81 *cs = container_of(to_delayed_work(work),
					  struct cs42l81, jack_work);
	unsigned int period = READ_ONCE(jack_poll_ms);
	unsigned int reg = READ_ONCE(jack_reg), mask = READ_ONCE(jack_mask);
	unsigned int i, val, det = 0;
	bool have_det = false, plugged, rearm;
	char sw[64];

	if (!period)
		return;

	mutex_lock(&cs->status_lock);
	for (i = 0; i < ARRAY_SIZE(cs42l81_status_regs); i++) {
		if (regmap_read(cs->regmap, cs42l81_status_regs[i], &val))
			continue;
		cs->status_seen[i] |= val;
		if (val != cs->status_last[i]) {
			cs42l81_hs_switch_str(cs, sw, sizeof(sw));
			dev_info_ratelimited(cs->dev,
					     "status %04x: %02x -> %02x%s\n",
					     cs42l81_status_regs[i],
					     cs->status_last[i], val, sw);
			cs->status_last[i] = val;
		}
		if (cs42l81_status_regs[i] == reg) {
			det = val;
			have_det = true;
		}
	}
	mutex_unlock(&cs->status_lock);

	/* reports take the DAPM mutex: never under status_lock */
	mutex_lock(&cs->jack_lock);
	if (cs->jack_ready && have_det && mask && cs->jack_forced < 0) {
		plugged = !!(det & mask) ^ READ_ONCE(jack_invert);
		if (plugged != cs->jack_pending) {
			cs->jack_pending = plugged;
			cs->jack_stable = 1;
		} else if (cs->jack_stable < CS42L81_JACK_DEBOUNCE) {
			cs->jack_stable++;
		}
		if (cs->jack_stable >= CS42L81_JACK_DEBOUNCE &&
		    plugged != cs->jack_state)
			cs42l81_jack_report(cs, plugged);
	}
	/* component remove clears jack_ready, then cancels: no requeue after */
	rearm = cs->jack_ready;
	mutex_unlock(&cs->jack_lock);

	if (rearm)
		schedule_delayed_work(&cs->jack_work,
				      msecs_to_jiffies(max(period,
							   CS42L81_JACK_POLL_MIN_MS)));
}

static int cs42l81_component_probe(struct snd_soc_component *component)
{
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);
	int ret;

	cs->component = component;
	ret = snd_soc_card_jack_new(component->card, "Headphone Jack",
				    SND_JACK_HEADPHONE, &cs->jack);
	if (ret)
		return dev_err_probe(cs->dev, ret, "cannot create jack\n");

	/*
	 * Until something reports a plug (the poll, once test T1 names the
	 * bit, or debugfs cs42l81-jack) the speakers stay selected.
	 */
	ret = snd_soc_dapm_disable_pin(snd_soc_component_to_dapm(component),
				       "HPOUT");
	if (ret)
		dev_warn(cs->dev, "cannot deselect the headphones: %d\n", ret);

	mutex_lock(&cs->jack_lock);
	cs->jack_state = false;
	cs->jack_pending = false;
	cs->jack_stable = 0;
	cs->jack_forced = -1;
	cs->jack_ready = true;
	mutex_unlock(&cs->jack_lock);
	if (READ_ONCE(jack_poll_ms))
		schedule_delayed_work(&cs->jack_work, 0);
	return 0;
}

static void cs42l81_component_remove(struct snd_soc_component *component)
{
	struct cs42l81 *cs = snd_soc_component_get_drvdata(component);

	mutex_lock(&cs->jack_lock);
	cs->jack_ready = false;
	mutex_unlock(&cs->jack_lock);
	cancel_delayed_work_sync(&cs->jack_work);
}

static const struct snd_soc_component_driver cs42l81_component = {
	.probe = cs42l81_component_probe,
	.remove = cs42l81_component_remove,
	.controls = cs42l81_controls,
	.num_controls = ARRAY_SIZE(cs42l81_controls),
	.dapm_widgets = cs42l81_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs42l81_widgets),
	.dapm_routes = cs42l81_routes,
	.num_dapm_routes = ARRAY_SIZE(cs42l81_routes),
	.endianness = 1,
};

/* --- debugfs (bring-up) ---------------------------------------------------- */

/*
 * Every register the driver touches, plus the latched status registers
 * (0x002e, 0x002f, 0x0033, 0x007b, 0x007c, 0x0106, 0x0528: read to clear).
 */
static const u16 cs42l81_dbg_regs[] = {
	0x0001, 0x0002, 0x0003, 0x0005, 0x0006, 0x0007, 0x000d, 0x000e,
	0x000f, 0x0011, 0x0012, 0x0014, 0x0015, 0x0016, 0x001e, 0x001f,
	0x0020, 0x0021, 0x0022, 0x0023, 0x002e, 0x002f, 0x0033, 0x007b,
	0x007c, 0x0106, 0x0115, 0x0116, 0x011f, 0x0120, 0x0123, 0x0124,
	0x0129, 0x012e, 0x012f, 0x0131, 0x0201, 0x0203, 0x0204, 0x0205,
	0x0206, 0x0207, 0x0208, 0x020f, 0x0210, 0x0211, 0x0212, 0x0215,
	0x0216, 0x0219, 0x0220, 0x0221, 0x0222, 0x0223, 0x0224, 0x0225,
	0x0226, 0x0227, 0x0228, 0x0229, 0x022e, 0x022f, 0x0230, 0x0231,
	0x0232, 0x0528, 0x9901, 0xc81f, 0xc85f, 0xc96f,
};

static int cs42l81_regs_show(struct seq_file *s, void *unused)
{
	struct cs42l81 *cs = dev_get_drvdata(s->private);
	unsigned int i, val;

	seq_printf(s, "core_on %d spk_on %d hp_on %d (cp %d) adc_on %d clk_provider %d rate %#x slot %u\n",
		   cs->core_on, cs->spk_on, cs->hp_on, cs->hp_cp_on, cs->adc_on,
		   cs->clk_provider, cs->rate_code, cs->slot_bytes);
	mutex_lock(&cs->status_lock);
	for (i = 0; i < ARRAY_SIZE(cs42l81_dbg_regs); i++) {
		if (regmap_read(cs->regmap, cs42l81_dbg_regs[i], &val))
			seq_printf(s, "%04x = (err)\n", cs42l81_dbg_regs[i]);
		else
			seq_printf(s, "%04x = %02x\n", cs42l81_dbg_regs[i], val);
	}
	mutex_unlock(&cs->status_lock);
	return 0;
}

/*
 * cs42l81-jack. Read: poll settings, jack state, the OR of every status
 * value the poll has seen (cleared by the read) and the hs-switch levels.
 * Write: "1"/"0" force a plug/unplug report (tests T3/T4 before the status
 * bit is known), "auto" returns to the poll, "poll" restarts the poll after
 * jack_poll_ms changed.
 */
static int cs42l81_jack_show(struct seq_file *s, void *unused)
{
	struct cs42l81 *cs = s->private;
	unsigned int i;
	char sw[64];

	seq_printf(s, "poll %u ms, reg %#x mask %#x invert %d\n",
		   READ_ONCE(jack_poll_ms), READ_ONCE(jack_reg),
		   READ_ONCE(jack_mask), READ_ONCE(jack_invert));
	mutex_lock(&cs->jack_lock);
	seq_printf(s, "jack %s, reported %s\n",
		   !cs->jack_ready ? "not created" :
		   cs->jack_forced < 0 ? "auto" :
		   cs->jack_forced ? "forced plugged" : "forced unplugged",
		   cs->jack_state ? "plugged" : "unplugged");
	mutex_unlock(&cs->jack_lock);

	mutex_lock(&cs->status_lock);
	seq_puts(s, "seen:");
	for (i = 0; i < ARRAY_SIZE(cs42l81_status_regs); i++) {
		seq_printf(s, " %04x=%02x", cs42l81_status_regs[i],
			   cs->status_seen[i]);
		cs->status_seen[i] = 0;
	}
	mutex_unlock(&cs->status_lock);

	cs42l81_hs_switch_str(cs, sw, sizeof(sw));
	seq_printf(s, "\nhs-switch:%s\n", sw[0] ? sw : " (none)");
	return 0;
}

static int cs42l81_jack_open(struct inode *inode, struct file *file)
{
	return single_open(file, cs42l81_jack_show, inode->i_private);
}

static ssize_t cs42l81_jack_write(struct file *f, const char __user *ubuf,
				  size_t len, loff_t *ppos)
{
	struct cs42l81 *cs = ((struct seq_file *)f->private_data)->private;
	char buf[16] = {}, *cmd;
	int forced;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	cmd = strim(buf);

	if (!strcmp(cmd, "1") || !strcmp(cmd, "plug"))
		forced = 1;
	else if (!strcmp(cmd, "0") || !strcmp(cmd, "unplug"))
		forced = 0;
	else if (!strcmp(cmd, "auto"))
		forced = -1;
	else if (!strcmp(cmd, "poll"))
		forced = -2;	/* only restart the poll */
	else
		return -EINVAL;

	mutex_lock(&cs->jack_lock);
	if (!cs->jack_ready) {
		mutex_unlock(&cs->jack_lock);
		return -ENODEV;
	}
	if (forced != -2) {
		cs->jack_forced = forced;
		cs->jack_stable = 0;
		if (forced >= 0 && forced != cs->jack_state)
			cs42l81_jack_report(cs, forced);
	}
	if (READ_ONCE(jack_poll_ms))
		mod_delayed_work(system_wq, &cs->jack_work, 0);
	mutex_unlock(&cs->jack_lock);
	return len;
}

static const struct file_operations cs42l81_jack_fops = {
	.open = cs42l81_jack_open,
	.read = seq_read,
	.write = cs42l81_jack_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* write "REG" to select, "REG VAL" to write; read prints the selection */
static ssize_t cs42l81_reg_write(struct file *f, const char __user *ubuf,
				 size_t len, loff_t *ppos)
{
	struct cs42l81 *cs = f->private_data;
	unsigned int reg, val;
	char buf[32] = {};
	int n;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	n = sscanf(buf, "%x %x", &reg, &val);
	if (n < 1 || reg > 0xffff || (reg & 0x80))
		return -EINVAL;
	cs->dbg_reg = reg;
	if (n == 2 && regmap_write(cs->regmap, reg, val & 0xff))
		return -EIO;
	return len;
}

static ssize_t cs42l81_reg_read(struct file *f, char __user *ubuf,
				size_t len, loff_t *ppos)
{
	struct cs42l81 *cs = f->private_data;
	unsigned int val = 0;
	char buf[32];
	int n;

	if (regmap_read(cs->regmap, cs->dbg_reg, &val))
		return -EIO;
	n = scnprintf(buf, sizeof(buf), "%04x = %02x\n", cs->dbg_reg, val);
	return simple_read_from_buffer(ubuf, len, ppos, buf, n);
}

static const struct file_operations cs42l81_reg_fops = {
	.open = simple_open,
	.read = cs42l81_reg_read,
	.write = cs42l81_reg_write,
	.llseek = default_llseek,
};

/* --- probe ---------------------------------------------------------------- */

static void cs42l81_debugfs_remove(void *dentry)
{
	debugfs_remove(dentry);
}

static void cs42l81_cancel_jack_work(void *data)
{
	struct cs42l81 *cs = data;

	cancel_delayed_work_sync(&cs->jack_work);
}

static int cs42l81_check_id(struct cs42l81 *cs)
{
	unsigned int id[3] = {}, rev = 0;
	int ret, i;

	/* MCLK is not known to be needed for the control port; keep it on
	 * anyway while probing so a negative result is meaningful. */
	dev_info(cs->dev, "codec ID MCLK enable begin\n");
	ret = clk_prepare_enable(cs->mclk);
	dev_info(cs->dev, "codec ID MCLK enable end ret=%d\n", ret);
	if (ret)
		return dev_err_probe(cs->dev, ret, "could not enable MCLK\n");
	dev_info(cs->dev, "codec ID SPI reads begin\n");

	for (i = 0; i < 3; i++) {
		ret = regmap_read(cs->regmap, CS42L81_DEVID_AB + i, &id[i]);
		if (ret)
			break;
	}
	if (!ret)
		ret = regmap_read(cs->regmap, CS42L81_REVID, &rev);
	dev_info(cs->dev, "codec ID SPI reads end ret=%d; MCLK disable begin\n", ret);
	clk_disable_unprepare(cs->mclk);
	dev_info(cs->dev, "codec ID MCLK disable end\n");

	if (ret)
		return dev_err_probe(cs->dev, ret, "SPI read failed\n");

	if (id[0] != 0x42 || id[1] != 0xa8 || (id[2] & 0xf0) != 0x10)
		return dev_err_probe(cs->dev, -ENODEV,
				     "unexpected ID %02x %02x %02x (want 42 a8 1x)\n",
				     id[0], id[1], id[2]);

	dev_info(cs->dev, "CS42L81 ID %02x%02x%02x rev %#x\n",
		 id[0], id[1], id[2], rev);
	return 0;
}

static int cs42l81_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct cs42l81 *cs;
	u32 max_db_q8 = 0;
	int ret;

	dev_info(dev, "codec probe begin\n");

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = dev;
	cs->spi = spi;
	cs->xfer_buf = devm_kzalloc(dev, 10, GFP_KERNEL);
	if (!cs->xfer_buf)
		return -ENOMEM;
	spi_set_drvdata(spi, cs);

	ret = devm_mutex_init(dev, &cs->status_lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &cs->jack_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&cs->jack_work, cs42l81_jack_work);
	ret = devm_add_action(dev, cs42l81_cancel_jack_work, cs);
	if (ret)
		return ret;
	cs->jack_forced = -1;

	cs->regmap = devm_regmap_init(dev, &cs42l81_bus, cs, &cs42l81_regmap);
	if (IS_ERR(cs->regmap))
		return dev_err_probe(dev, PTR_ERR(cs->regmap), "regmap init failed\n");

	cs->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(cs->mclk))
		return dev_err_probe(dev, PTR_ERR(cs->mclk), "failed to get MCLK\n");

	/* logical 1 = muted; the line itself is active low (inferred) */
	cs->mute_gpio = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->mute_gpio))
		return dev_err_probe(dev, PTR_ERR(cs->mute_gpio),
				     "failed to get mute GPIO\n");

	/*
	 * Headset switch lines: their direction is not known (iOS may drive
	 * them), so take them as they are and only read them for the jack log.
	 * GPIOD_ASIS does not touch the pad.
	 */
	cs->hs_switch = devm_gpiod_get_array_optional(dev, "hs-switch",
						      GPIOD_ASIS);
	if (IS_ERR(cs->hs_switch)) {
		if (PTR_ERR(cs->hs_switch) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "hs-switch GPIOs unavailable: %pe\n", cs->hs_switch);
		cs->hs_switch = NULL;
	}

	cs->spk_mode = 2;
	of_property_read_u32(dev->of_node, "apple,speaker-mode", &cs->spk_mode);

	/*
	 * apple,max-speaker-gain-db-q8: max speaker gain in 1/256 dB (iOS ADT
	 * spkr-gain-db is -3.0 dB). Clamp the control there.
	 */
	cs->vol_max = CS42L81_VOL_MAX_CTL;
	if (!of_property_read_u32(dev->of_node, "apple,max-speaker-gain-db-q8",
				  &max_db_q8)) {
		int db_x2 = (int)max_db_q8 * 2 / 256;	/* 0.5 dB units */
		int max = 204 + db_x2;			/* 204 = 0 dB */

		cs->vol_max = clamp(max, 0, CS42L81_VOL_MAX_CTL);
	}
	/* Default: -24 dB, quiet but audible */
	cs->spk_vol = min_t(unsigned int, 204 - 48, cs->vol_max);
	cs->spk_switch = true;
	/* the DAI only accepts codec-provided clocks (J71t, as on iOS) */
	cs->clk_provider = true;
	cs->rate_code = 0xc;
	cs->slot_bytes = 2;
	cs->hp_vol = CS42L81_HP_VOL_DEFAULT;
	cs->hp_switch = 1;
	cs->adc_vol = CS42L81_ADC_VOL_0DB;
	cs->adc_switch = 1;

	ret = cs42l81_check_id(cs);
	if (ret)
		return ret;

	debugfs_create_devm_seqfile(dev, "cs42l81-regs", NULL, cs42l81_regs_show);
	ret = devm_add_action_or_reset(dev, cs42l81_debugfs_remove,
				       debugfs_create_file("cs42l81-reg", 0600,
							   NULL, cs,
							   &cs42l81_reg_fops));
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, cs42l81_debugfs_remove,
				       debugfs_create_file("cs42l81-jack", 0600,
							   NULL, cs,
							   &cs42l81_jack_fops));
	if (ret)
		return ret;

	return devm_snd_soc_register_component(dev, &cs42l81_component,
					       &cs42l81_dai, 1);
}

static const struct of_device_id cs42l81_of_match[] = {
	{ .compatible = "cirrus,cs42l81" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l81_of_match);

static const struct spi_device_id cs42l81_spi_ids[] = {
	{ "cs42l81" },
	{ }
};
MODULE_DEVICE_TABLE(spi, cs42l81_spi_ids);

static struct spi_driver cs42l81_spi_driver = {
	.driver = {
		.name = "cs42l81",
		.of_match_table = cs42l81_of_match,
	},
	.probe = cs42l81_spi_probe,
	.id_table = cs42l81_spi_ids,
};
module_spi_driver(cs42l81_spi_driver);

MODULE_DESCRIPTION("Cirrus Logic CS42L81 codec (Apple iPad 5) driver");
MODULE_LICENSE("Dual MIT/GPL");
