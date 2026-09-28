// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6572 AFE platform driver.
 *
 * DL1 playback front-end feeding the ADDA downlink SRC and the AFE<->PMIC
 * serial link to the mt6323 codec, plus the CONSYS FM receiver's I2S input,
 * routed to the same downlink and captured through the AWB memif. The AFE
 * registers are in the parent audsys syscon; a fast_io regmap keeps the
 * triggers and the period IRQs atomic.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* AFE registers (classic mt65xx layout); stock magic values noted inline. */
#define AUDIO_TOP_CON0		0x0000
#define AUDIO_TOP_CON0_AFE_ON	0x60004000
#define AFE_DAC_CON0		0x0010
#define AFE_DAC_CON0_AFE_ON	BIT(0)
#define AFE_DAC_CON0_DL1_ON	BIT(1)
#define AFE_DAC_CON0_AWB_ON	BIT(6)
#define AFE_DAC_CON0_DL1_OUT	BIT(10)
#define AFE_DAC_CON1		0x0014
#define AFE_DAC_CON1_DL1_RATE	GENMASK(3, 0)
#define AFE_DAC_CON1_I2S_RATE	GENMASK(11, 8)	/* I2S-in ASRC output rate */
#define AFE_DAC_CON1_AWB_RATE	GENMASK(15, 12)
#define AFE_DAC_CON1_AWB_MONO	BIT(24)
#define AFE_DL1_BASE		0x0040
#define AFE_DL1_CUR		0x0044
#define AFE_DL1_END		0x0048		/* ring end, inclusive */
#define AFE_AWB_BASE		0x0070
#define AFE_AWB_END		0x0078		/* ring end, inclusive */
#define AFE_AWB_CUR		0x007c
#define AFE_MEMIF_MAXLEN	0x03d4
#define AFE_MEMIF_MAXLEN_DL1	GENMASK(3, 0)
#define AFE_MEMIF_PBUF_SIZE	0x03d8
#define AFE_MEMIF_PBUF_SIZE_DL1	GENMASK(17, 16)
#define AFE_IRQ_MCU_CON		0x03a0
#define AFE_IRQ_MCU_CON_IRQ1_ON		BIT(0)
#define AFE_IRQ_MCU_CON_IRQ2_ON		BIT(1)
#define AFE_IRQ_MCU_CON_IRQ1_RATE	GENMASK(7, 4)
#define AFE_IRQ_MCU_CON_IRQ2_RATE	GENMASK(11, 8)
#define AFE_IRQ_MCU_STATUS	0x03a4
#define AFE_IRQ_MCU_STATUS_IRQ1	BIT(0)
#define AFE_IRQ_MCU_STATUS_IRQ2	BIT(1)
#define AFE_IRQ_MCU_STATUS_MASK	GENMASK(3, 0)
#define AFE_IRQ_MCU_CLR		0x03a8
#define AFE_IRQ_MCU_CLR_NOSTATUS (BIT(6) | BIT(1) | BIT(0))	/* ack when STATUS=0 */
#define AFE_IRQ_MCU_CNT1	0x03ac
#define AFE_IRQ_MCU_CNT2	0x03b0

/* DL1 -> interconnect -> ADDA downlink SRC -> AFE<->PMIC link. */
#define AFE_I2S_CON1		0x0034
#define AFE_I2S_CON1_BASE	0x00000008	/* I2S DAC format */
#define AFE_I2S_CON1_RATE	GENMASK(11, 8)
#define AFE_I2S_CON1_ON		BIT(0)
#define AFE_CONN1		0x0024
#define AFE_CONN1_DL1_O3	BIT(21)		/* DL1 ch1 -> O3 */
#define AFE_CONN2		0x0028
#define AFE_CONN2_DL1_O4	BIT(6)		/* DL1 ch2 -> O4 */
#define AFE_CONN2_I2S_IN_O05	BIT(16)		/* I2S-in L (I00) -> AWB L (O05) */
#define AFE_CONN2_I2S_IN_O06	BIT(22)		/* I2S-in R (I01) -> AWB R (O06) */
#define AFE_ADDA_DL_SRC2_CON0	0x0108
#define AFE_ADDA_DL_SRC2_CON0_BASE 0x03001802	/* SRC-disabled base */
#define AFE_ADDA_DL_SRC2_CON0_RATE GENMASK(31, 28)
#define AFE_ADDA_DL_SRC2_CON0_ON   BIT(0)
#define AFE_ADDA_DL_SRC2_CON1	0x010c
#define AFE_ADDA_DL_SRC2_CON1_GAIN GENMASK(31, 16)
#define AFE_DL_GAIN_DEFAULT	0x203b		/* ~-18dB */
#define AFE_ADDA_UL_DL_CON0	0x0124
#define AFE_ADDA_UL_DL_CON0_ON	BIT(0)
#define AFE_ADDA_PREDIS_CON0	0x0260		/* ADDA downlink pre-distortion */
#define AFE_ADDA_PREDIS_CON1	0x0264
#define AFE_ADDA_NEWIF_CFG0	0x0138		/* AFE<->PMIC serial link (NEWIF) */
#define AFE_ADDA_NEWIF_CFG0_VAL	0x03f87200
#define AFE_ADDA_NEWIF_CFG1	0x013c
#define AFE_ADDA_NEWIF_CFG1_VAL	0x03117180

/*
 * FM: CONSYS drives the AFE I2S input as master at 32 kHz. The I2S-in ASRC
 * resamples it to FM_RATE; HW gain 2 (I00/I01 -> O15/O16, I12/I13 -> O3/O4)
 * then carries it onto the DL1 outputs with its own volume. Values follow
 * the stock audio HAL's FM path.
 */
#define AFE_I2S_CON		0x0018
#define AFE_I2S_CON_PHASE_FIX	BIT(31)
#define AFE_I2S_CON_FMT_I2S	BIT(3)
#define AFE_I2S_CON_SLAVE	BIT(2)
#define AFE_I2S_CON_EN		BIT(0)
#define AFE_CONN4		0x0030
#define AFE_CONN4_BYPASS_ASRC	BIT(30)
#define AFE_GAIN2_CON0		0x0428
#define AFE_GAIN2_CON0_RATE	GENMASK(7, 4)
#define AFE_GAIN2_CON0_ON	BIT(0)
#define AFE_GAIN2_CON1		0x042c		/* target gain, 0x80000 = 0 dB */
#define AFE_GAIN2_CON1_GAIN	GENMASK(19, 0)
#define AFE_GAIN2_CONN		0x0438
#define AFE_GAIN2_CONN_I12_O03	BIT(8)
#define AFE_GAIN2_CONN_I13_O04	BIT(10)
#define AFE_GAIN2_CONN_I00_O15	BIT(16)
#define AFE_GAIN2_CONN_I01_O16	BIT(23)
#define AFE_GAIN2_CONN_FM	(AFE_GAIN2_CONN_I00_O15 | AFE_GAIN2_CONN_I01_O16 | \
				 AFE_GAIN2_CONN_I12_O03 | AFE_GAIN2_CONN_I13_O04)
#define AFE_GAIN2_CUR		0x043c		/* ramp start */
#define AFE_ASRC_CON0		0x0500
#define AFE_ASRC_CON0_I2S_STR	BIT(6)		/* I2S channel set start/clear */
#define AFE_ASRC_CON0_ASM_ON	BIT(0)
#define AFE_ASRC_CON13		0x0550
#define AFE_ASRC_CON13_I2S_MONO	BIT(16)
#define AFE_ASRC_CON14		0x0554		/* I2S Rx output-rate palette */
#define AFE_ASRC_CON15		0x0558		/* I2S Rx input-rate palette */
#define AFE_ASRC_CON16		0x055c		/* frequency calibrator 2 */
#define AFE_ASRC_CON16_CYCLE	GENMASK(31, 16)
#define AFE_ASRC_CON16_AUTORST	BIT(14)
/* datasheet: AUTO_TUNE_FREQ4; in practice it retunes CON15, the input palette */
#define AFE_ASRC_CON16_TUNE_IFS	BIT(12)
#define AFE_ASRC_CON16_COMP	BIT(11)
#define AFE_ASRC_CON16_SEL	GENMASK(9, 8)
#define AFE_ASRC_CON16_BP_DGL	BIT(7)
#define AFE_ASRC_CON16_RESTART	BIT(2)
#define AFE_ASRC_CON16_FREQ_OUT	BIT(1)
#define AFE_ASRC_CON16_EN	BIT(0)
#define AFE_ASRC_CON17		0x0560		/* calibrator denominator */
#define AFE_ASRC_CON20		0x056c		/* calibrator auto-reset bound */

/* 32 kHz in -> 44.1 kHz out: palettes and denominator from the datasheet. */
#define FM_RATE			44100
#define FM_ASRC_OFS_44K		0xdc8000
#define FM_ASRC_IFS_32K		0xa00000
#define FM_ASRC_DENOM_44K	0x1fbd
#define FM_ASRC_AUTORST_HI	0x1b00
/* Calibrator 2 tracking the I2S-in rate (stock value 0x75987 without EN). */
#define FM_ASRC_CALI		(FIELD_PREP(AFE_ASRC_CON16_CYCLE, 7) | \
				 AFE_ASRC_CON16_AUTORST | AFE_ASRC_CON16_TUNE_IFS | \
				 AFE_ASRC_CON16_COMP | FIELD_PREP(AFE_ASRC_CON16_SEL, 1) | \
				 AFE_ASRC_CON16_BP_DGL | AFE_ASRC_CON16_RESTART | \
				 AFE_ASRC_CON16_FREQ_OUT)
#define FM_GAIN_MAX		0x80000		/* 0 dB */
#define FM_GAIN_DEFAULT		0x10000		/* -18 dB, on top of "Playback Volume" */

static const struct regmap_config mt6572_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,
	.max_register = 0x0ffc,
};

struct mt6572_afe {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk;
	struct snd_pcm_substream *dl1_substream;	/* active DL1 stream */
	struct snd_pcm_substream *awb_substream;	/* active AWB stream */
	unsigned int dl_gain;				/* "Playback Volume" */
	unsigned int fm_gain;				/* "FM Playback Volume" */
	bool fm_on;					/* FM path owns the DL rate */
};

/* Hz -> AFE sample-rate code. */
static int mt6572_afe_rate_code(unsigned int rate)
{
	switch (rate) {
	case 8000:	return 0;
	case 11025:	return 1;
	case 12000:	return 2;
	case 16000:	return 4;
	case 22050:	return 5;
	case 24000:	return 6;
	case 32000:	return 8;
	case 44100:	return 9;
	case 48000:	return 10;
	default:	return -EINVAL;
	}
}

/* Hz -> ADDA downlink SRC input-mode code. */
static int mt6572_afe_adda_rate_code(unsigned int rate)
{
	switch (rate) {
	case 8000:	return 0;
	case 11025:	return 1;
	case 12000:	return 2;
	case 16000:	return 3;
	case 22050:	return 4;
	case 24000:	return 5;
	case 32000:	return 6;
	case 44100:	return 7;
	case 48000:	return 8;
	default:	return -EINVAL;
	}
}

enum {
	MT6572_AFE_DAI_DL1,
	MT6572_AFE_DAI_AWB,
};

static struct snd_soc_dai_driver mt6572_afe_dais[] = {
	{
		.name = "mt6572-afe-dl1",
		.id = MT6572_AFE_DAI_DL1,
		.playback = {
			.stream_name = "DL1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		/* the FM stream after the I2S-in ASRC, so FM_RATE only */
		.name = "mt6572-afe-awb",
		.id = MT6572_AFE_DAI_AWB,
		.capture = {
			.stream_name = "AWB Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

static const struct snd_pcm_hardware mt6572_afe_hardware = {
	/* on-chip SRAM buffer, no mmap */
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.period_bytes_min = 1024,
	.period_bytes_max = 8192,
	.periods_min = 2,
	.periods_max = 16,
	.buffer_bytes_max = 16 * 1024,		/* AFE on-chip SRAM */
};

/* AWB capture: a DRAM ring, leaving the on-chip SRAM to DL1 */
static const struct snd_pcm_hardware mt6572_afe_awb_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_44100,
	.rate_min = FM_RATE,
	.rate_max = FM_RATE,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = 1024,
	.period_bytes_max = 32 * 1024,
	.periods_min = 2,
	.periods_max = 32,
	.buffer_bytes_max = 128 * 1024,
};

static bool mt6572_afe_is_awb(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_CAPTURE;
}

static int mt6572_afe_pcm_open(struct snd_soc_component *comp,
			       struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, mt6572_afe_is_awb(substream) ?
				     &mt6572_afe_awb_hardware : &mt6572_afe_hardware);
	/* memif END[2:0] must be 7: keep the period (so the buffer) 8-byte aligned. */
	return snd_pcm_hw_constraint_step(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 8);
}

static int mt6572_afe_pcm_hw_params(struct snd_soc_component *comp,
				    struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int bytes = params_buffer_bytes(params);
	u32 base = lower_32_bits(runtime->dma_addr);

	if (mt6572_afe_is_awb(substream)) {
		regmap_write(afe->regmap, AFE_AWB_BASE, base);
		regmap_write(afe->regmap, AFE_AWB_END, base + bytes - 1);
		return 0;
	}

	/* program the DL1 memif DMA ring (in the AFE on-chip SRAM) */
	regmap_write(afe->regmap, AFE_DL1_BASE, base);
	regmap_write(afe->regmap, AFE_DL1_END, base + bytes - 1);
	regmap_clear_bits(afe->regmap, AFE_MEMIF_MAXLEN, AFE_MEMIF_MAXLEN_DL1);
	regmap_clear_bits(afe->regmap, AFE_MEMIF_PBUF_SIZE, AFE_MEMIF_PBUF_SIZE_DL1);
	return 0;
}

/* ADDA downlink SRC + I2S out to the PMIC link, then the global AFE enable. */
static void mt6572_afe_dl_start(struct mt6572_afe *afe, int rate_code,
				int adda_code)
{
	u32 src = AFE_ADDA_DL_SRC2_CON0_BASE |
		  FIELD_PREP(AFE_ADDA_DL_SRC2_CON0_RATE, adda_code) |
		  AFE_ADDA_DL_SRC2_CON0_ON;

	regmap_write(afe->regmap, AFE_ADDA_PREDIS_CON0, 0);
	regmap_write(afe->regmap, AFE_ADDA_PREDIS_CON1, 0);

	/* stock's interleaving */
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0, src);
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON1,
		     FIELD_PREP(AFE_ADDA_DL_SRC2_CON1_GAIN, afe->dl_gain));
	regmap_write(afe->regmap, AFE_I2S_CON1,
		     AFE_I2S_CON1_BASE | FIELD_PREP(AFE_I2S_CON1_RATE, rate_code));
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0, src);
	regmap_set_bits(afe->regmap, AFE_I2S_CON1, AFE_I2S_CON1_ON);
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0, src);
	regmap_set_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, AFE_ADDA_UL_DL_CON0_ON);

	regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON);
}

/* AWB: IRQ2 paces its periods (DL1 keeps IRQ1); fed from the I2S input. */
static int mt6572_afe_awb_prepare(struct mt6572_afe *afe,
				  struct snd_pcm_runtime *runtime)
{
	int rate_code = mt6572_afe_rate_code(runtime->rate);

	if (rate_code < 0)
		return -EINVAL;

	regmap_update_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ2_RATE,
			   FIELD_PREP(AFE_IRQ_MCU_CON_IRQ2_RATE, rate_code));
	regmap_write(afe->regmap, AFE_IRQ_MCU_CNT2, runtime->period_size);
	regmap_update_bits(afe->regmap, AFE_DAC_CON1,
			   AFE_DAC_CON1_AWB_RATE | AFE_DAC_CON1_AWB_MONO,
			   FIELD_PREP(AFE_DAC_CON1_AWB_RATE, rate_code));
	regmap_set_bits(afe->regmap, AFE_CONN2,
			AFE_CONN2_I2S_IN_O05 | AFE_CONN2_I2S_IN_O06);
	regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON);
	return 0;
}

static int mt6572_afe_pcm_prepare(struct snd_soc_component *comp,
				  struct snd_pcm_substream *substream)
{
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int adda_code = mt6572_afe_adda_rate_code(runtime->rate);
	int rate_code = mt6572_afe_rate_code(runtime->rate);

	if (mt6572_afe_is_awb(substream))
		return mt6572_afe_awb_prepare(afe, runtime);

	if (adda_code < 0 || rate_code < 0)
		return -EINVAL;
	/* one downlink rate: FM holds it while its path is up */
	if (afe->fm_on && runtime->rate != FM_RATE)
		return -EBUSY;

	/* IRQ1 rate + per-period frame count (enabled in the trigger) */
	regmap_update_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ1_RATE,
			   FIELD_PREP(AFE_IRQ_MCU_CON_IRQ1_RATE, rate_code));
	regmap_write(afe->regmap, AFE_IRQ_MCU_CNT1, runtime->period_size);

	/* interconnect: DL1 ch1/ch2 -> O3/O4 */
	regmap_set_bits(afe->regmap, AFE_CONN1, AFE_CONN1_DL1_O3);
	regmap_set_bits(afe->regmap, AFE_CONN2, AFE_CONN2_DL1_O4);

	regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_DL1_OUT);
	mt6572_afe_dl_start(afe, rate_code, adda_code);

	/* the DL1 memif rate */
	regmap_update_bits(afe->regmap, AFE_DAC_CON1, AFE_DAC_CON1_DL1_RATE,
			   FIELD_PREP(AFE_DAC_CON1_DL1_RATE, rate_code));

	return 0;
}

static int mt6572_afe_pcm_hw_free(struct snd_soc_component *comp,
				  struct snd_pcm_substream *substream)
{
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);

	if (mt6572_afe_is_awb(substream))
		regmap_clear_bits(afe->regmap, AFE_CONN2,
				  AFE_CONN2_I2S_IN_O05 | AFE_CONN2_I2S_IN_O06);
	return 0;
}

static int mt6572_afe_awb_trigger(struct mt6572_afe *afe,
				  struct snd_pcm_substream *substream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		afe->awb_substream = substream;
		regmap_set_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ2_ON);
		regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AWB_ON);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		regmap_clear_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AWB_ON);
		regmap_clear_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ2_ON);
		afe->awb_substream = NULL;
		return 0;
	default:
		return -EINVAL;
	}
}

static int mt6572_afe_pcm_trigger(struct snd_soc_component *comp,
				  struct snd_pcm_substream *substream, int cmd)
{
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);

	if (mt6572_afe_is_awb(substream))
		return mt6572_afe_awb_trigger(afe, substream, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		afe->dl1_substream = substream;
		/* atomic: memif start + period IRQ only (HW-IRQ driven) */
		regmap_set_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ1_ON);
		regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_DL1_ON);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* minimal stop: DL1 memif + period IRQ only; .prepare re-asserts the rest */
		regmap_clear_bits(afe->regmap, AFE_IRQ_MCU_CON, AFE_IRQ_MCU_CON_IRQ1_ON);
		regmap_clear_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_DL1_ON);
		afe->dl1_substream = NULL;
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t mt6572_afe_pcm_pointer(struct snd_soc_component *comp,
						struct snd_pcm_substream *substream)
{
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 base = lower_32_bits(runtime->dma_addr);
	unsigned int cur = 0;

	regmap_read(afe->regmap, mt6572_afe_is_awb(substream) ? AFE_AWB_CUR : AFE_DL1_CUR,
		    &cur);
	if (cur < base || cur >= base + runtime->dma_bytes)
		return 0;
	return bytes_to_frames(runtime, cur - base);
}

static int mt6572_afe_pcm_new(struct snd_soc_component *comp,
				    struct snd_soc_pcm_runtime *rtd)
{
	size_t size = mt6572_afe_hardware.buffer_bytes_max;

	if (snd_soc_rtd_to_cpu(rtd, 0)->id == MT6572_AFE_DAI_AWB) {
		snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV, comp->dev, 0,
					       mt6572_afe_awb_hardware.buffer_bytes_max);
		return 0;
	}

	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV_IRAM, comp->dev,
				       size, size);
	return 0;
}

static const DECLARE_TLV_DB_LINEAR(dl_gain_tlv, TLV_DB_GAIN_MUTE, 0);

static int mt6572_dl_gain_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = afe->dl_gain;
	return 0;
}

static int mt6572_dl_gain_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	unsigned int gain = ucontrol->value.integer.value[0];

	if (gain > 0xffff)
		return -EINVAL;
	if (gain == afe->dl_gain)
		return 0;

	afe->dl_gain = gain;
	regmap_update_bits(afe->regmap, AFE_ADDA_DL_SRC2_CON1,
			   AFE_ADDA_DL_SRC2_CON1_GAIN,
			   FIELD_PREP(AFE_ADDA_DL_SRC2_CON1_GAIN, gain));
	return 1;
}

/*
 * FM receive side, shared by playback and capture: the CONSYS FM receiver on
 * the I2S input, resampled to FM_RATE. The receiver itself is driven over
 * /dev/fm.
 */
static int mt6572_afe_fm_rx_event(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* I2S in from CONSYS (pad select 0): slave, I2S format, 16-bit */
		regmap_write(afe->regmap, AFE_I2S_CON, AFE_I2S_CON_PHASE_FIX |
			     AFE_I2S_CON_FMT_I2S | AFE_I2S_CON_SLAVE);

		/* ASRC: 32 kHz (tracked by calibrator 2) -> FM_RATE, stereo */
		regmap_clear_bits(afe->regmap, AFE_CONN4, AFE_CONN4_BYPASS_ASRC);
		regmap_update_bits(afe->regmap, AFE_DAC_CON1, AFE_DAC_CON1_I2S_RATE,
				   FIELD_PREP(AFE_DAC_CON1_I2S_RATE,
					      mt6572_afe_rate_code(FM_RATE)));
		regmap_clear_bits(afe->regmap, AFE_ASRC_CON13, AFE_ASRC_CON13_I2S_MONO);
		regmap_write(afe->regmap, AFE_ASRC_CON14, FM_ASRC_OFS_44K);
		regmap_write(afe->regmap, AFE_ASRC_CON15, FM_ASRC_IFS_32K);
		regmap_write(afe->regmap, AFE_ASRC_CON17, FM_ASRC_DENOM_44K);
		regmap_write(afe->regmap, AFE_ASRC_CON16, FM_ASRC_CALI | AFE_ASRC_CON16_EN);
		regmap_write(afe->regmap, AFE_ASRC_CON20, FM_ASRC_AUTORST_HI);
		regmap_set_bits(afe->regmap, AFE_ASRC_CON0,
				AFE_ASRC_CON0_I2S_STR | AFE_ASRC_CON0_ASM_ON);

		regmap_set_bits(afe->regmap, AFE_I2S_CON, AFE_I2S_CON_EN);
		return 0;
	case SND_SOC_DAPM_POST_PMD:
		regmap_clear_bits(afe->regmap, AFE_I2S_CON, AFE_I2S_CON_EN);
		regmap_clear_bits(afe->regmap, AFE_ASRC_CON0,
				  AFE_ASRC_CON0_ASM_ON | AFE_ASRC_CON0_I2S_STR);
		regmap_clear_bits(afe->regmap, AFE_ASRC_CON16, AFE_ASRC_CON16_EN);
		regmap_set_bits(afe->regmap, AFE_CONN4, AFE_CONN4_BYPASS_ASRC);
		return 0;
	default:
		return 0;
	}
}

/*
 * FM playback: the FM stream mixed onto the DL1 outputs through HW gain 2, so
 * it reaches the codec through the DL1 DAI without a PCM stream.
 */
static int mt6572_afe_fm_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	struct snd_pcm_substream *dl1 = afe->dl1_substream;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (dl1 && dl1->runtime->rate != FM_RATE) {
			dev_err(afe->dev, "FM needs the downlink at %u Hz, DL1 runs at %u Hz\n",
				FM_RATE, dl1->runtime->rate);
			return -EBUSY;
		}
		afe->fm_on = true;
		mt6572_afe_dl_start(afe, mt6572_afe_rate_code(FM_RATE),
				    mt6572_afe_adda_rate_code(FM_RATE));

		/* HW gain 2 at FM_RATE, ramping up from silence */
		regmap_set_bits(afe->regmap, AFE_GAIN2_CONN, AFE_GAIN2_CONN_FM);
		regmap_update_bits(afe->regmap, AFE_GAIN2_CON0, AFE_GAIN2_CON0_RATE,
				   FIELD_PREP(AFE_GAIN2_CON0_RATE,
					      mt6572_afe_rate_code(FM_RATE)));
		regmap_write(afe->regmap, AFE_GAIN2_CON1,
			     FIELD_PREP(AFE_GAIN2_CON1_GAIN, afe->fm_gain));
		regmap_write(afe->regmap, AFE_GAIN2_CUR, 0);
		regmap_set_bits(afe->regmap, AFE_GAIN2_CON0, AFE_GAIN2_CON0_ON);
		return 0;
	case SND_SOC_DAPM_POST_PMD:
		regmap_clear_bits(afe->regmap, AFE_GAIN2_CON0, AFE_GAIN2_CON0_ON);
		regmap_clear_bits(afe->regmap, AFE_GAIN2_CONN, AFE_GAIN2_CONN_FM);
		afe->fm_on = false;
		return 0;
	default:
		return 0;
	}
}

static const struct snd_kcontrol_new mt6572_afe_fm_switch =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_soc_dapm_widget mt6572_afe_widgets[] = {
	SND_SOC_DAPM_INPUT("FM I2S In"),
	SND_SOC_DAPM_SUPPLY("FM I2S Rx", SND_SOC_NOPM, 0, 0, mt6572_afe_fm_rx_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH_E("FM Playback", SND_SOC_NOPM, 0, 0,
			      &mt6572_afe_fm_switch, mt6572_afe_fm_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route mt6572_afe_routes[] = {
	{ "FM Playback", "Switch", "FM I2S In" },
	{ "FM Playback", NULL, "FM I2S Rx" },
	{ "DL1 Playback", NULL, "FM Playback" },
	/* the AWB memif records the FM stream, listened to or not */
	{ "AWB Capture", NULL, "FM I2S In" },
	{ "AWB Capture", NULL, "FM I2S Rx" },
};

static const DECLARE_TLV_DB_LINEAR(fm_gain_tlv, TLV_DB_GAIN_MUTE, 0);

static int mt6572_fm_gain_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = afe->fm_gain;
	return 0;
}

static int mt6572_fm_gain_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct mt6572_afe *afe = snd_soc_component_get_drvdata(comp);
	unsigned int gain = ucontrol->value.integer.value[0];

	if (gain > FM_GAIN_MAX)
		return -EINVAL;
	if (gain == afe->fm_gain)
		return 0;

	afe->fm_gain = gain;
	/* the gain block ramps to the new target on its own */
	regmap_write(afe->regmap, AFE_GAIN2_CON1,
		     FIELD_PREP(AFE_GAIN2_CON1_GAIN, gain));
	return 1;
}

/*
 * DL digital gain (DL1 and FM), shadowed in afe->dl_gain so .prepare re-applies
 * it; FM's own HW-gain target in front of it.
 */
static const struct snd_kcontrol_new mt6572_afe_controls[] = {
	SOC_SINGLE_EXT_TLV("Playback Volume", SND_SOC_NOPM, 0, 0xffff, 0,
			   mt6572_dl_gain_get, mt6572_dl_gain_put, dl_gain_tlv),
	SOC_SINGLE_EXT_TLV("FM Playback Volume", SND_SOC_NOPM, 0, FM_GAIN_MAX, 0,
			   mt6572_fm_gain_get, mt6572_fm_gain_put, fm_gain_tlv),
};

static const struct snd_soc_component_driver mt6572_afe_component = {
	.name = "mt6572-afe-pcm",
	.controls = mt6572_afe_controls,
	.num_controls = ARRAY_SIZE(mt6572_afe_controls),
	.dapm_widgets = mt6572_afe_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6572_afe_widgets),
	.dapm_routes = mt6572_afe_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6572_afe_routes),
	.open = mt6572_afe_pcm_open,
	.hw_params = mt6572_afe_pcm_hw_params,
	.prepare = mt6572_afe_pcm_prepare,
	.hw_free = mt6572_afe_pcm_hw_free,
	.trigger = mt6572_afe_pcm_trigger,
	.pointer = mt6572_afe_pcm_pointer,
	.pcm_new = mt6572_afe_pcm_new,
};

/* IRQ1 marks a DL1 period, IRQ2 an AWB one; hardirq (fast_io regmap, atomic PCM). Active-low. */
static irqreturn_t mt6572_afe_irq(int irq, void *dev_id)
{
	struct mt6572_afe *afe = dev_id;
	unsigned int status;

	regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &status);
	status &= AFE_IRQ_MCU_STATUS_MASK;
	if (!status) {
		regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, AFE_IRQ_MCU_CLR_NOSTATUS);
		return IRQ_HANDLED;
	}

	if ((status & AFE_IRQ_MCU_STATUS_IRQ1) && afe->dl1_substream)
		snd_pcm_period_elapsed(afe->dl1_substream);
	if ((status & AFE_IRQ_MCU_STATUS_IRQ2) && afe->awb_substream)
		snd_pcm_period_elapsed(afe->awb_substream);

	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, status);
	return IRQ_HANDLED;
}

static int mt6572_afe_pcm_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6572_afe *afe;
	struct resource res;
	void __iomem *base;
	int ret, irq;

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;
	afe->dev = dev;
	afe->dl_gain = AFE_DL_GAIN_DEFAULT;
	afe->fm_gain = FM_GAIN_DEFAULT;
	platform_set_drvdata(pdev, afe);

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	afe->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(afe->clk))
		return dev_err_probe(dev, PTR_ERR(afe->clk),
				     "failed to get/enable the audio clock\n");

	/* The AFE registers are in the parent audsys syscon window. */
	ret = of_address_to_resource(dev->parent->of_node, 0, &res);
	if (ret)
		return dev_err_probe(dev, ret, "no AFE reg in parent syscon\n");
	base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!base)
		return dev_err_probe(dev, -ENOMEM, "failed to map AFE registers\n");
	afe->regmap = devm_regmap_init_mmio(dev, base, &mt6572_afe_regmap_config);
	if (IS_ERR(afe->regmap))
		return dev_err_probe(dev, PTR_ERR(afe->regmap),
				     "failed to init AFE regmap\n");

	/* power on the AFE top + the SoC side of the AFE<->PMIC link */
	regmap_write(afe->regmap, AUDIO_TOP_CON0, AUDIO_TOP_CON0_AFE_ON);
	regmap_write(afe->regmap, AFE_ADDA_NEWIF_CFG0, AFE_ADDA_NEWIF_CFG0_VAL);
	regmap_write(afe->regmap, AFE_ADDA_NEWIF_CFG1, AFE_ADDA_NEWIF_CFG1_VAL);

	/* mask all AFE IRQs + clear stale status before hooking the GIC */
	regmap_write(afe->regmap, AFE_IRQ_MCU_CON, 0);
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, AFE_IRQ_MCU_STATUS_MASK);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, mt6572_afe_irq, 0, "mt6572-afe", afe);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request AFE irq %d\n", irq);

	ret = devm_snd_soc_register_component(dev, &mt6572_afe_component,
					      mt6572_afe_dais,
					      ARRAY_SIZE(mt6572_afe_dais));
	if (ret)
		return dev_err_probe(dev, ret, "failed to register AFE component\n");

	return 0;
}

static const struct of_device_id mt6572_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6572-audio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6572_afe_pcm_dt_match);

static struct platform_driver mt6572_afe_pcm_driver = {
	.driver = {
		.name = "mt6572-afe-pcm",
		.of_match_table = mt6572_afe_pcm_dt_match,
	},
	.probe = mt6572_afe_pcm_dev_probe,
};
module_platform_driver(mt6572_afe_pcm_driver);

MODULE_DESCRIPTION("MediaTek MT6572 AFE platform driver");
MODULE_LICENSE("GPL");
