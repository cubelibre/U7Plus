/* wisky/sound/wisky_codec_wm8960.c
 *
 * Copyright (C) 2011 wisky
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * LOG
 * ------------------------------------------------------------
 * V001:20110403 cd huang
 *	1.Modify for wisky MID project base on wm8960.c in rk29xx paltform
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/iomux.h>
#include <mach/gpio.h>

#include "wisky_codec_wm8960.h"
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#define SPK_CON 		SPEAKER_CTRL_PIN

#if 0
int audio_trigger_flag;//0--audio off, 1---audio on
#if (defined(CONFIG_ANX7150) || defined(WISKY_HDMI_ANX7150)) && (WISKY_ENABLE_HDMI == 1)
#endif
extern int audio_volume;
#endif
#if WISKY_ENABLE_HDMI
extern int hdmi_status;// 0---lcd. 1--hdmi enable
#endif

static struct snd_soc_codec *wm8960_codec = NULL;
/* R25 - Power 1 */
#define WM8960_VREF      0x40

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL   0x80
#define WM8960_BUFDCOPEN 0x10
#define WM8960_BUFIOEN   0x08
#define WM8960_SOFT_ST   0x04
#define WM8960_HPSTBY    0x01

/* R29 - Anti-pop 2 */
#define WM8960_DISOP     0x40

//wisky-lxh@20110620,store speaker volume
static u16 spkl_reg = 0;
static u16 spkr_reg = 0;
/*
 * wm8960 register cache
 * We can't read the wm8960 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8960_reg[] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};

/* codec private data */
struct wm8960_priv {
	unsigned int sysclk;
	enum snd_soc_control_type control_type;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	u16 reg_cache[WM8960_NUM_REG];
	int is_startup;		// gModify.Add
	int is_biason;
};


#define wm8960_reset(c)	snd_soc_write(c, WM8960_RESET, 0)

/*
 * wm8960 Controls
 */

/* enumerated controls */
static const char *wm8960_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *wm8960_polarity[] = {"No Inversion", "Left Inverted",
	"Right Inverted", "Stereo Inversion"};
static const char *wm8960_3d_upper_cutoff[] = {"High", "Low"};
static const char *wm8960_3d_lower_cutoff[] = {"Low", "High"};
static const char *wm8960_alcfunc[] = {"Off", "Right", "Left", "Stereo"};
static const char *wm8960_alcmode[] = {"ALC", "Limiter"};

static const struct soc_enum wm8960_enum[] = {
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 1, 4, wm8960_deemph),
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_DACCTL2, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_3D, 6, 2, wm8960_3d_upper_cutoff),
	SOC_ENUM_SINGLE(WM8960_3D, 5, 2, wm8960_3d_lower_cutoff),
	SOC_ENUM_SINGLE(WM8960_ALC1, 7, 4, wm8960_alcfunc),
	SOC_ENUM_SINGLE(WM8960_ALC3, 8, 2, wm8960_alcmode),
};

static const DECLARE_TLV_DB_SCALE(adc_tlv, -9700, 50, 0);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -2100, 300, 0);
static const DECLARE_TLV_DB_SCALE(out_tlv, -12100, 100, 1);

static const struct snd_kcontrol_new wm8960_snd_controls[] = 
{
SOC_DOUBLE_R_TLV("Capture Volume", WM8960_LINVOL, WM8960_RINVOL,
		 0, 63, 0, adc_tlv),
SOC_DOUBLE_R("Capture Volume ZC Switch", WM8960_LINVOL, WM8960_RINVOL,
	6, 1, 0),
#if 1
SOC_DOUBLE_R("Capture Switch", WM8960_LINVOL, WM8960_RINVOL,
	7, 1, 1),
#endif
SOC_DOUBLE_R_TLV("Playback Volume", WM8960_LDAC, WM8960_RDAC,
		 0, 255, 0, dac_tlv),

SOC_DOUBLE_R_TLV("Headphone Playback Volume", WM8960_LOUT1, WM8960_ROUT1,
		 0, 127, 0, out_tlv),
SOC_DOUBLE_R("Headphone Playback ZC Switch", WM8960_LOUT1, WM8960_ROUT1,
	7, 1, 0),

SOC_DOUBLE_R_TLV("Speaker Playback Volume", WM8960_LOUT2, WM8960_ROUT2,
		 0, 127, 0, out_tlv),
SOC_DOUBLE_R("Speaker Playback ZC Switch", WM8960_LOUT2, WM8960_ROUT2,
	7, 1, 0),
SOC_SINGLE("Speaker DC Volume", WM8960_CLASSD3, 3, 5, 0),
SOC_SINGLE("Speaker AC Volume", WM8960_CLASSD3, 0, 5, 0),

SOC_SINGLE("PCM Playback -6dB Switch", WM8960_DACCTL1, 7, 1, 0),
SOC_ENUM("ADC Polarity", wm8960_enum[1]),
SOC_ENUM("Playback De-emphasis", wm8960_enum[0]),
SOC_SINGLE("ADC High Pass Filter Switch", WM8960_DACCTL1, 0, 1, 0),

SOC_ENUM("DAC Polarity", wm8960_enum[2]),

SOC_ENUM("3D Filter Upper Cut-Off", wm8960_enum[3]),
SOC_ENUM("3D Filter Lower Cut-Off", wm8960_enum[4]),
SOC_SINGLE("3D Volume", WM8960_3D, 1, 15, 0),
SOC_SINGLE("3D Switch", WM8960_3D, 0, 1, 0),

SOC_ENUM("ALC Function", wm8960_enum[5]),
SOC_SINGLE("ALC Max Gain", WM8960_ALC1, 4, 7, 0),
SOC_SINGLE("ALC Target", WM8960_ALC1, 0, 15, 1),
SOC_SINGLE("ALC Min Gain", WM8960_ALC2, 4, 7, 0),
SOC_SINGLE("ALC Hold Time", WM8960_ALC2, 0, 15, 0),
SOC_ENUM("ALC Mode", wm8960_enum[6]),
SOC_SINGLE("ALC Decay", WM8960_ALC3, 4, 15, 0),
SOC_SINGLE("ALC Attack", WM8960_ALC3, 0, 15, 0),

SOC_SINGLE("Noise Gate Threshold", WM8960_NOISEG, 3, 31, 0),
SOC_SINGLE("Noise Gate Switch", WM8960_NOISEG, 0, 1, 0),

SOC_DOUBLE_R("ADC PCM Capture Volume", WM8960_LINPATH, WM8960_RINPATH,
	0, 127, 0),

SOC_SINGLE_TLV("Left Output Mixer Boost Bypass Volume",
	       WM8960_BYPASS1, 4, 7, 1, bypass_tlv),
SOC_SINGLE_TLV("Left Output Mixer LINPUT3 Volume",
	       WM8960_LOUTMIX, 4, 7, 1, bypass_tlv),
SOC_SINGLE_TLV("Right Output Mixer Boost Bypass Volume",
	       WM8960_BYPASS2, 4, 7, 1, bypass_tlv),
SOC_SINGLE_TLV("Right Output Mixer RINPUT3 Volume",
	       WM8960_ROUTMIX, 4, 7, 1, bypass_tlv),
};



/*
 * DAPM Controls
 */

static const struct snd_kcontrol_new wm8960_lin_boost[] = {
SOC_DAPM_SINGLE("LINPUT2 Switch", WM8960_LINPATH, 6, 1, 0),
SOC_DAPM_SINGLE("LINPUT3 Switch", WM8960_LINPATH, 7, 1, 0),
SOC_DAPM_SINGLE("LINPUT1 Switch", WM8960_LINPATH, 8, 1, 0),
};

static const struct snd_kcontrol_new wm8960_lin[] = {
SOC_DAPM_SINGLE("Boost Switch", WM8960_LINPATH, 3, 1, 0),
};

static const struct snd_kcontrol_new wm8960_rin_boost[] = {
SOC_DAPM_SINGLE("RINPUT2 Switch", WM8960_RINPATH, 6, 1, 0),
SOC_DAPM_SINGLE("RINPUT3 Switch", WM8960_RINPATH, 7, 1, 0),
SOC_DAPM_SINGLE("RINPUT1 Switch", WM8960_RINPATH, 8, 1, 0),
};

static const struct snd_kcontrol_new wm8960_rin[] = {
SOC_DAPM_SINGLE("Boost Switch", WM8960_RINPATH, 3, 1, 0),
};

static const struct snd_kcontrol_new wm8960_loutput_mixer[] = {
SOC_DAPM_SINGLE("PCM Playback Switch", WM8960_LOUTMIX, 8, 1, 0),
SOC_DAPM_SINGLE("LINPUT3 Switch", WM8960_LOUTMIX, 7, 1, 0),
SOC_DAPM_SINGLE("Boost Bypass Switch", WM8960_BYPASS1, 7, 1, 0),
};

static const struct snd_kcontrol_new wm8960_routput_mixer[] = {
SOC_DAPM_SINGLE("PCM Playback Switch", WM8960_ROUTMIX, 8, 1, 0),
SOC_DAPM_SINGLE("RINPUT3 Switch", WM8960_ROUTMIX, 7, 1, 0),
SOC_DAPM_SINGLE("Boost Bypass Switch", WM8960_BYPASS2, 7, 1, 0),
};

static const struct snd_kcontrol_new wm8960_mono_out[] = {
SOC_DAPM_SINGLE("Left Switch", WM8960_MONOMIX1, 7, 1, 0),
SOC_DAPM_SINGLE("Right Switch", WM8960_MONOMIX2, 7, 1, 0),
};


static const struct snd_soc_dapm_widget wm8960_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("LINPUT1"),
SND_SOC_DAPM_INPUT("RINPUT1"),
SND_SOC_DAPM_INPUT("LINPUT2"),
SND_SOC_DAPM_INPUT("RINPUT2"),
SND_SOC_DAPM_INPUT("LINPUT3"),
SND_SOC_DAPM_INPUT("RINPUT3"),

SND_SOC_DAPM_MICBIAS("MICB", WM8960_POWER1, 1, 0),

SND_SOC_DAPM_MIXER("Left Boost Mixer", WM8960_POWER1, 5, 0,
		   wm8960_lin_boost, ARRAY_SIZE(wm8960_lin_boost)),
SND_SOC_DAPM_MIXER("Right Boost Mixer", WM8960_POWER1, 4, 0,
		   wm8960_rin_boost, ARRAY_SIZE(wm8960_rin_boost)),

SND_SOC_DAPM_MIXER("Left Input Mixer", WM8960_POWER3, 5, 0,
		   wm8960_lin, ARRAY_SIZE(wm8960_lin)),
SND_SOC_DAPM_MIXER("Right Input Mixer", WM8960_POWER3, 4, 0,
		   wm8960_rin, ARRAY_SIZE(wm8960_rin)),

SND_SOC_DAPM_ADC("Left ADC", "Capture", WM8960_POWER2, 3, 0),
SND_SOC_DAPM_ADC("Right ADC", "Capture", WM8960_POWER2, 2, 0),

SND_SOC_DAPM_DAC("Left DAC", "Playback", WM8960_POWER2, 8, 0),
SND_SOC_DAPM_DAC("Right DAC", "Playback", WM8960_POWER2, 7, 0),

SND_SOC_DAPM_MIXER("Left Output Mixer", WM8960_POWER3, 3, 0,
	&wm8960_loutput_mixer[0],
	ARRAY_SIZE(wm8960_loutput_mixer)),
SND_SOC_DAPM_MIXER("Right Output Mixer", WM8960_POWER3, 2, 0,
	&wm8960_routput_mixer[0],
	ARRAY_SIZE(wm8960_routput_mixer)),

SND_SOC_DAPM_MIXER("Mono Output Mixer", WM8960_POWER2, 1, 0,
	&wm8960_mono_out[0],
	ARRAY_SIZE(wm8960_mono_out)),

SND_SOC_DAPM_PGA("LOUT1 PGA", WM8960_POWER2, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("ROUT1 PGA", WM8960_POWER2, 5, 0, NULL, 0),

SND_SOC_DAPM_PGA("Left Speaker PGA", WM8960_POWER2, 4, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Speaker PGA", WM8960_POWER2, 3, 0, NULL, 0),

SND_SOC_DAPM_PGA("Right Speaker Output", WM8960_CLASSD1, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Speaker Output", WM8960_CLASSD1, 6, 0, NULL, 0),

SND_SOC_DAPM_OUTPUT("SPK_LP"),
SND_SOC_DAPM_OUTPUT("SPK_LN"),
SND_SOC_DAPM_OUTPUT("HP_L"),
SND_SOC_DAPM_OUTPUT("HP_R"),
SND_SOC_DAPM_OUTPUT("SPK_RP"),
SND_SOC_DAPM_OUTPUT("SPK_RN"),
SND_SOC_DAPM_OUTPUT("OUT3"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{ "Left Boost Mixer", "LINPUT1 Switch", "LINPUT1" },
	{ "Left Boost Mixer", "LINPUT2 Switch", "LINPUT2" },
	{ "Left Boost Mixer", "LINPUT3 Switch", "LINPUT3" },

	{ "Left Input Mixer", "Boost Switch", "Left Boost Mixer", },
	{ "Left Input Mixer", NULL, "LINPUT1", },  /* Really Boost Switch */
	{ "Left Input Mixer", NULL, "LINPUT2" },
	{ "Left Input Mixer", NULL, "LINPUT3" },

	{ "Right Boost Mixer", "RINPUT1 Switch", "RINPUT1" },
	{ "Right Boost Mixer", "RINPUT2 Switch", "RINPUT2" },
	{ "Right Boost Mixer", "RINPUT3 Switch", "RINPUT3" },

	{ "Right Input Mixer", "Boost Switch", "Right Boost Mixer", },
	{ "Right Input Mixer", NULL, "RINPUT1", },  /* Really Boost Switch */
	{ "Right Input Mixer", NULL, "RINPUT2" },
	{ "Right Input Mixer", NULL, "LINPUT3" },

	{ "Left ADC", NULL, "Left Input Mixer" },
	{ "Right ADC", NULL, "Right Input Mixer" },

	{ "Left Output Mixer", "LINPUT3 Switch", "LINPUT3" },
	{ "Left Output Mixer", "Boost Bypass Switch", "Left Boost Mixer"} ,
	{ "Left Output Mixer", "PCM Playback Switch", "Left DAC" },

	{ "Right Output Mixer", "RINPUT3 Switch", "RINPUT3" },
	{ "Right Output Mixer", "Boost Bypass Switch", "Right Boost Mixer" } ,
	{ "Right Output Mixer", "PCM Playback Switch", "Right DAC" },

	{ "Mono Output Mixer", "Left Switch", "Left Output Mixer" },
	{ "Mono Output Mixer", "Right Switch", "Right Output Mixer" },

	{ "LOUT1 PGA", NULL, "Left Output Mixer" },
	{ "ROUT1 PGA", NULL, "Right Output Mixer" },

	{ "HP_L", NULL, "LOUT1 PGA" },
	{ "HP_R", NULL, "ROUT1 PGA" },

	{ "Left Speaker PGA", NULL, "Left Output Mixer" },
	{ "Right Speaker PGA", NULL, "Right Output Mixer" },

	{ "Left Speaker Output", NULL, "Left Speaker PGA" },
	{ "Right Speaker Output", NULL, "Right Speaker PGA" },

	{ "SPK_LN", NULL, "Left Speaker Output" },
	{ "SPK_LP", NULL, "Left Speaker Output" },
	{ "SPK_RN", NULL, "Right Speaker Output" },
	{ "SPK_RP", NULL, "Right Speaker Output" },

	{ "OUT3", NULL, "Mono Output Mixer", }
};


struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:5;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x6, 0x0},
	{11289600, 8000, 1408, 0x16, 0x0},
	{18432000, 8000, 2304, 0x7, 0x0},
	{16934400, 8000, 2112, 0x17, 0x0},
	{12000000, 8000, 1500, 0x6, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x18, 0x0},
	{16934400, 11025, 1536, 0x19, 0x0},
	{12000000, 11025, 1088, 0x19, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0xa, 0x0},
	{18432000, 16000, 1152, 0xb, 0x0},
	{12000000, 16000, 750, 0xa, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x1a, 0x0},
	{16934400, 22050, 768, 0x1b, 0x0},
	{12000000, 22050, 544, 0x1b, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0xc, 0x0},
	{18432000, 32000, 576, 0xd, 0x0},
	{12000000, 32000, 375, 0xa, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x10, 0x0},
	{16934400, 44100, 384, 0x11, 0x0},
	{12000000, 44100, 272, 0x11, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0},
	{18432000, 48000, 384, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x1e, 0x0},
	{16934400, 88200, 192, 0x1f, 0x0},
	{12000000, 88200, 136, 0x1f, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0xe, 0x0},
	{18432000, 96000, 192, 0xf, 0x0},
	{12000000, 96000, 125, 0xe, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count	= ARRAY_SIZE(rates_12288),
	.list	= rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count	= ARRAY_SIZE(rates_112896),
	.list	= rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 2400, 32000, 41100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};


static int wm8960_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct wm8960_data *pdata = codec->dev->platform_data;
	u16 reg;

	WPRINTK("wm8960_set_bias_level:level:%d,codec->bias_level:%d\n",level,codec->bias_level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;


	case SND_SOC_BIAS_PREPARE:
		// Set VMID to 2x50k  
		reg = snd_soc_read(codec, WM8960_POWER1);
		reg &= ~0x180;
		reg |= 0x80;
		snd_soc_write(codec, WM8960_POWER1, reg);
		break;

	case SND_SOC_BIAS_STANDBY:
		#if 0
		if (codec->bias_level == SND_SOC_BIAS_OFF) {
			// Enable anti-pop features  
			snd_soc_write(codec, WM8960_APOP1,
				     WM8960_POBCTRL | WM8960_SOFT_ST |
				     WM8960_BUFDCOPEN | WM8960_BUFIOEN);

			// Discharge HP output  
			reg = WM8960_DISOP;
			if (pdata)
				reg |= pdata->dres << 4;
			snd_soc_write(codec, WM8960_APOP2, reg);

			msleep(400);

			snd_soc_write(codec, WM8960_APOP2, 0);

			// Enable & ramp VMID at 2x50k  
			reg = snd_soc_read(codec, WM8960_POWER1);
			reg |= 0x80;
			snd_soc_write(codec, WM8960_POWER1, reg);
			msleep(100);

			// Enable VREF  
			snd_soc_write(codec, WM8960_POWER1, reg | WM8960_VREF);

			// Disable anti-pop features  
			snd_soc_write(codec, WM8960_APOP1, WM8960_BUFIOEN);
		}

		// Set VMID to 2x250k 
		reg = snd_soc_read(codec, WM8960_POWER1);
		reg &= ~0x180;
		reg |= 0x100;
		snd_soc_write(codec, WM8960_POWER1, reg);
		#endif	
		snd_soc_write(codec, WM8960_POWER1, (WM_VMID50K|WM_VREF|WM_AINL|WM_AINR|WM_ADCL|WM_ADCR|WM_MICB));
	 	snd_soc_write(codec, WM8960_POWER2, (WM_DACL|WM_DACR|WM_LOUT1|WM_ROUT1|WM_SPKL|WM_SPKR|WM_PLLEN));
		break;

	case SND_SOC_BIAS_OFF:
		// Enable anti-pop features 
		snd_soc_write(codec, WM8960_APOP1,
			     WM8960_POBCTRL | WM8960_SOFT_ST |
			     WM8960_BUFDCOPEN | WM8960_BUFIOEN);

		// Disable VMID and VREF, let them discharge 
		snd_soc_write(codec, WM8960_POWER1, 0);
		msleep(100);//msleep(600);

		snd_soc_write(codec, WM8960_APOP1, 0);
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int wm8960_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8960_priv *wm8960 =snd_soc_codec_get_drvdata(codec);// codec->private_data;
	
	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		wm8960->sysclk_constraints = &constraints_112896;
		wm8960->sysclk = freq;
		return 0;

	case 12288000:
	case 16934400:
	case 24576000:
	case 33868800:
		wm8960->sysclk_constraints = &constraints_12288;
		wm8960->sysclk = freq;
		return 0;

	case 12000000:
	case 24000000:
		wm8960->sysclk_constraints = &constraints_12;
		wm8960->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int wm8960_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	WPRINTK("wm8960_set_dai_fmt:fmt:0x%x\n",fmt);
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface = 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	WPRINTK("Enter::%s----%d  iface=%x\n",__FUNCTION__,__LINE__,iface);
	snd_soc_write(codec, WM8960_IFACE1, iface);
	return 0;
}

//wisky-lxh@20110615,mute spk out
void codec_set_spk(int on)
{

	WPRINTK("%s,on:%d\n",__FUNCTION__,on);

	if (wm8960_codec == NULL) return;

	if(on){

		if (SPK_CON != NULL_GPIO)
			gpio_set_value(SPK_CON, SPEAKER_EN_VALUE);
		
		snd_soc_write(wm8960_codec, WM8960_LOUT2, spkl_reg);
		snd_soc_write(wm8960_codec, WM8960_ROUT2, spkr_reg);
	}else{
		//disable speaker
		snd_soc_write(wm8960_codec, WM8960_LOUT2, 0x100);
		snd_soc_write(wm8960_codec, WM8960_ROUT2, 0x100);
		
		if (SPK_CON != NULL_GPIO)
			gpio_set_value(SPK_CON, !SPEAKER_EN_VALUE);

	}
	
	return;
}
EXPORT_SYMBOL(codec_set_spk);
//end-wisky-lxh@20110615


static int wm8960_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct wm8960_priv *wm8960 = snd_soc_codec_get_drvdata(codec);//codec->private_data;
//	int ret;
//	u16 reg;
	WPRINTK("%s\n",__FUNCTION__);
	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/*wisky-lxh@20110621,when start the codec,judge headphone status,we only 
	control speaker here,control headphone in wm8960_trigger*/
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		//wisky-lxh@20110810,for skype,in capture
		WPRINTK("recording,mute speaker...\n");
		//codec_set_spk(0);
		return 0;
	}
	
	if(HEADPHONE_DET_IN_VALUE != gpio_get_value(HEADPHONE_DET_PIN) ){

		#if (defined(CONFIG_ANX7150) || defined(WISKY_HDMI_ANX7150)) && (WISKY_ENABLE_HDMI == 1)
			if(0 == hdmi_status){
				//enable speaker if headphone remove and not in HDMI output
				//gpio_set_value(SPK_CON, GPIO_HIGH);
				codec_set_spk(1);

			}else{
				//gpio_set_value(SPK_CON, GPIO_LOW);
				codec_set_spk(0);
			}
		#else
			//enable speaker if headphone remove or not in HDMI output
			//gpio_set_value(SPK_CON, GPIO_HIGH);
			codec_set_spk(1);
		#endif
	}else{//headphone insert
			WPRINTK("headphone insert\n");
			//gpio_set_value(SPK_CON, GPIO_LOW);//disable speaker			
			codec_set_spk(0);
	}

	//end-wisky-lxh@20110621
#if 0
	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	if (!wm8960->sysclk) {
		dev_err(codec->dev,
			"No MCLK configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   wm8960->sysclk_constraints);
#endif

	return 0;
}

static int wm8960_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct wm8960_priv *wm8960 = snd_soc_codec_get_drvdata(codec);
	u16 iface = snd_soc_read(codec, WM8960_IFACE1) & 0xfff3;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	}

	/* set iface */
	snd_soc_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int div)
{
/*	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int reg;
*/
	return 0;
}

static int wm8960_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{

	return 0;
}

static int wm8960_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = snd_soc_read(codec, WM8960_DACCTL1) & 0xfff7;

	if (mute){
		snd_soc_write(codec, WM8960_DACCTL1, mute_reg | 0x8);
	}	
	else{
		snd_soc_write(codec, WM8960_DACCTL1, mute_reg);			
	}

#if 0
	if(!mute){
		audio_trigger_flag = 1;
		/*if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		{
			gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
			WPRINTK("recording,mute headphone...\n");
			return 0;
		}*/
		if(HEADPHONE_DET_IN_VALUE != gpio_get_value(HEADPHONE_DET_PIN) ){
			WPRINTK("headphone remove\n");
		//headphone remove
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
				
			}
#endif
#if (defined(CONFIG_ANX7150) || defined(WISKY_HDMI_ANX7150)) && (WISKY_ENABLE_HDMI == 1)
			if(0 == hdmi_status){
				//enable speaker if headphone remove and not in HDMI output
				gpio_set_value(SPK_CON, GPIO_HIGH);				
			}else{
				gpio_set_value(SPK_CON, GPIO_LOW);
			}
#else
			//enable speaker if headphone remove or not in HDMI output
			gpio_set_value(SPK_CON, GPIO_HIGH);
#endif
		}else{//headphone insert
			WPRINTK("headphone insert\n");
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				if(audio_volume != 0){
					gpio_set_value(HEADPHONE_MUTE_PIN, !HEADPHONE_MUTE_ACTIVE_LEVEL);//enable headphone
				}
			}
#endif
			gpio_set_value(SPK_CON, GPIO_LOW);//disable speaker
		}
	}else{
#if defined(HEADPHONE_MUTE_PIN)
		if(HEADPHONE_MUTE_PIN != NULL_GPIO){
			gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
		}
#endif
		gpio_set_value(SPK_CON, GPIO_LOW);//disable speaker
		audio_trigger_flag = 0;
	}
#endif
	return 0;
}



static int wm8960_trigger(struct snd_pcm_substream *substream, int trigger, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

#if 0
	WPRINTK("wm8960_trigger:trigger:%d\n",trigger);
	
	if(trigger == 1){
		audio_trigger_flag = 1;
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		{
			gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
			WPRINTK("recording,mute headphone...\n");
			return 0;
		}
		if(HEADPHONE_DET_IN_VALUE != gpio_get_value(HEADPHONE_DET_PIN) ){
			WPRINTK("headphone remove\n");
		//headphone remove
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
				
			}
#endif
#if (defined(CONFIG_ANX7150) || defined(WISKY_HDMI_ANX7150)) && (WISKY_ENABLE_HDMI == 1)
			if(0 == hdmi_status){
				//enable speaker if headphone remove and not in HDMI output
				gpio_set_value(SPK_CON, GPIO_HIGH);				
			}else{
				gpio_set_value(SPK_CON, GPIO_LOW);
			}
#else
			//enable speaker if headphone remove or not in HDMI output
			gpio_set_value(SPK_CON, GPIO_HIGH);
#endif
		}else{//headphone insert
			WPRINTK("headphone insert\n");
#if defined(HEADPHONE_MUTE_PIN)
			if(HEADPHONE_MUTE_PIN != NULL_GPIO){
				if(audio_volume != 0){
					gpio_set_value(HEADPHONE_MUTE_PIN, !HEADPHONE_MUTE_ACTIVE_LEVEL);//enable headphone
				}
			}
#endif
			gpio_set_value(SPK_CON, GPIO_LOW);//disable speaker
		}
	}else{
#if defined(HEADPHONE_MUTE_PIN)
		if(HEADPHONE_MUTE_PIN != NULL_GPIO){
			gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
		}
#endif
		gpio_set_value(SPK_CON, GPIO_LOW);//disable speaker
		audio_trigger_flag = 0;
	}
#else
	if (trigger == 1 && substream->stream == SNDRV_PCM_STREAM_CAPTURE){
		gpio_set_value(HEADPHONE_MUTE_PIN, HEADPHONE_MUTE_ACTIVE_LEVEL);//disable headphone
		WPRINTK("recording,mute headphone...\n");
		return 0;
	}
#endif

	return 0;
}

#define WM8960_RATES SNDRV_PCM_RATE_8000_96000

#define WM8960_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops wm8960_ops = {
	.startup = wm8960_pcm_startup,
	.hw_params = wm8960_pcm_hw_params,
	.set_clkdiv = wm8960_set_dai_clkdiv,
	.set_pll = wm8960_set_dai_pll,
	.set_fmt = wm8960_set_dai_fmt,
	.set_sysclk = wm8960_set_dai_sysclk,
	.digital_mute = wm8960_mute,
	.trigger = wm8960_trigger,
};

static struct snd_soc_dai_driver wm8960_dai = {
	.name = "wm8960 hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,
	 },
	.ops = &wm8960_ops,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(wm8960_dai);

static int wm8960_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
//	struct snd_soc_codec *codec = socdev->card->codec;

	//close SPK control GPIO
	codec_set_spk(0);
	gpio_request(SPK_CON, NULL);
	gpio_direction_output(SPK_CON,GPIO_LOW);
	gpio_free(SPK_CON);
	
	wm8960_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8960_resume(struct snd_soc_codec *codec)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
//	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;
	
#if 0
	/* Sync reg_cache with the hardware */
	for (i = 0; i < wm8960_NUM_REG; i++) {
		if (i == wm8960_RESET)
			continue;
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}
#endif

	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
/*
static struct snd_soc_codec *wm8988_codec;

static int entry_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;

	snd_soc_write(wm8988_codec, WM8988_PWR1, 0x0000);
	snd_soc_write(wm8988_codec, WM8988_PWR2, 0x0000);

	len = sprintf(page, "wm8988 suspend...\n");

	return len ;
}
*/
static int wm8960_probe(struct snd_soc_codec *codec)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
//	struct snd_soc_codec *codec;
	struct wm8960_priv *wm8960= snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	int ret = 0,reg;

	if (codec == NULL) {
		dev_err(codec->dev, "Codec device not registered\n");
		return -ENODEV;
	}
	
	wm8960_codec=codec  ;

	/* register pcms */
/*	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, wm8960_snd_controls,
				ARRAY_SIZE(wm8960_snd_controls));
	snd_soc_dapm_new_controls(codec, wm8960_dapm_widgets,
				  ARRAY_SIZE(wm8960_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_new_widgets(codec);

	ret = snd_soc_init_card(socdev);*/
		ret = snd_soc_codec_set_cache_io(codec, 7, 9, wm8960->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "failed to register card: %d\n", ret);
		return ret;
	}
	
	ret = wm8960_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		return ret;
	}
	
	reg = snd_soc_read(codec, WM8960_LDAC);
	snd_soc_write(codec, WM8960_LDAC, reg | 0x100);
	reg = snd_soc_read(codec, WM8960_RDAC);
	snd_soc_write(codec, WM8960_RDAC, reg | 0x100);
	reg = snd_soc_read(codec, WM8960_LOUT1);
	snd_soc_write(codec, WM8960_LOUT1, reg | 0x100);
	reg = snd_soc_read(codec, WM8960_ROUT1);
	snd_soc_write(codec, WM8960_ROUT1, reg | 0x100);
	reg = snd_soc_read(codec, WM8960_LOUT2);
	snd_soc_write(codec, WM8960_LOUT2, reg | 0x100);
	reg = snd_soc_read(codec, WM8960_ROUT2);
	snd_soc_write(codec, WM8960_ROUT2, reg | 0x100);

#if defined(WISKY_OEM_PLOYER_MOMO)||defined(WISKY_BOARD_M809_V30)
	//headphone output volume update
	snd_soc_write(codec, WM8960_LOUT1,0x017a );//0x017f  0x0166
	snd_soc_write(codec, WM8960_ROUT1, 0x017a);//0x017f  0x0166
#else
	//headphone output volume update
	snd_soc_write(codec, WM8960_LOUT1,0x0166 );//0x017f  0x0166
	snd_soc_write(codec, WM8960_ROUT1, 0x0166);//0x017f  0x0166
#endif

	//set speaker volume
#if defined(WISKY_BOARD_M810_V11)||defined(WISKY_BOARD_M810_V10)		
	spkl_reg = 0x17c;//0x17c
	spkr_reg = 0x17c;
#else
	spkl_reg = 0x017f;
	spkr_reg = 0x017f;
#endif
	snd_soc_write(codec, WM8960_LOUT2, spkl_reg);//0x017f
	snd_soc_write(codec, WM8960_ROUT2, spkr_reg);//0x017f

#if defined(WISKY_BOARD_M810_V11)
	//ADC 
	snd_soc_write(codec, WM8960_LINVOL, 0x013f);// 0x013f-->+30dB,0x0127-->+12dB
	snd_soc_write(codec, WM8960_RINVOL, 0x013f);//0x013f
	snd_soc_write(codec, WM8960_LADC, 0x01f7);  //0x1c3        max 0x1ff
	snd_soc_write(codec, WM8960_RADC, 0x01f7);
#elif defined(WISKY_BOARD_M808_V22)
	snd_soc_write(codec, WM8960_LINVOL, 0x0132);// 0x013f-->+30dB,0x0127-->+12dB
	snd_soc_write(codec, WM8960_RINVOL, 0x0132);//0x013f
	snd_soc_write(codec, WM8960_LADC, 0x01e1);  //max 0x1ff
	snd_soc_write(codec, WM8960_RADC, 0x01e1);
#elif defined(WISKY_BOARD_M809_V30)
	snd_soc_write(codec, WM8960_LINVOL, 0x0130);// 0x013f-->+30dB,0x0127-->+12dB
	snd_soc_write(codec, WM8960_RINVOL, 0x0130);//0x013f
	snd_soc_write(codec, WM8960_LADC, 0x01c3);  //max 0x1ff
	snd_soc_write(codec, WM8960_RADC, 0x01c3);
#else
	snd_soc_write(codec, WM8960_LINVOL, 0x013f);// 0x013f-->+30dB,0x0127-->+12dB
	snd_soc_write(codec, WM8960_RINVOL, 0x013f);//0x013f
	snd_soc_write(codec, WM8960_LADC, 0x01c3);  //0x1c3        max 0x1ff
	snd_soc_write(codec, WM8960_RADC, 0x01c3);
#endif
	
	//MIC LINPUT1&LINPUT2 
	#if defined(WISKY_BOARD_M802H)
	snd_soc_write(codec, WM8960_LINPATH, 0x0158);
	snd_soc_write(codec, WM8960_RINPATH, 0x0158);
	snd_soc_write(codec, WM8960_CLASSD1, 0x00c0);
	#elif defined(WISKY_BOARD_M808_V22)
	snd_soc_write(codec, WM8960_LINPATH, 0x0168);
	snd_soc_write(codec, WM8960_RINPATH, 0x0168);
	#else
	snd_soc_write(codec, WM8960_LINPATH, 0x0158);//0x0178
	snd_soc_write(codec, WM8960_RINPATH, 0x0158);//0x0178
	#endif
	//snd_soc_write(codec, WM8960_RINPATH, 0x01ff);
	
	snd_soc_write(codec, WM8960_LOUTMIX,  0x0100);
	snd_soc_write(codec, WM8960_ROUTMIX,  0x0100);

	
//	snd_soc_write(codec, wm8960_SRATE, 0x100);  ///SET MCLK/8
	//snd_soc_write(codec, wm8960_PWR1, (WM_VMID50K|WM_VREF|WM_AINL|WM_AINR|WM_ADCL|WM_ADCR));
	snd_soc_write(codec, WM8960_POWER1, 0x000);
 	snd_soc_write(codec, WM8960_POWER2, 0x000);
	snd_soc_write(codec, WM8960_POWER3, 0x03c);
	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	snd_soc_add_controls(codec, wm8960_snd_controls,
				ARRAY_SIZE(wm8960_snd_controls));
	snd_soc_dapm_new_controls(dapm, wm8960_dapm_widgets,
				  ARRAY_SIZE(wm8960_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
//	create_proc_read_entry("wm8960_suspend", 0644, NULL, entry_read, NULL);

	return 0;
}

static int wm8960_remove(struct platform_device *pdev)
{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

//	snd_soc_free_pcms(socdev);
//	snd_soc_dapm_free(socdev);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_wm8960 = {
	.probe = 	wm8960_probe,
	.remove = wm8960_remove,
	.suspend = wm8960_suspend,
	.resume = wm8960_resume,
	.set_bias_level = wm8960_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(wm8960_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = wm8960_reg,
};

#if defined(CONFIG_SPI_MASTER)
static int __devinit wm8960_spi_probe(struct spi_device *spi)
{
	struct wm8960_priv *wm8960;
	struct snd_soc_codec *codec;

	wm8960 = kzalloc(sizeof(struct wm8960_priv), GFP_KERNEL);
	if (wm8960 == NULL)
		return -ENOMEM;

	codec = &wm8960->codec;
	codec->control_data = spi;
	codec->dev = &spi->dev;

	dev_set_drvdata(&spi->dev, wm8960);

	return wm8960_register(wm8960, SND_SOC_SPI);
}

static int __devexit wm8960_spi_remove(struct spi_device *spi)
{
	struct wm8960_priv *wm8960 = dev_get_drvdata(&spi->dev);

	wm8960_unregister(wm8960);

	return 0;
}


#ifdef CONFIG_PM
static int wm8960_spi_suspend(struct spi_device *spi, pm_message_t msg)
{
	return snd_soc_suspend_device(&spi->dev);
}

static int wm8960_spi_resume(struct spi_device *spi)
{
	return snd_soc_resume_device(&spi->dev);
}
#else
#define wm8960_spi_suspend NULL
#define wm8960_spi_resume NULL
#endif

static struct spi_driver wm8960_spi_driver = {
	.driver = {
		.name	= "wm8960",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= wm8960_spi_probe,
	.remove		= __devexit_p(wm8960_spi_remove),
	.suspend	= wm8960_spi_suspend,
	.resume		= wm8960_spi_resume,
};
#endif
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int wm8960_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wm8960_priv *wm8960;
	int ret;

	wm8960 = kzalloc(sizeof(struct wm8960_priv), GFP_KERNEL);
	if (wm8960 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, wm8960);
	wm8960->control_type = SND_SOC_I2C;

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_wm8960, &wm8960_dai, 1);
	if (ret < 0)
		kfree(wm8960);

}

static int wm8960_i2c_remove(struct i2c_client *client)
{
	struct wm8960_priv *wm8960 = i2c_get_clientdata(client);

	return 0;
}
/*
#ifdef CONFIG_PM
static int wm8960_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	return snd_soc_suspend_device(&client->dev);
}

static int wm8960_i2c_resume(struct i2c_client *client)
{
	return snd_soc_resume_device(&client->dev);
}
#else
#define wm8960_i2c_suspend NULL
#define wm8960_i2c_resume NULL
#endif
*/
static const struct i2c_device_id wm8960_i2c_id[] = {
	{ "wm8960", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wm8960_i2c_id);

static struct i2c_driver wm8960_i2c_driver = {
	.driver = {
		.name = "wm8960",
		.owner = THIS_MODULE,
	},
	.probe = wm8960_i2c_probe,
	.remove = wm8960_i2c_remove,
//	.suspend = wm8960_i2c_suspend,
//	.resume = wm8960_i2c_resume,
	.id_table = wm8960_i2c_id,
};
#endif



static int __init wm8960_modinit(void)
{
	int ret;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&wm8960_i2c_driver);
	if (ret != 0)
		pr_err("wm8960: Unable to register I2C driver: %d\n", ret);
#endif
#if defined(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&wm8960_spi_driver);
	if (ret != 0)
		pr_err("wm8960: Unable to register SPI driver: %d\n", ret);
#endif
	return ret;
}
module_init(wm8960_modinit);

static void __exit wm8960_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8960_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&wm8960_spi_driver);
#endif
}
module_exit(wm8960_exit);


MODULE_DESCRIPTION("ASoC wm8960 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
