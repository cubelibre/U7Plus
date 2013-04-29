/*
 * es8323.c -- es8323 ALSA SoC audio driver
 *
 * Copyright 2009 Wolfson Microelectronics plc
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <mach/board.h>
#include <mach/gpio.h>
//wisky libai 20120907
#include <linux/wisky.h>
//leaf #include <linux/tchip_sysinf.h>

#include "wisky_codec_es8323.h"

#include <linux/proc_fs.h>
//#include <linux/gpio.h>

#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif
#define alsa_dbg DBG

//#define SPK_CTL 		RK29_PIN6_PB7
//#define SPK_CON 		RK29_PIN6_PB6

//wisky libai 20120907
//#define SPK_CTL RK30_PIN4_PC5
#define SPK_CTL SPEAKER_CTRL_PIN
//    #undef SPK_CTL
//   #undef SPK_CON

//#define SPK_CTL             RK29_PIN6_PB6
//#define EAR_CON_PIN            RK30_PIN4_PC6 // RK29_PIN6_PB5
#define EAR_CON_PIN		RK30_PIN4_PC6

#ifndef ES8323_DEF_VOL
#define ES8323_DEF_VOL			0x17//wisky libai 0x20
#endif

/*
 * 8323 register cache
 * We can't read the 8323 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static u16 es8323_reg[] = {
	0x06, 0x1C, 0xC3, 0xFC,  /*  0 *////0x0100 0x0180
	0xC0, 0x00, 0x00, 0x7C,  /*  4 */
	0x80, 0x00, 0x00, 0x06,  /*  8 */
	0x00, 0x06, 0x30, 0x30,  /* 12 */
	0xC0, 0xC0, 0x38, 0xB0,  /* 16 */
	0x32, 0x06, 0x00, 0x00,  /* 20 */
	0x06, 0x30, 0xC0, 0xC0,  /* 24 */
	0x08, 0x06, 0x1F, 0xF7,  /* 28 */
	0xFD, 0xFF, 0x1F, 0xF7,  /* 32 */
	0xFD, 0xFF, 0x00, 0x38,  /* 36 */
	0x38, 0x38, 0x38, 0x38,  /* 40 */
	0x38, 0x00, 0x00, 0x00,  /* 44 */
	0x00, 0x00, 0x00, 0x00,  /* 48 */
	0x00, 0x00, 0x00, 0x00,  /* 52 */
};

/* codec private data */
struct es8323_priv {
	unsigned int sysclk;
	enum snd_soc_control_type control_type;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	int is_startup;		// gModify.Add
	int is_biason;
};

static unsigned int es8323_read_reg_cache(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(es8323_reg))
		return -1;
	return es8323_reg[reg];
}

static int es8323_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u16 *cache = codec->reg_cache;
	u8 data[2];
	int ret;

	BUG_ON(codec->volatile_register);

	data[0] = reg;
	data[1] = value & 0x00ff;

	if (reg < ARRAY_SIZE(es8323_reg))
		es8323_reg[reg] = value;
	ret = codec->hw_write(codec->control_data, data, 2);
	if (ret == 2)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

//#define 8323_reset(c)	snd_soc_write(c, 8323_RESET, 0)
 static int es8323_reset(struct snd_soc_codec *codec)
 {
 	snd_soc_write(codec, ES8323_CONTROL1, 0x80);
  return snd_soc_write(codec, ES8323_CONTROL1, 0x00);
 }
/*
 * 8323 Controls
 */
/*
static const char *bass_boost_txt[] = {"Linear Control", "Adaptive Boost"};
static const struct soc_enum bass_boost =
	SOC_ENUM_SINGLE(8323_BASS, 7, 2, bass_boost_txt);

static const char *bass_filter_txt[] = { "130Hz @ 48kHz", "200Hz @ 48kHz" };
static const struct soc_enum bass_filter =
	SOC_ENUM_SINGLE(8323_BASS, 6, 2, bass_filter_txt);

static const char *treble_txt[] = {"8kHz", "4kHz"};
static const struct soc_enum treble =
	SOC_ENUM_SINGLE(8323_TREBLE, 6, 2, treble_txt);

static const char *stereo_3d_lc_txt[] = {"200Hz", "500Hz"};
static const struct soc_enum stereo_3d_lc =
	SOC_ENUM_SINGLE(8323_3D, 5, 2, stereo_3d_lc_txt);

static const char *stereo_3d_uc_txt[] = {"2.2kHz", "1.5kHz"};
static const struct soc_enum stereo_3d_uc =
	SOC_ENUM_SINGLE(8323_3D, 6, 2, stereo_3d_uc_txt);

static const char *stereo_3d_func_txt[] = {"Capture", "Playback"};
static const struct soc_enum stereo_3d_func =
	SOC_ENUM_SINGLE(8323_3D, 7, 2, stereo_3d_func_txt);
*/
static const char *stereo_3d_txt[] = {"No 3D  ", "Level 1","Level 2","Level 3","Level 4","Level 5","Level 6","Level 7"};
static const struct soc_enum stereo_3d =
	SOC_ENUM_SINGLE(ES8323_DACCONTROL7, 2, 8, stereo_3d_txt);
	
static const char *alc_func_txt[] = {"Off", "Right", "Left", "Stereo"};
static const struct soc_enum alc_func =
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL10, 6, 4, alc_func_txt);

static const char *ng_type_txt[] = {"Constant PGA Gain",
				    "Mute ADC Output"};
static const struct soc_enum ng_type =
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL14, 1, 2, ng_type_txt);

static const char *deemph_txt[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const struct soc_enum deemph =
	SOC_ENUM_SINGLE(ES8323_DACCONTROL6, 6, 4, deemph_txt);

static const char *adcpol_txt[] = {"Normal", "L Invert", "R Invert",
				   "L + R Invert"};
static const struct soc_enum adcpol =
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL6, 6, 4, adcpol_txt);

static const DECLARE_TLV_DB_SCALE(pga_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -1500, 300, 0);

static const struct snd_kcontrol_new es8323_snd_controls[] = {
/*
SOC_ENUM("Bass Boost", bass_boost),
SOC_ENUM("Bass Filter", bass_filter),
SOC_SINGLE("Bass Volume", 8323_BASS, 0, 15, 1),

SOC_SINGLE("Treble Volume", 8323_TREBLE, 0, 15, 0),
SOC_ENUM("Treble Cut-off", treble),

SOC_SINGLE("3D Switch", 8323_3D, 0, 1, 0),
SOC_SINGLE("3D Volume", 8323_3D, 1, 15, 0),
SOC_ENUM("3D Lower Cut-off", stereo_3d_lc),
SOC_ENUM("3D Upper Cut-off", stereo_3d_uc),
SOC_ENUM("3D Mode", stereo_3d_func),
*/
SOC_ENUM("3D Mode", stereo_3d),
SOC_SINGLE("ALC Capture Target Volume", ES8323_ADCCONTROL11, 4, 15, 0),
SOC_SINGLE("ALC Capture Max PGA", ES8323_ADCCONTROL10, 3, 7, 0),
SOC_SINGLE("ALC Capture Min PGA", ES8323_ADCCONTROL10, 0, 7, 0),
SOC_ENUM("ALC Capture Function", alc_func),
SOC_SINGLE("ALC Capture ZC Switch", ES8323_ADCCONTROL13, 6, 1, 0),
SOC_SINGLE("ALC Capture Hold Time", ES8323_ADCCONTROL11, 0, 15, 0),
SOC_SINGLE("ALC Capture Decay Time", ES8323_ADCCONTROL12, 4, 15, 0),
SOC_SINGLE("ALC Capture Attack Time", ES8323_ADCCONTROL12, 0, 15, 0),
SOC_SINGLE("ALC Capture NG Threshold", ES8323_ADCCONTROL14, 3, 31, 0),
SOC_ENUM("ALC Capture NG Type", ng_type),
SOC_SINGLE("ALC Capture NG Switch", ES8323_ADCCONTROL14, 0, 1, 0),

SOC_SINGLE("ZC Timeout Switch", ES8323_ADCCONTROL13, 6, 1, 0),

SOC_DOUBLE_R_TLV("Capture Digital Volume", ES8323_ADCCONTROL8, ES8323_ADCCONTROL9,
		 0, 255, 1, adc_tlv),
		 
SOC_SINGLE("Capture Mute", ES8323_ADCCONTROL7, 2, 1, 0),
		 /*
SOC_DOUBLE_R_TLV("Capture Volume", 8323_LINVOL, 8323_RINVOL,
		 0, 63, 0, pga_tlv),
		 */
SOC_SINGLE_TLV("Left Channel Capture Volume",	ES8323_ADCCONTROL1, 4, 15, 0, bypass_tlv),
SOC_SINGLE_TLV("Right Channel Capture Volume",	ES8323_ADCCONTROL1, 0, 15, 0, bypass_tlv),
	 
//SOC_DOUBLE_R("Capture ZC Switch", 8323_LINVOL, 8323_RINVOL, 6, 1, 0),
//SOC_SINGLE("Capture Switch", ES8323_ADCPOWER, 4, 3, 1),

SOC_ENUM("Playback De-emphasis", deemph),

SOC_ENUM("Capture Polarity", adcpol),
//SOC_SINGLE("Playback 6dB Attenuate", 8323_ADCDAC, 7, 1, 0),
//SOC_SINGLE("Capture 6dB Attenuate", 8323_ADCDAC, 8, 1, 0),

SOC_DOUBLE_R_TLV("PCM Volume", ES8323_DACCONTROL4, ES8323_DACCONTROL5, 0, 255, 1, dac_tlv),

SOC_SINGLE_TLV("Left Mixer Left Bypass Volume", ES8323_DACCONTROL17, 3, 7, 1, bypass_tlv),
/*
SOC_SINGLE_TLV("Left Mixer Right Bypass Volume", 8323_LOUTM2, 4, 7, 1,
	       bypass_tlv),
SOC_SINGLE_TLV("Right Mixer Left Bypass Volume", 8323_ROUTM1, 4, 7, 1,
	       bypass_tlv),
*/
SOC_SINGLE_TLV("Right Mixer Right Bypass Volume", ES8323_DACCONTROL20, 3, 7, 1,
	       bypass_tlv),

//SOC_DOUBLE_R("Output 1 Playback ZC Switch", 8323_LOUT1V,8323_ROUT1V, 7, 1, 0),
SOC_DOUBLE_R_TLV("Output 1 Playback Volume", ES8323_DACCONTROL24, ES8323_DACCONTROL25,
		 0, 64, 0, out_tlv),

//SOC_DOUBLE_R("Output 2 Playback ZC Switch", 8323_LOUT2V,8323_ROUT2V, 7, 1, 0),
SOC_DOUBLE_R_TLV("Output 2 Playback Volume", ES8323_DACCONTROL26, ES8323_DACCONTROL27,
		 0, 64, 0, out_tlv),
};

/*
 * DAPM Controls
 */
/*
static int 8323_lrc_control(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u16 adctl2 = snd_soc_read(codec, 8323_ADCTL2);
*/
	/* Use the DAC to gate LRC if active, otherwise use ADC */
	/*
	if (snd_soc_read(codec, 8323_PWR2) & 0x180)
		adctl2 &= ~0x4;
	else
		adctl2 |= 0x4;

	DBG("Enter::%s----%d, adctl2 = %x\n",__FUNCTION__,__LINE__,adctl2);
	
	return snd_soc_write(codec, 8323_ADCTL2, adctl2);
}
*/
static const char *es8323_line_texts[] = {
	"Line 1", "Line 2", "PGA"};

static const unsigned int es8323_line_values[] = {
	0, 1, 3};

static const struct soc_enum es8323_lline_enum =
	SOC_VALUE_ENUM_SINGLE(ES8323_DACCONTROL16, 3, 7,
			      ARRAY_SIZE(es8323_line_texts),
			      es8323_line_texts,
			      es8323_line_values);
static const struct snd_kcontrol_new es8323_left_line_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8323_lline_enum);

static const struct soc_enum es8323_rline_enum =
	SOC_VALUE_ENUM_SINGLE(ES8323_DACCONTROL16, 0, 7,
			      ARRAY_SIZE(es8323_line_texts),
			      es8323_line_texts,
			      es8323_line_values);
static const struct snd_kcontrol_new es8323_right_line_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8323_rline_enum);

/* Left Mixer */
static const struct snd_kcontrol_new es8323_left_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", ES8323_DACCONTROL17, 7, 1, 0),
	SOC_DAPM_SINGLE("Left Bypass Switch", ES8323_DACCONTROL17, 6, 1, 0),
	//SOC_DAPM_SINGLE("Right Playback Switch", 8323_LOUTM2, 8, 1, 0),
	//SOC_DAPM_SINGLE("Right Bypass Switch", 8323_LOUTM2, 7, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new es8323_right_mixer_controls[] = {
	//SOC_DAPM_SINGLE("Left Playback Switch", 8323_ROUTM1, 8, 1, 0),
	//SOC_DAPM_SINGLE("Left Bypass Switch", 8323_ROUTM1, 7, 1, 0),
	SOC_DAPM_SINGLE("Right Playback Switch", ES8323_DACCONTROL20, 7, 1, 0),
	SOC_DAPM_SINGLE("Right Bypass Switch", ES8323_DACCONTROL20, 6, 1, 0),
};

static const char *es8323_pga_sel[] = {"Line 1", "Line 2", "Differential"};
static const unsigned int es8323_pga_val[] = { 0, 1, 3 };

/* Left PGA Mux */
static const struct soc_enum es8323_lpga_enum =
	SOC_VALUE_ENUM_SINGLE(ES8323_ADCCONTROL2, 6, 3,
			      ARRAY_SIZE(es8323_pga_sel),
			      es8323_pga_sel,
			      es8323_pga_val);
static const struct snd_kcontrol_new es8323_left_pga_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8323_lpga_enum);

/* Right PGA Mux */
static const struct soc_enum es8323_rpga_enum =
	SOC_VALUE_ENUM_SINGLE(ES8323_ADCCONTROL2, 4, 3,
			      ARRAY_SIZE(es8323_pga_sel),
			      es8323_pga_sel,
			      es8323_pga_val);
static const struct snd_kcontrol_new es8323_right_pga_controls =
	SOC_DAPM_VALUE_ENUM("Route", es8323_rpga_enum);

/* Differential Mux */
static const char *es8323_diff_sel[] = {"Line 1", "Line 2"};
static const struct soc_enum diffmux =
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL3, 7, 2, es8323_diff_sel);
static const struct snd_kcontrol_new es8323_diffmux_controls =
	SOC_DAPM_ENUM("Route", diffmux);

/* Mono ADC Mux */
static const char *es8323_mono_mux[] = {"Stereo", "Mono (Left)",
	"Mono (Right)"};
static const struct soc_enum monomux =
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL3, 3, 3, es8323_mono_mux);
static const struct snd_kcontrol_new es8323_monomux_controls =
	SOC_DAPM_ENUM("Route", monomux);

static const struct snd_soc_dapm_widget es8323_dapm_widgets[] = {
	SND_SOC_DAPM_MICBIAS("Mic Bias", ES8323_ADCPOWER, 3, 1),

	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
		&es8323_diffmux_controls),
		
	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
		&es8323_monomux_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
		&es8323_monomux_controls),
 
	SND_SOC_DAPM_MUX("Left PGA Mux", ES8323_ADCPOWER, 7, 1,
		&es8323_left_pga_controls),
	SND_SOC_DAPM_MUX("Right PGA Mux", ES8323_ADCPOWER, 6, 1,
		&es8323_right_pga_controls),

	SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
		&es8323_left_line_controls),
	SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
		&es8323_right_line_controls),

	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ES8323_ADCPOWER, 4, 1),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ES8323_ADCPOWER, 5, 1),

	/* gModify.Cmmt Implement when suspend/startup */
	/*SND_SOC_DAPM_DAC("Right DAC", "Right Playback", 8323_PWR2, 7, 0),*/
	/*SND_SOC_DAPM_DAC("Left DAC", "Left Playback", 8323_PWR2, 8, 0),*/

	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
		&es8323_left_mixer_controls[0],
		ARRAY_SIZE(es8323_left_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
		&es8323_right_mixer_controls[0],
		ARRAY_SIZE(es8323_right_mixer_controls)),

	SND_SOC_DAPM_PGA("Right Out 2", ES8323_DACPOWER, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", ES8323_DACPOWER, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", ES8323_DACPOWER, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 1", ES8323_DACPOWER, 5, 0, NULL, 0),

	//SND_SOC_DAPM_POST("LRC control", 8323_lrc_control),

	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	SND_SOC_DAPM_OUTPUT("VREF"),

	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("LINPUT2"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT2"),
};

static const struct snd_soc_dapm_route audio_map[] = {

	{ "Left Line Mux", "Line 1", "LINPUT1" },
	{ "Left Line Mux", "Line 2", "LINPUT2" },
	{ "Left Line Mux", "PGA", "Left PGA Mux" },
	//{ "Left Line Mux", "Differential", "Differential Mux" },

	{ "Right Line Mux", "Line 1", "RINPUT1" },
	{ "Right Line Mux", "Line 2", "RINPUT2" },
	{ "Right Line Mux", "PGA", "Right PGA Mux" },
	//{ "Right Line Mux", "Differential", "Differential Mux" },

	{ "Left PGA Mux", "Line 1", "LINPUT1" },
	{ "Left PGA Mux", "Line 2", "LINPUT2" },
	{ "Left PGA Mux", "Differential", "Differential Mux" },

	{ "Right PGA Mux", "Line 1", "RINPUT1" },
	{ "Right PGA Mux", "Line 2", "RINPUT2" },
	{ "Right PGA Mux", "Differential", "Differential Mux" },

	{ "Differential Mux", "Line 1", "LINPUT1" },
	{ "Differential Mux", "Line 1", "RINPUT1" },
	{ "Differential Mux", "Line 2", "LINPUT2" },
	{ "Differential Mux", "Line 2", "RINPUT2" },

	{ "Left ADC Mux", "Stereo", "Left PGA Mux" },
	{ "Left ADC Mux", "Mono (Left)", "Left PGA Mux" },
	//{ "Left ADC Mux", "Digital Mono", "Left PGA Mux" },

	{ "Right ADC Mux", "Stereo", "Right PGA Mux" },
	{ "Right ADC Mux", "Mono (Right)", "Right PGA Mux" },
	//{ "Right ADC Mux", "Digital Mono", "Right PGA Mux" },

	{ "Left ADC", NULL, "Left ADC Mux" },
	{ "Right ADC", NULL, "Right ADC Mux" },

	{ "Left Line Mux", "Line 1", "LINPUT1" },
	{ "Left Line Mux", "Line 2", "LINPUT2" },
	{ "Left Line Mux", "PGA", "Left PGA Mux" },
	//{ "Left Line Mux", "Differential", "Differential Mux" },

	{ "Right Line Mux", "Line 1", "RINPUT1" },
	{ "Right Line Mux", "Line 2", "RINPUT2" },
	{ "Right Line Mux", "PGA", "Right PGA Mux" },
	//{ "Right Line Mux", "Differential", "Differential Mux" },

	{ "Left Mixer", "Left Playback Switch", "Left DAC" },
	{ "Left Mixer", "Left Bypass Switch", "Left Line Mux" },
	//{ "Left Mixer", "Right Playback Switch", "Right DAC" },
	//{ "Left Mixer", "Right Bypass Switch", "Right Line Mux" },

	//{ "Right Mixer", "Left Playback Switch", "Left DAC" },
	//{ "Right Mixer", "Left Bypass Switch", "Left Line Mux" },
	{ "Right Mixer", "Right Playback Switch", "Right DAC" },
	{ "Right Mixer", "Right Bypass Switch", "Right Line Mux" },

	{ "Left Out 1", NULL, "Left Mixer" },
	{ "LOUT1", NULL, "Left Out 1" },
	{ "Right Out 1", NULL, "Right Mixer" },
	{ "ROUT1", NULL, "Right Out 1" },

	{ "Left Out 2", NULL, "Left Mixer" },
	{ "LOUT2", NULL, "Left Out 2" },
	{ "Right Out 2", NULL, "Right Mixer" },
	{ "ROUT2", NULL, "Right Out 2" },
};

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0xa, 0x0},
	{11289600, 8000, 1408, 0x9, 0x0},
	{18432000, 8000, 2304, 0xc, 0x0},
	{16934400, 8000, 2112, 0xb, 0x0},
	{12000000, 8000, 1500, 0xb, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x7, 0x0},
	{16934400, 11025, 1536, 0xa, 0x0},
	{12000000, 11025, 1088, 0x9, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0x6, 0x0},
	{18432000, 16000, 1152, 0x8, 0x0},
	{12000000, 16000, 750, 0x7, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x4, 0x0},
	{16934400, 22050, 768, 0x6, 0x0},
	{12000000, 22050, 544, 0x6, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x3, 0x0},
	{18432000, 32000, 576, 0x5, 0x0},
	{12000000, 32000, 375, 0x4, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x2, 0x0},
	{16934400, 44100, 384, 0x3, 0x0},
	{12000000, 44100, 272, 0x3, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x2, 0x0},
	{18432000, 48000, 384, 0x3, 0x0},
	{12000000, 48000, 250, 0x2, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x0, 0x0},
	{16934400, 88200, 192, 0x1, 0x0},
	{12000000, 88200, 136, 0x1, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x0, 0x0},
	{18432000, 96000, 192, 0x1, 0x0},
	{12000000, 96000, 125, 0x0, 0x1},
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
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};

static void on_off_ext_amp(int i)
{

    #ifdef SPK_CTL
    //wisky libai es8323�µ����ȶ����л� 
    //gpio_direction_output(SPK_CTL, GPIO_LOW);
    mdelay(50);
    //int state = gpio_get_value(RK30_PIN0_PC6);
    int state = gpio_get_value(HEADPHONE_DET_PIN);
    if(state!=0)
    gpio_set_value(SPK_CTL, i);
    //end
	DBG("*** %s() SPEAKER set as %d\n", __FUNCTION__, i);
    #endif
    #ifdef EAR_CON_PIN
    //gpio_direction_output(EAR_CON_PIN, GPIO_LOW);
    gpio_set_value(EAR_CON_PIN, i);
    DBG("*** %s() HEADPHONE set as %d\n", __FUNCTION__, i);
    mdelay(50);
    #endif
}

void codec_set_spk(bool on)
{
	on_off_ext_amp(on);
}
/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8323_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);

    DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
		
	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		es8323->sysclk_constraints = &constraints_112896;
		es8323->sysclk = freq;
		return 0;

	case 12288000:
	case 16934400:
	case 24576000:
	case 33868800:
		es8323->sysclk_constraints = &constraints_12288;
		es8323->sysclk = freq;
		return 0;

	case 12000000:
	case 24000000:
		es8323->sysclk_constraints = &constraints_12;
		es8323->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int es8323_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
  struct snd_soc_codec *codec = codec_dai->codec;
    u8 iface = 0;
    u8 adciface = 0;
    u8 daciface = 0;
    alsa_dbg("%s----%d, fmt[%02x]\n",__FUNCTION__,__LINE__,fmt);

    iface    = snd_soc_read(codec, ES8323_IFACE);
    adciface = snd_soc_read(codec, ES8323_ADC_IFACE);
    daciface = snd_soc_read(codec, ES8323_DAC_IFACE);

    /* set master/slave audio interface */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBM_CFM:    // MASTER MODE
        	  alsa_dbg("es8323 in master mode");
            iface |= 0x80;
            break;
        case SND_SOC_DAIFMT_CBS_CFS:    // SLAVE MODE
        	  alsa_dbg("es8323 in slave mode");
            iface &= 0x7F;
            break;
        default:
            return -EINVAL;
    }


    /* interface format */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
        case SND_SOC_DAIFMT_I2S:
            adciface &= 0xFC;
            //daciface &= 0xF9;  //updated by david-everest,5-25           
            daciface &= 0xF9;
            break;
        case SND_SOC_DAIFMT_RIGHT_J:
            break;
        case SND_SOC_DAIFMT_LEFT_J:
            break;
        case SND_SOC_DAIFMT_DSP_A:
            break;
        case SND_SOC_DAIFMT_DSP_B:
            break;
        default:
            return -EINVAL;
    }

    /* clock inversion */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
        case SND_SOC_DAIFMT_NB_NF:
            iface    &= 0xDF;
            adciface &= 0xDF;
            //daciface &= 0xDF;    //UPDATED BY david-everest,5-25        
            daciface &= 0xBF;
            break;
        case SND_SOC_DAIFMT_IB_IF:
            iface    |= 0x20;
            //adciface &= 0xDF;    //UPDATED BY david-everest,5-25
            adciface |= 0x20;
            //daciface &= 0xDF;   //UPDATED BY david-everest,5-25
            daciface |= 0x40;
            break;
        case SND_SOC_DAIFMT_IB_NF:
            iface    |= 0x20;
           // adciface |= 0x40;  //UPDATED BY david-everest,5-25
            adciface &= 0xDF;
            //daciface |= 0x40;  //UPDATED BY david-everest,5-25
            daciface &= 0xBF;
            break;
        case SND_SOC_DAIFMT_NB_IF:
            iface    &= 0xDF;
            adciface |= 0x20;
            //daciface |= 0x20;  //UPDATED BY david-everest,5-25
            daciface |= 0x40;
            break;
        default:
            return -EINVAL;
    }

    snd_soc_write(codec, ES8323_IFACE    , iface);
    snd_soc_write(codec, ES8323_ADC_IFACE, adciface);
    snd_soc_write(codec, ES8323_DAC_IFACE, daciface);

    return 0;
}

static int es8323_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);
  u16 i;
	if (!es8323->is_startup) {
		es8323->is_startup = 1;
		//on_off_ext_amp(0);
    /*
		snd_soc_write(codec, ES8323_CONTROL1, 0x06);		
		snd_soc_write(codec, ES8323_CONTROL2, 0x72);	
		snd_soc_write(codec, ES8323_DACPOWER, 0x00);		
		mdelay(30);
	  //snd_soc_write(codec, ES8323_CHIPPOWER, 0xf3);
		snd_soc_write(codec, ES8323_DACCONTROL21, 0x80);	
		*/
		snd_soc_write(codec, ES8323_ADCPOWER, 0x00);		
	 	snd_soc_write(codec, ES8323_DACPOWER, 0x3c);	
		snd_soc_write(codec, ES8323_CHIPPOWER, 0x00);
		//on_off_ext_amp(1);
	}

	DBG("Enter::%s----%d  es8323->sysclk=%d\n",__FUNCTION__,__LINE__,es8323->sysclk); 

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	if (!es8323->sysclk) {
		dev_err(codec->dev,
			"No MCLK configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   es8323->sysclk_constraints);

	return 0;
}

static int es8323_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);
	//u16 iface = snd_soc_read(codec, 8323_IFACE) & 0x1f3;
	//u16 srate = snd_soc_read(codec, 8323_SRATE) & 0x180;
	
	u16 srate    = snd_soc_read(codec, ES8323_IFACE) & 0x80;
	u16 adciface = snd_soc_read(codec, ES8323_ADC_IFACE) & 0xE3;
	u16 daciface = snd_soc_read(codec, ES8323_DAC_IFACE) & 0xC7;
	
	int coeff;

	coeff = get_coeff(es8323->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es8323->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8323->sysclk);
		return coeff;
	}

	/* bit size */
 switch (params_format(params)) {
  case SNDRV_PCM_FORMAT_S16_LE:
      adciface |= 0x000C;
      daciface |= 0x0018;
      break;
  case SNDRV_PCM_FORMAT_S20_3LE:
      adciface |= 0x0004;
      daciface |= 0x0008;
      break;
  case SNDRV_PCM_FORMAT_S24_LE:
      break;
  case SNDRV_PCM_FORMAT_S32_LE:
      adciface |= 0x0010;
      daciface |= 0x0020;
      break;
  }

  /* set iface & srate*/
  snd_soc_write(codec, ES8323_DAC_IFACE, daciface); //dac bits length
  snd_soc_write(codec, ES8323_ADC_IFACE, adciface); //adc bits length

	if (coeff >= 0)
		{
		 snd_soc_write(codec, ES8323_IFACE, srate);  //bclk div,mclkdiv2
		 snd_soc_write(codec, ES8323_ADCCONTROL5, coeff_div[coeff].sr | (coeff_div[coeff].usb) << 4);
		 snd_soc_write(codec, ES8323_DACCONTROL2, coeff_div[coeff].sr | (coeff_div[coeff].usb) << 4);
		}

	return 0;
}

static int es8323_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = snd_soc_read(codec, ES8323_DACCONTROL3) & 0xfb;

	DBG("Enter::%s----%d--mute=%d\n",__FUNCTION__,__LINE__,mute);

	if (mute)
		//snd_soc_write(codec, ES8323_DACCONTROL3, mute_reg | 0x4);
		snd_soc_write(codec, ES8323_DACCONTROL3,0x04);
	else
	{
		//snd_soc_write(codec, ES8323_DACCONTROL3, mute_reg);
		snd_soc_write(codec, ES8323_DACCONTROL3, 0x00);
		
	}
    on_off_ext_amp(!mute);
    
	return 0;
}

static int es8323_trigger(struct snd_pcm_substream *substream, int status, struct snd_soc_dai *dai)
{
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if(status == SNDRV_PCM_TRIGGER_START){
			//on_off_ext_amp(1);
		}
		else if(status == SNDRV_PCM_TRIGGER_STOP){
			//on_off_ext_amp(0);
		}
	}

	return 0;
}
/////////////////////////////////////////////////////////////////
static int es8323_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);
	u16 OUT_VOL = snd_soc_read(codec, ES8323_LOUT1_VOL);

	DBG("Enter::%s----%d level =%d\n",__FUNCTION__,__LINE__,level);
  u16 i;
	switch (level) {
	case SND_SOC_BIAS_ON:
		es8323->is_biason = 1;
		break;

	case SND_SOC_BIAS_PREPARE:
		if (es8323->is_startup && es8323->is_biason) {
			snd_soc_write(codec, ES8323_DACCONTROL3, 0x04);//0xe6);
			snd_soc_write(codec, ES8323_DACPOWER, 0xf0);
			msleep(10);
			snd_soc_write(codec, ES8323_DACPOWER, 0xc0);
			snd_soc_write(codec, ES8323_DACCONTROL3, 0x00);//0xe2);
			es8323->is_startup = 0;
			es8323->is_biason = 0;
		}
		/* VREF, VMID=2x50k, digital enabled */
		snd_soc_write(codec, ES8323_CONTROL1, 0x06);
		snd_soc_write(codec, ES8323_CHIPPOWER, 0x00);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* VREF, VMID=2x5k */
			//snd_soc_write(codec, ES8323_CONTROL1, 0x06);

			/* Charge caps */
			msleep(100);
		}

		/* VREF, VMID=2*500k, digital stopped */
		snd_soc_write(codec, ES8323_CHIPPOWER, 0x00);
		
		
		break;

	case SND_SOC_BIAS_OFF:
				
		//snd_soc_write(codec, ES8323_CONTROL1, 0x02);
		snd_soc_write(codec, ES8323_CHIPPOWER, 0xf3);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define ES8323_RATES SNDRV_PCM_RATE_8000_96000

#define ES8323_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops es8323_ops = {
	.startup = es8323_pcm_startup,
	.hw_params = es8323_pcm_hw_params,
	.set_fmt = es8323_set_dai_fmt,
	.set_sysclk = es8323_set_dai_sysclk,
	.digital_mute = es8323_mute,
//	.trigger = es8323_trigger,
};

static struct snd_soc_dai_driver es8323_dai = {
	.name = "ES8323 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ES8323_RATES,
		.formats = ES8323_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ES8323_RATES,
		.formats = ES8323_FORMATS,
	 },
	.ops = &es8323_ops,
	.symmetric_rates = 1,
};

static int es8323_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	u16 i;
	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

  snd_soc_write(codec, ES8323_DACCONTROL3, 0x04);//0xe6);  
  snd_soc_write(codec, ES8323_ADCPOWER, 0x39);  //adc pdn
  snd_soc_write(codec, ES8323_DACPOWER, 0xf0);  //dac pdn
	es8323_set_bias_level(codec, SND_SOC_BIAS_OFF);
	
	return 0;
}

static int es8323_resume(struct snd_soc_codec *codec)
{
	u16 i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);

	es8323_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
  snd_soc_write(codec, ES8323_ADCPOWER, 0x00);  //adc pdn
  snd_soc_write(codec, ES8323_DACPOWER, 0x3c);  //dac pdn	
  snd_soc_write(codec, ES8323_DACCONTROL3, 0x00);//0xe2);
	return 0;
}

static u32 cur_reg=0;
static struct snd_soc_codec *es8323_codec;
static int entry_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;

	snd_soc_write(es8323_codec, ES8323_ADCPOWER, 0xff);
	snd_soc_write(es8323_codec, ES8323_DACPOWER, 0xf0);
	snd_soc_write(es8323_codec, ES8323_DACPOWER, 0xc0);
	snd_soc_write(es8323_codec, ES8323_CHIPPOWER, 0xf3);

	len = sprintf(page, "es8323 suspend...\n");

	return len ;
}


static int es8323_probe(struct snd_soc_codec *codec)
{
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	u16 reg,i;
    printk("%s\n", __func__);
    
	if (codec == NULL) {
		dev_err(codec->dev, "Codec device not registered\n");
		return -ENODEV;
	}
    codec->read  = es8323_read_reg_cache;
    codec->write = es8323_write;
    codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = container_of(codec->dev, struct i2c_client, dev);

	es8323_codec = codec;
	ret = es8323_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		return ret;
	}
	
  es8323_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
////////////////////////////////////////////////////
#if 1	
  snd_soc_write(codec, 0x35, 0xA0);
  snd_soc_write(codec, 0x36, 0x08);
  snd_soc_write(codec, 0x01, 0x58);
  snd_soc_write(codec, 0x01, 0x40);
  snd_soc_write(codec, 0x02, 0xF3);
  snd_soc_write(codec, 0x02, 0xF0);
  snd_soc_write(codec, 0x2B, 0x80);
  snd_soc_write(codec, 0x00, 0x06);
  snd_soc_write(codec, 0x08, 0x00);
  snd_soc_write(codec, ES8323_DACPOWER, 0x00);	
  	
  snd_soc_write(codec, ES8323_ADCCONTROL1, 0x88);  //wisky libai 0xaa ��С¼������  // MIC PGA gain: +24dB
  snd_soc_write(codec, ES8323_ADCCONTROL2, 0xf0);     // LINSEL(L-R differential), RINGSEL(L-R differential)
  snd_soc_write(codec, ES8323_ADCCONTROL3, 0x82);     // Input Select: LIN2/RIN2
  snd_soc_write(codec, ES8323_ADCCONTROL4, 0x4C);     // Left data = left ADC, right data = right ADC, 24 bits I2S
  
  snd_soc_write(codec, ES8323_LADC_VOL, 0x00);        // 0dB
  snd_soc_write(codec, ES8323_RADC_VOL, 0x00);        // 0dB
  
  snd_soc_write(codec, ES8323_ADCCONTROL10, 0xea);    // ALC stereo, Max gain(17.5dB), Min gain(0dB),updated by david-everest,5-25
  snd_soc_write(codec, ES8323_ADCCONTROL11, 0xa0);    // ALCLVL(-1.5dB), ALCHLD(0ms)
  snd_soc_write(codec, ES8323_ADCCONTROL12, 0x05);    // ALCDCY(1.64ms/363us), ALCATK(1664us/363.2us)
  snd_soc_write(codec, ES8323_ADCCONTROL13, 0x06);    // ALCMODE(ALC mode), ALCZC(disable), TIME_OUT(disable), WIN_SIZE(96 samples)
  snd_soc_write(codec, ES8323_ADCCONTROL14, 0xb3);    // NGTH(XXX), NGG(mute ADC output), NGAT(enable)
  
  
  //----------------------------------------------------------------------------------------------------------------
  snd_soc_write(codec, ES8323_LDAC_VOL, 0x00);    // left DAC volume //00 max
  snd_soc_write(codec, ES8323_RDAC_VOL, 0x00);    // right DAC volume //00 max 
  
  snd_soc_write(codec, ES8323_DACCONTROL3, 0x02);// 0xE2);     // DAC unmute
 
 /* snd_soc_write(codec, 0x1E, 0x01);    //for 22uF capacitors ,6db Bass@200Hz,Fs=44100 //EQ
  snd_soc_write(codec, 0x1F, 0x64);
  snd_soc_write(codec, 0x20, 0xED);
  snd_soc_write(codec, 0x21, 0xBE);
  snd_soc_write(codec, 0x22, 0x20);
  snd_soc_write(codec, 0x23, 0x5C);
  snd_soc_write(codec, 0x24, 0x2B);
  snd_soc_write(codec, 0x25, 0x8F);
*/
  
  snd_soc_write(codec, 0x27, 0x50);
  snd_soc_write(codec, 0x2A, 0x50);
  snd_soc_write(codec, 0x02, 0x00);
  snd_soc_write(codec, 0x04, 0x0C);
  msleep(100);
	for(i = 0; i <= ES8323_DEF_VOL; i++)
	{		
		snd_soc_write(codec, ES8323_LOUT2_VOL, i);//0x1c);   //
		snd_soc_write(codec, ES8323_ROUT2_VOL, i);//0x1c);   // 
	} 
  snd_soc_write(codec, 0x03, 0x00);
  snd_soc_write(codec, 0x27, 0xB8);
  snd_soc_write(codec, 0x2A, 0xB8);
	/////////////////////////////////////////
#endif

  
#if 0	
  snd_soc_write(codec, ES8323_CHIPPOWER   , 0xf3);    // Power down: ADC DEM, DAC DSM/DEM, ADC/DAC state machine, ADC/DAC ananlog reference
  snd_soc_write(codec, ES8323_MASTERMODE  , 0x00);    // SLAVE MODE, MCLK not divide
 
  snd_soc_write(codec, ES8323_DACCONTROL21, 0x80);    // DACLRC and ADCLRC same, ADC/DAC DLL power up, Enable MCLK input from PAD.
  snd_soc_write(codec, ES8323_CONTROL1   , 0x06);     // VMIDSEL (500 kohme divider enabled)
  snd_soc_write(codec, ES8323_CONTROL2 , 0x74);   // 
  snd_soc_write(codec, ES8323_CONTROL2 , 0x72);   //   
  
  snd_soc_write(codec, ES8323_ADCPOWER, 0x00);		
	snd_soc_write(codec, ES8323_DACPOWER, 0x00);	
	snd_soc_write(codec, ES8323_CHIPPOWER, 0xf0);  
	snd_soc_write(codec, ES8323_CHIPPOWER, 0xc0);  
  //-----------------------------------------------------------------------------------------------------------------
  snd_soc_write(codec, ES8323_ADCCONTROL1, 0x88);     // MIC PGA gain: +24dB
  snd_soc_write(codec, ES8323_ADCCONTROL2, 0xf0);     // LINSEL(L-R differential), RINGSEL(L-R differential)
  snd_soc_write(codec, ES8323_ADCCONTROL3, 0x82);     // Input Select: LIN2/RIN2
  snd_soc_write(codec, ES8323_ADCCONTROL4, 0x4C);     // Left data = left ADC, right data = right ADC, 24 bits I2S
  
  snd_soc_write(codec, ES8323_LADC_VOL, 0x00);        // 0dB
  snd_soc_write(codec, ES8323_RADC_VOL, 0x00);        // 0dB
  
  snd_soc_write(codec, ES8323_ADCCONTROL10, 0xea);    // ALC stereo, Max gain(17.5dB), Min gain(0dB),updated by david-everest,5-25
  snd_soc_write(codec, ES8323_ADCCONTROL11, 0xa0);    // ALCLVL(-1.5dB), ALCHLD(0ms)
  snd_soc_write(codec, ES8323_ADCCONTROL12, 0x05);    // ALCDCY(1.64ms/363us), ALCATK(1664us/363.2us)
  snd_soc_write(codec, ES8323_ADCCONTROL13, 0x06);    // ALCMODE(ALC mode), ALCZC(disable), TIME_OUT(disable), WIN_SIZE(96 samples)
  snd_soc_write(codec, ES8323_ADCCONTROL14, 0xa3);    // NGTH(XXX), NGG(mute ADC output), NGAT(enable)
  
  
  //----------------------------------------------------------------------------------------------------------------
  snd_soc_write(codec, ES8323_LDAC_VOL, 0x00);    // left DAC volume
  snd_soc_write(codec, ES8323_RDAC_VOL, 0x00);    // right DAC volume
  
  snd_soc_write(codec, ES8323_DACCONTROL3, 0x00);// 0xE2);     // DAC unmute
  
  snd_soc_write(codec, ES8323_DACCONTROL17, 0xb8);    // left DAC to left mixer enable, 
  snd_soc_write(codec, ES8323_DACCONTROL18, 0x38);    // ???
  snd_soc_write(codec, ES8323_DACCONTROL19, 0x38);    // ???
  snd_soc_write(codec, ES8323_DACCONTROL20, 0xb8);    // right DAC to right mixer enable,

//  snd_soc_write(codec, ES8323_DACCONTROL21, 0x80); 
  snd_soc_write(codec, ES8323_CHIPPOWER   , 0x00);
  snd_soc_write(codec, ES8323_DACPOWER, 0x0c);
	for(i = 0; i <= E8323_DEF_VOL; i++)
	{		
		snd_soc_write(codec, ES8323_LOUT2_VOL, i);//0x1c);   //
		snd_soc_write(codec, ES8323_ROUT2_VOL, i);//0x1c);   // 
	}  
#endif	
	
  //8323_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
        
	snd_soc_add_controls(codec, es8323_snd_controls,
				ARRAY_SIZE(es8323_snd_controls));
	snd_soc_dapm_new_controls(dapm, es8323_dapm_widgets,
				  ARRAY_SIZE(es8323_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	create_proc_read_entry("es8323_suspend", 0644, NULL, entry_read, NULL);

	return 0;
}

static int es8323_remove(struct snd_soc_codec *codec)
{
	es8323_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_8323 = {
	.probe =	es8323_probe,
	.remove =	es8323_remove,
	.suspend =	es8323_suspend,
	.resume =	es8323_resume,
	.set_bias_level = es8323_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(es8323_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = es8323_reg,
	.read	= es8323_read_reg_cache,
	.write = es8323_write,	
};

#if defined(CONFIG_SPI_MASTER)
static int __devinit es8323_spi_probe(struct spi_device *spi)
{
	struct es8323_priv *es8323;
	int ret;

	es8323 = kzalloc(sizeof(struct es8323_priv), GFP_KERNEL);
	if (es8323 == NULL)
		return -ENOMEM;

	es8323->control_type = SND_SOC_SPI;
	spi_set_drvdata(spi, es8323);

	ret = snd_soc_register_codec(&spi->dev,
			&soc_codec_dev_8323, &es8323_dai, 1);
	if (ret < 0)
		kfree(es8323);
	return ret;
}

static int __devexit es8323_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	kfree(spi_get_drvdata(spi));
	return 0;
}

static struct spi_driver es8323_spi_driver = {
	.driver = {
		.name	= "ES8323",
		.owner	= THIS_MODULE,
	},
	.probe		= es8323_spi_probe,
	.remove		= __devexit_p(es8323_spi_remove),
};
#endif /* CONFIG_SPI_MASTER */

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static ssize_t es8323_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%s(): get 0x%04x=0x%04x\n", __FUNCTION__, cur_reg, 
		snd_soc_read(es8323_codec, cur_reg));
}

static u32 strtol(const char *nptr, int base)
{
	u32 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{

		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{


		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

static ssize_t es8323_store(struct device *dev,
					struct device_attribute *attr,
					const char *_buf, size_t _count)
{
	const char * p=_buf;
	u32 reg, val;
	
	if(!strncmp(_buf, "get", strlen("get")))
	{
		p+=strlen("get");
		cur_reg=(u32)strtol(p, 16);
		val=snd_soc_read(es8323_codec, cur_reg);
		printk("%s(): get 0x%04x=0x%04x\n", __FUNCTION__, cur_reg, val);
	}
	else if(!strncmp(_buf, "put", strlen("put")))
	{
		p+=strlen("put");
		reg=strtol(p, 16);
		p=strchr(_buf, '=');
		if(p)
		{
			++ p;
			val=strtol(p, 16);
			snd_soc_write(es8323_codec, reg, val);
			printk("%s(): set 0x%04x=0x%04x\n", __FUNCTION__, reg, val);
		}
		else
			printk("%s(): Bad string format input!\n", __FUNCTION__);
	}
	else
		printk("%s(): Bad string format input!\n", __FUNCTION__);
	
	return _count;
} 

static struct device *es8323_dev = NULL;
static struct class *es8323_class = NULL;
static DEVICE_ATTR(es8323, 0666, es8323_show, es8323_store);
static __devinit int es8323_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct es8323_priv *es8323;
	int ret;

	es8323 = kzalloc(sizeof(struct es8323_priv), GFP_KERNEL);
	if (es8323 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8323);
	es8323->control_type = SND_SOC_I2C;

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_8323, &es8323_dai, 1);
	if (ret < 0) {
		kfree(es8323);
		return ret;
	}
	es8323_class = class_create(THIS_MODULE, "es8323");
	if (IS_ERR(es8323_class)) 
	{
		printk("Create class audio_es8323.\n");
		return -ENOMEM;
	}
	es8323_dev = device_create(es8323_class, NULL, MKDEV(0, 1), NULL, "dev");
	device_create_file(es8323_dev, &dev_attr_es8323);

	return ret;
}

static __devexit int es8323_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id es8323_i2c_id[] = {
	{ "es8323", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8323_i2c_id);

void es8323_i2c_shutdown(struct i2c_client *client)
{
    printk("Chenzy----%s\n", __func__);
    on_off_ext_amp(0);
   // es8323_set_bias_level(es8323_codec, SND_SOC_BIAS_OFF);

//#define SPK_CTL  RK29_PIN6_PB6
//    #ifdef SPK_CTL
//    //gpio_direction_output(SPK_CTL, GPIO_LOW);
//    gpio_set_value(SPK_CTL, i);
//	DBG("*** %s() SPEAKER set as %d\n", __FUNCTION__, i);
//    #endif
//#undef SPK_CTL
    snd_soc_write(es8323_codec, 0x08,0x00); 
    snd_soc_write(es8323_codec, 0X19, 0XE6); 
    //mdelay(20);

    snd_soc_write(es8323_codec, 0x04,0xc0); 
    {	
        snd_soc_write(es8323_codec, 0x2e,0);
        snd_soc_write(es8323_codec, 0x2f,0);
        snd_soc_write(es8323_codec, 0x30,0);
        snd_soc_write(es8323_codec, 0x31,0);		
        //mdelay(2);
    }

    snd_soc_write(es8323_codec, 0x02,0xc3);  
   
    mdelay(100);
}

static struct i2c_driver es8323_i2c_driver = {
	.driver = {
		.name = "ES8323",
		.owner = THIS_MODULE,
	},
	.probe =    es8323_i2c_probe,
	.remove =   __devexit_p(es8323_i2c_remove),
	.shutdown = es8323_i2c_shutdown,
	.id_table = es8323_i2c_id,
};
#endif

static int __init es8323_modinit(void)
{
//leaf 2012-7-26	if(0 == tcsi_get_value(TCSI_CODEC_ES8323))
//leaf 2012-7-26		return;
#if 1
    //extern int get_sound_card_exist() ;
    //extern void set_sound_card_exist(int i) ;
    //extern int i2c0_prober_verify(u32 dev_addr, u16 reg, u32 reg_addr_len, u32 reg_val_len, u32 id);
    //typedef void (*fp_codec_set_spk)(bool on);
    //extern void set_codec_set_spk(fp_codec_set_spk fp);
    //if(get_sound_card_exist()) {
    //    printk("%s():Sound card already exist!\n", __FUNCTION__);
    //    return -ENODEV;
    //}
    //if(i2c0_prober_verify(0x10, 0x35, 1, 1, 0x0000) != 0) {
    //    printk("%s(): Ping error with 0x1a\n", __FUNCTION__);
    //    return -ENODEV;
    //}
    //else
    //    printk("%s(): Ping OK with 0x1a\n", __FUNCTION__);
    //set_sound_card_exist(0x8388);
    //set_codec_set_spk(8323_codec_set_spk);
#endif
	return i2c_add_driver(&es8323_i2c_driver);
}
module_init(es8323_modinit);

static void __exit es8323_exit(void)
{

//leaf	if(0 == tcsi_get_value(TCSI_CODEC_ES8323))
//leaf		return;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&es8323_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&es8323_spi_driver);
#endif
}
module_exit(es8323_exit);


MODULE_DESCRIPTION("ASoC es8323 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");

