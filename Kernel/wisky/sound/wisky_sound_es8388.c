/*
 * rk29_es8388.c  --  SoC audio for rockchip
 *
 * Driver for rockchip es8388 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/rk29_iomap.h>
#include "wisky_codec_es8388.h"
#include "../../sound/soc/rk29/rk29_pcm.h"
#include "../../sound/soc/rk29/rk29_i2s.h"
#include <mach/gpio.h>
#include <linux/clk.h>

static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
        unsigned int pll_out = 0; 
        unsigned int lrclk = 0;
		int div_bclk,div_mclk;
        int ret;
		struct clk	*general_pll;
          
        WPRINTK("Enter::%s----%d\n",__FUNCTION__,__LINE__);    
        /*by Vincent Hsiung for EQ Vol Change*/
        #define HW_PARAMS_FLAG_EQVOL_ON 0x21
        #define HW_PARAMS_FLAG_EQVOL_OFF 0x22
        if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
        {
        	ret = codec_dai->ops->hw_params(substream, params, codec_dai); //by Vincent
        	WPRINTK("Enter::%s----%d\n",__FUNCTION__,__LINE__);
        }
        else
        {
                
                /* set codec DAI configuration */
                #if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
                ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
                #endif	
                #if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
                ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
                #endif
                if (ret < 0)
                  return ret; 

                /* set cpu DAI configuration */
                #if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
                ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
                #endif	
                #if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
                ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
                #endif		
                if (ret < 0)
                  return ret;

        }

        return 0;

}

static const struct snd_soc_dapm_widget rk29_es8388_dapm_widgets[] = {
    SND_SOC_DAPM_HP("Headphone Jack", NULL),
    SND_SOC_DAPM_MIC("Mic Jack", NULL),
    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_LINE("Line Jack", NULL),
    SND_SOC_DAPM_HP("Headset Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[]= {

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic Jack"},

	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "LOUT1"},
    {"Headset Jack", NULL, "ROUT1"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LOUT1"},
	{"Headphone Jack", NULL, "ROUT1"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT1"},
	{"Ext Spk", NULL, "LOUT1"},

};

static const char *jack_function[] = {"HeadPhone", "Speaker"};
static const char *spk_function[]  = {"On", "Off"};

static const struct soc_enum rk29_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new es8388_rk29_controls[] = {
	SOC_ENUM_EXT("Jack Function"   , rk29_enum[0], NULL, NULL),
	SOC_ENUM_EXT("Speaker Function", rk29_enum[1], NULL , NULL),
};

/*
 * Logic for a es8388 as connected on a rockchip board.
 */
static int es8388_init(struct snd_soc_codec *codec)
{
	int i, err;
	int ret=0;
	WPRINTK("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	struct snd_soc_dai *codec_dai=&codec->dai[0];
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
		/*12000000*/11289600, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set ES8388 SYSCLK: %d\n", ret);
		return ret;
	}
		
	snd_soc_dapm_enable_pin(codec, "MICIN");

	/* Add tcc specific controls */
	for (i = 0; i < ARRAY_SIZE(es8388_rk29_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&es8388_rk29_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	/* Add tcc specific widgets */
	snd_soc_dapm_new_controls(codec, rk29_es8388_dapm_widgets,
				  ARRAY_SIZE(rk29_es8388_dapm_widgets));

	/* Set up Telechips specific audio path telechips audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	WPRINTK("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	return 0;
}

static struct snd_soc_ops rk29_ops = {
	  .hw_params = rk29_hw_params,
};

static struct snd_soc_dai_link rk29_dai = {
		.name = "ES8388",
		.stream_name = "ES8388",
		.cpu_dai = &rk29_i2s_dai,
		.codec_dai = &es8388_dai,
		.init = es8388_init,
		.ops = &rk29_ops,
};

static struct snd_soc_card snd_soc_card_rk29 = {
	  .name = "RK29_ES8388",
	  .platform = &rk29_soc_platform,
	  .dai_link = &rk29_dai,
	  .num_links = 1,
};


static struct snd_soc_device rk29_snd_devdata = {
	  .card = &snd_soc_card_rk29,
	  .codec_dev = &soc_codec_dev_es8388,
};

static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;	
        WPRINTK("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		  WPRINTK("platform device allocation failed\n");
		  ret = -ENOMEM;
		  return ret;
	}
	platform_set_drvdata(rk29_snd_device, &rk29_snd_devdata);
	rk29_snd_devdata.dev = &rk29_snd_device->dev;
	ret = platform_device_add(rk29_snd_device);
	if (ret) {
	        WPRINTK("platform device add failed\n");
	        platform_device_put(rk29_snd_device);
	}
	return ret;
}

static void __exit audio_card_exit(void)
{
	platform_device_unregister(rk29_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");
