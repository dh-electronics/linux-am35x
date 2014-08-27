/*
 * dhcm3517.c
 *
 * Copyright (C) 2009 DH electronics GmbH
 *
 * Author: Ludwig Zenz <lzenz@dh-electronics.de>
 *
 * Based on:
 *
 * am3517evm.c  -- ALSA SoC support for OMAP3517 / AM3517 EVM
 *
 * Author: Anuj Aggarwal <anuj.aggarwal@ti.com>
 *
 * Based on sound/soc/omap/beagle.c by Steve Sakoman
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic23.h"

#define CODEC_CLOCK 	12000000

static int dhcm3517_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
			CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_CLKR_SRC_CLKX, 0,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_CLKR_SRC_CLKX\n");
		return ret;
	}

	snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, 0,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_FSR_SRC_FSX\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops dhcm3517_ops = {
	.hw_params = dhcm3517_hw_params,
};

/* dhcm3517 machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic23_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LOUT"},
	{"Line Out", NULL, "ROUT"},

	{"LLINEIN", NULL, "Line In"},
	{"RLINEIN", NULL, "Line In"},

	{"MICIN", NULL, "Mic In"},
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dhcm3517_dai = {
	.name = "TLV320AIC23",
	.stream_name = "AIC23",
	.cpu_dai_name ="omap-mcbsp-dai.0",
	.codec_dai_name = "tlv320aic23-hifi",
	.platform_name = "omap-pcm-audio",
	.codec_name = "tlv320aic23-codec.1-001a",
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBM_CFM,
	.ops = &dhcm3517_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_dhcm3517 = {
	.name = "dhcm3517",
	.dai_link = &dhcm3517_dai,
	.num_links = 1,

	.dapm_widgets = tlv320aic23_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tlv320aic23_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static struct platform_device *dhcm3517_snd_device;

static int __init dhcm3517_soc_init(void)
{
	int ret;

	pr_info("OMAP3517 / AM3517 EVM SoC init\n");

	dhcm3517_snd_device = platform_device_alloc("soc-audio", -1);
	if (!dhcm3517_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dhcm3517_snd_device, &snd_soc_dhcm3517);

	ret = platform_device_add(dhcm3517_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk("Unable to add platform device\n");
	platform_device_put(dhcm3517_snd_device);

	return ret;
}

static void __exit dhcm3517_soc_exit(void)
{
	platform_device_unregister(dhcm3517_snd_device);
}

module_init(dhcm3517_soc_init);
module_exit(dhcm3517_soc_exit);

MODULE_AUTHOR("Anuj Aggarwal <anuj.aggarwal@ti.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3517 / AM3517 EVM");
MODULE_LICENSE("GPL v2");
