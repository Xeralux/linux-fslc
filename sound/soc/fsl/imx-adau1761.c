/*
 *	Author: Ajeet Vijayvergiya <ajeet.vijay@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "../codecs/adau17x1.h"
#include "imx-audmux.h"



struct clk *cko;
struct clk *cko2_sel;
struct clk *cko2;
struct clk *osc;

static const struct snd_soc_dapm_widget imx_adau1761_widgets[] = {
/*	SND_SOC_DAPM_SPK("Line Out", NULL),
	SND_SOC_DAPM_HP("Headphone Out", NULL),*/
	SND_SOC_DAPM_OUTPUT("Mono Out"),
	SND_SOC_DAPM_MIC("Mic In", NULL),
	SND_SOC_DAPM_MIC("Line In", NULL),
};

static const struct snd_soc_dapm_route imx_adau1761_routes[] = {
/*	{ "Line Out", NULL, "LOUT" },
	{ "Line Out", NULL, "ROUT" },
	{ "Headphone Out", NULL, "LHP" },
	{ "Headphone Out", NULL, "RHP" },*/
	{ "Mono Out", NULL, "MONOOUT" },
	{ "Mic In", NULL, "MICBIAS" },
	{ "LINN", NULL, "Mic In" },
/*	{ "RINN", NULL, "Mic In" },
	{ "LAUX", NULL, "Line In" },
	{ "RAUX", NULL, "Line In" },*/
};

static int imx_adau1761_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_rate;
	int ret;

	switch (params_rate(params)) {
	case 48000:
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 96000:
		pll_rate = 48000 * 1024;
		break;
	case 44100:
	case 7350:
	case 11025:
	case 14700:
	case 22050:
	case 29400:
	case 88200:
		pll_rate = 44100 * 1024;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_pll(codec_dai, ADAU17X1_PLL,
			ADAU17X1_PLL_SRC_MCLK, 24000000, pll_rate);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, ADAU17X1_CLK_SRC_PLL, pll_rate,
			SND_SOC_CLOCK_IN);
	
	ret = snd_soc_dai_set_tdm_slot(codec_dai,0,0,0,0);

	return ret;
}

static struct snd_soc_ops imx_adau1761_ops = {
	.hw_params = imx_adau1761_hw_params,
};

static struct snd_soc_dai_link imx_adau1761_dai_link = {
	.name = "adau1761",
	.stream_name = "adau1761",
	.codec_dai_name = "adau-hifi",
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
	.ops = &imx_adau1761_ops,
};

static struct snd_soc_card imx_adau1761_card = {
	.name = "IMX ADAU1761",
	.owner = THIS_MODULE,
	.dai_link = &imx_adau1761_dai_link,
	.num_links = 1,
	.dapm_widgets = imx_adau1761_widgets,
	.num_dapm_widgets = ARRAY_SIZE(imx_adau1761_widgets),
	.dapm_routes = imx_adau1761_routes,
	.num_dapm_routes = ARRAY_SIZE(imx_adau1761_routes),
	.fully_routed = true,
};

static int imx_adau1761_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &imx_adau1761_card;
	struct device_node *of_node = pdev->dev.of_node;
        struct device_node *ssi_np, *codec_np;
        struct platform_device *ssi_pdev;
        int int_port, ext_port;
        int ret;
	struct device *dev = &pdev->dev;
	
	if (!of_node)
		return -ENXIO;
	

	ret = of_property_read_u32(of_node, "mux-int-port", &int_port);
        if (ret) {
                dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
                return ret;
        }
        ret = of_property_read_u32(of_node, "mux-ext-port", &ext_port);
        if (ret) {
                dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
                return ret;
        }
	
	int_port--;
        ext_port--;
        ret = imx_audmux_v2_configure_port(int_port,
                        IMX_AUDMUX_V2_PTCR_SYN |
                        IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
                        IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
                        IMX_AUDMUX_V2_PTCR_TFSDIR |
                        IMX_AUDMUX_V2_PTCR_TCLKDIR,
                        IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
        if (ret) {
                dev_err(&pdev->dev, "audmux internal port setup failed\n");
                return ret;
        }
        imx_audmux_v2_configure_port(ext_port,
                        IMX_AUDMUX_V2_PTCR_SYN,
                        IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
        if (ret) {
                dev_err(&pdev->dev, "audmux external port setup failed\n");
                return ret;
        }

        ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
        codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	   if (!ssi_np || !codec_np) {
                dev_err(&pdev->dev, "phandle missing or invalid\n");
                ret = -EINVAL;
                goto fail;
        }
	
	ssi_pdev = of_find_device_by_node(ssi_np);
        if (!ssi_pdev) {
                dev_err(&pdev->dev, "failed to find SSI platform device\n");
                ret = -EINVAL;
                goto fail;
        }

	card->dev = &pdev->dev;

	cko=devm_clk_get(dev,"cko");
	if (IS_ERR(cko)) {
		dev_err(dev,
			"cko missing or invalid\n");
		goto fail;
	}


	cko2_sel = devm_clk_get(dev, "cko2_sel");
	if (IS_ERR(cko2_sel)) {
		dev_err(dev,
			"cko2_sel missing or invalid\n");
		goto fail;
	}

	cko2 = devm_clk_get(dev, "cko2");
	if (IS_ERR(cko2)) {
		dev_err(dev,
			"codec clock source missing or invalid\n");
		goto fail;
	}

	osc = devm_clk_get(dev, "osc");
	if (IS_ERR(osc)) {
		dev_err(dev,
			"osc clock missing or invalid\n");
		goto fail;
	}

	ret=clk_set_parent(cko,cko2);
	if (ret)
	{
        	pr_err("cko  cko2 parent clock enable failed\n");
		goto fail;

	}
	
	ret=clk_set_parent(cko2_sel,osc);
	if (ret)
	{
        	pr_err("cko2_sel parent clock osc enable failed\n");
		goto fail;

	}
	
	ret=clk_prepare_enable(osc);
        if (ret) {
        	pr_err("osc clock enable failed\n");
		goto fail;
	}
        ret=clk_prepare_enable(cko2_sel);
        if (ret) {
        	pr_err("cko2_sel clock enable failed");
		goto fail;
	}
        ret=clk_prepare_enable(cko2);
        if (ret) {	
        	pr_err("cko2 enable failed");
		goto fail;
	}
	
	imx_adau1761_dai_link.codec_of_node = codec_np;
	imx_adau1761_dai_link.cpu_dai_name = dev_name(&ssi_pdev->dev);
	imx_adau1761_dai_link.platform_of_node = ssi_np;
	imx_adau1761_dai_link.cpu_of_node = ssi_np;

	if (!imx_adau1761_dai_link.codec_of_node ||
		!imx_adau1761_dai_link.cpu_dai_name)
		return -ENXIO;

	return snd_soc_register_card(card);
fail:
	printk(KERN_ERR "imx-adau1761 In fail\n");
	return -ENXIO;
}

static int imx_adau1761_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (cko2)
	clk_disable_unprepare(cko2);

	if (cko2_sel)
	clk_disable_unprepare(cko2_sel);

	if (osc)
	clk_disable_unprepare(osc);
	
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_adau1761_of_match[] = {
	{ .compatible = "adi,imx-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, imx_adau1761_of_match);

static struct platform_driver imx_adau1761_card_driver = {
	.driver = {
		.name = "imx-adau1761-snd",
		.owner = THIS_MODULE,
		.of_match_table = imx_adau1761_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = imx_adau1761_probe,
	.remove = imx_adau1761_remove,
};
module_platform_driver(imx_adau1761_card_driver);

MODULE_DESCRIPTION("ASoC IMX board ADAU1761 driver");
MODULE_AUTHOR("ajeet vijayvergiya <ajeet.vijay@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-adau1761-snd");
