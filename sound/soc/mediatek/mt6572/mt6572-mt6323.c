// SPDX-License-Identifier: GPL-2.0
//
// MediaTek MT6572 + MT6323 sound card.
//
// Machine driver binding the MT6572 AFE platform (DL1 playback) to the MT6323
// PMIC analog codec, with headphone-jack detection that auto-routes between the
// speaker and the headphones, and an optional external speaker amplifier.
// Modelled on the mt8183-mt6358 PMIC-codec card.

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <sound/jack.h>
#include <sound/soc.h>

static const struct snd_soc_dapm_widget mt6572_mt6323_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* The speaker amplifier is fed from the codec headphone output stage. */
static const struct snd_soc_dapm_route mt6572_mt6323_routes[] = {
	{ "INL", NULL, "HP Driver" },
	{ "INR", NULL, "HP Driver" },
	{ "Speaker", NULL, "OUTL" },
	{ "Speaker", NULL, "OUTR" },
};

SND_SOC_DAILINK_DEFS(playback,
	DAILINK_COMP_ARRAY(COMP_CPU("mt6572-afe-dl1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("mt6323-sound", "mt6323-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mt6572_mt6323_dai_links[] = {
	{
		.name = "DL1",
		.stream_name = "DL1 Playback",
		SND_SOC_DAILINK_REG(playback),
	},
};

static struct snd_soc_jack mt6572_mt6323_hp_jack;

static struct snd_soc_jack_pin mt6572_mt6323_jack_pins[] = {
	{ .pin = "Headphone", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
};

static struct snd_soc_jack_gpio mt6572_mt6323_jack_gpio = {
	.name = "hp-det",
	.report = SND_JACK_HEADPHONE,
	/*
	 * Stock detects the jack through the PMIC ACCDET block rather than a
	 * GPIO, but debounces that accessory-detect EINT by 256 ms (MTK BSP,
	 * CUST_EINT_ACCDET_DEBOUNCE_CN) to settle 3.5mm contact bounce on
	 * insert/removal; mirror that here for the plug-detect GPIO. Units: ms.
	 */
	.debounce_time = 256,
};

static void mt6572_mt6323_jack_free(void *jack)
{
	snd_soc_jack_free_gpios(jack, 1, &mt6572_mt6323_jack_gpio);
}

/*
 * Optional headphone-jack plug detection ("hp-det-gpios"): insert routes audio
 * to the headphones and powers down the speaker amp; removal does the reverse.
 */
static int mt6572_mt6323_late_probe(struct snd_soc_card *card)
{
	int ret;

	if (!device_property_present(card->dev, "hp-det-gpios"))
		return 0;

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack", SND_JACK_HEADPHONE,
					 &mt6572_mt6323_hp_jack,
					 mt6572_mt6323_jack_pins,
					 ARRAY_SIZE(mt6572_mt6323_jack_pins));
	if (ret)
		return ret;

	mt6572_mt6323_jack_gpio.gpiod_dev = card->dev;
	ret = snd_soc_jack_add_gpios(&mt6572_mt6323_hp_jack, 1,
				     &mt6572_mt6323_jack_gpio);
	if (ret)
		return ret;

	return devm_add_action_or_reset(card->dev, mt6572_mt6323_jack_free,
					&mt6572_mt6323_hp_jack);
}

static struct snd_soc_aux_dev mt6572_mt6323_speaker_amp;

static struct snd_soc_card mt6572_mt6323_card = {
	.name = "mt6572-mt6323",
	.owner = THIS_MODULE,
	.dai_link = mt6572_mt6323_dai_links,
	.num_links = ARRAY_SIZE(mt6572_mt6323_dai_links),
	.dapm_widgets = mt6572_mt6323_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6572_mt6323_widgets),
	.dapm_routes = mt6572_mt6323_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6572_mt6323_routes),
	.late_probe = mt6572_mt6323_late_probe,
};

static int mt6572_mt6323_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6572_mt6323_card;
	struct device_node *platform_node;
	struct snd_soc_dai_link *dai_link;
	int i, ret;

	card->dev = &pdev->dev;

	platform_node = of_parse_phandle(pdev->dev.of_node, "mediatek,platform", 0);
	if (!platform_node)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing mediatek,platform\n");

	/* The DL1 CPU DAI and the PCM platform both live on the AFE node. */
	for_each_card_prelinks(card, i, dai_link) {
		dai_link->cpus->of_node = platform_node;
		dai_link->platforms->of_node = platform_node;
	}

	/* Optional external speaker amplifier (simple-audio-amplifier aux dev). */
	if (!mt6572_mt6323_speaker_amp.dlc.of_node) {
		mt6572_mt6323_speaker_amp.dlc.of_node =
			of_parse_phandle(pdev->dev.of_node, "mediatek,speaker-amp", 0);
		if (mt6572_mt6323_speaker_amp.dlc.of_node) {
			card->aux_dev = &mt6572_mt6323_speaker_amp;
			card->num_aux_devs = 1;
		}
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	of_node_put(platform_node);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register sound card\n");

	return 0;
}

static const struct of_device_id mt6572_mt6323_dt_match[] = {
	{ .compatible = "mediatek,mt6572-mt6323-sound" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6572_mt6323_dt_match);

static struct platform_driver mt6572_mt6323_driver = {
	.driver = {
		.name = "mt6572-mt6323",
		.of_match_table = mt6572_mt6323_dt_match,
	},
	.probe = mt6572_mt6323_dev_probe,
};
module_platform_driver(mt6572_mt6323_driver);

MODULE_DESCRIPTION("MediaTek MT6572 MT6323 sound card");
MODULE_LICENSE("GPL");
