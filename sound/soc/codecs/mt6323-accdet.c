// SPDX-License-Identifier: GPL-2.0

/* mt6323-accdet.c -- ALSA SoC mt6323 accdet driver
 *
 * Copyright (c) 2026 Roman Vivchar <rva333@protonmail.com>
 */

#include "linux/interrupt.h"
#include "linux/irqreturn.h"
#include "linux/jiffies.h"
#include <linux/of.h>
#include <linux/bitfield.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/sched/clock.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6323/registers.h>

#include "mt6323-accdet.h"

#define REGISTER_VAL(x)	((x) - 1)

#define TOP_CKPDN1_ACCDET		BIT(9)

#define TOP_RST_CON_ACCDET		BIT(4)

#define INT_CON1_ACCDET			BIT(2)

#define ACCDET_CON0_1V9_MODE_OFF	0x1a10
#define ACCDET_CON0_1V9_MODE_ON		0x1e10

#define ACCDET_CON2_SWCTRL		GENMASK(2, 0)

#define ACCDET_CON3_PWM_WIDTH		REGISTER_VAL(0x500)

#define ACCDET_CON4_PWM_THRESHOLD	REGISTER_VAL(0x400)

#define ACCDET_CON5_FALL_DELAY		GENMASK(16, 15)
#define ACCDET_CON5_FALL_DELAY_DEFAULT	1
#define ACCDET_CON5_RISE_DELAY		GENMASK(14, 0)
#define ACCDET_CON5_RISE_DELAY_DEFAULT	0x03f0

#define ACCDET_CON6_DEBOUNCE_DEFAULT	0x3000
#define ACCDET_CON7_DEBOUNCE_DEFAULT	0x3000
#define ACCDET_CON9_DEBOUNCE_DEFAULT	0x0020

#define ACCDET_CON11_IRQ_CLR		BIT(8)
#define ACCDET_CON11_IRQ_STA		BIT(0)

#define ACCDET_CON13_STATE_MASK		GENMASK(7, 6)

#define MT6323_ACCDET_JACK_MASK (SND_JACK_HEADPHONE | \
				SND_JACK_HEADSET | \
				SND_JACK_BTN_0 | \
				SND_JACK_BTN_1 | \
				SND_JACK_BTN_2)

static const struct snd_soc_component_driver mt6323_accdet_soc_driver;

enum mt6323_accdet_jack_type {
	ACCDET_HEADPHONE,
	ACCDET_HEADSET,
	ACCDET_NO_DEVICE = 3
};

struct mt6323_accdet {
	struct snd_soc_jack *jack;
	struct device *dev;
	struct regmap *regmap;
	/* power status lock */
	struct mutex lock;
	struct delayed_work work;

	unsigned long debounce_delay;

	int jack_type;
	int btn_type;
	bool plugged;
};

static int mt6323_accdet_get_jack_type(struct mt6323_accdet *accdet,
                                       enum mt6323_accdet_jack_type *jack) {
	int ret;
	u32 val;

	dev_err(accdet->dev, "%s()\n", __func__);

	ret = regmap_read(accdet->regmap, MT6323_ACCDET_CON13, &val);
	if (ret)
		return ret;

	*jack = FIELD_GET(ACCDET_CON13_STATE_MASK, val);
	return 0;
}

static void mt6323_accdet_jack_report(struct mt6323_accdet *accdet)
{
	if (!accdet->jack)
		return;

	dev_err(accdet->dev, "%s()\n", __func__);

	snd_soc_jack_report(accdet->jack, accdet->jack_type | accdet->btn_type,
	                    MT6323_ACCDET_JACK_MASK);
}

int mt6323_accdet_enable_jack_detect(struct snd_soc_component *component,
				     struct snd_soc_jack *jack)
{
	struct mt6323_accdet *accdet =
		snd_soc_component_get_drvdata(component);

	dev_err(accdet->dev, "%s()\n", __func__);

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEDOWN);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);

	accdet->jack = jack;

	mt6323_accdet_jack_report(accdet);

	return 0;
}
EXPORT_SYMBOL_GPL(mt6323_accdet_enable_jack_detect);

static int mt6323_accdet_enable(struct mt6323_accdet *accdet) {
	struct regmap *map = accdet->regmap;
	int ret;

	dev_err(accdet->dev, "%s()\n", __func__);

	/* ungate clock */
	ret = regmap_write(map, MT6323_TOP_CKPDN1_CLR, TOP_CKPDN1_ACCDET);
	if (ret)
		return ret;

	/* enable software control */
	ret = regmap_write(map, MT6323_ACCDET_CON2, ACCDET_CON2_SWCTRL);
	if (ret)
		return ret;

	/* enable accdet */
	return regmap_write(map, MT6323_ACCDET_CON1, 0);
}

static int mt6323_accdet_disable(struct mt6323_accdet *accdet) {
	struct regmap *map = accdet->regmap;
	int ret;

	dev_err(accdet->dev, "%s()\n", __func__);

	/* disable accdet */
	ret = regmap_write(map, MT6323_ACCDET_CON1, 0);
	if (ret)
		return ret;

	/* disable software control */
	ret = regmap_write(map, MT6323_ACCDET_CON2, 0);
	if (ret)
		return ret;

	/* gate clock */
	return regmap_write(map, MT6323_TOP_CKPDN1_SET, TOP_CKPDN1_ACCDET);
}

static int mt6323_accdet_init(struct mt6323_accdet *accdet) {
	struct regmap *map = accdet->regmap;
	int ret;

	dev_err(accdet->dev, "%s()\n", __func__);

	ret = regmap_write(map, MT6323_ACCDET_CON0, ACCDET_CON0_1V9_MODE_OFF);
	if (ret)
		return ret;

	/* ungate clock */
	ret = regmap_write(map, MT6323_TOP_CKPDN1_CLR, TOP_CKPDN1_ACCDET);
	if (ret)
		return ret;

	/* reset */
	ret = regmap_write(map, MT6323_TOP_RST_CON_SET, TOP_RST_CON_ACCDET);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_TOP_RST_CON_CLR, TOP_RST_CON_ACCDET);
	if (ret)
		return ret;

	/* setup pwm */
	ret = regmap_write(map, MT6323_ACCDET_CON3, ACCDET_CON3_PWM_WIDTH);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_ACCDET_CON4, ACCDET_CON4_PWM_THRESHOLD);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_ACCDET_CON2, ACCDET_CON2_SWCTRL);
	if (ret)
		return ret;

	/* setup delay */
	ret = regmap_set_bits(map, MT6323_ACCDET_CON5,
	                      FIELD_PREP(ACCDET_CON5_FALL_DELAY,
	                                 ACCDET_CON5_FALL_DELAY_DEFAULT));
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6323_ACCDET_CON5,
	                      FIELD_PREP(ACCDET_CON5_RISE_DELAY,
	                                 ACCDET_CON5_RISE_DELAY_DEFAULT));
	if (ret)
		return ret;

	/* setup debounce time */
	ret = regmap_write(map, MT6323_ACCDET_CON6, ACCDET_CON6_DEBOUNCE_DEFAULT);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_ACCDET_CON7, ACCDET_CON7_DEBOUNCE_DEFAULT);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_ACCDET_CON9, ACCDET_CON9_DEBOUNCE_DEFAULT);
	if (ret)
		return ret;

	/* clear irq if any */
	ret = regmap_clear_bits(map, MT6323_ACCDET_CON11, ACCDET_CON11_IRQ_CLR);
	if (ret)
		return ret;

	/* enable interrupt (?) */
	ret = regmap_write(map, MT6323_INT_CON1_SET, INT_CON1_ACCDET);
	if (ret)
		return ret;

	/* we have ACCDET_EINT defined */
	ret = mt6323_accdet_disable(accdet);
	if (ret)
		return ret;

	ret = regmap_write(map, MT6323_ACCDET_CON2, 0);
	if (ret)
		return ret;

	return 0;
}

/* PMIC interrupt handles key and unplug events */
static irqreturn_t mt6323_accdet_pmic_irq(int irq, void *data)
{
	struct mt6323_accdet *accdet = data;

	guard(mutex)(&accdet->lock);

	dev_err(accdet->dev, "%s()\n", __func__);

	if (accdet->plugged) {
		regmap_clear_bits(accdet->regmap, MT6323_ACCDET_CON11,
		                  ACCDET_CON11_IRQ_CLR);

		schedule_delayed_work(&accdet->work, accdet->debounce_delay);
	}

	return IRQ_HANDLED;
}

/* EINT interrupt handles plug and unplug events */
static irqreturn_t mt6323_accdet_eint_irq(int irq, void *data)
{
	struct mt6323_accdet *accdet = data;
	struct device *dev = accdet->dev;

	mutex_lock(&accdet->lock);

	if (mt6323_accdet_enable(accdet)) {
		dev_err(dev, "failed to enable accdet\n");
		goto out;
	}
	
	if (!accdet->plugged) {
		/* plug in */
		accdet->plugged = true;

		schedule_delayed_work(&accdet->work, accdet->debounce_delay);

	} else {
		/* plug out */
		accdet->plugged = false;

		/*
		 * we need to unlock mutex to prevent deadlock, because
		 * mt6323_accdet_work_func locks it too
		 */
		mutex_unlock(&accdet->lock);

		cancel_delayed_work_sync(&accdet->work);

		/* ...and lock again... */
		mutex_lock(&accdet->lock);

		accdet->jack_type = 0;
		accdet->btn_type = 0;
		mt6323_accdet_jack_report(accdet);

		if (mt6323_accdet_disable(accdet))
			dev_err(dev, "failed to disable accdet");
	}

out:
	mutex_unlock(&accdet->lock);
	return IRQ_HANDLED;
}

static void mt6323_accdet_work_func(struct work_struct *work)
{
	struct mt6323_accdet *accdet = container_of(work, struct mt6323_accdet, work.work);
	enum mt6323_accdet_jack_type jack;

	guard(mutex)(&accdet->lock);

	dev_err(accdet->dev, "%s()\n", __func__);

	if (!accdet->plugged)
		/* accdet will be disabled by eint interrupt handler */
		return;

	if (mt6323_accdet_get_jack_type(accdet, &jack)) {
		dev_err(accdet->dev, "failed to get jack type\n");
		return;
	}

	switch (jack) {
	case ACCDET_HEADPHONE:
		accdet->jack_type = SND_JACK_HEADPHONE;
		break;
	case ACCDET_HEADSET:
		accdet->jack_type = SND_JACK_HEADSET;
		break;
	/* unplug is handled by interrupt */
	default:
		break;
	}

	mt6323_accdet_jack_report(accdet);
}

static int mt6323_accdet_probe(struct platform_device *pdev)
{
	struct mt6397_chip *mt6397 = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct mt6323_accdet *accdet;
	u32 delay = 256;
	int ret, irq;

	accdet = devm_kzalloc(&pdev->dev, sizeof(*accdet), GFP_KERNEL);
	if (!accdet)
		return -ENOMEM;

	accdet->dev = dev;

	dev_err(accdet->dev, "%s()\n", __func__);
	
	accdet->regmap = mt6397->regmap;
	if (IS_ERR(accdet->regmap))
		return dev_err_probe(dev, PTR_ERR(accdet->regmap), "failed to get regmap\n");

	irq = platform_get_irq_byname(pdev, "eint");
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get eint irq\n");

	ret = devm_request_threaded_irq(&pdev->dev, irq,
					NULL, mt6323_accdet_eint_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"accdet-eint", accdet);
	if (ret)
		return ret;

	irq = platform_get_irq_byname(pdev, "pmic");
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get pmic irq\n");

	ret = devm_request_threaded_irq(&pdev->dev, irq,
					NULL, mt6323_accdet_pmic_irq,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT, "accdet-pmic", accdet);
	if (ret)
		return ret;

	/* optional for now */
	device_property_read_u32(dev, "debounce-delay-ms", &delay);
	accdet->debounce_delay = msecs_to_jiffies(delay);

	ret = devm_mutex_init(dev, &accdet->lock);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&accdet->work, mt6323_accdet_work_func);

	platform_set_drvdata(pdev, accdet);

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &mt6323_accdet_soc_driver,
					      NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component\n");
		return ret;
	}

	mt6323_accdet_init(accdet);
	mt6323_accdet_jack_report(accdet);

	return 0;
}

static const struct of_device_id mt6323_accdet_match[] = {
	{ .compatible = "mediatek,mt6323-accdet" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6323_accdet_match);

static struct platform_driver mt6323_accdet_driver = {
	.driver = {
		.name = "mt6323-accdet",
		.of_match_table = mt6323_accdet_match,
	},
	.probe = mt6323_accdet_probe,
};

module_platform_driver(mt6323_accdet_driver);

MODULE_DESCRIPTION("MT6323 ALSA SoC codec jack driver");
MODULE_LICENSE("GPL");
