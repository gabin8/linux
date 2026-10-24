// SPDX-License-Identifier: GPL-2.0
/*
 * Power off through MediaTek PMIC
 *
 * Copyright (C) 2018 MediaTek Inc.
 *
 * Author: Sean Wang <sean.wang@mediatek.com>
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6397/rtc.h>
#include <linux/reboot.h>

#define MT6323_RGS_CHRDET	BIT(5)	/* CHR_CON0: valid charger detected */
#define MT6323_PWRKEY_DEB	BIT(1)	/* CHRSTATUS: clear while the power key is pressed */
#define MT6323_PWRC_POLL_MS	50

struct mt6323_pwrc {
	struct device *dev;
	struct regmap *regmap;
	u32 base;
};

static bool mt6323_pwrc_charger_present(struct mt6323_pwrc *pwrc)
{
	unsigned int val;

	return !regmap_read(pwrc->regmap, MT6323_CHR_CON0, &val) &&
	       (val & MT6323_RGS_CHRDET);
}

/*
 * A valid charger is a power-on condition of the PMIC, so with one plugged
 * in, clearing BBPU leaves the system running. Wait for the charger to go
 * away, which lets the PMIC power off, or for a press of the power key,
 * which restarts the system as a power-on would. Restart only once the key
 * is released: with USB connected, a key held while the bootloader runs
 * stalls the boot.
 */
static void mt6323_pwrc_wait_for_charger(struct mt6323_pwrc *pwrc)
{
	bool key_down = false;
	unsigned int val;

	dev_notice(pwrc->dev,
		   "charger connected: powering off when it is unplugged, press the power key to restart\n");

	while (mt6323_pwrc_charger_present(pwrc)) {
		if (regmap_read(pwrc->regmap, MT6323_CHRSTATUS, &val))
			return;

		if (!(val & MT6323_PWRKEY_DEB))
			key_down = true;
		else if (key_down)
			machine_restart(NULL);

		mdelay(MT6323_PWRC_POLL_MS);
	}

	/* Give the PMIC time to power off once the charger is gone */
	mdelay(1000);
}

static int mt6323_do_pwroff(struct sys_off_data *data)
{
	struct mt6323_pwrc *pwrc = data->cb_data;
	unsigned int val;
	int ret;

	regmap_write(pwrc->regmap, pwrc->base + RTC_BBPU, RTC_BBPU_KEY);
	regmap_write(pwrc->regmap, pwrc->base + RTC_WRTGR_MT6323, 1);

	ret = regmap_read_poll_timeout(pwrc->regmap,
					pwrc->base + RTC_BBPU, val,
					!(val & RTC_BBPU_CBUSY),
					MTK_RTC_POLL_DELAY_US,
					MTK_RTC_POLL_TIMEOUT);
	if (ret)
		dev_err(pwrc->dev, "failed to write BBPU: %d\n", ret);

	/* Wait some time until system down, otherwise, notice with a warn */
	mdelay(1000);

	if (mt6323_pwrc_charger_present(pwrc))
		mt6323_pwrc_wait_for_charger(pwrc);

	WARN_ONCE(1, "Unable to power off system\n");

	return NOTIFY_DONE;
}

static int mt6323_pwrc_probe(struct platform_device *pdev)
{
	struct mt6397_chip *mt6397_chip = dev_get_drvdata(pdev->dev.parent);
	struct mt6323_pwrc *pwrc;
	struct resource *res;
	int ret;

	pwrc = devm_kzalloc(&pdev->dev, sizeof(*pwrc), GFP_KERNEL);
	if (!pwrc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	pwrc->base = res->start;
	pwrc->regmap = mt6397_chip->regmap;
	pwrc->dev = &pdev->dev;

	ret = devm_register_sys_off_handler(pwrc->dev,
					    SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    mt6323_do_pwroff,
					    pwrc);
	if (ret)
		return dev_err_probe(pwrc->dev, ret, "failed to register power-off handler\n");

	return 0;
}

static const struct of_device_id mt6323_pwrc_dt_match[] = {
	{ .compatible = "mediatek,mt6323-pwrc" },
	{},
};
MODULE_DEVICE_TABLE(of, mt6323_pwrc_dt_match);

static struct platform_driver mt6323_pwrc_driver = {
	.probe          = mt6323_pwrc_probe,
	.driver         = {
		.name   = "mt6323-pwrc",
		.of_match_table = mt6323_pwrc_dt_match,
	},
};

module_platform_driver(mt6323_pwrc_driver);

MODULE_DESCRIPTION("Poweroff driver for MT6323 PMIC");
MODULE_AUTHOR("Sean Wang <sean.wang@mediatek.com>");
