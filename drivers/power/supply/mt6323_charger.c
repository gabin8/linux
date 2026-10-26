// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6323 PMIC linear charger
 *
 * The PMIC drives an external pass transistor and regulates, in hardware,
 * the charge current (sensed across a resistor in the charge path) and the
 * battery voltage. Software picks the limits, kicks the charger watchdog,
 * which turns charging off 4 s after the last kick, and decides when the
 * battery is full.
 */

#include <linux/bitfield.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/workqueue.h>

/* CHR_CON0 */
#define MT6323_RGS_VCDT_HV_DET		BIT(7)
#define MT6323_RGS_CHRDET		BIT(5)
#define MT6323_RG_CHR_EN		BIT(4)
#define MT6323_RG_CSDAC_EN		BIT(3)
#define MT6323_RG_VCDT_HV_EN		BIT(0)
/* CHR_CON1 */
#define MT6323_RG_VCDT_HV_VTH		GENMASK(7, 4)
#define MT6323_VCDT_HV_VTH_6V5		0xa
/* CHR_CON2 */
#define MT6323_RGS_VBAT_CC_DET		BIT(7)
#define MT6323_RG_CS_EN			BIT(3)
#define MT6323_RG_VBAT_CV_EN		BIT(1)
/* CHR_CON3 */
#define MT6323_RG_VBAT_CV_VTH		GENMASK(4, 0)
/* CHR_CON4 */
#define MT6323_RG_CS_VTH		GENMASK(3, 0)
/* CHR_CON6 */
#define MT6323_RGS_VBAT_OV_DET		BIT(6)
#define MT6323_RG_VBAT_OV_VTH		GENMASK(3, 1)
#define MT6323_RG_VBAT_OV_EN		BIT(0)
/* CHR_CON7 */
#define MT6323_RGS_BATON_UNDET		BIT(12)
#define MT6323_RG_BATON_EN		BIT(0)
/* CHR_CON13 */
#define MT6323_RG_CHRWDT_WR		BIT(8)
#define MT6323_RG_CHRWDT_EN		BIT(4)
#define MT6323_RG_CHRWDT_TD		GENMASK(3, 0)
#define MT6323_CHRWDT_TD_4S		0
/* CHR_CON15 */
#define MT6323_RG_CHRWDT_FLAG_WR	BIT(1)
#define MT6323_RG_CHRWDT_INT_EN		BIT(0)
/* CHR_CON20 */
#define MT6323_RG_CSDAC_STP_DEC		GENMASK(6, 4)
#define MT6323_RG_CSDAC_STP_INC		GENMASK(2, 0)
/* CHR_CON21 */
#define MT6323_RG_CSDAC_STP		GENMASK(6, 4)
#define MT6323_RG_CSDAC_DLY		GENMASK(2, 0)
/* CHR_CON22 */
#define MT6323_RG_LOW_ICH_DB		GENMASK(5, 0)
#define MT6323_LOW_ICH_DB_16MS		1
/* CHR_CON23 */
#define MT6323_RG_ULC_DET_EN		BIT(7)
#define MT6323_RG_HWCV_EN		BIT(6)
#define MT6323_RG_CSDAC_MODE		BIT(2)
#define MT6323_RG_VCDT_MODE		BIT(1)

/* The current DAC ramp the vendor kernel programs before every enable */
#define MT6323_CSDAC_DLY		4
#define MT6323_CSDAC_STP		1
#define MT6323_CSDAC_STP_INC		1
#define MT6323_CSDAC_STP_DEC		2

/*
 * Without BC1.2 detection, stay within what a USB 2.0 port may supply
 * (500 mA), with room for the pass transistor's drive current.
 */
#define MT6323_CHG_USB_MAX_UA		450000
#define MT6323_CHG_TERM_DEFAULT_UA	150000
#define MT6323_CHG_RESTART_MARGIN_UV	90000
/* Below CV by this much the battery counts as being topped off */
#define MT6323_CHG_TOPOFF_MARGIN_UV	150000
#define MT6323_CHG_TEMP_HYST_MC		3000
#define MT6323_CHG_ADC_SAMPLES		8
/* Averaging weight of the charge current: about 16 s at the kick period */
#define MT6323_CHG_CURRENT_AVG_WEIGHT	8

/* The charger watchdog times out 4 s after the last kick */
#define MT6323_CHG_KICK_PERIOD		(2 * HZ)
#define MT6323_CHG_IDLE_PERIOD		(10 * HZ)
#define MT6323_CHG_TAPER_TIME		(60 * HZ)
#define MT6323_CHG_TOPOFF_TIMEOUT	(3 * 60 * 60 * HZ)
#define MT6323_CHG_TIMEOUT		(10 * 60 * 60 * HZ)

/* Voltage across the sense resistor that each CS_VTH code regulates to */
static const u32 mt6323_cs_vth_uv[] = {
	320000, 300000, 280000, 260000, 240000, 220000, 200000, 180000,
	160000, 140000, 130000, 110000,  90000,  60000,  40000,  14000,
};

/* CV threshold per VBAT_CV_VTH code; 15 is the 2.2 V BC1.2 level */
static const u32 mt6323_cv_vth_uv[] = {
	4200000, 4212500, 4225000, 4237500, 4250000, 4262500, 4275000, 4300000,
	4325000, 4350000, 4375000, 4400000, 4425000, 4162500, 4175000, 0,
	4050000, 4100000, 4125000, 3775000, 3800000, 3850000, 3900000, 4000000,
	4050000, 4100000, 4125000, 4137500, 4150000, 4162500, 4175000, 4187500,
};

static const u32 mt6323_ov_vth_uv[] = { 4200000, 4300000, 4400000, 4450000 };

struct mt6323_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct iio_channel *isense;
	struct iio_channel *batsns;
	struct iio_channel *temp;
	struct delayed_work work;

	u32 rsense_uohm;
	int cv_max_uv;
	unsigned int cv_code;
	unsigned int ov_code;
	int cc_max_ua;
	int term_ua;
	int restart_uv;
	int topoff_uv;
	int temp_min_mc;
	int temp_max_mc;

	struct mutex lock;	/* the state below and the charger registers */
	unsigned int cs_code;
	bool online;
	bool charging;
	bool full;
	bool precharge;
	int health;
	int latched_health;
	int current_ua;
	int current_avg_ua;
	unsigned long charge_start;
	unsigned long topoff_start;
	unsigned long taper_start;
	bool in_topoff;
	bool in_taper;
};

static int mt6323_chg_cs_ua(struct mt6323_charger *chg, unsigned int code)
{
	return div_u64((u64)mt6323_cs_vth_uv[code] * MICRO, chg->rsense_uohm);
}

/* The largest current at or below @ua, or the smallest one there is. */
static unsigned int mt6323_chg_cs_code(struct mt6323_charger *chg, int ua)
{
	unsigned int code;

	for (code = 0; code < ARRAY_SIZE(mt6323_cs_vth_uv) - 1; code++)
		if (mt6323_chg_cs_ua(chg, code) <= ua)
			break;

	return code;
}

static int mt6323_chg_cv_code(int uv, unsigned int *code)
{
	unsigned int i;
	u32 best = 0;

	for (i = 0; i < ARRAY_SIZE(mt6323_cv_vth_uv); i++) {
		if (mt6323_cv_vth_uv[i] <= uv && mt6323_cv_vth_uv[i] > best) {
			best = mt6323_cv_vth_uv[i];
			*code = i;
		}
	}

	return best ? 0 : -EINVAL;
}

/* The lowest cut-off above the CV level, and not above @limit_uv. */
static int mt6323_chg_ov_code(u32 cv_uv, int limit_uv, unsigned int *code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mt6323_ov_vth_uv); i++) {
		if (mt6323_ov_vth_uv[i] <= cv_uv)
			continue;
		if (limit_uv > 0 && mt6323_ov_vth_uv[i] > limit_uv)
			break;
		*code = i;
		return 0;
	}

	return -EINVAL;
}

static int mt6323_chg_kick(struct mt6323_charger *chg)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, MT6323_CHR_CON13,
				 MT6323_RG_CHRWDT_TD,
				 FIELD_PREP(MT6323_RG_CHRWDT_TD, MT6323_CHRWDT_TD_4S));
	if (ret)
		return ret;
	ret = regmap_set_bits(chg->regmap, MT6323_CHR_CON13, MT6323_RG_CHRWDT_WR);
	if (ret)
		return ret;
	ret = regmap_set_bits(chg->regmap, MT6323_CHR_CON15,
			      MT6323_RG_CHRWDT_INT_EN);
	if (ret)
		return ret;
	ret = regmap_set_bits(chg->regmap, MT6323_CHR_CON13, MT6323_RG_CHRWDT_EN);
	if (ret)
		return ret;

	/* Also clears the time-out flag */
	return regmap_set_bits(chg->regmap, MT6323_CHR_CON15,
			       MT6323_RG_CHRWDT_FLAG_WR);
}

static int mt6323_chg_enable(struct mt6323_charger *chg)
{
	struct regmap *map = chg->regmap;
	int ret;

	ret = mt6323_chg_kick(chg);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MT6323_CHR_CON4, MT6323_RG_CS_VTH,
				 FIELD_PREP(MT6323_RG_CS_VTH, chg->cs_code));
	if (ret)
		return ret;
	ret = regmap_update_bits(map, MT6323_CHR_CON3, MT6323_RG_VBAT_CV_VTH,
				 FIELD_PREP(MT6323_RG_VBAT_CV_VTH, chg->cv_code));
	if (ret)
		return ret;

	ret = regmap_update_bits(map, MT6323_CHR_CON21,
				 MT6323_RG_CSDAC_STP | MT6323_RG_CSDAC_DLY,
				 FIELD_PREP(MT6323_RG_CSDAC_STP, MT6323_CSDAC_STP) |
				 FIELD_PREP(MT6323_RG_CSDAC_DLY, MT6323_CSDAC_DLY));
	if (ret)
		return ret;
	ret = regmap_update_bits(map, MT6323_CHR_CON20,
				 MT6323_RG_CSDAC_STP_DEC | MT6323_RG_CSDAC_STP_INC,
				 FIELD_PREP(MT6323_RG_CSDAC_STP_DEC, MT6323_CSDAC_STP_DEC) |
				 FIELD_PREP(MT6323_RG_CSDAC_STP_INC, MT6323_CSDAC_STP_INC));
	if (ret)
		return ret;

	ret = regmap_set_bits(map, MT6323_CHR_CON2,
			      MT6323_RG_CS_EN | MT6323_RG_VBAT_CV_EN);
	if (ret)
		return ret;
	ret = regmap_set_bits(map, MT6323_CHR_CON23, MT6323_RG_HWCV_EN);
	if (ret)
		return ret;
	ret = regmap_set_bits(map, MT6323_CHR_CON0, MT6323_RG_CSDAC_EN);
	if (ret)
		return ret;

	return regmap_set_bits(map, MT6323_CHR_CON0, MT6323_RG_CHR_EN);
}

static int mt6323_chg_disable(struct mt6323_charger *chg)
{
	struct regmap *map = chg->regmap;
	int ret;

	ret = regmap_clear_bits(map, MT6323_CHR_CON15,
				MT6323_RG_CHRWDT_INT_EN | MT6323_RG_CHRWDT_FLAG_WR);
	if (ret)
		return ret;
	ret = regmap_clear_bits(map, MT6323_CHR_CON13, MT6323_RG_CHRWDT_EN);
	if (ret)
		return ret;
	ret = regmap_clear_bits(map, MT6323_CHR_CON0,
				MT6323_RG_CSDAC_EN | MT6323_RG_CHR_EN);
	if (ret)
		return ret;

	return regmap_clear_bits(map, MT6323_CHR_CON23, MT6323_RG_HWCV_EN);
}

static int mt6323_chg_hw_init(struct mt6323_charger *chg)
{
	struct regmap *map = chg->regmap;
	int ret;

	ret = regmap_update_bits(map, MT6323_CHR_CON1, MT6323_RG_VCDT_HV_VTH,
				 FIELD_PREP(MT6323_RG_VCDT_HV_VTH,
					    MT6323_VCDT_HV_VTH_6V5));
	if (ret)
		return ret;
	ret = regmap_set_bits(map, MT6323_CHR_CON0, MT6323_RG_VCDT_HV_EN);
	if (ret)
		return ret;

	/* Arm the battery over-voltage cut-off before any charging */
	ret = regmap_update_bits(map, MT6323_CHR_CON6,
				 MT6323_RG_VBAT_OV_VTH | MT6323_RG_VBAT_OV_EN,
				 FIELD_PREP(MT6323_RG_VBAT_OV_VTH, chg->ov_code) |
				 MT6323_RG_VBAT_OV_EN);
	if (ret)
		return ret;
	ret = regmap_set_bits(map, MT6323_CHR_CON7, MT6323_RG_BATON_EN);
	if (ret)
		return ret;

	/* Plug-out detection, and charging gated by CSDAC_EN alone */
	ret = regmap_update_bits(map, MT6323_CHR_CON23,
				 MT6323_RG_ULC_DET_EN | MT6323_RG_CSDAC_MODE |
				 MT6323_RG_VCDT_MODE,
				 MT6323_RG_ULC_DET_EN | MT6323_RG_CSDAC_MODE);
	if (ret)
		return ret;
	ret = regmap_update_bits(map, MT6323_CHR_CON22, MT6323_RG_LOW_ICH_DB,
				 FIELD_PREP(MT6323_RG_LOW_ICH_DB,
					    MT6323_LOW_ICH_DB_16MS));
	if (ret)
		return ret;

	/* Whatever the bootloader left running */
	return mt6323_chg_disable(chg);
}

static int mt6323_chg_measure(struct mt6323_charger *chg, int *vbat_uv,
			      int *ichg_ua)
{
	s64 isense = 0, batsns = 0;
	int i, val, ret;

	for (i = 0; i < MT6323_CHG_ADC_SAMPLES; i++) {
		ret = iio_read_channel_processed_scale(chg->isense, &val, MILLI);
		if (ret < 0)
			return ret;
		isense += val;

		ret = iio_read_channel_processed_scale(chg->batsns, &val, MILLI);
		if (ret < 0)
			return ret;
		batsns += val;
	}

	*vbat_uv = div_s64(batsns, MT6323_CHG_ADC_SAMPLES);
	*ichg_ua = div_s64((isense - batsns) * MICRO,
			   MT6323_CHG_ADC_SAMPLES * chg->rsense_uohm);

	return 0;
}

static int mt6323_chg_temp_health(struct mt6323_charger *chg, int *health)
{
	int min = chg->temp_min_mc, max = chg->temp_max_mc;
	int temp, ret;

	if (!chg->temp) {
		*health = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	}

	ret = iio_read_channel_processed(chg->temp, &temp);
	if (ret < 0)
		return ret;

	if (chg->health == POWER_SUPPLY_HEALTH_OVERHEAT)
		max -= MT6323_CHG_TEMP_HYST_MC;
	if (chg->health == POWER_SUPPLY_HEALTH_COLD)
		min += MT6323_CHG_TEMP_HYST_MC;

	if (temp >= max)
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (temp < min)
		*health = POWER_SUPPLY_HEALTH_COLD;
	else
		*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static void mt6323_chg_session_reset(struct mt6323_charger *chg)
{
	chg->full = false;
	chg->latched_health = POWER_SUPPLY_HEALTH_GOOD;
	chg->in_topoff = false;
	chg->in_taper = false;
}

static int mt6323_chg_start(struct mt6323_charger *chg)
{
	int ret;

	ret = mt6323_chg_enable(chg);
	if (ret)
		return ret;

	chg->charging = true;
	chg->current_avg_ua = -1;
	chg->charge_start = jiffies;
	chg->in_topoff = false;
	chg->in_taper = false;
	dev_dbg(chg->dev, "charging at %d uA\n",
		mt6323_chg_cs_ua(chg, chg->cs_code));

	return 0;
}

static int mt6323_chg_stop(struct mt6323_charger *chg)
{
	chg->charging = false;
	chg->current_ua = 0;
	chg->current_avg_ua = 0;

	return mt6323_chg_disable(chg);
}

/* Full: topped off and the current tapered below the termination level. */
static void mt6323_chg_check_full(struct mt6323_charger *chg, int vbat_uv,
				  int ichg_ua)
{
	if (vbat_uv < chg->topoff_uv) {
		chg->in_topoff = false;
		chg->in_taper = false;
		return;
	}

	if (!chg->in_topoff) {
		chg->in_topoff = true;
		chg->topoff_start = jiffies;
	}

	if (ichg_ua > chg->term_ua) {
		chg->in_taper = false;
	} else if (!chg->in_taper) {
		chg->in_taper = true;
		chg->taper_start = jiffies;
	}

	if ((chg->in_taper &&
	     time_after_eq(jiffies, chg->taper_start + MT6323_CHG_TAPER_TIME)) ||
	    time_after_eq(jiffies, chg->topoff_start + MT6323_CHG_TOPOFF_TIMEOUT))
		chg->full = true;
}

static int mt6323_chg_update(struct mt6323_charger *chg)
{
	unsigned int con0, con2, con6, con7;
	int vbat_uv, ichg_ua, health, ret;

	ret = regmap_read(chg->regmap, MT6323_CHR_CON0, &con0);
	if (ret)
		return ret;

	if (!(con0 & MT6323_RGS_CHRDET)) {
		chg->online = false;
		chg->health = POWER_SUPPLY_HEALTH_GOOD;
		return chg->charging ? mt6323_chg_stop(chg) : 0;
	}

	if (!chg->online) {
		chg->online = true;
		mt6323_chg_session_reset(chg);
	}

	ret = regmap_read(chg->regmap, MT6323_CHR_CON2, &con2);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, MT6323_CHR_CON6, &con6);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, MT6323_CHR_CON7, &con7);
	if (ret)
		return ret;

	chg->precharge = !(con2 & MT6323_RGS_VBAT_CC_DET);

	if (chg->charging &&
	    time_after_eq(jiffies, chg->charge_start + MT6323_CHG_TIMEOUT))
		chg->latched_health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	if (con6 & MT6323_RGS_VBAT_OV_DET)
		chg->latched_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	if (chg->latched_health != POWER_SUPPLY_HEALTH_GOOD) {
		health = chg->latched_health;
	} else if (con0 & MT6323_RGS_VCDT_HV_DET) {
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if (con7 & MT6323_RGS_BATON_UNDET) {
		health = POWER_SUPPLY_HEALTH_NO_BATTERY;
	} else {
		ret = mt6323_chg_temp_health(chg, &health);
		if (ret)
			return ret;
	}

	if (health != chg->health && health != POWER_SUPPLY_HEALTH_GOOD)
		dev_warn(chg->dev, "charging held off, health %d\n", health);
	chg->health = health;

	if (health != POWER_SUPPLY_HEALTH_GOOD)
		return chg->charging ? mt6323_chg_stop(chg) : 0;

	ret = mt6323_chg_measure(chg, &vbat_uv, &ichg_ua);
	if (ret)
		return ret;

	if (!chg->charging) {
		if (chg->full && vbat_uv >= chg->restart_uv)
			return 0;
		chg->full = false;
		return mt6323_chg_start(chg);
	}

	/* A late kick lets the watchdog turn the charger off: turn it back on */
	if (!(con0 & MT6323_RG_CHR_EN)) {
		dev_warn(chg->dev, "charger watchdog expired, restarting\n");
		return mt6323_chg_enable(chg);
	}

	ret = mt6323_chg_kick(chg);
	if (ret)
		return ret;

	/*
	 * Load steps elsewhere in the phone show up in the charge current
	 * too: decide on the average, as the vendor driver does.
	 */
	chg->current_ua = ichg_ua;
	if (chg->current_avg_ua < 0)
		chg->current_avg_ua = ichg_ua;
	else
		chg->current_avg_ua += (ichg_ua - chg->current_avg_ua) /
				       MT6323_CHG_CURRENT_AVG_WEIGHT;

	mt6323_chg_check_full(chg, vbat_uv, chg->current_avg_ua);
	if (chg->full) {
		dev_info(chg->dev, "battery full at %d uV, %d uA\n",
			 vbat_uv, chg->current_avg_ua);
		return mt6323_chg_stop(chg);
	}

	return 0;
}

static void mt6323_chg_work(struct work_struct *work)
{
	struct mt6323_charger *chg = container_of(work, struct mt6323_charger,
						  work.work);
	bool online, charging, full;
	int health, ret;

	scoped_guard(mutex, &chg->lock) {
		online = chg->online;
		charging = chg->charging;
		full = chg->full;
		health = chg->health;

		ret = mt6323_chg_update(chg);
		if (ret) {
			dev_err(chg->dev, "update failed: %pe\n", ERR_PTR(ret));
			mt6323_chg_stop(chg);
		}

		if (chg->online)
			queue_delayed_work(system_power_efficient_wq, &chg->work,
					   chg->charging ? MT6323_CHG_KICK_PERIOD :
							   MT6323_CHG_IDLE_PERIOD);

		if (online == chg->online && charging == chg->charging &&
		    full == chg->full && health == chg->health)
			return;
	}

	power_supply_changed(chg->psy);
}

static irqreturn_t mt6323_chg_irq(int irq, void *data)
{
	struct mt6323_charger *chg = data;

	mod_delayed_work(system_power_efficient_wq, &chg->work, 0);

	return IRQ_HANDLED;
}

static int mt6323_chg_status(struct mt6323_charger *chg)
{
	if (!chg->online)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (chg->charging)
		return POWER_SUPPLY_STATUS_CHARGING;
	if (chg->full)
		return POWER_SUPPLY_STATUS_FULL;

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int mt6323_chg_charge_type(struct mt6323_charger *chg)
{
	if (!chg->charging)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	return chg->precharge ? POWER_SUPPLY_CHARGE_TYPE_TRICKLE :
				POWER_SUPPLY_CHARGE_TYPE_FAST;
}

static int mt6323_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6323_charger *chg = power_supply_get_drvdata(psy);

	guard(mutex)(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mt6323_chg_status(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = mt6323_chg_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chg->current_ua;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = max(chg->current_avg_ua, 0);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = mt6323_chg_cs_ua(chg, chg->cs_code);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chg->cc_max_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = mt6323_cv_vth_uv[chg->cv_code];
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = chg->cv_max_uv;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt6323_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct mt6323_charger *chg = power_supply_get_drvdata(psy);

	guard(mutex)(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval <= 0 || val->intval > chg->cc_max_ua)
			return -EINVAL;
		chg->cs_code = mt6323_chg_cs_code(chg, val->intval);
		if (!chg->charging)
			return 0;
		return regmap_update_bits(chg->regmap, MT6323_CHR_CON4,
					  MT6323_RG_CS_VTH,
					  FIELD_PREP(MT6323_RG_CS_VTH,
						     chg->cs_code));
	default:
		return -EINVAL;
	}
}

static int mt6323_chg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static const enum power_supply_property mt6323_chg_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
};

static const struct power_supply_desc mt6323_chg_desc = {
	.name = "mt6323-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mt6323_chg_properties,
	.num_properties = ARRAY_SIZE(mt6323_chg_properties),
	.get_property = mt6323_chg_get_property,
	.set_property = mt6323_chg_set_property,
	.property_is_writeable = mt6323_chg_property_is_writeable,
};

static int mt6323_chg_parse_battery(struct mt6323_charger *chg,
				    struct power_supply_battery_info *bat)
{
	struct device *dev = chg->dev;
	unsigned int code;
	int cc_max, ret;

	if (bat->constant_charge_voltage_max_uv <= 0 ||
	    bat->constant_charge_current_max_ua <= 0)
		return dev_err_probe(dev, -EINVAL,
				     "battery lacks a charge voltage or current limit\n");

	ret = mt6323_chg_cv_code(bat->constant_charge_voltage_max_uv,
				 &chg->cv_code);
	if (ret)
		return dev_err_probe(dev, ret, "no CV level at or below %d uV\n",
				     bat->constant_charge_voltage_max_uv);
	chg->cv_max_uv = bat->constant_charge_voltage_max_uv;

	ret = mt6323_chg_ov_code(mt6323_cv_vth_uv[chg->cv_code],
				 bat->overvoltage_limit_uv, &chg->ov_code);
	if (ret)
		return dev_err_probe(dev, ret, "no over-voltage cut-off fits\n");

	cc_max = min(bat->constant_charge_current_max_ua, MT6323_CHG_USB_MAX_UA);
	code = mt6323_chg_cs_code(chg, cc_max);
	if (mt6323_chg_cs_ua(chg, code) > cc_max)
		return dev_err_probe(dev, -EINVAL, "%d uA is below the lowest current\n",
				     cc_max);
	chg->cs_code = code;
	chg->cc_max_ua = mt6323_chg_cs_ua(chg, code);

	chg->term_ua = bat->charge_term_current_ua > 0 ?
		       bat->charge_term_current_ua : MT6323_CHG_TERM_DEFAULT_UA;
	chg->restart_uv = bat->charge_restart_voltage_uv > 0 ?
			  bat->charge_restart_voltage_uv :
			  mt6323_cv_vth_uv[chg->cv_code] - MT6323_CHG_RESTART_MARGIN_UV;
	chg->topoff_uv = mt6323_cv_vth_uv[chg->cv_code] - MT6323_CHG_TOPOFF_MARGIN_UV;

	if (!chg->temp)
		return 0;

	if (bat->temp_min == INT_MIN || bat->temp_max == INT_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "battery lacks an operating temperature range\n");
	chg->temp_min_mc = bat->temp_min * MILLIDEGREE_PER_DEGREE;
	chg->temp_max_mc = bat->temp_max * MILLIDEGREE_PER_DEGREE;

	return 0;
}

static void mt6323_chg_teardown(void *data)
{
	struct mt6323_charger *chg = data;

	cancel_delayed_work_sync(&chg->work);
	mt6323_chg_disable(chg);
}

static int mt6323_chg_probe(struct platform_device *pdev)
{
	struct mt6397_chip *pmic = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_battery_info *bat;
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	static const char * const irq_names[] = { "chrdet", "ov", "watchdog" };
	struct mt6323_charger *chg;
	unsigned int i;
	int irq, ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->regmap = pmic->regmap;
	chg->health = POWER_SUPPLY_HEALTH_GOOD;

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms",
				       &chg->rsense_uohm);
	if (ret || !chg->rsense_uohm)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "missing shunt-resistor-micro-ohms\n");

	chg->isense = devm_iio_channel_get(dev, "isense");
	if (IS_ERR(chg->isense))
		return dev_err_probe(dev, PTR_ERR(chg->isense),
				     "failed to get the isense channel\n");

	chg->batsns = devm_iio_channel_get(dev, "batsns");
	if (IS_ERR(chg->batsns))
		return dev_err_probe(dev, PTR_ERR(chg->batsns),
				     "failed to get the batsns channel\n");

	chg->temp = devm_iio_channel_get(dev, "temp");
	if (IS_ERR(chg->temp)) {
		if (PTR_ERR(chg->temp) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(chg->temp),
					     "failed to get the temp channel\n");
		dev_warn(dev, "no battery temperature, charging without a temperature check\n");
		chg->temp = NULL;
	}

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &mt6323_chg_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "failed to register the power supply\n");

	ret = power_supply_get_battery_info(chg->psy, &bat);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the battery info\n");

	ret = mt6323_chg_parse_battery(chg, bat);
	if (ret)
		return ret;

	ret = mt6323_chg_hw_init(chg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up the charger\n");

	INIT_DELAYED_WORK(&chg->work, mt6323_chg_work);
	ret = devm_add_action_or_reset(dev, mt6323_chg_teardown, chg);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev, irq, NULL, mt6323_chg_irq,
						IRQF_ONESHOT, irq_names[i], chg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request the %s irq\n",
					     irq_names[i]);
	}

	queue_delayed_work(system_power_efficient_wq, &chg->work, 0);

	return 0;
}

static const struct of_device_id mt6323_chg_of_match[] = {
	{ .compatible = "mediatek,mt6323-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6323_chg_of_match);

static struct platform_driver mt6323_chg_driver = {
	.probe = mt6323_chg_probe,
	.driver = {
		.name = "mt6323-charger",
		.of_match_table = mt6323_chg_of_match,
	},
};
module_platform_driver(mt6323_chg_driver);

MODULE_DESCRIPTION("MediaTek MT6323 PMIC linear charger driver");
MODULE_LICENSE("GPL");
