// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rohm RPR-0400 ambient-light and proximity sensor driver.
 *
 * The RPR-0400 is an older sibling of the RPR-0521 with the same broad
 * register layout (control regs starting at 0x40, 16-bit little-endian
 * data registers from 0x44) but a different control-register split and
 * no readable ID register. This driver supports polled reads only.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>

#define RPR0400_REG_SYSTEM_CTRL		0x40
#define RPR0400_REG_MODE_CTRL		0x41
#define RPR0400_REG_ALSPS_CTRL		0x42
#define RPR0400_REG_PS_DATA		0x44 /* 16-bit LE */
#define RPR0400_REG_ALS_DATA0		0x46 /* 16-bit LE */
#define RPR0400_REG_ALS_DATA1		0x48 /* 16-bit LE */
#define RPR0400_REG_ALS_TL		0x51 /* 16-bit LE — last register */

/* SYSTEM_CTRL */
#define RPR0400_SW_RESET		BIT(7)
#define RPR0400_INT_RESET		BIT(6)

/* MODE_CTRL — bits[3:0] select a (PS-rate, ALS-rate) table entry. */
#define RPR0400_MEAS_STANDBY		0x0
#define RPR0400_MEAS_PS_ALS_100		0x6 /* PS+ALS, 100 ms each */

/* ALSPS_CTRL: LED current + ALS gain. */
#define RPR0400_LED_100MA		0x2
#define RPR0400_ALS_GAIN_X2X2		(0x5 << 2)
#define RPR0400_DEFAULT_ALSPS \
	(RPR0400_LED_100MA | RPR0400_ALS_GAIN_X2X2)

#define RPR0400_RESET_DELAY_MS		5
#define RPR0400_FIRST_MEAS_DELAY_MS	120

static const struct regmap_config rpr0400_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = RPR0400_REG_ALS_TL + 1,
};

struct rpr0400_data {
	struct regmap *regmap;
	struct mutex lock;
	bool active;
};

static int rpr0400_set_active(struct rpr0400_data *d, bool on)
{
	u8 mode = on ? RPR0400_MEAS_PS_ALS_100 : RPR0400_MEAS_STANDBY;
	int ret;

	ret = regmap_write(d->regmap, RPR0400_REG_MODE_CTRL, mode);
	if (ret)
		return ret;

	d->active = on;
	if (on)
		msleep(RPR0400_FIRST_MEAS_DELAY_MS);
	return 0;
}

static int rpr0400_read_word(struct rpr0400_data *d, u8 reg, u16 *out)
{
	__le16 raw;
	int ret;

	ret = regmap_bulk_read(d->regmap, reg, &raw, sizeof(raw));
	if (ret)
		return ret;
	*out = le16_to_cpu(raw);
	return 0;
}

static int rpr0400_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct rpr0400_data *d = iio_priv(indio_dev);
	u16 raw;
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	guard(mutex)(&d->lock);

	if (!d->active) {
		ret = rpr0400_set_active(d, true);
		if (ret)
			return ret;
	}

	ret = rpr0400_read_word(d, chan->address, &raw);
	if (ret)
		return ret;

	*val = raw;
	return IIO_VAL_INT;
}

static const struct iio_chan_spec rpr0400_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.address = RPR0400_REG_PS_DATA,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.modified = 1,
		.address = RPR0400_REG_ALS_DATA0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.channel2 = IIO_MOD_LIGHT_IR,
		.modified = 1,
		.address = RPR0400_REG_ALS_DATA1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static const struct iio_info rpr0400_info = {
	.read_raw = rpr0400_read_raw,
};

static int rpr0400_init_chip(struct rpr0400_data *d)
{
	int ret;

	ret = regmap_write(d->regmap, RPR0400_REG_SYSTEM_CTRL,
			   RPR0400_SW_RESET | RPR0400_INT_RESET);
	if (ret)
		return ret;

	msleep(RPR0400_RESET_DELAY_MS);

	ret = regmap_write(d->regmap, RPR0400_REG_ALSPS_CTRL,
			   RPR0400_DEFAULT_ALSPS);
	if (ret)
		return ret;

	/* Leave the chip in standby; the first IIO read activates it. */
	return regmap_write(d->regmap, RPR0400_REG_MODE_CTRL,
			    RPR0400_MEAS_STANDBY);
}

static const char * const rpr0400_supplies[] = { "vdd", "vddio" };

static int rpr0400_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct rpr0400_data *d;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*d));
	if (!indio_dev)
		return -ENOMEM;

	d = iio_priv(indio_dev);
	mutex_init(&d->lock);

	d->regmap = devm_regmap_init_i2c(client, &rpr0400_regmap_config);
	if (IS_ERR(d->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(d->regmap),
				     "regmap init failed\n");

	ret = devm_regulator_bulk_get_enable(&client->dev,
					     ARRAY_SIZE(rpr0400_supplies),
					     rpr0400_supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "regulator setup failed\n");

	usleep_range(2000, 3000);

	ret = rpr0400_init_chip(d);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "chip init failed\n");

	indio_dev->name = "rpr0400";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = rpr0400_channels;
	indio_dev->num_channels = ARRAY_SIZE(rpr0400_channels);
	indio_dev->info = &rpr0400_info;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id rpr0400_of_match[] = {
	{ .compatible = "rohm,rpr0400" },
	{}
};
MODULE_DEVICE_TABLE(of, rpr0400_of_match);

static const struct i2c_device_id rpr0400_id[] = {
	{ "rpr0400" },
	{}
};
MODULE_DEVICE_TABLE(i2c, rpr0400_id);

static struct i2c_driver rpr0400_driver = {
	.driver = {
		.name = "rpr0400",
		.of_match_table = rpr0400_of_match,
	},
	.probe = rpr0400_probe,
	.id_table = rpr0400_id,
};
module_i2c_driver(rpr0400_driver);

MODULE_DESCRIPTION("Rohm RPR-0400 ambient light and proximity sensor");
MODULE_LICENSE("GPL");
