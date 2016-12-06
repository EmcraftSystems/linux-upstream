/*
 * Support for NXP PCT2075 temperature sensor
 *
 * Copyright (C) 2016 Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define DRIVER_NAME		"pct2075"

#define PCT2075_REG_CONF	0x01
#define PCT2075_REG_TEMP	0x00
#define PCT2075_REG_TOS		0x02
#define PCT2075_REG_THYST	0x03
#define PCT2075_REG_TIDLE	0x04

#define PCT2075_CONF_SHUTDOWN	(0x1 << 0)

struct pct2075_data {
	struct i2c_client *client;
	u8 config;
};

static const struct iio_chan_spec pct2075_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int pct2075_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct pct2075_data *data = iio_priv(indio_dev);
	s32 ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_smbus_read_word_swapped(data->client, PCT2075_REG_TEMP);
		if (ret < 0)
			return ret;
		*val = sign_extend32(ret, 15) >> 5;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 8;
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info pct2075_info = {
	.read_raw = pct2075_read_raw,
	.driver_module = THIS_MODULE,
};

static int pct2075_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct pct2075_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &pct2075_info;

	indio_dev->channels = pct2075_channels;
	indio_dev->num_channels = ARRAY_SIZE(pct2075_channels);

	ret = i2c_smbus_read_byte_data(data->client, PCT2075_REG_CONF);
	if (ret < 0)
		return ret;
	data->config = ret;

	return iio_device_register(indio_dev);
}

static int pct2075_powerdown(struct pct2075_data *data)
{
	return i2c_smbus_write_byte_data(data->client, PCT2075_REG_CONF,
		data->config | PCT2075_CONF_SHUTDOWN);
}

static int pct2075_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	pct2075_powerdown(iio_priv(indio_dev));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pct2075_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	return pct2075_powerdown(iio_priv(indio_dev));
}

static int pct2075_resume(struct device *dev)
{
	struct pct2075_data *data = iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
	return i2c_smbus_write_byte_data(data->client, PCT2075_REG_CONF,
		data->config & ~PCT2075_CONF_SHUTDOWN);
}
#endif

static SIMPLE_DEV_PM_OPS(pct2075_pm_ops, pct2075_suspend, pct2075_resume);

static const struct i2c_device_id pct2075_id[] = {
	{ "pct2075", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pct2075_id);

static struct i2c_driver pct2075_driver = {
	.driver = {
		.name	= "pct2075",
		.pm	= &pct2075_pm_ops,
		.owner	= THIS_MODULE,
	},
	.probe = pct2075_probe,
	.remove = pct2075_remove,
	.id_table = pct2075_id,
};

module_i2c_driver(pct2075_driver);

MODULE_AUTHOR("Vladimir Skvortsov <vskvortsov@emcraft.com>");
MODULE_DESCRIPTION("NXP PCT2075 temperature sensor");
MODULE_LICENSE("GPL");
