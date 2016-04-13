/*
 * Support for ON Semiconductor NOA1305 ambient light sensor
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

#define DRIVER_NAME		"noa1305"

#define NOA1305_REG_POWER_CONTROL	0x0
#define NOA1305_REG_RESET		0x1
#define NOA1305_REG_INTEGRATION_TIME	0x2
#define NOA1305_REG_INT_SELECT		0x3
#define NOA1305_REG_INT_THRESH_LSB	0x4
#define NOA1305_REG_INT_THRESH_MSB	0x5
#define NOA1305_REG_ALS_DATA_LSB	0x6
#define NOA1305_REG_ALS_DATA_MSB	0x7
#define NOA1305_REG_DEVICE_ID_LSB	0x8
#define NOA1305_REG_DEVICE_ID_MSB	0x9

#define NOA1305_DEVICE_ID		0x0519

#define NOA1305_POWER_ON		0x08
#define NOA1305_POWER_DOWN		0x00
#define NOA1305_RESET			0x10
#define NOA1305_INT_ACTIVE_HIGH		0x01
#define NOA1305_INT_ACTIVE_LOW		0x02
#define NOA1305_INTEGR_TIME_200MS	0x02

struct noa1305_priv {
	struct i2c_client *client;
};

static int noa1305_measure(struct noa1305_priv *priv)
{
	u8 data[2];
	int status;

	status = i2c_smbus_read_i2c_block_data(priv->client, NOA1305_REG_ALS_DATA_LSB, 2, data);
	if (status < 0)
		return status;

	return (data[1] << 8) | data[0];
}

static const struct iio_chan_spec noa1305_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int noa1305_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	int ret = -EINVAL;
	struct noa1305_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = noa1305_measure(priv);
			if (ret < 0)
				return ret;
			*val = ret;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}

static const struct iio_info noa1305_info = {
	.read_raw = noa1305_read_raw,
};

static int noa1305_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct noa1305_priv *priv;
	struct iio_dev *indio_dev;
	u8 data[2];
	unsigned int dev_id = 0;
	int status;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	priv->client = client;

	status = i2c_smbus_read_i2c_block_data(client, NOA1305_REG_DEVICE_ID_LSB, 2, data);
	if (status < 0) {
		dev_err(&client->dev, "ID reading failed: %d\n", status);
		return status;
	}

	dev_id = (data[1] << 8) | data[0];
	if (dev_id != NOA1305_DEVICE_ID) {
		dev_err(&client->dev, "Unkown device ID: 0x%x\n", dev_id);
		return -ENODEV;
	}

	i2c_smbus_write_byte_data(client, NOA1305_REG_POWER_CONTROL, NOA1305_POWER_ON);
	i2c_smbus_write_byte_data(client, NOA1305_REG_RESET, NOA1305_RESET);
	i2c_smbus_write_byte_data(client, NOA1305_REG_INTEGRATION_TIME, NOA1305_INTEGR_TIME_200MS);
	i2c_smbus_write_byte_data(client, NOA1305_REG_INT_SELECT, NOA1305_INT_ACTIVE_LOW);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &noa1305_info;
	indio_dev->channels = noa1305_channels;
	indio_dev->num_channels = ARRAY_SIZE(noa1305_channels);
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&client->dev, "NOA1305 Ambient light sensor\n");

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id noa1305_of_match[] = {
	{ .compatible = "on,noa1305" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, noa1305_of_match);

static const struct i2c_device_id noa1305_ids[] = {
	{ "noa1305", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, noa1305_id);


static struct i2c_driver noa1305_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.of_match_table	= noa1305_of_match,
	},
	.probe		= noa1305_probe,
	.id_table	= noa1305_ids,
};

module_i2c_driver(noa1305_driver);

MODULE_AUTHOR("Sergei Miroshnichenko <sergeimir@emcraft.com>");
MODULE_DESCRIPTION("ON Semiconductor NOA1305 ambient light sensor");
MODULE_LICENSE("GPL");
