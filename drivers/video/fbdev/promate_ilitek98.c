/*
 * Initialization (not framebuffer) driver for Promate RF-20150132_H01/
 * Ilitek ILI9806E
 *
 * Copyright (C) 2016 Emcraft Systems
 * Author: Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>

#define DRIVER_NAME		"promate-ili98"

static int promate_send_data(struct spi_device *spi, u8 value)
{
	u16 data[1] = { value | (1 << 8) };
	int err = spi_write(spi, data, 2);

	if (err)
		dev_err(&spi->dev, "%s: err %d\n", __func__, err);

	return err;
}

static int promate_send_command(struct spi_device *spi, u8 cmd, int args_cnt, ...)
{
	u16 data[1] = { cmd };
	int err = spi_write(spi, data, 2);

	if (!err && args_cnt) {
		va_list ap;
		int i;
		va_start(ap, args_cnt);

		for (i = 0; i < args_cnt; ++i) {
			u8 next_byte = va_arg(ap, unsigned int);

			err = promate_send_data(spi, next_byte);

			if (err)
				break;
		}

		va_end(ap);
	}

	if (err)
		dev_err(&spi->dev, "%s: err %d\n", __func__, err);

	return err;
}

static int promate_hw_init(struct spi_device *spi)
{
	/* LCD SETING */
	/* Change to Page 1 CMD */
	promate_send_command(spi, 0xFF, 5, 0xFF, 0x98, 6, 4, 1);

	/* Output    SDA */
	promate_send_command(spi, 0x08, 1, 0x10);

	/* set DE/VSYNC mode    (platform select) */
	promate_send_command(spi, 0x20, 1, 0);

	/* DE = 1 Active */
	promate_send_command(spi, 0x21, 1, 1);

	/* Resolution setting 480 X 800 */
	promate_send_command(spi, 0x30, 1, 2);

	/* Inversion setting 2-dot */
	promate_send_command(spi, 0x31, 1, 2);

	/* 24 bit */
	promate_send_command(spi, 0x3A, 1, 0x70);

	promate_send_command(spi, 0x60, 1, 7);
	promate_send_command(spi, 0x61, 1, 0);
	promate_send_command(spi, 0x62, 1, 8);
	promate_send_command(spi, 0x63, 1, 0);

	/* BT  AVDD,AVDD 10,14,18 */
	promate_send_command(spi, 0x40, 1, 0x18);

	/* avdd +5.4v,avee-5.2v 55 */
	promate_send_command(spi, 0x41, 1, 0x33);

	/* VGL=DDVDH+VCIP -DDVDL,VGH=2DDVDL-VCIP */
	promate_send_command(spi, 0x42, 1, 0x11);

	/* SET VGH clamp level */
	promate_send_command(spi, 0x43, 1, 0x8);

	/* SET VGL clamp level */
	promate_send_command(spi, 0x44, 1, 5);

	promate_send_command(spi, 0x46, 1, 0x55);
	promate_send_command(spi, 0x47, 1, 0x55);

	/* VREG1 */
	promate_send_command(spi, 0x50, 1, 0x78);

	/* VREG2 */
	promate_send_command(spi, 0x51, 1, 0x78);

	/* Flicker MSB */
	promate_send_command(spi, 0x52, 1, 0);

	/* Flicker LSB, VCOM */
	promate_send_command(spi, 0x53, 1, 0x60);

	/* LVD  Discharge */
	promate_send_command(spi, 0x57, 1, 0x50);

	/* Positive Gamma */
	promate_send_command(spi, 0xA0, 1, 0);
	promate_send_command(spi, 0xA1, 1, 0xA);
	promate_send_command(spi, 0xA2, 1, 0x11);
	promate_send_command(spi, 0xA3, 1, 0xC);
	promate_send_command(spi, 0xA4, 1, 5);
	promate_send_command(spi, 0xA5, 1, 9);
	promate_send_command(spi, 0xA6, 1, 6);
	promate_send_command(spi, 0xA7, 1, 5);
	promate_send_command(spi, 0xA8, 1, 9);
	promate_send_command(spi, 0xA9, 1, 0xC);
	promate_send_command(spi, 0xAA, 1, 0x11);
	promate_send_command(spi, 0xAB, 1, 9);
	promate_send_command(spi, 0xAC, 1, 0xD);
	promate_send_command(spi, 0xAD, 1, 0x19);
	promate_send_command(spi, 0xAE, 1, 0x11);
	promate_send_command(spi, 0xAF, 1, 0);

	/* Negative Gamma */
	promate_send_command(spi, 0xC0, 1, 0);
	promate_send_command(spi, 0xC1, 1, 0xA);
	promate_send_command(spi, 0xC2, 1, 0x11);
	promate_send_command(spi, 0xC3, 1, 0xC);
	promate_send_command(spi, 0xC4, 1, 5);
	promate_send_command(spi, 0xC5, 1, 9);
	promate_send_command(spi, 0xC6, 1, 6);
	promate_send_command(spi, 0xC7, 1, 5);
	promate_send_command(spi, 0xC8, 1, 9);
	promate_send_command(spi, 0xC9, 1, 0xC);
	promate_send_command(spi, 0xCA, 1, 0x11);
	promate_send_command(spi, 0xCB, 1, 9);
	promate_send_command(spi, 0xCC, 1, 0xD);
	promate_send_command(spi, 0xCD, 1, 0x19);
	promate_send_command(spi, 0xCE, 1, 0x11);
	promate_send_command(spi, 0xCF, 1, 0);


	/* Change to Page 6 CMD for GIP timing */
	promate_send_command(spi, 0xFF, 5, 0xFF, 0x98, 6, 4, 6);

	promate_send_command(spi, 0x00, 1, 0x21);
	promate_send_command(spi, 0x01, 1, 0xA);
	promate_send_command(spi, 0x02, 1, 0);
	promate_send_command(spi, 0x03, 1, 0);
	promate_send_command(spi, 0x04, 1, 5);
	promate_send_command(spi, 0x05, 1, 5);
	promate_send_command(spi, 0x06, 1, 0x80);
	promate_send_command(spi, 0x07, 1, 6);
	promate_send_command(spi, 0x08, 1, 1);
	promate_send_command(spi, 0x09, 1, 0);
	promate_send_command(spi, 0x0A, 1, 0);
	promate_send_command(spi, 0x0B, 1, 0);
	promate_send_command(spi, 0x0C, 1, 6);
	promate_send_command(spi, 0x0D, 1, 6);
	promate_send_command(spi, 0x0E, 1, 0);
	promate_send_command(spi, 0x0F, 1, 0);

	promate_send_command(spi, 0x10, 1, 0xF0);
	promate_send_command(spi, 0x11, 1, 0xF4);
	promate_send_command(spi, 0x12, 1, 3);
	promate_send_command(spi, 0x13, 1, 0);
	promate_send_command(spi, 0x14, 1, 0);
	promate_send_command(spi, 0x15, 1, 0xC0);
	promate_send_command(spi, 0x16, 1, 8);
	promate_send_command(spi, 0x17, 1, 0);
	promate_send_command(spi, 0x18, 1, 0);
	promate_send_command(spi, 0x19, 1, 0);
	promate_send_command(spi, 0x1A, 1, 0);
	promate_send_command(spi, 0x1B, 1, 0);
	promate_send_command(spi, 0x1C, 1, 0);
	promate_send_command(spi, 0x1D, 1, 0);

	promate_send_command(spi, 0x20, 1, 1);
	promate_send_command(spi, 0x21, 1, 0x23);
	promate_send_command(spi, 0x22, 1, 0x45);
	promate_send_command(spi, 0x23, 1, 0x67);
	promate_send_command(spi, 0x24, 1, 1);
	promate_send_command(spi, 0x25, 1, 0x23);
	promate_send_command(spi, 0x26, 1, 0x45);
	promate_send_command(spi, 0x27, 1, 0x67);

	promate_send_command(spi, 0x30, 1, 1);
	promate_send_command(spi, 0x31, 1, 0x11);
	promate_send_command(spi, 0x32, 1, 0);
	promate_send_command(spi, 0x33, 1, 0xEE);
	promate_send_command(spi, 0x34, 1, 0xFF);
	promate_send_command(spi, 0x35, 1, 0xCB);
	promate_send_command(spi, 0x36, 1, 0xDA);
	promate_send_command(spi, 0x37, 1, 0xAD);
	promate_send_command(spi, 0x38, 1, 0xBC);
	promate_send_command(spi, 0x39, 1, 0x76);
	promate_send_command(spi, 0x3A, 1, 0x67);
	promate_send_command(spi, 0x3B, 1, 0x22);
	promate_send_command(spi, 0x3C, 1, 0x22);
	promate_send_command(spi, 0x3D, 1, 0x22);
	promate_send_command(spi, 0x3E, 1, 0x22);
	promate_send_command(spi, 0x3F, 1, 0x22);
	promate_send_command(spi, 0x40, 1, 0x22);
	promate_send_command(spi, 0x52, 1, 0x10);
	promate_send_command(spi, 0x53, 1, 0x10);
	promate_send_command(spi, 0x54, 1, 0x13);

	/* Change to Page 7 CMD for GIP timing */
	promate_send_command(spi, 0xFF, 5, 0xFF, 0x98, 6, 4, 7);
	promate_send_command(spi, 0x17, 1, 0x22);
	promate_send_command(spi, 0x18, 1, 0x1D);
	promate_send_command(spi, 0x02, 1, 0x77);
	promate_send_command(spi, 0xE1, 1, 0x79);
	promate_send_command(spi, 0xb3, 1, 0x10);

	/* Change to Page 7 CMD for Normal command */
	promate_send_command(spi, 0xFF, 5, 0xFF, 0x98, 6, 4, 0);

	promate_send_command(spi, 0x11, 0);
	msleep(120);
	promate_send_command(spi, 0x29, 0);
	msleep(25);
	promate_send_command(spi, 0x2C, 0);

	return 0;
}

static int promate_probe(struct spi_device *spi)
{
	int err;
	spi->bits_per_word = 9;
	err = promate_hw_init(spi);
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int promate_suspend(struct device *dev)
{
	//struct spi_device *spi = to_spi_device(dev);
	return 0;
}

static int promate_resume(struct device *dev)
{
	//struct spi_device *spi = to_spi_device(dev);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id promate_of_match[] = {
	{ .compatible = "promate,ili98", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, promate_of_match);

static const struct dev_pm_ops promate_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(promate_suspend, promate_resume)
};

static struct spi_driver promate_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = promate_of_match,
		.pm		= &promate_pm_ops,
	},
	.probe			= promate_probe,
};

module_spi_driver(promate_driver);

MODULE_AUTHOR("Emcraft Systems");
MODULE_DESCRIPTION("Promate RF-20150132_H01/Ilitek ILI9806E driver");
MODULE_LICENSE("GPL");
