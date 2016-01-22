/*
 * Numeric display
 *
 * Copyright (C) 2015 Emcraft Systems
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
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/freezer.h>

#define DRIVER_NAME	"num_lcd"

struct numdisp_info {
	int			width;
	int			count;
	struct gpio_desc	*le_gpio;
	struct gpio_desc	*oe_gpio;
	struct spi_device	*spi;
	char			text[128];
};

static char syms[] = { ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'C', 'E', 'F', 'H', 'L', 'P', 'R', 'U' };
static unsigned int sym_codes[] = { 0, 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x39, 0x79, 0x71, 0x76, 0x38, 0x73, 0x31, 0x3E };

static unsigned int numdisp_sym_to_code(char sym)
{
	int i;

	for (i = 0; i < sizeof(syms); ++i)
		if (sym == syms[i])
			return sym_codes[i];

	return 0;
}

static int numdisp_set(struct numdisp_info *numdisp, int lcd, char sym, bool dot)
{
	unsigned char buf[1];

	gpiod_set_value(numdisp->oe_gpio, 0);

	buf[0] = numdisp_sym_to_code(sym) | (dot ? 0x80 : 0);
	spi_write(numdisp->spi, buf, 1);

	buf[0] = 1 << lcd;
	spi_write(numdisp->spi, buf, 1);

	gpiod_set_value(numdisp->le_gpio, 0);
	gpiod_set_value(numdisp->le_gpio, 1);

	gpiod_set_value(numdisp->oe_gpio, 1);

	return 0;
}

static ssize_t numdisp_get_text(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i;
	char *buf_orig = buf;
	buf += sprintf(buf, "Acceptable characters: ");
	for (i = 0; i < sizeof(syms); ++i)
		sprintf(buf++, "%c", syms[i]);
	sprintf(buf, "\n");
	return strlen(buf_orig);
}

static void numdisp_print(struct numdisp_info *numdisp, const char *msg)
{
	int full_len = numdisp->width * numdisp->count;
	static char buf[sizeof(numdisp->text)];
	int len = strlen(msg);
	int i;

	if (len > full_len)
		len = full_len;

	memset(buf, ' ', full_len);
	buf[full_len] = '\0';
	strncpy(buf + (full_len - len), msg, len);

	for (i = 0; i < numdisp->width; ++i) {
		int j;
		for (j = 0; j < numdisp->count; ++j)
			numdisp_set(numdisp, (i % numdisp->width), buf[i + j * numdisp->width], false);
		schedule_timeout_interruptible(msecs_to_jiffies(1));
	}
}

static ssize_t numdisp_set_text(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct numdisp_info *numdisp = spi_get_drvdata(spi);
	int full_len = numdisp->width * numdisp->count;
	int len = strlen(buf) ? (strlen(buf) - 1) : 0;

	memset(numdisp->text, 0, sizeof(numdisp->text));
	if (len) {
		strncpy(numdisp->text, buf, (len > full_len) ? full_len : len);
	} else {
		numdisp_print(numdisp, "");
	}

	return count;
}

static DEVICE_ATTR(text, S_IWUSR | S_IRUGO, numdisp_get_text, numdisp_set_text);

static struct attribute *dev_attrs[] = {
	&dev_attr_text.attr,
	NULL
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int numdisp_task(void *param)
{
	struct spi_device *spi = (struct spi_device *)param;
	struct numdisp_info *numdisp = spi_get_drvdata(spi);

	set_freezable();

	while (!kthread_should_stop()) {
		if (strlen(numdisp->text))
			numdisp_print(numdisp, numdisp->text);
		else
			schedule_timeout_interruptible(msecs_to_jiffies(1000));

		try_to_freeze();
	}

	return 0;
}

static int numdisp_probe(struct spi_device *spi)
{
	struct device_node *node = spi->dev.of_node;
	struct numdisp_info *numdisp = devm_kzalloc(&spi->dev, sizeof(*numdisp), GFP_KERNEL);
	int err;

	err = of_property_read_u32(node, "width", &numdisp->width);
	if (err) {
		dev_err(&spi->dev, "Can't get width of LCD: %d\n", err);
		return err;
	}
	err = of_property_read_u32(node, "count", &numdisp->count);
	if (err) {
		numdisp->count = 1;
	}
	numdisp->le_gpio = devm_gpiod_get(&spi->dev, "le", GPIOD_OUT_LOW);
	numdisp->oe_gpio = devm_gpiod_get(&spi->dev, "oe", GPIOD_OUT_LOW);
	numdisp->spi = spi;

	if (!numdisp->le_gpio || !numdisp->oe_gpio) {
		dev_err(&spi->dev, "Can't request gpios\n");
		return -1;
	}

	spi_set_drvdata(spi, numdisp);

	err = sysfs_create_group(&spi->dev.kobj, &dev_attr_grp);

	if (err) {
		dev_err(&spi->dev, "Can't setup sysfs\n");
		return -1;
	}

	gpiod_set_value(numdisp->oe_gpio, 0);
	memset(numdisp->text, 0, sizeof(numdisp->text));

	kthread_run(numdisp_task, spi, DRIVER_NAME);

	return 0;
}

static int numdisp_remove(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &dev_attr_grp);
	return 0;
}

static const struct of_device_id numdisp_of_match[] = {
	{ .compatible = "emcraft,numeric_display", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, numdisp_of_match);

static struct spi_driver numdisp_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = numdisp_of_match,
	},
	.probe		= numdisp_probe,
	.remove		= numdisp_remove,
};

module_spi_driver(numdisp_driver);

MODULE_AUTHOR("Emcraft Systems");
MODULE_DESCRIPTION("7-Seg SPI Numeric LCD driver");
MODULE_LICENSE("GPL");
