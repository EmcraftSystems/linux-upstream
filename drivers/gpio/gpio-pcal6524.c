/*
 *  PCAL6524 24 bit I/O ports
 *
 *  Copyright (C) 2016 EmCraft Systems
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif

#define PCAL6524_INPUT			0x0
#define PCAL6524_OUTPUT			0x4
#define PCAL6524_INVERT			0x8
#define PCAL6524_DIRECTION		0xc
#define PCAL6524_DIRECTION_IN		1

struct pcal6524_chip {
	struct mutex i2c_lock;
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
};

static inline struct pcal6524_chip *to_pca(struct gpio_chip *gc)
{
	return container_of(gc, struct pcal6524_chip, gpio_chip);
}

static int pcal6524_write_reg(struct pcal6524_chip *chip, int reg, u8 val)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pcal6524_read_reg(struct pcal6524_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	*val = ret;

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	return 0;
}

static int pcal6524_gpio_get_value(struct gpio_chip *gc,
				   unsigned off)
{
	struct pcal6524_chip *chip = to_pca(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);

	ret = pcal6524_read_reg(chip, PCAL6524_INPUT + off / 8, &reg_val);
	if (ret < 0)
		goto exit;
	ret = !!(reg_val & (1 << (off % 8)));

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static void pcal6524_gpio_set_value(struct gpio_chip *gc,
				    unsigned off, int val)
{
	struct pcal6524_chip *chip = to_pca(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);

	ret = pcal6524_read_reg(chip, PCAL6524_OUTPUT + off / 8, &reg_val);
	if (ret < 0)
		goto exit;
	reg_val &= ~(1 << (off % 8));
	reg_val |= (!!val) << (off % 8);
	ret = pcal6524_write_reg(chip, PCAL6524_OUTPUT + off / 8, reg_val);

exit:
	mutex_unlock(&chip->i2c_lock);
}

static int pcal6524_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pcal6524_chip *chip = to_pca(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);

	ret = pcal6524_read_reg(chip, PCAL6524_DIRECTION + off / 8, &reg_val);
	if (ret < 0)
		goto exit;
	reg_val |= PCAL6524_DIRECTION_IN << (off % 8);
	ret = pcal6524_write_reg(chip, PCAL6524_DIRECTION + off / 8, reg_val);

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pcal6524_gpio_direction_output(struct gpio_chip *gc,
					 unsigned off, int val)
{
	struct pcal6524_chip *chip = to_pca(gc);
	u8 reg_val;
	int ret;

	pcal6524_gpio_set_value(gc, off, val);

	mutex_lock(&chip->i2c_lock);

	ret = pcal6524_read_reg(chip, PCAL6524_DIRECTION + off / 8, &reg_val);
	if (ret < 0)
		goto exit;
	reg_val &= ~(PCAL6524_DIRECTION_IN << (off % 8));
	ret = pcal6524_write_reg(chip, PCAL6524_DIRECTION + off / 8, reg_val);

exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static void pcal6524_setup_gpio(struct pcal6524_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input = pcal6524_gpio_direction_input;
	gc->direction_output = pcal6524_gpio_direction_output;
	gc->get = pcal6524_gpio_get_value;
	gc->set = pcal6524_gpio_set_value;
	gc->can_sleep = true;

	gc->base = -1;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = NULL;
}

static int pcal6524_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct pcal6524_chip *chip;
	int ret, ngpios, i;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;
	ngpios = id->driver_data;
	mutex_init(&chip->i2c_lock);
	pcal6524_setup_gpio(chip, ngpios);

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		return ret;

	i2c_set_clientdata(client, chip);

	return 0;
}

static int pcal6524_remove(struct i2c_client *client)
{
	struct pcal6524_chip *chip = i2c_get_clientdata(client);

	gpiochip_remove(&chip->gpio_chip);

	return 0;
}

static const struct i2c_device_id pcal6524_id[] = {
	{ "pcal6524", 24 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcal6524_id);

static const struct of_device_id pcal6524_dt_ids[] = {
	{ .compatible = "nxp,pcal6524", },
	{ }
};

MODULE_DEVICE_TABLE(of, pcal6524_dt_ids);

static struct i2c_driver pcal6524_i2c_driver = {
	.driver = {
		.name	= "pcal6524",
		.of_match_table = pcal6524_dt_ids,
	},
	.probe		= pcal6524_probe,
	.remove		= pcal6524_remove,
	.id_table	= pcal6524_id,
};

module_i2c_driver(pcal6524_i2c_driver);

MODULE_AUTHOR("Sergei Miroshnichenko <sergeimir@emcraft.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCAL6524");
MODULE_LICENSE("GPL");
