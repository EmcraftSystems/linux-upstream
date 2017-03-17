/*
 * nxp_ntag.c - handle NXP NT3H2211 NFC tag
 *
 * Copyright (C) 2016 Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/mod_devicetable.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/i2c.h>


#define NTAG_PAGE_SIZE		16
#define NTAG_BANK1_OFFSET	0x0
#define NTAG_BANK1_SIZE		59
#define NTAG_BANK2_OFFSET	0x40
#define NTAG_BANK2_SIZE		64
#define NTAG_SRAM_OFFSET	0xF8
#define NTAG_SRAM_SIZE		4

struct ntag_bank {
	const char *name;
	int offset;
	int size;
};

static struct ntag_bank ntag_banks [] = {
	{
		.name = "bank1",
		.offset = NTAG_BANK1_OFFSET,
		.size = NTAG_BANK1_SIZE,
	},
	{
		.name = "bank2",
		.offset = NTAG_BANK2_OFFSET,
		.size = NTAG_BANK2_SIZE,
	},
	{
		.name = "sram",
		.offset = NTAG_SRAM_OFFSET,
		.size = NTAG_SRAM_SIZE,
	},
};

struct ntag_data {
	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;
	struct bin_attribute bin;

	int num_banks;
	struct bin_attribute *bin_banks;

	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct i2c_client *client;
};

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 10000;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

static const struct i2c_device_id ntag_ids[] = {
	{ "nxp-nfc-tag", 0 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, ntag_ids);

/*-------------------------------------------------------------------------*/

static ssize_t ntag_page_read(struct ntag_data *ntag, char *buf,
		unsigned offset, size_t count)
{
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	struct i2c_client *client;
	unsigned long timeout, read_time;
	int status, i;

	memset(msg, 0, sizeof(msg));

	client = ntag->client;

	if (count > NTAG_PAGE_SIZE)
		count = NTAG_PAGE_SIZE;

	i = 0;
	msgbuf[i++] = offset;

	msg[0].addr = client->addr;
	msg[0].buf = msgbuf;
	msg[0].len = i;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = count;

	/*
	 * Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
		status = i2c_transfer(client->adapter, msg, 2);

		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == 2) {
			status = count;
			return count;
		}
		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t ntag_read(struct ntag_data *ntag,
			 char *buf, loff_t off, size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&ntag->lock);

	if (off % NTAG_PAGE_SIZE) {
		/* read unaligned page */
		ssize_t	status;
		char tmp[NTAG_PAGE_SIZE];
		status = ntag_page_read(ntag, tmp, off / NTAG_PAGE_SIZE, NTAG_PAGE_SIZE);
		if (status <= 0) {
			retval = status;
		} else {
			int sz = NTAG_PAGE_SIZE - (off % NTAG_PAGE_SIZE);
			memcpy(buf, tmp + (off % NTAG_PAGE_SIZE), count < sz ? count : sz);
			off += sz;
			buf += sz;
			count -= sz;
			retval += sz;
		}
	}

	while (count) {
		ssize_t	status;

		status = ntag_page_read(ntag, buf, off / NTAG_PAGE_SIZE, count < NTAG_PAGE_SIZE ? count : NTAG_PAGE_SIZE);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&ntag->lock);

	return retval;
}

static ssize_t ntag_bin_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct ntag_data *ntag;
	struct ntag_bank *ntag_bank = attr->private;

	ntag = dev_get_drvdata(container_of(kobj, struct device, kobj));
	off += (ntag_bank->offset * NTAG_PAGE_SIZE);

	return ntag_read(ntag, buf, off, count);
}

static ssize_t ntag_page_write(struct ntag_data *ntag, const char *buf,
		unsigned offset)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	char writebuf[NTAG_PAGE_SIZE + 1];
	ssize_t status = 0;
	unsigned long timeout, write_time;
	size_t count = NTAG_PAGE_SIZE;

	/* Get corresponding I2C address and adjust offset */
	client = ntag->client;

	/* If we'll use I2C calls for I/O, set up the message */
	msg.addr = client->addr;
	msg.flags = 0;

	/* msg.buf is u8 and casts will mask the values */
	msg.buf = writebuf;

	msg.buf[0] = offset;
	memcpy(&msg.buf[1], buf, NTAG_PAGE_SIZE);
	msg.len = 1 + NTAG_PAGE_SIZE;

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;

		status = i2c_transfer(client->adapter, &msg, 1);

		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == 1)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(write_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t ntag_write(struct ntag_data *ntag, const char *buf, loff_t off,
			  size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	if (off % NTAG_PAGE_SIZE || count < NTAG_PAGE_SIZE) {
		return -EINVAL;
	}

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&ntag->lock);

	while (count) {
		ssize_t	status;

		if (count < NTAG_PAGE_SIZE) {
			status = -EINVAL;
		} else {
			status = ntag_page_write(ntag, buf, off / NTAG_PAGE_SIZE);
		}

		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}

		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&ntag->lock);

	return retval;
}

static ssize_t ntag_bin_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct ntag_data *ntag;
	struct ntag_bank *ntag_bank = attr->private;

	ntag = dev_get_drvdata(container_of(kobj, struct device, kobj));
	off += (ntag_bank->offset * NTAG_PAGE_SIZE);

	return ntag_write(ntag, buf, off, count);
}

/*-------------------------------------------------------------------------*/

static int ntag_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ntag_data *ntag;
	int err;
	int i;

	ntag = devm_kzalloc(&client->dev, sizeof(struct ntag_data), GFP_KERNEL);
	if (!ntag)
		return -ENOMEM;

	mutex_init(&ntag->lock);

	ntag->client = client;

	ntag->num_banks = ARRAY_SIZE(ntag_banks);
	ntag->bin_banks = devm_kzalloc(&client->dev, ntag->num_banks * sizeof(struct bin_attribute), GFP_KERNEL);
	if (!ntag->bin_banks) {
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(ntag_banks); i++) {
		sysfs_bin_attr_init(&ntag->bin_banks[i]);
		ntag->bin_banks[i].attr.name = ntag_banks[i].name;
		ntag->bin_banks[i].private = &ntag_banks[i];
		ntag->bin_banks[i].attr.mode = S_IRUSR | S_IWUSR;
		ntag->bin_banks[i].read = ntag_bin_read;
		ntag->bin_banks[i].write = ntag_bin_write;
		ntag->bin_banks[i].size = ntag_banks[i].size * NTAG_PAGE_SIZE;

		err = sysfs_create_bin_file(&client->dev.kobj, &ntag->bin_banks[i]);
		if (err) {
			for ( ; i > 0; i--) {
				sysfs_remove_bin_file(&client->dev.kobj, &ntag->bin_banks[i - 1]);
			}
			return err;
		}
	}


	i2c_set_clientdata(client, ntag);

	return 0;
}

static int ntag_remove(struct i2c_client *client)
{
	struct ntag_data *ntag;

	ntag = i2c_get_clientdata(client);
	sysfs_remove_bin_file(&client->dev.kobj, &ntag->bin);

	return 0;
}
/*-------------------------------------------------------------------------*/

static struct i2c_driver ntag_driver = {
	.driver = {
		.name = "nxp_ntag",
		.owner = THIS_MODULE,
	},
	.probe = ntag_probe,
	.remove = ntag_remove,
	.id_table = ntag_ids,
};

static int __init ntag_init(void)
{
	return i2c_add_driver(&ntag_driver);
}
module_init(ntag_init);

static void __exit ntag_exit(void)
{
	i2c_del_driver(&ntag_driver);
}
module_exit(ntag_exit);

MODULE_DESCRIPTION("Driver for NXP NT3H2211 NFC tag");
MODULE_AUTHOR("Vladimir Skvortsov");
MODULE_LICENSE("GPL");
