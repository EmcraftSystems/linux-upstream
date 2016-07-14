/*
 * Frame buffer driver for Sitronix ST7529 LCD controller
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
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/freezer.h>
#include <linux/dma-mapping.h>

#define DRIVER_NAME		"st7529"

#define RAM_WIDTH		256
#define RAM_HEIGHT		144

#define A0_CMD_MODE		0
#define A0_DATA_MODE		1

#define CMD_EXT_IN		0x30
#define CMD_EXT_OUT		0x31
#define CMD_SLEEP_OUT		0x94
#define CMD_SLEEP_IN		0x95
#define CMD_OSC_ON		0xD1
#define CMD_PWRCTRL		0x20
#define ARG_PWRCTRL_VB		0x8
#define ARG_PWRCTRL_VF		0x2
#define ARG_PWRCTRL_VR		0x1
#define CMD_VOLCTRL		0x81
#define VOLCTRL_DEFAULT		300
#define CMD_DIS_NORMAL		0xA6
#define CMD_DISPLAY_ON		0xAF
#define CMD_DISPLAY_OFF		0xAE
#define CMD_DISCTRL		0xCA
#define ARG_DISCTRL_CLK_RATIO_1	0
#define ARG_DISCTRL_DUTY_1_144	0x23
#define ARG_DISCTRL_INVERSE_SET	9
#define CMD_COMSCN		0xBB
#define ARG_COMSCN_79_0_80_159	0x2
#define CMD_DATSDR		0xBC
#define ARG_DATSDR_COLUMN_NORM	0
#define ARG_DATSDR_ARRG_1_2_3	0
#define ARG_DATSDR_GRAY_3_BYTES	2
#define CMD_SWINT		0x34
#define CMD_ANASET		0x32
#define ARG_ANASET_OCS_12K	0x0
#define ARG_ANASET_BOOSTER_6K	0x1
#define ARG_ANASET_BIAS_1_12	0x2
#define CMD_CASET		0x15
#define CMD_LASET		0x75
#define CMD_RAMWR		0x5C
#define CMD_SET_GRAY_1		0x20
#define CMD_SET_GRAY_2		0x21

struct mfb_info
{
	struct spi_device	*spi;
	struct task_struct	*task;
	int			width;
	int			height;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*a0_gpio;
	int			spi_mode;
	unsigned int		volctrl;
};

static struct fb_ops st7529_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int st7529_data(struct mfb_info *mfbi, unsigned char *buf, int size)
{
	int err = 0;

	if (size) {
		if (mfbi->spi_mode == 8) {
			gpiod_set_value(mfbi->a0_gpio, A0_DATA_MODE);
		}
		err = spi_write(mfbi->spi, buf, size);
	}

	if (err)
		dev_err(&mfbi->spi->dev, "Failed to write data: %d\n", err);

	return err;
}

static int st7529_command(struct mfb_info *mfbi, unsigned char cmd,
			  int args_cnt, ...)
{
	int data_size = mfbi->spi_mode == 8 ? args_cnt : args_cnt * 2;
	unsigned char buf[data_size ? data_size : 2];
	int err;

	buf[0] = cmd;
	if (mfbi->spi_mode == 8) {
		gpiod_set_value(mfbi->a0_gpio, A0_CMD_MODE);
		err = spi_write(mfbi->spi, buf, 1);
	} else {
		buf[1] = 0;
		err = spi_write(mfbi->spi, buf, 2);
	}

	if (!err && args_cnt) {
		va_list ap;
		int i;
		va_start(ap, args_cnt);

		for (i = 0; i < args_cnt; ++i) {
			if (mfbi->spi_mode == 8) {
				buf[i] = va_arg(ap, unsigned int);
			} else {
				buf[i * 2] = va_arg(ap, unsigned int);
				buf[i * 2 + 1] = 1;
			}

			if (err)
				break;
		}

		va_end(ap);

		err = st7529_data(mfbi, buf, data_size);
	}

	if (err)
		dev_err(&mfbi->spi->dev, "Failed to write command 0x%x: %d\n", cmd, err);

	return err;
}

static void st7529_set_voltage(struct mfb_info *mfbi, unsigned int voltage) {
	u8 v_down = voltage & 0x3F;
	u8 v_up = (voltage >> 6) & 0x7;

	st7529_command(mfbi, CMD_VOLCTRL, 2, v_down, v_up);
}

static void st7529_hw_init(struct mfb_info *mfbi)
{
	if (gpiod_cansleep(mfbi->reset_gpio)) {
		gpiod_set_value_cansleep(mfbi->reset_gpio, 0);
		msleep(150);
		gpiod_set_value_cansleep(mfbi->reset_gpio, 1);
		msleep(150);
	} else {
		gpiod_set_value(mfbi->reset_gpio, 0);
		msleep(150);
		gpiod_set_value(mfbi->reset_gpio, 1);
		msleep(150);
	}

	st7529_command(mfbi, CMD_EXT_IN, 0);
	st7529_command(mfbi, CMD_SLEEP_OUT, 0);
	st7529_command(mfbi, CMD_OSC_ON, 0);
	/* Power Control Set: Booster must be on first */
	st7529_command(mfbi, CMD_PWRCTRL, 1, ARG_PWRCTRL_VB);
	msleep(2);
	st7529_command(mfbi, CMD_PWRCTRL, 1,
		       ARG_PWRCTRL_VB | ARG_PWRCTRL_VF | ARG_PWRCTRL_VR);
	st7529_set_voltage(mfbi, mfbi->volctrl);
	st7529_command(mfbi, CMD_DISCTRL, 3,
		       ARG_DISCTRL_CLK_RATIO_1,
		       ARG_DISCTRL_DUTY_1_144,
		       ARG_DISCTRL_INVERSE_SET);
	st7529_command(mfbi, CMD_DIS_NORMAL, 0);
	st7529_command(mfbi, CMD_COMSCN, 1,
		       ARG_COMSCN_79_0_80_159);
	st7529_command(mfbi, CMD_DATSDR, 3,
		       ARG_DATSDR_COLUMN_NORM,
		       ARG_DATSDR_ARRG_1_2_3,
		       ARG_DATSDR_GRAY_3_BYTES);
	st7529_command(mfbi, CMD_EXT_OUT, 0);
	st7529_command(mfbi, CMD_ANASET, 3,
		       ARG_ANASET_OCS_12K,
		       ARG_ANASET_BOOSTER_6K,
		       ARG_ANASET_BIAS_1_12);
	st7529_command(mfbi, CMD_SWINT, 0);
	st7529_command(mfbi, CMD_SET_GRAY_1, 16,
		       0x0,  0x2,  0x4,  0x6,
		       0x8,  0xA,  0xC,  0xE,
		       0x10, 0x12, 0x14, 0x16,
		       0x18, 0x1A, 0x1C, 0x1F);
	st7529_command(mfbi, CMD_SET_GRAY_2, 16,
		       0x0,  0x2,  0x4,  0x6,
		       0x8,  0xA,  0xC,  0xE,
		       0x10, 0x12, 0x14, 0x16,
		       0x18, 0x1A, 0x1C, 0x1F);
	st7529_command(mfbi, CMD_EXT_IN, 0);
	st7529_command(mfbi, CMD_DISPLAY_ON, 0);
}

static int st7529_task(void *param)
{
	struct fb_info *fb_info = (struct fb_info *)param;
	struct mfb_info *mfbi = fb_info->par;
	int buf_size = mfbi->spi_mode == 8
		? mfbi->width * mfbi->height
		: mfbi->width * mfbi->height * 2;
	u8 *buf = devm_kzalloc(&mfbi->spi->dev, buf_size, GFP_KERNEL);
	u8 *rgb_buf = fb_info->screen_base;

	set_freezable();

	while (!kthread_should_stop()) {
		int i;

		for (i = 0; i < mfbi->width * mfbi->height; ++i) {
			u8 *rgb_pix	= &rgb_buf[i*3];
			u8 red		= rgb_pix[2];
			u8 green	= rgb_pix[1];
			u8 blue		= rgb_pix[0];

			/* Formula:
			 * gray = 0.2125*r + 0.7152*g + 0.0722*b
			 */
			if (mfbi->spi_mode == 8) {
				buf[i] = ((red * 54) + (green * 183) + (blue * 18)) >> 8;
			} else {
				buf[i * 2] = ((red * 54) + (green * 183) + (blue * 18)) >> 8;
				buf[i * 2 + 1] = 1;
			}
		}

		st7529_command(mfbi, CMD_EXT_IN, 0);
		st7529_command(mfbi, CMD_CASET, 2,
			       0, (mfbi->width / 3) - 1);
		st7529_command(mfbi, CMD_LASET, 2,
			       RAM_HEIGHT - mfbi->height, RAM_HEIGHT);
		st7529_command(mfbi, CMD_RAMWR, 0);
		st7529_data(mfbi, buf, buf_size);

		try_to_freeze();
		schedule_timeout_interruptible(msecs_to_jiffies(1000));
	}

	devm_kfree(&mfbi->spi->dev, buf);

	return 0;
}

static ssize_t st7529_get_volctrl(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fb_info *fb_info = spi_get_drvdata(spi);
	struct mfb_info *mfbi = fb_info->par;

	return sprintf(buf, "%d\n", mfbi->volctrl);
}

static ssize_t st7529_set_volctrl(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fb_info *fb_info = spi_get_drvdata(spi);
	struct mfb_info *mfbi = fb_info->par;
	int volctrl = 0;

	if (1 == sscanf(buf, "%d", &volctrl)) {
		mfbi->volctrl = volctrl;
		st7529_set_voltage(mfbi, volctrl);
	}

	return count;
}

static DEVICE_ATTR(volctrl, S_IWUSR | S_IRUGO, st7529_get_volctrl, st7529_set_volctrl);

static struct attribute *dev_attrs[] = {
	&dev_attr_volctrl.attr,
	NULL
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int st7529_probe(struct spi_device *spi)
{
	int xres = -1, yres = -1, bits_per_pixel = -1, spi_mode = -1;
	struct device_node *node = spi->dev.of_node;
	struct fb_info *fb_info = NULL;
	struct mfb_info *mfbi = NULL;
	int err = 0;
	dma_addr_t dma_handle;
	u8 save;

	if (NULL == node) {
		dev_err(&spi->dev, "No device info\n");
		return -ENODEV;
	}

	if (of_property_read_u32(node, "spi_bits_per_word", &spi_mode)) {
		dev_err(&spi->dev, "failed to get SPI bits per word mode\n");
		return -EINVAL;
	}

	if (spi_mode != 8 && spi_mode != 9) {
		dev_err(&spi->dev, "Incorrect spi bits per word mode: %d."
			" Valid values are 8 or 9.\n", spi_mode);
		return -EINVAL;
	}

	/* Setup SPI */
	save = spi->bits_per_word;
	spi->bits_per_word = spi_mode;
	err = spi_setup(spi);
	if (err < 0) {
		spi->bits_per_word = save;
		dev_err(&spi->dev, "failed to setup SPI\n");
		return err;
	}

	if (of_property_read_u32(node, "xres", &xres))
		dev_err(&spi->dev, "failed to get x-resolution\n");
	if (of_property_read_u32(node, "yres", &yres))
		dev_err(&spi->dev, "failed to get y-resolution\n");
	if (of_property_read_u32(node, "bits_per_pixel", &bits_per_pixel))
		dev_err(&spi->dev, "failed to get color depth\n");

	if (xres < 0 || yres < 0 || bits_per_pixel < 0)
		return -EINVAL;

	fb_info = framebuffer_alloc(sizeof(struct mfb_info), &spi->dev);
	if (NULL == fb_info) {
		dev_err(&spi->dev, "Failed to allocate framebuffer info\n");
		return -ENOMEM;
	}

	fb_info->var.xres		= xres;
	fb_info->var.yres		= yres;
	fb_info->var.bits_per_pixel	= bits_per_pixel;
	fb_info->var.xres_virtual	= fb_info->var.xres;
	fb_info->var.yres_virtual	= fb_info->var.yres;
	fb_info->fix.line_length	= fb_info->var.xres * fb_info->var.bits_per_pixel / 8;
	fb_info->screen_size		= fb_info->fix.line_length * fb_info->var.yres_virtual;
	fb_info->screen_base = dma_alloc_coherent(NULL, PAGE_ALIGN(fb_info->screen_size), &dma_handle, GFP_KERNEL);

	if (NULL == fb_info->screen_base) {
		dev_err(&spi->dev, "Failed to allocate framebuffer\n");
		err = -ENOMEM;
		goto failed_alloc_framebuffer;
	}

	strcpy(fb_info->fix.id, DRIVER_NAME);

	fb_info->fbops			= &st7529_ops;
	fb_info->var.grayscale		= 0;
	fb_info->var.vmode		= FB_VMODE_NONINTERLACED;
	fb_info->var.red.length		= 8;
	fb_info->var.red.offset		= 16;
	fb_info->var.green.length	= 8;
	fb_info->var.green.offset	= 8;
	fb_info->var.blue.length	= 8;
	fb_info->var.blue.offset	= 0;
	fb_info->var.transp.length	= 0;
	fb_info->var.transp.offset	= 0;
	fb_info->fix.type		= FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual		= FB_VISUAL_TRUECOLOR;
	fb_info->fix.accel		= FB_ACCEL_NONE;
	fb_info->fix.smem_start		= dma_handle;
	fb_info->fix.smem_len		= fb_info->screen_size;

	mfbi				= fb_info->par;
	mfbi->spi			= spi;
	mfbi->width			= fb_info->var.xres;
	mfbi->height			= fb_info->var.yres;
	mfbi->volctrl			= VOLCTRL_DEFAULT;
	mfbi->spi_mode = spi_mode;

	mfbi->reset_gpio = devm_gpiod_get(&spi->dev,
					  "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(mfbi->reset_gpio)) {
		err = mfbi->reset_gpio ? PTR_ERR(mfbi->reset_gpio) : -EINVAL;
		dev_err(&spi->dev, "Failed to get reset gpio: %d\n", err);
		goto failed_install_fb;
	}

	if (mfbi->spi_mode == 8) {
		mfbi->a0_gpio = devm_gpiod_get(&spi->dev,
					       "a0", GPIOD_OUT_HIGH);
		if (IS_ERR_OR_NULL(mfbi->a0_gpio)) {
			err = mfbi->a0_gpio ? PTR_ERR(mfbi->a0_gpio) : -EINVAL;
			dev_err(&spi->dev, "Failed to get A0 gpio: %d\n", err);
			goto failed_install_fb;
		}
	}

	st7529_hw_init(mfbi);

	err = register_framebuffer(fb_info);
	if (err < 0) {
		dev_err(&spi->dev, "Failed to register framebuffer\n");
		goto failed_install_fb;
	}

	mfbi->task = kthread_run(st7529_task, fb_info, DRIVER_NAME);

	err = sysfs_create_group(&spi->dev.kobj, &dev_attr_grp);

	spi_set_drvdata(spi, fb_info);

	return 0;

failed_install_fb:
	devm_kfree(&spi->dev, fb_info->screen_base);
failed_alloc_framebuffer:
	framebuffer_release(fb_info);

	return err;
}

static int st7529_remove(struct spi_device *spi)
{
	struct fb_info *fb_info = spi_get_drvdata(spi);
	struct mfb_info *mfbi = fb_info->par;

	kthread_stop(mfbi->task);
	unregister_framebuffer(fb_info);
	devm_kfree(&spi->dev, fb_info->screen_base);
	framebuffer_release(fb_info);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int st7529_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fb_info *fb_info = spi_get_drvdata(spi);
	struct mfb_info *mfbi = fb_info->par;

	st7529_command(mfbi, CMD_DISPLAY_OFF, 0);
	st7529_command(mfbi, CMD_SLEEP_IN, 0);
	return 0;
}

static int st7529_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fb_info *fb_info = spi_get_drvdata(spi);
	struct mfb_info *mfbi = fb_info->par;

	st7529_command(mfbi, CMD_SLEEP_OUT, 0);
	st7529_command(mfbi, CMD_DISPLAY_ON, 0);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id st7529_of_match[] = {
	{ .compatible = "emcraft,st7529", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, st7529_of_match);

static const struct dev_pm_ops st7529_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st7529_suspend, st7529_resume)
};

static struct spi_driver st7529_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = st7529_of_match,
		.pm		= &st7529_pm_ops,
	},
	.probe		= st7529_probe,
	.remove		= st7529_remove,
};

module_spi_driver(st7529_driver);

MODULE_AUTHOR("Emcraft Systems");
MODULE_DESCRIPTION("Sitronix ST7529 framebuffer driver");
MODULE_LICENSE("GPL");
