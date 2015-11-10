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
#include <linux/freezer.h>

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
#define ARG_VOLCTRL_12V_UP	0x38
#define ARG_VOLCTRL_12V_DOWN	0x4
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
	unsigned char		*buffer;
	struct task_struct	*task;
	int			width;
	int			height;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*a0_gpio;
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
		gpiod_set_value(mfbi->a0_gpio, A0_DATA_MODE);
		err = spi_write(mfbi->spi, buf, size);
	}

	if (err)
		dev_err(&mfbi->spi->dev, "Failed to write data: %d\n", err);

	return err;
}

static int st7529_command(struct mfb_info *mfbi, unsigned char cmd,
			  int args_cnt, ...)
{
	unsigned char buf[1 + args_cnt];
	int err;

	buf[0] = cmd;
	gpiod_set_value(mfbi->a0_gpio, A0_CMD_MODE);
	err = spi_write(mfbi->spi, buf, 1);

	if (!err && args_cnt) {
		va_list ap;
		int i;
		va_start(ap, args_cnt);

		for (i = 0; i < args_cnt; ++i) {
			buf[i] = va_arg(ap, unsigned int);

			if (err)
				break;
		}

		va_end(ap);

		err = st7529_data(mfbi, buf, args_cnt);
	}

	if (err)
		dev_err(&mfbi->spi->dev, "Failed to write command 0x%x: %d\n", cmd, err);

	return err;
}

static void st7529_hw_init(struct mfb_info *mfbi)
{
	gpiod_set_value(mfbi->reset_gpio, 1);
	msleep(150);
	gpiod_set_value(mfbi->reset_gpio, 0);
	msleep(150);

	st7529_command(mfbi, CMD_EXT_IN, 0);
	st7529_command(mfbi, CMD_SLEEP_OUT, 0);
	st7529_command(mfbi, CMD_OSC_ON, 0);
	/* Power Control Set: Booster must be on first */
	st7529_command(mfbi, CMD_PWRCTRL, 1, ARG_PWRCTRL_VB);
	msleep(2);
	st7529_command(mfbi, CMD_PWRCTRL, 1,
		       ARG_PWRCTRL_VB | ARG_PWRCTRL_VF | ARG_PWRCTRL_VR);
	st7529_command(mfbi, CMD_VOLCTRL, 2,
		       ARG_VOLCTRL_12V_UP,
		       ARG_VOLCTRL_12V_DOWN);
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
	struct mfb_info *mfbi = (struct mfb_info*)param;

	set_freezable();

	while (!kthread_should_stop()) {
		st7529_command(mfbi, CMD_EXT_IN, 0);
		st7529_command(mfbi, CMD_CASET, 2,
			       0, (mfbi->width / 3) - 1);
		st7529_command(mfbi, CMD_LASET, 2,
			       RAM_HEIGHT - mfbi->height, RAM_HEIGHT);
		st7529_command(mfbi, CMD_RAMWR, 0);
		st7529_data(mfbi, mfbi->buffer, mfbi->width * mfbi->height);

		try_to_freeze();
		schedule_timeout_interruptible(msecs_to_jiffies(1000));
	}

	return 0;
}

static int st7529_probe(struct spi_device *spi)
{
	int xres = -1, yres = -1, bits_per_pixel = -1, grayscale = -1;
	struct device_node *node = spi->dev.of_node;
	struct fb_info *fb_info = NULL;
	int nres_gpio, a0_gpio;
	struct mfb_info *mfbi = NULL;
	int err = 0;

	if (NULL == node) {
		dev_err(&spi->dev, "No device info\n");
		return -ENODEV;
	}

	if (of_property_read_u32(node, "xres", &xres))
		dev_err(&spi->dev, "failed to get x-resolution\n");
	if (of_property_read_u32(node, "yres", &yres))
		dev_err(&spi->dev, "failed to get y-resolution\n");
	if (of_property_read_u32(node, "bits_per_pixel", &bits_per_pixel))
		dev_err(&spi->dev, "failed to get color depth\n");
	if (of_property_read_u32(node, "grayscale", &grayscale))
		dev_err(&spi->dev, "failed to get if LCD is grayscale\n");

	if (xres < 0 || yres < 0 || bits_per_pixel < 0 || grayscale < 0)
		return -EINVAL;

	nres_gpio = of_get_named_gpio(node, "nres-gpios", 0);
	if (!gpio_is_valid(nres_gpio)) {
		dev_err(&spi->dev, "Invalid nReset gpio %d\n", nres_gpio);
		return -EINVAL;
	}

	a0_gpio = of_get_named_gpio(node, "a0-gpios", 0);
	if (!gpio_is_valid(a0_gpio)) {
		dev_err(&spi->dev, "Invalid A0 gpio %d\n", a0_gpio);
		return -EINVAL;
	}

	err = devm_gpio_request_one(&spi->dev, nres_gpio,
				    GPIOF_OUT_INIT_HIGH, "lcd_nreset");
	if (err) {
		dev_err(&spi->dev, "Failed to request nReset gpio %d: %d\n",
			nres_gpio, err);
		return err;
	}

	err = devm_gpio_request_one(&spi->dev, a0_gpio,
				    GPIOF_OUT_INIT_HIGH, "lcd_a0");
	if (err) {
		dev_err(&spi->dev, "Failed to request A0 gpio %d: %d\n",
			a0_gpio, err);
		return err;
	}

	fb_info = framebuffer_alloc(sizeof(struct mfb_info), &spi->dev);
	if (NULL == fb_info) {
		dev_err(&spi->dev, "Failed to allocate framebuffer info\n");
		return -ENOMEM;
	}

	fb_info->screen_size = xres * yres;
	fb_info->screen_base = devm_kzalloc(&spi->dev, fb_info->screen_size, GFP_KERNEL);
	if (NULL == fb_info->screen_base) {
		dev_err(&spi->dev, "Failed to allocate framebuffer\n");
		err = -ENOMEM;
		goto failed_alloc_framebuffer;
	}

	strcpy(fb_info->fix.id, DRIVER_NAME);

	fb_info->fbops			= &st7529_ops;
	fb_info->var.xres		= xres;
	fb_info->var.yres		= yres;
	fb_info->var.bits_per_pixel	= bits_per_pixel;
	fb_info->var.grayscale		= grayscale;
	fb_info->var.xres_virtual	= fb_info->var.xres;
	fb_info->var.yres_virtual	= fb_info->var.yres;
	fb_info->var.vmode		= FB_VMODE_NONINTERLACED;
	fb_info->var.red.length		= 8;
	fb_info->var.green.length	= 8;
	fb_info->var.blue.length	= 8;
	fb_info->var.transp.length	= 8;
	fb_info->fix.type		= FB_TYPE_PACKED_PIXELS;
	fb_info->fix.visual		= FB_VISUAL_TRUECOLOR;
	fb_info->fix.accel		= FB_ACCEL_NONE;
	fb_info->fix.line_length	= fb_info->var.xres * fb_info->var.bits_per_pixel / 8;
	fb_info->fix.smem_start		= virt_to_phys(fb_info->screen_base);
	fb_info->fix.smem_len		= fb_info->screen_size;

	mfbi				= fb_info->par;
	mfbi->spi			= spi;
	mfbi->buffer			= fb_info->screen_base;
	mfbi->width			= fb_info->var.xres;
	mfbi->height			= fb_info->var.yres;

	mfbi->reset_gpio = devm_gpiod_get(&spi->dev,
					  "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(mfbi->reset_gpio)) {
		err = mfbi->reset_gpio ? PTR_ERR(mfbi->reset_gpio) : -EINVAL;
		dev_err(&spi->dev, "Failed to get reset gpio: %d\n", err);
		goto failed_install_fb;
	}

	mfbi->a0_gpio = devm_gpiod_get(&spi->dev,
				       "a0", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(mfbi->a0_gpio)) {
		err = mfbi->a0_gpio ? PTR_ERR(mfbi->a0_gpio) : -EINVAL;
		dev_err(&spi->dev, "Failed to get A0 gpio: %d\n", err);
		goto failed_install_fb;
	}

	st7529_hw_init(mfbi);

	err = register_framebuffer(fb_info);
	if (err < 0) {
		dev_err(&spi->dev, "Failed to register framebuffer\n");
		goto failed_install_fb;
	}

	mfbi->task = kthread_run(st7529_task, mfbi, DRIVER_NAME);

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
