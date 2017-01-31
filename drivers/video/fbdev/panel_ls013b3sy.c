/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * Sharp LS013B3SY 320x300 DSI LCD panel driver
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include "stm32_dsi.h"

#define LS013B3SY_CMD_TEEON				0x35
#define LS013B3SY_CMD_SLPOUT				0x11
#define LS013B3SY_CMD_DISPON				0x29

#define LS013B3SY_TEEON_TEP_ACTIVE_HIGH			(0 << 1)
#define LS013B3SY_TEEON_TEP_ACTIVE_LOW			(1 << 1)
#define LS013B3SY_TEEON_M_VBLANKING_INFO_ONLY		0
#define LS013B3SY_TEEON_M_VBLANKING_AND_HBLANKING_INFO	1

static const uint8_t ShortRegData1[] = {0xff, 0x10};
static const uint8_t ShortRegData2[] = {0xfb, 0x1};
static const uint8_t ShortRegData3[] = {0xb3, 0};
static const uint8_t ShortRegData4[] = {0xbb, 0x10};
static const uint8_t ShortRegData5[] = {0x3a, 0x6};
static const uint8_t ShortRegData6[] = {LS013B3SY_CMD_SLPOUT, 0x00};
static const uint8_t ShortRegData7[] = {LS013B3SY_CMD_DISPON, 0x00};

static const uint8_t lcdRegData1[]  = {0, 0x8, 0x8, 0x20, 0x20, 0x3b};

static const uint8_t ShortRegDataTE[] = {LS013B3SY_CMD_TEEON,
				  LS013B3SY_TEEON_M_VBLANKING_INFO_ONLY
				  | LS013B3SY_TEEON_TEP_ACTIVE_HIGH};

static int ls013b3sy_init(void *dsi, void (*writecmd)(void *dsi, int NbrParams, u8 *pParams), int orientation, bool te)
{
	/* According to LCM-15009 document */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 1, (uint8_t *)ShortRegData2);
	writecmd(dsi, 1, (uint8_t *)ShortRegData3);
	writecmd(dsi, 1, (uint8_t *)ShortRegData4);
	writecmd(dsi, 1, (uint8_t *)ShortRegData5);

	writecmd(dsi, 5, (uint8_t *)lcdRegData1);

	if (te)
		writecmd(dsi, 1, (uint8_t *)ShortRegDataTE);

	writecmd(dsi, 1, (uint8_t *)ShortRegData6);
	writecmd(dsi, 1, (uint8_t *)ShortRegData7);

	(void)orientation;
	return 0;
}

static struct stm32_dsi_panel ls013b3sy_panel = {
	.init = ls013b3sy_init,
};

static int ls013b3sy_probe(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, &ls013b3sy_panel);
	return 0;
}

static const struct of_device_id ls013b3sy_dt_ids[] = {
	{ .compatible = "sharp,ls013b3sy", .data = &ls013b3sy_panel },
	{ /* sentinel */ },
};

static struct platform_driver ls013b3sy_driver = {
	.probe = ls013b3sy_probe,
	.driver = {
		.name = "ls013b3sy",
		.of_match_table = of_match_ptr(ls013b3sy_dt_ids),
	},
};

module_platform_driver(ls013b3sy_driver);
