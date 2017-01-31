/*
 * Copyright (C) 2016 STMicroelectronics
 * MCD Application Team <www.st.com>
 *
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * Orise Tech OTM8009A LCD panel driver
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include "stm32_dsi.h"

#define OTM8009A_FORMAT_RGB888		((uint32_t)0x00) /* Pixel format chosen is RGB888 : 24 bpp */
#define OTM8009A_FORMAT_RBG565		((uint32_t)0x02) /* Pixel format chosen is RGB565 : 16 bpp */

#define OTM8009A_CMD_NOP		0x00  /* NOP command */
#define OTM8009A_CMD_SWRESET		0x01  /* Sw reset command */
#define OTM8009A_CMD_RDDMADCTL		0x0B  /* Read Display MADCTR command : read memory display access ctrl */
#define OTM8009A_CMD_RDDCOLMOD		0x0C  /* Read Display pixel format */
#define OTM8009A_CMD_SLPIN		0x10  /* Sleep In command */
#define OTM8009A_CMD_SLPOUT		0x11  /* Sleep Out command */
#define OTM8009A_CMD_PTLON		0x12  /* Partial mode On command */

#define OTM8009A_CMD_DISPOFF		0x28  /* Display Off command */
#define OTM8009A_CMD_DISPON		0x29  /* Display On command */

#define OTM8009A_CMD_CASET		0x2A  /* Column address set command */
#define OTM8009A_CMD_PASET		0x2B  /* Page address set command */

#define OTM8009A_CMD_RAMWR		0x2C  /* Memory (GRAM) write command */
#define OTM8009A_CMD_RAMRD		0x2E  /* Memory (GRAM) read command */

#define OTM8009A_CMD_PLTAR		0x30  /* Partial area command (4 parameters) */

#define OTM8009A_CMD_TEOFF		0x34  /* Tearing Effect Line Off command : command with no parameter */
#define OTM8009A_CMD_TEEON		0x35  /* Tearing Effect Line On command : command with 1 parameter 'TELOM' */

#define OTM8009A_TEEON_TELOM_VBLANKING_INFO_ONLY		0x00
#define OTM8009A_TEEON_TELOM_VBLANKING_AND_HBLANKING_INFO	0x01

#define OTM8009A_CMD_MADCTR		0x36  /* Memory Access write control command */

#define OTM8009A_MADCTR_MODE_PORTRAIT	0x00
#define OTM8009A_MADCTR_MODE_LANDSCAPE	0x60  /* MY = 0, MX = 1, MV = 1, ML = 0, RGB = 0 */

#define OTM8009A_CMD_IDMOFF		0x38  /* Idle mode Off command */
#define OTM8009A_CMD_IDMON		0x39  /* Idle mode On command */

#define OTM8009A_CMD_COLMOD		0x3A  /* Interface Pixel format command */

#define OTM8009A_COLMOD_RGB565		0x55
#define OTM8009A_COLMOD_RGB888		0x77

#define OTM8009A_CMD_RAMWRC		0x3C  /* Memory write continue command */
#define OTM8009A_CMD_RAMRDC		0x3E  /* Memory read continue command */

#define OTM8009A_CMD_WRTESCN		0x44  /* Write Tearing Effect Scan line command */
#define OTM8009A_CMD_RDSCNL		0x45  /* Read  Tearing Effect Scan line command */

#define OTM8009A_CMD_WRDISBV		0x51  /* Write Display Brightness command */
#define OTM8009A_CMD_WRCTRLD		0x53  /* Write CTRL Display command */
#define OTM8009A_CMD_WRCABC		0x55  /* Write Content Adaptive Brightness command */
#define OTM8009A_CMD_WRCABCMB		0x5E  /* Write CABC Minimum Brightness command */

static const uint8_t lcdRegData1[]  = {0x80,0x09,0x01,0xFF};
static const uint8_t lcdRegData2[]  = {0x80,0x09,0xFF};
static const uint8_t lcdRegData3[]  = {0x00,0x09,0x0F,0x0E,0x07,0x10,0x0B,0x0A,0x04,0x07,0x0B,0x08,0x0F,0x10,0x0A,0x01,0xE1};
static const uint8_t lcdRegData4[]  = {0x00,0x09,0x0F,0x0E,0x07,0x10,0x0B,0x0A,0x04,0x07,0x0B,0x08,0x0F,0x10,0x0A,0x01,0xE2};
static const uint8_t lcdRegData5[]  = {0x79,0x79,0xD8};
static const uint8_t lcdRegData6[]  = {0x00,0x01,0xB3};
static const uint8_t lcdRegData7[]  = {0x85,0x01,0x00,0x84,0x01,0x00,0xCE};
static const uint8_t lcdRegData8[]  = {0x18,0x04,0x03,0x39,0x00,0x00,0x00,0x18,0x03,0x03,0x3A,0x00,0x00,0x00,0xCE};
static const uint8_t lcdRegData9[]  = {0x18,0x02,0x03,0x3B,0x00,0x00,0x00,0x18,0x01,0x03,0x3C,0x00,0x00,0x00,0xCE};
static const uint8_t lcdRegData10[] = {0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x00,0xCF};
static const uint8_t lcdRegData11[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData12[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData13[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData14[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData15[] = {0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData16[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData17[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCB};
static const uint8_t lcdRegData18[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xCB};
static const uint8_t lcdRegData19[] = {0x00,0x26,0x09,0x0B,0x01,0x25,0x00,0x00,0x00,0x00,0xCC};
static const uint8_t lcdRegData20[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x0A,0x0C,0x02,0xCC};
static const uint8_t lcdRegData21[] = {0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC};
static const uint8_t lcdRegData22[] = {0x00,0x25,0x0C,0x0A,0x02,0x26,0x00,0x00,0x00,0x00,0xCC};
static const uint8_t lcdRegData23[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0B,0x09,0x01,0xCC};
static const uint8_t lcdRegData24[] = {0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC};
static const uint8_t lcdRegData25[] = {0xFF,0xFF,0xFF,0xFF};
/*
 * CASET value (Column Address Set) : X direction LCD GRAM boundaries
 * depending on LCD orientation mode and PASET value (Page Address Set) : Y direction
 * LCD GRAM boundaries depending on LCD orientation mode
 * XS[15:0] = 0x000 = 0, XE[15:0] = 0x31F = 799 for landscape mode : apply to CASET
 * YS[15:0] = 0x000 = 0, YE[15:0] = 0x31F = 799 for portrait mode : : apply to PASET
 */
static const uint8_t lcdRegData27[] = {0x00, 0x00, 0x03, 0x1F, OTM8009A_CMD_CASET};
/*
 * XS[15:0] = 0x000 = 0, XE[15:0] = 0x1DF = 479 for portrait mode : apply to CASET
 * YS[15:0] = 0x000 = 0, YE[15:0] = 0x1DF = 479 for landscape mode : apply to PASET
 */
static const uint8_t lcdRegData28[] = {0x00, 0x00, 0x01, 0xDF, OTM8009A_CMD_PASET};


static const uint8_t ShortRegData0[]  = {OTM8009A_CMD_SWRESET, 0x00};
static const uint8_t ShortRegData1[]  = {OTM8009A_CMD_NOP, 0x00};
static const uint8_t ShortRegData2[]  = {OTM8009A_CMD_NOP, 0x80};
static const uint8_t ShortRegData3[]  = {0xC4, 0x30};
static const uint8_t ShortRegData4[]  = {OTM8009A_CMD_NOP, 0x8A};
static const uint8_t ShortRegData5[]  = {0xC4, 0x40};
static const uint8_t ShortRegData6[]  = {OTM8009A_CMD_NOP, 0xB1};
static const uint8_t ShortRegData7[]  = {0xC5, 0xA9};
static const uint8_t ShortRegData8[]  = {OTM8009A_CMD_NOP, 0x91};
static const uint8_t ShortRegData9[]  = {0xC5, 0x34};
static const uint8_t ShortRegData10[] = {OTM8009A_CMD_NOP, 0xB4};
static const uint8_t ShortRegData11[] = {0xC0, 0x50};
static const uint8_t ShortRegData12[] = {0xD9, 0x4E};
static const uint8_t ShortRegData13[] = {OTM8009A_CMD_NOP, 0x81};
static const uint8_t ShortRegData14[] = {0xC1, 0x66};
static const uint8_t ShortRegData15[] = {OTM8009A_CMD_NOP, 0xA1};
static const uint8_t ShortRegData16[] = {0xC1, 0x08};
static const uint8_t ShortRegData17[] = {OTM8009A_CMD_NOP, 0x92};
static const uint8_t ShortRegData18[] = {0xC5, 0x01};
static const uint8_t ShortRegData19[] = {OTM8009A_CMD_NOP, 0x95};
static const uint8_t ShortRegData20[] = {OTM8009A_CMD_NOP, 0x94};
static const uint8_t ShortRegData21[] = {0xC5, 0x33};
static const uint8_t ShortRegData22[] = {OTM8009A_CMD_NOP, 0xA3};
static const uint8_t ShortRegData23[] = {0xC0, 0x1B};
static const uint8_t ShortRegData24[] = {OTM8009A_CMD_NOP, 0x82};
static const uint8_t ShortRegData25[] = {0xC5, 0x83};
static const uint8_t ShortRegData26[] = {0xC4, 0x83};
static const uint8_t ShortRegData27[] = {0xC1, 0x0E};
static const uint8_t ShortRegData28[] = {OTM8009A_CMD_NOP, 0xA6};
static const uint8_t ShortRegData29[] = {OTM8009A_CMD_NOP, 0xA0};
static const uint8_t ShortRegData30[] = {OTM8009A_CMD_NOP, 0xB0};
static const uint8_t ShortRegData31[] = {OTM8009A_CMD_NOP, 0xC0};
static const uint8_t ShortRegData32[] = {OTM8009A_CMD_NOP, 0xD0};
static const uint8_t ShortRegData33[] = {OTM8009A_CMD_NOP, 0x90};
static const uint8_t ShortRegData34[] = {OTM8009A_CMD_NOP, 0xE0};
static const uint8_t ShortRegData35[] = {OTM8009A_CMD_NOP, 0xF0};
static const uint8_t ShortRegData36[] = {OTM8009A_CMD_SLPOUT, 0x00};
static const uint8_t ShortRegData37[] = {OTM8009A_CMD_COLMOD, OTM8009A_COLMOD_RGB565};
static const uint8_t ShortRegData38[] = {OTM8009A_CMD_COLMOD, OTM8009A_COLMOD_RGB888};
static const uint8_t ShortRegData39[] = {OTM8009A_CMD_MADCTR, OTM8009A_MADCTR_MODE_LANDSCAPE};
static const uint8_t ShortRegData40[] = {OTM8009A_CMD_WRDISBV, 0x7F};
static const uint8_t ShortRegData41[] = {OTM8009A_CMD_WRCTRLD, 0x2C};
static const uint8_t ShortRegData42[] = {OTM8009A_CMD_WRCABC, 0x02};
static const uint8_t ShortRegData43[] = {OTM8009A_CMD_WRCABCMB, 0xFF};
static const uint8_t ShortRegData44[] = {OTM8009A_CMD_DISPON, 0x00};
static const uint8_t ShortRegData45[] = {OTM8009A_CMD_RAMWR, 0x00};
static const uint8_t ShortRegData46[] = {0xCF, 0x00};
static const uint8_t ShortRegData47[] = {0xC5, 0x66};
static const uint8_t ShortRegData48[] = {OTM8009A_CMD_NOP, 0xB6};
static const uint8_t ShortRegData49[] = {0xF5, 0x06};
static const uint8_t ShortRegDataTE[] = {OTM8009A_CMD_TEEON, OTM8009A_TEEON_TELOM_VBLANKING_INFO_ONLY};

static int otm8009a_init(void *dsi, void (*writecmd)(void *dsi, int NbrParams, u8 *pParams), int orientation, bool te)
{
	/*
	 * Enable CMD2 to access vendor specific commands
	 * Enter in command 2 mode and set EXTC to enable address shift function (0x00)
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 3, (uint8_t *)lcdRegData1);

	/* Enter ORISE Command 2 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData2); /* Shift address to 0x80 */
	writecmd(dsi, 2, (uint8_t *)lcdRegData2);

	/*
	 * SD_PCH_CTRL - 0xC480h - 129th parameter - Default 0x00
	 * Set SD_PT
	 * -> Source output level during porch and non-display area to GND
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData2);
	writecmd(dsi, 1, (uint8_t *)ShortRegData3);
	msleep(10);
	/* Not documented */
	writecmd(dsi, 1, (uint8_t *)ShortRegData4);
	writecmd(dsi, 1, (uint8_t *)ShortRegData5);
	msleep(10);

	/*
	 * PWR_CTRL4 - 0xC4B0h - 178th parameter - Default 0xA8
	 * Set gvdd_en_test
	 * -> enable GVDD test mode !!!
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData6);
	writecmd(dsi, 1, (uint8_t *)ShortRegData7);

	/*
	 * PWR_CTRL2 - 0xC590h - 146th parameter - Default 0x79
	 * Set pump 4 vgh voltage
	 * -> from 15.0v down to 13.0v
	 * Set pump 5 vgh voltage
	 * -> from -12.0v downto -9.0v
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData8);
	writecmd(dsi, 1, (uint8_t *)ShortRegData9);

	/* P_DRV_M - 0xC0B4h - 181th parameter - Default 0x00
	 * -> Column inversion
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData10);
	writecmd(dsi, 1, (uint8_t *)ShortRegData11);

	/*
	 * VCOMDC - 0xD900h - 1st parameter - Default 0x39h
	 * VCOM Voltage settings
	 * -> from -1.0000v downto -1.2625v
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 1, (uint8_t *)ShortRegData12);

	/* Oscillator adjustment for Idle/Normal mode (LPDT only) set to 65Hz (default is 60Hz) */
	writecmd(dsi, 1, (uint8_t *)ShortRegData13);
	writecmd(dsi, 1, (uint8_t *)ShortRegData14);

	/* Video mode internal */
	writecmd(dsi, 1, (uint8_t *)ShortRegData15);
	writecmd(dsi, 1, (uint8_t *)ShortRegData16);

	/*
	 * PWR_CTRL2 - 0xC590h - 147h parameter - Default 0x00
	 * Set pump 4&5 x6
	 * -> ONLY VALID when PUMP4_EN_ASDM_HV = "0"
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData17);
	writecmd(dsi, 1, (uint8_t *)ShortRegData18);

	/*
	 * PWR_CTRL2 - 0xC590h - 150th parameter - Default 0x33h
	 * Change pump4 clock ratio
	 * -> from 1 line to 1/2 line
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData19);
	writecmd(dsi, 1, (uint8_t *)ShortRegData9);

	/* GVDD/NGVDD settings */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi,  2, (uint8_t *)lcdRegData5);

	/*
	 * PWR_CTRL2 - 0xC590h - 149th parameter - Default 0x33h
	 * Rewrite the default value !
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData20);
	writecmd(dsi, 1, (uint8_t *)ShortRegData21);

	/* Panel display timing Setting 3 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData22);
	writecmd(dsi, 1, (uint8_t *)ShortRegData23);

	/* Power control 1 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData24);
	writecmd(dsi, 1, (uint8_t *)ShortRegData25);

	/* Source driver precharge */
	writecmd(dsi, 1, (uint8_t *)ShortRegData13);
	writecmd(dsi, 1, (uint8_t *)ShortRegData26);

	writecmd(dsi, 1, (uint8_t *)ShortRegData15);
	writecmd(dsi, 1, (uint8_t *)ShortRegData27);

	writecmd(dsi, 1, (uint8_t *)ShortRegData28);
	writecmd(dsi, 2, (uint8_t *)lcdRegData6);

	/* GOAVST */
	writecmd(dsi, 1, (uint8_t *)ShortRegData2);
	writecmd(dsi, 6, (uint8_t *)lcdRegData7);

	writecmd(dsi, 1, (uint8_t *)ShortRegData29);
	writecmd(dsi, 14, (uint8_t *)lcdRegData8);

	writecmd(dsi, 1, (uint8_t *)ShortRegData30);
	writecmd(dsi, 14, (uint8_t *)lcdRegData9);

	writecmd(dsi, 1, (uint8_t *)ShortRegData31);
	writecmd(dsi, 10, (uint8_t *)lcdRegData10);

	writecmd(dsi, 1, (uint8_t *)ShortRegData32);
	writecmd(dsi, 1, (uint8_t *)ShortRegData46);

	writecmd(dsi, 1, (uint8_t *)ShortRegData2);
	writecmd(dsi, 10, (uint8_t *)lcdRegData11);

	writecmd(dsi, 1, (uint8_t *)ShortRegData33);
	writecmd(dsi, 15, (uint8_t *)lcdRegData12);

	writecmd(dsi, 1, (uint8_t *)ShortRegData29);
	writecmd(dsi, 15, (uint8_t *)lcdRegData13);

	writecmd(dsi, 1, (uint8_t *)ShortRegData30);
	writecmd(dsi, 10, (uint8_t *)lcdRegData14);

	writecmd(dsi, 1, (uint8_t *)ShortRegData31);
	writecmd(dsi, 15, (uint8_t *)lcdRegData15);

	writecmd(dsi, 1, (uint8_t *)ShortRegData32);
	writecmd(dsi, 15, (uint8_t *)lcdRegData16);

	writecmd(dsi, 1, (uint8_t *)ShortRegData34);
	writecmd(dsi, 10, (uint8_t *)lcdRegData17);

	writecmd(dsi, 1, (uint8_t *)ShortRegData35);
	writecmd(dsi, 10, (uint8_t *)lcdRegData18);

	writecmd(dsi, 1, (uint8_t *)ShortRegData2);
	writecmd(dsi, 10, (uint8_t *)lcdRegData19);

	writecmd(dsi, 1, (uint8_t *)ShortRegData33);
	writecmd(dsi, 15, (uint8_t *)lcdRegData20);

	writecmd(dsi, 1, (uint8_t *)ShortRegData29);
	writecmd(dsi, 15, (uint8_t *)lcdRegData21);

	writecmd(dsi, 1, (uint8_t *)ShortRegData30);
	writecmd(dsi, 10, (uint8_t *)lcdRegData22);

	writecmd(dsi, 1, (uint8_t *)ShortRegData31);
	writecmd(dsi, 15, (uint8_t *)lcdRegData23);

	writecmd(dsi, 1, (uint8_t *)ShortRegData32);
	writecmd(dsi, 15, (uint8_t *)lcdRegData24);

	/*
	 * PWR_CTRL1 - 0xc580h - 130th parameter - default 0x00
	 * Pump 1 min and max DM
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData13);
	writecmd(dsi, 1, (uint8_t *)ShortRegData47);
	writecmd(dsi, 1, (uint8_t *)ShortRegData48);
	writecmd(dsi, 1, (uint8_t *)ShortRegData49);

	/* Exit CMD2 mode */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 3, (uint8_t *)lcdRegData25);

	/*************************************************************************** */
	/* Standard DCS Initialization TO KEEP CAN BE DONE IN HSDT                   */
	/*************************************************************************** */

	/* NOP - goes back to DCS std command ? */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);

	/* Gamma correction 2.2+ table (HSDT possible) */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 16, (uint8_t *)lcdRegData3);

	/* Gamma correction 2.2- table (HSDT possible) */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);
	writecmd(dsi, 16, (uint8_t *)lcdRegData4);

	/* Send Sleep Out command to display : no parameter */
	writecmd(dsi, 1, (uint8_t *)ShortRegData36);

	/* Wait for sleep out exit */
	msleep(120);

	/* Set Pixel color format to RGB888 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData38);

	/* By default the orientation mode is portrait */
	if(orientation == STM32_DSI_ORIENTATION_LANDSCAPE)
	{
		writecmd(dsi, 1, (uint8_t *)ShortRegData39);
		writecmd(dsi, 4, (uint8_t *)lcdRegData27);
		writecmd(dsi, 4, (uint8_t *)lcdRegData28);
	}

	/* By default the Tearing Signal is off */
	if (te)
		writecmd(dsi, 1, (uint8_t *)ShortRegDataTE);

	/* Note : defaut is 0 (lowest Brightness), 0xFF is highest Brightness, try 0x7F : intermediate value */
	writecmd(dsi, 1, (uint8_t *)ShortRegData40);

	/* defaut is 0, try 0x2C - Brightness Control Block, Display Dimming & BackLight on */
	writecmd(dsi, 1, (uint8_t *)ShortRegData41);

	/* defaut is 0, try 0x02 - image Content based Adaptive Brightness [Still Picture] */
	writecmd(dsi, 1, (uint8_t *)ShortRegData42);

	/* defaut is 0 (lowest Brightness), 0xFF is highest Brightness */
	writecmd(dsi, 1, (uint8_t *)ShortRegData43);

	/* Send Command Display On */
	writecmd(dsi, 1, (uint8_t *)ShortRegData44);

	/* NOP command */
	writecmd(dsi, 1, (uint8_t *)ShortRegData1);

	/*
	 * Send Command GRAM memory write (no parameters) : this initiates
	 * frame write via other DSI commands sent by
	 * DSI host from LTDC incoming pixels in video mode
	 */
	writecmd(dsi, 1, (uint8_t *)ShortRegData45);

	return 0;
}

static struct stm32_dsi_panel otm8009a_panel = {
	.init = otm8009a_init,
};

static int otm8009a_probe(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, &otm8009a_panel);
	return 0;
}

static const struct of_device_id otm8009a_dt_ids[] = {
	{ .compatible = "orise,otm8009a", .data = &otm8009a_panel },
	{ /* sentinel */ },
};

static struct platform_driver otm8009a_driver = {
	.probe = otm8009a_probe,
	.driver = {
		.name = "otm8009a",
		.of_match_table = of_match_ptr(otm8009a_dt_ids),
	},
};

module_platform_driver(otm8009a_driver);
