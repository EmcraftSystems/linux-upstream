/* Copyright (c) 2012 by SNDC AB
 *
 * This software is copyrighted by and is the sole property of Samsung Nanoradio Design Centre (SNDC) AB.
 * All rights, title, ownership, or other interests in the software remain the property of SNDC AB.
 * This software may only be used in accordance with the corresponding license agreement.  Any unauthorized use, duplication, transmission, distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior written consent of SNDC AB.
 *
 * SNDC AB reserves the right to modify this software without notice.
 *
 * SNDC AB
 * Torshamnsgatan 39                       info@nanoradio.se
 * 164 40 Kista                            http://www.nanoradio.se
 * SWEDEN
 */

#ifndef __sdio_reg_h__
#define __sdio_reg_h__

#define SDIO_VENDOR_ID_NANORADIO          0x03bb
#define SDIO_DEVICE_ID_NANORADIO_NRX701   0x0000
#define SDIO_DEVICE_ID_NANORADIO_NRX900C  0x3000 /* upper byte */
#define SDIO_DEVICE_ID_NANORADIO_NRX900D  0x4000 /* upper byte */
#define SDIO_DEVICE_ID_NANORADIO_NRX900EF 0x4100 /* upper byte */
#define SDIO_DEVICE_ID_NANORADIO_NRX900G  0x4200 /* upper byte */
#define SDIO_DEVICE_ID_NANORADIO_NRX600   0x2000 /* upper byte */
#define SDIO_DEVICE_ID_NANORADIO_NRX615   0x2100 /* upper byte */
#define DEVICE_ID_CHIP_MASK 0xff00
#define DEVICE_ID_REV_MASK  0x00ff

/* Standard SDIO control registers */
#define NRX_CCCR_IF           0x07   /* bus interface controls */
#define NRX_CCCR_IENx         0x04   /* Function/Master Interrupt Enable */

/* Nanoradio specific SDIO control registers */
#define NRX_CCCR_INTACK     0xf0 /* Bit 1, Interrupt pending, write '1' to clear */
#define NRX_CCCR_WAKEUP     0xf1 /* Bit 0, Wakeup, read/write */
#define NRX_CCCR_NOCLK_INT  0xf2 /* Bit 0, Assert interrupt without SDIO CLK, read/write */
#define NRX_CCCR_HISPEED_EN 0xf3 /* Bit 0, Hi-Speed Mode Enable, read/write */
#define NRX_CCCR_FIFO_STAT  0xf4 /* Bit 0, Fifo Underrun ; Bit 1, Fifo Overrun, read/write */
#define NRX_CCCR_RESET      0xf5 /* Bit 0, Reset all ; Bit 1, Reset all but SDIO */

/* SDIO commands */
#define NRX_SET_RELATIVE_ADDR 3      /* ac   [31:16] RCA        R1  */
#define NRX_SELECT_CARD       7      /* ac   [31:16] RCA        R1  */




#endif /* __sdio_reg_h__ */
