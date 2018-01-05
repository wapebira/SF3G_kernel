/*
 * HDMI support
 *
 * Copyright (C) 2013 ITE Tech. Inc.
 * Author: Jau-Chih.Tseng@ite.com.tw
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hdmitx.h"
#include "hdmitx_drv.h"
#define FALLING_EDGE_TRIGGER

#define MSCOUNT 1000
#define LOADING_UPDATE_TIMEOUT (3000/32)	/* 3sec */
/* USHORT u8msTimer = 0; */
/* USHORT TimerServF = TRUE; */

/* Authentication status */

/* #define TIMEOUT_WAIT_AUTH MS(2000) */

struct hdmitx_dev hdmitxdev[HDMITX_MAX_DEV_COUNT];

#ifndef INV_INPUT_PCLK
#define PCLKINV 0
#else
#define PCLKINV B_TX_VDO_LATCH_EDGE
#endif

#ifndef INV_INPUT_ACLK
#define invaudclk 0
#else
#define invaudclk B_TX_AUDFMT_FALL_EDGE_SAMPLE_WS
#endif

#define INIT_CLK_HIGH
/* #define INIT_CLK_LOW */

_CODE struct regsetentry hdmitx_init_table[] = {
	{0x0F, 0x40, 0x00},

	{0x62, 0x08, 0x00},
	{0x64, 0x04, 0x00},
	{0x01, 0x00, 0x00}, /* idle(100); */

	{0x04, 0x20, 0x20},
	{0x04, 0x1D, 0x1D},
	{0x01, 0x00, 0x00}, /* idle(100); */
	{0x0F, 0x01, 0x00}, /* bank 0; */
#ifdef INIT_CLK_LOW
	{0x62, 0x90, 0x10},
	{0x64, 0x89, 0x09},
	{0x68, 0x10, 0x10},
#endif

	{0xD1, 0x0E, 0x0C},
	{0x65, 0x03, 0x00},
#ifdef NON_SEQUENTIAL_YCBCR422 /* for ITE HDMIRX */
	{0x71, 0xFC, 0x1C},
#else
	{0x71, 0xFC, 0x18},
#endif

	{0x8D, 0xFF, CEC_I2C_SLAVE_ADDR},
	{0x0F, 0x08, 0x08},

	{0xF8, 0xFF, 0xC3},
	{0xF8, 0xFF, 0xA5},
	{0x20, 0x80, 0x80},
	{0x37, 0x01, 0x00},
	{0x20, 0x80, 0x00},
	{0xF8, 0xFF, 0xFF},

	{0x59, 0xD8, 0x40|PCLKINV},
	{0xE1, 0x20, invaudclk},
	{0x05, 0xC0, 0x40},
	{REG_TX_INT_MASK1, 0xFF, ~(B_TX_RXSEN_MASK | B_TX_HPD_MASK)},
	{REG_TX_INT_MASK2, 0xFF,
	 ~(B_TX_KSVLISTCHK_MASK | B_TX_AUTH_DONE_MASK | B_TX_AUTH_FAIL_MASK)},
	{REG_TX_INT_MASK3, 0xFF, ~(0x0)},
	{0x0C, 0xFF, 0xFF},
	{0x0D, 0xFF, 0xFF},
	{0x0E, 0x03, 0x03},

	{0x0C, 0xFF, 0x00},
	{0x0D, 0xFF, 0x00},
	{0x0E, 0x02, 0x00},
	{0x09, 0x03, 0x00}, /* Enable HPD and RxSen Interrupt */
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_defaultvideo_table[] = {
	/* Config default output format. */
	{0x72, 0xff, 0x00},
	{0x70, 0xff, 0x00},
#ifndef DEFAULT_INPUT_YCBCR
	/* GenCSC\RGB2YUV_ITU709_16_235.c */
	{0x72, 0xFF, 0x02},
	{0x73, 0xFF, 0x00},
	{0x74, 0xFF, 0x80},
	{0x75, 0xFF, 0x00},
	{0x76, 0xFF, 0xB8},
	{0x77, 0xFF, 0x05},
	{0x78, 0xFF, 0xB4},
	{0x79, 0xFF, 0x01},
	{0x7A, 0xFF, 0x93},
	{0x7B, 0xFF, 0x00},
	{0x7C, 0xFF, 0x49},
	{0x7D, 0xFF, 0x3C},
	{0x7E, 0xFF, 0x18},
	{0x7F, 0xFF, 0x04},
	{0x80, 0xFF, 0x9F},
	{0x81, 0xFF, 0x3F},
	{0x82, 0xFF, 0xD9},
	{0x83, 0xFF, 0x3C},
	{0x84, 0xFF, 0x10},
	{0x85, 0xFF, 0x3F},
	{0x86, 0xFF, 0x18},
	{0x87, 0xFF, 0x04},
#else
	/* GenCSC\YUV2RGB_ITU709_16_235.c */
	{0x0F, 0x01, 0x00},
	{0x72, 0xFF, 0x03},
	{0x73, 0xFF, 0x00},
	{0x74, 0xFF, 0x80},
	{0x75, 0xFF, 0x00},
	{0x76, 0xFF, 0x00},
	{0x77, 0xFF, 0x08},
	{0x78, 0xFF, 0x53},
	{0x79, 0xFF, 0x3C},
	{0x7A, 0xFF, 0x89},
	{0x7B, 0xFF, 0x3E},
	{0x7C, 0xFF, 0x00},
	{0x7D, 0xFF, 0x08},
	{0x7E, 0xFF, 0x51},
	{0x7F, 0xFF, 0x0C},
	{0x80, 0xFF, 0x00},
	{0x81, 0xFF, 0x00},
	{0x82, 0xFF, 0x00},
	{0x83, 0xFF, 0x08},
	{0x84, 0xFF, 0x00},
	{0x85, 0xFF, 0x00},
	{0x86, 0xFF, 0x87},
	{0x87, 0xFF, 0x0E},
#endif
	/* 2012/12/20 added by Keming's suggestion test */
	{0x88, 0xF0, 0x00},
	/* ~jauchih.tseng@ite.com.tw */
	{0x04, 0x08, 0x00},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_sethdmi_table[] = {
	/* Config default HDMI Mode */
	{0xC0, 0x01, 0x01},
	{0xC1, 0x03, 0x03},
	{0xC6, 0x03, 0x03},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_setdvi_table[] = {
	/* Config default HDMI Mode */
	{0x0F, 0x01, 0x01},
	{0x58, 0xFF, 0x00},
	{0x0F, 0x01, 0x00},
	{0xC0, 0x01, 0x00},
	{0xC1, 0x03, 0x02},
	{0xC6, 0x03, 0x00},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_defaultaviinfo_table[] = {
	/* Config default avi infoframe */
	{0x0F, 0x01, 0x01},
	{0x58, 0xFF, 0x10},
	{0x59, 0xFF, 0x08},
	{0x5A, 0xFF, 0x00},
	{0x5B, 0xFF, 0x00},
	{0x5C, 0xFF, 0x00},
	{0x5D, 0xFF, 0x57},
	{0x5E, 0xFF, 0x00},
	{0x5F, 0xFF, 0x00},
	{0x60, 0xFF, 0x00},
	{0x61, 0xFF, 0x00},
	{0x62, 0xFF, 0x00},
	{0x63, 0xFF, 0x00},
	{0x64, 0xFF, 0x00},
	{0x65, 0xFF, 0x00},
	{0x0F, 0x01, 0x00},
	{0xCD, 0x03, 0x03},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_deaultaudioinfo_table[] = {
	/* Config default audio infoframe */
	{0x0F, 0x01, 0x01},
	{0x68, 0xFF, 0x00},
	{0x69, 0xFF, 0x00},
	{0x6A, 0xFF, 0x00},
	{0x6B, 0xFF, 0x00},
	{0x6C, 0xFF, 0x00},
	{0x6D, 0xFF, 0x71},
	{0x0F, 0x01, 0x00},
	{0xCE, 0x03, 0x03},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_aud_chstatus_lpcm_20bit_48khz[] = {
	{0x0F, 0x01, 0x01},
	{0x33, 0xFF, 0x00},
	{0x34, 0xFF, 0x18},
	{0x35, 0xFF, 0x00},
	{0x91, 0xFF, 0x00},
	{0x92, 0xFF, 0x00},
	{0x93, 0xFF, 0x01},
	{0x94, 0xFF, 0x00},
	{0x98, 0xFF, 0x02},
	{0x99, 0xFF, 0xDA},
	{0x0F, 0x01, 0x00},
	{0, 0, 0}
};

_CODE struct regsetentry HDMITX_AUD_SPDIF_2ch_24bit[] = {
	{0x0F, 0x11, 0x00},
	{0x04, 0x14, 0x04},
	{0xE0, 0xFF, 0xD1},
	{0xE1, 0xFF, 0x01},
	{0xE2, 0xFF, 0xE4},
	{0xE3, 0xFF, 0x10},
	{0xE4, 0xFF, 0x00},
	{0xE5, 0xFF, 0x00},
	{0x04, 0x14, 0x00},
	{0, 0, 0}
};

_CODE struct regsetentry HDMITX_AUD_I2S_2ch_24bit[] = {
	{0x0F, 0x11, 0x00},
	{0x04, 0x14, 0x04},
	{0xE0, 0xFF, 0xC1},
	{0xE1, 0xFF, 0x01},
	{0xE2, 0xFF, 0xE4},
	{0xE3, 0xFF, 0x00},
	{0xE4, 0xFF, 0x00},
	{0xE5, 0xFF, 0x00},
	{0x04, 0x14, 0x00},
	{0, 0, 0}
};

_CODE struct regsetentry hdmitx_defaultaudio_table[] = {
	/* Config default audio output format. */
	{0x0F, 0x21, 0x00},
	{0x04, 0x14, 0x04},
	{0xE0, 0xFF, 0xC1},
	{0xE1, 0xFF, 0x01},
	{0xE2, 0xFF, 0xE4},
	{0xE3, 0xFF, 0x00},
	{0xE4, 0xFF, 0x00},
	{0xE5, 0xFF, 0x00},
	{0x0F, 0x01, 0x01},
	{0x33, 0xFF, 0x00},
	{0x34, 0xFF, 0x18},
	{0x35, 0xFF, 0x00},
	{0x91, 0xFF, 0x00},
	{0x92, 0xFF, 0x00},
	{0x93, 0xFF, 0x01},
	{0x94, 0xFF, 0x00},
	{0x98, 0xFF, 0x02},
	{0x99, 0xFF, 0xDB},
	{0x0F, 0x01, 0x00},
	{0x04, 0x14, 0x00},

	{0x00, 0x00, 0x00} /* End of Table. */
};

_CODE struct regsetentry hdmitx_pwrdown_table[] = {
	/* Enable GRCLK */
	{0x0F, 0x40, 0x00},
	/* PLL Reset */
	{0x61, 0x10, 0x10},   /* DRV_RST */
	{0x62, 0x08, 0x00},   /* XP_RESETB */
	{0x64, 0x04, 0x00},   /* IP_RESETB */
	{0x01, 0x00, 0x00}, /* idle(100); */

	/* PLL PwrDn */
	{0x61, 0x20, 0x20},   /* PwrDn DRV */
	{0x62, 0x44, 0x44},   /* PwrDn XPLL */
	{0x64, 0x40, 0x40},   /* PwrDn IPLL */

	/* HDMITX PwrDn */
	{0x05, 0x01, 0x01},   /* PwrDn PCLK */
	{0x0F, 0x78, 0x78},   /* PwrDn GRCLK */
	{0x00, 0x00, 0x00} /* End of Table. */
};

_CODE struct regsetentry hdmitx_pwron_table[] = {
	{0x0F, 0x78, 0x38},   /* PwrOn GRCLK */
	{0x05, 0x01, 0x00},   /* PwrOn PCLK */

	/* PLL PwrOn */
	{0x61, 0x20, 0x00},   /* PwrOn DRV */
	{0x62, 0x44, 0x00},   /* PwrOn XPLL */
	{0x64, 0x40, 0x00},   /* PwrOn IPLL */

	/* PLL Reset OFF */
	{0x61, 0x10, 0x00},   /* DRV_RST */
	{0x62, 0x08, 0x08},   /* XP_RESETB */
	{0x64, 0x04, 0x04},   /* IP_RESETB */
	{0x0F, 0x78, 0x08},   /* PwrOn IACLK */
	{0x00, 0x00, 0x00} /* End of Table. */
};

#ifdef DETECT_VSYNC_CHG_IN_SAV
BOOL ensavvsync = FALSE;
#endif
static bool PowerStatus = FALSE;

/* Function Prototype */

void hdmitx_init_txdev(struct hdmitx_dev *pinstance)
{
	if (pinstance && 0 < HDMITX_MAX_DEV_COUNT)
		hdmitxdev[0] = *pinstance;
}

void init_hdmitx(void)
{
	hdmitx_loadregsetting(hdmitx_init_table);
	/* HDMITX_WRITEI2C_BYTE(REG_TX_PLL_CTRL, 0xff); */
	hdmitxdev[0].bintpol =
		(hdmitxdev[0].binttype&B_TX_INTPOL_ACTH) ? TRUE : FALSE;

	/* Avoid power loading in un play status. */

	/* Setup HDCP ROM */

#ifdef HDMITX_INPUT_INFO
	hdmitxdev[0].RCLK = calcrclk();
#endif
	hdmitx_loadregsetting(hdmitx_defaultvideo_table);
	hdmitx_loadregsetting(hdmitx_sethdmi_table);
	hdmitx_loadregsetting(hdmitx_defaultaviinfo_table);
	hdmitx_loadregsetting(hdmitx_deaultaudioinfo_table);
	hdmitx_loadregsetting(hdmitx_aud_chstatus_lpcm_20bit_48khz);
	hdmitx_loadregsetting(HDMITX_AUD_SPDIF_2ch_24bit);
	hdmitx_powerdown();

	HDMITX_DEBUG_PRINTF("----------------------------------------\n"
			    "Init HDMITX\n"
			    "----------------------------------------\n");

	dumphdmitxreg();
}

BOOL gethdmitx_linkstatus(void)
{
	if (B_TX_RXSENDETECT & HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS)) {
		if (0 == HDMITX_READI2C_BYTE(REG_TX_AFE_DRV_CTRL))
			return TRUE;
	}
	/* HDMITX_DEBUG_PRINTF("GetTMDS not Ready()!!\n"); */

	return FALSE;
}

void hdmitx_poweron(void)
{
	PowerStatus = TRUE;
	hdmitx_loadregsetting(hdmitx_pwron_table);
}

void hdmitx_powerdown(void)
{
	PowerStatus = FALSE;
	hdmitx_loadregsetting(hdmitx_pwrdown_table);
}

BOOL gethdmitx_powerstatus(void)
{
	return PowerStatus;
}

void sethdmitx_avmute(BYTE benable)
{
	SWITCH_HDMITX_BANK(0);
	HDMITX_SETI2C_BYTE(REG_TX_GCP, B_TX_SETAVMUTE,
			   benable ? B_TX_SETAVMUTE : 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_GENERAL_CTRL,
			     B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}

/* Function: hdmitx_loadregsetting() */
/* Input: struct regsetentry SettingTable[]; */
/* Return: N/A */
/* Remark: if an entry {0, 0, 0} will be terminated. */

void hdmitx_loadregsetting(struct regsetentry table[])
{
	int i;

	for (i = 0; ; i++) {
		if (table[i].offset == 0 &&
		    table[i].invandmask == 0 && table[i].ormask == 0)
			return;
		else if (table[i].invandmask == 0 && table[i].ormask == 0)
			delay1ms(table[i].offset);
		else if (table[i].invandmask == 0xFF)
			HDMITX_WRITEI2C_BYTE(table[i].offset, table[i].ormask);
		else
			HDMITX_SETI2C_BYTE(table[i].offset,
					   table[i].invandmask,
					   table[i].ormask);
	}
}

/* **************************************** */
/* @file   <hdmitx_ddc.c> */
/* ******************************************/

BOOL gethdmitx_edidblock(int edidblockid, BYTE *pediddata)
{
	if (!pediddata)
		return FALSE;
	if (gethdmitx_edidbytes(pediddata, edidblockid/2,
				(edidblockid%2)*128, 128) == ER_FAIL)
		return FALSE;
#if debug_message
	{
		int j = 0;

		EDID_DEBUG_PRINTF("------BlockID=%d------\n", edidblockid);
		for (j = 0; j < 128; j++)
			EDID_DEBUG_PRINTF("%02X%c", (int)pediddata[j],
					  (7 == (j & 7)) ? '\n' : ' ');
	}
#endif
	return TRUE;
}

/* Function: gethdmitx_edidbytes */
/* Parameter: pdata - the pointer of buffer to receive EDID ucdata. */
/* bsegment - the segment of EDID readback. */
/* offset - the offset of EDID ucdata in the segment. in byte. */
/* count - the read back bytes count, cannot exceed 32 */
/* Return: ER_SUCCESS if successfully getting EDID. ER_FAIL otherwise. */
/* Remark: function for read EDID ucdata from receiver. */
/* Side-Effect: DDC master will set to be HOST.
 * DDC FIFO will be used and dirty. */

enum SYS_STATUS gethdmitx_edidbytes(BYTE *pdata, BYTE bsegment,
				    BYTE offset, SHORT count)
{
	SHORT remainedcount, reqcount;
	BYTE bcurroffset;
	SHORT timeout;
	BYTE *pbuff = pdata;
	BYTE ucdata;

	if (!pdata)
		return ER_FAIL;
	if (HDMITX_READI2C_BYTE(REG_TX_INT_STAT1) & B_TX_INT_DDC_BUS_HANG) {
		HDMITX_DEBUG_PRINTF("Called hdmitx_AboutDDC()\n");
		hdmitx_abortddc();
	}

	hdmitx_clearddcfifo();

	remainedcount = count;
	bcurroffset = offset;

	SWITCH_HDMITX_BANK(0);

	while (remainedcount > 0) {
		reqcount = (remainedcount > DDC_FIFO_MAXREQ) ?
			   DDC_FIFO_MAXREQ : remainedcount;
		HDMITX_DEBUG_PRINTF("gethdmitx_edidbytes():");
		HDMITX_DEBUG_PRINTF("reqcount = %d, bcurroffset = %d\n",
				    (int)reqcount, (int)bcurroffset);

		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
				     B_TX_MASTERDDC | B_TX_MASTERHOST);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_FIFO_CLR);

		for (timeout = 0; timeout < 200; timeout++) {
			ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);

			if (ucdata&B_TX_DDC_DONE)
				break;
			if ((ucdata & B_TX_DDC_ERROR) ||
			    (HDMITX_READI2C_BYTE(REG_TX_INT_STAT1) &
			     B_TX_INT_DDC_BUS_HANG)) {
				HDMITX_DEBUG_PRINTF("Called hdmitx_AboutDDC(n");
				hdmitx_abortddc();
				return ER_FAIL;
			}
		}
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
				     B_TX_MASTERDDC | B_TX_MASTERHOST);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_HEADER, DDC_EDID_ADDRESS);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQOFF, bcurroffset);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQCOUNT, (BYTE)reqcount);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_EDIDSEG, bsegment);
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_EDID_READ);

		bcurroffset += reqcount;
		remainedcount -= reqcount;

		for (timeout = 250; timeout > 0; timeout--) {
			delay1ms(1);
			ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);
			if (ucdata & B_TX_DDC_DONE)
				break;
			if (ucdata & B_TX_DDC_ERROR) {
				HDMITX_DEBUG_PRINTF("gethdmitx_edidbytes():");
				HDMITX_DEBUG_PRINTF("DDC_STATUS=%02X fail.\n",
						    (int)ucdata);
				return ER_FAIL;
			}
		}
		if (timeout == 0) {
			HDMITX_DEBUG_PRINTF("gethdmitx_edidbytes():");
			HDMITX_DEBUG_PRINTF("DDC timeout %d.\n", (int)ucdata);
			return ER_FAIL;
		}
		do {
			*(pbuff++) = HDMITX_READI2C_BYTE(REG_TX_DDC_READFIFO);
			reqcount--;
		} while (reqcount > 0);
	}

	return ER_SUCCESS;
}

/* DDC Function. */

/* Function: hdmitx_clearddcfifo */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: clear the DDC FIFO. */
/* Side-Effect: DDC master will set to be HOST. */

void hdmitx_clearddcfifo(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_FIFO_CLR);
}

void hdmitx_generateddcsclk(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_GEN_SCLCLK);
}

/* Function: hdmitx_abortddc */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: Force abort DDC and reset DDC bus. */
/* Side-Effect: */

void hdmitx_abortddc(void)
{
	BYTE cpdesire, swreset, ddcmaster;
	BYTE uc, timeout, i;
	/* save the SW reset, DDC master, and CP Desire setting. */
	swreset = HDMITX_READI2C_BYTE(REG_TX_SW_RST);
	cpdesire = HDMITX_READI2C_BYTE(REG_TX_HDCP_DESIRE);
	ddcmaster = HDMITX_READI2C_BYTE(REG_TX_DDC_MASTER_CTRL);

	HDMITX_WRITEI2C_BYTE(REG_TX_HDCP_DESIRE, cpdesire&(~B_TX_CPDESIRE));
	HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST, swreset | B_TX_HDCP_RST_HDMITX);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);

	/* 2009/01/15 modified by Jau-Chih.Tseng@ite.com.tw */
	/* do abort DDC twice. */
	for (i = 0; i < 2; i++) {
		HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_DDC_ABORT);

		for (timeout = 0; timeout < 200; timeout++) {
			uc = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);
			if (uc&B_TX_DDC_DONE)
				break; /* success */
			if (uc & (B_TX_DDC_NOACK | B_TX_DDC_WAITBUS |
			    B_TX_DDC_ARBILOSE))
				break;
			delay1ms(1); /* delay 1 ms to stable. */
		}
	}
	/* ~Jau-Chih.Tseng@ite.com.tw */
}

/* ***************************************** */
/* @file   <hdmitx_vid.c> */
/* ******************************************/

/* utility function for main.. */

/* Function Body. */

void hdmitx_disable_videooutput(void)
{
	BYTE uc = HDMITX_READI2C_BYTE(REG_TX_SW_RST) | B_HDMITX_VID_RST;

	HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST, uc);
	HDMITX_WRITEI2C_BYTE(REG_TX_AFE_DRV_CTRL,
			     B_TX_AFE_DRV_RST | B_TX_AFE_DRV_PWD);
	HDMITX_SETI2C_BYTE(0x62, 0x90, 0x00);
	HDMITX_SETI2C_BYTE(0x64, 0x89, 0x00);
}

BOOL hdmitx_enable_videooutput(enum pclk_level level, BYTE input_colormode,
			       BYTE output_colormode, BYTE bhdmi)
{
	/* should be configured by initsys.c */
	/* enum pclk_level level; */
	switch (level) {
	case PCLK_HIGH:
		HDMITX_WRITEI2C_BYTE(REG_TX_PLL_CTRL, 0x30 /*0xff*/);
		break;
	default:
		HDMITX_WRITEI2C_BYTE(REG_TX_PLL_CTRL, 0x00);
		break;
	}
	HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST,
			     B_HDMITX_VID_RST | B_HDMITX_AUD_RST |
			     B_TX_AREF_RST|B_TX_HDCP_RST_HDMITX);

	hdmitxdev[0].bhdmimode = (BYTE)bhdmi;
	/* 2009/12/09 added by jau-chih.tseng@ite.com.tw */
	SWITCH_HDMITX_BANK(1);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB1, 0x00);
	SWITCH_HDMITX_BANK(0);
	/* ~jau-chih.tseng@ite.com.tw */

	if (hdmitxdev[0].bhdmimode)
		sethdmitx_avmute(TRUE);
	hdmitx_setinputmode(input_colormode,
			    hdmitxdev[0].binputvideosignaltype);

	hdmitx_setcscscale(input_colormode, output_colormode);

	if (hdmitxdev[0].bhdmimode)
		HDMITX_WRITEI2C_BYTE(REG_TX_HDMI_MODE, B_TX_HDMI_MODE);
	else
		HDMITX_WRITEI2C_BYTE(REG_TX_HDMI_MODE, B_TX_DVI_MODE);
#ifdef INVERT_VID_LATCHEDGE
	uc = HDMITX_READI2C_BYTE(REG_TX_CLK_CTRL1);
	uc |= B_TX_VDO_LATCH_EDGE;
	HDMITX_WRITEI2C_BYTE(REG_TX_CLK_CTRL1, uc);
#endif

	hdmitx_setupafe(level); /* pass if High Freq request */
	HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST,
			     B_HDMITX_AUD_RST | B_TX_AREF_RST |
			     B_TX_HDCP_RST_HDMITX);

	hdmitx_fireafe();

	return TRUE;
}

/* export this for dynamic change input signal */

BOOL sethdmitx_videosignaltype(BYTE input_signaltype)
{
	hdmitxdev[0].binputvideosignaltype = input_signaltype;
	return TRUE;
}

void waittxvidstable(void)
{
}

void sethdmitx_colordepthphase(BYTE color_depth, BYTE bphase)
{
#ifdef IT6615
	BYTE uc;
	BYTE bcolor_depth;

	if (color_depth == 30) {
		bcolor_depth = B_TX_CD_30;
		HDMITX_DEBUG_PRINTF("bcolor_depth = B_TX_CD_30\n");
	} else if (color_depth == 36) {
		bcolor_depth = B_TX_CD_36;
		HDMITX_DEBUG_PRINTF("bcolor_depth = B_TX_CD_36\n");
	} else if (color_depth == 24) {
		bcolor_depth = B_TX_CD_24;
	} else {
		bcolor_depth = 0; /* not indicated */
	}
	SWITCH_HDMITX_BANK(0);
	HDMITX_SETI2C_BYTE(REG_TX_GCP, B_TX_COLOR_DEPTH_MASK , bcolor_depth);
#endif
}

#ifdef SUPPORT_SYNCEMBEDDED

struct crt_timingsetting {
	BYTE fmt;
	WORD hactive;
	WORD vactive;
	WORD htotal;
	WORD vtotal;
	WORD H_FBH;
	WORD h_syncw;
	WORD H_BBH;
	WORD V_FBH;
	WORD v_syncw;
	WORD V_BBH;
	BYTE scan:1;
	BYTE vpolarity:1;
	BYTE hpolarity:1;
};

_CODE struct crt_timingsetting timingtable[] = {
	/* VIC H V htotal vtotal HFT HSW HBP VF VSW VB*/
	/* =====640x480@60Hz		 - CEA Mode [ 1]===== */
	{  1, 640, 480, 800, 525, 16, 96, 48, 10, 2, 33, PROG, vneg, hneg},
	/* =====720x480@60Hz		 - CEA Mode [ 2]===== */
	{  2, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30, PROG, vneg, hneg},
	/* =====720x480@60Hz		 - CEA Mode [ 3]===== */
	{  3, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30, PROG, vneg, hneg},
	/* =====1280x720@60Hz		- CEA Mode [ 4]===== */
	{  4, 1280, 720, 1650, 750, 110, 40, 220, 5, 5, 20, PROG, vpos, hpos},
	/* =====1920x1080(I)@60Hz	- CEA Mode [ 5]===== */
	{  5, 1920, 540, 2200, 562, 88, 44, 148, 2, 5, 15,
	   INTERLACE, vpos, hpos},
	/* =====720x480(I)@60Hz	  - CEA Mode [ 6]===== */
	{  6, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, INTERLACE, vneg, hneg},
	/* =====720x480(I)@60Hz	  - CEA Mode [ 7]===== */
	{  7, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, INTERLACE, vneg, hneg},
	/* =====720x480(I)@60Hz	  - CEA Mode [ 8]===== */
	/* {  8, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, PROG, vneg, hneg}, */
	/* =====720x480(I)@60Hz	  - CEA Mode [ 9]===== */
	/* {  9, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, PROG, vneg, hneg}, */
	/* =====720x480(I)@60Hz	  - CEA Mode [10]===== */
	/* { 10, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x480(I)@60Hz	  - CEA Mode [11]===== */
	/* { 11, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x480(I)@60Hz	  - CEA Mode [12]===== */
	/* { 12, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, PROG, vneg, hneg}, */
	/* =====720x480(I)@60Hz	  - CEA Mode [13]===== */
	/* { 13, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15, PROG, vneg, hneg}, */
	/* =====1440x480@60Hz		- CEA Mode [14]===== */
	/* { 14, 1440, 480, 1716, 525, 32, 124, 120, 9, 6, 30,
	 *   PROG, vneg, hneg}, */
	/* =====1440x480@60Hz		- CEA Mode [15]===== */
	/* { 15, 1440, 480, 1716, 525, 32, 124, 120, 9, 6, 30,
	 *   PROG, vneg, hneg}, */
	/* =====1920x1080@60Hz	   - CEA Mode [16]===== */
	{ 16, 1920, 1080, 2200, 1125, 88, 44, 148, 4, 5, 36, PROG, vpos, hpos},
	/* =====720x576@50Hz		 - CEA Mode [17]===== */
	{ 17, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg},
	/* =====720x576@50Hz		 - CEA Mode [18]===== */
	{ 18, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg},
	/* =====1280x720@50Hz		- CEA Mode [19]===== */
	{ 19, 1280, 720, 1980, 750, 440, 40, 220, 5, 5, 20, PROG, vpos, hpos},
	/* =====1920x1080(I)@50Hz	- CEA Mode [20]===== */
	{ 20, 1920, 540, 2640, 562, 528, 44, 148, 2, 5, 15,
	  INTERLACE, vpos, hpos},
	/* =====1440x576(I)@50Hz	 - CEA Mode [21]===== */
	{ 21, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, INTERLACE, vneg, hneg},
	/* =====1440x576(I)@50Hz	 - CEA Mode [22]===== */
	{ 22, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, INTERLACE, vneg, hneg},
	/* =====1440x288@50Hz		- CEA Mode [23]===== */
	/* { 23, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, PROG, vneg, hneg}, */
	/* =====1440x288@50Hz		- CEA Mode [24]===== */
	/* { 24, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, PROG, vneg, hneg}, */
	/* =====1440x576(I)@50Hz	 - CEA Mode [25]===== */
	/* { 25, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1440x576(I)@50Hz	 - CEA Mode [26]===== */
	/* { 26, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1440x288@50Hz		- CEA Mode [27]===== */
	/* { 27, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, PROG, vneg, hneg}, */
	/* =====1440x288@50Hz		- CEA Mode [28]===== */
	/* { 28, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19, PROG, vneg, hneg}, */
	/* =====1440x576@50Hz		- CEA Mode [29]===== */
	/* { 29, 1440, 576, 1728, 625, 24, 128, 136, 5, 5, 39,
	 *   PROG, vpos, hneg}, */
	/* =====1440x576@50Hz		- CEA Mode [30]===== */
	/* { 30, 1440, 576, 1728, 625, 24, 128, 136, 5, 5, 39,
	 *   PROG, vpos, hneg}, */
	/* =====1920x1080@50Hz	   - CEA Mode [31]===== */
	{ 31, 1920, 1080, 2640, 1125, 528, 44, 148, 4, 5, 36, PROG, vpos, hpos},
	/* =====1920x1080@24Hz	   - CEA Mode [32]===== */
	{ 32, 1920, 1080, 2750, 1125, 638, 44, 148, 4, 5, 36, PROG, vpos, hpos},
	/* =====1920x1080@25Hz	   - CEA Mode [33]===== */
	{ 33, 1920, 1080, 2640, 1125, 528, 44, 148, 4, 5, 36, PROG, vpos, hpos},
	/* =====1920x1080@30Hz	   - CEA Mode [34]===== */
	{ 34, 1920, 1080, 2200, 1125, 88, 44, 148, 4, 5, 36, PROG, vpos, hpos},
	/* =====1920x1080@50Hz	   - CEA Mode [39]===== */
	/* { 39, 1920, 540, 2304, 625, 32, 168, 184, 23, 5, 57,
	 *   INTERLACE, vneg, hpos}, */
	/* =====1920x1080(I)@100Hz   - CEA Mode [40]===== */
	/* { 40, 1920, 540, 2640, 562, 528, 44, 148, 2, 5, 15,
	 *   INTERLACE, vpos, hpos}, */
	/* =====1280x720@100Hz	   - CEA Mode [41]===== */
	/* { 41, 1280, 720, 1980, 750, 440, 40, 220, 5, 5, 20,
	 *   PROG, vpos, hpos}, */
	/* =====720x576@100Hz		- CEA Mode [42]===== */
	/* { 42, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg}, */
	/* =====720x576@100Hz		- CEA Mode [43]===== */
	/* { 43, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg}, */
	/* =====1440x576(I)@100Hz	- CEA Mode [44]===== */
	/* { 44, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1440x576(I)@100Hz	- CEA Mode [45]===== */
	/* { 45, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1920x1080(I)@120Hz   - CEA Mode [46]===== */
	/* { 46, 1920, 540, 2200, 562, 88, 44, 148, 2, 5, 15,
	 *   INTERLACE, vpos, hpos}, */
	/* =====1280x720@120Hz	   - CEA Mode [47]===== */
	/* { 47, 1280, 720, 1650, 750, 110, 40, 220, 5, 5, 20,
	 *   PROG, vpos, hpos}, */
	/* =====720x480@120Hz		- CEA Mode [48]===== */
	/* { 48, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30, PROG, vneg, hneg}, */
	/* =====720x480@120Hz		- CEA Mode [49]===== */
	/* { 49, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30, PROG, vneg, hneg}, */
	/* =====720x480(I)@120Hz	 - CEA Mode [50]===== */
	/* { 50, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x480(I)@120Hz	 - CEA Mode [51]===== */
	/* { 51, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x576@200Hz		- CEA Mode [52]===== */
	/* { 52, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg}, */
	/* =====720x576@200Hz		- CEA Mode [53]===== */
	/* { 53, 720, 576, 864, 625, 12, 64, 68, 5, 5, 39, PROG, vneg, hneg}, */
	/* =====1440x576(I)@200Hz	- CEA Mode [54]===== */
	/* { 54, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1440x576(I)@200Hz	- CEA Mode [55]===== */
	/* { 55, 720, 288, 864, 312, 12, 63, 69, 2, 3, 19,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x480@120Hz		- CEA Mode [56]===== */
	/* { 56, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30,
	 *   PROG, vneg, hneg}, */
	/* =====720x480@120Hz		- CEA Mode [57]===== */
	/* { 57, 720, 480, 858, 525, 16, 62, 60, 9, 6, 30,
	 *   PROG, vneg, hneg}, */
	/* =====720x480(I)@120Hz	 - CEA Mode [58]===== */
	/* { 58, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====720x480(I)@120Hz	 - CEA Mode [59]===== */
	/* { 59, 720, 240, 858, 262, 19, 62, 57, 4, 3, 15,
	 *   INTERLACE, vneg, hneg}, */
	/* =====1280x720@24Hz		- CEA Mode [60]===== */
	{ 60, 1280, 720, 3300, 750, 1760, 40, 220, 5, 5, 20, PROG, vpos, hpos},
	/* =====1280x720@25Hz		- CEA Mode [61]===== */
	{ 61, 1280, 720, 3960, 750, 2420, 40, 220, 5, 5, 20, PROG, vpos, hpos},
	/* =====1280x720@30Hz		- CEA Mode [62]===== */
	{ 62, 1280, 720, 3300, 750, 1760, 40, 220, 5, 5, 20, PROG, vpos, hpos},
	/* =====1920x1080@120Hz	  - CEA Mode [63]===== */
	/* { 63, 1920, 1080, 2200, 1125, 88, 44, 148, 4, 5, 36,
	 *   PROG, vpos, hpos}, */
	/* =====1920x1080@100Hz	  - CEA Mode [64]===== */
	/* { 64, 1920, 1080, 2640, 1125, 528, 44, 148, 4, 5, 36,
	 *   PROG, vpos, hpos}, */
};

#define maxindex (sizeof(timingtable)/sizeof(struct crt_timingsetting))
BOOL sethdmitx_syncembedded_by_vic(BYTE VIC, BYTE binputtype)
{
	int i;
	BYTE fmt_index = 0;

	/* if Embedded Video, need to generate timing with pattern register */
	SWITCH_HDMITX_BANK(0);

	if (VIC > 0) {
		for (i = 0; i < maxindex; i++) {
			if (timingtable[i].fmt == VIC) {
				fmt_index = i;
				break;
			}
		}
	} else {
		HDMITX_DEBUG_PRINTF("***No Match VIC == 0 ***\n");
		return FALSE;
	}

	if (i >= maxindex) {
		/* return FALSE; */
		HDMITX_DEBUG_PRINTF("***No Match VIC ***\n");
		return FALSE;
	}
	/* if (binput_signaltype & T_MODE_SYNCEMB) */ {
		int htotal, HDES, vtotal, VDES;
		int HDEW, VDEW, HFP, HSW, VFP, VSW;
		int HRS, HRE;
		int VRS, VRE;
		int h2ndvrrise;
		int VRS2nd, VRE2nd;
		BYTE pol;

		htotal = timingtable[fmt_index].htotal;
		HDEW = timingtable[fmt_index].hactive;
		HFP = timingtable[fmt_index].H_FBH;
		HSW = timingtable[fmt_index].h_syncw;
		HDES = HSW+timingtable[fmt_index].H_BBH;
		vtotal = timingtable[fmt_index].vtotal;
		VDEW = timingtable[fmt_index].vactive;
		VFP = timingtable[fmt_index].V_FBH;
		VSW = timingtable[fmt_index].v_syncw;
		VDES = VSW+timingtable[fmt_index].V_BBH;

		pol = (timingtable[fmt_index].hpolarity == hpos) ? (1 << 1) : 0;
		pol |= (timingtable[fmt_index].vpolarity == vpos) ?
		       (1 << 2) : 0;

		/* SyncEmb case == === */
		if (binputtype & T_MODE_CCIR656)
			HRS = HFP - 1;
		else
			HRS = HFP - 2;
		HRE = HRS + HSW;
		h2ndvrrise = HRS + htotal / 2;

		VRS = VFP;
		VRE = VRS + VSW;

		/* vtotal>>=1; */

		if (PROG == timingtable[fmt_index].scan) {
			VRS2nd = 0xFFF;
			VRE2nd = 0x3F;
		} else {
			if (39 == timingtable[fmt_index].fmt) {
				VRS2nd = VRS + vtotal - 1;
				VRE2nd = VRS2nd + VSW;
			} else {
				VRS2nd = VRS + vtotal;
				VRE2nd = VRS2nd + VSW;
			}
		}
#ifdef DETECT_VSYNC_CHG_IN_SAV
		if (ensavvsync) {
			VRS -= 1;
			VRE -= 1;
			if (!psetvtiming->scanmode) {
				VRS2nd -= 1;
				VRE2nd -= 1;
			}
		}
#endif /* DETECT_VSYNC_CHG_IN_SAV */
		HDMITX_SETI2C_BYTE(0x90, 0x06, pol);
		/* write h2ndvrrise */
		HDMITX_SETI2C_BYTE(0x90, 0xF0, (h2ndvrrise & 0x0F) << 4);
		HDMITX_WRITEI2C_BYTE(0x91, (h2ndvrrise & 0x0FF0) >> 4);
		/* write HRS/HRE */
		HDMITX_WRITEI2C_BYTE(0x95, HRS & 0xFF);
		HDMITX_WRITEI2C_BYTE(0x96, HRE & 0xFF);
		HDMITX_WRITEI2C_BYTE(0x97, ((HRE & 0x0F00) >> 4) +
				     ((HRS & 0x0F00) >> 8));
		/* write VRS/VRE */
		HDMITX_WRITEI2C_BYTE(0xa0, VRS & 0xFF);
		HDMITX_WRITEI2C_BYTE(0xa1, ((VRE & 0x0F) << 4) +
				     ((VRS&0x0F00) >> 8));
		HDMITX_WRITEI2C_BYTE(0xa2, VRS2nd & 0xFF);
		HDMITX_WRITEI2C_BYTE(0xa6, (VRE2nd & 0xF0) +
				     ((VRE & 0xF0) >> 4));
		HDMITX_WRITEI2C_BYTE(0xa3, ((VRE2nd & 0x0F) << 4) +
				     ((VRS2nd & 0xF00) >> 8));
		HDMITX_WRITEI2C_BYTE(0xa4, h2ndvrrise & 0xFF);
		HDMITX_WRITEI2C_BYTE(0xa5, (0 << 5) +
				     ((timingtable[fmt_index].scan ==
				       INTERLACE) ? (1 << 4) : 0) +
				     ((h2ndvrrise & 0xF00) >> 8));
		HDMITX_SETI2C_BYTE(0xb1, 0x51, ((HRE & 0x1000) >> 6) +
				   ((HRS & 0x1000) >> 8) +
				   ((HDES & 0x1000) >> 12));
		HDMITX_SETI2C_BYTE(0xb2, 0x05, ((h2ndvrrise & 0x1000) >> 10) +
				   ((h2ndvrrise&0x1000) >> 12));
	}

	return TRUE;
}

#endif /* SUPPORT_SYNCEMBEDDED */

/* ~jj_tseng@chipadvanced.com 2007/01/02 */

/* Function: hdmitx_setinputmode */
/* Parameter: inputmode, binput_signaltype */
/* inputmode - use [1:0] to identify the color space for reg70[7:6], */
/* definition: */
/* #define F_MODE_RGB444  0 */
/* #define F_MODE_YUV422 1 */
/* #define F_MODE_YUV444 2 */
/* #define F_MODE_CLRMOD_MASK 3 */
/* binput_signaltype - defined the CCIR656 D[0], SYNC Embedded D[1], and */
/* DDR input in D[2]. */
/* Return: N/A */
/* Remark: program reg70 with the input value. */
/* Side-Effect: reg70. */

void hdmitx_setinputmode(BYTE inputcolormode, BYTE binput_signaltype)
{
	BYTE ucdata;

	ucdata = HDMITX_READI2C_BYTE(REG_TX_INPUT_MODE);
	ucdata &= ~(M_TX_INCOLMOD|B_TX_2X656CLK | B_TX_SYNCEMB |
		    B_TX_INDDR | B_TX_PCLKDIV2);
	ucdata |= 0x01;

	switch (inputcolormode & F_MODE_CLRMOD_MASK) {
	case F_MODE_YUV422:
		ucdata |= B_TX_IN_YUV422;
		break;
	case F_MODE_YUV444:
		ucdata |= B_TX_IN_YUV444;
		break;
	case F_MODE_RGB444:
	default:
		ucdata |= B_TX_IN_RGB;
		break;
	}
	if (binput_signaltype & T_MODE_PCLKDIV2)
		ucdata |= B_TX_PCLKDIV2;
	if (binput_signaltype & T_MODE_CCIR656)
		ucdata |= B_TX_2X656CLK;
	if (binput_signaltype & T_MODE_SYNCEMB)
		ucdata |= B_TX_SYNCEMB;
	if (binput_signaltype & T_MODE_INDDR)
		ucdata |= B_TX_INDDR;
	HDMITX_WRITEI2C_BYTE(REG_TX_INPUT_MODE, ucdata);
}

/* Function: hdmitx_setcscscale */
/* Parameter: binputmode - */
/* D[1:0] - Color Mode */
/* D[4] - colorimetry 0: ITU_BT601 1: ITU_BT709 */
/* D[5] - Quantization 0: 0_255 1: 16_235 */
/* D[6] - Up/Dn Filter 'Required' */
/* 0: no up/down filter */
/* 1: enable up/down filter when csc need. */
/* D[7] - Dither Filter 'Required' */
/* 0: no dither enabled. */
/* 1: enable dither and dither free go "when required". */
/* boutputmode - */
/* D[1:0] - Color mode. */
/* Return: N/A */
/* Remark: reg72~reg8D will be programmed depended the input with table. */
/* Side-Effect: */

void hdmitx_setcscscale(BYTE binputmode, BYTE boutputmode)
{
	BYTE ucdata = 0, csc = B_HDMITX_CSC_BYPASS;
	/* filter is for Video CTRL DN_FREE_GO, EN_DITHER, and ENUDFILT */
	BYTE filter = 0;

	/* (1) YUV422 in, RGB/YUV444 output (Output is 8-bit, input is 12-bit)
	 * (2) YUV444/422 in, RGB output (CSC enable, and output is not YUV422)
	 * (3) RGB in, YUV444 output   (CSC enable, and output is not YUV422)
	 */
	/* YUV444/RGB24 <-> YUV422 need set up/down filter. */
	switch (binputmode & F_MODE_CLRMOD_MASK) {
#ifdef SUPPORT_INPUTYUV444
	case F_MODE_YUV444:
		HDMITX_DEBUG_PRINTF("Input mode is YUV444\n");
		HDMITX_DEBUG_PRINTF("Output mode is ");
		switch (boutputmode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			HDMITX_DEBUG_PRINTF("YUV444\n");
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_YUV422:
			HDMITX_DEBUG_PRINTF("YUV422\n");
			if (binputmode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_RGB444:
			HDMITX_DEBUG_PRINTF("RGB24\n");
			csc = B_HDMITX_CSC_YUV2RGB;
			if (binputmode & F_VIDMODE_EN_DITHER) {
				filter |= B_TX_EN_DITHER;
				filter |= B_TX_DNFREE_GO;
			}
			break;
		}
		break;
#endif

#ifdef SUPPORT_INPUTYUV422
	case F_MODE_YUV422:
		HDMITX_DEBUG_PRINTF("Input mode is YUV422\n");
		HDMITX_DEBUG_PRINTF("Output mode is ");
		switch (boutputmode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			HDMITX_DEBUG_PRINTF("YUV444\n");
			csc = B_HDMITX_CSC_BYPASS;
			if (binputmode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			if (binputmode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
					break;
		case F_MODE_YUV422:
			HDMITX_DEBUG_PRINTF("YUV422\n");
			csc = B_HDMITX_CSC_BYPASS;
			break;

		case F_MODE_RGB444:
			HDMITX_DEBUG_PRINTF("RGB24\n");
			csc = B_HDMITX_CSC_YUV2RGB;
			if (binputmode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			if (binputmode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			break;
		}
		break;
#endif

#ifdef SUPPORT_INPUTRGB
	case F_MODE_RGB444:
		HDMITX_DEBUG_PRINTF("Input mode is RGB24\n");
		HDMITX_DEBUG_PRINTF("Output mode is ");
		switch (boutputmode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			HDMITX_DEBUG_PRINTF("YUV444\n");
			csc = B_HDMITX_CSC_RGB2YUV;

			if (binputmode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER |
					  B_TX_DNFREE_GO;
			break;

		case F_MODE_YUV422:
			HDMITX_DEBUG_PRINTF("YUV422\n");
			if (binputmode & F_VIDMODE_EN_UDFILT)
				filter |= B_TX_EN_UDFILTER;
			if (binputmode & F_VIDMODE_EN_DITHER)
				filter |= B_TX_EN_DITHER |
					  B_TX_DNFREE_GO;
			csc = B_HDMITX_CSC_RGB2YUV;
			break;

		case F_MODE_RGB444:
			HDMITX_DEBUG_PRINTF("RGB24\n");
			csc = B_HDMITX_CSC_BYPASS;
			break;
		}
		break;
#endif
	}
#ifndef DISABLE_HDMITX_CSC

#ifdef SUPPORT_INPUTRGB
	/* set the CSC metrix registers by colorimetry and quantization */
	if (csc == B_HDMITX_CSC_RGB2YUV) {
		HDMITX_DEBUG_PRINTF("CSC = RGB2YUV %x ", csc);
		switch (binputmode & (F_VIDMODE_ITU709 | F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709 | F_VIDMODE_16_235:
			HDMITX_DEBUG_PRINTF("ITU709 16-235 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_rgb2yuv_itu709_16_235[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU709 | F_VIDMODE_0_255:
			HDMITX_DEBUG_PRINTF("ITU709 0-255 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_rgb2yuv_itu709_0_255[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_16_235:
			HDMITX_DEBUG_PRINTF("ITU601 16-235 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_rgb2yuv_itu601_16_235[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_0_255:
		default:
			HDMITX_DEBUG_PRINTF("ITU601 0-255 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_rgb2yuv_itu601_0_255[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		}
	}
#endif

#ifdef SUPPORT_INPUTYUV
	if (csc == B_HDMITX_CSC_YUV2RGB) {
		HDMITX_DEBUG_PRINTF("CSC = YUV2RGB %x ", csc);

		switch (binputmode&(F_VIDMODE_ITU709|F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709|F_VIDMODE_16_235:
			HDMITX_DEBUG_PRINTF("ITU709 16-235 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_yuv2rgb_itu709_16_235[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU709|F_VIDMODE_0_255:
			HDMITX_DEBUG_PRINTF("ITU709 0-255 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_yuv2rgb_itu709_0_255[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU601|F_VIDMODE_16_235:
			HDMITX_DEBUG_PRINTF("ITU601 16-235 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_yuv2rgb_itu601_16_235[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		case F_VIDMODE_ITU601|F_VIDMODE_0_255:
		default:
			HDMITX_DEBUG_PRINTF("ITU601 0-255 ");
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				tmp = bcscmtx_yuv2rgb_itu601_0_255[i];
				HDMITX_WRITEI2C_BYTE(REG_TX_CSC_YOFF + i, tmp);
			}
			break;
		}
	}
#endif
#else/* DISABLE_HDMITX_CSC */
	csc = B_HDMITX_CSC_BYPASS;
#endif/* DISABLE_HDMITX_CSC */

	if (csc == B_HDMITX_CSC_BYPASS)
		HDMITX_SETI2C_BYTE(0xF, 0x10, 0x10);
	else
		HDMITX_SETI2C_BYTE(0xF, 0x10, 0x00);
	ucdata = HDMITX_READI2C_BYTE(REG_TX_CSC_CTRL) &
		~(M_TX_CSC_SEL | B_TX_DNFREE_GO |
		  B_TX_EN_DITHER | B_TX_EN_UDFILTER);
	ucdata |= filter|csc;

	HDMITX_WRITEI2C_BYTE(REG_TX_CSC_CTRL, ucdata);
}

/* Function: hdmitx_setupafe */
/* Parameter: enum pclk_level level */
/* PCLK_LOW - for 13.5MHz (for mode less than 1080p) */
/* PCLK MEDIUM - for 25MHz~74MHz */
/* PCLK HIGH - PCLK > 80Hz (for 1080p mode or above) */
/* Return: N/A */
/* Remark: set reg62~reg65 depended on HighFreqMode */
/* reg61 have to be programmed at last and after video stable input. */
/* Side-Effect: */

void hdmitx_setupafe(enum pclk_level level)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_AFE_DRV_CTRL, B_TX_AFE_DRV_RST);/* 0x10 */
	switch (level) {
	case PCLK_HIGH:
		HDMITX_SETI2C_BYTE(0x62, 0x90, 0x80);
		HDMITX_SETI2C_BYTE(0x64, 0x89, 0x80);
		HDMITX_SETI2C_BYTE(0x68, 0x10, 0x80);
		HDMITX_DEBUG_PRINTF("hdmitx_setupafe() = HIGHT\n");
		break;
	default:
		HDMITX_SETI2C_BYTE(0x62, 0x90, 0x10);
		HDMITX_SETI2C_BYTE(0x64, 0x89, 0x09);
		HDMITX_SETI2C_BYTE(0x68, 0x10, 0x10);
		HDMITX_DEBUG_PRINTF("hdmitx_setupafe() = LOW\n");
		break;
	}
	HDMITX_SETI2C_BYTE(REG_TX_SW_RST,
			   B_TX_REF_RST_HDMITX | B_HDMITX_VID_RST, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_AFE_DRV_CTRL, 0);
	delay1ms(1);
}

/* Function: hdmitx_fireafe */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: write reg61 with 0x04 */
/* When program reg61 with 0x04, then audio and video circuit work. */
/* Side-Effect: N/A */

void hdmitx_fireafe(void)
{
	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_AFE_DRV_CTRL, 0);
}

/* ***************************************** */
/* @file   <hdmitx_aud.c> */
/* ******************************************/

BYTE audiodelaycnt = 0;
BYTE lastrefaudfreqnum = 0;
BOOL bforcects = FALSE;

/* Audio Output */

void sethdmitx_chstat(BYTE uciec60958chstat[])
{
	BYTE uc;

	SWITCH_HDMITX_BANK(1);
	uc = (uciec60958chstat[0] << 1) & 0x7C;
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_MODE, uc);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_CAT, uciec60958chstat[1]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_SRCNUM, uciec60958chstat[2] & 0xF);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD0CHST_CHTNUM,
			     (uciec60958chstat[2] >> 4) & 0xF);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_CA_FS, uciec60958chstat[3]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_OFS_WL, uciec60958chstat[4]);
	SWITCH_HDMITX_BANK(0);
}

void sethdmitx_updatechstatfs(ULONG fs)
{
	BYTE uc;

	/* fs should be the following value. */
	/* #define audfs_22p05khz  4 */
	/* #define audfs_44p1khz 0 */
	/* #define audfs_88p2khz 8 */
	/* #define audfs_176p4khz	12 */
	/*  */
	/* #define audfs_24khz  6 */
	/* #define audfs_48khz  2 */
	/* #define audfs_96khz  10 */
	/* #define audfs_192khz 14 */
	/*  */
	/* #define audfs_768khz 9 */
	/*  */
	/* #define audfs_32khz  3 */
	/* #define AUDFS_OTHER	1 */

	SWITCH_HDMITX_BANK(1);
	uc = HDMITX_READI2C_BYTE(REG_TX_AUDCHST_CA_FS);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_CA_FS, uc);
	uc &= 0xF0;
	uc |= (fs&0xF);

	uc = HDMITX_READI2C_BYTE(REG_TX_AUDCHST_OFS_WL);
	uc &= 0xF;
	uc |= ((~fs) << 4) & 0xF0;
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_OFS_WL, uc);

	SWITCH_HDMITX_BANK(0);
}

void sethdmitx_lpcmaudio(BYTE audiosrcnum, BYTE audswl, BOOL bspdif)
{
	BYTE audioenable, audioformat;

	audioenable = 0;
	audioformat = hdmitxdev[0].boutputaudiomode;

	switch (audswl) {
	case 16:
		audioenable |= M_TX_AUD_16BIT;
		break;
	case 18:
		audioenable |= M_TX_AUD_18BIT;
		break;
	case 20:
		audioenable |= M_TX_AUD_20BIT;
		break;
	case 24:
	default:
		audioenable |= M_TX_AUD_24BIT;
		break;
	}
	if (bspdif) {
		audioformat &= ~0x40;
		audioenable |= B_TX_AUD_SPDIF | B_TX_AUD_EN_I2S0;
	} else {
		audioformat |= 0x40;
		switch (audiosrcnum) {
		case 4:
			audioenable |= B_TX_AUD_EN_I2S3 | B_TX_AUD_EN_I2S2 |
				       B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;
		case 3:
			audioenable |= B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 |
				       B_TX_AUD_EN_I2S0;
			break;

		case 2:
			audioenable |= B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 1:
		default:
			audioformat &= ~0x40;
			audioenable |= B_TX_AUD_EN_I2S0;
			break;
		}
	}
	audioformat |= 0x01;
	hdmitxdev[0].baudiochannelenable = audioenable;

	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0, audioenable & 0xF0);

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL1, audioformat);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_FIFOMAP, 0xE4);
#ifdef USE_SPDIF_CHSTAT
	if (bspdif)
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	else
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, 0);
#else /* not USE_SPDIF_CHSTAT */
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, 0);
#endif /* USE_SPDIF_CHSTAT */

	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_HDAUDIO, 0x00);

	if (bspdif) {
		BYTE i;

		HDMITX_ORREG_BYTE(0x5c, (1 << 6));
		for (i = 0; i < 100; i++) {
			if (HDMITX_READI2C_BYTE(REG_TX_CLK_STATUS2) &
			    B_TX_OSF_LOCK)
				break; /* stable clock. */
		}
	}
}

void sethdmitx_nlpcmaudio(BOOL bspdif)
{
	BYTE audioenable, audioformat;
	BYTE i;

	audioformat = 0x01;
	if (bspdif)
		audioenable = M_TX_AUD_24BIT|B_TX_AUD_SPDIF;
	else
		audioenable = M_TX_AUD_24BIT;

	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0, audioenable);

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL1, 0x01);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_FIFOMAP, 0xE4);

#ifdef USE_SPDIF_CHSTAT
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
#else /* not USE_SPDIF_CHSTAT */
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, 0);
#endif /* USE_SPDIF_CHSTAT */

	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_HDAUDIO, 0x00);

	if (bspdif) {
		for (i = 0; i < 100; i++) {
			if (HDMITX_READI2C_BYTE(REG_TX_CLK_STATUS2) &
			    B_TX_OSF_LOCK)
				break;
		}
	}
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0, audioenable|B_TX_AUD_EN_I2S0);
}

void sethdmitx_hbraudio(BOOL bspdif)
{
	/* BYTE rst; */
	SWITCH_HDMITX_BANK(0);

	/* rst = HDMITX_READI2C_BYTE(REG_TX_SW_RST); */
	/* rst &= ~(B_HDMITX_AUD_RST|B_TX_AREF_RST); */

	/* HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST, rst | B_HDMITX_AUD_RST); */

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL1, 0x47);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_FIFOMAP, 0xE4);

	if (bspdif) {
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT|B_TX_AUD_SPDIF);
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	} else {
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0, M_TX_AUD_24BIT);
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, 0);
	}
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_SRCVALID_FLAT, 0x08);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_HDAUDIO, B_TX_HBR); /* regE5 = 0; */

	/* uc = HDMITX_READI2C_BYTE(REG_TX_CLK_CTRL1); */
	/* uc &= ~M_TX_AUD_DIV; */
	/* HDMITX_WRITEI2C_BYTE(REG_TX_CLK_CTRL1, uc); */

	if (bspdif) {
		BYTE i;

		for (i = 0; i < 100; i++) {
			if (HDMITX_READI2C_BYTE(REG_TX_CLK_STATUS2) &
			    B_TX_OSF_LOCK)
				break; /* stable clock. */
		}
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT | B_TX_AUD_SPDIF |
				     B_TX_AUD_EN_SPDIF);
	} else {
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
				     M_TX_AUD_24BIT | B_TX_AUD_EN_I2S3 |
				     B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 |
				     B_TX_AUD_EN_I2S0);
	}
	HDMITX_ANDREG_BYTE(0x5c, ~(1 << 6));
	hdmitxdev[0].baudiochannelenable =
		HDMITX_READI2C_BYTE(REG_TX_AUDIO_CTRL0);
	/* HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST, rst ); */
}

void sethdmitx_dsdaudio(void)
{
	/* to be continue */
	/* BYTE rst; */
	/* rst = HDMITX_READI2C_BYTE(REG_TX_SW_RST); */

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL1, 0x41);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_FIFOMAP, 0xE4);

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0, M_TX_AUD_24BIT);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL3, 0);

	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_SRCVALID_FLAT, 0x00);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUD_HDAUDIO, B_TX_DSD);

	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
			     M_TX_AUD_24BIT | B_TX_AUD_EN_I2S3 |
			     B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 |
			     B_TX_AUD_EN_I2S0);
}

void hdmitx_disable_audiooutput(void)
{
	audiodelaycnt = audiooutdelaycnt;
	lastrefaudfreqnum = 0;
	HDMITX_SETI2C_BYTE(REG_TX_SW_RST,
			   (B_HDMITX_AUD_RST | B_TX_AREF_RST),
			   (B_HDMITX_AUD_RST | B_TX_AREF_RST));
	HDMITX_SETI2C_BYTE(0x0F, 0x10, 0x10);
}

void hdmitx_enable_audiooutput(BYTE audiotype, BOOL bspdif,
			       ULONG samplefreq, BYTE chnum,
			       BYTE *piec60958chstat, ULONG TMDSCLOCK)
{
	static _IDATA BYTE uciec60958chstat[5];
	BYTE fs;

	audiodelaycnt = 36;
	lastrefaudfreqnum = 0;
	hdmitxdev[0].TMDSCLOCK = TMDSCLOCK;
	hdmitxdev[0].baudiochannelenable = 0;
	hdmitxdev[0].bspdif_out = bspdif;

	HDMITX_ORREG_BYTE(REG_TX_SW_RST, B_HDMITX_AUD_RST | B_TX_AREF_RST);
	HDMITX_WRITEI2C_BYTE(REG_TX_CLK_CTRL0,
			     B_TX_AUTO_OVER_SAMPLING_CLOCK |
			     B_TX_EXT_256FS|0x01);

	HDMITX_SETI2C_BYTE(0x0F, 0x10, 0x00);

	if (bspdif) {
		if (audiotype == T_AUDIO_HBR)
			HDMITX_WRITEI2C_BYTE(REG_TX_CLK_CTRL0, 0x81);
		HDMITX_ORREG_BYTE(REG_TX_AUDIO_CTRL0, B_TX_AUD_SPDIF);
	} else {
		HDMITX_ANDREG_BYTE(REG_TX_AUDIO_CTRL0, (~B_TX_AUD_SPDIF));
	}
	if (audiotype != T_AUDIO_DSD) {
		/* one bit audio have no channel status. */
		switch (samplefreq) {
		case  44100L:
			fs = audfs_44p1khz;
			break;
		case  88200L:
			fs = audfs_88p2khz;
			break;
		case 176400L:
			fs = audfs_176p4khz;
			break;
		case  32000L:
			fs = audfs_32khz;
			break;
		case  48000L:
			fs = audfs_48khz;
			break;
		case  96000L:
			fs = audfs_96khz;
			break;
		case 192000L:
			fs = audfs_192khz;
			break;
		case 768000L:
			fs = audfs_768khz;
			break;
		default:
		      samplefreq = 48000L;
		      fs = audfs_48khz;
		      break; /* default, set fs = 48KHz. */
		}
#ifdef SUPPORT_AUDIO_MONITOR
		hdmitxdev[0].baudfs = AUDFS_OTHER;
#else
		hdmitxdev[0].baudfs = fs;
#endif
		sethdmitx_ncts(hdmitxdev[0].baudfs);
		if (piec60958chstat == NULL) {
			uciec60958chstat[0] = 0;
			uciec60958chstat[1] = 0;
			uciec60958chstat[2] = (chnum + 1) / 2;

			if (uciec60958chstat[2] < 1)
				uciec60958chstat[2] = 1;
			else if (uciec60958chstat[2] > 4)
				uciec60958chstat[2] = 4;
			uciec60958chstat[3] = fs;
			uciec60958chstat[4] = (((~fs) << 4) & 0xF0) |
					      CHTSTS_SWCODE;
			piec60958chstat = uciec60958chstat;
		}
	}
	HDMITX_SETI2C_BYTE(REG_TX_SW_RST,
			   (B_HDMITX_AUD_RST|B_TX_AREF_RST), B_TX_AREF_RST);

	switch (audiotype) {
	case T_AUDIO_HBR:
		HDMITX_DEBUG_PRINTF("T_AUDIO_HBR\n");
		piec60958chstat[0] |= 1 << 1;
		piec60958chstat[2] = 0;
		piec60958chstat[3] &= 0xF0;
		piec60958chstat[3] |= audfs_768khz;
		piec60958chstat[4] |=
			(((~audfs_768khz) << 4) & 0xF0) | 0xB;
		sethdmitx_chstat(piec60958chstat);
		sethdmitx_hbraudio(bspdif);
		break;
	case T_AUDIO_DSD:
		HDMITX_DEBUG_PRINTF("T_AUDIO_DSD\n");
		sethdmitx_dsdaudio();
		break;
	case T_AUDIO_NLPCM:
		HDMITX_DEBUG_PRINTF("T_AUDIO_NLPCM\n");
		piec60958chstat[0] |= 1 << 1;
		sethdmitx_chstat(piec60958chstat);
		sethdmitx_nlpcmaudio(bspdif);
		break;
	case T_AUDIO_LPCM:
		HDMITX_DEBUG_PRINTF("T_AUDIO_LPCM\n");
		piec60958chstat[0] &= ~(1 << 1);

		sethdmitx_chstat(piec60958chstat);
		sethdmitx_lpcmaudio((chnum + 1) / 2,
				    SUPPORT_AUDI_audswl, bspdif);
		/* can add auto adjust */
		break;
	}
	HDMITX_ANDREG_BYTE(REG_TX_INT_MASK1, (~B_TX_AUDIO_OVFLW_MASK));
	HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
			     hdmitxdev[0].baudiochannelenable);

	HDMITX_SETI2C_BYTE(REG_TX_SW_RST, B_HDMITX_AUD_RST | B_TX_AREF_RST, 0);
}

void hdmitx_autoadjustaudio(void)
{
	unsigned long samplefreq, ctmdsclock;
	unsigned long N;
	ULONG acts = 0;
	BYTE fs, uc, loopcnt = 10;

	if (bforcects) {
		SWITCH_HDMITX_BANK(0);
		HDMITX_WRITEI2C_BYTE(0xF8, 0xC3);
		HDMITX_WRITEI2C_BYTE(0xF8, 0xA5);
		HDMITX_ANDREG_BYTE(REG_TX_PKT_SINGLE_CTRL, ~B_TX_SW_CTS);
		HDMITX_WRITEI2C_BYTE(0xF8, 0xFF);
	}
	/* delay1ms(50); */
	SWITCH_HDMITX_BANK(1);
	N = ((unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDN2) & 0xF) << 16;
	N |= ((unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDN1)) << 8;
	N |= ((unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDN0));

	while (loopcnt--) {
		ULONG tempcts = 0;

		acts = (unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT2)
			<< 12;
		acts |= (unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT1)
			<<4;
		acts |= (unsigned long)HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT0)
			& 0xf0 >> 4;
		if (acts == tempcts)
			break;
		tempcts = acts;
	}
	SWITCH_HDMITX_BANK(0);
	if (acts == 0) {
		HDMITX_DEBUG_PRINTF("acts== 0");
		return;
	}
	uc = HDMITX_READI2C_BYTE(REG_TX_GCP);

	ctmdsclock = hdmitxdev[0].TMDSCLOCK;
	/* TMDSCLOCK = GetInputPclk(); */
	HDMITX_DEBUG_PRINTF("PCLK = %u0, 000\n", (WORD)(ctmdsclock/10000));
	switch (uc & 0x70) {
	case 0x50:
		ctmdsclock *= 5;
		ctmdsclock /= 4;
		break;
	case 0x60:
		ctmdsclock *= 3;
		ctmdsclock /= 2;
	}
	samplefreq = ctmdsclock/acts;
	samplefreq *= N;
	samplefreq /= 128;
	/* samplefreq = 48000; */

	HDMITX_DEBUG_PRINTF("samplefreq = %u0\n", (WORD)(samplefreq/10));
	if (samplefreq > 31000L && samplefreq <= 38050L) {
		fs = audfs_32khz;
	} else if (samplefreq < 46550L) {
		fs = audfs_44p1khz;
	} else if (samplefreq < 68100L) {
		fs = audfs_48khz;
	} else if (samplefreq < 92100L) {
		fs = audfs_88p2khz;
	} else if (samplefreq < 136200L) {
		fs = audfs_96khz;
	} else if (samplefreq < 184200L) {
		fs = audfs_176p4khz;
	} else if (samplefreq < 240200L) {
		fs = audfs_192khz;
	} else if (samplefreq < 800000L) {
		fs = audfs_768khz;
	} else {
		fs = AUDFS_OTHER;
		HDMITX_DEBUG_PRINTF("fs = AUDFS_OTHER\n");
	}
	if (hdmitxdev[0].baudfs != fs) {
		hdmitxdev[0].baudfs = fs;
		sethdmitx_ncts(hdmitxdev[0].baudfs);
		/* CurrCTS = 0; */
		return;
	}
}

BOOL hdmitx_isaudiochang(void)
{
	/* ULONG pCTS = 0; */
	BYTE frediff = 0, refaudfreqnum;

	SWITCH_HDMITX_BANK(0);
	refaudfreqnum = HDMITX_READI2C_BYTE(0x60);
	if ((1 << 4) & HDMITX_READI2C_BYTE(0x5f))
		return FALSE;
	if (lastrefaudfreqnum > refaudfreqnum)
		frediff = lastrefaudfreqnum-refaudfreqnum;
	else
		frediff = refaudfreqnum-lastrefaudfreqnum;
	lastrefaudfreqnum = refaudfreqnum;
	if (3 < frediff) {
		HDMITX_DEBUG_PRINTF("Aduio frediff = %d\n", (int)frediff);
		HDMITX_ORREG_BYTE(REG_TX_PKT_SINGLE_CTRL, (1 << 5));
		HDMITX_ANDREG_BYTE(REG_TX_AUDIO_CTRL0, 0xF0);
		return TRUE;
	} else {
		return FALSE;
	}
}

void sethdmitx_audiochannelenable(BOOL enableaudio_b)
{
	static BOOL audiooutstatus = FALSE;
	BYTE audioenable = hdmitxdev[0].baudiochannelenable;

	if (enableaudio_b) {
		if (audiodelaycnt == 0) {
#ifdef SUPPORT_AUDIO_MONITOR
			if (hdmitx_isaudiochang()) {
				hdmitx_autoadjustaudio();
#else
			if (!audiooutstatus) {
				sethdmitx_ncts(hdmitxdev[0].baudfs);
#endif
				HDMITX_WRITEI2C_BYTE(REG_TX_AUD_SRCVALID_FLAT,
						     0);
				HDMITX_ORREG_BYTE(REG_TX_PKT_SINGLE_CTRL,
						  (1 << 5));

				HDMITX_WRITEI2C_BYTE(REG_TX_AUDIO_CTRL0,
						     audioenable);
				HDMITX_ANDREG_BYTE(REG_TX_PKT_SINGLE_CTRL,
						   (~0x3C));
				HDMITX_ANDREG_BYTE(REG_TX_PKT_SINGLE_CTRL,
						   (~(1 << 5)));
				HDMITX_DEBUG_PRINTF("Audio Out Enable\n");
#ifndef SUPPORT_AUDIO_MONITOR
				audiooutstatus = TRUE;
#endif
			}
		} else {
			audiooutstatus = FALSE;
			if (0 ==
			    (HDMITX_READI2C_BYTE(REG_TX_CLK_STATUS2) & 0x10))
					audiodelaycnt--;
				else
					audiodelaycnt = audiooutdelaycnt;
		}
	}
}

/* Function: sethdmitx_ncts */
/* Parameter: PCLK - video clock in Hz. */
/* fs - Encoded audio sample rate */
/* audfs_22p05khz  4 */
/* audfs_44p1khz 0 */
/* audfs_88p2khz 8 */
/* audfs_176p4khz	12 */
/*  */
/* audfs_24khz  6 */
/* audfs_48khz  2 */
/* audfs_96khz  10 */
/* audfs_192khz 14 */
/*  */
/* audfs_768khz 9 */
/*  */
/* audfs_32khz  3 */
/* AUDFS_OTHER	1 */
/* Return: ER_SUCCESS if success */
/* Remark: set N value, the CTS will be auto generated by HW. */
/* Side-Effect: register bank will reset to bank 0. */

void sethdmitx_ncts(BYTE fs)
{
	ULONG n;
	BYTE loopcnt = 255, ctsstablecnt = 0;
	ULONG diff;
	ULONG CTS = 0, lastcts = 0;
	BOOL HBR_mode;
	/* BYTE aVIC; */

	if (B_TX_HBR & HDMITX_READI2C_BYTE(REG_TX_AUD_HDAUDIO))
		HBR_mode = TRUE;
	else
		HBR_mode = FALSE;
	switch (fs) {
	case audfs_32khz:
		n = 4096;
		break;
	case audfs_44p1khz:
		n = 6272;
		break;
	case audfs_48khz:
		n = 6144;
		break;
	case audfs_88p2khz:
		n = 12544;
		break;
	case audfs_96khz:
		n = 12288;
		break;
	case audfs_176p4khz:
		n = 25088;
		break;
	case audfs_192khz:
		n = 24576;
		break;
	case audfs_768khz:
		n = 24576;
		break;
	default:
		n = 6144;
		break;
	}
	SWITCH_HDMITX_BANK(1);
	HDMITX_WRITEI2C_BYTE(REGPKTAUDN0, (BYTE)((n)&0xFF));
	HDMITX_WRITEI2C_BYTE(REGPKTAUDN1, (BYTE)((n>>8)&0xFF));
	HDMITX_WRITEI2C_BYTE(REGPKTAUDN2, (BYTE)((n>>16)&0xF));

	if (bforcects) {
		ULONG sumcts = 0;

		while (loopcnt--) {
			delay1ms(30);
			CTS = HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT2) << 12;
			CTS |= HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT1) << 4;
			CTS |= (HDMITX_READI2C_BYTE(REGPKTAUDCTSCNT0) &
				0xf0) >> 4;
			if (CTS == 0) {
				continue;
			} else {
				if (lastcts > CTS)
					diff = lastcts-CTS;
				else
					diff = CTS-lastcts;
				lastcts = CTS;
				if (5 > diff) {
					ctsstablecnt++;
					sumcts += CTS;
				} else {
					ctsstablecnt = 0;
					sumcts = 0;
					continue;
				}
				if (ctsstablecnt >= 32) {
					lastcts = sumcts >> 5;
					break;
				}
			}
		}
	}
	HDMITX_WRITEI2C_BYTE(REGPKTAUDCTS0, (BYTE)((lastcts)&0xFF));
	HDMITX_WRITEI2C_BYTE(REGPKTAUDCTS1, (BYTE)((lastcts>>8)&0xFF));
	HDMITX_WRITEI2C_BYTE(REGPKTAUDCTS2, (BYTE)((lastcts>>16)&0xF));
	SWITCH_HDMITX_BANK(0);
#ifdef force_cts
	bforcects = TRUE;
#endif
	HDMITX_WRITEI2C_BYTE(0xF8, 0xC3);
	HDMITX_WRITEI2C_BYTE(0xF8, 0xA5);
	if (bforcects)
		HDMITX_ORREG_BYTE(REG_TX_PKT_SINGLE_CTRL, B_TX_SW_CTS);
	else
		HDMITX_ANDREG_BYTE(REG_TX_PKT_SINGLE_CTRL, ~B_TX_SW_CTS);
	HDMITX_WRITEI2C_BYTE(0xF8, 0xFF);

	if (!HBR_mode) {
		BYTE udata;

		SWITCH_HDMITX_BANK(1);
		fs = audfs_768khz;
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_CA_FS, 0x00|fs);
		fs = ~fs; /* OFS is the one's complement of FS */
		udata = (0x0f&HDMITX_READI2C_BYTE(REG_TX_AUDCHST_OFS_WL));
		HDMITX_WRITEI2C_BYTE(REG_TX_AUDCHST_OFS_WL, (fs << 4)|udata);
		SWITCH_HDMITX_BANK(0);
	}
}

/* ***************************************** */
/* @file   <hdmitx_pkt.c> */
/* ******************************************/
BOOL hdmitx_enablevsinfoframe(BYTE benable, BYTE *pvsinfoframe)
{
	if (!benable) {
		hdmitx_DISABLE_VSDB_PKT();
		return TRUE;
	}
	if (hdmitx_setvsiinfoframe((union vendorspecific_infoframe *)
				   pvsinfoframe) == ER_SUCCESS)
		return TRUE;
	return FALSE;
}

BOOL hdmitx_enableaviinfoframe(BYTE benable, BYTE *paviinfoframe)
{
	if (!benable) {
		hdmitx_DISABLE_AVI_INFOFRM_PKT();
		return TRUE;
	}
	if (hdmitx_setaviinfoframe((union avi_infoframe *)
				   paviinfoframe) == ER_SUCCESS)
		return TRUE;
	return FALSE;
}

BOOL hdmitx_enableaudioinfoframe(BYTE benable, BYTE *paudioinfoframe)
{
	if (!benable) {
		hdmitx_DISABLE_AVI_INFOFRM_PKT();
		return TRUE;
	}
	if (hdmitx_setaudioinfoframe((union audio_infoframe *)
				     paudioinfoframe) == ER_SUCCESS)
		return TRUE;
	return FALSE;
}

/* Function: hdmitx_setaviinfoframe() */
/* Parameter: paviinfoframe - the pointer to HDMI AVI Infoframe ucdata */
/* Return: N/A */
/* Remark: Fill the AVI InfoFrame ucdata, and count checksum, then fill into */
/* AVI InfoFrame registers. */
/* Side-Effect: N/A */

enum SYS_STATUS hdmitx_setaviinfoframe(union avi_infoframe *paviinfoframe)
{
	int i;
	BYTE checksum;
	union avi_infoframe *info = paviinfoframe;

	if (!info)
		return ER_FAIL;
	SWITCH_HDMITX_BANK(1);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB1, info->pktbyte.AVI_DB[0]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB2, info->pktbyte.AVI_DB[1]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB3, info->pktbyte.AVI_DB[2]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB4, info->pktbyte.AVI_DB[3]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB5, info->pktbyte.AVI_DB[4]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB6, info->pktbyte.AVI_DB[5]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB7, info->pktbyte.AVI_DB[6]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB8, info->pktbyte.AVI_DB[7]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB9, info->pktbyte.AVI_DB[8]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB10, info->pktbyte.AVI_DB[9]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB11, info->pktbyte.AVI_DB[10]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB12, info->pktbyte.AVI_DB[11]);
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_DB13, info->pktbyte.AVI_DB[12]);
	for (i = 0, checksum = 0; i < 13; i++)
		checksum -= info->pktbyte.AVI_DB[i];
	checksum -= AVI_INFOFRAME_VER+AVI_INFOFRAME_TYPE+AVI_INFOFRAME_LEN;
	HDMITX_WRITEI2C_BYTE(REG_TX_AVIINFO_SUM, checksum);

	SWITCH_HDMITX_BANK(0);
	hdmitx_ENABLE_AVI_INFOFRM_PKT();
	return ER_SUCCESS;
}

/* Function: hdmitx_setaudioinfoframe()
 * Parameter: paudioinfoframe - the pointer to HDMI Audio Infoframe ucdata
 * Return: N/A
 * Remark: Fill the Audio InfoFrame ucdata, and count checksum, then fill into
 * Audio InfoFrame registers.
 * Side-Effect: N/A
 */

enum SYS_STATUS
hdmitx_setaudioinfoframe(union audio_infoframe *paudioinfoframe)
{
	BYTE checksum;

	if (!paudioinfoframe)
		return ER_FAIL;
	SWITCH_HDMITX_BANK(1);
	checksum = 0x100 - (AUDIO_INFOFRAME_VER + AUDIO_INFOFRAME_TYPE +
			    AUDIO_INFOFRAME_LEN);
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_AUDINFO_CC,
			     paudioinfoframe->pktbyte.AUD_DB[0]);
	checksum -= HDMITX_READI2C_BYTE(REG_TX_PKT_AUDINFO_CC);
	checksum &= 0xFF;
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_AUDINFO_SF,
			     paudioinfoframe->pktbyte.AUD_DB[1]);
	checksum -= HDMITX_READI2C_BYTE(REG_TX_PKT_AUDINFO_SF);
	checksum &= 0xFF;
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_AUDINFO_CA,
			     paudioinfoframe->pktbyte.AUD_DB[3]);
	checksum -= HDMITX_READI2C_BYTE(REG_TX_PKT_AUDINFO_CA);
	checksum &= 0xFF;
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_AUDINFO_DM_LSV,
			     paudioinfoframe->pktbyte.AUD_DB[4]);
	checksum -= HDMITX_READI2C_BYTE(REG_TX_PKT_AUDINFO_DM_LSV);
	checksum &= 0xFF;

	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_AUDINFO_SUM, checksum);

	SWITCH_HDMITX_BANK(0);
	hdmitx_ENABLE_AUD_INFOFRM_PKT();
	return ER_SUCCESS;
}

/* Function: hdmitx_setspdinfoframe() */
/* Parameter: pspdinfoframe - the pointer to HDMI SPD Infoframe ucdata */
/* Return: N/A */
/* Remark: Fill the SPD InfoFrame ucdata, and count checksum, then fill into */
/* SPD InfoFrame registers. */
/* Side-Effect: N/A */

enum SYS_STATUS hdmitx_setspdinfoframe(union spd_infoframe *pspdinfoframe)
{
	int i;
	BYTE ucdata;

	if (!pspdinfoframe)
		return ER_FAIL;
	SWITCH_HDMITX_BANK(1);
	for (i = 0, ucdata = 0; i < 25; i++) {
		ucdata -= pspdinfoframe->pktbyte.SPD_DB[i];
		HDMITX_WRITEI2C_BYTE(REG_TX_PKT_SPDINFO_PB1 + i,
				     pspdinfoframe->pktbyte.SPD_DB[i]);
	}
	ucdata -= SPD_INFOFRAME_VER+SPD_INFOFRAME_TYPE + SPD_INFOFRAME_LEN;
	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_SPDINFO_SUM, ucdata); /* checksum */
	SWITCH_HDMITX_BANK(0);
	hdmitx_ENABLE_SPD_INFOFRM_PKT();
	return ER_SUCCESS;
}

/* Function: hdmitx_setmpeginfoframe() */
/* Parameter: pMPEGInfoFrame - the pointer to HDMI MPEG Infoframe ucdata */
/* Return: N/A */
/* Remark: Fill the MPEG InfoFrame ucdata, and count checksum, then fill into */
/* MPEG InfoFrame registers. */
/* Side-Effect: N/A */

enum SYS_STATUS hdmitx_setmpeginfoframe(union mpeg_infoframe *pmpginfoframe)
{
	int i;
	BYTE ucdata;

	if (!pmpginfoframe)
		return ER_FAIL;
	SWITCH_HDMITX_BANK(1);

	HDMITX_WRITEI2C_BYTE(REG_TX_PKT_MPGINFO_FMT,
			     pmpginfoframe->info.fieldrepeat |
			     (pmpginfoframe->info.mpegframe << 1));
	HDMITX_WRITEI2C_BYTE(REG_TX_PKG_MPGINFO_DB0,
			     pmpginfoframe->pktbyte.MPG_DB[0]);
	HDMITX_WRITEI2C_BYTE(REG_TX_PKG_MPGINFO_DB1,
			     pmpginfoframe->pktbyte.MPG_DB[1]);
	HDMITX_WRITEI2C_BYTE(REG_TX_PKG_MPGINFO_DB2,
			     pmpginfoframe->pktbyte.MPG_DB[2]);
	HDMITX_WRITEI2C_BYTE(REG_TX_PKG_MPGINFO_DB3,
			     pmpginfoframe->pktbyte.MPG_DB[3]);

	for (ucdata = 0, i = 0; i < 5; i++)
		ucdata -= pmpginfoframe->pktbyte.MPG_DB[i];
	ucdata -= MPEG_INFOFRAME_VER + MPEG_INFOFRAME_TYPE + MPEG_INFOFRAME_LEN;

	HDMITX_WRITEI2C_BYTE(REG_TX_PKG_MPGINFO_SUM, ucdata);

	SWITCH_HDMITX_BANK(0);
	hdmitx_ENABLE_SPD_INFOFRM_PKT();

	return ER_SUCCESS;
}

/* 2009/12/04 added by Ming-chih.lung@ite.com.tw */

enum SYS_STATUS
hdmitx_setvsiinfoframe(union vendorspecific_infoframe *pvsiinfoframe)
{
	BYTE ucdata = 0;

	if (!pvsiinfoframe)
		return ER_FAIL;

	SWITCH_HDMITX_BANK(1);
	HDMITX_WRITEI2C_BYTE(0x80, pvsiinfoframe->pktbyte.VS_DB[3]);
	HDMITX_WRITEI2C_BYTE(0x81, pvsiinfoframe->pktbyte.VS_DB[4]);

	ucdata -= pvsiinfoframe->pktbyte.VS_DB[3];
	ucdata -= pvsiinfoframe->pktbyte.VS_DB[4];

	if (pvsiinfoframe->pktbyte.VS_DB[4] & (1 << 7)) {
		ucdata -= pvsiinfoframe->pktbyte.VS_DB[5];
		HDMITX_WRITEI2C_BYTE(0x82, pvsiinfoframe->pktbyte.VS_DB[5]);
		ucdata -= VENDORSPEC_INFOFRAME_TYPE +
			  VENDORSPEC_INFOFRAME_VER + 6 + 0x0C + 0x03;
	} else {
		ucdata -= VENDORSPEC_INFOFRAME_TYPE +
			  VENDORSPEC_INFOFRAME_VER + 5 + 0x0C + 0x03;
	}

	pvsiinfoframe->pktbyte.checksum = ucdata;

	HDMITX_WRITEI2C_BYTE(0x83, pvsiinfoframe->pktbyte.checksum);
	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_3D_INFO_CTRL,
			     B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
	return ER_SUCCESS;
}

enum SYS_STATUS hdmitx_set_generalpurpose_pkt(BYTE *pdata)
{
	int i;

	if (pdata == NULL)
		return ER_FAIL;
	SWITCH_HDMITX_BANK(1);
	for (i = 0x38; i <= 0x56; i++)
		HDMITX_WRITEI2C_BYTE(i, pdata[i-0x38]);
	SWITCH_HDMITX_BANK(0);
	hdmitx_ENABLE_GENERALPURPOSE_PKT();
	/* hdmitx_ENABLE_NULL_PKT(); */
	return ER_SUCCESS;
}

/* Function: dumphdmitxreg() */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: Debug function, dumps the registers of CAT6611. */
/* Side-Effect: N/A */

#if debug_message
void dumphdmitxreg(void)
{
	int i, j;
	BYTE tmp;
	BYTE ucdata;

	HDMITX_DEBUG_PRINTF("[%s]\n", __func__);
	HDMITX_DEBUG_PRINTF("	   ");
	for (j = 0; j < 16; j++) {
		HDMITX_DEBUG_PRINTF(" %02X", (int)j);
		if ((j == 3) || (j == 7) || (j == 11))
			HDMITX_DEBUG_PRINTF("  ");
	}
	HDMITX_DEBUG_PRINTF("\n------------------\n");

	SWITCH_HDMITX_BANK(0);

	for (i = 0; i < 0x100; i += 16) {
		HDMITX_DEBUG_PRINTF("[%3X]  ", i);
		for (j = 0; j < 16; j++) {
			tmp = (BYTE)((i + j) & 0xFF);
			if ((i + j) != 0x17)
				ucdata = HDMITX_READI2C_BYTE(tmp);
			else
				HDMITX_DEBUG_PRINTF(" XX");
			if ((j == 3) || (j == 7) || (j == 11))
				HDMITX_DEBUG_PRINTF(" -");
		}
		HDMITX_DEBUG_PRINTF("\n");
		if ((i % 0x40) == 0x30)
			HDMITX_DEBUG_PRINTF("------------");
	}
	SWITCH_HDMITX_BANK(1);
	for (i = 0x130; i < 0x200; i += 16) {
		HDMITX_DEBUG_PRINTF("[%3X]  ", i);
		for (j = 0; j < 16; j++) {
			tmp = (BYTE)((i + j) & 0xFF);
			ucdata = HDMITX_READI2C_BYTE(tmp);
			HDMITX_DEBUG_PRINTF(" %02X", (int)ucdata);
			if ((j == 3) || (j == 7) || (j == 11))
				HDMITX_DEBUG_PRINTF(" -");
		}
		HDMITX_DEBUG_PRINTF("\n");
		if ((i % 0x40) == 0x20)
			HDMITX_DEBUG_PRINTF("------------");
	}
	HDMITX_DEBUG_PRINTF("------------");
	SWITCH_HDMITX_BANK(0);
}
#endif
