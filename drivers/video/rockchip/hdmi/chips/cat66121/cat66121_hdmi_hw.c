/*
 * Copyright (C) 2012-2015 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include "cat66121_hdmi.h"
#include "cat66121_hdmi_hw.h"
#include "hdmitx.h"

#define HDMITX_INPUT_SIGNAL_TYPE 0  /* for default(Sync Sep Mode) */
#define INPUT_SPDIF_ENABLE	0
/*******************************
 * Global Data
 ******************************/
static _XDATA unsigned char communbuff[128];
static unsigned int pixelrep;
static BYTE binputcolormode = F_MODE_RGB444;
/* static BYTE binputcolormode = INPUT_COLOR_MODE; */
static char boutputcolormode = F_MODE_RGB444;
#ifdef SUPPORT_HDCP
static void hdcp_delay_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(hdcp_delay_work, hdcp_delay_work_func);
#endif
static DEFINE_MUTEX(handler_mutex);

struct hdmitx_dev instancedata = {
	0,	  /* BYTE I2C_DEV; */
	HDMI_TX_I2C_SLAVE_ADDR,	/* BYTE I2C_ADDR; */
	/* Interrupt type */

	0x40,	  /* BYTE binttype; // = 0; */

	/* Video Property */
	INPUT_SIGNAL_TYPE,

	/* Audio Property */
	I2S_FORMAT, /* BYTE boutputaudiomode; // = 0; */
	FALSE , /* BYTE baudiochannelswap; // = 0; */
	0x01, /* BYTE baudiochannelenable; */
	INPUT_SAMPLE_FREQ , /* BYTE baudfs; */
	0, /* unsigned long TMDSCLOCK; */
	FALSE, /* BYTE bauthenticated:1; */
	FALSE, /* BYTE bhdmimode: 1; */
	FALSE, /* BYTE bintpol:1; // 0 = Low Active */
	FALSE, /* BYTE bhpd:1; */
};

/* I2C read/write funcs */
BYTE HDMITX_READI2C_BYTE(BYTE regaddr)
{
	struct i2c_msg msgs[2];
	enum SYS_STATUS ret = -1;
	BYTE buf[1];

	buf[0] = regaddr;

	/* Write device addr fisrt */
	msgs[0].addr = cat66121_hdmi->client->addr;
	msgs[0].flags = !I2C_M_RD;
	msgs[0].len = 1;
	msgs[0].buf = &buf[0];
	/* Then, begin to read data */
	msgs[1].addr = cat66121_hdmi->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &buf[0];

	ret = i2c_transfer(cat66121_hdmi->client->adapter, msgs, 2);

	return buf[0];
}

enum SYS_STATUS HDMITX_WRITEI2C_BYTE(BYTE regaddr, BYTE data)
{
	struct i2c_msg msg;
	enum SYS_STATUS ret = -1;
	BYTE buf[2];

	buf[0] = regaddr;
	buf[1] = data;

	msg.addr = cat66121_hdmi->client->addr;
	msg.flags = !I2C_M_RD;
	msg.len	= 2;
	msg.buf	= buf;

	ret = i2c_transfer(cat66121_hdmi->client->adapter, &msg, 1);

	return ret;
}

enum SYS_STATUS HDMITX_READI2C_BYTEN(BYTE regaddr, BYTE *pdata, int N)
{
	struct i2c_msg msgs[2];
	enum SYS_STATUS ret = -1;

	pdata[0] = regaddr;

	msgs[0].addr	= cat66121_hdmi->client->addr;
	msgs[0].flags	= !I2C_M_RD;
	msgs[0].len		= 1;
	msgs[0].buf		= &pdata[0];

	msgs[1].addr	= cat66121_hdmi->client->addr;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len		= N;
	msgs[1].buf		= pdata;

	ret = i2c_transfer(cat66121_hdmi->client->adapter, msgs, 2);

	return ret;
}

enum SYS_STATUS HDMITX_WRITEI2C_BYTEN(BYTE regaddr, BYTE *pdata, int N)
{
	struct i2c_msg msg;
	enum SYS_STATUS ret = -1;
	BYTE buf[N + 1];

	buf[0] = regaddr;
	memcpy(&buf[1], pdata, N);

	msg.addr	= cat66121_hdmi->client->addr;
	msg.flags	= !I2C_M_RD;
	msg.len		= N + 1;
	msg.buf		= buf;		/* gModify.Exp."Include regaddr" */

	ret = i2c_transfer(cat66121_hdmi->client->adapter, &msg, 1);

	return ret;
}

enum SYS_STATUS HDMITX_SETI2C_BYTE(BYTE reg, BYTE mask, BYTE value)
{
	BYTE temp;

	if (mask != 0xFF) {
		temp = HDMITX_READI2C_BYTE(reg);
		temp &= (~mask);
		temp |= value&mask;
	} else {
		temp = value;
	}
	return HDMITX_WRITEI2C_BYTE(reg, temp);
}

int cat66121_detect_device(void)
{
	uint8_t vendorid0, vendorid1, deviceid0, deviceid1;

	SWITCH_HDMITX_BANK(0);
	vendorid0 = HDMITX_READI2C_BYTE(REG_TX_VENDOR_ID0);
	vendorid1 = HDMITX_READI2C_BYTE(REG_TX_VENDOR_ID1);
	deviceid0 = HDMITX_READI2C_BYTE(REG_TX_DEVICE_ID0);
	deviceid1 = HDMITX_READI2C_BYTE(REG_TX_DEVICE_ID1);
	pr_info("CAT66121: reg[0-3] = 0x[%02x].[%02x].[%02x].[%02x]\n",
		vendorid0, vendorid1, deviceid0, deviceid1);
	if ((vendorid0 == 0x54) && (vendorid1 == 0x49))
		/*	&&(deviceid0 == 0x12) && (deviceid1 == 0x16)) */
		return 1;

	pr_info("[CAT66121] Device not found!\n");

	return 0;
}

int cat66121_hdmi_sys_init(struct hdmi *hdmi_drv)
{
	hdmi_dbg(hdmi_drv->dev, "[%s]\n", __func__);
	hdmitx_init_txdev(&instancedata);
	init_hdmitx();
	usleep_range(500, 1000);

	return HDMI_ERROR_SUCCESS;
}

#ifdef SUPPORT_HDCP
static void hdcp_delay_work_func(struct work_struct *work)
{
	if (0 == (B_TXVIDSTABLE&HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS)))
		schedule_delayed_work(&hdcp_delay_work, msecs_to_jiffies(100));
	else
		hdmitx_enablehdcp(TRUE);
}
#endif

void cat66121_interruptclr(void)
{
	char intclr3, intdata4;

	intdata4 = HDMITX_READI2C_BYTE(0xEE);
	HDMITX_DEBUG_PRINTF("REG_TX_INT_STAT4=%x\n", intdata4);
	intclr3 = (HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS)) |
		  B_TX_CLR_AUD_CTS | B_TX_INTACTDONE;
	if (intdata4) {
		HDMITX_WRITEI2C_BYTE(0xEE, intdata4);
		HDMITX_DEBUG_PRINTF("%s%s%s%s%s%s%s\n",
				    (intdata4 & 0x40) ?
				    "video parameter change\n" : "",
				    (intdata4 & 0x20) ?
				    "HDCP Pj check done\n" : "",
				    (intdata4 & 0x10) ?
				    "HDCP Ri check done\n" : "",
				    (intdata4 & 0x8) ? "DDC bus hang\n" : "",
				    (intdata4 & 0x4) ?
				    "Video input FIFO auto reset\n" : "",
				    (intdata4 & 0x2) ?
				    "No audio input interrupt\n" : "",
				    (intdata4 & 0x1) ?
				    "Audio decode error interrupt\n" : "");
	}

	HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR0, 0xFF);
	HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR1, 0xFF);
	HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS, intclr3);
	intclr3 &= ~(B_TX_INTACTDONE);
	HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS, intclr3);
}

void cat66121_hdmi_interrupt(struct hdmi *hdmi_drv)
{
	char sysstat = 0;

	mutex_lock(&handler_mutex);
	sysstat = HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS);

	if ((sysstat & B_TX_INT_ACTIVE) ||
	    ((B_TX_HPDETECT & cat66121_hdmi->plug_status) !=
	     (B_TX_HPDETECT & sysstat)))  {
		char intdata1, intdata2, intdata3;

		intdata1 = HDMITX_READI2C_BYTE(REG_TX_INT_STAT1);
		intdata2 = HDMITX_READI2C_BYTE(REG_TX_INT_STAT2);
		intdata3 = HDMITX_READI2C_BYTE(REG_TX_INT_STAT3);
		HDMITX_DEBUG_PRINTF("REG_TX_INT_STAT1=%x\n", intdata1);
		HDMITX_DEBUG_PRINTF("REG_TX_INT_STAT2=%x\n", intdata2);
		HDMITX_DEBUG_PRINTF("REG_TX_INT_STAT3=%x\n", intdata3);
		if (gethdmitx_powerstatus() == FALSE)
			hdmitx_poweron();

		/******* Clear interrupt **********/
		cat66121_interruptclr();
		/******** handler interrupt event ********/

		if (intdata1 & B_TX_INT_DDCFIFO_ERR) {
			HDMITX_DEBUG_PRINTF("DDC FIFO Error.\n");
			hdmitx_clearddcfifo();
		}
		if (intdata1 & B_TX_INT_DDC_BUS_HANG) {
			HDMITX_DEBUG_PRINTF("DDC BUS HANG.\n");
			hdmitx_abortddc();
#ifdef SUPPORT_HDCP
			if (hdmitxdev[0].bauthenticated)
				hdmitx_hdcp_resumeauthentication();
#endif
		}
		if (intdata1 & B_TX_INT_AUD_OVERFLOW) {
			HDMITX_DEBUG_PRINTF("AUDIO FIFO OVERFLOW.\n");
			HDMITX_ORREG_BYTE(REG_TX_SW_RST,
					  (B_HDMITX_AUD_RST|B_TX_AREF_RST));
			HDMITX_ANDREG_BYTE(REG_TX_SW_RST,
					   ~(B_HDMITX_AUD_RST|B_TX_AREF_RST));
		}

		if (intdata3 & B_TX_INT_VIDSTABLE) {
			sysstat = HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS);
			if (sysstat & B_TXVIDSTABLE)
				hdmitx_fireafe();
		}

#ifdef SUPPORT_HDCP
		if (intdata2 & B_TX_INT_AUTH_FAIL) {
			hdmitxdev[0].bauthenticated = FALSE;
			hdmitx_abortddc();
		} else if (intdata2 & B_TX_INT_AUTH_DONE) {
			HDMITX_SETI2C_BYTE(REG_TX_INT_MASK2,
					   B_TX_AUTH_DONE_MASK,
					   B_TX_AUTH_DONE_MASK);
			HDMITX_DEBUG_PRINTF("authenticationdone ==SUCCESS\n");
		}
#endif
		if ((intdata1 & B_TX_INT_HPD_PLUG) ||
		    ((B_TX_HPDETECT & cat66121_hdmi->plug_status) !=
		     (B_TX_HPDETECT & sysstat))) {
			hdmitxdev[0].bauthenticated = FALSE;
			if (sysstat & B_TX_HPDETECT)
				HDMITX_DEBUG_PRINTF("HPD plug\n");
			else
				HDMITX_DEBUG_PRINTF("HPD unplug\n");
			cat66121_hdmi->plug_status = sysstat;
			if (hdmi_drv->state == HDMI_SLEEP)
				hdmi_drv->state = WAIT_HOTPLUG;
			queue_delayed_work(hdmi_drv->workqueue,
					   &hdmi_drv->delay_work,
					   msecs_to_jiffies(0));
		}
		if (intdata1 & (B_TX_INT_RX_SENSE))
			hdmitxdev[0].bauthenticated = FALSE;
	}

#ifdef SUPPORT_HDCP
	if (hdmi_drv->display == HDMI_ENABLE) {
		if (gethdmitx_linkstatus()) {
			/* AudioModeDetect(); */
			if (gethdmitx_authenticationdone() == FALSE) {
				HDMITX_DEBUG_PRINTF("authentication==FALSE\n");
				hdmitx_enablehdcp(TRUE);
				sethdmitx_avmute(FALSE);
			}
		}
	}
#endif

	mutex_unlock(&handler_mutex);
}

int cat66121_hdmi_sys_detect_hpd(struct hdmi *hdmi_drv)
{
	char HPD = 0;
	BYTE sysstat;

#ifdef SUPPORT_HDCP
	if ((cat66121_hdmi->plug_status != 0) &&
	    (cat66121_hdmi->plug_status != 1))
		cat66121_hdmi->plug_status =
			HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS);

	sysstat = cat66121_hdmi->plug_status;
#else
	sysstat = HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS);
#endif

	HPD = ((sysstat & B_TX_HPDETECT) == B_TX_HPDETECT) ? TRUE : FALSE;
	if (HPD)
		return HDMI_HPD_ACTIVED;
	else
		return HDMI_HPD_REMOVED;
}

int cat66121_hdmi_sys_read_edid(struct hdmi *hdmi_drv,
				int block, unsigned char *buff)
{
	hdmi_dbg(hdmi_drv->dev, "[%s]\n", __func__);
	return (gethdmitx_edidblock(block, buff) == TRUE) ?
			HDMI_ERROR_SUCCESS : HDMI_ERROR_FALSE;
}

void configfhdmivendorspecificinfoframe(BYTE _3d_stru)
{
	union vendorspecific_infoframe *vs_info;

	vs_info = (union vendorspecific_infoframe *)communbuff;

	vs_info->pktbyte.VS_HB[0] = VENDORSPEC_INFOFRAME_TYPE|0x80;
	vs_info->pktbyte.VS_HB[1] = VENDORSPEC_INFOFRAME_VER;
	vs_info->pktbyte.VS_HB[2] = (_3d_stru == SIDE_BY_SIDE) ? 6 : 5;
	vs_info->pktbyte.VS_DB[0] = 0x03;
	vs_info->pktbyte.VS_DB[1] = 0x0C;
	vs_info->pktbyte.VS_DB[2] = 0x00;
	vs_info->pktbyte.VS_DB[3] = 0x40;
	switch (_3d_stru) {
	case SIDE_BY_SIDE:
	case FRAME_PCAKING:
	case TOP_AND_BOTTON:
		vs_info->pktbyte.VS_DB[4] = (_3d_stru << 4);
		break;
	default:
		vs_info->pktbyte.VS_DB[4] = (FRAME_PCAKING << 4);
		break;
	}
	vs_info->pktbyte.VS_DB[5] = 0x00;
	hdmitx_enablevsinfoframe(TRUE, (BYTE *)vs_info);
}

static void cat66121_sys_config_avi(int VIC, int boutputcolormode,
				    int aspec, int colorimetry, int pixelrep)
{
	union avi_infoframe *aviinfo;

	aviinfo = (union avi_infoframe *)communbuff;

	aviinfo->pktbyte.AVI_HB[0] = AVI_INFOFRAME_TYPE|0x80;
	aviinfo->pktbyte.AVI_HB[1] = AVI_INFOFRAME_VER;
	aviinfo->pktbyte.AVI_HB[2] = AVI_INFOFRAME_LEN;

	switch (boutputcolormode) {
	case F_MODE_YUV444:
		/* aviinfo->info.colormode = 2; */
		aviinfo->pktbyte.AVI_DB[0] = (2 << 5)|(1 << 4);
		break;
	case F_MODE_YUV422:
		/* aviinfo->info.colormode = 1; */
		aviinfo->pktbyte.AVI_DB[0] = (1 << 5)|(1 << 4);
		break;
	case F_MODE_RGB444:
	default:
		/* aviinfo->info.colormode = 0; */
		aviinfo->pktbyte.AVI_DB[0] = (0 << 5)|(1 << 4);
		break;
	}
	aviinfo->pktbyte.AVI_DB[0] |= 0x02;
	aviinfo->pktbyte.AVI_DB[1] = 8;
	aviinfo->pktbyte.AVI_DB[1] |=
		(aspec != HDMI_16x9) ? (1 << 4) : (2 << 4);
	aviinfo->pktbyte.AVI_DB[1] |=
		(colorimetry != HDMI_ITU709) ? (1 << 6) : (2 << 6);
	aviinfo->pktbyte.AVI_DB[2] = 0;
	aviinfo->pktbyte.AVI_DB[3] = VIC;
	aviinfo->pktbyte.AVI_DB[4] = pixelrep & 3;
	aviinfo->pktbyte.AVI_DB[5] = 0;
	aviinfo->pktbyte.AVI_DB[6] = 0;
	aviinfo->pktbyte.AVI_DB[7] = 0;
	aviinfo->pktbyte.AVI_DB[8] = 0;
	aviinfo->pktbyte.AVI_DB[9] = 0;
	aviinfo->pktbyte.AVI_DB[10] = 0;
	aviinfo->pktbyte.AVI_DB[11] = 0;
	aviinfo->pktbyte.AVI_DB[12] = 0;

	hdmitx_enableaviinfoframe(TRUE, (unsigned char *)aviinfo);
}

int cat66121_hdmi_sys_config_video(struct hdmi *hdmi_drv,
				   struct hdmi_video_para *vpara)
{
	struct fb_videomode *mode;
	enum hdmi_aspec aspec;
	enum hdmi_colorimetry colorimetry;
	enum pclk_level level;

	if (vpara == NULL) {
		hdmi_err(hdmi_drv->dev, "[%s] input parameter error\n",
			 __func__);
		return -1;
	}

#ifdef SUPPORT_HDCP
	hdmitx_enablehdcp(FALSE);
#endif

	/* output Color mode */
#ifndef DISABLE_HDMITX_CSC
	switch (vpara->output_color) {
	case HDMI_COLOR_YCBCR444:
		boutputcolormode = F_MODE_YUV444;
		break;
	case HDMI_COLOR_YCBCR422:
		boutputcolormode = F_MODE_YUV422;
		break;
	case HDMI_COLOR_RGB:
	default:
		boutputcolormode = F_MODE_RGB444;
		break;
	}
#else
	if (vpara->input_mode == VIDEO_INPUT_YCBCR422_EMBEDDED_SYNC) {
		binputcolormode = F_MODE_YUV422;
		boutputcolormode = F_MODE_YUV422;
	} else {
		binputcolormode = F_MODE_RGB444;
		boutputcolormode = F_MODE_RGB444;
	}

	/*	boutputcolormode = F_MODE_RGB444; */
#endif

	/* binputcolormode = F_MODE_YUV422; */
	/* boutputcolormode = F_MODE_YUV422; */
	/* Set ext video */
	mode = (struct fb_videomode *)hdmi_vic_to_videomode(vpara->vic);
	if (mode == NULL) {
		hdmi_err(hdmi_drv->dev, "[%s] not found vic %d\n",
			 __func__, vpara->vic);
		return -ENOENT;
	}

	hdmi_drv->tmdsclk = mode->pixclock;
	switch (vpara->vic) {
	case HDMI_640x480p60:
		pixelrep = 0;
		aspec = HDMI_4x3;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_480p60:
		pixelrep = 0;
		aspec = HDMI_4x3;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_480p60_16x9:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_720p60:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080i60:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_480i60:
		pixelrep = 1;
		aspec = HDMI_4x3;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_480i60_16x9:
		pixelrep = 1;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_1080p60:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_576p50:
		pixelrep = 0;
		aspec = HDMI_4x3;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_576p50_16x9:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_720p50:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080i50:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_576i50:
		pixelrep = 1;
		aspec = HDMI_4x3;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_576i50_16x9:
		pixelrep = 1;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU601;
		break;
	case HDMI_1080p50:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p24:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p25:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	case HDMI_1080p30:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;

	case HDMI_720p30:
		pixelrep = 0;
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	default:
		aspec = HDMI_16x9;
		colorimetry = HDMI_ITU709;
		break;
	}
	if (colorimetry == HDMI_ITU709)
		binputcolormode |= F_VIDMODE_ITU709;
	else
		binputcolormode &= ~F_VIDMODE_ITU709;
	if (vpara->vic != HDMI_640x480p60)
		binputcolormode |= F_VIDMODE_16_235;
	else
		binputcolormode &= ~F_VIDMODE_16_235;

	if ((hdmi_drv->tmdsclk * (pixelrep + 1)) > 80000000L)
		level = PCLK_HIGH;
	else if ((hdmi_drv->tmdsclk * (pixelrep + 1)) > 20000000L)
		level = PCLK_MEDIUM;
	else
		level = PCLK_LOW;

	if (vpara->input_mode == VIDEO_INPUT_YCBCR422_EMBEDDED_SYNC) {
		sethdmitx_videosignaltype(T_MODE_SYNCEMB | T_MODE_CCIR656);
		sethdmitx_syncembedded_by_vic(vpara->vic,
					      T_MODE_SYNCEMB | T_MODE_CCIR656);
	} else {
		sethdmitx_videosignaltype(0);
		sethdmitx_syncembedded_by_vic(vpara->vic, 0);
	}

	hdmitx_enable_videooutput(level, binputcolormode, boutputcolormode,
				  vpara->output_mode);

	if (vpara->output_mode == OUTPUT_HDMI) {
		cat66121_sys_config_avi(vpara->vic, boutputcolormode, aspec,
					colorimetry, pixelrep);
#ifdef OUTPUT_3D_MODE
		configfhdmivendorspecificinfoframe(OUTPUT_3D_MODE);
#endif

	} else {
		hdmitx_enableaviinfoframe(FALSE, NULL);
		hdmitx_enablevsinfoframe(FALSE, NULL);
	}

	return HDMI_ERROR_SUCCESS;
}

static void cat66121_hdmi_config_aai(void)
{
	int i;
	union audio_infoframe *audioinfo;

	audioinfo = (union audio_infoframe *)communbuff;

	audioinfo->pktbyte.AUD_HB[0] = AUDIO_INFOFRAME_TYPE;
	audioinfo->pktbyte.AUD_HB[1] = 1;
	audioinfo->pktbyte.AUD_HB[2] = AUDIO_INFOFRAME_LEN;
	audioinfo->pktbyte.AUD_DB[0] = 1;
	for (i = 1; i < AUDIO_INFOFRAME_LEN; i++)
		audioinfo->pktbyte.AUD_DB[i] = 0;
	hdmitx_enableaudioinfoframe(TRUE, (unsigned char *)audioinfo);
}

int cat66121_hdmi_sys_config_audio(struct hdmi *hdmi_drv,
				   struct hdmi_audio *audio)
{
	cat66121_hdmi_config_aai();
	hdmitx_enable_audiooutput(
			CNOFIG_INPUT_AUDIO_TYPE,
			CONFIG_INPUT_AUDIO_SPDIF,
			48000L,
			audio->channel,
			NULL, /* pointer to cahnnel status. */
			hdmi_drv->tmdsclk*(pixelrep+1));
	return HDMI_ERROR_SUCCESS;
}

void cat66121_hdmi_sys_enalbe_output(struct hdmi *hdmi_drv, int enable)
{
	hdmi_dbg(hdmi_drv->dev, "[%s]\n", __func__);

	if (enable)
		sethdmitx_avmute(FALSE);
	else
		sethdmitx_avmute(TRUE);
	dumphdmitxreg();
}

int cat66121_hdmi_sys_insert(struct hdmi *hdmi_drv)
{
	hdmi_dbg(hdmi_drv->dev, "[%s]\n", __func__);

	if (!gethdmitx_powerstatus())
		hdmitx_poweron();

	hdmitx_disable_audiooutput();
	return 0;
}

int cat66121_hdmi_sys_remove(struct hdmi *hdmi_drv)
{
	hdmi_dbg(hdmi_drv->dev, "[%s]\n", __func__);
	hdmitx_disable_videooutput();
	if (gethdmitx_powerstatus() == TRUE)
		hdmitx_powerdown();

	return 0;
}
