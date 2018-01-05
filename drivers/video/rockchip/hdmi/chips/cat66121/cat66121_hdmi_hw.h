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

#ifndef _CAT6611_HDMI_HW_H
#define _CAT6611_HDMI_HW_H

#include "typedef.h"
#include "config.h"
#include "debug.h"
#include "hdmitx_drv.h"
#define CAT6611_SCL_RATE	(100 * 1000)
#define I2S 0
#define SPDIF 1

#ifndef I2S_FORMAT
#define I2S_FORMAT 0x01 /* 32bit audio */
#endif

#ifndef INPUT_SAMPLE_FREQ
 #define INPUT_SAMPLE_FREQ audfs_48khz
#endif /* INPUT_SAMPLE_FREQ */

#ifndef INPUT_SAMPLE_FREQ_HZ
 #define INPUT_SAMPLE_FREQ_HZ 44100L
#endif /* INPUT_SAMPLE_FREQ_HZ */

#ifndef OUTPUT_CHANNEL
 #define OUTPUT_CHANNEL 2
#endif /* OUTPUT_CHANNEL */

#ifndef CNOFIG_INPUT_AUDIO_TYPE
 #define CNOFIG_INPUT_AUDIO_TYPE T_AUDIO_LPCM
/* #define CNOFIG_INPUT_AUDIO_TYPE T_AUDIO_NLPCM */
/* #define CNOFIG_INPUT_AUDIO_TYPE T_AUDIO_HBR */
#endif /* CNOFIG_INPUT_AUDIO_TYPE */

#ifndef CONFIG_INPUT_AUDIO_SPDIF
 #define CONFIG_INPUT_AUDIO_SPDIF I2S
/* #define CONFIG_INPUT_AUDIO_SPDIF  SPDIF */
#endif /* CONFIG_INPUT_AUDIO_SPDIF */

#ifndef INPUT_SIGNAL_TYPE
#define INPUT_SIGNAL_TYPE 0 /* 24 bit sync separate */
#endif

/* Internal Data type */

enum {
	OUTPUT_DVI = 0,
	OUTPUT_HDMI
};

enum  hdmi_video_type {
	hdmi_unknown = 0,
	HDMI_640x480p60 = 1,
	HDMI_480p60,
	HDMI_480p60_16x9,
	HDMI_720p60,
	HDMI_1080i60,
	HDMI_480i60,
	HDMI_480i60_16x9,
	HDMI_1080p60 = 16,
	HDMI_576p50,
	HDMI_576p50_16x9,
	HDMI_720p50,
	HDMI_1080i50,
	HDMI_576i50,
	HDMI_576i50_16x9,
	HDMI_1080p50 = 31,
	HDMI_1080p24,
	HDMI_1080p25,
	HDMI_1080p30,
	HDMI_720p30 = 61,
};

enum hdmi_aspec {
	HDMI_4x3,
	HDMI_16x9
};

enum hdmi_outputcolormode {
	HDMI_RGB444,
	HDMI_YUV444,
	HDMI_YUV422
};

enum hdmi_colorimetry {
	HDMI_ITU601,
	HDMI_ITU709
};

struct videotiming {
	ULONG videopixelclock;
	BYTE VIC;
	BYTE pixelrep;
	BYTE outputvideomode;
};

enum txvideo_state_type {
	txvstate_unplug = 0,
	TXVSTATE_HPD,
	txvstate_waitformode,
	txvstate_waitforvstable,
	txvstate_videoinit,
	txvstate_videosetup,
	txvstate_videoon,
	txvstate_reserved
};

enum txaudio_state_type {
	txastate_audiooff = 0,
	txastate_audioprepare,
	txastate_audioon,
	txastate_audiofifofail,
	txastate_reserved
};

/* RX Capability. */

struct lpcm_bitwidth {
	BYTE b16bit:1;
	BYTE b20bit:1;
	BYTE b24bit:1;
	BYTE rsrv:5;
};

enum {
	AUD_RESERVED_0 = 0,
	AUD_LPCM,
	AUD_AC3,
	AUD_MPEG1,
	AUD_MP3,
	AUD_MPEG2,
	AUD_AAC,
	AUD_DTS,
	AUD_ATRAC,
	AUD_ONE_BIT_AUDIO,
	AUD_DOLBY_DIGITAL_PLUS,
	AUD_DTS_HD,
	AUD_MAT_MLP,
	AUD_DST,
	AUD_WMA_PRO,
	AUD_RESERVED_15
};

union auddescriptor {
	struct {
		BYTE channel:3;
		BYTE audioformatcode:4;
		BYTE rsrv1:1;

		BYTE b32khz:1;
		BYTE b44_1khz:1;
		BYTE b48khz:1;
		BYTE b88_2khz:1;
		BYTE b96khz:1;
		BYTE b176_4khz:1;
		BYTE b192khz:1;
		BYTE rsrv2:1;
		BYTE uccode;
	} s;
	BYTE uc[3];
};

union spk_alloc {
	struct {
		BYTE FL_FR:1;
		BYTE LFE:1;
		BYTE FC:1;
		BYTE RL_RR:1;
		BYTE RC:1;
		BYTE FLC_FRC:1;
		BYTE RLC_RRC:1;
		BYTE reserve:1;
		BYTE unuse[2];
	} s;
	BYTE uc[3];
};

#define CEA_SUPPORT_UNDERSCAN (1<<7)
#define CEA_SUPPORT_AUDIO (1<<6)
#define CEA_SUPPORT_YUV444 (1<<5)
#define CEA_SUPPORT_YUV422 (1<<4)
#define CEA_NATIVE_MASK 0xF

#define HDMI_DC_SUPPORT_AI (1<<7)
#define HDMI_DC_SUPPORT_48 (1<<6)
#define HDMI_DC_SUPPORT_36 (1<<5)
#define HDMI_DC_SUPPORT_30 (1<<4)
#define HDMI_DC_SUPPORT_Y444 (1<<3)
#define HDMI_DC_SUPPORT_DVI_DUAL 1

union dcsupport {
	struct {
		BYTE dvi_dual:1;
		BYTE rsvd:2;
		BYTE DC_Y444:1;
		BYTE dc_30bit:1;
		BYTE dc_36bit:1;
		BYTE dc_48bit:1;
		BYTE SUPPORT_AI:1;
	} info;
	BYTE uc;
};

union latency_support {
	struct {
		BYTE rsvd:6;
		BYTE i_latency_present:1;
		BYTE latency_present:1;
	} info;
	BYTE uc;
};

#define HDMI_IEEEOUI 0x0c03
#define MAX_VODMODE_COUNT 32
#define MAX_AUDDES_COUNT 4

struct rx_cap {
	BYTE videomode;
	BYTE nativevdomode;
	BYTE vdomode[8];
	BYTE auddescount;
	union auddescriptor auddes[MAX_AUDDES_COUNT];
	BYTE PA[2];
	ULONG IEEEOUI;
	union dcsupport dc;
	BYTE maxtmdsclock;
	union latency_support lsupport;
	union spk_alloc  speakerallocblk;
	BYTE validcea:1;
	BYTE validhdmi:1;
	BYTE valid3d:1;
};

/* Output Mode type */
#define RES_ASPEC_4x3 0
#define RES_ASPEC_16x9 1
#define F_MODE_REPT_NO 0
#define F_MODE_REPT_TWICE 1
#define F_MODE_REPT_QUATRO 3
#define F_MODE_CSC_ITU601 0
#define F_MODE_CSC_ITU709 1

BYTE HDMITX_READI2C_BYTE(BYTE regaddr);
enum SYS_STATUS HDMITX_WRITEI2C_BYTE(BYTE regaddr, BYTE d);
enum SYS_STATUS HDMITX_READI2C_BYTEN(BYTE regaddr, BYTE *pdata, int N);
enum SYS_STATUS HDMITX_WRITEI2C_BYTEN(BYTE regaddr, BYTE *pdata, int N);
enum SYS_STATUS HDMITX_SETI2C_BYTE(BYTE reg, BYTE mask, BYTE value);

void init_hdmitx_variable(void);
/* void HDMITX_ChangeDisplayoption(enum hdmi_video_type videomode,
 * enum hdmi_outputcolormode Outputcolormode);
 */
/* void HDMITX_SetOutput(); */
/* int  HDMITX_DevLoopProc(); */
/* void configfhdmivendorspecificinfoframe(BYTE _3d_stru); */
/* void hdmitx_changeaudiooption(BYTE option, BYTE channelnum, BYTE audiofs); */
/* void hdmitx_setaudiooutput(); */
/* void hdmitx_changecolor_depth(BYTE colordepth); */
#endif
