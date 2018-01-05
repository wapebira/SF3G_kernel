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

#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_

/* data type */

#ifdef _MCU_8051_
	#define _CODE code
	#define _DATA data
	#define _XDATA xdata
	#define _IDATA idata
	#define BOOL bit
#else
	#define _CODE /* const */
	#define _DATA
	#define _IDATA
	#define _XDATA
	#define BOOL int
#endif /* _MCU_8051_ */

#define cbyte	_CODE unsigned char

/*
typedef unsigned char UCHAR, *PUCHAR;
typedef unsigned char byte, *pbyte;
typedef unsigned char BYTE, *PBYTE;

typedef short SHORT, *PSHORT;
typedef unsigned short *pushort;
typedef unsigned short USHORT, *PUSHORT;
typedef unsigned short word, *pword;
typedef unsigned short WORD, *PWORD;
typedef unsigned int UINT, *PUINT;

typedef long LONG, *PLONG;
typedef unsigned long *pulong;
typedef unsigned long ULONG, *PULONG;
typedef unsigned long dword, *pdword;
typedef unsigned long DWORD, *PDWORD;
*/
#define BYTE unsigned char
#define PBYTE unsigned char*
#define SHORT unsigned short
#define PUSHORT unsigned short*
#define WORD unsigned short
#define ULONG unsigned long
#define USHORT unsigned long

#define FALSE 0
#define TRUE 1

#define SUCCESS 0
#define FAIL -1

#define ON 1
#define OFF 0

#define LO_ACTIVE TRUE
#define HI_ACTIVE FALSE

enum SYS_STATUS {
	ER_SUCCESS = 0,
	ER_FAIL,
	ER_RESERVED
};

#define ABS(x) (((x) >= 0) ? (x) : (-(x)))

/* Video Data type */

#define F_MODE_RGB444  0
#define F_MODE_YUV422 1
#define F_MODE_YUV444 2
#define F_MODE_CLRMOD_MASK 3

#define F_MODE_INTERLACE  1

#define F_VIDMODE_ITU709  (1 << 4)
#define F_VIDMODE_ITU601  0

#define F_VIDMODE_0_255   0
#define F_VIDMODE_16_235  (1 << 5)

#define F_VIDMODE_EN_UDFILT (1 << 6)
#define F_VIDMODE_EN_DITHER (1 << 7)

#define T_MODE_CCIR656 (1 << 0)
#define T_MODE_SYNCEMB (1 << 1)
#define T_MODE_INDDR   (1 << 2)
#define T_MODE_PCLKDIV2 (1 << 3)
#define T_MODE_DEGEN (1 << 4)
#define T_MODE_SYNCGEN (1 << 5)

/* Packet and Info Frame definition and datastructure. */

#define VENDORSPEC_INFOFRAME_TYPE 0x81
#define AVI_INFOFRAME_TYPE  0x82
#define SPD_INFOFRAME_TYPE 0x83
#define AUDIO_INFOFRAME_TYPE 0x84
#define MPEG_INFOFRAME_TYPE 0x85

#define VENDORSPEC_INFOFRAME_VER 0x01
#define AVI_INFOFRAME_VER  0x02
#define SPD_INFOFRAME_VER 0x01
#define AUDIO_INFOFRAME_VER 0x01
#define MPEG_INFOFRAME_VER 0x01

#define VENDORSPEC_INFOFRAME_LEN 5
#define AVI_INFOFRAME_LEN 13
#define SPD_INFOFRAME_LEN 25
#define AUDIO_INFOFRAME_LEN 10
#define MPEG_INFOFRAME_LEN 10

#define ACP_PKT_LEN 9
#define ISRC1_PKT_LEN 16
#define ISRC2_PKT_LEN 16

union vendorspecific_infoframe {
	struct {
		BYTE type;
		BYTE ver;
		BYTE len;

		BYTE checksum;

		BYTE IEEE_0;/* PB1 */
		BYTE IEEE_1;/* PB2 */
		BYTE IEEE_2;/* PB3 */

		BYTE rsvd:5;/* PB4 */
		BYTE hdmi_video_format:3;

		BYTE reserved_PB5:4;/* PB5 */
		BYTE _3d_structure:4;

		BYTE reserved_PB6:4;/* PB6 */
		BYTE _3d_ext_data:4;
	} info;
	struct {
		BYTE VS_HB[3];
		BYTE checksum;
		BYTE VS_DB[28];
	} pktbyte;
};

union avi_infoframe {
	struct {
		BYTE type;
		BYTE ver;
		BYTE len;

		BYTE checksum;

		BYTE scan:2;
		BYTE barinfo:2;
		BYTE activefmtinfopresent:1;
		BYTE colormode:2;
		BYTE FU1:1;

		BYTE activeformataspectratio:4;
		BYTE pictureaspectratio:2;
		BYTE colorimetry:2;

		BYTE scaling:2;
		BYTE FU2:6;

		BYTE VIC:7;
		BYTE FU3:1;

		BYTE pixelrepetition:4;
		BYTE FU4:4;

		short ln_end_top;
		short ln_start_bottom;
		short pix_end_left;
		short pix_start_right;
	} info;

	struct {
		BYTE AVI_HB[3];
		BYTE checksum;
		BYTE AVI_DB[AVI_INFOFRAME_LEN];
	} pktbyte;
};

union audio_infoframe {
	struct {
		BYTE type;
		BYTE ver;
		BYTE len;
		BYTE checksum;

		BYTE audiochannelcount:3;
		BYTE RSVD1:1;
		BYTE audiocodingtype:4;

		BYTE samplesize:2;
		BYTE samplefreq:3;
		BYTE rsvd2:3;

		BYTE fmtcoding;

		BYTE speakerplacement;

		BYTE rsvd3:3;
		BYTE levelshiftvalue:4;
		BYTE DM_INH:1;
	} info;

	struct {
		BYTE AUD_HB[3];
		BYTE checksum;
		BYTE AUD_DB[5];
	} pktbyte;
};

union mpeg_infoframe {
	struct {
		BYTE type;
		BYTE ver;
		BYTE len;
		BYTE checksum;

		ULONG mpegbitrate;

		BYTE mpegframe:2;
		BYTE rvsd1:2;
		BYTE fieldrepeat:1;
		BYTE rvsd2:3;
	} info;
	struct {
		BYTE MPG_HB[3];
		BYTE checksum;
		BYTE MPG_DB[MPEG_INFOFRAME_LEN];
	} pktbyte;
};

union spd_infoframe {
	struct {
		BYTE type;
		BYTE ver;
		BYTE len;
		BYTE checksum;

		char VN[8];
		char PD[16];
		BYTE sourcedeviceinfomation;
	} info;
	struct {
		BYTE SPD_HB[3];
		BYTE checksum;
		BYTE SPD_DB[SPD_INFOFRAME_LEN];
	} pktbyte;
};

/* Using for interface. */
#define PROG 1
#define INTERLACE 0
#define vneg 0
#define hneg 0
#define vpos 1
#define hpos 1

struct ceavtiming {
	WORD	h_activestart;
	WORD	h_activeend;
	WORD	h_syncstart;
	WORD	h_syncend;
	WORD	v_activestart;
	WORD	v_activeend;
	WORD	v_syncstart;
	WORD	v_syncend;
	WORD	v2_activestart;
	WORD	v2_activeend;
	WORD	htotal;
	WORD	vtotal;
};

struct hdmi_vtiming {
	BYTE VIC;
	BYTE pixelrep;
	WORD	hactive;
	WORD	vactive;
	WORD	htotal;
	WORD	vtotal;
	ULONG	PCLK;
	BYTE	xcnt;
	WORD	hfrontporch;
	WORD	hsyncwidth;
	WORD	hbackporch;
	BYTE	vfrontporch;
	BYTE	vsyncwidth;
	BYTE	vbackporch;
	BYTE	scanmode:1;
	BYTE	vpolarity:1;
	BYTE	hpolarity:1;
};

/* Audio relate definition and macro. */

/* 2008/08/15 added by jj_tseng@chipadvanced */
#define F_AUDIO_ON  (1 << 7)
#define F_AUDIO_HBR (1 << 6)
#define F_AUDIO_DSD (1 << 5)
#define F_AUDIO_NLPCM (1 << 4)
#define F_AUDIO_LAYOUT_1 (1 << 3)
#define F_AUDIO_LAYOUT_0 (0 << 3)

/* HBR - 1100 */
/* DSD - 1010 */
/* NLPCM - 1001 */
/* LPCM - 1000 */

#define T_AUDIO_MASK 0xF0
#define T_AUDIO_OFF 0
#define T_AUDIO_HBR (F_AUDIO_ON|F_AUDIO_HBR)
#define T_AUDIO_DSD (F_AUDIO_ON|F_AUDIO_DSD)
#define T_AUDIO_NLPCM (F_AUDIO_ON|F_AUDIO_NLPCM)
#define T_AUDIO_LPCM (F_AUDIO_ON)

/* for sample clock */
#define audfs_22p05khz  4
#define audfs_44p1khz 0
#define audfs_88p2khz 8
#define audfs_176p4khz	12

#define audfs_24khz  6
#define audfs_48khz  2
#define audfs_96khz  10
#define audfs_192khz 14

#define audfs_768khz 9

#define audfs_32khz  3
#define AUDFS_OTHER	1

/* Audio Enable */
#define ENABLE_SPDIF	(1 << 4)
#define ENABLE_I2S_SRC3  (1 << 3)
#define ENABLE_I2S_SRC2  (1 << 2)
#define ENABLE_I2S_SRC1  (1 << 1)
#define ENABLE_I2S_SRC0  (1 << 0)

#define AUD_SWL_NOINDICATE  0x0
#define AUD_SWL_16		  0x2
#define AUD_SWL_17		  0xC
#define AUD_SWL_18		  0x4
#define AUD_SWL_20		  0xA /* for maximum 20 bit */
#define AUD_SWL_21		  0xD
#define AUD_SWL_22		  0x5
#define AUD_SWL_23		  0x9
#define AUD_SWL_24		  0xB

#endif /* _TYPEDEF_H_ */
