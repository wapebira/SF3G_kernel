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

#ifdef HDMITX_INPUT_INFO

#define initcec() HDMITX_SETI2C_BYTE(0x0F, 0x08, 0x00)
#define disablecec() HDMITX_SETI2C_BYTE(0x0F, 0x08, 0x08)

LONG calcaudfs(void)
{
	/* LONG RCLK; */
	LONG cnt;
	LONG FS;

	/* RCLK = calcrclk(); */
	SWITCH_HDMITX_BANK(0);
	cnt = (LONG)HDMITX_READI2C_BYTE(0x60);
	FS =  hdmitxdev[0].RCLK / 2;
	FS /= cnt;
	HDMITX_DEBUG_PRINTF1("FS = %ld RCLK = %ld, cnt = %ld\n",
			     FS, hdmitxdev[0].RCLK, cnt);
	return FS;
}

LONG calcpclk(void)
{
	BYTE uc, div;
	int i;
	long sum , count, PCLK;

	SWITCH_HDMITX_BANK(0);
	uc = HDMITX_READI2C_BYTE(0x5F) & 0x80;

	if (!uc)
		return 0;
	/* initcec(); */
	/* // uc = cec_readi2c_byte(0x09) & 0xFE; */
	/* cec_writei2c_byte(0x09, 1); */
	/* delay1ms(100); */
	/* cec_writei2c_byte(0x09, 0); */
	/* RCLK = cec_readi2c_byte(0x47); */
	/* RCLK <<= 8; */
	/* RCLK |= cec_readi2c_byte(0x46); */
	/* RCLK <<= 8; */
	/* RCLK |= cec_readi2c_byte(0x45); */
	/* disablecec(); */
	/* // RCLK *= 160; // RCLK /= 100; */
	/* // RCLK in KHz. */

	HDMITX_SETI2C_BYTE(0xD7, 0xF0, 0x80);
	delay1ms(1);
	HDMITX_SETI2C_BYTE(0xD7, 0x80, 0x00);

	count = HDMITX_READI2C_BYTE(0xD7) & 0xF;
	count <<= 8;
	count |= HDMITX_READI2C_BYTE(0xD8);

	for (div = 7; div > 0; div--) {
		/* printf("div = %d\n", (int)div); */
		if (count < (1 << (11-div)))
			break;
	}
	HDMITX_SETI2C_BYTE(0xD7, 0x70, div << 4);

	uc = HDMITX_READI2C_BYTE(0xD7) & 0x7F;
	for (i = 0 , sum = 0; i < 100; i++) {
		HDMITX_WRITEI2C_BYTE(0xD7, uc|0x80);
		delay1ms(1);
		HDMITX_WRITEI2C_BYTE(0xD7, uc);

		count = HDMITX_READI2C_BYTE(0xD7) & 0xF;
		count <<= 8;
		count |= HDMITX_READI2C_BYTE(0xD8);
		sum += count;
	}
	sum /= 100; count = sum;

	HDMITX_DEBUG_PRINTF1("RCLK(in GetPCLK) = %ld\n", hdmitxdev[0].RCLK);
	HDMITX_DEBUG_PRINTF1("div = %d, count = %d\n", (int)div, (int)count);
	HDMITX_DEBUG_PRINTF1("count = %ld\n", count);

	PCLK = hdmitxdev[0].RCLK * 128 / count * 16;
	PCLK *= (1 << div);

	if (HDMITX_READI2C_BYTE(0x70) & 0x10)
		PCLK /= 2;

	HDMITX_DEBUG_PRINTF1("PCLK = %ld\n", PCLK);
	return PCLK;
}

LONG calcrclk(void)
{
	/* BYTE uc; */
	int i;
	long sum, RCLKCNT;

	initcec();
	sum = 0;
	for (i = 0; i < 5; i++) {
		/* uc = cec_readi2c_byte(0x09) & 0xFE; */
		cec_writei2c_byte(0x09, 1);
		delay1ms(100);
		cec_writei2c_byte(0x09, 0);
		RCLKCNT = cec_readi2c_byte(0x47);
		RCLKCNT <<= 8;
		RCLKCNT |= cec_readi2c_byte(0x46);
		RCLKCNT <<= 8;
		RCLKCNT |= cec_readi2c_byte(0x45);
		/* HDMITX_DEBUG_PRINTF1("RCLK = %ld\n", RCLKCNT); */
		sum += RCLKCNT;
	}
	disablecec();
	RCLKCNT = sum * 32;
	HDMITX_DEBUG_PRINTF("RCLK = %ld, %03ld, %03ld\n",
			    RCLKCNT/1000000, (RCLKCNT%1000000)/1000,
			    RCLKCNT%1000);
	return RCLKCNT;
}

USHORT hdmitx_getinputhtotal(void)
{
	BYTE uc;
	USHORT htotal;

	HDMITX_SETI2C_BYTE(0x0F, 1, 0);
	HDMITX_SETI2C_BYTE(0xA8, 8, 8);

	uc = HDMITX_READI2C_BYTE(0xB2);
	htotal = (uc&1) ? (1 << 12) : 0;
	uc = HDMITX_READI2C_BYTE(0x91);
	htotal |= ((USHORT)uc) << 4;
	uc = HDMITX_READI2C_BYTE(0x90);
	htotal |= (uc&0xF0) >> 4;
	HDMITX_SETI2C_BYTE(0xA8, 8, 0);
	return htotal;
}

USHORT hdmitx_getinputvtotal(void)
{
	BYTE uc;
	USHORT vtotal;

	HDMITX_SETI2C_BYTE(0x0F, 1, 0);
	HDMITX_SETI2C_BYTE(0xA8, 8, 8);

	uc = HDMITX_READI2C_BYTE(0x99);
	vtotal = ((USHORT)uc & 0xF) << 8;
	uc = HDMITX_READI2C_BYTE(0x98);
	vtotal |= uc;
	HDMITX_SETI2C_BYTE(0xA8, 8, 0);

	return vtotal;
}

BOOL hdmitx_isinputinterlace(void)
{
	BYTE uc;

	HDMITX_SETI2C_BYTE(0x0F, 1, 0);
	HDMITX_SETI2C_BYTE(0xA8, 8, 8);

	uc = HDMITX_READI2C_BYTE(0xA5);
	HDMITX_SETI2C_BYTE(0xA8, 8, 0);

	return uc & (1 << 4) ? TRUE : FALSE;
}

BYTE hdmitx_getaudiocount(void)
{
	return HDMITX_READI2C_BYTE(REG_TX_AUD_COUNT);
}
#endif
