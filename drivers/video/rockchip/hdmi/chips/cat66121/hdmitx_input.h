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

#ifndef _HDMITX_DEBUG_H_
#define _HDMITX_DEBUG_H_

#ifdef HDMITX_INPUT_INFO
LONG calcpclk(void);
LONG calcaudfs(void);
LONG calcrclk(void);
BYTE hdmitx_getaudiocount(void);

USHORT hdmitx_getinputhtotal(void);
USHORT hdmitx_getinputvtotal(void);
BOOL hdmitx_isinputinterlace(void);
#endif
#endif
