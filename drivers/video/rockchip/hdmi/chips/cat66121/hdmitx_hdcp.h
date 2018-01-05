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

#ifndef _HDMITX_HDCP_H_
#define _HDMITX_HDCP_H_

#define REG_TX_HDCP_DESIRE 0x20
    #define B_TX_ENABLE_HDPC11 (1<<1)
    #define B_TX_CPDESIRE  (1<<0)

#define REG_TX_AUTHFIRE    0x21
#define REG_TX_LISTCTRL    0x22
    #define B_TX_LISTFAIL  (1<<1)
    #define B_TX_LISTDONE  (1<<0)

#define REG_TX_AKSV    0x23
#define REG_TX_AKSV0   0x23
#define REG_TX_AKSV1   0x24
#define REG_TX_AKSV2   0x25
#define REG_TX_AKSV3   0x26
#define REG_TX_AKSV4   0x27

#define REG_TX_AN  0x28
#define REG_TX_AN_GEN  0x30
#define REG_TX_ARI     0x38
#define REG_TX_ARI0    0x38
#define REG_TX_ARI1    0x39
#define REG_TX_APJ     0x3A

#define REG_TX_BKSV    0x3B
#define REG_TX_BRI     0x40
#define REG_TX_BRI0    0x40
#define REG_TX_BRI1    0x41
#define REG_TX_BPJ     0x42
#define REG_TX_BCAP    0x43
    #define B_TX_CAP_HDMI_REPEATER (1<<6)
    #define B_TX_CAP_KSV_FIFO_RDY  (1<<5)
    #define B_TX_CAP_HDMI_FAST_MODE    (1<<4)
    #define B_CAP_HDCP_1p1  (1<<1)
    #define B_TX_CAP_FAST_REAUTH   (1<<0)
#define REG_TX_BSTAT   0x44
#define REG_TX_BSTAT0   0x44
#define REG_TX_BSTAT1   0x45
    #define B_TX_CAP_HDMI_MODE (1<<12)
    #define B_TX_CAP_DVI_MODE (0<<12)
    #define B_TX_MAX_CASCADE_EXCEEDED  (1<<11)
    #define M_TX_REPEATER_DEPTH    (0x7<<8)
    #define O_TX_REPEATER_DEPTH    8
    #define B_TX_DOWNSTREAM_OVER   (1<<7)
    #define M_TX_DOWNSTREAM_COUNT  0x7F

#define REG_TX_AUTH_STAT 0x46
#define B_TX_AUTH_DONE (1<<7)

/* Function Prototype */

BOOL gethdmitx_authenticationdone(void);
void hdmitx_hdcp_clearauthinterrupt(void);
void hdmitx_hdcp_resetauth(void);
void hdmitx_hdcp_auth_fire(void);
void hdmitx_hdcp_startancipher(void);
void hdmitx_hdcp_stopancipher(void);
void hdmitx_hdcp_generatean(void);
enum SYS_STATUS hdmitx_hdcp_getbcaps(PBYTE pbcaps, PUSHORT pbstatus);
enum SYS_STATUS hdmitx_hdcp_getbksv(BYTE *pbksv);

void hdmitx_hdcp_reset(void);
enum SYS_STATUS hdmitx_hdcp_authenticate(void);
enum SYS_STATUS hdmitx_hdcp_verifyintegration(void);
void hdmitx_hdcp_cancelrepeaterauthenticate(void);
void hdmitx_hdcp_resumerepeaterauthenticate(void);
enum SYS_STATUS hdmitx_hdcp_checksha(BYTE pm0[], USHORT bstatus,
				     BYTE pksvlist[],
				     int cdownstream, BYTE vr[]);
enum SYS_STATUS hdmitx_hdcp_getksvlist(BYTE *pksvlist, BYTE cdownstream);
enum SYS_STATUS hdmitx_hdcp_getvr(BYTE *pvr);
enum SYS_STATUS hdmitx_hdcp_getm0(BYTE *pm0);
enum SYS_STATUS hdmitx_hdcp_authenticate_repeater(void);
void hdmitx_hdcp_resumeauthentication(void);
#endif /* _HDMITX_HDCP_H_ */
