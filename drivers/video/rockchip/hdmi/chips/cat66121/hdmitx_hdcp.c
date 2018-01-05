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
#include "sha1.h"

static BYTE countbit(BYTE b);

#ifdef SUPPORT_SHA
_XDATA BYTE shabuff[64];
_XDATA BYTE V[20];
_XDATA BYTE ksvlist[32];
_XDATA BYTE vr[20];
_XDATA BYTE M0[8];
#endif

BOOL hdmitx_enablehdcp(BYTE benable)
{
#ifdef SUPPORT_HDCP
	if (benable) {
		if (ER_FAIL == hdmitx_hdcp_authenticate()) {
			HDCP_DEBUG_PRINTF("FAIL hdmitx_hdcp_authenticate\n");
			hdmitx_hdcp_resetauth();
			return FALSE;
		}
		HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate SUCCESS\n");
	} else {
		hdmitxdev[0].bauthenticated = FALSE;
		hdmitx_hdcp_resetauth();
	}
#endif
	return TRUE;
}

#ifdef SUPPORT_HDCP

BOOL gethdmitx_authenticationdone(void)
{
	return hdmitxdev[0].bauthenticated;
}

/* Authentication */

void hdmitx_hdcp_clearauthinterrupt(void)
{
	HDMITX_SETI2C_BYTE(REG_TX_INT_MASK2, B_TX_KSVLISTCHK_MASK |
			   B_TX_AUTH_DONE_MASK | B_TX_AUTH_FAIL_MASK, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR0, B_TX_CLR_AUTH_FAIL |
			     B_TX_CLR_AUTH_DONE | B_TX_CLR_KSVLISTCHK);
	HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR1, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS, B_TX_INTACTDONE);
}

void hdmitx_hdcp_resetauth(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_LISTCTRL, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_HDCP_DESIRE, 0);
	HDMITX_ORREG_BYTE(REG_TX_SW_RST, B_TX_HDCP_RST_HDMITX);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	hdmitx_hdcp_clearauthinterrupt();
	hdmitx_abortddc();
}

/* Function: hdmitx_hdcp_auth_fire() */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: write anything to reg21 to enable HDCP authentication by HW */
/* Side-Effect: N/A */

void hdmitx_hdcp_auth_fire(void)
{
	/* HDCP_DEBUG_PRINTF("hdmitx_hdcp_auth_fire():\n"); */
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHDCP);
	HDMITX_WRITEI2C_BYTE(REG_TX_AUTHFIRE, 1);
}

/* Function: hdmitx_hdcp_startancipher */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: Start the Cipher to free run for random number. When stop, An is */
/* ready in reg30. */
/* Side-Effect: N/A */

void hdmitx_hdcp_startancipher(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_AN_GENERATE, B_TX_START_CIPHER_GEN);
	delay1ms(1); /* delay 1 ms */
}

/* Function: hdmitx_hdcp_stopancipher */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: Stop the Cipher, and An is ready in reg30. */
/* Side-Effect: N/A */

void hdmitx_hdcp_stopancipher(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_AN_GENERATE, B_TX_STOP_CIPHER_GEN);
}

/* Function: hdmitx_hdcp_generatean */
/* Parameter: N/A */
/* Return: N/A */
/* Remark: start An ciper random run at first, then stop it. Software can get */
/* an in reg30~reg38, the write to reg28~2F */
/* Side-Effect: */

void hdmitx_hdcp_generatean(void)
{
	BYTE Data[8];
	BYTE i = 0;

	hdmitx_hdcp_startancipher();
	/* HDMITX_WRITEI2C_BYTE(REG_TX_AN_GENERATE, B_TX_START_CIPHER_GEN); */
	/* delay1ms(1); // delay 1 ms */
	/* HDMITX_WRITEI2C_BYTE(REG_TX_AN_GENERATE, B_TX_STOP_CIPHER_GEN); */

	hdmitx_hdcp_stopancipher();

	SWITCH_HDMITX_BANK(0);
	/* new An is ready in reg30 */
	HDMITX_READI2C_BYTEN(REG_TX_AN_GEN, Data, 8);
	for (i = 0; i < 8; i++)
		HDMITX_WRITEI2C_BYTE(REG_TX_AN+i, Data[i]);
	/* HDMITX_WRITEI2C_BYTEN(REG_TX_AN, Data, 8); */
}

/* Function: hdmitx_hdcp_getbcaps */
/* Parameter: pbcaps - pointer of byte to get bcaps. */
/* pbstatus - pointer of two bytes to get bstatus */
/* Return: ER_SUCCESS if successfully got bcaps and bstatus. */
/* Remark: get B status and capability from HDCP receiver via DDC bus. */
/* Side-Effect: */

enum SYS_STATUS hdmitx_hdcp_getbcaps(PBYTE pbcaps , PUSHORT pbstatus)
{
	BYTE ucdata;
	BYTE timeout;

	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_HEADER, DDC_HDCP_ADDRESS);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQOFF, 0x40); /* bcaps offset */
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQCOUNT, 3);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (timeout = 200; timeout > 0; timeout--) {
		delay1ms(1);

		ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);

		if (ucdata & B_TX_DDC_DONE)
			break;
		if (ucdata & B_TX_DDC_ERROR)
			return ER_FAIL;
	}
	if (timeout == 0)
		return ER_FAIL;
	ucdata = HDMITX_READI2C_BYTE(REG_TX_BSTAT+1);

	*pbstatus = (USHORT)ucdata;
	*pbstatus < < = 8;
	ucdata = HDMITX_READI2C_BYTE(REG_TX_BSTAT);
	*pbstatus |= ((USHORT)ucdata&0xFF);
	*pbcaps = HDMITX_READI2C_BYTE(REG_TX_BCAP);

	return ER_SUCCESS;
}

/* Function: hdmitx_hdcp_getbksv */
/* Parameter: pbksv - pointer of 5 bytes buffer for getting BKSV */
/* Return: ER_SUCCESS if successfuly got BKSV from Rx. */
/* Remark: Get BKSV from HDCP receiver. */
/* Side-Effect: N/A */

enum SYS_STATUS hdmitx_hdcp_getbksv(BYTE *pbksv)
{
	BYTE ucdata;
	BYTE timeout;

	SWITCH_HDMITX_BANK(0);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_HEADER, DDC_HDCP_ADDRESS);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQOFF, 0x00); /* BKSV offset */
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQCOUNT, 5);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (timeout = 200; timeout > 0; timeout--) {
		delay1ms(1);

		ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);
		if (ucdata & B_TX_DDC_DONE) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getbcaps():");
			HDCP_DEBUG_PRINTF("DDC Done.\n");
			break;
		}
		if (ucdata & B_TX_DDC_ERROR) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getbcaps():");
			HDCP_DEBUG_PRINTF("DDC No ack or arbilose %x,"m ucdata);
			HDCP_DEBUG_PRINTF("maybe cable did not connected.\n");
			return ER_FAIL;
		}
	}
	if (timeout == 0)
		return ER_FAIL;
	HDMITX_READI2C_BYTEN(REG_TX_BKSV, (PBYTE)pbksv, 5);

	return ER_SUCCESS;
}

/* Function:hdmitx_hdcp_authenticate
 * Parameter: N/A
 * Return: ER_SUCCESS if Authenticated without error.
 * Remark: do Authentication with Rx
 * Side-Effect:
 * 1. hdmitxdev[0].bauthenticated global variable will
 *    be TRUE when authenticated.
 * 2. Auth_done interrupt and AUTH_FAIL interrupt will be enabled.
 */

static BYTE countbit(BYTE b)
{
	BYTE i, count;

	for (i = 0, count = 0; i < 8; i++) {
		if (b & (1 << i))
			count++;
	}

	return count;
}

void hdmitx_hdcp_reset(void)
{
	BYTE uc;

	uc = HDMITX_READI2C_BYTE(REG_TX_SW_RST) | B_TX_HDCP_RST_HDMITX;
	HDMITX_WRITEI2C_BYTE(REG_TX_SW_RST, uc);
	HDMITX_WRITEI2C_BYTE(REG_TX_HDCP_DESIRE, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_LISTCTRL, 0);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL, B_TX_MASTERHOST);
	hdmitx_clearddcfifo();
	hdmitx_abortddc();
}

enum SYS_STATUS hdmitx_hdcp_authenticate(void)
{
	BYTE ucdata;
	BYTE bcaps;
	USHORT bstatus;
	USHORT timeout;

	/* BYTE revoked = FALSE; */
	BYTE BKSV[5];

	hdmitxdev[0].bauthenticated = FALSE;
	if (0 == (B_TXVIDSTABLE&HDMITX_READI2C_BYTE(REG_TX_SYS_STATUS))) {
		HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate():");
		HDCP_DEBUG_PRINTF("Video not stable\n");
		return ER_FAIL;
	}
	/* Authenticate should be called after AFE setup up. */

	HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate():\n");
	hdmitx_hdcp_reset();

	SWITCH_HDMITX_BANK(0);

	for (timeout = 0; timeout < 80; timeout++) {
		delay1ms(15);

		if (hdmitx_hdcp_getbcaps(&bcaps, &bstatus) != ER_SUCCESS) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getbcaps fail.\n");
			return ER_FAIL;
		}

		if (B_TX_HDMI_MODE ==
		    (HDMITX_READI2C_BYTE(REG_TX_HDMI_MODE) & B_TX_HDMI_MODE)) {
			if ((bstatus & B_TX_CAP_HDMI_MODE) ==
			    B_TX_CAP_HDMI_MODE)
				break;
		} else {
			if ((bstatus & B_TX_CAP_HDMI_MODE) !=
			    B_TX_CAP_HDMI_MODE)
				break;
		}
	}
	HDCP_DEBUG_PRINTF("BCAPS = %02X BSTATUS = %04X\n", (int)bcaps, bstatus);
	hdmitx_hdcp_getbksv(BKSV);
	HDCP_DEBUG_PRINTF("BKSV %02X %02X %02X %02X %02X\n",
			  (int)BKSV[0], (int)BKSV[1], (int)BKSV[2],
			  (int)BKSV[3], (int)BKSV[4]);

	for (timeout = 0, ucdata = 0; timeout < 5; timeout++)
		ucdata += countbit(BKSV[timeout]);
	if (ucdata != 20) {
		HDCP_DEBUG_PRINTF("countbit error\n");
		return ER_FAIL;
	}
	SWITCH_HDMITX_BANK(0);

	HDMITX_ANDREG_BYTE(REG_TX_SW_RST, ~(B_TX_HDCP_RST_HDMITX));

	HDMITX_WRITEI2C_BYTE(REG_TX_HDCP_DESIRE, B_TX_CPDESIRE);
	hdmitx_hdcp_clearauthinterrupt();

	hdmitx_hdcp_generatean();
	HDMITX_WRITEI2C_BYTE(REG_TX_LISTCTRL, 0);
	hdmitxdev[0].bauthenticated = FALSE;

	hdmitx_clearddcfifo();

	if ((bcaps & B_TX_CAP_HDMI_REPEATER) == 0) {
		hdmitx_hdcp_auth_fire();
		/* wait for status; */

		for (timeout = 250; timeout > 0; timeout--) {
			delay1ms(5); /* delay 1ms */
			ucdata = HDMITX_READI2C_BYTE(REG_TX_AUTH_STAT);

			if (ucdata & B_TX_AUTH_DONE) {
				hdmitxdev[0].bauthenticated = TRUE;
				HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate:")
				HDCP_DEBUG_PRINTF(" Authenticate SUCCESS\n");
				break;
			}
			ucdata = HDMITX_READI2C_BYTE(REG_TX_INT_STAT2);
			if (ucdata & B_TX_INT_AUTH_FAIL) {
				HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR0,
						     B_TX_CLR_AUTH_FAIL);
				HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR1, 0);
				HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS,
						     B_TX_INTACTDONE);
				HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS, 0);

				HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate:")
				HDCP_DEBUG_PRINTF(" Authenticate fail\n");
				hdmitxdev[0].bauthenticated = FALSE;
				return ER_FAIL;
			}
		}
		if (timeout == 0) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate:")
			HDCP_DEBUG_PRINTF(" Authenticate Time out\n");
			hdmitxdev[0].bauthenticated = FALSE;
			return ER_FAIL;
		}
		return ER_SUCCESS;
	}

	return hdmitx_hdcp_authenticate_repeater();
}

/* Function: hdmitx_hdcp_verifyintegration */
/* Parameter: N/A */
/* Return: ER_SUCCESS if success, if AUTH_FAIL interrupt status, return fail. */
/* Remark: no used now. */
/* Side-Effect: */

enum SYS_STATUS hdmitx_hdcp_verifyintegration(void)
{
	if (HDMITX_READI2C_BYTE(REG_TX_INT_STAT1) & B_TX_INT_AUTH_FAIL) {
		hdmitx_hdcp_clearauthinterrupt();
		hdmitxdev[0].bauthenticated = FALSE;
		return ER_FAIL;
	}
	if (hdmitxdev[0].bauthenticated)
		return ER_SUCCESS;

	return ER_FAIL;
}

/* Function: hdmitx_hdcp_authenticate_repeater */
/* Parameter: bcaps and bstatus */
/* Return: ER_SUCCESS if success, if AUTH_FAIL interrupt status, return fail. */
/* Remark: */
/* Side-Effect: as Authentication */

void hdmitx_hdcp_cancelrepeaterauthenticate(void)
{
	HDCP_DEBUG_PRINTF("hdmitx_hdcp_cancelrepeaterauthenticate");
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL,
			     B_TX_MASTERDDC | B_TX_MASTERHOST);
	hdmitx_abortddc();
	HDMITX_WRITEI2C_BYTE(REG_TX_LISTCTRL, B_TX_LISTFAIL | B_TX_LISTDONE);
	hdmitx_hdcp_clearauthinterrupt();
}

void hdmitx_hdcp_resumerepeaterauthenticate(void)
{
	HDMITX_WRITEI2C_BYTE(REG_TX_LISTCTRL, B_TX_LISTDONE);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL, B_TX_MASTERHDCP);
}

#ifdef SUPPORT_SHA
enum SYS_STATUS hdmitx_hdcp_checksha(BYTE pm0[], USHORT bstatus,
				     BYTE pksvlist[], int cdownstream,
				     BYTE vr[])
{
	int i, n;

	for (i = 0; i < cdownstream*5; i++)
		shabuff[i] = pksvlist[i];
	shabuff[i++] = bstatus & 0xFF;
	shabuff[i++] = (bstatus>>8) & 0xFF;
	for (n = 0; n < 8; n++, i++)
		shabuff[i] = pm0[n];
	n = i;
	/* shabuff[i++] = 0x80; // end mask */
	for (; i < 64; i++)
		shabuff[i] = 0;
	/* n = cdownstream * 5 + 2 /* for bstatus */ + 8 /* for M0 */; */
		/* n *= 8; */
		/* shabuff[62] = (n>>8) & 0xff; */
		/* shabuff[63] = (n>>8) & 0xff; */
		/*
		   for (i = 0; i < 64; i++) {
		   if (i % 16 == 0)
		   HDCP_DEBUG_PRINTF("SHA[]: ");
		   HDCP_DEBUG_PRINTF(" %02X", shabuff[i]);
		   if ((i%16) = =15)
		   HDCP_DEBUG_PRINTF("\n");
		   }
		   */
		sha_simple(shabuff, n, V);
	for (i = 0; i < 20; i++) {
		if (V[i] != vr[i]) {
			HDCP_DEBUG_PRINTF("V[] =");
			for (i = 0; i < 20; i++)
				HDCP_DEBUG_PRINTF(" %02X", (int)V[i]);
			HDCP_DEBUG_PRINTF("\nvr[] =");
			for (i = 0; i < 20; i++)
				HDCP_DEBUG_PRINTF(" %02X", (int)vr[i]);
			return ER_FAIL;
		}
	}

	return ER_SUCCESS;
}

#endif /* SUPPORT_SHA */

enum SYS_STATUS hdmitx_hdcp_getksvlist(BYTE *pksvlist, BYTE cdownstream)
{
	BYTE timeout = 100;
	BYTE ucdata;

	if (cdownstream == 0)
		return ER_SUCCESS;
	if (/* cdownstream == 0 || */ pksvlist == NULL)
		return ER_FAIL;
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL, B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_HEADER, 0x74);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQOFF, 0x43);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQCOUNT, cdownstream * 5);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (timeout = 200; timeout > 0; timeout--) {
		ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);
		if (ucdata & B_TX_DDC_DONE) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getksvlist():");
			HDCP_DEBUG_PRINTF("DDC Done.\n");
			break;
		}
		if (ucdata & B_TX_DDC_ERROR) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getksvlist():");
			HDCP_DEBUG_PRINTF("DDC Fail DDC_STATUS = %x.\n",
					  ucdata);
			return ER_FAIL;
		}
		delay1ms(5);
	}
	if (timeout == 0)
		return ER_FAIL;
	HDCP_DEBUG_PRINTF("hdmitx_hdcp_getksvlist(): KSV");
	for (timeout = 0; timeout < cdownstream * 5; timeout++) {
		pksvlist[timeout] = HDMITX_READI2C_BYTE(REG_TX_DDC_READFIFO);
		HDCP_DEBUG_PRINTF(" %02X", (int)pksvlist[timeout]);
	}
	HDCP_DEBUG_PRINTF("\n");
	return ER_SUCCESS;
}

enum SYS_STATUS hdmitx_hdcp_getvr(BYTE *pvr)
{
	BYTE timeout;
	BYTE ucdata;

	if (pvr == NULL)
		return ER_FAIL;
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_MASTER_CTRL, B_TX_MASTERHOST);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_HEADER, 0x74);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQOFF, 0x20);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_REQCOUNT, 20);
	HDMITX_WRITEI2C_BYTE(REG_TX_DDC_CMD, CMD_DDC_SEQ_BURSTREAD);

	for (timeout = 200; timeout > 0; timeout--) {
		ucdata = HDMITX_READI2C_BYTE(REG_TX_DDC_STATUS);
		if (ucdata & B_TX_DDC_DONE) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getvr(): DDC Done.\n");
			break;
		}
		if (ucdata & B_TX_DDC_ERROR) {
			HDCP_DEBUG_PRINTF("hdmitx_hdcp_getvr(): ");
			HDCP_DEBUG_PRINTF("DDC fail DDC_STATUS = %x.\n",
					  (int)ucdata);
			return ER_FAIL;
		}
		delay1ms(5);
	}
	if (timeout == 0) {
		HDCP_DEBUG_PRINTF("hdmitx_hdcp_getvr(): DDC timeout.\n");
		return ER_FAIL;
	}
	SWITCH_HDMITX_BANK(0);

	for (timeout = 0; timeout < 5; timeout++) {
		HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL , timeout);
		pvr[timeout * 4]  =
			(ULONG)HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE1);
		pvr[timeout * 4 + 1] =
			(ULONG)HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE2);
		pvr[timeout * 4 + 2] =
			(ULONG)HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE3);
		pvr[timeout * 4 + 3] =
			(ULONG)HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE4);
	}

	return ER_SUCCESS;
}

enum SYS_STATUS hdmitx_hdcp_getm0(BYTE *pm0)
{
	int i;

	if (!pm0)
		return ER_FAIL;
	HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL, 5);
	pm0[0] = HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE1);
	pm0[1] = HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE2);
	pm0[2] = HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE3);
	pm0[3] = HDMITX_READI2C_BYTE(REG_TX_SHA_RD_BYTE4);
	HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL, 0);
	pm0[4] = HDMITX_READI2C_BYTE(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL, 1);
	pm0[5] = HDMITX_READI2C_BYTE(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL, 2);
	pm0[6] = HDMITX_READI2C_BYTE(REG_TX_AKSV_RD_BYTE5);
	HDMITX_WRITEI2C_BYTE(REG_TX_SHA_SEL, 3);
	pm0[7] = HDMITX_READI2C_BYTE(REG_TX_AKSV_RD_BYTE5);

	HDCP_DEBUG_PRINTF("M[] =");
	for (i = 0; i < 8; i++)
		HDCP_DEBUG_PRINTF("0x%02x, ", (int)pm0[i]);
	HDCP_DEBUG_PRINTF("\n");

	return ER_SUCCESS;
}

enum SYS_STATUS hdmitx_hdcp_authenticate_repeater(void)
{
	BYTE uc , ii;
	/* BYTE revoked; */
	/* int i; */
	BYTE cdownstream;

	BYTE bcaps;
	USHORT bstatus;
	USHORT timeout;

	HDCP_DEBUG_PRINTF("Authentication for repeater\n");
	/* emily add for test, abort HDCP */
	/* 2007/10/01 marked by jj_tseng@chipadvanced.com */
	/* HDMITX_WRITEI2C_BYTE(0x20, 0x00); */
	/* HDMITX_WRITEI2C_BYTE(0x04, 0x01); */
	/* HDMITX_WRITEI2C_BYTE(0x10, 0x01); */
	/* HDMITX_WRITEI2C_BYTE(0x15, 0x0F); */
	/* delay1ms(100); */
	/* HDMITX_WRITEI2C_BYTE(0x04, 0x00); */
	/* HDMITX_WRITEI2C_BYTE(0x10, 0x00); */
	/* HDMITX_WRITEI2C_BYTE(0x20, 0x01); */
	/* delay1ms(100); */
	/* test07 = HDMITX_READI2C_BYTE(0x7); */
	/* test06 = HDMITX_READI2C_BYTE(0x6); */
	/* test08 = HDMITX_READI2C_BYTE(0x8); */
	/* ~jj_tseng@chipadvanced.com */
	/* end emily add for test */

	/* Authenticate Fired */

	hdmitx_hdcp_getbcaps(&bcaps, &bstatus);
	delay1ms(2);
	if ((B_TX_INT_HPD_PLUG | B_TX_INT_RX_SENSE) &
	    HDMITX_READI2C_BYTE(REG_TX_INT_STAT1)) {
		HDCP_DEBUG_PRINTF("HPD Before Fire Auth\n");
		goto hdmitx_hdcp_repeater_fail;
	}
	hdmitx_hdcp_auth_fire();
	/* delay1ms(550); // emily add for test */
	for (ii = 0; ii < 55; ii++) {
		if ((B_TX_INT_HPD_PLUG | B_TX_INT_RX_SENSE) &
		    HDMITX_READI2C_BYTE(REG_TX_INT_STAT1))
			goto hdmitx_hdcp_repeater_fail;
		delay1ms(10);
	}
	for (timeout = /*250*6*/10; timeout > 0; timeout--) {
		HDCP_DEBUG_PRINTF("timeout = %d wait part 1\n", timeout);
		if ((B_TX_INT_HPD_PLUG | B_TX_INT_RX_SENSE) &
		    HDMITX_READI2C_BYTE(REG_TX_INT_STAT1)) {
			HDCP_DEBUG_PRINTF("HPD at wait part 1\n");
			goto hdmitx_hdcp_repeater_fail;
		}
		uc = HDMITX_READI2C_BYTE(REG_TX_INT_STAT1);
		if (uc & B_TX_INT_DDC_BUS_HANG) {
			HDCP_DEBUG_PRINTF("DDC Bus hang\n");
			goto hdmitx_hdcp_repeater_fail;
		}
		uc = HDMITX_READI2C_BYTE(REG_TX_INT_STAT2);

		if (uc & B_TX_INT_AUTH_FAIL) {
			HDCP_DEBUG_PRINTF("hdcp_authenticaterepeater: ");
			HDCP_DEBUG_PRINTF("B_TX_INT_AUTH_FAIL.\n");
			goto hdmitx_hdcp_repeater_fail;
		}
		/* emily add for test */
		/* test =(HDMITX_READI2C_BYTE(0x7)&0x4)>>2; */
		if (uc & B_TX_INT_KSVLIST_CHK) {
			HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR0,
					     B_TX_CLR_KSVLISTCHK);
			HDMITX_WRITEI2C_BYTE(REG_TX_INT_CLR1, 0);
			HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS,
					     B_TX_INTACTDONE);
			HDMITX_WRITEI2C_BYTE(REG_TX_SYS_STATUS, 0);
			HDCP_DEBUG_PRINTF("B_TX_INT_KSVLIST_CHK\n");
			break;
		}
		delay1ms(5);
	}
	if (timeout == 0)
		goto hdmitx_hdcp_repeater_fail;

	/* clear ksvlist check interrupt. */

	for (timeout = 500; timeout > 0; timeout--) {
		HDCP_DEBUG_PRINTF("timeout = %d at wait FIFO ready\n", timeout);
		if ((B_TX_INT_HPD_PLUG | B_TX_INT_RX_SENSE) &
		    HDMITX_READI2C_BYTE(REG_TX_INT_STAT1)) {
			HDCP_DEBUG_PRINTF("HPD at wait FIFO ready\n");
			goto hdmitx_hdcp_repeater_fail;
		}
		if (hdmitx_hdcp_getbcaps(&bcaps, &bstatus) == ER_FAIL) {
			HDCP_DEBUG_PRINTF("Get bcaps fail\n");
			goto hdmitx_hdcp_repeater_fail;
		}
		if (bcaps & B_TX_CAP_KSV_FIFO_RDY) {
			HDCP_DEBUG_PRINTF("FIFO Ready\n");
			break;
		}
		delay1ms(5);
	}
	if (timeout == 0) {
		HDCP_DEBUG_PRINTF("Get KSV FIFO ready timeout\n");
		goto hdmitx_hdcp_repeater_fail;
	}
	HDCP_DEBUG_PRINTF("Wait timeout = %d\n", timeout);

	hdmitx_clearddcfifo();
	hdmitx_generateddcsclk();
	cdownstream =  (bstatus & M_TX_DOWNSTREAM_COUNT);

	if (/*cdownstream == 0 | | */ cdownstream > 6 ||
	    bstatus & (B_TX_MAX_CASCADE_EXCEEDED | B_TX_DOWNSTREAM_OVER)) {
		HDCP_DEBUG_PRINTF("Invalid Down stream count, fail\n");
		goto hdmitx_hdcp_repeater_fail;
	}
#ifdef SUPPORT_SHA
	if (hdmitx_hdcp_getksvlist(ksvlist, cdownstream) == ER_FAIL)
		goto hdmitx_hdcp_repeater_fail;

	if (hdmitx_hdcp_getvr(vr) == ER_FAIL)
		goto hdmitx_hdcp_repeater_fail;
	if (hdmitx_hdcp_getm0(M0) == ER_FAIL)
		goto hdmitx_hdcp_repeater_fail;
	/* do check SHA */
	if (hdmitx_hdcp_checksha(M0, bstatus,
				 ksvlist, cdownstream, vr) == ER_FAIL)
		goto hdmitx_hdcp_repeater_fail;
	if ((B_TX_INT_HPD_PLUG | B_TX_INT_RX_SENSE) &
	    HDMITX_READI2C_BYTE(REG_TX_INT_STAT1)) {
		HDCP_DEBUG_PRINTF("HPD at Final\n");
		goto hdmitx_hdcp_repeater_fail;
	}
#endif /* SUPPORT_SHA */

	HDCP_DEBUG_PRINTF("hdmitx_hdcp_authenticate()-receiver:");
	HDCP_DEBUG_PRINTF("Authenticate SUCCESS\n");
	hdmitx_hdcp_resumerepeaterauthenticate();
	hdmitxdev[0].bauthenticated = TRUE;
	return ER_SUCCESS;

hdmitx_hdcp_repeater_fail:
	hdmitx_hdcp_cancelrepeaterauthenticate();
	return ER_FAIL;
}

/*
 * Function: hdmitx_hdcp_resumeauthentication
 * Parameter: N/A
 * Return: N/A
 * Remark: called by interrupt handler to restart Authentication and Encryption.
 * Side-Effect: as Authentication and Encryption.
 */

void hdmitx_hdcp_resumeauthentication(void)
{
	sethdmitx_avmute(TRUE);
	sethdmitx_avmute(FALSE);
}

#endif /* SUPPORT_HDCP */
