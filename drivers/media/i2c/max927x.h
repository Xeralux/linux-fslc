#ifndef max927x_h__
#define max927x_h__
/*
 * Maxim 9721 seralizer/9272 deserializer device definitions
 *
 * Copyright (c) 2013-2014 Leopard Imaging, Inc.
 * Copyright (c) 2015 Sensity Systems, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/types.h>

struct max927x_setting {
	const char *name;
	u8 reg;
	unsigned pos;
	unsigned width;
	unsigned idx;
};

#define MAX9271_SS_NONE 0
#define MAX9271_SS_0p5  1
#define MAX9271_SS_1p5  2
#define MAX9271_SS_2p0  3
#define MAX9271_SS_1p0  5
#define MAX9271_SS_3p0  6
#define MAX9271_SS_4p0  7

#define MAX927x_PRNG_25MHZ 0
#define MAX927x_PRNG_50MHZ 1
#define MAX927x_PRNG_AUTO  3

#define MAX927x_SRNG_1GBPS 0
#define MAX927x_SRNG_2GBPS 1
#define MAX927x_SRNG_AUTO  3

#define MAX927x_AUTOFM_ONCE  0
#define MAX927x_AUTOFM_2MS   1
#define MAX927x_AUTOFM_16MS  2
#define MAX927x_AUTOFM_256MS 3

#define MAX9271_CMLLVL_100MV 2
#define MAX9271_CMLLVL_150MV 3
#define MAX9271_CMLLVL_200MV 4
#define MAX9271_CMLLVL_250MV 5
#define MAX9271_CMLLVL_300MV 6
#define MAX9271_CMLLVL_350MV 7
#define MAX9271_CMLLVL_400MV 8
#define MAX9271_CMLLVL_450MV 9
#define MAX9271_CMLLVL_500MV 10

#define MAX9271_PREEMP_OFF         0
#define MAX9271_PREEMP_NEG1p2DB    1
#define MAX9271_PREEMP_NEG2p5DB    2
#define MAX9271_PREEMP_NEG4p1DB    3
#define MAX9271_PREEMP_NEG5p0DB    4
#define MAX9271_PREEMP_POS1p1DB    8
#define MAX9271_PREEMP_POS2p2DB    9
#define MAX9271_PREEMP_POS3p3DB    10
#define MAX9271_PREEMP_POS4p4DB    11
#define MAX9271_PREEMP_POS6p0DB    12
#define MAX9271_PREEMP_POS8p0DB    13
#define MAX9271_PREEMP_POS10p5DB   14
#define MAX9271_PREEMP_POS14p0DB   15

/*
 * Max9271 undocumented settings related to
 * the reverse channel:
 *  DIGFLT - digital filter
 *  LOGAIN - decreases pre-amplifier gain by 1.4
 *  HIGAIN - increases pre-amplifier gain by 1.5
 *  HIBW   - filter bandwidth (0=low, 1=high)
 *  HIVTH  - receiver threshold (0=low, 1=high)
 */
#define MAX9271_DIGFLT_NONE 0
#define MAX9271_DIGFLT_32   1
#define MAX9271_DIGFLT_64   2
#define MAX9271_DIGFLT_128  3

#define MAX927X_DRS_HIGH 0
#define MAX927X_DRS_LOW  1

#define MAX927X_BWS_24BIT 0
#define MAX927X_BWS_32BIT 1

#define MAX927X_ES_RISING 0
#define MAX927X_ES_FALLING 1

#define MAX927X_EDC_PARITY  0
#define MAX927X_EDC_CRC6    1
#define MAX927X_EDC_HAMMING 2

#define MAX9271_CHIPID 0x09
#define MAX9272_CHIPID 0x0A

#define MAX9271_CAPS_HDCP 1

/*
 * MAX9271_SETTING(<name>, <register>, <position>, <width>)
 */
#define MAX9271_SETTINGS \
	MAX9271_SETTING(SERID,          0x00, 1, 7) \
	MAX9271_SETTING(CFGBLOCK,       0x00, 0, 1) \
	MAX9271_SETTING(DESID,          0x01, 1, 7) \
        MAX9271_SETTING(SS,             0x02, 5, 3) \
	MAX9271_SETTING(PRNG,           0x02, 2, 2) \
	MAX9271_SETTING(SRNG,           0x02, 0, 2) \
	MAX9271_SETTING(AUTOFM,         0x03, 6, 2) \
	MAX9271_SETTING(SDIV,           0x03, 0, 6) \
	MAX9271_SETTING(SEREN,          0x04, 7, 1) \
	MAX9271_SETTING(CLINKEN,        0x04, 6, 1) \
	MAX9271_SETTING(PRBSEN,         0x04, 5, 1) \
	MAX9271_SETTING(SLEEP,          0x04, 4, 1) \
	MAX9271_SETTING(INTTYPE,        0x04, 2, 2) \
	MAX9271_SETTING(REVCCEN,        0x04, 1, 1) \
	MAX9271_SETTING(FWDCCEN,        0x04, 0, 1) \
	MAX9271_SETTING(I2CMETHOD,      0x05, 7, 1) \
	MAX9271_SETTING(ENJITFILT,      0x05, 6, 1) \
	MAX9271_SETTING(PRBSLEN,        0x05, 4, 2) \
	MAX9271_SETTING(ENWAKEN,        0x05, 1, 1) \
	MAX9271_SETTING(ENWAKEP,        0x05, 0, 1) \
	MAX9271_SETTING(CMLLVL,         0x06, 4, 4) \
	MAX9271_SETTING(PREEMP,         0x06, 0, 4) \
	MAX9271_SETTING(DBL,            0x07, 7, 1) \
	MAX9271_SETTING(DRS,            0x07, 6, 1) \
	MAX9271_SETTING(BWS,            0x07, 5, 1) \
	MAX9271_SETTING(ES,             0x07, 4, 1) \
	MAX9271_SETTING(HVEN,           0x07, 2, 1) \
	MAX9271_SETTING(EDC,            0x07, 0, 2) \
	MAX9271_SETTING(INVVS,          0x08, 7, 1) \
	MAX9271_SETTING(INVHS,          0x08, 6, 1) \
	MAX9271_SETTING(DIGFLT,         0x08, 4, 3) \
	MAX9271_SETTING(LOGAIN,         0x08, 3, 1) \
	MAX9271_SETTING(HIGAIN,         0x08, 2, 1) \
	MAX9271_SETTING(HIBW,           0x08, 1, 1) \
	MAX9271_SETTING(HIVTH,          0x08, 0, 1) \
	MAX9271_SETTING(I2CSRCA,        0x09, 1, 7) \
	MAX9271_SETTING(I2CDSTA,        0x0A, 1, 7) \
	MAX9271_SETTING(I2CSRCB,        0x0B, 1, 7) \
	MAX9271_SETTING(I2CDSTB,        0x0C, 1, 7) \
	MAX9271_SETTING(I2CLOCACK,      0x0D, 7, 1) \
	MAX9271_SETTING(I2CSLVSH,       0x0D, 5, 2) \
	MAX9271_SETTING(I2CMSTBT,       0x0D, 2, 3) \
	MAX9271_SETTING(I2CSLVTO,       0x0D, 0, 2) \
	MAX9271_SETTING(DIS_REV_P,      0x0E, 7, 1) \
	MAX9271_SETTING(DIS_REV_N,      0x0E, 6, 1) \
	MAX9271_SETTING(GPIO_EN,        0x0E, 1, 5) \
	MAX9271_SETTING(GPIO_SET,       0x0F, 1, 5) \
	MAX9271_SETTING(SETGPO,         0x0F, 0, 1) \
	MAX9271_SETTING(GPIO_READ,      0x10, 1, 5) \
	MAX9271_SETTING(GPO_L,          0x10, 0, 1) \
	MAX9271_SETTING(ERRGRATE,       0x11, 6, 2) \
	MAX9271_SETTING(ERRGTYPE,       0x11, 4, 2) \
	MAX9271_SETTING(ERRGCNT,        0x11, 2, 2) \
	MAX9271_SETTING(ERRGPER,        0x11, 1, 1) \
	MAX9271_SETTING(ERRGEN,         0x11, 0, 1) \
	MAX9271_SETTING(CXTP,           0x15, 7, 1) \
	MAX9271_SETTING(I2CSEL,         0x15, 6, 1) \
	MAX9271_SETTING(LCCEN,          0x15, 5, 1) \
	MAX9271_SETTING(OUTPUTEN,       0x15, 1, 1) \
	MAX9271_SETTING(PCLKDET,        0x15, 0, 1) \
	MAX9271_SETTING(ID,             0x1E, 0, 8) \
	MAX9271_SETTING(CAPS,           0x1F, 4, 1) \
	MAX9271_SETTING(REVISION,       0x1F, 0, 4)

#define MAX9272_SS_NONE 0
#define MAX9272_SS_2p0  1
#define MAX9272_SS_4p0  3

#define MAX9272_REV_TRF_211_100 0
#define MAX9272_REV_TRF_281_200 1
#define MAX9272_REV_TRF_422_300 2
#define MAX9272_REV_TRF_563_400 3

/*
 * Max9272 undocumented setting: REV_AMP
 * Reverse-channel transmitter pulse amplitude
 */
static inline __attribute__((unused)) unsigned int max9272_rev_amp_to_millivolts(u8 val) {
	return (val < 8 ? (30 + 10 * val) : (90 + 10 * (val - 8)));
}
static inline __attribute__((unused)) int max9272_millivolts_to_rev_amp(unsigned int mvolts, u8 *valp) {
	unsigned int centivolts = (mvolts + 5) / 10;
	if (centivolts < 3 || centivolts > 16)
		return -EINVAL;
	*valp = (centivolts < 10 ? centivolts - 3 : (centivolts - 9) + 8);
	return 0;
}

/*
 * MAX9272_SETTING(<name>, <register>, <position>, <width>)
 */
#define MAX9272_SETTINGS \
	MAX9272_SETTING(SERID,          0x00, 1, 7) \
	MAX9272_SETTING(DESID,          0x01, 1, 7) \
	MAX9272_SETTING(CFGBLOCK,       0x01, 0, 1) \
        MAX9272_SETTING(SS,             0x02, 6, 2) \
	MAX9272_SETTING(PRNG,           0x02, 2, 2) \
	MAX9272_SETTING(SRNG,           0x02, 0, 2) \
	MAX9272_SETTING(AUTOFM,         0x03, 6, 2) \
	MAX9272_SETTING(SDIV,           0x03, 0, 5) \
	MAX9272_SETTING(LOCKED,         0x04, 7, 1) \
	MAX9272_SETTING(OUTENB,         0x04, 6, 1) \
	MAX9272_SETTING(PRBSEN,         0x04, 5, 1) \
	MAX9272_SETTING(SLEEP,          0x04, 4, 1) \
	MAX9272_SETTING(INTTYPE,        0x04, 2, 2) \
	MAX9272_SETTING(REVCCEN,        0x04, 1, 1) \
	MAX9272_SETTING(FWDCCEN,        0x04, 0, 1) \
	MAX9272_SETTING(I2CMETHOD,      0x05, 7, 1) \
	MAX9272_SETTING(DCS,            0x05, 6, 1) \
	MAX9272_SETTING(HVTRMODE,       0x05, 5, 1) \
	MAX9272_SETTING(ENEQ,           0x05, 4, 1) \
	MAX9272_SETTING(EQTUNE,         0x05, 0, 4) \
	MAX9272_SETTING(DBL,            0x07, 7, 1) \
	MAX9272_SETTING(DRS,            0x07, 6, 1) \
	MAX9272_SETTING(BWS,            0x07, 5, 1) \
	MAX9272_SETTING(ES,             0x07, 4, 1) \
	MAX9272_SETTING(HVTRACK,        0x07, 3, 1) \
	MAX9272_SETTING(HVEN,           0x07, 2, 1) \
	MAX9272_SETTING(EDC,            0x07, 0, 2) \
	MAX9272_SETTING(INVVS,          0x08, 7, 1) \
	MAX9272_SETTING(INVHS,          0x08, 6, 1) \
	MAX9272_SETTING(UNEQDBL,        0x08, 4, 1) \
	MAX9272_SETTING(DISSTAG,        0x08, 3, 1) \
	MAX9272_SETTING(AUTORST,        0x08, 2, 1) \
	MAX9272_SETTING(ERRSEL,         0x08, 0, 2) \
	MAX9272_SETTING(I2CSRCA,        0x09, 1, 7) \
	MAX9272_SETTING(I2CDSTA,        0x0A, 1, 7) \
	MAX9272_SETTING(I2CSRCB,        0x0B, 1, 7) \
	MAX9272_SETTING(I2CDSTB,        0x0C, 1, 7) \
	MAX9272_SETTING(I2CLOCACK,      0x0D, 7, 1) \
	MAX9272_SETTING(I2CSLVSH,       0x0D, 5, 2) \
	MAX9272_SETTING(I2CMSTBT,       0x0D, 2, 3) \
	MAX9272_SETTING(I2CSLVTO,       0x0D, 0, 2) \
	MAX9272_SETTING(GPIEN,          0x0E, 5, 1) \
	MAX9272_SETTING(GPI_READ,       0x0E, 4, 1) \
	MAX9272_SETTING(GPIO1_SET,      0x0E, 3, 1) \
	MAX9272_SETTING(GPIO1_READ,     0x0E, 2, 1) \
	MAX9272_SETTING(GPIO0_SET,      0x0E, 1, 1) \
	MAX9272_SETTING(GPIO0_READ,     0x0E, 0, 1) \
	MAX9272_SETTING(DETTHR,         0x0F, 0, 8) \
	MAX9272_SETTING(DETERR,         0x10, 0, 8) \
	MAX9272_SETTING(CORRTHR,        0x11, 0, 8) \
	MAX9272_SETTING(CORRERR,        0x12, 0, 8) \
	MAX9272_SETTING(PRBSERR,        0x13, 0, 8) \
	MAX9272_SETTING(PRBSOK,         0x14, 7, 1) \
	MAX9272_SETTING(REV_TRF,        0x15, 5, 2) \
	MAX9272_SETTING(REV_AMP,        0x15, 0, 4) \
	MAX9272_SETTING(CXTP,           0x1D, 7, 1) \
	MAX9272_SETTING(CXSEL,          0x1D, 6, 1) \
	MAX9272_SETTING(I2CSEL,         0x1D, 5, 1) \
	MAX9272_SETTING(LCCEN,          0x1D, 4, 1) \
	MAX9272_SETTING(ID,             0x1E, 0, 8) \
	MAX9272_SETTING(CAPS,           0x1F, 4, 1) \
	MAX9272_SETTING(REVISION,       0x1F, 0, 4)

#endif /* max927x_h__ */
