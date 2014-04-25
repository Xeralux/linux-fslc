/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif


extern s32 max9271_read_reg(u8 reg, u8 *val);
extern s32 max9271_write_reg(u8 reg, u8 val);
extern s32 max9272_read_reg(u8 reg, u8 *val);
extern s32 max9272_write_reg(u8 reg, u8 val);

int max9272_link_locked(void);

extern s32 max9272_I2C_test(int test_num, int *w_fail, int *r_fail);
extern s32 max9271_I2C_test(int test_num, int *w_retry, int *r_retry, int *w_fail, int *r_fail);

#define MAX9271_REG_04_SEREN_SHIFT (7)
#define MAX9271_REG_04_CLINKEN_SHIFT (6)

#define MAX9272_REG_04_LOCKED_SHIFT (7)
#define MAX9272_REG_04_OUTENB_SHIFT (6)

#define MAX927X_REG_04_PRBSEN_SHIFT (5)
#define MAX927X_REG_04_SLEEP_SHIFT (4)
#define MAX927X_REG_04_INTTYPE_SHIFT (2)
#define MAX927X_REG_04_REVCCEN_SHIFT (1)
#define MAX927X_REG_04_FWDCCEN_SHIFT (0)


#define MAX9272_REG_04_SEREN_SHIFT (7)
#define MAX9272_REG_04_CLINKEN_SHIFT (6)
#define MAX9272_REG_04_PRBSEN_SHIFT (5)
#define MAX9272_REG_04_SLEEP_SHIFT (4)
#define MAX9272_REG_04_INTTYPE_SHIFT (2)
#define MAX9272_REG_04_REVCCEN_SHIFT (1)
#define MAX9272_REG_04_FWDCCEN_SHIFT (0)

#define MAX927X_REG_05_I2CMETHOD_SHIFT (7)

#define MAX9272_REG_05_DCS_SHIFT (6)
#define MAX9272_REG_05_HVTRMODE_SHIFT (5)
#define MAX9272_REG_05_ENEQ_SHIFT (4)
#define MAX9272_REG_05_EQTUNE_SHIFT (0)

#define MAX9271_REG_05_ENJITFILT_SHIFT (6)
#define MAX9271_REG_05_PRBSLEN_SHIFT (4)
#define MAX9271_REG_05_ENWAKEN_SHIFT (1)
#define MAX9271_REG_05_ENWAKEP_SHIFT (0)

#define MAX9271_REG_06_CMLLVL_SHIFT (4)
#define MAX9271_REG_06_PREEMP_SHIFT (0)

#define MAX927X_REG_07_DBL_SHIFT  (7)
#define MAX927X_REG_07_DRS_SHIFT  (6)
#define MAX927X_REG_07_BWS_SHIFT (5)
#define MAX927X_REG_07_ES_SHIFT (4)
#define MAX9272_REG_07_HVTRACK_SHIFT (3)
#define MAX927X_REG_07_HVEN_SHIFT (2)
#define MAX927X_REG_07_EDC_SHIFT (0)

#define MAX927X_REG_08_INVVS_SHIFT (7)
#define MAX927X_REG_08_INVHS_SHIFT (6)
#define MAX9272_REG_08_UNEQDBL_SHIFT (4)
#define MAX9272_REG_08_DISSTAG_SHIFT (3)
#define MAX9272_REG_08_AUTORST_SHIFT (2)
#define MAX9272_REG_08_ERRSEL_SHIFT (0)

#define MAX927X_REG_0D_I2CLOCACK_SHIFT (7)
#define MAX927X_REG_0D_I2CSLVSH_SHIFT  (5)
#define MAX927X_REG_0D_I2CMSTBT_SHIFT  (2)
#define MAX927X_REG_0D_I2CSLVTO_SHIFT  (0)

#define MAX9271_REG_0E_DIS_REV_P_SHIFT (7)
#define MAX9271_REG_0E_DIS_REV_N_SHIFT (6)
#define MAX9271_REG_0E_DIS_GPIO5EN_SHIFT (5)
#define MAX9271_REG_0E_DIS_GPIO4EN_SHIFT (4)
#define MAX9271_REG_0E_DIS_GPIO3EN_SHIFT (3)
#define MAX9271_REG_0E_DIS_GPIO2EN_SHIFT (2)
#define MAX9271_REG_0E_DIS_GPIO1EN_SHIFT (1)

#define MAX9271_REG_0F_GPIO5OUT_SHIFT (5)
#define MAX9271_REG_0F_GPIO4OUT_SHIFT (4)
#define MAX9271_REG_0F_GPIO3OUT_SHIFT (3)
#define MAX9271_REG_0F_GPIO2OUT_SHIFT (2)
#define MAX9271_REG_0F_GPIO1OUT_SHIFT (1)
#define MAX9271_REG_0F_SETGP0_SHIFT (0)

static s32 max9271_enable(void)
{
	s32 retval=0;
	u8 regval;
	pr_debug("max9271 enable\n");

	regval = (1 << MAX9271_REG_04_SEREN_SHIFT) |
			 (0 << MAX9271_REG_04_CLINKEN_SHIFT) |
			 (0 << MAX927X_REG_04_PRBSEN_SHIFT) |
			 (0 << MAX927X_REG_04_SLEEP_SHIFT) |
			 (0x1 << MAX927X_REG_04_INTTYPE_SHIFT) |
			 (1 << MAX927X_REG_04_REVCCEN_SHIFT) |
			 (1 << MAX927X_REG_04_FWDCCEN_SHIFT);
	retval = max9271_write_reg(0x04, regval);
//	retval = max9271_write_reg(0x04, 0x87);

	if (retval) {
		pr_debug("max9271 enable failed \n");
		return -1;
	}

	pr_debug("max9271 enable done \n");

	return retval;

}

/*
static s32 max9272_remote_ena(int ena)
{
	s32 retval=0;

	if (ena) {
		retval = max9272_write_reg(0x04, 0x07);
		if ( retval == 0) {
			pr_debug("max9272 remote reverse control channel enabled \n");
		} else {
			pr_debug("max9272 remote reverse control channel enable failed \n");
		}
	} else {
		retval = max9272_write_reg(0x04, 0x05);
		if ( retval == 0) {
			pr_debug("max9272 remote reverse control channel disabled\n");
		} else {
			pr_debug("max9272 remote reverse control channel disable failed\n");
		}
	}

	mdelay(5);

	return retval;

}
*/

static s32 max9272_magic_reg(void)
{
	s32 retval=0;

	retval = max9272_write_reg(0x15, 0x24);
	mdelay(5);

	return retval;
}

static s32 max927x_init(void)
{
	s32 retval=0;
	u8 regval;
	pr_debug("max927x init\n");

	retval = max9272_write_reg(0x15, 0x24);
	mdelay(5);
	//retval  |= max9271_write_reg(0x08, 0x08);
	/*
	 *
	 */
#if 0
	regval = (0 << MAX927X_REG_05_I2CMETHOD_SHIFT) |
			 (0 << MAX9272_REG_05_DCS_SHIFT) |
			 (1 << MAX9272_REG_05_HVTRMODE_SHIFT) |
			 (1 << MAX9272_REG_05_ENEQ_SHIFT) |
			 (0x0 << MAX9272_REG_05_EQTUNE_SHIFT);
	retval  |= max9272_write_reg(0x05, regval);
#endif
	regval = (1 << MAX927X_REG_0D_I2CLOCACK_SHIFT) |
			 (0x1 << MAX927X_REG_0D_I2CSLVSH_SHIFT) |
			 (0x2 << MAX927X_REG_0D_I2CMSTBT_SHIFT) |
			 (0x2 << MAX927X_REG_0D_I2CSLVTO_SHIFT);
	retval  |= max9272_write_reg(0x0d, regval);
	mdelay(5);

	regval = (0 << MAX9271_REG_04_SEREN_SHIFT) |
			 (1 << MAX9271_REG_04_CLINKEN_SHIFT) |
			 (0 << MAX927X_REG_04_PRBSEN_SHIFT) |
			 (0 << MAX927X_REG_04_SLEEP_SHIFT) |
			 (0x1 << MAX927X_REG_04_INTTYPE_SHIFT) |
			 (1 << MAX927X_REG_04_REVCCEN_SHIFT) |
			 (1 << MAX927X_REG_04_FWDCCEN_SHIFT);
	retval  |= max9271_write_reg(0x04, regval);

#if 0
	regval = (0 << MAX927X_REG_05_I2CMETHOD_SHIFT) |
			 (1 << MAX9271_REG_05_ENJITFILT_SHIFT) |
			 (0x0 << MAX9271_REG_05_PRBSLEN_SHIFT) |
			 (0 << MAX9271_REG_05_ENWAKEN_SHIFT) |
			 (1 << MAX9271_REG_05_ENWAKEP_SHIFT);
	retval  |= max9271_write_reg(0x05, regval);

	regval = (0xA << MAX9271_REG_06_CMLLVL_SHIFT) |
			 (0x0 << MAX9271_REG_06_PREEMP_SHIFT);
	retval  |= max9271_write_reg(0x06, regval);
#endif

	regval = (1 << MAX927X_REG_0D_I2CLOCACK_SHIFT) |
			 (0x1 << MAX927X_REG_0D_I2CSLVSH_SHIFT) |
			 (0x2 << MAX927X_REG_0D_I2CMSTBT_SHIFT) |
			 (0x2 << MAX927X_REG_0D_I2CSLVTO_SHIFT);
	retval  |= max9271_write_reg(0x0d, regval);

	regval = (1 << MAX927X_REG_07_DBL_SHIFT) |
			 (0 << MAX927X_REG_07_DRS_SHIFT) |
			 (0 << MAX927X_REG_07_BWS_SHIFT) |
			 (0 << MAX927X_REG_07_ES_SHIFT) |
			 (1 << MAX927X_REG_07_HVEN_SHIFT) |
			 (0x2 << MAX927X_REG_07_EDC_SHIFT);
	max9271_write_reg(0x07, regval);

	regval = (1 << MAX927X_REG_07_DBL_SHIFT) |
			 (0 << MAX927X_REG_07_DRS_SHIFT) |
			 (0 << MAX927X_REG_07_BWS_SHIFT) |
			 (1 << MAX927X_REG_07_ES_SHIFT) |
			 (0 << MAX9272_REG_07_HVTRACK_SHIFT) |
			 (1 << MAX927X_REG_07_HVEN_SHIFT) |
			 (0x2 << MAX927X_REG_07_EDC_SHIFT);
	retval  |= max9272_write_reg(0x07, regval);

	regval = (0 << MAX927X_REG_08_INVVS_SHIFT) |
			 (0 << MAX927X_REG_08_INVHS_SHIFT) |
			 (0 << MAX9272_REG_08_UNEQDBL_SHIFT ) |
		 	 (1 << MAX9272_REG_08_DISSTAG_SHIFT) |
			 (1 << MAX9272_REG_08_AUTORST_SHIFT) |
			 (0x2 << MAX9272_REG_08_ERRSEL_SHIFT);
	retval  |= max9272_write_reg(0x08, regval);
	msleep(5);

	regval = (0 << MAX9271_REG_0E_DIS_REV_P_SHIFT) |
			 (1 << MAX9271_REG_0E_DIS_REV_N_SHIFT) |
			 (1 << MAX9271_REG_0E_DIS_GPIO5EN_SHIFT) |
			 (0 << MAX9271_REG_0E_DIS_GPIO4EN_SHIFT) |
			 (0 << MAX9271_REG_0E_DIS_GPIO3EN_SHIFT) |
			 (0 << MAX9271_REG_0E_DIS_GPIO2EN_SHIFT) |
			 (0 << MAX9271_REG_0E_DIS_GPIO1EN_SHIFT);
	retval  |= max9271_write_reg(0x0E, regval); // enable GPIO5
	msleep(5);

	if (retval) {
		pr_debug("max927x init failed \n");
		return -1;
	}

	pr_debug("max927x init done \n");

	return retval;
}

static s32 max9272_read_error_cnt(unsigned char *detected_err_cnt, unsigned char *corrected_err_cnt)
{
	s32 retval=0;

	retval = max9272_read_reg(0x10, detected_err_cnt);
	if (retval != 0)
		return -1;

	retval = max9272_read_reg(0x12, corrected_err_cnt);
	if (retval != 0)
		return -1;

	return 0;
}
