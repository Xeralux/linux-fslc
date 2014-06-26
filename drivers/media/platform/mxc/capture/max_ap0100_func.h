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
#ifndef __max_ap0100_func_h__
#define __max_ap0100_func_h__

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

#include "max927x.h"
// times to retry when init failed
#define INIT_RETRY (5)

/* I/O expander PCA9557 to control the reset logic on the FPGA and camera boards */
#define MEDIANODE_PCA9557_BASE_ADDR	IMX_GPIO_NR(8, 24)
#define MEDIANODE_IO_EXP_GPIO(x)	(MEDIANODE_PCA9557_BASE_ADDR + (x))

#define MEDIANODE_PARALLEL_TC_RESET	MEDIANODE_IO_EXP_GPIO(0)
#define MEDIANODE_MIPI_TC_RESET		MEDIANODE_IO_EXP_GPIO(1)
#define MEDIANODE_CAM_PARALLEL_PWR	MEDIANODE_IO_EXP_GPIO(2)
#define MEDIANODE_CAM_MIPI_PWR		MEDIANODE_IO_EXP_GPIO(3)
#define MEDIANODE_AUDIO_RESET		MEDIANODE_IO_EXP_GPIO(4)

extern s32 ap0100_m034_sensor_init(int mode);
extern void pca954x_select_channel (int chan);
extern void pca954x_release_channel (void);
extern s32 tc_mipi_bridge_dev_init(int mode);
extern s32 ap0100_m034_I2C_test(int test_num, int *w_retry, int *r_retry, int *w_fail, int *r_fail);
extern s32 ap0100_m034_read_temperature(signed char *cur_temp, signed char *min_temp, signed char *max_temp);
extern s32 ap0100_m034_cmd_status(void);
extern s32 ap0100_m034_cmd_write(char *buf, int size);
extern s32 ap0100_m034_cmd_read(u16 reg, char *read_buf, int sensor_read_len);
extern s32 ap0100_m034_sensor_set_cmd(int cmd);
extern s32 ap0100_m034_sensor_update_init_buf(unsigned char *buf, int count);
extern s32 ap0100_m034_sensor_mode_init(void);

static s32 ap0100_hw_reset(void)
{
	s32 retval=0;
	u8 val=0;

	retval = max9271_read_reg(0x0F, &val);
	val &= ~(1 << MAX9271_REG_0F_GPIO5OUT_SHIFT);
//	val &= (~0x20); // set GPIO5 low
	retval |= max9271_write_reg(0x0F, val);

	msleep(1100);

	val |= (1 << MAX9271_REG_0F_GPIO5OUT_SHIFT);
//	val |= 0x20; // set GPIO5 high
	retval |= max9271_write_reg(0x0F, val);

	if (retval) {
		pr_debug("ap0100 hw reset failed \n");
		return -1;
	}

	msleep(1000);
	pr_debug("ap0100 hw reset done \n");

	return retval;

}


static int ap0100_init(int mode, int hw_reset)
{
	if (hw_reset) {
		if ( ap0100_hw_reset() != 0)
			return -1;
	}

	if (ap0100_m034_sensor_init(mode) != 0)
		return -1;

	if (max9271_enable() != 0)
		return -1;

	//max9272_remote_ena(0);

	return 0;
}

#define TEST_RETRY_NUM (5)
static s32 max927x_I2C_test(int test_num, int *w_retry, int *r_retry, int *w_fail, int *r_fail)
{
	u8 reg, val, w_val;
	int w_retry_cnt, r_retry_cnt, retry_cnt;
	int i, w_fail_cnt, r_fail_cnt;
	s32 ret=0;

	w_retry_cnt = 0;
	r_retry_cnt = 0;
	w_fail_cnt = 0;
	r_fail_cnt = 0;
	reg = 0x0A; // a register for testing
	for (i=0; i<test_num; i++) {
		if ( i % (test_num / 100) == 0) {
			pr_debug("test in progress : %d%%, w_retry_cnt = %d, r_retry_cnt = %d\n", \
				i / (test_num / 100) , w_retry_cnt, r_retry_cnt);
			pr_debug("                         w_fail_cnt = %d,  r_fail_cnt = %d\n",  \
				w_fail_cnt, r_fail_cnt);
		}

		w_val = (u8)(( i << 1) & 0xFE); // bit 0 should be 0 alwasy

		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = max9271_write_reg(reg, w_val);
			if (ret == 0) {
				break;
			} else {
				ret = max9271_write_reg(reg, w_val);
				retry_cnt++;
			}
		}
		w_retry_cnt += retry_cnt;
		if (retry_cnt == TEST_RETRY_NUM)
			w_fail_cnt++;

		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = max9271_read_reg(reg, &val);
			if (ret == 0 && val == w_val) {
				break;
			} else {
				ret = max9271_read_reg(reg, &val);
				retry_cnt++;
			}
		}
		r_retry_cnt += retry_cnt;

		if (retry_cnt == TEST_RETRY_NUM || val != w_val) {
			r_fail_cnt++;
			pr_debug("failed: test_num = %d, r_retry_cnt = %d, write = 0x%x, read = 0x%x\n", test_num, retry_cnt, w_val, val);
		}

		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = max9272_write_reg(reg, w_val);
			if (ret == 0) {
				break;
			} else {
				ret = max9272_write_reg(reg, w_val);
				retry_cnt++;
			}
		}
		w_retry_cnt += retry_cnt;
		if (retry_cnt == TEST_RETRY_NUM)
			w_fail_cnt++;

		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = max9272_read_reg(reg, &val);
			if (ret == 0 && val == w_val) {
				break;
			} else {
				ret = max9272_read_reg(reg, &val);
				retry_cnt++;
			}
		}
		r_retry_cnt += retry_cnt;

		if (retry_cnt == TEST_RETRY_NUM || val != w_val) {
			r_fail_cnt++;
			pr_debug("failed: test_num = %d, r_retry_cnt = %d, write = 0x%x, read = 0x%x\n", test_num, retry_cnt, w_val, val);
		}


	}

	*w_retry = w_retry_cnt;
	*r_retry = r_retry_cnt;
	*w_fail = w_fail_cnt;
	*r_fail = r_fail_cnt;

	max9271_write_reg(reg, 0x0);
	max9272_write_reg(reg, 0x0);

	if (w_fail_cnt == 0 && r_fail_cnt == 0)
		return 0;
	else
		return -1;
}

#endif
