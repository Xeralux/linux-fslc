/*
 * Copyright (C) 2013-2014 Leopard Imaging, Inc. All Rights Reserved.
 */

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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include "max927x.h"

#define TEST_RETRY_NUM (5)

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

struct i2c_client *max9271_i2cclient = NULL;

s32 max9271_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2];
	int err;
	u8 check;
	if (max9271_i2cclient== NULL)
		return -1;

	au8Buf[0] = reg;
	au8Buf[1] = val;

	if ((err = i2c_master_send(max9271_i2cclient, au8Buf, 2)) < 0) {
		pr_debug("%s:write reg error:reg=%x,val=%x,err=%d\n",
			__func__, reg, val, err);
		return -1;
	}
	mdelay(10);
	err = max9271_read_reg(reg,&check);
	if(err == 0 && check != val) {
		pr_debug("%s:check error:reg=%x,val=%x,check=%x\n", __func__, reg, val,
					check);
		return -1;
	}
	return err;
}
EXPORT_SYMBOL(max9271_write_reg);

s32 max9271_read_reg(u8 reg, u8 *val)
{
	u8 au8RegBuf[1] = {0};
	u8 au8RdBuf[2] = {0};
	int err;
	if (max9271_i2cclient== NULL)
		return -1;

	au8RegBuf[0] = reg;

	if (1 != (err = i2c_master_send(max9271_i2cclient, au8RegBuf, 1))) {
		pr_debug("%s:write reg error:reg=%x,err=%d\n",
				__func__, reg,err);
		return -1;
	}

	if (1 != (err = i2c_master_recv(max9271_i2cclient, au8RdBuf, 1))) {
		pr_debug("%s:read reg error:reg=%x,val=%x,err=%d\n", __func__, reg, au8RdBuf[0], 
					err);
		return -1;
	}

	*val = au8RdBuf[0];

	return 0;
}

EXPORT_SYMBOL(max9271_read_reg);

s32 max9271_I2C_test(int test_num, int *w_retry, int *r_retry, int *w_fail, int *r_fail)
{
	u8 reg, val, w_val;
	int w_retry_cnt, r_retry_cnt, retry_cnt;
	int i, w_fail_cnt, r_fail_cnt;
	s32 ret=0;

	if (max9271_i2cclient== NULL)
		return -1;

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
	}

	*w_retry = w_retry_cnt;
	*r_retry = r_retry_cnt;
	*w_fail = w_fail_cnt;
	*r_fail = r_fail_cnt;

	max9271_write_reg(reg, 0x0);

	if (w_fail_cnt == 0 && r_fail_cnt == 0)
		return 0;
	else
		return -1;
}

EXPORT_SYMBOL(max9271_I2C_test);

/*!
 * max9271 I2C probe function
 *
 */
static int max9271_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	max9271_i2cclient= client;
	pr_debug("max9271_probe done\n");
	return 0;
}

/*!
 * max9271 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int max9271_remove(struct i2c_client *client)
{
	max9271_i2cclient = NULL;
	return 0;
}

static struct of_device_id max9271_dt_ids[] = {
	{ .compatible = "maxim,max9271" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max9271_dt_ids);

static const struct i2c_device_id max9271_i2c_id[] = {
	{ "max9271", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max9271_i2c_id);

static struct i2c_driver max9271_driver = {
	.driver = {
		   .name = "max9271",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(max9271_dt_ids),
		   },
	.probe = max9271_probe,
	.remove = max9271_remove,
	.id_table = max9271_i2c_id,
};

module_i2c_driver(max9271_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_DESCRIPTION("max9271 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

