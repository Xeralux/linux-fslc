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

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

struct i2c_client *max9272_i2cclient = NULL;

s32 max9272_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2];
	int err;
	if (max9272_i2cclient== NULL)
		return -1;

	au8Buf[0] = reg;
	au8Buf[1] = val;

	if ((err = i2c_master_send(max9272_i2cclient, au8Buf, 2)) < 0) {
		pr_debug("%s:write reg error:reg=%x,val=%x,err=%d\n",
			__func__, reg, val, err);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(max9272_write_reg);

s32 max9272_read_reg(u8 reg, u8 *val)
{
	u8 au8RegBuf[1] = {0};
	u8 au8RdBuf[2] = {0};
	int err;
	if (max9272_i2cclient== NULL)
		return -1;

	au8RegBuf[0] = reg;

	if (1 != (err = i2c_master_send(max9272_i2cclient, au8RegBuf, 1))) {
		pr_debug("%s:write reg error:reg=%x,err=%d\n",
				__func__, reg, err);
		return -1;
	}

	if (1 != (err = i2c_master_recv(max9272_i2cclient, au8RdBuf, 1))) {
		pr_debug("%s:read reg error:reg=%x,val=%x,err=%d\n", __func__, reg, au8RdBuf[0], 
					err);
		return -1;
	}

	*val = au8RdBuf[0];

	return 0;
}

EXPORT_SYMBOL(max9272_read_reg);

#define LOCK_MAX_RETRIES (5)

int max9272_link_locked(void)
{
	int i;
	int err;
	u8 val;

	for(i = 0; i < LOCK_MAX_RETRIES; i++) {
		err = max9272_read_reg(0x04,&val);
		if(err == 0 && (val & (1 << MAX9272_REG_04_LOCKED_SHIFT)))
			break;
		mdelay(5);
	}
	if(i == LOCK_MAX_RETRIES) {
		pr_debug("%s:lock error:val=%x,err=%d\n",
			__func__, val, err);
		return -1;
	}
	return 0;
}

EXPORT_SYMBOL(max9272_link_locked);

s32 max9272_I2C_test(int test_num, int *w_fail, int *r_fail)
{
	u8 reg, val;
	int i, w_fail_cnt, r_fail_cnt;
	s32 ret=0;

	if (max9272_i2cclient== NULL)
		return -1;

	w_fail_cnt = 0;
	r_fail_cnt = 0;
	reg = 0x0A; // a register for testing
	for (i=0; i<test_num; i++) {
		if ( i % (test_num / 100) == 0) {
			pr_debug("test in progress : %d%%\n", \
				i / (test_num / 100));
			pr_debug("                         w_fail_cnt = %d,  r_fail_cnt = %d\n",  \
				w_fail_cnt, r_fail_cnt);
		}
		val = (u8) i;
		ret = max9272_write_reg(reg, val);
		if (ret != 0) {
			w_fail_cnt++;
			//max9272_write_reg(reg, 0x0); // restore default
			//return (s32)i + 1;
		}

		ret = max9272_read_reg(reg, &val);
		if (ret != 0 || val != (u8) i) {
			r_fail_cnt++;
			//max9272_write_reg(reg, 0x0); // restore default
			//return (s32)i + 1;
		}
	}

	*w_fail = w_fail_cnt;
	*r_fail = r_fail_cnt;

	max9272_write_reg(reg, 0x0);

	if (w_fail_cnt == 0 && r_fail_cnt == 0)
		return 0;
	else
		return -1;
}

EXPORT_SYMBOL(max9272_I2C_test);
/*!
 * max9272 I2C probe function
 *
 */
static int max9272_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	max9272_i2cclient= client;
	pr_debug("max9272_probe done\n");

	return 0;
}

/*!
 * max9272 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int max9272_remove(struct i2c_client *client)
{
	max9272_i2cclient = NULL;
	return 0;
}

static struct of_device_id max9272_dt_ids[] = {
	{ .compatible = "maxim,max9272" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max9272_dt_ids);

static const struct i2c_device_id max9272_i2c_id[] = {
	{ "max9272", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max9272_i2c_id);

static struct i2c_driver max9272_driver = {
	.driver = {
		   .name = "max9272",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(max9272_dt_ids),
		   },
	.probe = max9272_probe,
	.remove = max9272_remove,
	.id_table = max9272_i2c_id,
};

module_i2c_driver(max9272_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_DESCRIPTION("max9272 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

