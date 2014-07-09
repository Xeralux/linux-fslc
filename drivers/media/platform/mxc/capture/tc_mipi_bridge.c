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
#include "max_ap0100_func.h"

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

// Test mode from Toshiba bridge chip on camera daughter-board
//#define MIPI_BRIDGE_TEST_MODE

static int tc_mipi_bridge_pos=0;

struct i2c_client *tc_mipi_bridge_i2cclient = NULL;

static MIPI_DATA_TYPE parallel_output[] = {
   {2, 0x0002, 0x0001},
   {2, 0x0002, 0x0000},
   {2, 0x0016, 0x70A2}, // 97.8Mhz
   {2, 0x0018, 0x0413},

   {2, 0x0020, 0x000A},
   {2, 0x000C, 0x0201},
   {2, 0x0006, 0x0080},
   {2, 0x0008, 0x0000},
   {2, 0x0060, 0x8002},

   {2, 0x0004, 0x8045},
};

static const struct tc_mipi_bridge_platform parallel_output_normal = {
	.write_test_pattern = false,
	.regs = parallel_output,
};

static s32 tc_mipi_bridge_write_reg(u16 reg, u16 val)
{
	u8 au8Buf[4] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	if (i2c_master_send(tc_mipi_bridge_i2cclient, au8Buf, 4) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 tc_mipi_bridge_write_reg_4B(u16 reg, u32 val)
{
	u8 au8Buf[6];

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;
	au8Buf[4] = val >> 24;
	au8Buf[5] = val >> 16;

	if (i2c_master_send(tc_mipi_bridge_i2cclient, au8Buf, 6) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

s32 tc_mipi_bridge_read_reg(u16 reg, u16 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 au8RdBuf[2] = {0};

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(tc_mipi_bridge_i2cclient, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (2 != i2c_master_recv(tc_mipi_bridge_i2cclient, au8RdBuf, 2)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, ((au8RdBuf[0] << 8) & 0xff00) | (au8RdBuf[1] & 0x00ff));
		return -1;
	}

	*val = ((au8RdBuf[0] << 8) & 0xff00) | (au8RdBuf[1] & 0x00ff);

	return 0;
}

#if 0
static s32 tc_mipi_bridge_read_reg_4B(u16 reg, u32 val)
{
	u8 au8Buf[6];

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;
	au8Buf[4] = val >> 24;
	au8Buf[5] = val >> 16;

	if (i2c_master_send(tc_mipi_bridge_i2cclient, au8Buf, 6) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}
#endif

static void write_color_bar(void)
{
	int i,j;

	for (j=0; j<4; j++) {
		// 64 pixels of red
		for (i=0; i<32; i++) {
			tc_mipi_bridge_write_reg(0x00e8, 0x0000);
			tc_mipi_bridge_write_reg(0x00e8, 0x00ff);
		}

		// 64 pixels of green
		for (i=0; i<32; i++) {
			tc_mipi_bridge_write_reg(0x00e8, 0x7f00);
			tc_mipi_bridge_write_reg(0x00e8, 0x7f00);
		}

		// 64 pixels of blue
		for (i=0; i<32; i++) {
			tc_mipi_bridge_write_reg(0x00e8, 0xc0ff);
			tc_mipi_bridge_write_reg(0x00e8, 0xc000);
			}

		// 64 pixels of white
		for (i=0; i<32; i++) {
			tc_mipi_bridge_write_reg(0x00e8, 0xff7f);
			tc_mipi_bridge_write_reg(0x00e8, 0xff7f);
			}
   	}
	tc_mipi_bridge_write_reg(0x00e0, 0xC2CF);
}

void tc_mipi_bridge_reset(void)
{
	tc_mipi_bridge_write_reg(0x0002,0x0001);
	tc_mipi_bridge_write_reg(0x0002,0x0000);
}
EXPORT_SYMBOL(tc_mipi_bridge_reset);

void tc_mipi_bridge_stop(void)
{
	u16 val;

	//Set FrmStop
	tc_mipi_bridge_read_reg(0x0032, &val);
	val |= 1 << 15;
	tc_mipi_bridge_write_reg(0x0032, val);
	msleep(1000);

	//Clear PP_En
	tc_mipi_bridge_read_reg(0x0004, &val);
	val &= ~(1 << 6);
	tc_mipi_bridge_write_reg(0x0004,val);

	//Set RstPtr
	tc_mipi_bridge_read_reg(0x0032, &val);
	val |= 1 << 14;
	tc_mipi_bridge_write_reg(0x0032, val);
}
EXPORT_SYMBOL(tc_mipi_bridge_stop);

void tc_mipi_bridge_start(void)
{
	u16 val;
	//Clear FrmStop and RstPtr
	tc_mipi_bridge_read_reg(0x0032, &val);
	val &= ~(1 << 15);
	val &= ~(1 << 14);
	tc_mipi_bridge_write_reg(0x0032, val);
	//Set PP_En
	tc_mipi_bridge_read_reg(0x0004, &val);
	val |= (1 << 6);
	tc_mipi_bridge_write_reg(0x0004, val);
	msleep(1000);
}
EXPORT_SYMBOL(tc_mipi_bridge_start);

// mode: 0: mipi output, 1: mipi test pattern output, 2: parallel output, 3: mipi test output for parallel
s32 tc_mipi_bridge_dev_init(const struct tc_mipi_bridge_platform* data)
{
	u16  val=0;
	MIPI_DATA_TYPE *mipi_reg_ptr;

	pr_debug("%s: pos = %d\n", __func__, tc_mipi_bridge_pos);

	tc_mipi_bridge_read_reg(0x0, &val);
	if (val !=  0x4401) {
		pr_err("mipi bridge not found = 0x%x\n", val);
		return -1;
	} else {
		pr_debug("mipi bridge found = 0x%x\n", val);
	}

	mipi_reg_ptr = data->regs;
	while (mipi_reg_ptr && mipi_reg_ptr->data_size != 0) {
		if (mipi_reg_ptr->data_size == 2)
			tc_mipi_bridge_write_reg(mipi_reg_ptr->reg_addr, mipi_reg_ptr->data);
		else
			tc_mipi_bridge_write_reg_4B(mipi_reg_ptr->reg_addr, mipi_reg_ptr->data);
		mipi_reg_ptr++;
	}

	if (data->write_test_pattern)
		write_color_bar();

	pr_debug("mipi bridge init done\n");
	return 0;
}

EXPORT_SYMBOL(tc_mipi_bridge_dev_init);

/*!
 * tc_mipi_bridge I2C probe function
 *
 */
static int tc_mipi_bridge_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval;

	retval = of_property_read_u32(dev->of_node, "tc_mipi_bridge",
	                              &tc_mipi_bridge_pos);
	if (retval) {
		dev_err(dev, "%s: Field <tc_mipi_bridge> missing or invalid in dtb\n", __func__);
		return retval;
	}

	pr_debug("%s: pos = %d\n", __func__, tc_mipi_bridge_pos);

	tc_mipi_bridge_i2cclient = client;

	// if this device is at position 0, it should be configured as parallel output
	if (tc_mipi_bridge_pos == 0)
		tc_mipi_bridge_dev_init(&parallel_output_normal);

	pr_debug("tc_mipi_bridge_probe done: pos = %d\n", tc_mipi_bridge_pos);

	return 0;
}

/*!
 * tc_mipi_bridge I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int tc_mipi_bridge_remove(struct i2c_client *client)
{
	tc_mipi_bridge_i2cclient = NULL;
	return 0;
}

static struct of_device_id tc_mipi_bridge_dt_ids[] = {
	{ .compatible = "toshiba,tc_mipi_bridge" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tc_mipi_bridge_dt_ids);

static const struct i2c_device_id tc_mipi_bridge_i2c_id[] = {
	{ .name = "tc_mipi_bridge", .driver_data = 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tc_mipi_bridge_i2c_id);

static struct i2c_driver tc_mipi_bridge_driver = {
	.driver = {
		   .name = "tc_mipi_bridge",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(tc_mipi_bridge_dt_ids),
		   },
	.probe = tc_mipi_bridge_probe,
	.remove = tc_mipi_bridge_remove,
	.id_table = tc_mipi_bridge_i2c_id,
};

module_i2c_driver(tc_mipi_bridge_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_DESCRIPTION("tc_mipi_bridge Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

