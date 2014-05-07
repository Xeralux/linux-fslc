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

#define TEST_RETRY_NUM (5)

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

// Ap0100 program is stored in the on-board flash
#define PROG_IN_FLASH
#define I2C_RETRIES (5)
#define CMD_CHECK_RETRIES (20)

typedef struct AP0100_M034_DATA {
    unsigned char data_size;  // bit 7 of data_size is a flag for "read and then write"
    short reg_addr;
    long data;
} AP0100_M034_DATA;

struct i2c_client *ap0100_m034_i2cclient = NULL;

static AP0100_M034_DATA ap0100_normal_mode_reg[] = {
{2, 0x098E, 0xC88C},
{1, 0xC88C, 0x00},
{2, 0xFC00, 0x2800},
{0xC2, 0x0040, 0x8100},
};

static AP0100_M034_DATA ap0100_test_mode_reg[] = {
{2, 0x098E, 0xC88F},
{1, 0xC88F, 0x02},
{1, 0xC88C, 0x02},
{2, 0xFC00, 0x2800},
{0xC2, 0x0040, 0x8100},
};

#ifdef PROG_IN_FLASH
static AP0100_M034_DATA ap0100_cmd_reg[] = {
{0x02, 0x098E, 0x7C00 	},// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
{0x02, 0xFC00, 0x0000 	},// CMD_HANDLER_PARAMS_POOL_0
{0xC2, 0x0040, 0x8900 	},// COMMAND_REGISTER
};
#else
static AP0100_M034_DATA M720p_30fps_hdr_reg[] = {
// [PLL_settings]1: REG=0xCA84, 0x0005
{2, 0x098E, 0xCA84}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
{1, 0xCA84, 0x05}, 	// CAM_SYSCTL_PLL_CONTROL
// [PLL_settings]2: REG=0xCA88, 0x011C
{2, 0xCA88, 0x011C}, 	// CAM_SYSCTL_PLL_DIVIDER_M_N_1_CLK
// [PLL_settings]3: REG=0xCA8C, 0x0088
{2, 0xCA8C, 0x0088}, 	// CAM_SYSCTL_PLL_DIVIDER_P_1_CLK
// [PLL_settings]4: REG=0xCA9C, 0x0485
{2, 0xCA9C, 0x0485}, 	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]1: REG=0xC838, 0x4840871
{4, 0xC838, 0x04840871}, 	// CAM_SENSOR_CONTROL_EXTERNAL_PLL
// [Timing_settings]2: REG=0xC840, 0x0685
{2, 0xC840, 0x0685}, 	// CAM_SENSOR_CONTROL_EXTERNAL_OUTPUT_CLK_DIV
// [Timing_settings]3: REG=0xC804, 0x0080
{2, 0xC804, 0x0080}, 	// CAM_SENSOR_CFG_Y_ADDR_START
// [Timing_settings]4: REG=0xC806, 0x0002
{2, 0xC806, 0x0002 },	// CAM_SENSOR_CFG_X_ADDR_START
// [Timing_settings]5: REG=0xC808, 0x034F
{2, 0xC808, 0x034F}, 	// CAM_SENSOR_CFG_Y_ADDR_END
// [Timing_settings]6: REG=0xC80A, 0x0501
{2, 0xC80A, 0x0501}, 	// CAM_SENSOR_CFG_X_ADDR_END
// [Timing_settings]7: REG=0xC80C, 0x280DE80
{4, 0xC80C, 0x0280DE80},	// CAM_SENSOR_CFG_PIXCLK
// [Timing_settings]8: REG=0xC810, 0x02BC
{2, 0xC810, 0x02BC}, 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
// [Timing_settings]9: REG=0xC812, 0x0754
{2, 0xC812, 0x0754 },	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
// [Timing_settings]10: REG=0xC814, 0x02EA
{2, 0xC814, 0x02EA },	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
// [Timing_settings]11: REG=0xC816, 0x0754
{2, 0xC816, 0x0754},	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
// [Timing_settings]12: REG=0xC818, 0x0000
{2, 0xC818, 0x0000 	},// CAM_SENSOR_CFG_FINE_CORRECTION
// [Timing_settings]13: BITFIELD=0xC844, 0x0007, 0x0002
{0x82, 0xC844, 0x0002 },	// CAM_SENSOR_CONTROL_OPERATION_MODE
// [Timing_settings]14: REG=0xC846, 0x0000
{2, 0xC846, 0x0000 },	// CAM_SENSOR_CONTROL_READ_MODE
// [Timing_settings]15: REG=0xC89C, 0x0000
{2, 0xC89C, 0x0000 },	// CAM_CROP_WINDOW_XOFFSET
// [Timing_settings]16: REG=0xC89E, 0x0000
{2, 0xC89E, 0x0000 },	// CAM_CROP_WINDOW_YOFFSET
// [Timing_settings]17: REG=0xC8A0, 0x0500
{2, 0xC8A0, 0x0500 	},// CAM_CROP_WINDOW_WIDTH
// [Timing_settings]18: REG=0xC8A2, 0x02D0
{2, 0xC8A2, 0x02D0 },	// CAM_CROP_WINDOW_HEIGHT
// [Timing_settings]19: REG=0xC8A4, 0x0011
{2, 0xC8A4, 0x0011 },	// CAM_FRAME_SCAN_CONTROL
// [Timing_settings]20: REG=0xC8A8, 0x0000
{1, 0xC8A8, 0x00 },	// CAM_FOV_CALIB_X_OFFSET
// [Timing_settings]21: REG=0xC8A9, 0x0000
{1, 0xC8A9, 0x00 },	// CAM_FOV_CALIB_Y_OFFSET
// [Timing_settings]22: REG=0xCA90, 0x0500
{2, 0xCA90, 0x0500 	},// CAM_OUTPUT_WIDTH
// [Timing_settings]23: REG=0xCA92, 0x02D0
{2, 0xCA92, 0x02D0 },	// CAM_OUTPUT_HEIGHT
// [Timing_settings]24: REG=0xCA94, 0x001C
{2, 0xCA94, 0x001C },	// CAM_OUTPUT_FORMAT_YUV
// [Timing_settings]25: REG=0xC8BC, 0x0080
{1, 0xC8BC, 0x80 },	// CAM_AET_AEMODE
// [Timing_settings]26: REG=0xC8D4, 0x0000
{2, 0xC8D4, 0x0000 	},// CAM_AET_FRAME_RATE_0
// [Timing_settings]27: REG=0xC8D6, 0x0000
{2, 0xC8D6, 0x0000 	},// CAM_AET_FRAME_RATE_1
// [Timing_settings]28: REG=0xC8D8, 0x0000
{2, 0xC8D8, 0x0000 	},// CAM_AET_FRAME_RATE_2
// [CHANGE COMMAND]1: REG=0xCA9C, 0x0405
{2, 0xCA9C, 0x0405 },	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]30: FIELD_WR=CMD_HANDLER_PARAMS_POOL_0, 0x2800
{0x82, 0xFC00, 0x2800 	},// CMD_HANDLER_PARAMS_POOL_0
// [Timing_settings]31: FIELD_WR=COMMAND_REGISTER, 0x8100
{0xC2, 0x0040, 0x8100 	},// COMMAND_REGISTER
};

static AP0100_M034_DATA M480p_22fps_hdr_reg[] = {
// [PLL_settings]1: REG=0xCA84, 0x0005
{2, 0x098E, 0xCA84}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
{1, 0xCA84, 0x05}, 	// CAM_SYSCTL_PLL_CONTROL
// [PLL_settings]2: REG=0xCA88, 0x011C
{2, 0xCA88, 0x011C}, 	// CAM_SYSCTL_PLL_DIVIDER_M_N_1_CLK
// [PLL_settings]3: REG=0xCA8C, 0x0088
{2, 0xCA8C, 0x0088}, 	// CAM_SYSCTL_PLL_DIVIDER_P_1_CLK
// [PLL_settings]4: REG=0xCA9C, 0x0485
{2, 0xCA9C, 0x0485}, 	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]1: REG=0xC838, 0x4840871
{4, 0xC838, 0x04840871}, 	// CAM_SENSOR_CONTROL_EXTERNAL_PLL
// [Timing_settings]2: REG=0xC840, 0x0685
{2, 0xC840, 0x0685}, 	// CAM_SENSOR_CONTROL_EXTERNAL_OUTPUT_CLK_DIV
// [Timing_settings]3: REG=0xC804, 0x00F8
{2, 0xC804, 0x0008}, 	// CAM_SENSOR_CFG_Y_ADDR_START
// [Timing_settings]4: REG=0xC806, 0x0142
{2, 0xC806, 0x0002 },	// CAM_SENSOR_CFG_X_ADDR_START
// [Timing_settings]5: REG=0xC808, 0x034F
{2, 0xC808, 0x03C7}, 	// CAM_SENSOR_CFG_Y_ADDR_END
// [Timing_settings]6: REG=0xC80A, 0x0501
{2, 0xC80A, 0x0501}, 	// CAM_SENSOR_CFG_X_ADDR_END
// [Timing_settings]7: REG=0xC80C, 0x280DE80
{4, 0xC80C, 0x0280DE80},	// CAM_SENSOR_CFG_PIXCLK
// [Timing_settings]8: REG=0xC810, 0x02BC
{2, 0xC810, 0x02BC}, 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
// [Timing_settings]9: REG=0xC812, 0x0754
{2, 0xC812, 0x0764 },	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
// [Timing_settings]10: REG=0xC814, 0x02EA
{2, 0xC814, 0x03DA },	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
// [Timing_settings]11: REG=0xC816, 0x0754
{2, 0xC816, 0x0764},	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
// [Timing_settings]12: REG=0xC818, 0x0000
{2, 0xC818, 0x0000 	},// CAM_SENSOR_CFG_FINE_CORRECTION
// [Timing_settings]13: BITFIELD=0xC844, 0x0007, 0x0002
{0x82, 0xC844, 0x0002 },	// CAM_SENSOR_CONTROL_OPERATION_MODE
// [Timing_settings]14: REG=0xC846, 0x0000
{2, 0xC846, 0x0000 },	// CAM_SENSOR_CONTROL_READ_MODE
// [Timing_settings]15: REG=0xC89C, 0x0000
{2, 0xC89C, 0x0000 },	// CAM_CROP_WINDOW_XOFFSET
// [Timing_settings]16: REG=0xC89E, 0x0000
{2, 0xC89E, 0x0000 },	// CAM_CROP_WINDOW_YOFFSET
// [Timing_settings]17: REG=0xC8A0, 0x0500
{2, 0xC8A0, 0x0500 	},// CAM_CROP_WINDOW_WIDTH
// [Timing_settings]18: REG=0xC8A2, 0x02D0
{2, 0xC8A2, 0x03C0 },	// CAM_CROP_WINDOW_HEIGHT
// [Timing_settings]19: REG=0xC8A4, 0x0011
{2, 0xC8A4, 0x0011 },	// CAM_FRAME_SCAN_CONTROL
// [Timing_settings]20: REG=0xC8A8, 0x0000
{1, 0xC8A8, 0x00 },	// CAM_FOV_CALIB_X_OFFSET
// [Timing_settings]21: REG=0xC8A9, 0x0000
{1, 0xC8A9, 0x00 },	// CAM_FOV_CALIB_Y_OFFSET
// [Timing_settings]22: REG=0xCA90, 0x0500
{2, 0xCA90, 0x0280 	},// CAM_OUTPUT_WIDTH
// [Timing_settings]23: REG=0xCA92, 0x02D0
{2, 0xCA92, 0x01E0 },	// CAM_OUTPUT_HEIGHT
// [Timing_settings]24: REG=0xCA94, 0x001C
{2, 0xCA94, 0x001C },	// CAM_OUTPUT_FORMAT_YUV
// [Timing_settings]25: REG=0xC8BC, 0x0080
{1, 0xC8BC, 0x80 },	// CAM_AET_AEMODE
// [Timing_settings]26: REG=0xC8D4, 0x0000
{2, 0xC8D4, 0x0000 	},// CAM_AET_FRAME_RATE_0
// [Timing_settings]27: REG=0xC8D6, 0x0000
{2, 0xC8D6, 0x0000 	},// CAM_AET_FRAME_RATE_1
// [Timing_settings]28: REG=0xC8D8, 0x0000
{2, 0xC8D8, 0x0000 	},// CAM_AET_FRAME_RATE_2
// [CHANGE COMMAND]1: REG=0xCA9C, 0x0405
{2, 0xCA9C, 0x0405 },	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]30: FIELD_WR=CMD_HANDLER_PARAMS_POOL_0, 0x2800
{0x82, 0xFC00, 0x2800 	},// CMD_HANDLER_PARAMS_POOL_0
// [Timing_settings]31: FIELD_WR=COMMAND_REGISTER, 0x8100
{0xC2, 0x0040, 0x8100 	},// COMMAND_REGISTER
};

static AP0100_M034_DATA M960p_22fps_hdr_reg[] = {
// [PLL_settings]1: REG=0xCA84, 0x0005
{2, 0x098E, 0xCA84}, 	// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
{1, 0xCA84, 0x05}, 	// CAM_SYSCTL_PLL_CONTROL
// [PLL_settings]2: REG=0xCA88, 0x011C
{2, 0xCA88, 0x011C}, 	// CAM_SYSCTL_PLL_DIVIDER_M_N_1_CLK
// [PLL_settings]3: REG=0xCA8C, 0x0088
{2, 0xCA8C, 0x0088}, 	// CAM_SYSCTL_PLL_DIVIDER_P_1_CLK
// [PLL_settings]4: REG=0xCA9C, 0x0485
{2, 0xCA9C, 0x0485}, 	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]1: REG=0xC838, 0x4840871
{4, 0xC838, 0x04840871}, 	// CAM_SENSOR_CONTROL_EXTERNAL_PLL
// [Timing_settings]2: REG=0xC840, 0x0685
{2, 0xC840, 0x0685}, 	// CAM_SENSOR_CONTROL_EXTERNAL_OUTPUT_CLK_DIV
// [Timing_settings]3: REG=0xC804, 0x0080
{2, 0xC804, 0x0008}, 	// CAM_SENSOR_CFG_Y_ADDR_START
// [Timing_settings]4: REG=0xC806, 0x0002
{2, 0xC806, 0x0002 },	// CAM_SENSOR_CFG_X_ADDR_START
// [Timing_settings]5: REG=0xC808, 0x034F
{2, 0xC808, 0x03C7}, 	// CAM_SENSOR_CFG_Y_ADDR_END
// [Timing_settings]6: REG=0xC80A, 0x0501
{2, 0xC80A, 0x0501}, 	// CAM_SENSOR_CFG_X_ADDR_END
// [Timing_settings]7: REG=0xC80C, 0x280DE80
{4, 0xC80C, 0x0280DE80},	// CAM_SENSOR_CFG_PIXCLK
// [Timing_settings]8: REG=0xC810, 0x02BC
{2, 0xC810, 0x02BC}, 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
// [Timing_settings]9: REG=0xC812, 0x0754
{2, 0xC812, 0x0672 },	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
// [Timing_settings]10: REG=0xC814, 0x02EA
{2, 0xC814, 0x046B },	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
// [Timing_settings]11: REG=0xC816, 0x0754
{2, 0xC816, 0x0672},	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
// [Timing_settings]12: REG=0xC818, 0x0000
{2, 0xC818, 0x0000 	},// CAM_SENSOR_CFG_FINE_CORRECTION
// [Timing_settings]13: BITFIELD=0xC844, 0x0007, 0x0002
{0x82, 0xC844, 0x0002 },	// CAM_SENSOR_CONTROL_OPERATION_MODE
// [Timing_settings]14: REG=0xC846, 0x0000
{2, 0xC846, 0x0000 },	// CAM_SENSOR_CONTROL_READ_MODE
// [Timing_settings]15: REG=0xC89C, 0x0000
{2, 0xC89C, 0x0000 },	// CAM_CROP_WINDOW_XOFFSET
// [Timing_settings]16: REG=0xC89E, 0x0000
{2, 0xC89E, 0x0000 },	// CAM_CROP_WINDOW_YOFFSET
// [Timing_settings]17: REG=0xC8A0, 0x0500
{2, 0xC8A0, 0x0500 	},// CAM_CROP_WINDOW_WIDTH
// [Timing_settings]18: REG=0xC8A2, 0x02D0
{2, 0xC8A2, 0x03C0 },	// CAM_CROP_WINDOW_HEIGHT
// [Timing_settings]19: REG=0xC8A4, 0x0011
{2, 0xC8A4, 0x0011 },	// CAM_FRAME_SCAN_CONTROL
// [Timing_settings]20: REG=0xC8A8, 0x0000
{1, 0xC8A8, 0x00 },	// CAM_FOV_CALIB_X_OFFSET
// [Timing_settings]21: REG=0xC8A9, 0x0000
{1, 0xC8A9, 0x00 },	// CAM_FOV_CALIB_Y_OFFSET
// [Timing_settings]22: REG=0xCA90, 0x0500
{2, 0xCA90, 0x0500 	},// CAM_OUTPUT_WIDTH
// [Timing_settings]23: REG=0xCA92, 0x02D0
{2, 0xCA92, 0x03C0 },	// CAM_OUTPUT_HEIGHT
// [Timing_settings]24: REG=0xCA94, 0x001C
{2, 0xCA94, 0x001C },	// CAM_OUTPUT_FORMAT_YUV
// [Timing_settings]25: REG=0xC8BC, 0x0080
{1, 0xC8BC, 0x80 },	// CAM_AET_AEMODE
// [Timing_settings]26: REG=0xC8D4, 0x0000
{2, 0xC8D4, 0x0000 	},// CAM_AET_FRAME_RATE_0
// [Timing_settings]27: REG=0xC8D6, 0x0000
{2, 0xC8D6, 0x0000 	},// CAM_AET_FRAME_RATE_1
// [Timing_settings]28: REG=0xC8D8, 0x0000
{2, 0xC8D8, 0x0000 	},// CAM_AET_FRAME_RATE_2
// [CHANGE COMMAND]1: REG=0xCA9C, 0x0405
{2, 0xCA9C, 0x0405 },	// CAM_PORT_PARALLEL_CONTROL
// [Timing_settings]30: FIELD_WR=CMD_HANDLER_PARAMS_POOL_0, 0x2800
{0x82, 0xFC00, 0x2800 	},// CMD_HANDLER_PARAMS_POOL_0
// [Timing_settings]31: FIELD_WR=COMMAND_REGISTER, 0x8100
{0xC2, 0x0040, 0x8100 	},// COMMAND_REGISTER
};
#endif

static s32 AM_read_reg_1B(u16 reg, u8 *val);
static s32 AM_read_reg_2B(u16 reg, u16 *val);
#ifndef PROG_IN_FLASH
static s32 AM_read_reg_4B(u16 reg, u32 *val);
#endif
s32 ap0100_doorbell_cleared(void);

static s32 AM_write_reg_1B(u16 reg, u8 val)
{
	int err;
	int i;
	u8 check;
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	for(i = 0; i < I2C_RETRIES; i++) {
		if ((err = i2c_master_send(ap0100_m034_i2cclient, au8Buf, 3)) < 0) {
			pr_warn("%s:write reg error:reg=%x,val=%x,err=%d\n",
					__func__, reg, val, err);
			continue;
		}
		if((err = AM_read_reg_1B(reg,&check)) != 0)
			continue;

	    if(val != check) {
			pr_warn("%s:check reg error:reg=%x,val=%x,check=%x\n",
					__func__, reg, val, check);
			continue;
		}
		return 0;
	}
	pr_err("%s: too many retries, giving up\n",__func__);
	return -1;
}

static s32 AM_write_reg_2B(u16 reg, u16 val)
{
	int err;
	int i;
	u16 check;
	u8 au8Buf[4] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	for(i=0; i< I2C_RETRIES; i++) {
		if ((err = i2c_master_send(ap0100_m034_i2cclient, au8Buf, 4)) < 0) {
			pr_warn("%s:write reg error:reg=%x,val=%x,err=%d\n",
					__func__, reg, val, err);
			continue;
		}
		if((err = AM_read_reg_2B(reg,&check)) != 0)
			continue;
		if(val != check) {
			pr_warn("%s:check reg error:reg=%x,val=%x,check=%x\n",
					__func__, reg, val, check);
			continue;
		}
		return 0;
	}
	pr_err("%s: too many retries, giving up\n",__func__);
	return -1;
}

static s32 AM_send_command(u16 reg, u16 val)
{
	int err;
	u16 ap0100_err;
	u8 au8Buf[4] = {0};
	int i;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	for(i=0; i< I2C_RETRIES; i++) {
		if ((err = i2c_master_send(ap0100_m034_i2cclient, au8Buf, 4)) < 0) {
			pr_warn("%s:write reg error:reg=%x,val=%x,err=%d\n",
					__func__, reg, val, err);
		} else {
			break;
		}
	}
	if(i == I2C_RETRIES) {
		pr_err("%s: too many retries, giving up\n",__func__);
		return -1;
	}

	err = ap0100_doorbell_cleared();
	if(err < 0)
		return err;

	for(i=0; i < I2C_RETRIES; i++) {
		if((err = AM_read_reg_2B(reg,&ap0100_err)) == 0 && ap0100_err == 0) {
			return 0;
		}
	}

	pr_err("%s:command error:reg=%x,err=%x,ap0100_err=%x\n",
			__func__, reg, err, ap0100_err);
	return -1;
}

#ifndef PROG_IN_FLASH
static s32 AM_write_reg_4B(u16 reg, u32 val)
{
	int err;
	u8 au8Buf[6];
	u32 check;;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 24;
	au8Buf[3] = val >> 16;
	au8Buf[4] = val >> 8;
	au8Buf[5] = val;

	if ((err = i2c_master_send(ap0100_m034_i2cclient, au8Buf, 6)) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x,err=%d\n",
			__func__, reg, val, err);

	if((err = AM_read_reg_4B(reg,&check)) == 0 && val != check) {
		pr_err("%s:check reg error:reg=%x,val=%x,check=%x\n",
			__func__, reg, val, check);
		return -1;
	}
	return err;
}
#endif

static s32 AM_read_reg_1B(u16 reg, u8 *val)
{
	int err;
	int i;
	u8 au8RegBuf[2] = {0};

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	for(i=0; i< I2C_RETRIES; i++) {
		if (2 != (err = i2c_master_send(ap0100_m034_i2cclient, au8RegBuf, 2))) {
			pr_warn("%s:write reg error:reg=%x,err=%d\n",
				__func__, reg, err);
			continue;
		}

		if (1 != (err = i2c_master_recv(ap0100_m034_i2cclient, val, 1))) {
			pr_warn("%s:read reg error:reg=%x,val=%x,err=%x\n",
					__func__, reg, *val, err);
			continue;
		}
		return 0;
	}
	pr_err("%s: too many retries, giving up\n",__func__);
	return -1;
}

static s32 AM_read_reg_2B(u16 reg, u16 *val)
{
	int err;
	int i;
	u8 au8RegBuf[2] = {0};
	u8 au8RdBuf[2] = {0};

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	for(i=0; i<I2C_RETRIES; i++) {
		if (2 != (err = i2c_master_send(ap0100_m034_i2cclient, au8RegBuf, 2))) {
			pr_warn("%s:write reg error:reg=%x,err=%d\n",
					__func__, reg, err);
			continue;
		}

		if (2 != (err = i2c_master_recv(ap0100_m034_i2cclient, au8RdBuf, 2))) {
			pr_warn("%s:read reg error:reg=%x,val=%x,err=%d\n",
				__func__, reg, ((au8RdBuf[0] << 8) & 0xff00) | (au8RdBuf[1] & 0x00ff),
				err);
			continue;
		}

		*val = ((au8RdBuf[0] << 8) & 0xff00) | (au8RdBuf[1] & 0x00ff);
		return 0;
	}
	pr_err("%s: too many retries, giving up\n",__func__);
	return -1;
}

#if 0
static s32 AM_read_reg_4B(u16 reg, u32 *val)
{
	int err;
	u8 au8RegBuf[2] = {0};
	u8 au8RdBuf[4] = {0};

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != (err = i2c_master_send(ap0100_m034_i2cclient, au8RegBuf, 2))) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (4 != (err = i2c_master_recv(ap0100_m034_i2cclient, au8RdBuf, 4))) {
		pr_err("%s:read reg error:reg=%x,val=%x,err=%x\n",
				__func__, reg, ((au8RdBuf[0] << 8) & 0xff00) | (au8RdBuf[1] & 0x00ff), err);
		return -1;
	}

	*val = ((au8RdBuf[0] << 24) & 0xff000000) | ((au8RdBuf[1] << 16) & 0x00ff0000) \
		| ((au8RdBuf[2] << 24) & 0x0000ff00) | ((au8RdBuf[3] ) & 0x000000ff);

	return 0;
}
#endif

#ifndef PROG_IN_FLASH
static void ap0100_m034_soft_reset(void)
{
	AM_write_reg_2B(0x001A, 0x0015);
	mdelay(10);
	AM_write_reg_2B(0x001A, 0x0E14);
	mdelay(10);
}
#endif

s32 ap0100_doorbell_cleared(void)
{
	int i;
	u16 val=0;

	//mdelay(100);
	for (i=0; i<CMD_CHECK_RETRIES; i++) {
		if ((AM_read_reg_2B(0x0040, &val) == 0) &&
				(val & 0x8000) == 0)
			return 0;
		msleep(5);
	}
	pr_err("%s: doorbell never cleared\n", __func__);
	return -1;
}
s32 ap0100_m034_cmd_status(void)
{
	int i;

	for (i=0; i<CMD_CHECK_RETRIES; i++) {
		if(AM_send_command(0x0040, 0x8101) == 0)
			return 0;
	}
	pr_err("%s: ap0100 never went idle\n", __func__);
	return -1;
}
EXPORT_SYMBOL(ap0100_m034_cmd_status);

s32 ap0100_m034_cmd_write(char *buf, int size)
{
	int err;
	if (size > 2) {
		if ((err = i2c_master_send(ap0100_m034_i2cclient, buf, size)) < 0) {
			pr_err("%s:write reg error:reg=%x\n,err=%d",
				__func__, (u16)buf[0] << 8 | buf[1], err);
			return -1;
		}
	} else {
		pr_err("%s: size=%x\n", __func__, size);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(ap0100_m034_cmd_write);

s32 ap0100_m034_cmd_read(u16 reg, char *read_buf, int sensor_read_len)
{
	u8 au8RegBuf[2] = {0};

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ap0100_m034_i2cclient, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (sensor_read_len != i2c_master_recv(ap0100_m034_i2cclient, read_buf, sensor_read_len)) {
		pr_err("%s:read reg error: len=%d\n",
				__func__, sensor_read_len);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(ap0100_m034_cmd_read);

static s32 ap0100_handle_registers(AP0100_M034_DATA *ap0100_reg, int reg_size)
{
	u16  val=0;
	int i;
	for (i=0; i<reg_size  ; i++) {
		if (ap0100_reg[i].data_size == 1) {
			if ( AM_write_reg_1B(ap0100_reg[i].reg_addr, ap0100_reg[i].data) != 0)
				return -1;
		} else if ( ap0100_reg[i].data_size == 2) {
			if ( AM_write_reg_2B(ap0100_reg[i].reg_addr, ap0100_reg[i].data) != 0)
				return -1;
#ifndef PROG_IN_FLASH
		} else if (ap0100_reg[i].data_size == 4) {
			if ( AM_write_reg_4B(ap0100_reg[i].reg_addr, ap0100_reg[i].data) != 0)
				return -1;
		} else if (ap0100_reg[i].data_size == 0x82) {
			if ( AM_read_reg_2B(ap0100_reg[i].reg_addr, &val) != 0)
				return -1;
			val |= ap0100_reg[i].data;
			if ( AM_write_reg_2B(ap0100_reg[i].reg_addr, val) != 0)
				return -1;
#endif
		} else if (ap0100_reg[i].data_size == 0xC2) {
			if ( AM_send_command(ap0100_reg[i].reg_addr, ap0100_reg[i].data) != 0)
				return -1;
		} else {
			pr_err("Unhandled data size %x\n", ap0100_reg[i].data_size);
			return -1;
		}
	}
	return 0;
}

s32 ap0100_m034_test_mode(int enable)
{
	int reg_size;
	u16  val=0;
	AP0100_M034_DATA *ap0100_reg;
	int err;
	if (enable) {
		reg_size = sizeof(ap0100_test_mode_reg) / sizeof(AP0100_M034_DATA);
		ap0100_reg = ap0100_test_mode_reg;
		pr_debug("Ap0100 to test pattern mode \n");
	} else {
		reg_size = sizeof(ap0100_normal_mode_reg) / sizeof(AP0100_M034_DATA);
		ap0100_reg = ap0100_normal_mode_reg;
		pr_debug("Ap0100 to normal mode \n");
	}

	err = ap0100_handle_registers(ap0100_reg, reg_size);
	if(err < 0)
		return err;

	mdelay(50);
	val = ap0100_m034_cmd_status();

	if (val == 0) {
		return 0;
	} else {
		return -1;
	}

	return 0;
}

// mode: 2: 480p, 0: 720p, 1: 960p, 6: 720p 100% color bar test pattern
s32 ap0100_m034_sensor_init(int mode)
{
	int reg_size;
	u16  val=0;
	AP0100_M034_DATA *ap0100_reg;
	int err;

	AM_read_reg_2B(0x0000, &val);
	if (val != 0x0062) {
		pr_err("ap0100 not found=%x\n", val);
		return -1;
	} else {
		pr_debug("ap0100 found = 0x%x\n", val);
	}

	val = ap0100_m034_cmd_status();
	if(val != 0)
		return val;

#ifdef PROG_IN_FLASH
			reg_size = sizeof(ap0100_cmd_reg) / sizeof(AP0100_M034_DATA);
			ap0100_reg = ap0100_cmd_reg;

	switch (mode) {
		case 2: // VGA
			ap0100_cmd_reg[1].data = 2;
			pr_debug("ap0100 init to 640x480 mode \n");
			break;
		case 0: // 720p
		case 6:
			ap0100_cmd_reg[1].data = 0;
			pr_debug("ap0100 init to 1280x720 mode \n");
			break;
		case 1: // 960p
			ap0100_cmd_reg[1].data = 1;
			pr_debug("ap0100 init to 1280x960 mode \n");
			break;
		default:
			ap0100_cmd_reg[1].data = 1;
			pr_debug("ap0100 init mode not found,  now set to 1280x960 mode \n");
			break;
	}

#else
	ap0100_m034_soft_reset();

	switch (mode) {
		case 2: // VGA
			reg_size = sizeof(M480p_22fps_hdr_reg) / sizeof(AP0100_M034_DATA);
			ap0100_reg = M480p_22fps_hdr_reg;
			pr_debug("ap0100 init to 640x480 mode \n");
			break;
		case 0: // 720p
		case 6:
			reg_size = sizeof(M720p_30fps_hdr_reg) / sizeof(AP0100_M034_DATA);
			ap0100_reg = M720p_30fps_hdr_reg;
			pr_debug("ap0100 init to 1280x720 mode \n");
			break;
		case 1: // 960p
			reg_size = sizeof(M960p_22fps_hdr_reg) / sizeof(AP0100_M034_DATA);
			ap0100_reg = M960p_22fps_hdr_reg;
			pr_debug("ap0100 init to 1280x960 mode \n");
			break;
		default:
			reg_size = sizeof(M480p_22fps_hdr_reg) / sizeof(AP0100_M034_DATA);
			ap0100_reg = M480p_22fps_hdr_reg;
			break;
	}
#endif
	err = ap0100_handle_registers(ap0100_reg, reg_size);
	if(err < 0)
		return err;

	mdelay(50);
	val = ap0100_m034_cmd_status();

	if (val == 0) {
		if (mode == 6) {// test mode
			val = ap0100_m034_test_mode(1);
		} else // normal mode
			val = ap0100_m034_test_mode(0);
		if (val == 0) {
			pr_debug("ap0100 init OK, mode = %d\n", mode);
		} else {
			pr_debug("ap0100 init failed!!\n");
			return -1;
		}
	} else {
		pr_debug("ap0100 init failed!!\n");
		return -1;
	}

	return 0;
}

EXPORT_SYMBOL(ap0100_m034_sensor_init);

s32 ap0100_m034_I2C_test(int test_num, int *w_retry, int *r_retry, int *w_fail, int *r_fail)
{
	u16 reg, val, w_val;
	int w_retry_cnt, r_retry_cnt, retry_cnt;
	int i, w_fail_cnt, r_fail_cnt;
	s32 ret=0;

	AM_read_reg_2B(0x0000, &val);
	if (val != 0x0062) {
		pr_err("ap0100 not found=%x\n", val);
		return -1;
	} else {
		pr_debug("ap0100 found = 0x%x\n", val);
	}

	w_retry_cnt = 0;
	r_retry_cnt = 0;
	w_fail_cnt = 0;
	r_fail_cnt = 0;
	reg = 0xCA90;  // a register for testing
	for (i=0; i<test_num; i++) {
		if ( i % (test_num / 100) == 0) {
			pr_debug("test in progress : %d%%, w_retry_cnt = %d, r_retry_cnt = %d\n", \
				i / (test_num / 100) , w_retry_cnt, r_retry_cnt);
			pr_debug("                         w_fail_cnt = %d,  r_fail_cnt = %d\n",  \
				w_fail_cnt, r_fail_cnt);
		}

		w_val = (u16) (( test_num << 1) & 0x0FFE);
		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = AM_write_reg_2B(reg, w_val);
			if (ret == 0) {
				break;
			} else {
				ret = AM_write_reg_2B(reg, w_val);
				retry_cnt++;
			}
		}
		w_retry_cnt += retry_cnt;
		if (retry_cnt == TEST_RETRY_NUM)
			w_fail_cnt++;

		retry_cnt = 0;
		while (retry_cnt < TEST_RETRY_NUM) {
			ret = AM_read_reg_2B(reg, &val);
			if (ret == 0 && val == w_val) {
				break;
			} else {
				ret = AM_read_reg_2B(reg, &val);
				retry_cnt++;
			}
		}
		r_retry_cnt += retry_cnt;
		if (retry_cnt == TEST_RETRY_NUM || val != w_val) {
			r_fail_cnt++;
			pr_debug("failed: i = %d, r_retry_cnt = %d, write = 0x%x, read = 0x%x\n", i, retry_cnt, w_val, val);
		}
	}

	*w_retry = w_retry_cnt;
	*r_retry = r_retry_cnt;
	*w_fail = w_fail_cnt;
	*r_fail = r_fail_cnt;

	val = 0x0500; // default value
	AM_write_reg_2B(reg, val);

	if (w_fail_cnt == 0 && r_fail_cnt == 0)
		return 0;
	else
		return -1;
}

EXPORT_SYMBOL(ap0100_m034_I2C_test);

s32 ap0100_m034_read_temperature(signed char *cur_temp, signed char *min_temp, signed char *max_temp)
{
	u16  reg;
	s32 ret=0;

	// current temperature
	reg = 0xCAAF;
	ret = AM_read_reg_1B(reg, (u8 *)cur_temp);
	if (ret != 0) {
		//pr_err("ap0100 read cur temperature error \n");
		return -1;
	}

	// minimal temperature
	reg = 0xCAB0;
	ret = AM_read_reg_1B(reg, (u8 *)min_temp);
	if (ret != 0) {
		//pr_err("ap0100 read min temperature error \n");
		return -1;
	}

	// maximal temperature
	reg = 0xCAB1;
	ret = AM_read_reg_1B(reg, (u8 *)max_temp);
	if (ret != 0) {
		//pr_err("ap0100 read max temperature error \n");
		return -1;
	}

	return 0;
}

EXPORT_SYMBOL(ap0100_m034_read_temperature);

/*!
 * ap0100 I2C probe function
 *
 */
static int ap0100_m034_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	ap0100_m034_i2cclient = client;
	pr_debug("ap0100_m034_probe done\n");
	return 0;
}

/*!
 * ap0100_m034 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ap0100_m034_remove(struct i2c_client *client)
{
	ap0100_m034_i2cclient = NULL;
	return 0;
}

static struct of_device_id ap0100_m034_dt_ids[] = {
	{ .compatible = "aptina,ap0100_m034" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ap0100_m034_dt_ids);

static const struct i2c_device_id ap0100_m034_i2c_id[] = {
	{ "ap0100_m034", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ap0100_m034_i2c_id);

static struct i2c_driver ap0100_m034_driver = {
	.driver = {
		   .name = "ap0100_m034",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ap0100_m034_dt_ids),
		   },
	.probe = ap0100_m034_probe,
	.remove = ap0100_m034_remove,
	.id_table = ap0100_m034_i2c_id,
};

module_i2c_driver(ap0100_m034_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_DESCRIPTION("ap0100_m034 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

