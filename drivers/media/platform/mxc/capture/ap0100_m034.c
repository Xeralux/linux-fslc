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

//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/reset.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include <linux/moduleparam.h>
static unsigned long i2c_retries = 10;
module_param(i2c_retries, ulong, 0444);

enum AP0100_CMD {
  CMD_SYS_SET_STATE = 0x8100,
  CMD_SYS_GET_STATE = 0x8101,
  CMD_SYS_CFG_PWR = 0x8102,
  CMD_INVOKE_CMD_SEQ = 0x8900,
  CMD_FLASH_GET_LOCK 		= 0x8500,
  CMD_FLASH_LOCK_STATUS 	= 0x8501,
  CMD_FLASH_RELEASE_LOCK	= 0x8502,
  CMD_FLASH_READ		= 0x8504,	
  CMD_FLASH_WRITE		= 0x8505,
  CMD_FLASH_ERASE_BLOCK	= 0x8506,
  CMD_FLASH_QUERY_DEV	= 0x8508,
  CMD_FLASH_STATUS	= 0x8509,
  CMD_FLASH_CONFIG_DEV	= 0x850a,
  CMD_SEQUENCER_REFRESH = 0x8606,
  CMD_CCMGR_GET_LOCK		= 0X8D00,
  CMD_CCMGR_LOCK_STATUS	= 0X8D01,
  CMD_CCMGR_RELEASE_LOCK	= 0X8D02,
  CMD_CCMGR_READ		= 0X8D05,
  CMD_CCMGR_STATUS		= 0X8D08,
};

enum AP0100_STORED_CMDS {
	STORED_CMD_WDR_START = 0,
	STORED_CMD_720P_84MHz_yuv8_WDR = 0,
	STORED_CMD_960P_84MHz_yuv8_WDR = 1,
	STORED_CMD_VGA_84MHz_yuv8_WDR = 2,
	STORED_CMD_960P_84MHz_yuv8_WDR_LDC = 3,
	STORED_CMD_SDR_START = 4,
	STORED_CMD_720P_84MHz_yuv8_SDR = 4,
	STORED_CMD_960P_84MHz_yuv8_SDR = 5,
	STORED_CMD_VGA_84MHz_yuv8_SDR = 6,
	STORED_CMD_960P_84MHz_yuv8_SDR_LDC = 7,
	STORED_CMD_Test_Pattern_ColorBar_need_to_start_streaming_first = 8,
	STORED_CMD_Recover_from_test_pattern = 9,
	STORED_CMD_Image_Orientation_original = 0xa,
	STORED_CMD_Flip = 0xb,
	STORED_CMD_Mirror = 0xc,
	STORED_CMD_Flip_And_Mirror = 0xd,
};

enum AP0100_REG {
  REG_CHIP_VERSION = 0x0000,
  REG_RST_MISC_CTL = 0x001A, //2 bytes
  REG_MCU_BOOT_OPT = 0x0020, //2 bytes
  REG_CMD = 0x0040,
  REG_XDMA_ACCESS_CTL_STAT = 0x0982,
  REG_LOGICAL_ADDR_ACCESS = 0x098E,
  REG_CAM_MODE_SELECT = 0xC88C,
  REG_CAM_MODE_TEST_PATTERN_SELECT = 0xC88F,
  REG_CAM_MODE_TEST_PATTERN_RED = 0xC890,
  REG_CAM_MODE_TEST_PATTERN_GREEN = 0xC894,
  REG_CAM_MODE_TEST_PATTERN_BLUE = 0xC898,
  REG_CAM_AET_AEMODE = 0xC8BC, //1, changes from change-config and refresh
  REG_CAM_AET_EXPOSURE_TIME_MS = 0xC8C0, //2, changes during vertical blanking
  REG_CAM_SENSOR_CONTROL_REQUEST = 0xC842, //1, changes during vertical blanking
  //Register document says bits 5-4, 2-0 are reserved and set to 1
  REG_AE_TRACK_ALGO = 0xA804, //2, changes during vertical blanking
  //pertinent if luma calc algo. is disabled with bit 3 of REG_AE_TRACK_ALGO = 0
  REG_AE_TRACK_AVG_LOG_Y_TARGET = 0xA806, //2, updates during vertical blanking
  REG_CAM_AWB_MODE = 0xC97D, //1, b3 changes during vertical blanking and  b2-0change-config
  REG_CAM_AWB_COLOR_TEMPERATURE = 0xC928, //2, changes during vertical blanking
  REG_CAM_AET_FLICKER_FREQ_HZ = 0xC8D1, //1, changes after change-config
  REG_CAM_PGA_PGA_CONTROL = 0xCA80, //bit 1 needs change-config, bit 0 needs vertical blanking
  REG_RULE_AE_WEIGHT_TABLE_0_0 = 0xA40A, //1, changes during vertical blanking
  REG_CAM_TEMP_CUR = 0xCAAF,
  REG_CAM_TEMP_MIN = 0xCAB0,
  REG_CAM_TEMP_MAX = 0xCAB1,
  REG_CMD_PARAMS_POOL_0 = 0xFC00,
  REG_CMD_PARAMS_POOL_1 = 0xFC02,
  REG_CMD_PARAMS_POOL_2 = 0xFC04,
  REG_CMD_PARAMS_POOL_3 = 0xFC06,
  REG_CMD_PARAMS_POOL_4 = 0xFC08,
  REG_CMD_PARAMS_POOL_5 = 0xFC0A,
  REG_CMD_PARAMS_POOL_6 = 0xFC0C,
  REG_CMD_PARAMS_POOL_7 = 0xFC0E,
};
#define REG_CMD_DOORBELL_MASK (0x8000)


struct __attribute__((__packed__))  flash_query_resp {
	u16 deviceSizeKb;
	u16 blockSizeKb;
	u16 pageSize;
	u8 manuID;
	u8 deviceID1;
	u8 deviceID2;
	u8 padding;
	u8 driverID;
	u8 logicalID;
};

enum LENS_TYPE {
	LENS_M12_2_6mm = 0, // 2.6mm
	LENS_M12_4_2mm, // 4.2mm
	LENS_M12_6mm,     // 6mm
	LENS_M12_3_6mm,     // 3.6mm
};

struct __attribute__((__packed__)) fw_header {
	u32 header_const; // a constant to identify the file
	u32 time_stamp;  // the time stamp
	s16 hw_version;  // hardware version
	s16 fw_version;   // firmware version
	s16 lens_type;  // lens_type
	s16 reserved2;
	u32 serial_number;    // serial number
	s32 reserved;    // reserved
	u32 bin_size;      // bin file file size
	u32 bin_crc;        // bin file system CRC
	u32 header_crc;  // header CRC
};
static const u32 FLASH_HEADER_MAGIC = 0x68A3F297;
static const u32 FLASH_HEADER_ADDR = (32*1024);


#define INIT_BUF_MAX (256)
enum SENSOR_SYSFS_STATUS {
	SSS_SENSOR_SET_CMD,
	SSS_IDLE,
};

enum ap0100_m034_frame_mode {
	MODE_720P_30 = 0,
	MODE_960P_22_5 = 1,
};


typedef struct __attribute__((__packed__)) cam_param {
	u32 exp_time;
	u32 luma_target;
	u32 color_temperature;
	u32 ae_weight_table[25];
} cam_param_t;

struct ap0100_m034_data {
	struct i2c_client *client;
	struct device *dev;
    int ap0100_in_wdr_mode;
    unsigned char mode_init_buf[INIT_BUF_MAX];
    int mode_init_buf_count; // how many modes to do in sequence
    enum SENSOR_SYSFS_STATUS cur_sss_status;
    int sensor_read_len;
    bool already_reset;
    u8  read_buf[256];
	bool use_test_input;
	enum ap0100_m034_frame_mode mode;
	struct v4l2_fract pending_fi;
	struct v4l2_subdev	subdev;
	bool operational;
	unsigned error_count;
	struct mutex lock;
	bool unavailable;
	struct fw_header flash_header;
	int fw_status;
	cam_param_t cam_param_s;
};

static int _ap0100_m034_probe(struct ap0100_m034_data* data);
static int _ap0100_m034_remove(struct ap0100_m034_data *data);
static int _ap0100_reset(struct ap0100_m034_data* data);
static int _ap0100_unlock_flash(struct ap0100_m034_data *data);

static struct ap0100_m034_data *to_ap0100_m034_from_i2c(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ap0100_m034_data, subdev);
}

static struct ap0100_m034_data *to_ap0100_m034_from_v4l2(const struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return to_ap0100_m034_from_i2c(client);
}

static struct ap0100_m034_data *to_ap0100_m034_from_dev(const struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	return container_of(sd, struct ap0100_m034_data, subdev);
}

#define SENSOR_SET_CMD_UUID		"23917fa2-bc3a-11e3-a5e2-0800200c9a66"

enum ap0100_set_mode {
	MODE_NO_FLIP_NO_MIRROR 	= 0,
	MODE_FLIP_IMAGE 	= 1,
	MODE_MIRROR_IMAGE	= 2,
	MODE_FLIP_N_MIRROR	= 3,
	MODE_WDR_MODE		= 4,
	MODE_SDR_MODE		= 5,
	MODE_AE_OFF			= 6,
	MODE_AE_ON			= 7,
	MODE_SET_EXP_TIME	= 8,
	MODE_SET_TARGET_LUMA = 9,
	MODE_AUTO_TARGET_LUMA = 10,
	MODE_SET_COLOR_TEMPERATURE = 11,
	MODE_SET_AE_WEIGHT_TABLE = 12,
	MODE_INDOOR_MODE_ON = 13,
	MODE_INDOOR_MODE_OFF = 14,
	MODE_FLICKER_50HZ = 15,
	MODE_FLICKER_60HZ = 16,
	MODE_PGA_ON = 17,
	MODE_PGA_OFF = 18,
	MODE_CNT 		= 19,
	MODE_INIT_PARAM_BUF = 0xFE,
	MODE_INIT_MODE_BUF	= 0xFF,
};

#define WRITE_IS_CMD 		(1 << 0)
#define WRITE_IS_UNCHECKED (1 << 1)
#define READ_IS_UNCHECKED true
struct ap0100_m034_reg_data {
    unsigned data_size;
    u16 reg_addr;
    unsigned data;
	unsigned flags;
};

struct ap0100_m034_frame_data {
	__u32 width;
	__u32 height;
	struct v4l2_fract fi;
};

struct ap0100_m034_frame_data ap0100_m034_frame_data[] =
{
	{.width = 1280, .height = 720, .fi = { .denominator = 30, .numerator = 1,}},
	{.width = 1280, .height = 960, .fi = { .denominator = 45, .numerator = 2,}},
};

static int _AM_try_read_data(struct i2c_client * client, u16 reg, u8* buf, int buf_len, bool unchecked)
{
	int err;
	u8 au8RegBuf[2] = {0};
	struct i2c_msg msgs[2];
	u8 check[buf_len];

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = au8RegBuf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = buf_len;
	msgs[1].buf = buf;

	err = i2c_transfer(client->adapter, msgs, 2);
	if(err < 0 || unchecked)
		return err;

	msgs[1].buf = check;
	err = i2c_transfer(client->adapter, msgs, 2);
	if(err < 0)
		return err;
	if(!memcmp(buf, check, buf_len))
		return 0;
	return -EAGAIN;
}

static int _AM_try_read_reg(struct i2c_client * client, u16 reg, unsigned *val, int data_size, bool unchecked)
{
	int err;

	u8 buf[4] = {0};
	err = _AM_try_read_data(client, reg, buf, data_size, unchecked);

	switch(data_size){
	case 1:
		*val = buf[0];
		break;
	case 2:
		*val = ((buf[0] << 8) & 0xff00) | (buf[1] & 0x00ff);
		break;
	case 4:
		*val =  ((buf[0] << 24) & 0xff000000) |
				((buf[1] << 16) & 0x00ff0000) |
				((buf[2] << 8)  & 0x0000ff00) |
				 (buf[3]        & 0x000000ff);
		break;
	};
	if (err < 0) {
		dev_warn(&client->dev,"%s:read reg warn:reg=%x,val=%x,err=%d\n", __func__, reg, *val,
					err);
	}
	return err;
}

static int _AM_read_data(struct i2c_client * client, u16 reg, u8* buf, int buf_len, unsigned *error_count)
{
	int ret;
	int i;

	for(i = 0; i <= i2c_retries; i++) {
		ret = _AM_try_read_data(client, reg, buf, buf_len, !READ_IS_UNCHECKED);
		if(ret >= 0)
			return ret;
	}
	dev_err(&client->dev, "%s: too many retries, got %d", __func__, ret);
	if(error_count)
		*error_count +=1;

	return ret;
}


static int _AM_read_reg_maybe_unchecked(struct i2c_client * client, u16 reg, unsigned *val, int data_size, unsigned *error_count, bool unchecked)
{
	int ret;
	int i;

	if(data_size != 1 && data_size != 2 && data_size != 4) {
		dev_err(&client->dev,"Unsupported data length");
		return -EINVAL;
	}

	for(i = 0; i <= i2c_retries; i++) {
		ret = _AM_try_read_reg(client, reg, val, data_size, unchecked);
		if(ret >= 0)
			return ret;
	}
	dev_err(&client->dev, "%s: too many retries, got %d", __func__, ret);
	if(error_count)
		*error_count +=1;

	return ret;
}

static int _AM_read_reg(struct i2c_client * client, u16 reg, unsigned *val, int data_size, unsigned *error_count)
{
	return _AM_read_reg_maybe_unchecked(client, reg, val, data_size, error_count, !READ_IS_UNCHECKED);
}

static int _AM_read_reg_unchecked(struct i2c_client * client, u16 reg, unsigned *val, int data_size, unsigned *error_count)
{
	return _AM_read_reg_maybe_unchecked(client, reg, val, data_size, error_count, READ_IS_UNCHECKED);
}

static int _AM_try_write_data(struct i2c_client * client, u16 reg, const u8* buf, unsigned buf_len, bool unchecked)
{
	int err;
	u8 write_buf[2 + buf_len];
	u8 check[buf_len];

	write_buf[0] = reg >> 8;
	write_buf[1] = reg & 0xff;
	memcpy(&write_buf[2], buf, buf_len);

	err =  i2c_master_send(client, write_buf, sizeof(write_buf));
	if (err < 0) {
		dev_warn(&client->dev, "%s:write reg warn:reg=%x, err=%d\n",
			__func__, reg, err);
		return err;
	}
	if(unchecked)
		return err;

	if((err = _AM_read_data(client, reg, check, buf_len, NULL)) == 0 && memcmp(buf, check, buf_len)) {
		dev_warn(&client->dev,"%s:check reg warn:reg=%x\n",
			__func__, reg);
		return -EIO;
	}
	return err;
}

static void _AM_format_reg(u8* buf, unsigned val, int data_size)
{
	switch(data_size){
	case 1:
		buf[0] = val;
		break;
	case 2:
		buf[0] = val >> 8;
		buf[1] = val & 0xff;
		break;
	case 4:
		buf[0] = val >> 24;
		buf[1] = val >> 16;
		buf[2] = val >> 8;
		buf[3] = val & 0xff;
		break;
	};
}

static int _AM_try_write_reg(struct i2c_client * client, u16 reg, unsigned val, int data_size, bool unchecked)
{
	u8 buf[4];
	_AM_format_reg(buf, val, data_size);
	return _AM_try_write_data(client, reg, buf, data_size, unchecked);
}

static int _AM_write_reg_maybe_unchecked(struct i2c_client * client, u16 reg, unsigned val, int data_size, unsigned *error_count, bool unchecked)
{
	int i=0;
	int ret;

	if(data_size != 1 && data_size != 2 && data_size != 4) {
		dev_err(&client->dev,"Unsupported data length");
		return -EINVAL;
	}

	for(i = 0; i <= i2c_retries; i++) {
		ret = _AM_try_write_reg(client, reg, val, data_size, unchecked);
		if(ret >= 0)
			return ret;
	}
	dev_err(&client->dev, "%s: too many retries, got %d", __func__, ret);
	if(error_count)
		*error_count +=1;
	return ret;
}

static int _AM_write_reg(struct i2c_client * client, u16 reg, unsigned val, int data_size, unsigned *error_count)
{
	return _AM_write_reg_maybe_unchecked(client, reg, val, data_size, error_count, !WRITE_IS_UNCHECKED);
}

static int _AM_write_reg_unchecked(struct i2c_client * client, u16 reg, unsigned val, int data_size, unsigned *error_count)
{
	return _AM_write_reg_maybe_unchecked(client, reg, val, data_size, error_count, WRITE_IS_UNCHECKED);
}

static int _AM_write_data_maybe_unchecked(struct i2c_client * client, u16 reg, const u8 *buf, ssize_t buf_len, unsigned *error_count, bool unchecked)
{
	int i=0;
	int ret;

	for(i = 0; i <= i2c_retries; i++) {
		ret = _AM_try_write_data(client, reg, buf, buf_len, unchecked);
		if(ret >= 0)
			return ret;
	}
	dev_err(&client->dev, "%s: too many retries, got %d", __func__, ret);
	if(error_count)
		*error_count +=1;
	return ret;
}

static int _AM_send_command(struct i2c_client * client, u16 val, unsigned *error_count)
{
	static const int cmd_error_codes[0x11] = {
			 0, -ENOENT, -EINTR, -EIO,
			 -E2BIG, -EBADF, -EAGAIN, -ENOMEM,
			 -EACCES, -EBUSY, -EEXIST, -ENODEV,
			 -EINVAL, -ENOSPC, -ERANGE, -ENOSYS,
			 -EALREADY
	};
	const int DOORBELL_RETRIES = i2c_retries*8;
	int err;
	unsigned ap0100_err;
	u8 au8Buf[4] = {0};
	int i;

	au8Buf[0] = REG_CMD >> 8;
	au8Buf[1] = REG_CMD & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	err = i2c_master_send(client, au8Buf, 4);
	if( err < 0) {
		dev_err(&client->dev, "%s: too many retries, got %d",__func__, err);
		if(error_count)
			*error_count += 1;
		return err;
	}
	for(i = 0; i < DOORBELL_RETRIES; i++) {
		err = _AM_read_reg_unchecked(client, REG_CMD, &ap0100_err, 2, NULL);
		if(err >= 0 && !(ap0100_err & REG_CMD_DOORBELL_MASK) && ap0100_err < ARRAY_SIZE(cmd_error_codes))
			break;
		msleep(5);
	}
	if(err < 0) {
		*error_count += 1;
		return err;
	}

	if(ap0100_err & REG_CMD_DOORBELL_MASK) {
		dev_err(&client->dev,"doorbell never cleared\n");
		return -EBUSY;
	}

	if(ap0100_err < ARRAY_SIZE(cmd_error_codes))
		return cmd_error_codes[ap0100_err];

	return -EFAULT;
}

static int _ap0100_m034_cmd_status(struct i2c_client * client, unsigned *error_count)
{
	const int RETRIES = i2c_retries;
	int i;
	int err;
	unsigned val;

	for (i=0; i < RETRIES; i++) {
		err = _AM_read_reg(client, REG_CMD, &val, 2, error_count);
		if(err < 0)
			return err;
		if (val == 0)
			return 0;
		msleep(1);
	}
	return -EBUSY;
}

static int _ap0100_handle_adjacent_registers(struct i2c_client * client,
		const struct ap0100_m034_reg_data *ap0100_reg, const ssize_t reg_size, unsigned *error_count)
{
	int i;
	u8 flags;
	u8 buf[4*reg_size];
	u8 *buf_ptr = buf;
	unsigned start_addr;
	unsigned cur_addr;

	if(reg_size == 0)
		return -EINVAL;

	start_addr = ap0100_reg[0].reg_addr;
	cur_addr = start_addr;

	flags = ap0100_reg[0].flags & WRITE_IS_UNCHECKED;
	for (i=0; i < reg_size; i++) {
		if(ap0100_reg[i].flags != flags) {
				dev_err(&client->dev, "%s:no flags in adjacent regs\n",
						__func__);
				return -EINVAL;
		} else {
			if(ap0100_reg[i].data_size != 1 &&
				ap0100_reg[i].data_size != 2 &&
				ap0100_reg[i].data_size != 4)
			{
				dev_err(&client->dev, "%s:bad data size %d\n",
						__func__, ap0100_reg[i].data_size);
				return -EINVAL;
			}

			if(ap0100_reg[i].reg_addr != cur_addr) {
				dev_err(&client->dev, "%s:bad address %x\n",
						__func__, ap0100_reg[i].reg_addr);
				return -EINVAL;
			}
			_AM_format_reg(buf_ptr, ap0100_reg[i].data, ap0100_reg[i].data_size);
			buf_ptr += ap0100_reg[i].data_size;
			cur_addr +=  ap0100_reg[i].data_size;
		}
	}

	return _AM_write_data_maybe_unchecked(client, start_addr, buf, (cur_addr - start_addr), error_count, flags);
}

static int _ap0100_handle_registers(struct i2c_client * client,
		const struct ap0100_m034_reg_data *ap0100_reg, const int reg_size, unsigned *error_count)
{
	int i;
	int err;
	for (i=0; i < reg_size; i++) {
		if(ap0100_reg[i].flags & WRITE_IS_CMD) {
			err = _AM_send_command(client, ap0100_reg[i].data, error_count);
			if(err < 0) {
				dev_err(&client->dev, "%s:command error: err=%d\n",
						__func__, err);
			}
		} else if (ap0100_reg[i].flags & WRITE_IS_UNCHECKED) {
			err =_AM_write_reg_unchecked(client,ap0100_reg[i].reg_addr,
						ap0100_reg[i].data, ap0100_reg[i].data_size, error_count);
		} else {
			err = _AM_write_reg(client, ap0100_reg[i].reg_addr,
					ap0100_reg[i].data, ap0100_reg[i].data_size, error_count);
		}

		if (err < 0)
			return err;
	}
	return 0;
}

static int _ap0100_m034_test_mode(struct ap0100_m034_data* data)
{
	struct i2c_client *client = data->client;

	static const struct ap0100_m034_reg_data ap0100_normal_mode_reg[] = {
			{2, REG_LOGICAL_ADDR_ACCESS, 0xC88C},
			{1, REG_CAM_MODE_SELECT, 0x00},
			{2, REG_CMD_PARAMS_POOL_0, 0x2800},
			{0xC2, REG_CMD, CMD_SYS_SET_STATE, WRITE_IS_CMD},
	};

	static const struct ap0100_m034_reg_data ap0100_test_mode_reg[] = {
			{2, REG_LOGICAL_ADDR_ACCESS, 0xC88F},
			{1, REG_CAM_MODE_TEST_PATTERN_SELECT, 0x02},
			{1, REG_CAM_MODE_SELECT, 0x02},
			{4, REG_CAM_MODE_TEST_PATTERN_RED, 0x000000},
			{4, REG_CAM_MODE_TEST_PATTERN_GREEN, 0x000000},
			{4, REG_CAM_MODE_TEST_PATTERN_BLUE, 0x000000},
			{2, REG_CMD_PARAMS_POOL_0, 0x2800},
			{0xC2, REG_CMD, CMD_SYS_SET_STATE, WRITE_IS_CMD},
	};

	int err;

	if (data->use_test_input) {
		dev_dbg(&client->dev, "test pattern mode");
		err = _ap0100_handle_registers(client,
				ap0100_test_mode_reg, ARRAY_SIZE(ap0100_test_mode_reg), &data->error_count);
	} else {
		dev_dbg(&client->dev, "normal mode");
		err = _ap0100_handle_registers(client,
				ap0100_normal_mode_reg, ARRAY_SIZE(ap0100_normal_mode_reg), &data->error_count);
	}

	if(err < 0)
		return err;

	msleep(50);
	return _ap0100_m034_cmd_status(client, &data->error_count);
}

static int _ap0100_m034_sensor_init(struct ap0100_m034_data* data, enum ap0100_m034_frame_mode new_mode)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int ret=0;
	int err;
	struct ap0100_m034_reg_data ap0100_cmd_reg[] = {
			{0x02, REG_LOGICAL_ADDR_ACCESS, 0x7C00 	},// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
			{0x02, REG_CMD_PARAMS_POOL_0, 0x0000 	},// CMD_HANDLER_PARAMS_POOL_0
			{0xC2, REG_CMD, CMD_INVOKE_CMD_SEQ, WRITE_IS_CMD},// COMMAND_REGISTER
	};

	ret = _ap0100_m034_cmd_status(client, &data->error_count);
	if(ret < 0)
		return ret;

	switch (new_mode) {
		case MODE_720P_30:
			ap0100_cmd_reg[1].data = STORED_CMD_720P_84MHz_yuv8_WDR;
			dev_dbg(dev, "1280x720 mode");
			break;
		case MODE_960P_22_5:
			ap0100_cmd_reg[1].data = STORED_CMD_960P_84MHz_yuv8_WDR;
			dev_dbg(dev, "1280x960 mode");
			break;
		default:
			dev_err(dev, "Bad mode");
			return -EINVAL;
	}

	err = _ap0100_handle_registers(client, ap0100_cmd_reg, ARRAY_SIZE(ap0100_cmd_reg), &data->error_count);
	if(err < 0)
		return err;

	msleep(50);
	ret = _ap0100_m034_cmd_status(client, &data->error_count);
	if(ret < 0) {
		dev_err(dev, "init failed!!");
		return ret;
	}

	ret = _ap0100_m034_test_mode(data);
	if (ret >= 0) {
		dev_dbg(dev, "init OK, mode = %d", data->mode);
		data->mode = new_mode;
		data->ap0100_in_wdr_mode = 1;
	} else {
		dev_err(dev, "init failed!!");
	}
	return ret;
}

static int _ap0100_set_low_power(struct ap0100_m034_data *data)
{
	static const struct ap0100_m034_reg_data ap0100_power_regs[] = {
			{1, REG_CMD_PARAMS_POOL_0, 0x00},
			{1, REG_CMD_PARAMS_POOL_0+1, 0x1},
			{1, REG_CMD_PARAMS_POOL_1+0, 0x1},
			{1, REG_CMD_PARAMS_POOL_1+1, 0x1},
			{0xC2, REG_CMD, CMD_SYS_CFG_PWR, WRITE_IS_CMD},
			{2, REG_CMD_PARAMS_POOL_0, 0x4000},
			{0xC2, REG_CMD, CMD_SYS_SET_STATE, WRITE_IS_CMD},
	};
	return  _ap0100_handle_registers(data->client, ap0100_power_regs, ARRAY_SIZE(ap0100_power_regs), &data->error_count);
}

static int _ap0100_wait_for_flash(struct ap0100_m034_data *data, int wait_time_ms)
{
	int ret = -EINVAL;
	const int WAIT_MS = 10;
	for(; wait_time_ms > 0; wait_time_ms -= WAIT_MS) {
		ret  =_AM_send_command(data->client, CMD_FLASH_STATUS, &data->error_count);
		if(ret == 0)
			return 0;
		msleep(WAIT_MS);
	}
	return ret;
}

static int _ap0100_read_flash_inner(struct ap0100_m034_data *data, unsigned addr, unsigned length, u8 *buf)
{
	/*Set rest of registers to 0xFF in case this is translated as write and not read(?)*/
	struct ap0100_m034_reg_data read_flash_regs[] = {
			{4, REG_CMD_PARAMS_POOL_0, },
			{1, REG_CMD_PARAMS_POOL_2, },
			{1, REG_CMD_PARAMS_POOL_2+1, 0xFF},
			{2, REG_CMD_PARAMS_POOL_3, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_4, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_5, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_6, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_7, 0xFFFF},
	};
	int ret, i;

	if(length > 16)
		return -EINVAL;

	read_flash_regs[0].data = addr;
	read_flash_regs[1].data = length;

	ret = -EINVAL;
    for(i = 0; i < i2c_retries && ret < 0; i++) {
		ret = _ap0100_handle_adjacent_registers(data->client, read_flash_regs, ARRAY_SIZE(read_flash_regs), &data->error_count);
		if(ret < 0)
			continue;

		ret =  _AM_send_command(data->client, CMD_FLASH_READ, &data->error_count);
		if(ret < 0)
			continue;
		ret = _ap0100_wait_for_flash(data, 100);
	}
	if(ret < 0) {
		dev_err(data->dev, "%s: could not read flash:%d\n", __func__, ret);
		return ret;
	}
	return _AM_read_data(data->client, REG_CMD_PARAMS_POOL_0, buf, length, &data->error_count);
}

static int _ap0100_read_flash(struct ap0100_m034_data *data, unsigned addr, unsigned length, u8 *buf)
{
	static const unsigned READ_FLASH_BLOCK = 8;
	int ret;
	if(length == 0)
		return -EINVAL;

	for(; length > READ_FLASH_BLOCK; length -= READ_FLASH_BLOCK) {
		ret  = _ap0100_read_flash_inner(data, addr, READ_FLASH_BLOCK, buf);
		if(ret < 0) {
			dev_err(data->dev, "%s: _ap0100_read_flash_inner() faild (%d)\n", __func__, ret);
			return ret;
		}
		addr += READ_FLASH_BLOCK;
		buf += READ_FLASH_BLOCK;
	}
	if(length > 0)
		ret  = _ap0100_read_flash_inner(data, addr, length, buf);
	return ret;
}

static int _ap0100_validate_flash_inner(struct ap0100_m034_data *data, unsigned addr, unsigned length, const u8 *buf)
{
	u8 check[length];
	int ret = -EINVAL;
	int i;

	for(i = 0; i < i2c_retries; i++) {
		ret  = _ap0100_read_flash_inner(data, addr, length, check);
		if(ret < 0)
			continue;
		if(!memcmp(check, buf, length))
			return 0;
		ret = -EIO;
	}
	dev_err(data->dev,"Validate failure at %x:%d\n", addr, ret);
	return ret;
}

static int _ap0100_validate_flash(struct ap0100_m034_data *data, unsigned addr, unsigned length, const u8 *buf)
{
	static const unsigned READ_FLASH_BLOCK = 8;
	int ret;

	const unsigned total_to_read = length;
	const unsigned start_addr = addr;
	unsigned percent_done = 0;

	if(length == 0)
		return -EINVAL;

	for(; length > READ_FLASH_BLOCK; length -= READ_FLASH_BLOCK) {
		unsigned current_complete = (total_to_read - length)*100;
		if(current_complete/total_to_read > percent_done) {
			percent_done = current_complete/total_to_read;
			dev_err(data->dev,"Validate %u(%u):%u%%\n", start_addr, total_to_read, percent_done);
		}

		ret = _ap0100_validate_flash_inner(data, addr, READ_FLASH_BLOCK, buf);
		if(ret < 0) {
			return ret;
		}
		addr += READ_FLASH_BLOCK;
		buf += READ_FLASH_BLOCK;
	}
	if(length == 0)
		return 0;

	return _ap0100_validate_flash_inner(data, addr, length, buf);
}

/*Validates writes to registers were correct but does not validate flash.  Flash validation is performed
	separately.*/
static int _ap0100_write_flash_inner(struct ap0100_m034_data *data, unsigned addr, unsigned length, const u8 *buf)
{
	struct ap0100_m034_reg_data write_flash_regs[] = {
			{4, REG_CMD_PARAMS_POOL_0, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_2, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_2+1, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_3, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_3+1, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_4, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_4+1, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_5, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_5+1, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_6, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_6+1, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_7, 0, WRITE_IS_UNCHECKED},
			{1, REG_CMD_PARAMS_POOL_7+1, 0, WRITE_IS_UNCHECKED},
	};
	static const int MAX_DATA_LENGTH = 16 - 5;
	int i;
	int ret = -EINVAL;
	u8 ff[length];

	if(length > MAX_DATA_LENGTH)
		return -EINVAL;

	/*Skip all FFs*/
	memset(ff,0xff,sizeof(ff));
	if(!memcmp(buf, ff, length))
		return 0;

	write_flash_regs[0].data = addr;
	write_flash_regs[1].data = length;
	for(i = 0; i < length; i++) {
		write_flash_regs[2+i].data = buf[i];
	}

	for(i = 0; i < i2c_retries; i++) {
		ret = _ap0100_handle_adjacent_registers(data->client, write_flash_regs, 2 + length, &data->error_count);
		if(ret < 0) {
			continue;
		}
		ret =  _AM_send_command(data->client, CMD_FLASH_WRITE, &data->error_count);
		if(ret < 0) {
			msleep(50);
			continue;
		}

		ret = _ap0100_wait_for_flash(data, 100);
		if(ret == 0) {
			return 0;
		}
	}
	ret = (ret < 0) ? ret : -EIO;
	dev_err(data->dev, "Bad flash write at %u:%d\n", addr, ret);
	return ret;
}

/*WARNING: it seems like having too much delay in between write commands may lead to some sort
 * of time-out in the camera firmware which leads to statuses of EBUSY, ENOSYS, EACCES. Given
 * only 11 bytes can be written at once via the command IF, and  the EEPROM data sheet
 * recommends writing a full page at once, it is quite possible that there is some fancy logic
 * in the firmware which buffers writes up to a page size or until a certain number of milliseconds
 * has elapsed.
 * This function and the surrounding code has been cut down to issue the minimum amount of i2c
 * accesses. Validation of the flash happens in a separate phase.
 */
static int _ap0100_write_flash(struct ap0100_m034_data *data, unsigned addr, unsigned length, const u8 *buf)
{
	/*Maximize data per write in order to minimize number of transfers total*/
	static const unsigned MAX_BYTES_PER_WRITE = 11;
	static const unsigned FLASH_PAGE_SHIFT = 8;
	static const unsigned FLASH_PAGE_SIZE = 1 << FLASH_PAGE_SHIFT;

	const unsigned total_to_write = length;
	const unsigned start_addr = addr;
	unsigned percent_done = 0;

	unsigned to_write;
	unsigned end_addr;

	int ret = 0;

	if(length == 0)
		return -EINVAL;

	do {
		unsigned current_complete = (total_to_write - length)*100;
		if(current_complete/total_to_write > percent_done) {
			percent_done = current_complete/total_to_write;
			dev_err(data->dev,"Write %u(%u):%u%%\n", start_addr, total_to_write, percent_done);
		}

		to_write = (length > MAX_BYTES_PER_WRITE) ? MAX_BYTES_PER_WRITE : length;

		/*Cannot cross page boundaries*/
		end_addr = addr + to_write - 1;
		if((end_addr >> FLASH_PAGE_SHIFT) != (addr >> FLASH_PAGE_SHIFT)) {
			to_write = FLASH_PAGE_SIZE - (addr & (FLASH_PAGE_SIZE-1));
		}

		ret = _ap0100_write_flash_inner(data, addr, to_write, buf);
		if(ret < 0)
			return ret;
		addr += to_write;
		buf += to_write;
		length -= to_write;
	} while(length > 0);

	return ret;
}

static int _ap0100_lock_flash(struct ap0100_m034_data *data)
{
	int ret = -EINVAL;
	int tmpret = -EINVAL;
	int i, j;
	static const struct ap0100_m034_reg_data config_dev_regs[] = {
			{2, REG_CMD_PARAMS_POOL_0, 0x0400},
			{2, REG_CMD_PARAMS_POOL_1, 0x0318},
			{2, REG_CMD_PARAMS_POOL_2, 0x0001},
			{2, REG_CMD_PARAMS_POOL_3, 0x0000},
	};

	static const struct ap0100_m034_reg_data query_regs[] = {
			{2, REG_CMD_PARAMS_POOL_0, 0x0000},
			{2, REG_CMD_PARAMS_POOL_1, 0x0000},
			{2, REG_CMD_PARAMS_POOL_2, 0x0000},
			{2, REG_CMD_PARAMS_POOL_3, 0x0000},
			{2, REG_CMD_PARAMS_POOL_4, 0x0000},
			{2, REG_CMD_PARAMS_POOL_5, 0x0000},
			{2, REG_CMD_PARAMS_POOL_6, 0x0000},
			{2, REG_CMD_PARAMS_POOL_7, 0x0000},
	};
	struct flash_query_resp query_resp;
	u8 buf[sizeof(struct flash_query_resp)];

	for(i = 0; i < i2c_retries; i++) {
		ret = _AM_send_command(data->client, CMD_FLASH_GET_LOCK, &data->error_count);
		/*compiler complains if tmpret is not set here*/
		if(ret >= 0) {
			tmpret = _AM_send_command(data->client, CMD_FLASH_LOCK_STATUS, &data->error_count);
		} else {
			tmpret = -EINVAL;
		}
		if(ret >= 0 && tmpret >= 0)
			break;
		msleep(5);
	}

	if(ret < 0) {
		dev_err(data->dev, "%s:Unable to get lock:%d\n", __func__, ret);
		return ret;
	}

	if(tmpret < 0) {
		dev_err(data->dev, "%s:Unable to get lock status:%d\n", __func__, ret);
		ret = tmpret;
		goto out;
	}

	ret = -EINVAL;
	for(i = 0; i < i2c_retries && ret < 0; i++) {
		ret = _ap0100_handle_adjacent_registers(data->client,
				config_dev_regs, ARRAY_SIZE(config_dev_regs), &data->error_count);
		if(ret < 0) {
			continue;
		}
		ret = _AM_send_command(data->client, CMD_FLASH_CONFIG_DEV, &data->error_count);
	}
	if(ret < 0) {
		dev_err(data->dev, "%s: unable to send flash config dev:%d\n", __func__, ret);
		goto out;
	}

	ret = -EINVAL;
	for(i = 0 ; i < i2c_retries && ret < 0; i++) {
		ret = _ap0100_handle_adjacent_registers(data->client,
			query_regs, ARRAY_SIZE(query_regs), &data->error_count);
		if(ret < 0) {
			continue;
		}
		ret = _AM_send_command(data->client, CMD_FLASH_QUERY_DEV, &data->error_count);
		if(ret < 0)
			continue;
		ret = _ap0100_wait_for_flash(data, 100);

		memset(&query_resp, 0, sizeof(query_resp));
		for(j = 0; j < i2c_retries && query_resp.deviceSizeKb != 64; i++) {
			ret = _AM_read_data(data->client, REG_CMD_PARAMS_POOL_0, buf, sizeof(buf), NULL);
			if(ret < 0) {
				dev_err(data->dev, "Unable to read flash config:%d\n",ret);
				goto out;
			}
			memcpy(&query_resp, buf, sizeof(buf));
			query_resp.deviceSizeKb = buf[0] << 8 | buf[1];
			query_resp.blockSizeKb = buf[2] << 8 | buf[3];
			query_resp.pageSize = buf[4] << 8 | buf[5];
		}
	}

	if(ret < 0) {
		dev_err(data->dev, "%s:Unable to query dev:%d\n", __func__, ret);
		goto out;
	}

	if(query_resp.deviceSizeKb != 64 ) {
		dev_err(data->dev, "Bad flash size KB:%d, expected 64\n", query_resp.deviceSizeKb);
		ret = -EIO;
		goto out;
	}

	return 0;

out:
	tmpret = _ap0100_unlock_flash(data);
	if(tmpret < 0)
		ret = tmpret;
	return ret;
}
static int _ap0100_unlock_flash(struct ap0100_m034_data *data)
{
	int ret, i;
	ret = -EINVAL;
	for(i = 0; i < i2c_retries && (ret < 0 && ret != -EBUSY); i++)
		ret = _AM_send_command(data->client, CMD_FLASH_RELEASE_LOCK, &data->error_count);

	if(ret != -EBUSY && ret != 0) {
		dev_err(data->dev, "%s: unable to release lock:%d\n", __func__, ret);
	} else {
		ret = 0;
	}
	return ret;
}


static int _ap0100_validate_fw_header(struct ap0100_m034_data *data, struct fw_header *header)
{
	u32 crc;

	if(header->header_const != FLASH_HEADER_MAGIC) {
		dev_err(data->dev, "Header magic is %x\n", header->header_const);
		return -EINVAL;
	}

	crc = ~0;
	crc = crc32_le(crc, (unsigned char*)header, sizeof(*header)-sizeof(header->header_crc));
	crc = ~crc;

	if(crc != header->header_crc) {
		dev_err(data->dev, "header crc is %x, should be %x\n", crc, header->header_crc);
		return -EINVAL;
	}
	return 0;
}

/*Assumes little endian*/
static int _ap0100_read_flash_header(struct ap0100_m034_data *data, struct fw_header *header)
{
	int ret, tmpret;

	memset(header, 0, sizeof(*header));

	ret = _ap0100_lock_flash(data);
	if(ret < 0)
		return ret;

	ret = _ap0100_read_flash(data, FLASH_HEADER_ADDR, sizeof(*header), (u8*)header);
	tmpret = _ap0100_unlock_flash(data);
	if(tmpret < 0)
		return tmpret;
	if(ret < 0)
		return ret;

	return _ap0100_validate_fw_header(data, header);
}

static int _ap0100_erase_flash_block(struct ap0100_m034_data *data, u32 addr)
{
	int ret, i;
	u8 ff[16];
	struct ap0100_m034_reg_data erase_flash_regs[] = {
			{4, REG_CMD_PARAMS_POOL_0, },
			{0xC2, REG_CMD, CMD_FLASH_ERASE_BLOCK, WRITE_IS_CMD },
	};

	erase_flash_regs[0].data = addr;
	memset(ff,0xff,sizeof(ff));

	ret = -EINVAL;
	for(i = 0; i < i2c_retries && ret < 0; i++) {
		ret = _ap0100_handle_registers(data->client, erase_flash_regs, ARRAY_SIZE(erase_flash_regs), &data->error_count);
		if(ret < 0)
			continue;
		/*Erasing takes a really long time.*/
		ret = _ap0100_wait_for_flash(data, 10000);
		if(ret < 0)
			continue;
		/*Verify first 16 bytes are really blank*/
		ret  = _ap0100_validate_flash(data, addr, sizeof(ff), ff);
	}
	if(ret < 0) {
		dev_err(data->dev, "%s: could not erase flash at %u:%d\n", __func__, addr, ret);
		return ret;
	}

	return 0;
}

static int _ap0100_wait_for_ccmgr(struct ap0100_m034_data *data)
{
	int i;
	int ret = -EINVAL;
	const unsigned RETRIES = i2c_retries;
	for(i = 0; i < RETRIES; i++) {
		ret  =_AM_send_command(data->client, CMD_CCMGR_STATUS, &data->error_count);
		if(ret == 0)
			return 0;
		msleep(100);
	}
	return ret;
}
static int _ap0100_read_ccmgr(struct ap0100_m034_data *data, unsigned addr, unsigned length, u8 *buf)
{
	/*Set rest of registers to 0xFF in case this is translated as write and not read(?)*/
	struct ap0100_m034_reg_data read_cci_regs[] = {
			{2, REG_CMD_PARAMS_POOL_0, },
			{1, REG_CMD_PARAMS_POOL_1, },
			{1, REG_CMD_PARAMS_POOL_1+1,0xFF},
			{2, REG_CMD_PARAMS_POOL_2, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_3, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_4, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_5, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_6, 0xFFFF},
			{2, REG_CMD_PARAMS_POOL_7, 0xFFFF},
	};
	int ret, i;

	if(length > 16)
		return -EINVAL;

	read_cci_regs[0].data = addr;
	read_cci_regs[1].data = length;

	ret = -EINVAL;
    for(i = 0; i < i2c_retries && ret < 0; i++) {
		ret = _ap0100_handle_adjacent_registers(data->client, read_cci_regs, ARRAY_SIZE(read_cci_regs), &data->error_count);
		if(ret < 0)
			continue;

		ret =  _AM_send_command(data->client, CMD_CCMGR_READ, &data->error_count);
		if(ret < 0)
			continue;
		ret = _ap0100_wait_for_ccmgr(data);
	}
	if(ret < 0) {
		dev_err(data->dev, "%s: could not read cci:%d\n", __func__, ret);
		return ret;
	}
	return _AM_read_data(data->client, REG_CMD_PARAMS_POOL_0, buf, length, &data->error_count);
}

static int _ap0100_unlock_ccmgr(struct ap0100_m034_data *data);
static int _ap0100_lock_ccmgr(struct ap0100_m034_data *data)
{
	int ret = -EINVAL;
	int tmpret = -EINVAL;
	int i;

	for(i = 0; i < i2c_retries; i++) {
		ret = _AM_send_command(data->client, CMD_CCMGR_GET_LOCK, &data->error_count);
		/*compiler complains if tmpret is not set here*/
		if(ret >= 0) {
			tmpret = _AM_send_command(data->client, CMD_CCMGR_LOCK_STATUS, &data->error_count);
		} else {
			tmpret = -EINVAL;
		}
		if(ret >= 0 && tmpret >= 0)
			break;
		msleep(5);
	}

	if(ret < 0) {
		dev_err(data->dev, "%s:Unable to get lock:%d\n", __func__, ret);
		return ret;
	}

	if(tmpret < 0) {
		dev_err(data->dev, "%s:Unable to get lock status:%d\n", __func__, ret);
		ret = tmpret;
		goto out;
	}
	return 0;

out:
	tmpret = _ap0100_unlock_ccmgr(data);
	if(tmpret < 0)
		ret = tmpret;
	return ret;
}
static int _ap0100_unlock_ccmgr(struct ap0100_m034_data *data)
{
	int ret, i;
	ret = -EINVAL;
	for(i = 0; i < i2c_retries && (ret < 0 && ret != -EBUSY); i++)
		ret = _AM_send_command(data->client, CMD_CCMGR_RELEASE_LOCK, &data->error_count);

	if(ret != -EBUSY && ret != 0) {
		dev_err(data->dev, "%s: unable to release lock:%d\n", __func__, ret);
	} else {
		ret = 0;
	}
	return ret;
}

static int _ap0100_get_serial(struct ap0100_m034_data *data, u64 *serial)
{
	int ret, tmpret, i;
	u8 buf[8];

	*serial = 0;
	ret = _ap0100_lock_ccmgr(data);
	if(ret < 0)
		return ret;
	ret = _ap0100_read_ccmgr(data, 0x31f4, 8, buf);
	if(ret >= 0) {
		for(i = 0; i < 8; i++)
			*serial = (*serial << 8) | buf[i];
	}
	tmpret = _ap0100_unlock_ccmgr(data);
	if(tmpret < 0)
		ret = tmpret;
	return ret;
}

static int _ap0100_write_fw_bin(struct ap0100_m034_data *data, const u8 *bin_start, unsigned bin_size, struct fw_header *header)
{
	int ret;
	/*Erase header first because once we erase main flash the data is invalid.*/
	data->flash_header.header_const = 0;
	ret = _ap0100_erase_flash_block(data, FLASH_HEADER_ADDR);
	if(ret < 0)
		return ret;
	ret = _ap0100_erase_flash_block(data, 0);
	if(ret < 0)
		return ret;


	/*Not erasing flash block on write failure because this does not seem
 	to be possible to do anyway.*/
	ret = _ap0100_write_flash(data, 0, bin_size, bin_start);
	return ret;
}

static int _ap0100_write_fw_header(struct ap0100_m034_data *data, const u8 *bin_start, unsigned bin_size, struct fw_header *header)
{
	int ret;
	u32 crc;
	ret = _ap0100_validate_flash(data, 0, bin_size, bin_start);
	if(ret < 0)
		return ret;

	crc = ~0;
	crc = crc32_le(crc,(const u8*)header,sizeof(struct fw_header) - sizeof(header->header_crc));
	crc = ~crc;
	header->header_crc = crc;

	ret = _ap0100_validate_flash(data, FLASH_HEADER_ADDR, sizeof(struct fw_header), (const u8*)header);
	if(ret >= 0)
		return ret;
	data->flash_header.header_const = 0;
	ret = _ap0100_erase_flash_block(data, FLASH_HEADER_ADDR);
	if(ret < 0)
		return ret;
	ret = _ap0100_write_flash(data, FLASH_HEADER_ADDR, sizeof(struct fw_header), (const u8*)header);
	if(ret < 0)
		return ret;
	return _ap0100_validate_flash(data, FLASH_HEADER_ADDR, sizeof(struct fw_header), (const u8*)header);
}

static int _ap0100_validate_fw_in_flash(struct ap0100_m034_data *data, const u8 *bin_start, unsigned bin_size, struct fw_header *header)
{
	int ret;
	u32 crc;
	ret = _ap0100_validate_flash(data, 0, bin_size, bin_start);
	if(ret < 0)
		return ret;

	crc = ~0;
	crc = crc32_le(crc,(const u8*)header,sizeof(struct fw_header) - sizeof(header->header_crc));
	crc = ~crc;
	header->header_crc = crc;

	return _ap0100_validate_flash(data, FLASH_HEADER_ADDR, sizeof(struct fw_header), (const u8*)header);
}

static int _ap0100_handle_cbf(struct ap0100_m034_data *data, const char *name,
		int (*handler)(struct ap0100_m034_data *data, const u8 *bin_start, unsigned bin_size, struct fw_header *header))
{
	int ret, tmpret;
	u32 crc;
	const struct firmware *fw;
	const u8 *bin_start;
	unsigned bin_size;

	struct fw_header cbf_header;

	ret = request_firmware(&fw, name, data->dev);
	if(ret < 0) {
		dev_err(data->dev,"Loading cbf file %s failed\n", name);
		goto release_fw;
	}
	if(fw->size < sizeof(cbf_header)) {
		dev_err(data->dev,"cbf file %s too small\n", name);
		goto release_fw;
	}

	bin_start = fw->data + sizeof(cbf_header);
	bin_size = fw->size - sizeof(cbf_header);
	memcpy(&cbf_header, fw->data, sizeof(cbf_header));

	if(cbf_header.header_const != FLASH_HEADER_MAGIC) {
		dev_err(data->dev,"Invalid flash magic\n");
		ret = -EINVAL;
		goto release_fw;
	}

	if(cbf_header.bin_size != bin_size) {
		dev_err(data->dev, "Bad bin size: expected %d, got %d\n", cbf_header.bin_size, bin_size);
		ret = -EINVAL;
		goto release_fw;
	}
	crc = ~0;
	crc = crc32_le(crc, bin_start, bin_size);
	crc = ~crc;
	if(cbf_header.bin_crc != crc) {
		dev_err(data->dev, "Bad bin crc: expected %d, got %d\n", cbf_header.bin_crc, crc);
		ret = -EINVAL;
		goto release_fw;
	}

	ret = _ap0100_lock_flash(data);
	if(ret < 0)
		goto release_fw;

	ret = handler(data, bin_start, bin_size, &cbf_header);

	tmpret = _ap0100_unlock_flash(data);
	if(tmpret < 0)
		ret = tmpret;
release_fw:
	release_firmware(fw);
	return ret;
}

static int _ap0100_m034_sensor_set_mode(struct ap0100_m034_data *data, enum ap0100_set_mode mode)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	const char *modestr = NULL;
	int err, i;
#define HANDLE_REGS(_x_) do { \
		err = _ap0100_handle_registers(client, _x_, ARRAY_SIZE(_x_), &data->error_count); \
}while(0)

	struct ap0100_m034_reg_data ap0100_cmd_reg[] = {
			{0x02, REG_LOGICAL_ADDR_ACCESS, 0x7C00 	},// LOGICAL_ADDRESS_ACCESS [CAM_SYSCTL_PLL_CONTROL]
			{0x02, REG_CMD_PARAMS_POOL_0, 0x0000 	},// CMD_HANDLER_PARAMS_POOL_0
			{0xC2, REG_CMD, CMD_INVOKE_CMD_SEQ, WRITE_IS_CMD},// COMMAND_REGISTER
	};
	struct ap0100_m034_reg_data ae_reg[] = {
			{0x1, REG_CAM_AET_AEMODE, },
			{0x1, REG_CAM_AWB_MODE, },
			{0xC2, REG_CMD, CMD_SEQUENCER_REFRESH, WRITE_IS_CMD},
	};
	struct ap0100_m034_reg_data set_exposure_time_reg[] = {
			{0x2, REG_CAM_AET_EXPOSURE_TIME_MS},
			{0x1, REG_CAM_SENSOR_CONTROL_REQUEST, 0x01, WRITE_IS_UNCHECKED},
	};
	struct ap0100_m034_reg_data set_target_luma_reg[] = {
			{0x2, REG_AE_TRACK_ALGO, 0x0035},
			{0x2, REG_AE_TRACK_AVG_LOG_Y_TARGET},
	};
	struct ap0100_m034_reg_data auto_target_luma_reg[] = {
			{0x2, REG_AE_TRACK_ALGO, 0x003D},
	};
	struct ap0100_m034_reg_data set_color_temperature_reg[] = {
			{0x02, REG_CAM_AWB_COLOR_TEMPERATURE},
			{0x1, REG_CAM_SENSOR_CONTROL_REQUEST, 0x02, WRITE_IS_UNCHECKED},
	};
	struct ap0100_m034_reg_data indoor_mode_reg[] = {
			{0x1, REG_CAM_AET_AEMODE},
			{2, REG_CMD_PARAMS_POOL_0, 0x2800},
			{0xC2, REG_CMD, CMD_SYS_SET_STATE, WRITE_IS_CMD},
	};
	struct ap0100_m034_reg_data flicker_reg[] = {
			{0x1, REG_CAM_AET_FLICKER_FREQ_HZ},
			{2, REG_CMD_PARAMS_POOL_0, 0x2800},
			{0xC2, REG_CMD, CMD_SYS_SET_STATE, WRITE_IS_CMD},
	};
	struct ap0100_m034_reg_data pga_reg[] = {
			{0x02, REG_CAM_PGA_PGA_CONTROL},
	};

	err = _ap0100_m034_cmd_status(client, &data->error_count);
	if(err < 0)
		return err;

	switch (mode) {
		case MODE_NO_FLIP_NO_MIRROR:
			modestr = "MODE_NO_FLIP_NO_MIRROR";
			ap0100_cmd_reg[1].data = STORED_CMD_Image_Orientation_original;
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_FLIP_IMAGE:
			ap0100_cmd_reg[1].data = STORED_CMD_Flip;
			modestr = "MODE_FLIP_IMAGE";
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_MIRROR_IMAGE:
			ap0100_cmd_reg[1].data = STORED_CMD_Mirror;
			modestr = "MODE_MIRROR_IMAGE";
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_FLIP_N_MIRROR:
			ap0100_cmd_reg[1].data = STORED_CMD_Flip_And_Mirror;
			modestr = "MODE_FLIP_N_MIRROR";
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_WDR_MODE:
			ap0100_cmd_reg[1].data = data->mode + STORED_CMD_WDR_START;
			modestr = "MODE_WDR_MODE";
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_SDR_MODE:
			ap0100_cmd_reg[1].data = data->mode + STORED_CMD_SDR_START;
			modestr = "MODE_SDR_MODE";
			HANDLE_REGS(ap0100_cmd_reg);
			break;
		case MODE_AE_OFF:
			ae_reg[0].data = 0xA0;
			ae_reg[1].data = 0x02;
			modestr = "CMD_AE_OFF";
			HANDLE_REGS(ae_reg);
			break;
		case MODE_AE_ON:
			ae_reg[0].data = 0x00;
			ae_reg[1].data = 0x00;
			modestr = "CMD_AE_ON";
			HANDLE_REGS(ae_reg);
			break;
		case MODE_SET_EXP_TIME:
			set_exposure_time_reg[0].data = data->cam_param_s.exp_time;
			modestr = "CMD_SET_EXP_TIME";
			HANDLE_REGS(set_exposure_time_reg);
			break;
		case MODE_SET_TARGET_LUMA:
			set_target_luma_reg[1].data = data->cam_param_s.luma_target;
			modestr = "CMD_SET_TARGET_LUMA";
			HANDLE_REGS(set_target_luma_reg);
		break;
		case MODE_AUTO_TARGET_LUMA:
			modestr = "AUTO_TARGET_LUMA";
			HANDLE_REGS(auto_target_luma_reg);
			break;
		case MODE_SET_COLOR_TEMPERATURE:
			set_color_temperature_reg[0].data = data->cam_param_s.color_temperature;
			modestr = "CMD_SET_COLOR_TEMPERATURE";
			HANDLE_REGS(set_color_temperature_reg);
			break;
		case MODE_SET_AE_WEIGHT_TABLE:
			/*TODO: if this could be converted to an enum of different weights then
			 * the v4l2 subdev control interface would work here (see v4l2-controls.txt and
			 * sub-device drivers)
			 */
			err = 0;
			modestr = "CMD_SET_AE_WEIGHT_TABLE";
			for(i = 0; i < 25 && err >= 0; i++) {
				err = _AM_write_reg(client, REG_RULE_AE_WEIGHT_TABLE_0_0 + i,
						data->cam_param_s.ae_weight_table[i], 1, &data->error_count);
			}
			break;
		case MODE_INDOOR_MODE_ON:
			indoor_mode_reg[0].data = 0x1;
			modestr = "CMD_INDOOR_MODE_ON";
			HANDLE_REGS(indoor_mode_reg);
			break;
		case MODE_INDOOR_MODE_OFF:
			indoor_mode_reg[0].data = 0x0;
			modestr = "CMD_INDOOR_MODE_OFF";
			HANDLE_REGS(indoor_mode_reg);
			break;
		case MODE_FLICKER_50HZ:
			flicker_reg[0].data = 0x32;
			modestr = "CMD_FLICKER_50HZ";
			HANDLE_REGS(flicker_reg);
			break;
		case MODE_FLICKER_60HZ:
			flicker_reg[0].data = 0x3C;
			modestr = "CMD_FLICKER_60HZ";
			HANDLE_REGS(flicker_reg);
			break;
		case MODE_PGA_ON:
			pga_reg[0].data = 0x0001;
			modestr = "CMD_PGA_ON";
			HANDLE_REGS(pga_reg);
			break;
		case MODE_PGA_OFF:
			pga_reg[0].data = 0x0000;
			modestr = "CMD_PGA_OFF";
			HANDLE_REGS(pga_reg);
			break;
		default:
			dev_dbg(dev, "ap0100 set cmd : cmd not supported ");
			return -EINVAL;
	}

	if(err < 0)
		return err;

	msleep(50);
	err = _ap0100_m034_cmd_status(client, &data->error_count);

	if(err < 0) {
		dev_err(dev, "ap0100 set cmd failed!!");
	} else {
		dev_info(dev, "%s OK", modestr);
		// update mode based on WDR or SDR
		if ( mode == MODE_WDR_MODE) { // MODE_WDR_MODE
			data->ap0100_in_wdr_mode = 1;
		} else if ( mode == MODE_SDR_MODE) {
			data->ap0100_in_wdr_mode = 0;
		}
	}

	return err;
#undef HANDLE_REGS
}

static int _ap0100_m034_sensor_mode_init(struct ap0100_m034_data* data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int i;
	int err;
	
	if (data->mode_init_buf_count <= 0)
		return 0;

	for (i=0; i< data->mode_init_buf_count; i++) {
		int mode = (int)data->mode_init_buf[i];
		if(mode >= MODE_CNT) {
			dev_err(dev, "%s:Invalid command %d", __func__, mode);
			return -EINVAL;
		}
		dev_dbg(dev, "sensor set mode: %d", mode);
		err = _ap0100_m034_sensor_set_mode(data,(enum ap0100_set_mode)mode);
		if (err < 0)
			return err;
	}
	return 0;
}

static void _ap0100_m034_I2C_test(struct i2c_client * client, unsigned cycles)
{
	struct device *dev = &client->dev;
	static const u16 reg = 0xCA90;  // a register for testing
	unsigned val, w_val;
	int w_retry_cnt, r_retry_cnt, check_cnt;
	int i, j, k;
	int fail_cnt;
	int ret=0;
	int max_retry_cnt = 0;
	int total_transactions = 0;

	dev_err(dev, "AP0100  i2c test running ......");

	w_retry_cnt = 0;
	r_retry_cnt = 0;
	check_cnt = 0;
	fail_cnt = 0;
	for (i=0; i< cycles; i++) {
		if ( i % (cycles / 100) == 0) {
			dev_err(dev, "test in progress : %d%%, w_retry_cnt = %d, r_retry_cnt = %d, check_cnt=%d", \
				i / (cycles / 100) , w_retry_cnt, r_retry_cnt, check_cnt);
			dev_err(dev, "                         fail_cnt = %d",  \
				fail_cnt);
		}

		w_val = (i << 1) & 0x0FFE;

		ret = -1;

		for(j = 0; j <= i2c_retries && (ret < 0 || w_val != val); j++) {
			u8 buf[2 + 4];
			buf[0] = reg >> 8;
			buf[1] = reg & 0xff;
			buf[2] = w_val >> 8;
			buf[3] = w_val & 0xff;
			ret = i2c_master_send(client, buf, 4);
			total_transactions++;
			if(ret < 0) {
				dev_warn(&client->dev,"%s:write reg warn:reg=%x,val=%x\n",
						__func__, reg, w_val);
				w_retry_cnt++;
				continue;
			}

			ret = -1;
			for(k = 0; k < i2c_retries && ret < 0; k++) {
				ret = _AM_try_read_reg(client, reg, &val, 2, !READ_IS_UNCHECKED);
				total_transactions++;
				if(ret < 0) {
					r_retry_cnt++;
				}
			}
			if(ret >= 0 && val != w_val) {
				dev_warn(&client->dev,"%s:check reg warn:reg=%x,val=%x,check=%x\n",
						__func__, reg, w_val, val);
				check_cnt++;
			}
		}
		if (ret < 0 || w_val != val) {
			fail_cnt++;
			dev_err(dev, "write failed: i = %d, write = 0x%x, w_retry_cnt=%d, r_retry_cnt=%d, check_cnt=%d\n",
					i, w_val, w_retry_cnt, r_retry_cnt, check_cnt);
		} else if(j > max_retry_cnt) {
			max_retry_cnt = j;
		}
	}

	val = 0x0500; // default value
	_AM_write_reg(client, reg, val, 2, NULL);

	dev_err(dev, "ap0100 i2c test %s !, total test count = %d, fail_cnt=%d, w_retry_cnt = %d, "
			"r_retry_cnt = %d, check_cnt = %d, max retries=%d, total transactions=%d\n",
				fail_cnt == 0 ? "passed" : "failed", cycles, fail_cnt, w_retry_cnt,
						r_retry_cnt, check_cnt, max_retry_cnt,total_transactions);
}

static ssize_t ap0100_m034_i2c_test(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data* data = to_ap0100_m034_from_dev(dev);
	unsigned val;
	if(kstrtouint(buf, 10, &val) || val < 100) {
		dev_err(dev,"Must supply test cycles > 100 in decimal.");
		return count;
	}
	mutex_lock(&data->lock);
	_ap0100_m034_I2C_test(data->client, val);
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(ap0100_m034_i2c_test, 0222, NULL, (void *)ap0100_m034_i2c_test);


static int _ap0100_m034_sensor_update_init_buf(struct ap0100_m034_data *data,
		unsigned char *buf, int count)
{
	if (count > INIT_BUF_MAX) {
		dev_err(&data->client->dev,"length greater than %d", INIT_BUF_MAX);
		return -EINVAL;
	}
	memcpy(data->mode_init_buf, buf, count);
	data->mode_init_buf_count = count;
	return 0;
}

static ssize_t sensor_sysfs_read(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	struct i2c_client *client = data->client;
	const int RETRIES=5;
	int ret = 0;
	u16 cmd_status;
	int i;

	mutex_lock(&data->lock);
	switch (data->cur_sss_status) {
		case SSS_SENSOR_SET_CMD:
			dev_dbg(dev, "%s, SSS_SENSOR_SET_CMD ", __func__);
			for(i = 0; i < RETRIES; i++) {
				cmd_status = _ap0100_m034_cmd_status(client, &data->error_count);
				if (cmd_status >= 0)
					break;
				dev_dbg(dev, "%s, cmd retry", __func__);
			}
			ret = snprintf(buf, PAGE_SIZE, "%d\n", cmd_status);
			break;

		case SSS_IDLE:
			dev_dbg(dev, "%s, SSS_IDLE", __func__);
			break;

		default:
			dev_err(dev, "%s: unknown cur_sss_status %d", __func__, data->cur_sss_status);
			break;
	}
	mutex_unlock(&data->lock);
	return ret;
}

static enum SENSOR_SYSFS_STATUS _check_buf_command(const char * buf)
{
	if(strncmp(buf,SENSOR_SET_CMD_UUID,sizeof(SENSOR_SET_CMD_UUID)-1) == 0)
		return SSS_SENSOR_SET_CMD;
	return SSS_IDLE;
}

static ssize_t sensor_sysfs_write(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	static const int RETRIES=5;
	unsigned char sensor_mode;
	int i;
	int err = 0;

	mutex_lock(&data->lock);

	data->cur_sss_status = _check_buf_command(buf);

	switch (data->cur_sss_status  ) {
		case SSS_SENSOR_SET_CMD:
			if (count < sizeof(SENSOR_SET_CMD_UUID))
				break;

			sensor_mode = (buf[sizeof(SENSOR_SET_CMD_UUID)-1 + 1]);

			dev_dbg(dev, "%s, SSS_SENSOR_SET_CMD, read cmd %d", __func__, sensor_mode );

			if (sensor_mode == MODE_INIT_MODE_BUF) {
				_ap0100_m034_sensor_update_init_buf(data,
						(unsigned char *)buf + sizeof(SENSOR_SET_CMD_UUID) + 1,
						count - sizeof(SENSOR_SET_CMD_UUID)-1);
			} else if (sensor_mode == MODE_INIT_PARAM_BUF) {
				memcpy(&data->cam_param_s, (unsigned char *)buf + sizeof(SENSOR_SET_CMD_UUID) + 1, sizeof(data->cam_param_s));
			} else if(sensor_mode >= MODE_CNT) {
					dev_err(dev, "Bad sensor cmd %d", sensor_mode);
					break;
			} else {
				for(i = 0; i < RETRIES; i++) {
					err = _ap0100_m034_sensor_set_mode(data,
							(enum ap0100_set_mode) sensor_mode);
					if(err >= 0)
						break;
					dev_warn(dev, "%s, cmd retry:%d", __func__, err);
				}
			}
			break;

		case SSS_IDLE:
			data->already_reset = false;
			dev_dbg(dev, "sensor sysfs command idle!");
			break;
	}
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(ap0100_param, 0666, (void *)sensor_sysfs_read, (void *)sensor_sysfs_write);

static ssize_t ap0100_show_min_temp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	struct i2c_client *client = data->client;
	unsigned val;
	int ret;

	mutex_lock(&data->lock);
	ret = _AM_read_reg(client, REG_CAM_TEMP_MIN, &val, 1, &data->error_count);
	mutex_unlock(&data->lock);

	if(ret < 0) {
		dev_err(dev, "Bad read: %d", ret);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE,"%hhd\n",(s8)val);
}
static DEVICE_ATTR(min_temp, 0444, (void *)ap0100_show_min_temp, NULL);


static ssize_t ap0100_show_serial(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	int ret;
	u64 serial;

	mutex_lock(&data->lock);
	ret = _ap0100_get_serial(data, &serial);
	mutex_unlock(&data->lock);

	if(ret < 0)
		return 0;

	return snprintf(buf, PAGE_SIZE,"%08llx\n", serial);
}
static DEVICE_ATTR(serial, 0444, (void *)ap0100_show_serial, (void *)NULL);

static ssize_t ap0100_show_unavailable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	bool unavailable;

	mutex_lock(&data->lock);
	unavailable = data->unavailable;
	mutex_unlock(&data->lock);

	return snprintf(buf, PAGE_SIZE,"%d\n", unavailable);
}
static DEVICE_ATTR(unavailable, 0644, (void *)ap0100_show_unavailable, (void *)NULL);

static ssize_t ap0100_show_max_temp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	struct i2c_client *client = data->client;
	unsigned val;
	int ret;

	mutex_lock(&data->lock);
	ret = _AM_read_reg(client, REG_CAM_TEMP_MAX, &val, 1, &data->error_count);
	mutex_unlock(&data->lock);

	if(ret < 0) {
		dev_err(dev, "Bad read: %d", ret);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE,"%hhd\n",(s8)val);
}
static DEVICE_ATTR(max_temp, 0444, (void *)ap0100_show_max_temp, NULL);

static ssize_t ap0100_show_cur_temp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	struct i2c_client *client = data->client;
	unsigned val;
	int ret;

	mutex_lock(&data->lock);
	ret = _AM_read_reg(client, REG_CAM_TEMP_CUR, &val, 1, &data->error_count);
	mutex_unlock(&data->lock);

	if(ret < 0) {
		dev_err(dev, "Bad read: %d", ret);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE,"%hhd\n",(s8)val);
}
static DEVICE_ATTR(cur_temp, 0444, (void *)ap0100_show_cur_temp, NULL);

static ssize_t ap0100_show_fw_metadata(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	struct fw_header header;
	int offset = 0;
	int ret;

	mutex_lock(&data->lock);
	ret = _ap0100_validate_fw_header(data, &data->flash_header);
	if (ret >= 0) {
		memcpy(&header,&data->flash_header, sizeof(header));
		goto out;
	}

	ret = _ap0100_read_flash_header(data, &header);
	if(ret >= 0) {
		memcpy(&data->flash_header, &header, sizeof(header));
	}
out:
	mutex_unlock(&data->lock);
	if(ret < 0)
		return 0;

	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "header_const = 0x%x\n", header.header_const);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "time = %u\n", header.time_stamp);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "hw_version = %d\n", header.hw_version);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "fw_version = %d\n", header.fw_version);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "lens type = %d\n", header.lens_type);
	switch (header.lens_type) {
		case LENS_M12_2_6mm:
			offset += snprintf(&buf[offset],PAGE_SIZE-offset, "lens type: 2.6mm lens\n");
			break;
		case LENS_M12_4_2mm:
			offset += snprintf(&buf[offset],PAGE_SIZE-offset, "lens type: 4.2mm lens\n");
			break;
		case LENS_M12_6mm:
			offset += snprintf(&buf[offset],PAGE_SIZE-offset, "lens type: 6mm lens\n");
			break;
		case LENS_M12_3_6mm:
			offset += snprintf(&buf[offset],PAGE_SIZE-offset, "lens type: 3.6mm lens\n");
			break;
		default:
			offset += snprintf(&buf[offset],PAGE_SIZE-offset, "error lens type = %d\n", header.lens_type);
			break;
		}
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "bin_size = %d\n", header.bin_size);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "bin crc = 0x%x\n", header.bin_crc);
	offset += snprintf(&buf[offset],PAGE_SIZE-offset, "header crc = 0x%x\n\n", header.header_crc);

	return offset;
}
static DEVICE_ATTR(fw_metadata, 0444, (void *)ap0100_show_fw_metadata, NULL);


static ssize_t ap0100_show_fw_status(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	int status;
	mutex_lock(&data->lock);
	status = data->fw_status;
	mutex_unlock(&data->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->fw_status);
}

static ssize_t ap0100_write_bin(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	char name[count+1];
	int name_length = count;
	char *end;
	int ret;
	end = strpbrk(buf,"\n\r");
	if(end != NULL)
		name_length = (int)(end-buf);
	memcpy(name, buf, name_length);
	name[name_length] = '\0';
	mutex_lock(&data->lock);
	if(data->operational) {
		dev_err(data->dev, "Device must not be operational\n");
		goto out;
	}
	ret = _ap0100_handle_cbf(data, name, _ap0100_write_fw_bin);
	dev_err(data->dev, "Bin update %s:%d\n", ret < 0 ? "failed" : "succeeded", ret);
	data->fw_status = ret;
out:
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(write_bin, 0644, (void *)ap0100_show_fw_status, (void *)ap0100_write_bin);


static ssize_t ap0100_write_header(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	char name[count+1];
	int name_length = count;
	char *end;
	int ret;
	struct fw_header header;
	end = strpbrk(buf,"\n\r");
	if(end != NULL)
		name_length = (int)(end-buf);
	memcpy(name, buf, name_length);
	name[name_length] = '\0';
	mutex_lock(&data->lock);
	if(data->operational) {
		dev_err(data->dev, "Device must not be operational\n");
		goto out;
	}
	ret = _ap0100_handle_cbf(data, name, _ap0100_write_fw_header);
	dev_err(data->dev, "Header update %s:%d\n", ret < 0 ? "failed" : "succeeded", ret);
	data->fw_status = ret;
	if(ret < 0)
		goto out;

	ret = _ap0100_read_flash_header(data, &header);
	if(ret >= 0) {
		memcpy(&data->flash_header, &header, sizeof(header));
	}
out:
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(write_header, 0644, (void *)ap0100_show_fw_status, (void *)ap0100_write_header);

static ssize_t ap0100_validate_fw_in_flash(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_dev(dev);
	char name[count+1];
	int name_length = count;
	char *end;
	int ret;
	end = strpbrk(buf,"\n\r");
	if(end != NULL)
		name_length = (int)(end-buf);
	memcpy(name, buf, name_length);
	name[name_length] = '\0';
	mutex_lock(&data->lock);
	if(data->operational) {
		dev_err(data->dev, "Device must not be operational\n");
		goto out;
	}
	ret = _ap0100_handle_cbf(data, name, _ap0100_validate_fw_in_flash);
	dev_err(data->dev, "Validation %s:%d\n", ret < 0 ? "failed" : "succeeded", ret);
	data->fw_status = ret;
out:
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(validate_flash, 0200, (void *)ap0100_show_fw_status, (void *)ap0100_validate_fw_in_flash);


static struct attribute *attributes[] = {
	&dev_attr_ap0100_param.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int ap0100_m034_g_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);
	mutex_lock(&data->lock);
	mf->width = ap0100_m034_frame_data[data->mode].width;
	mf->height = ap0100_m034_frame_data[data->mode].height;
	mutex_unlock(&data->lock);
	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->code = V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field	= V4L2_FIELD_NONE;
	return 0;
}

static int ap0100_m034_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);
	int i;

	for(i=0; i < ARRAY_SIZE(ap0100_m034_frame_data); i++){
		if(ap0100_m034_frame_data[i].fi.denominator == interval->interval.denominator &&
		   ap0100_m034_frame_data[i].fi.numerator == interval->interval.numerator) {

			mutex_lock(&data->lock);
			data->pending_fi = interval->interval;
			mutex_unlock(&data->lock);

			return 0;
		}
	}
	return -EINVAL;
}

static int ap0100_m034_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);

	mutex_lock(&data->lock);
	interval->interval = ap0100_m034_frame_data[data->mode].fi;
	mutex_unlock(&data->lock);

	return 0;
}

static int ap0100_m034_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_frmsizeenum *fsize)
{
	__u32 index = fsize->index;
	if(index > ARRAY_SIZE(ap0100_m034_frame_data))
		return -EINVAL;

	if(fsize->pixel_format != V4L2_PIX_FMT_UYVY)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.height = ap0100_m034_frame_data[index].height;
	fsize->discrete.width = ap0100_m034_frame_data[index].width;
	return 0;
}

static int ap0100_m034_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config)
{
	int ret = 0;
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);

	mutex_lock(&data->lock);
	switch(input){
	case 0:
		data->use_test_input = false;
		break;
	case 1:
		data->use_test_input = true;
		break;
	default:
		ret = -EINVAL;
		break;
	};
	mutex_unlock(&data->lock);

	return ret;
}

static	int ap0100_m034_find_frame_format(struct ap0100_m034_data *data,
			    struct v4l2_mbus_framefmt *fmt)
{
	int i;

	if(fmt->colorspace != V4L2_COLORSPACE_JPEG)
		return -EINVAL;
	if(fmt->code != V4L2_MBUS_FMT_UYVY8_2X8)
		return -EINVAL;
	for(i = 0; i < ARRAY_SIZE(ap0100_m034_frame_data); i++) {
		if(data->pending_fi.denominator == ap0100_m034_frame_data[i].fi.denominator &&
			data->pending_fi.numerator == ap0100_m034_frame_data[i].fi.numerator &&
			fmt->width == ap0100_m034_frame_data[i].width &&
			fmt->height == ap0100_m034_frame_data[i].height) {
					return i;
		}
	}
	return -EINVAL;
}

static	int ap0100_m034_try_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);
	int ret;

	mutex_lock(&data->lock);
	ret = ap0100_m034_find_frame_format(data,fmt);
	mutex_unlock(&data->lock);

	return ret < 0 ? ret : 0;
}

static	int ap0100_m034_s_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	struct ap0100_m034_data *data = to_ap0100_m034_from_v4l2(sd);
	int ret;
	int mode;

	mutex_lock(&data->lock);

	mode = ap0100_m034_find_frame_format(data,fmt);
	if (mode < 0) {
		ret = mode;
		goto out;
	}

	if(!data->operational) {
		ret = -ENODEV;
		goto out;
	}
	ret = _ap0100_m034_sensor_init(data, mode);
	if(ret < 0)
		goto out;

	ret = _ap0100_m034_sensor_mode_init(data);

out:
	mutex_unlock(&data->lock);
	return ret;
}

#define REG_MCU_BOOT_OPT_SPI_CFG_DISABLE_SHIFT 5
#define REG_MCU_BOOT_OPT_SPI_CFG_DISABLE_MASK (1 << REG_MCU_BOOT_OPT_SPI_CFG_DISABLE_SHIFT)

static int _ap0100_host_config_mode(struct ap0100_m034_data* data)
{
	struct i2c_client *client = data->client;
	unsigned  val;
	int ret;

	ret = device_reset(data->dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	ret = _AM_read_reg(client, REG_MCU_BOOT_OPT, &val, 2, &data->error_count);
	if(ret < 0)
		return ret;
	val |= REG_MCU_BOOT_OPT_SPI_CFG_DISABLE_MASK;
	ret = _AM_write_reg(client, REG_MCU_BOOT_OPT, val, 2,  &data->error_count);
	if(ret < 0)
		return ret;

	ret = _AM_write_reg_unchecked(client, REG_RST_MISC_CTL, 0x0015, 2, &data->error_count);
	if(ret < 0)
		return ret;

	msleep(500);
	ret = _AM_write_reg_unchecked(client, REG_RST_MISC_CTL, 0x0E14, 2, &data->error_count);
	if(ret < 0)
		return ret;
	msleep(500);

	ret = _AM_read_reg(client, REG_MCU_BOOT_OPT, &val, 2, &data->error_count);
	if(ret < 0)
		return ret;
	val &= ~REG_MCU_BOOT_OPT_SPI_CFG_DISABLE_MASK;
	ret = _AM_write_reg(client, REG_MCU_BOOT_OPT, val, 2,  &data->error_count);
	if(ret < 0)
		return ret;

	ret = _ap0100_set_low_power(data);
	return ret;
}

static int _ap0100_reset(struct ap0100_m034_data* data)
{
	struct device *dev = data->dev;
	struct i2c_client *client = data->client;
	unsigned  val;
	int ret;
	int i;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	_AM_read_reg(client, REG_CHIP_VERSION, &val, 2, NULL);
	if (val != 0x0062) {
		dev_err(dev, "ap0100 not found=0x%x", val);
		return -1;
	} else {
		dev_err(dev, "ap0100 found=0x%x", val);
	}

	ret = -EBUSY;
	for(i=0;i < i2c_retries && ret == -EBUSY;i++) {
		ret = _AM_send_command(client, CMD_SYS_GET_STATE, &data->error_count);
		msleep(5);
	}

	if(ret < 0) {
		dev_err(dev, "Get state returned %d", ret);
	}
	return ret;
}

static void _ap0100_subdev_init(struct ap0100_m034_data* data);
static int _ap0100_m034_probe(struct ap0100_m034_data* data)
{
	struct device *dev = data->dev;
	int ret, tmpret;
	dev_info(dev, __func__);

	if(data->operational)
		return 0;

	ret = _ap0100_reset(data);
	if(ret < 0) {
		goto error;
	}

	ret = _ap0100_m034_sensor_init(data, MODE_720P_30);
	if(ret < 0) {
		goto error;
	}

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if(ret < 0)
		goto error;

	data->error_count = 0;
	data->operational = true;
	_ap0100_subdev_init(data);
	return ret;
error:
	tmpret = _ap0100_host_config_mode(data);
	data->unavailable = tmpret < 0;
	dev_err(dev, "Switch to host mode %s\n", tmpret < 0 ? "failed" : "succeeded");
	return ret;
}

static void _ap0100_subdev_clear(struct ap0100_m034_data* data);
static int _ap0100_m034_remove(struct ap0100_m034_data *data)
{
	_ap0100_subdev_clear(data);
	data->operational = false;
	sysfs_remove_group(&data->dev->kobj, &attr_group);
	return 0;
}


static ssize_t ap0100_m034_operational_store(struct device *dev,
                                  struct device_attribute *attr, const char *buf, int count)
{
	struct ap0100_m034_data* data = to_ap0100_m034_from_dev(dev);
	unsigned long val;

	if(_kstrtoul(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}

	mutex_lock(&data->lock);
	if(!!val == data->operational)
		goto out;

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
		goto out;
	}

	if(data->operational)
		_ap0100_m034_remove(data);

	if(val)
		_ap0100_m034_probe(data);
	else
		_ap0100_reset(data);
out:
	mutex_unlock(&data->lock);
	return count;
}

static ssize_t ap0100_m034_operational_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
       struct ap0100_m034_data* data = to_ap0100_m034_from_dev(dev);
       return sprintf(buf, "%d\n",data->operational);
}
static DEVICE_ATTR(operational, 0666, (void *)ap0100_m034_operational_show, (void *)ap0100_m034_operational_store);

static ssize_t ap0100_m034_error_count_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
       struct ap0100_m034_data* data = to_ap0100_m034_from_dev(dev);
       return sprintf(buf, "%d\n",data->error_count);
}
static DEVICE_ATTR(error_count, 0444, (void *)ap0100_m034_error_count_show, NULL);

static struct attribute *init_attributes[] = {
	&dev_attr_operational.attr,
	&dev_attr_error_count.attr,
	&dev_attr_write_bin.attr,
	&dev_attr_write_header.attr,
	&dev_attr_validate_flash.attr,
	&dev_attr_fw_metadata.attr,
	&dev_attr_serial.attr,
	&dev_attr_unavailable.attr,
	&dev_attr_min_temp.attr,
	&dev_attr_max_temp.attr,
	&dev_attr_cur_temp.attr,
	&dev_attr_ap0100_m034_i2c_test.attr,
	NULL,
};

static const struct attribute_group init_attr_group = {
	.attrs	= init_attributes,
};

static struct v4l2_subdev_video_ops ap0100_m034_subdev_video_ops = {
	.g_mbus_fmt	= ap0100_m034_g_mbus_fmt,
	.s_frame_interval = ap0100_m034_s_frame_interval,
	.g_frame_interval = ap0100_m034_g_frame_interval,
	.enum_framesizes = ap0100_m034_enum_framesizes,
	.s_routing = ap0100_m034_s_routing,
	.try_mbus_fmt = ap0100_m034_try_mbus_fmt,
	.s_mbus_fmt = ap0100_m034_s_mbus_fmt,
};

static struct v4l2_subdev_ops ap0100_m034_subdev_ops = {
	.video	= &ap0100_m034_subdev_video_ops,
};

static void _ap0100_subdev_init(struct ap0100_m034_data* data)
{
	struct i2c_client *client = data->client;
	struct v4l2_subdev *subdev = &data->subdev;
	snprintf(subdev->name, sizeof(subdev->name), "%s %d-%04x",
		client->driver->driver.name, i2c_adapter_id(client->adapter),
		client->addr);
	subdev->name[sizeof(subdev->name)-1] = 0;
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
}

static void _ap0100_subdev_clear(struct ap0100_m034_data* data)
{
	struct v4l2_subdev *subdev = &data->subdev;
	subdev->name[0] = 0;
}


static int ap0100_m034_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *subdev;
	struct device *dev = &client->dev;
	struct ap0100_m034_data* data;
	int ret;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	mutex_init(&data->lock);
	data->ap0100_in_wdr_mode = 1;
	data->client = client;
	data->dev = dev;

	/*Not using v4l2_i2c_subdev_init because we don't want i2c devices to be
	 * automatically removed
	 */
	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &ap0100_m034_subdev_ops);
	subdev->owner = client->driver->driver.owner;
	v4l2_set_subdevdata(subdev, client);
	i2c_set_clientdata(client, subdev);

	mutex_lock(&data->lock);
	ret = sysfs_create_group(&dev->kobj, &init_attr_group);
	if(ret < 0) {
	   dev_err(dev,"Could not create sysfs file.\n");
	   goto out;
	}

	_ap0100_m034_probe(data);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int ap0100_m034_remove(struct i2c_client *client)
{
	int ret = 0;
	struct ap0100_m034_data *data = to_ap0100_m034_from_i2c(client);

	mutex_lock(&data->lock);
	if(data->subdev.v4l2_dev != NULL) {
		dev_err(data->dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
	}

	if(data->operational)
		ret = _ap0100_m034_remove(data);

	sysfs_remove_group(&data->dev->kobj, &init_attr_group);
	mutex_unlock(&data->lock);
	return ret;
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
MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("ap0100_m034 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

