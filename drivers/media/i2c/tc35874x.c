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
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

struct tc35874x_reg {
    unsigned char data_size;
    short reg_addr;
    long data;
};

static const struct tc35874x_reg parallel_output_normal_regs[] = {
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
   {0,0,0},
};

static const struct tc35874x_reg mipi_probe_regs[] = {
#if 0
   {2, 0x0002, 0x0001},
   {2, 0x0002, 0x0000},
#endif
   {0,0,0},
};

static const struct tc35874x_reg mipi_init_regs[] = {

   {0,0,0},
};

static const struct tc35874x_reg mipi_output_normal_regs[] = {
   {2, 0x00e0, 0x0000},
   {2, 0x0016, 0x612c}, //PLL setting
   {2, 0x0018, 0x0613}, //PLL setting
   {2, 0x0006, 0x0104}, //FIFO write level for parallel port output(?)
   {2, 0x0008, 0x0060}, //Set YUV422 8-bit PDFormat
   {2, 0x0022, 0x0A00}, //Set 2560 bytes per line?

   {4, 0x0140, 0x00000000}, //Clock Bypass Lane Enable from PPI Layer enable.
   {4, 0x0144, 0x00000000}, //data0 Bypass Lane Enable from PPI Layer enable.
   {4, 0x0148, 0x00000000},  //data1 Bypass Lane Enable from PPI Layer enable.
   {4, 0x014C, 0x00000001}, //data2 Force Lane Disable
   {4, 0x0150, 0x00000001}, //data3 Force Lane Disable
   {4, 0x0210, 0x00000900}, //send LP-11 for at least 100us (hopefully)
   {4, 0x0214, 0x00000001}, //timing for LPTX
   {4, 0x0218, 0x00000400}, //TCLK_ZEROCNT set to 4, TCLK_PREPARECNT set to 0   {0,0,0},
   {4, 0x021C, 0x00000000}, //TCLK_TRAILCNT set to 0
   {4, 0x0220, 0x00000001}, //THS_ZEROCNT set to 0, THS_PREPARECNT set to 1
   {4, 0x0224, 0x00002800}, //TWAKEUPCNT counter
   {4, 0x0228, 0x00000000}, //TCLK_POSTCNT
   {4, 0x022C, 0x00000000}, //THS_TRAIL
   {4, 0x0234, 0x00000007}, //enable power for data lanes 1, 0, and clk
   {4, 0x0238, 0x00000001}, //set continuous clock mode
   {4, 0x0204, 0x00000001}, //start PPI-TX

   {4, 0x0518, 0x00000001}, //csi start (no description in reference manual)
   {4, 0x0500, 0xA30080A3}, //This seems like... not sure. LP command of A3, writing 1 to some reserved bit? BTA executed on lane 0

   {2, 0x0032, 0x0000}, //not originally present, clear frame stop and reset pointers
   {2, 0x0004, 0x0041}, //enable parallel port, csi2 rx 2 data lanes?
   {0,0,0},
};

static const struct tc35874x_reg mipi_output_test_regs[] = {
   {2, 0x00e0, 0x0000},
   {2, 0x0016, 0x612c}, //PLL setting
   {2, 0x0018, 0x0613}, //PLL setting
   {2, 0x00e0, 0x0000},
   {2, 0x0006, 0x0000},

   {4, 0x0140, 0x00000000}, //Clock Bypass Lane Enable from PPI Layer enable.
   {4, 0x0144, 0x00000000}, //data0 Bypass Lane Enable from PPI Layer enable.
   {4, 0x0148, 0x00000000},  //data1 Bypass Lane Enable from PPI Layer enable.
   {4, 0x014C, 0x00000001}, //data2 Force Lane Disable
   {4, 0x0150, 0x00000001}, //data3 Force Lane Disable
   {4, 0x0210, 0x00000900}, //send LP-11 for at least 100us (hopefully)
   {4, 0x0214, 0x00000001}, //timing for LPTX
   {4, 0x0218, 0x00000400}, //TCLK_ZEROCNT set to 4, TCLK_PREPARECNT set to 0   {0,0,0},
   {4, 0x021C, 0x00000000}, //TCLK_TRAILCNT set to 0
   {4, 0x0220, 0x00000001}, //THS_ZEROCNT set to 0, THS_PREPARECNT set to 1
   {4, 0x0224, 0x00002800}, //TWAKEUPCNT counter
   {4, 0x0228, 0x00000000}, //TCLK_POSTCNT
   {4, 0x022C, 0x00000000}, //THS_TRAIL
   {4, 0x0234, 0x00000007}, //enable power for data lanes 1, 0, and clk
   {4, 0x0238, 0x00000001}, //set continuous clock mode

   {4, 0x0204, 0x00000001}, //start PPI-TX
   {4, 0x0518, 0x00000001}, //csi start (no description in reference manual)
   {4, 0x0500, 0xA30080A3}, //This seems like... not sure. LP command of A3, writing 1 to some reserved bit? BTA executed on lane 0
   {2, 0x0008, 0x0001}, // Set PDFormat = RAW8, User Data Type ID enable opposite
   {2, 0x0050, 0x001e}, // set user data type register to 0x1E
   {2, 0x0022, 0x0A00}, //Set word count register (bytes/line) to 2560
   {2, 0x00e0, 0x8000}, //debug active line count register, enable i2c/spi write to VB sram
   {2, 0x00e2, 0x0A00}, //Set debug word count register (bytes/line) to 2560
   {2, 0x00e4, 0x058}, //set vertical blank line count register
   {2, 0x0032, 0x0000}, //clear frame stop and reset pointers
   {0,0,0},
};

enum tc35874x_output {
	TC35874X_OUTPUT_PARALLEL = 0,
	TC35874X_OUTPUT_MIPI = 1,
};

struct tc35874x_data {
	struct i2c_client* client;
	enum tc35874x_output output;
	bool use_test_input; //technically this is output, but ends up being the same.
	struct v4l2_subdev	subdev;
    unsigned int frame_time_ms;
};

static struct tc35874x_data *to_tc35874x_from_i2c(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct tc35874x_data, subdev);
}

static struct tc35874x_data *to_tc35874x_from_v4l2(const struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return to_tc35874x_from_i2c(client);
}

static struct tc35874x_data *to_tc35874x_from_dev(const struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	return container_of(sd, struct tc35874x_data, subdev);
}

static int tc35874x_write_reg(struct i2c_client * client, u16 reg, unsigned val, int data_len)
{
	int err;
	u8 buf[2 + 4];
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	switch(data_len){
	case 1:
		buf[2] = val;
		break;
	case 2:
		buf[2] = val >> 8;
		buf[3] = val & 0xff;
		break;
	case 4:
		buf[2] = val >> 8;
		buf[3] = val & 0xff;
		buf[4] = val >> 24;
		buf[5] = val >> 16;
		break;
	};
	dev_dbg(&client->dev,"wr %hx:%x", reg, val);
	if ((err = i2c_master_send(client, buf, 2+data_len)) < 0) {
		dev_err(&client->dev, "%s:write reg warn:reg=%x,val=%x,err=%d\n",
			__func__, reg, val, err);
	}
	return err;
}

static int tc35874x_read_reg(struct i2c_client * client, u16 reg, unsigned *val, int data_len)
{
	int err;
	u8 buf[4] = {0};
	u8 au8RegBuf[2] = {0};
	struct i2c_msg msgs[2];

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = au8RegBuf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = data_len;
	msgs[1].buf = buf;

	err = i2c_transfer(client->adapter, msgs, 2);
	dev_dbg(&client->dev,"rd %hx:%02hhx%02hhx%02hhx%02hhx", reg, buf[0],buf[1],buf[2],buf[3]);

	switch(data_len){
	case 1:
		*val = buf[0];
		break;
	case 2:
		*val = ((buf[0] << 8) & 0xff00) | (buf[1] & 0x00ff);
		break;
	case 4:
		*val =  ((buf[2] << 24) & 0xff000000) |
				((buf[3] << 16) & 0x00ff0000) |
				((buf[0] << 8)  & 0x0000ff00) |
				 (buf[1]        & 0x000000ff);
		break;
	};
	if (err < 0) {
		dev_err(&client->dev,"%s:read reg warn:reg=%x,val=%x,err=%d\n", __func__, reg, *val,
					err);
	}
	return err;
}

static void write_color_bar(struct i2c_client* client)
{
	int i,j;

	for (j=0; j<4; j++) {
		// 64 pixels of red
		for (i=0; i<32; i++) {
			tc35874x_write_reg(client, 0x00e8, 0x0000, 2);
			tc35874x_write_reg(client, 0x00e8, 0x00ff, 2);
		}

		// 64 pixels of green
		for (i=0; i<32; i++) {
			tc35874x_write_reg(client, 0x00e8, 0x7f00, 2);
			tc35874x_write_reg(client, 0x00e8, 0x7f00, 2);
		}

		// 64 pixels of blue
		for (i=0; i<32; i++) {
			tc35874x_write_reg(client, 0x00e8, 0xc0ff, 2);
			tc35874x_write_reg(client, 0x00e8, 0xc000, 2);
			}

		// 64 pixels of white
		for (i=0; i<32; i++) {
			tc35874x_write_reg(client, 0x00e8, 0xff7f, 2);
			tc35874x_write_reg(client, 0x00e8, 0xff7f, 2);
			}
   	}
	tc35874x_write_reg(client, 0x00e0, 0xC2CF, 2);
}

void tc35874x_reset(struct i2c_client* client)
{
	tc35874x_write_reg(client, 0x0002,0x0001, 2);
	tc35874x_write_reg(client, 0x0002,0x0000, 2);
}

void tc35874x_stop(struct tc35874x_data *data)
{
	struct i2c_client* client = data->client;
	unsigned val;
	//Set FrmStop
	tc35874x_read_reg(client, 0x0032, &val, 2);
	val |= 1 << 15;
	tc35874x_write_reg(client, 0x0032, val, 2);

	msleep(2*data->frame_time_ms);

	//Clear PP_En
	tc35874x_read_reg(client, 0x0004, &val, 2);
	val &= ~(1 << 6);
	tc35874x_write_reg(client, 0x0004,val, 2);

	//Set RstPtr
	tc35874x_read_reg(client, 0x0032, &val, 2);
	val |= 1 << 14;
	tc35874x_write_reg(client, 0x0032, val, 2);
}

int tc35874x_write_regs(struct i2c_client *client, const struct tc35874x_reg *reg_ptr)
{
	int ret;
	while (reg_ptr && reg_ptr->data_size != 0) {
		ret = tc35874x_write_reg(client, reg_ptr->reg_addr, reg_ptr->data, reg_ptr->data_size);
		if(ret < 0)
			return ret;
		reg_ptr++;
	}
	return 0;
}
int tc35874x_write_config(struct tc35874x_data *data)
{
	struct i2c_client *client = data->client;

	if(data->output == TC35874X_OUTPUT_PARALLEL)
		tc35874x_write_regs(client, parallel_output_normal_regs);
	else if(data->use_test_input)
		tc35874x_write_regs(client, mipi_output_test_regs);
	else
		tc35874x_write_regs(client, mipi_output_normal_regs);


	if (data->use_test_input)
		write_color_bar(client);

	dev_dbg(&client->dev, "mipi bridge init done");
	return 0;
}



static ssize_t tc35874x_csi_err(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	int ret =tc35874x_read_reg(data->client,0x044C, &val, 2);
	if(ret < 0)
		dev_err(&data->client->dev,"reg read error");
	return sprintf(buf, "%04x\n", val);
}
static DEVICE_ATTR(csi_err, 0444, (void *)tc35874x_csi_err, (void *)NULL);

static ssize_t tc35874x_csi_status(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	int ret =tc35874x_read_reg(data->client,0x0410, &val, 2);
	if(ret < 0)
		dev_err(&data->client->dev,"reg read error");
	return sprintf(buf, "%04x\n", val);
}
static DEVICE_ATTR(csi_status, 0444, (void *)tc35874x_csi_status, (void *)NULL);

static ssize_t tc35874x_csi_control(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	int ret =tc35874x_read_reg(data->client,0x040c, &val, 2);
	if(ret < 0)
		dev_err(&data->client->dev,"reg read error");
	return sprintf(buf, "%04x\n", val);
}
static DEVICE_ATTR(csi_control, 0444, (void *)tc35874x_csi_control, (void *)NULL);

static ssize_t tc35874x_csi_int(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	int ret =tc35874x_read_reg(data->client,0x0414, &val, 4);
	if(ret < 0)
		dev_err(&data->client->dev,"reg read error");
	return sprintf(buf, "%08x\n", val);
}
static DEVICE_ATTR(csi_int, 0444, (void *)tc35874x_csi_int, (void *)NULL);

static ssize_t tc35874x_ppi_status(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	int ret =tc35874x_read_reg(data->client,0x0208, &val, 4);
	if(ret < 0)
		dev_err(&data->client->dev,"reg read error");
	return sprintf(buf, "%08x\n", val);
}
static DEVICE_ATTR(ppi_status, 0444, (void *)tc35874x_ppi_status, (void *)NULL);

static ssize_t tc35874x_error_counters(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	unsigned val;
	size_t count = 0;
	int ret;

	static const char *fifostatus[4] = { "normal", "overflow",
					     "underflow", "over- and underflow" };

	ret = tc35874x_read_reg(data->client, 0x0080, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Frame errors:\t\t\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x0082, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "CRC errors:\t\t\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x0084, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Recoverable header errors:\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x0086, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Unrecoverable header errors:\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x0088, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Packet ID errors:\t\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x008A, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Control errors:\t\t\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x008C, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Recoverable sync errors:\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x008E, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "Unrecoverable sync errors:\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x0090, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "MDL sync errors:\t\t%u\n", val);

	ret = tc35874x_read_reg(data->client, 0x00F8, &val, 2);
	if (ret < 0)
		return ret;
	count += scnprintf(buf+count, PAGE_SIZE-count, "FIFO status:\t\t\t%s\n", fifostatus[val & 3]);

	return count;
}
static ssize_t tc35874x_clear_counters(struct device *dev, struct device_attribute *attr,
				       const char *buf, int count)
{
	struct tc35874x_data* data = to_tc35874x_from_dev(dev);
	int ret;

	ret = tc35874x_write_reg(data->client, 0x0080, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x0082, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x0084, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x0086, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x0088, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x008A, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x008C, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x008E, 0, 2);
	if (ret < 0)
		return ret;

	ret = tc35874x_write_reg(data->client, 0x0090, 0, 2);
	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR(error_counters, 0666, (void *)tc35874x_error_counters, (void *)tc35874x_clear_counters);

static struct attribute *attributes[] = {
		&dev_attr_csi_err.attr,
		&dev_attr_csi_status.attr,
		&dev_attr_csi_control.attr,
		&dev_attr_ppi_status.attr,
		&dev_attr_csi_int.attr,
		&dev_attr_error_counters.attr,
		NULL,
};
static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int tc35874x_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config)
{
	struct tc35874x_data *data = to_tc35874x_from_v4l2(sd);
	if(input > 1)
		return -EINVAL;

	if(data->output == TC35874X_OUTPUT_PARALLEL && input == 1) {
		return -EINVAL;
	} else
		data->use_test_input = input;

	return 0;
}

static int tc35874x_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct tc35874x_data *data = to_tc35874x_from_v4l2(sd);

	data->frame_time_ms = ((interval->interval.numerator * 1000) / interval->interval.denominator);

	return 0;
}

int tc35874x_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc35874x_data *data = to_tc35874x_from_v4l2(sd);
	int ret = 0;

	if(data->output == TC35874X_OUTPUT_PARALLEL)
		return 0;

	if(!enable) {
		tc35874x_stop(data);
	} else {
		ret = tc35874x_write_config(data);
		msleep(2*data->frame_time_ms);
	}
	return ret;
}


/*Expect this to be run once at start.  Attempt to set MIPI in PL-11 (STOP) state.*/
int tc35874x_v4l2_init(struct v4l2_subdev *sd, u32 val)
{
	struct tc35874x_data *data = to_tc35874x_from_v4l2(sd);

	if(data->output == TC35874X_OUTPUT_PARALLEL)
		return 0;

	return tc35874x_write_regs(data->client, mipi_init_regs);
}

static struct v4l2_subdev_core_ops tc35874x_v4l2_subdev_core_ops =
{
		.init = tc35874x_v4l2_init,
};

static struct v4l2_subdev_video_ops tc35874x_subdev_video_ops = {
	.s_routing = tc35874x_s_routing,
	.s_stream = tc35874x_video_s_stream,
	.s_frame_interval = tc35874x_s_frame_interval,
};

static struct v4l2_subdev_ops tc35874x_subdev_ops = {
	.core = &tc35874x_v4l2_subdev_core_ops,
	.video	= &tc35874x_subdev_video_ops,
};

static int tc35874x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tc35874x_data *data;
	struct v4l2_subdev *subdev;
	int ret;
	unsigned val;
	const char *mode;

	ret = tc35874x_read_reg(client, 0x0, &val, 2);
	if (ret < 0 || val !=  0x4401) {
		dev_err(dev, "mipi bridge not found = 0x%x", val);
		return -ENODEV;
	} else {
		dev_info(dev, "mipi bridge found = 0x%x", val);
	}

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	data->client = client;
	/*Not using v4l2_i2c_subdev_init because we don't want i2c devices to be
	 * automatically removed
	 */
	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &tc35874x_subdev_ops);
	subdev->owner = client->driver->driver.owner;
	v4l2_set_subdevdata(subdev, client);
	i2c_set_clientdata(client, subdev);
	snprintf(subdev->name, sizeof(subdev->name), "%s %d-%04x",
		client->driver->driver.name, i2c_adapter_id(client->adapter),
		client->addr);
	subdev->name[sizeof(subdev->name)-1] = 0;

	mode = of_get_property(dev->of_node,
						   "output-mode", NULL);
	if(!mode) {
		dev_err(dev, "property output-mode missing");
		return -EINVAL;
	}

	if(!strcmp(mode,"parallel")) {
		data->output = TC35874X_OUTPUT_PARALLEL;
		tc35874x_write_regs(client,parallel_output_normal_regs);
	} else if (!strcmp(mode,"mipi")) {
		data->output = TC35874X_OUTPUT_MIPI;
		tc35874x_write_regs(client,mipi_probe_regs);
	} else {
		dev_err(dev, "output-mode has unknown value %s", mode);
		return -EINVAL;
	}
	return sysfs_create_group(&dev->kobj, &attr_group);
}

/*!
 * tc35874x I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int tc35874x_remove(struct i2c_client *client)
{
	struct tc35874x_data *data = to_tc35874x_from_i2c(client);

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(&client->dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
	}

	sysfs_remove_group(&client->dev.kobj, &attr_group);
	return 0;
}

static struct of_device_id tc35874x_dt_ids[] = {
	{ .compatible = "toshiba,tc35874x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tc35874x_dt_ids);

static const struct i2c_device_id tc35874x_i2c_id[] = {
	{ .name = "tc35874x", .driver_data = 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tc35874x_i2c_id);

static struct i2c_driver tc35874x_driver = {
	.driver = {
		   .name = "tc35874x",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(tc35874x_dt_ids),
		   },
	.probe = tc35874x_probe,
	.remove = tc35874x_remove,
	.id_table = tc35874x_i2c_id,
};

module_i2c_driver(tc35874x_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("tc35874x Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

