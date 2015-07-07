/*
 * Copyright (C) 2014 Xeralux, Inc. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <linux/of_i2c.h>
#include "mxc_v4l2_capture.h"

static u32 MXC_PIPELINE_XCLK_MIN = 6000000;
static u32 MXC_PIPELINE_XCLK_MAX = 67500000;

//max_subdev must be no greater than this - 1
#define MAX_PIPELINE_LENGTH 20

struct mxc_pipeline_data {
	struct device *dev;

	struct v4l2_int_slave v4l2_int_slave;
	struct v4l2_int_device v4l2_int_dev;

	struct v4l2_device v4l2_dev;
	/*IMPORTANT v4l2_dev keeps an internal representation of all the subdevices, however
	 * we have no guarantee of the ordering of those devices without making assumptions about
	 * the implementation which may not be safe to make (for example, switching from list_add_tail
	 * to list_add.  Therefore keep our own list here.
	 */
	struct v4l2_subdev **subdevs;
	/*The first entry will be the source input device (sensor) and the last entry will be the one nearest
	 * to the processor (the sink.)
	 */
	int max_subdev;
	struct sensor_data sensor;
	bool running; /*This is set/unset by the power ioctl.  All other ioctls are only accessed when power is on.*/
	bool operational; /*Set when v4l2 int device is registered*/
	struct mutex lock;
	struct mutex config_lock;
};

static struct mxc_pipeline_data *v4l2_int_to_mxc_pipeline(struct v4l2_int_device *_dev_)
{
	return container_of(_dev_,struct mxc_pipeline_data,v4l2_int_dev);
}

static struct mxc_pipeline_data *dev_to_mxc_pipeline(struct device *dev)
{
	return dev_get_drvdata(dev);
}

struct pixelcode_map {
	enum v4l2_mbus_pixelcode mbus;
	__u32			pixelformat;
};

static const struct pixelcode_map pixelcode_mappings[] =
{
		{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_PIX_FMT_UYVY},
		{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_PIX_FMT_YUYV},
};

struct mxc_pipeline_mode_info {
	u32 mode;
	u32 width;
	u32 height;
	struct v4l2_dv_timings timings;
};

struct mxc_timings_map {
	struct v4l2_dv_timings timings;
	struct v4l2_fract fi;
};

#define V4L2_DV_CUSTOM_1280X720P30 {.type = V4L2_DV_BT_656_1120, \
			.bt = {.width = 1280, .height = 720, .interlaced = 0, \
				.polarities = V4L2_DV_HSYNC_POS_POL | V4L2_DV_VSYNC_POS_POL, \
				.pixelclock = 29909520, .hfrontporch = 1, .hsync = 80, \
				.hbackporch = 1, .vfrontporch = 3, .vsync = 5, .vbackporch = 4}}

/*Mode is defined by frame size.  The timings are for setting only.
*/
#define MODE_TEST_START 10
static struct mxc_pipeline_mode_info mxc_pipeline_mode_info[] =
{
		{0, 1280, 720, V4L2_DV_CUSTOM_1280X720P30},
		{1, 1280, 960, V4L2_DV_BT_DMT_1280X960P60},
		{2, 640, 480, V4L2_DV_BT_CEA_640X480P59_94},
		/*2 is  input device default*/
		/*MODE_TEST_START is reserved for test input*/
};
#define MAX_MODE ARRAY_SIZE(mxc_pipeline_mode_info)

static struct mxc_timings_map mxc_timings_map[] = {
		{ V4L2_DV_CUSTOM_1280X720P30, { .denominator = 30, .numerator = 1,}},
		{ V4L2_DV_BT_CEA_1280X720P30, { .denominator = 30, .numerator = 1,}},
		{ V4L2_DV_BT_DMT_1280X960P60, { .denominator = 60, .numerator = 1}},
		{ V4L2_DV_BT_CEA_640X480P59_94, { .denominator = 5994, .numerator = 100,}},
};

static int _run_from_sink(struct mxc_pipeline_data *data,
							int (*dofunc)(struct v4l2_subdev *sd),
							int (*undofunc)(struct v4l2_subdev *sd))
{
	int i, ret = 0;
	int tmpret;
	for(i=data->max_subdev; i >= 0; i--) {
		struct v4l2_subdev *sd = data->subdevs[i];
		tmpret = dofunc(sd);
		if(tmpret == -ENOIOCTLCMD || tmpret >= 0)
			continue;
		if(undofunc != NULL)
			goto unwind;
		if(ret == 0)
			ret = tmpret;
	}
	return ret;
unwind:
	ret = tmpret;
	for(; i <= data->max_subdev; i++ ) {
		struct v4l2_subdev *sd = data->subdevs[i];
		undofunc(sd);
	}
	return ret;

}

static int _run_from_source(struct mxc_pipeline_data *data,
							int (*dofunc)(struct v4l2_subdev *sd),
							int (*undofunc)(struct v4l2_subdev *sd))
{
	int i, ret = 0;
	int tmpret;
	for(i=0; i <= data->max_subdev; i++) {
		struct v4l2_subdev *sd = data->subdevs[i];
		tmpret = dofunc(sd);
		if(tmpret == -ENOIOCTLCMD || tmpret >= 0)
			continue;
		if(undofunc != NULL)
			goto unwind;
		if(ret == 0)
			ret = tmpret;
	}
	return ret;
unwind:
	ret = tmpret;
	for(; i >= 0; i--) {
		struct v4l2_subdev *sd = data->subdevs[i];
		undofunc(sd);
	}
	return ret;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_subdev_frame_interval fi;
	fi.interval = sensor->streamcap.timeperframe;

	sensor->mclk = min(sensor->mclk,MXC_PIPELINE_XCLK_MAX);
	sensor->mclk = max(sensor->mclk,MXC_PIPELINE_XCLK_MIN);

	return 0;
}

/*TODO parse dt endpoints, contains a lot of data about this*/
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor = s->priv;
	memset(p, 0, sizeof(*p));

	/*this is assumed*/
	p->if_type = V4L2_IF_TYPE_BT656;

	p->u.bt656.clock_curr = sensor->mclk;
	p->u.bt656.clock_min = MXC_PIPELINE_XCLK_MIN;
	p->u.bt656.clock_max = MXC_PIPELINE_XCLK_MAX;

	p->u.bt656.latch_clk_inv = 0; /*pixelclock polarity*/

	/*TODO grab data width from elsewhere*/
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;

	p->u.bt656.nobt_vs_inv = 0; /*vsync polarity*/
	p->u.bt656.nobt_hs_inv = 0; /*hsync polarity*/
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
	return 0;
}

static int _parse_mbus_framefmt(struct v4l2_mbus_framefmt *mf, struct mxc_pipeline_data *data)
{
	struct sensor_data *sensor = &data->sensor;
	int i;

	for(i=0; i<ARRAY_SIZE(mxc_pipeline_mode_info); i++) {
		if(mxc_pipeline_mode_info[i].height == mf->height &&
			mxc_pipeline_mode_info[i].width == mf->width)
			break;
	}

	if(i == ARRAY_SIZE(mxc_pipeline_mode_info))
		sensor->streamcap.capturemode = MAX_MODE;
	else
		sensor->streamcap.capturemode = mxc_pipeline_mode_info[i].mode;

	sensor->pix.width = mf->width;
	sensor->pix.height = mf->height;
	sensor->pix.colorspace = mf->colorspace;
	for(i=0; i < ARRAY_SIZE(pixelcode_mappings); i++) {
		if(pixelcode_mappings[i].mbus == mf->code)
			break;
	}
	if(i == ARRAY_SIZE(pixelcode_mappings)) {
		dev_err(data->dev,"%s:no mapping for code %x",__func__, mf->code);
		return -EINVAL;
	}
	sensor->pix.pixelformat = pixelcode_mappings[i].pixelformat;
	return 0;
}

static int _input_device_to_sensor(struct mxc_pipeline_data *data)
{
	struct sensor_data *sensor = &data->sensor;
	struct v4l2_subdev *input_device = (data->subdevs == NULL ? NULL : data->subdevs[0]);
	struct v4l2_subdev_format format;
	struct v4l2_subdev_frame_interval fi;
	struct v4l2_dv_timings timings;
	int i;
	int ret;

	if (input_device == NULL)
		return -ENODEV;

	ret = v4l2_subdev_call(input_device, video, g_mbus_fmt, &format.format);
	if(ret < 0){
		format.pad = 1;
		format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(input_device, pad, get_fmt, NULL, &format);
	}
	if(ret < 0) {
		dev_err(data->dev, "Unable to determine frame size\n");
		return ret;
	}
	ret = _parse_mbus_framefmt(&format.format, data);
	if(ret < 0)
		return ret;

	ret = v4l2_subdev_call(input_device, video, g_frame_interval, &fi);
	if(ret >= 0) {
		sensor->streamcap.timeperframe = fi.interval;
		return 0;
	}

	ret = v4l2_subdev_call(input_device, video, g_dv_timings, &timings);
	if (ret < 0) {
		sensor->streamcap.timeperframe.denominator = 1;
		sensor->streamcap.timeperframe.numerator = 1;
		return 0;
	}

	for(i=0; i<ARRAY_SIZE(mxc_timings_map); i++) {
		if(v4l_match_dv_timings(&timings,
			&mxc_timings_map[i].timings,
			250000))
			break;
	}
	if(i == ARRAY_SIZE(mxc_timings_map)) {
		sensor->streamcap.timeperframe.denominator = 1;
		sensor->streamcap.timeperframe.numerator = 1;
	} else {
		sensor->streamcap.timeperframe = mxc_timings_map[i].fi;
	}

	return 0;
}

/*Only look at source (end) */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mxc_pipeline_data *data = v4l2_int_to_mxc_pipeline(s);
	char buf[4];
	int ret;

	if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret = _input_device_to_sensor(data);

	f->fmt.pix = data->sensor.pix;
	f->fmt.pix.priv = 0; //mxc_v4l2_capture uses this to decide if the input device is a TV or not(?)

	buf[3] = (char)(f->fmt.pix.pixelformat >> 24);
	buf[2] = (char)(f->fmt.pix.pixelformat >> 16);
	buf[1] = (char)(f->fmt.pix.pixelformat >> 8);
	buf[0] = (char)(f->fmt.pix.pixelformat >> 0);
	dev_dbg(data->dev, "%s: width %d height %d pixelformat %c%c%c%c",
			__func__, f->fmt.pix.width, f->fmt.pix.height, buf[0], buf[1], buf[2], buf[3]);

	return ret;
}

/*Only look at source (atc end), not trying to do anything fancy like look at multiple
 * subdevices and find intersection of capabilities
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct mxc_pipeline_data *data = v4l2_int_to_mxc_pipeline(s);
	struct v4l2_subdev *input_device = data->subdevs[0];
	int ret;
	__u32 index = fsize->index;
	ret = v4l2_subdev_call(input_device, video, enum_framesizes, fsize);
	if (ret < 0 && fsize->index <= (MODE_TEST_START+ data->max_subdev)) {
		fsize->index = 0;
		ret = v4l2_subdev_call(input_device, video, enum_framesizes, fsize);
		fsize->index = index;
	}
	if(ret < 0 && fsize->index <= (MODE_TEST_START+ data->max_subdev)) {
		struct sensor_data *sensor = &data->sensor;
		if(fsize->pixel_format != sensor->pix.pixelformat)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.height = (__u32)sensor->pix.height;
		fsize->discrete.width = (__u32)sensor->pix.width;
	}
	return 0;
}

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	if(a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = sensor->streamcap.capability;
	cparm->timeperframe = sensor->streamcap.timeperframe;
	cparm->capturemode = sensor->streamcap.capturemode;
	return 0;
}

static int subdev_s_stream_on(struct v4l2_subdev *sd)
{
	return v4l2_subdev_call(sd, video, s_stream, 1);
}
static int subdev_s_stream_off(struct v4l2_subdev *sd)
{
	return v4l2_subdev_call(sd, video, s_stream, 0);
}

static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mxc_pipeline_data *data = v4l2_int_to_mxc_pipeline(s);
	struct device* dev = data->dev;
	struct sensor_data *sensor = s->priv;
	struct v4l2_subdev_frame_interval fi;
	struct v4l2_subdev_format format;
	struct v4l2_frmsizeenum fsize;
	struct v4l2_dv_timings timings;
	int test_output = -1;
	int capturemode = a->parm.capture.capturemode;
	int i;
	int ret;

	if(a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fsize.index = capturemode;
	fsize.pixel_format = V4L2_PIX_FMT_UYVY;
	ret = ioctl_enum_framesizes(s, &fsize);
	if(ret < 0) {
		dev_err(dev, "Invalid capture mode %d", capturemode);
		return ret;
	}

	format.pad = 1;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.height = fsize.discrete.height;
	format.format.width  = fsize.discrete.width;

	/*Translate capture mode to subdev parameters.*/
	format.format.colorspace = sensor->pix.colorspace;
	for(i=0; i < ARRAY_SIZE(pixelcode_mappings); i++) {
		if(pixelcode_mappings[i].pixelformat == sensor->pix.pixelformat)
			break;
	}
	if(i == ARRAY_SIZE(pixelcode_mappings))
		return -EINVAL;
	format.format.code = pixelcode_mappings[i].mbus;

	if (capturemode >= MODE_TEST_START){
		test_output = capturemode - MODE_TEST_START;
		if(test_output > data->max_subdev) {
			dev_err(dev,"Invalid test output %d\n", test_output);
			return -EINVAL;
		}
	}

	dev_dbg(dev, "%s: height %d width %d", __func__, format.format.height, format.format.width);

	/*Stop subdevices before changing anything.*/
	ret = _run_from_sink(data, subdev_s_stream_off, subdev_s_stream_on);
	if(ret < 0) {
		dev_err(dev,"Failed to stop video in prep for changing parameters\n");
		return ret;
	}

	/*Set new parameters.*/
	fi.interval = a->parm.capture.timeperframe;
	ret = v4l2_device_call_until_err(&data->v4l2_dev, 0, video, s_frame_interval, &fi);
	if(ret < 0) {
		dev_err(dev, "Invalid frame rate: %d, denominator %d, numerator %d\n", ret,
				fi.interval.denominator, fi.interval.numerator);
		return ret;
	}

	if(capturemode < ARRAY_SIZE(mxc_pipeline_mode_info))  {
		timings = mxc_pipeline_mode_info[capturemode].timings;
		ret = v4l2_device_call_until_err(&data->v4l2_dev, 0, video, s_dv_timings, &timings);
		if(ret < 0) {
			dev_err(dev, "Unable to set dv timings\n");
			return ret;
		}
	}

	ret = v4l2_device_call_until_err(&data->v4l2_dev, 0, video, s_routing, 0, 0, 0);
	if(ret < 0) {
		dev_err(dev, "Failed to set inputs to defaults\n");
		return ret;
	}
	if(test_output >= 0) {
		struct v4l2_subdev *test_subdev = data->subdevs[test_output];
		ret = v4l2_subdev_call(test_subdev, video, s_routing, 1, 0, 0);
		if(ret < 0) {
			dev_err(dev,"Failed to set test mode on subdev %d from source\n",
				test_output);
			return ret;
		}
	};

	ret = v4l2_device_call_until_err(&data->v4l2_dev, 0,  video, try_mbus_fmt, &format.format);
	if(ret < 0) {
		dev_err(dev,"Incompatible mode %d %dx%d, clrspc %x code %x \n", capturemode,
			format.format.width, format.format.height, format.format.colorspace, format.format.code);
		return -EINVAL;
	}

	ret = v4l2_device_call_until_err(&data->v4l2_dev, 0,  video, s_mbus_fmt, &format.format);
	if(ret < 0) {
		dev_err(dev,"Failed to set mode %d %d x %d, clrspc %x code %x \n", capturemode,
			format.format.width, format.format.height, format.format.colorspace, format.format.code);
		return -EINVAL;
	}

	ret = v4l2_device_call_until_err(&data->v4l2_dev, 0,  pad, set_fmt, NULL, &format);
	if(ret < 0) {
		dev_err(dev,"Failed to set mode %d\n", capturemode);
		return -EINVAL;
	}

	/*Start pipeline again.*/
	ret = _run_from_source(data, subdev_s_stream_on, subdev_s_stream_off);
	if(ret < 0) {
		dev_err(dev,"Failed to (re)start video\n");
		return ret;
	}

	sensor->streamcap.timeperframe = fi.interval;
	sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

	dev_dbg(dev, "%s: denominator %d numerator %d", __func__,
			sensor->streamcap.timeperframe.denominator,
			sensor->streamcap.timeperframe.numerator);

	return 0;
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct mxc_pipeline_data *data = v4l2_int_to_mxc_pipeline(s);
	mutex_lock(&data->lock);
	data->running = on;
	mutex_unlock(&data->lock);
	return 0;
}

static struct v4l2_int_ioctl_desc mxc_pipeline_ioctl_desc[] =
{
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_enum_framesizes_num,
			(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
};

static ssize_t mxc_pipeline_subdevs_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);
	int count = 0;
	int i;

	if (data == NULL)
		return -ENODEV;

	mutex_lock(&data->lock);

	if(!data->operational)
		goto out;

	if (data->subdevs == NULL)
		goto out;

	for(i = 0; i <= data->max_subdev; i++) {
		count += snprintf(&buf[count], PAGE_SIZE-count,"%s\n", data->subdevs[i]->name);
	}
out:
	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR(subdevs, 0444, (void *)mxc_pipeline_subdevs_show, NULL);

static struct attribute *attributes[] = {
	&dev_attr_subdevs.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int walk_mxc_pipeline(struct mxc_pipeline_data *data,
		const struct device_node *prev_node,
		struct device_node *cur_node,
		unsigned cur_depth)
{
	struct i2c_client *client;
	struct device_node *cur_endpoint;
	int ret;
	int endpoint_count;
	struct v4l2_subdev *sd;

	if(cur_node == NULL) {
		dev_err(data->dev, "endpoint with no remote-port?");
		return -EINVAL;
	}

	if(cur_depth >= MAX_PIPELINE_LENGTH) {
		ret = -E2BIG;
		goto of_put;
	}
	client = of_find_i2c_device_by_node(cur_node);
	if(!client) {
		ret = -EPROBE_DEFER;
		goto of_put;
	}
	if(!device_trylock(&client->dev)) {
		ret = -EPROBE_DEFER;
		goto of_put;
	}

	if (!client->driver ||
	    !try_module_get(client->driver->driver.owner)) {
		ret = -EPROBE_DEFER;
		goto dev_put;
	}

	sd = i2c_get_clientdata(client);
	if(!sd || !sd->name[0]) {
		ret = -EPROBE_DEFER;
		goto mod_put;
	}

	endpoint_count = 1;
	ret = 0;
	for(cur_endpoint = v4l2_of_get_next_endpoint(cur_node, NULL);
		cur_endpoint != NULL;
		cur_endpoint = v4l2_of_get_next_endpoint(cur_node,cur_endpoint)) {

		struct device_node *remote_node;
		remote_node = v4l2_of_get_remote_port_parent(cur_endpoint);
		if(remote_node == prev_node)
			continue;

		endpoint_count++;
		ret = walk_mxc_pipeline(data, cur_node, remote_node, cur_depth+1);
		if(ret == 0)
			break;
	}
	if(endpoint_count == 1) {
		/*At the end.*/
		data->max_subdev = cur_depth;
		data->subdevs = devm_kzalloc(data->dev,
				sizeof(data->subdevs[0])*(cur_depth+1),
				GFP_KERNEL);
		if(!data->subdevs) {
			ret = -ENOMEM;
			goto mod_put;
		}
	}
	if(ret == 0) {
		data->subdevs[data->max_subdev - cur_depth] = sd;
		ret = v4l2_device_register_subdev(&data->v4l2_dev, sd);
		if(ret < 0) {
			dev_err(data->dev, "%s:unable to register device 0x%hx", __func__, client->addr);
		} else {
			dev_dbg(data->dev,"sd name %s, pos %d max_subdev %d",
					sd->name, data->max_subdev - cur_depth, data->max_subdev);
		}
	}
mod_put:
	module_put(client->driver->driver.owner);
dev_put:
	device_unlock(&client->dev);
	put_device(&client->dev);
of_put:
	/*Previous call to v4l2_of_get_remote_port_parent
	 * says we neet to call of_node_put() on the returned port.
	 */
	of_node_put(cur_node);
	return ret;
}

static int subdev_init(struct v4l2_subdev *sd)
{
	return v4l2_subdev_call(sd, core, init, 0);
}

static int _mxc_pipeline_probe(struct mxc_pipeline_data *data)
{
	struct device *dev = data->dev;
	struct sensor_data *sensor;
	struct v4l2_device *v4l2_dev;
	struct device_node *mxc_node;
	struct platform_device *mxc_plat;
	struct v4l2_subdev *mxc_subdev;
	struct device_node *mxc_endpoint;
	struct v4l2_int_device *v4l2_int_dev;
	struct v4l2_int_slave *v4l2_int_slave;
	struct v4l2_int_ioctl_desc *ioctl_desc_array;
	int ret;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	mxc_node = of_parse_phandle(dev->of_node,
			"mxc-node", 0);
	if(mxc_node == NULL) {
		dev_err(dev, "Missing mxc-endpoint node\n");
		return -EINVAL;
	}
	mxc_plat = of_find_device_by_node(mxc_node);
	if(mxc_plat == NULL) {
		dev_err(dev, "mxc-node is not a platform device\n");
		return -EINVAL;
	}

	/*Grab csi_id early because we use it in the v4l2_dev name.*/
	sensor = &data->sensor;
	ret = of_property_read_u32(mxc_node, "csi_id",
					&(sensor->csi));
	if (ret <0) {
		dev_err(dev, "csi id missing or invalid: %d", ret);
		return ret;
	}

	v4l2_dev = &data->v4l2_dev;
	snprintf(v4l2_dev->name,
			sizeof(v4l2_dev->name),
			"mxc-pipeline-cam-%d",sensor->csi);
	v4l2_dev->name[sizeof(v4l2_dev->name)-1] = '\0';
	ret = v4l2_device_register(dev, v4l2_dev);
	if(ret < 0)	{
		dev_err(dev, "Failed to register v4l2_device: %d\n", ret);
		return ret;
	}

	device_lock(&mxc_plat->dev);
	if (!mxc_plat->dev.driver ||
	    !try_module_get(mxc_plat->dev.driver->owner)) {
		ret = -EPROBE_DEFER;
		goto error1;
	}

	ret = -EINVAL;
	/*Parse pipeline*/
	for(mxc_endpoint = v4l2_of_get_next_endpoint(mxc_node, NULL);
		 mxc_endpoint != NULL;
		 mxc_endpoint = v4l2_of_get_next_endpoint(mxc_node, mxc_endpoint)) {
		struct device_node *remote_node;
		remote_node = v4l2_of_get_remote_port_parent(mxc_endpoint);
		ret = walk_mxc_pipeline(data, mxc_node, remote_node, 1);
	}
	if(ret < 0) {
		goto error2;
	}
	/*Have a valid pipeline.  Add the mxc device to the end.*/
	mxc_subdev = platform_get_drvdata(mxc_plat);
	data->subdevs[data->max_subdev] = mxc_subdev;
	ret = v4l2_device_register_subdev(&data->v4l2_dev, mxc_subdev);
	if(ret < 0) {
		dev_err(data->dev, "%s:unable to register mxc-device", __func__);
		goto error2;
	}
	/*Initialize pixel data*/
	ret = _input_device_to_sensor(data);
	if(ret < 0) {
		dev_err(dev, "Unable to get format data from input device\n");
		goto error2;
	}

	/*Since we know the mxc device is OK now, parse its clock information.*/
    /* Not using devm_clk_get because we don't use the clock ourselves.
	 * It should have been gotten and enabled by the mxc subdevice.
	 */
	sensor->sensor_clk = devm_clk_get(&mxc_plat->dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		ret = PTR_ERR(sensor->sensor_clk);
		dev_err(dev, "clock-frequency missing or invalid: %d\n", ret);
		goto error2;
	}
	ret = of_property_read_u32(mxc_node, "mclk",&sensor->mclk);
	if(ret < 0) {
		dev_err(dev, "mclk missing or invalid: %d\n", ret);
		goto error2;
	}

	ret = of_property_read_u32(mxc_node, "mclk_source",
					(u32 *) &(sensor->mclk_source));
	if (ret < 0) {
		dev_err(dev, "mclk_source missing or invalid: %d\n", ret);
		goto error2;
	}

	ret = of_property_read_u32(mxc_node, "vdev", &sensor->vdev);
	if (ret) {
		dev_err(dev, "vdev missing or invalid\n");
		return ret;
	}

	ret = _run_from_source(data, subdev_init, NULL);
	if (ret < 0) {
		dev_err(dev, "unable to initialize subdevices: %d", ret);
		goto error2;
	}

	/*Dont know if capability is meaningful.*/
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	/*register v4l2_int_dev*/
	v4l2_int_dev = &data->v4l2_int_dev;
	v4l2_int_dev->module = THIS_MODULE;
	snprintf(v4l2_int_dev->name,
			sizeof(v4l2_int_dev->name),
			"mxc-pipeline-cam-%d",sensor->csi);
	v4l2_int_dev->name[sizeof(v4l2_int_dev->name)-1] = '\0';
	v4l2_int_dev->type = v4l2_int_type_slave;

	v4l2_int_slave= &data->v4l2_int_slave;
	ioctl_desc_array = devm_kzalloc(data->dev, sizeof(mxc_pipeline_ioctl_desc),
					GFP_KERNEL);
	if (ioctl_desc_array == NULL) {
		dev_err(data->dev, "could not allocate ioctl array");
		ret = -ENOMEM;
		goto error2;
	}
	memcpy(ioctl_desc_array, mxc_pipeline_ioctl_desc, sizeof(mxc_pipeline_ioctl_desc));
	v4l2_int_slave->ioctls = ioctl_desc_array;
	v4l2_int_slave->num_ioctls = ARRAY_SIZE(mxc_pipeline_ioctl_desc);
	v4l2_int_dev->u.slave = v4l2_int_slave;

	v4l2_int_dev->priv = sensor;

	mutex_unlock(&data->lock);
	ret = v4l2_int_device_register(&data->v4l2_int_dev);
	mutex_lock(&data->lock);
	if(ret < 0) {
		dev_err(dev,"%s:unable to register v4l2_int device", __func__);
		goto error2;
	}

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if(ret < 0) {
		dev_err(dev,"Cannot create sysfs group\n");
		goto error2;
	}

	ret = v4l2_device_register_subdev_nodes(&data->v4l2_dev);
	if(ret < 0) {
		dev_err(dev,"Cannot register subdev nodes\n");
		goto error2;
	}
	data->operational = true;
error2:
	module_put(mxc_plat->dev.driver->owner);
error1:
	device_unlock(&mxc_plat->dev);
	put_device(&mxc_plat->dev);
	if (ret < 0)
		v4l2_device_unregister(&data->v4l2_dev);
	return ret;
}

static void _mxc_pipeline_remove(struct mxc_pipeline_data *data)
{
	int ret;

	data->operational = false;
	ret = _run_from_sink(data, subdev_s_stream_off, NULL);
	if(ret < 0) {
		dev_err(data->dev, "Unable to stop video pipeline, removing anyway:%d\n", ret);
	}
	sysfs_remove_group(&data->dev->kobj, &attr_group);
	dev_dbg(data->dev, "Calling %s", "v4l2_int_device_unregister");
	v4l2_int_device_unregister(&data->v4l2_int_dev);
	devm_kfree(data->dev, data->v4l2_int_slave.ioctls);
	data->v4l2_int_slave.ioctls = NULL;
	dev_dbg(data->dev, "Calling %s", "v4l2_device_unregister");
	v4l2_device_unregister(&data->v4l2_dev);
}

static ssize_t mxc_pipeline_operational_store(struct device *dev,
                                  struct device_attribute *attr, const char *buf, int count)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);
	unsigned long val;

	if (data == NULL)
		return -ENODEV;

	if(_kstrtoul(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}

	mutex_lock(&data->config_lock);
	mutex_lock(&data->lock);
	if(!!val == data->operational)
		goto out;
	if(data->running) {
		dev_err(dev, "Cannot change operational state while running");
		goto out;
	}
	if(data->operational)
		_mxc_pipeline_remove(data);

	if(val)
		_mxc_pipeline_probe(data);

out:
	mutex_unlock(&data->lock);
	mutex_unlock(&data->config_lock);
	return count;
}

static ssize_t mxc_pipeline_operational_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);

	if (data == NULL)
		return -ENODEV;

	return sprintf(buf, "%d\n",data->operational);
}
static DEVICE_ATTR(operational, 0666, (void *)mxc_pipeline_operational_show, (void *)mxc_pipeline_operational_store);

static struct attribute *init_attributes[] = {
	&dev_attr_operational.attr,
	NULL,
};

static const struct attribute_group init_attr_group = {
	.attrs	= init_attributes,
};

static int mxc_pipeline_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxc_pipeline_data *data;
	int ret;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	data->dev = dev;
	mutex_init(&data->lock);
	mutex_init(&data->config_lock);

	/*Must come before call to v4l2_device_register*/
	platform_set_drvdata(pdev, data);

	mutex_lock(&data->lock);
	ret = _mxc_pipeline_probe(data);
	mutex_unlock(&data->lock);
	if(ret < 0)
		return ret;

	ret = sysfs_create_group(&dev->kobj, &init_attr_group);
	if(ret < 0) {
		dev_err(dev,"Cannot create sysfs group\n");
	}

	return ret;
}

static int mxc_pipeline_remove(struct platform_device *pdev)
{
	struct mxc_pipeline_data *data = platform_get_drvdata(pdev);
	sysfs_remove_group(&data->dev->kobj, &init_attr_group);
	_mxc_pipeline_remove(data);
	return 0;
}

static const struct of_device_id mxc_pipeline_of_match[] = {
	{ .compatible = "fsl,v4l2-subdev-pipeline" },
	{ },
};
MODULE_DEVICE_TABLE(of, mxc_pipeline_of_match);

static struct platform_driver mxc_pipeline_driver = {
	.probe		= mxc_pipeline_probe,
	.remove		= mxc_pipeline_remove,
	.driver = {
		.of_match_table = of_match_ptr(mxc_pipeline_of_match),
		.name		= "mxc_v4l2_subdev_pipeline",
		.owner		= THIS_MODULE,
	}
};

module_platform_driver(mxc_pipeline_driver);

MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("V4L2 subdevice pipeline for mxc cameras");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
