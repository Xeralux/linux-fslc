/*
 * Copyright (C) 2014-2015, Sensity Systems, Inc. All Rights Reserved.
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
#include <linux/media.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include "v4l2-int-device.h"
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <linux/i2c.h>
#include <linux/atomic.h>
#include "mxc_v4l2_capture.h"

static u32 MXC_PIPELINE_XCLK_MIN = 6000000;
static u32 MXC_PIPELINE_XCLK_MAX = 67500000;

#define MAX_PIPELINE_LENGTH 8
#define MAX_SOURCE_CANDIDATES 2

struct mxc_pipeline_data {
	struct device *dev;

	/*
	 * For interfacing with mxc_v4l2_capture
	 */
	struct v4l2_int_slave v4l2_int_slave;
	struct v4l2_int_device v4l2_int_dev;

	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct sensor_data sensor;
	struct media_entity *sink;
	struct device_node *sink_node;

	atomic_t running;
	atomic_t operational;
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

static struct media_entity *next_source(struct mxc_pipeline_data *data,
					struct media_entity *ment)
{
	struct media_entity *source = NULL;
	int i;

	mutex_lock(&data->media_dev.graph_mutex);
	for (i = 0; i < ment->num_links; i++) {
		struct media_link *link = &ment->links[i];
		if ((link->flags & MEDIA_LNK_FL_ENABLED) &&
		    link->sink->entity == ment) {
			source = link->source->entity;
			break;
		}
	}
	mutex_unlock(&data->media_dev.graph_mutex);
	return source;
}

static struct media_entity *next_sink(struct mxc_pipeline_data *data,
				      struct media_entity *ment)
{
	struct media_entity *sink = NULL;
	int i;

	mutex_lock(&data->media_dev.graph_mutex);
	for (i = 0; i < ment->num_links; i++) {
		struct media_link *link = &ment->links[i];
		if ((link->flags & MEDIA_LNK_FL_ENABLED) &&
		    link->source->entity == ment) {
			sink = link->sink->entity;
			break;
		}
	}
	mutex_unlock(&data->media_dev.graph_mutex);
	return sink;
}

static struct media_entity *find_source(struct mxc_pipeline_data *data)
{
	struct media_entity *ment, *lastent;

	for (ment = data->sink, lastent = NULL; ment != NULL; lastent = ment, ment = next_source(data, ment));
	if (lastent == NULL || lastent->num_pads > 1 || (lastent->pads[0].flags & MEDIA_PAD_FL_SOURCE) == 0)
		return NULL;

	return lastent;
}

static int pipeline_length(struct mxc_pipeline_data *data)
{
	struct media_entity *ment;
	int count;

	for (ment = data->sink, count = 0; ment != NULL; ment = next_source(data, ment), count += 1);

	return count;
}

static struct v4l2_subdev *find_source_subdev(struct mxc_pipeline_data *data)
{
	struct media_entity *source = find_source(data);

	return (source == NULL ? NULL : media_entity_to_v4l2_subdev(source));
}

static struct media_entity *find_entity_in_pipeline(struct mxc_pipeline_data *data,
						    struct media_entity *starting_point,
						    int which)
{
	struct media_entity *ment;

	for (ment = starting_point; ment != NULL && which > 0;
	     which -= 1, ment = next_sink(data, ment));

	return ment;
}

typedef enum {
	MXC_RUN_FROM_SOURCE,
	MXC_RUN_FROM_SINK
} pipeline_direction;

static int run_through_pipeline(struct mxc_pipeline_data *data,
				struct media_entity *starting_point,
				pipeline_direction dir,
				int (*dofunc)(struct v4l2_subdev *sd),
				int (*undofunc)(struct v4l2_subdev *sd),
				const char *info)
{
	struct media_entity *ment;
	struct v4l2_subdev *sd;
	int err, ret;

	ret = 0;
	for (ment = starting_point;
	     ment != NULL;
	     ment = (dir == MXC_RUN_FROM_SINK ? next_source(data, ment) : next_sink(data, ment))) {
		sd = media_entity_to_v4l2_subdev(ment);
		err = dofunc(sd);
		dev_dbg(data->dev, "%s: running %s on %s returned %d\n",
			__func__, info, sd->name, err);
		if (err < 0 && err != -ENOIOCTLCMD) {
			ret = err;
			if (undofunc != NULL) {
				struct media_entity *undoent;
				for (undoent = (dir == MXC_RUN_FROM_SINK ? next_sink(data, ment) : next_source(data, ment));
				     undoent != NULL;
				     undoent = (dir == MXC_RUN_FROM_SINK ? next_sink(data, undoent) : next_source(data, undoent))) {
					sd = media_entity_to_v4l2_subdev(undoent);
					dev_dbg(data->dev, "%s: undoing %s on %s\n", __func__, info, sd->name);
					undofunc(sd);
				}
				break;
			}
		}
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

static int _input_device_to_sensor(struct mxc_pipeline_data *data, struct v4l2_subdev *input_device)
{
	struct sensor_data *sensor = &data->sensor;
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
		if(v4l2_match_dv_timings(&timings,
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

	ret = _input_device_to_sensor(data, find_source_subdev(data));

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
	struct v4l2_subdev *input_device = find_source_subdev(data);
	int subdev_count = pipeline_length(data);
	int ret;
	__u32 index = fsize->index;

	if (WARN_ON(input_device == NULL))
		return -ENODEV;

	ret = v4l2_subdev_call(input_device, video, enum_framesizes, fsize);
	if (ret < 0 && fsize->index < (MODE_TEST_START + subdev_count)) {
		fsize->index = 0;
		ret = v4l2_subdev_call(input_device, video, enum_framesizes, fsize);
		fsize->index = index;
	}
	if(ret < 0 && fsize->index < (MODE_TEST_START + subdev_count)) {
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

	if (capturemode >= MODE_TEST_START) {
		test_output = capturemode - MODE_TEST_START;
		if (test_output >= pipeline_length(data)) {
			dev_err(dev,"Invalid test output %d\n", test_output);
			return -EINVAL;
		}
	}

	dev_dbg(dev, "%s: height %d width %d", __func__, format.format.height, format.format.width);

	/*Stop subdevices before changing anything.*/
	ret = run_through_pipeline(data, data->sink, MXC_RUN_FROM_SINK,
				   subdev_s_stream_off, subdev_s_stream_on,
				   "subdev_s_stream_off");
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
		struct media_entity *test_ent = find_entity_in_pipeline(data, find_source(data), test_output);
		if (test_ent == NULL)
			ret = -ENODEV;
		else
			ret = v4l2_subdev_call(media_entity_to_v4l2_subdev(test_ent),
					       video, s_routing, 1, 0, 0);
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
	ret = run_through_pipeline(data, find_source(data),
				   MXC_RUN_FROM_SOURCE,
				   subdev_s_stream_on, subdev_s_stream_off,
				   "subdev_s_stream_on");
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

/*
 * XXX - Do we even need this?  Doesn't look like this status
 *       is used anywhere.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct mxc_pipeline_data *data = v4l2_int_to_mxc_pipeline(s);
	atomic_xchg(&data->running, (on ? 1 : 0));
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

static struct v4l2_subdev *get_subdev_for_i2c_device(struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct v4l2_subdev *sd = NULL;

	if (client == NULL)
		return NULL;
	if (!device_trylock(&client->dev))
		return NULL;
	if (client->dev.driver && try_module_get(client->dev.driver->owner)) {
		sd = i2c_get_clientdata(client);
		module_put(client->dev.driver->owner);
	}
	device_unlock(&client->dev);
	return sd;
}

static int find_remotes(struct device_node *node,
			struct device_node *visited[],
			int visitcount,
			struct device_node *remotes[],
			int max_remotes)
{
	struct device_node *cur_endpoint, *next_endpoint, *remote_node;
	int i, remote_count = 0;

	for (cur_endpoint = v4l2_of_get_next_endpoint(node, NULL);
	     cur_endpoint != NULL; cur_endpoint = next_endpoint) {
		remote_node = v4l2_of_get_remote_port_parent(cur_endpoint);
		for (i = 0; i < visitcount && remote_node != visited[i]; i++);
		if (i < visitcount) {
			of_node_put(remote_node);
			next_endpoint = v4l2_of_get_next_endpoint(node, cur_endpoint);
			of_node_put(cur_endpoint);
			continue;
		}
		if (remotes != NULL) {
			if (remote_count >= max_remotes) {
				for (i = 0; i < remote_count; i++)
					of_node_put(remotes[i]);
				of_node_put(cur_endpoint);
				of_node_put(remote_node);
				return -E2BIG;
			}
			remotes[remote_count] = remote_node;
		} else
			of_node_put(remote_node);
		remote_count += 1;
		next_endpoint = v4l2_of_get_next_endpoint(node, cur_endpoint);
		of_node_put(cur_endpoint);
	}

	return remote_count;
}

static int find_all_subdevs(struct mxc_pipeline_data *data,
			    struct v4l2_subdev *subdevs[MAX_PIPELINE_LENGTH])
{
	struct device *dev = data->dev;
	struct device_node *cur_node, *remote_node, *visited[MAX_PIPELINE_LENGTH];
	struct v4l2_subdev *remote_subdev;
	int visitcount, subdev_count;

	/*
	 * Trace the path through the device tree.  Each node is expected to have
	 * exactly one possible outward (towards the sensor) link, except for the
	 * second-to-last node, which connects to one of MAX_SOURCE_CANDIDATES possible
	 * devices.
	 */
	subdev_count = 1;
	subdevs[0] = media_entity_to_v4l2_subdev(data->sink);
	for (cur_node = data->sink_node, visitcount = 0;
	     visitcount < ARRAY_SIZE(visited) && cur_node != NULL;
	     cur_node = remote_node) {
		struct device_node *possible_remotes[MAX_SOURCE_CANDIDATES];
		int i, remote_count;

		/*
		 * Find all possible remotes connected here
		 */
		visited[visitcount++] = cur_node;
		remote_count = find_remotes(cur_node, visited, visitcount,
					    possible_remotes, ARRAY_SIZE(possible_remotes));
		if (remote_count < 0) {
			of_node_put(cur_node);
			dev_dbg(dev, "find_remotes error: %d\n", remote_count);
			return remote_count;
		}
		/*
		 * Are any of the remotes available?  We need to walk through all of
		 * the candidates to at least drop the of_node refcount, so no
		 * early exit from the loop.
		 */
		remote_subdev = NULL;
		remote_node = NULL;
		for (i = 0; i < remote_count; i++) {
			if (remote_subdev == NULL) {
				remote_subdev = get_subdev_for_i2c_device(possible_remotes[i]);
				if (remote_subdev != NULL)
					remote_node = of_node_get(possible_remotes[i]);
			}
			of_node_put(possible_remotes[i]);
		}

		if (remote_count > 0 && remote_subdev == NULL) {
			of_node_put(cur_node);
			return -ENODEV;
		} else if (remote_subdev != NULL) {
			if (subdev_count < MAX_PIPELINE_LENGTH)
				subdevs[subdev_count++] = remote_subdev;
			else
				dev_err(dev, "Too many subdevs in pipeline\n");
		}
		of_node_put(cur_node);
	}

	dev_dbg(dev, "%s: exiting, subdev_count is %d\n", __func__, subdev_count);

	return subdev_count;
}

static int create_links(struct mxc_pipeline_data *data, struct v4l2_subdev *subdevs[], int subdev_count)
{
	struct media_entity *source, *sink;
	u16 source_pad, sink_pad;
	int i, ret;
	for (i = 1; i < subdev_count; i++) {
		sink = &subdevs[i-1]->entity;
		for (sink_pad = 0;
		     sink_pad < sink->num_pads &&
			     (sink->pads[sink_pad].flags & MEDIA_PAD_FL_SINK) == 0;
		     sink_pad++);
		if (sink_pad >= sink->num_pads)
			return -ENOENT;
		source = &subdevs[i]->entity;
		for (source_pad = 0;
		     source_pad < source->num_pads &&
			     (source->pads[source_pad].flags & MEDIA_PAD_FL_SOURCE) == 0;
		     source_pad++);
		if (source_pad >= source->num_pads)
			return -ENOENT;
		ret = media_entity_create_link(source, source_pad,
					       sink, sink_pad,
					       (MEDIA_LNK_FL_ENABLED|MEDIA_LNK_FL_IMMUTABLE));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int subdev_init(struct v4l2_subdev *sd)
{
	return v4l2_subdev_call(sd, core, init, 0);
}

static int setup_v4l2_int_device(struct mxc_pipeline_data *data)
{
	struct v4l2_int_device *v4l2_int_dev = &data->v4l2_int_dev;
	struct v4l2_int_slave  *v4l2_int_slave = &data->v4l2_int_slave;
	struct v4l2_int_ioctl_desc *ioctl_desc_array;

	v4l2_int_dev->module = THIS_MODULE;
	strlcpy(v4l2_int_dev->name, data->v4l2_dev.name, sizeof(v4l2_int_dev->name));
	v4l2_int_dev->type = v4l2_int_type_slave;

	ioctl_desc_array = devm_kzalloc(data->dev, sizeof(mxc_pipeline_ioctl_desc),
					GFP_KERNEL);
	if (ioctl_desc_array == NULL) {
		dev_err(data->dev, "could not allocate ioctl array");
		return -ENOMEM;
	}
	memcpy(ioctl_desc_array, mxc_pipeline_ioctl_desc, sizeof(mxc_pipeline_ioctl_desc));
	v4l2_int_slave->ioctls = ioctl_desc_array;
	v4l2_int_slave->num_ioctls = ARRAY_SIZE(mxc_pipeline_ioctl_desc);
	v4l2_int_dev->u.slave = v4l2_int_slave;

	v4l2_int_dev->priv = &data->sensor;

	return v4l2_int_device_register(&data->v4l2_int_dev);
}

static int create_pipeline(struct mxc_pipeline_data *data)
{
	struct v4l2_subdev *subdevs[MAX_PIPELINE_LENGTH];
	int ret, i, subdev_count;

	memset(&data->media_dev, 0, sizeof(data->media_dev));
	data->media_dev.dev = data->dev;
	strlcpy(data->media_dev.model, "Sensity Camera Board", sizeof(data->media_dev.model));
	ret = media_device_register(&data->media_dev);
	if (ret < 0) {
		dev_err(data->dev, "could not register media device: %d\n", ret);
		return ret;
	}
	memset(&data->v4l2_dev, 0, sizeof(data->v4l2_dev));
	snprintf(data->v4l2_dev.name, sizeof(data->v4l2_dev.name), "mxc-pipeline-cam-%d",
		 data->sensor.csi);
	data->v4l2_dev.mdev = &data->media_dev;
	ret = v4l2_device_register(data->dev, &data->v4l2_dev);
	if (ret < 0) {
		dev_err(data->dev, "could not register v4l2 device: %d\n", ret);
		goto cleanup_media_device;
	}

	subdev_count = find_all_subdevs(data, subdevs);
	if (subdev_count < 0) {
		dev_warn(data->dev, "failed to find all subdevices (%d)\n", subdev_count);
		ret = subdev_count;
		goto cleanup_v4l2_device;
	}

	for (i = subdev_count-1, ret = 0; i >= 0 && ret == 0; i--) {
		dev_dbg(data->dev, "registering subdevice %s\n", subdevs[i]->name);
		ret = v4l2_device_register_subdev(&data->v4l2_dev, subdevs[i]);
	}
	if (ret != 0) {
		dev_err(data->dev, "error registering v4l2 subdevs: %d\n", ret);
		goto cleanup_v4l2_device;
	}
	ret = create_links(data, subdevs, subdev_count);
	if (ret < 0) {
		dev_err(data->dev, "error setting up media_links: %d\n", ret);
		goto cleanup_v4l2_device;
	}
	ret = v4l2_device_register_subdev_nodes(&data->v4l2_dev);
	if (ret < 0) {
		dev_err(data->dev, "error registering device nodes for subdevs: %d\n", ret);
		goto cleanup_v4l2_device;
	}

	ret = _input_device_to_sensor(data, subdevs[subdev_count-1]);
	if (ret < 0) {
		dev_err(data->dev, "could not get format data from iput device\n");
		goto cleanup_v4l2_device;
	}

	ret = run_through_pipeline(data, &subdevs[subdev_count-1]->entity,
				   MXC_RUN_FROM_SOURCE,
				   subdev_init, NULL, "subdev_init");
	if (ret < 0) {
		dev_err(data->dev, "could not initialize subdevices: %d\n", ret);
		goto cleanup_v4l2_device;
	}

	ret = setup_v4l2_int_device(data);
	if (ret < 0) {
		dev_err(data->dev, "could not initialize v4l2-int device: %d\n", ret);
		goto cleanup_v4l2_device;
	}
	return 0;

cleanup_v4l2_device:
	v4l2_device_unregister(&data->v4l2_dev);
cleanup_media_device:
	media_device_unregister(&data->media_dev);
	return ret;
}

static int teardown_pipeline(struct mxc_pipeline_data *data)
{
	run_through_pipeline(data, data->sink, MXC_RUN_FROM_SINK,
			     subdev_s_stream_off, NULL,
			     "subdev_s_stream_off (teardown)");
	v4l2_int_device_unregister(&data->v4l2_int_dev);
	devm_kfree(data->dev, data->v4l2_int_slave.ioctls);
	data->v4l2_int_slave.ioctls = NULL;
	v4l2_device_unregister(&data->v4l2_dev);
	media_device_unregister(&data->media_dev);
	return 0;
}

static ssize_t mxc_pipeline_operational_store(struct device *dev, struct device_attribute *attr,
					      const char *buf, int count)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);
	unsigned long val;

	if (WARN_ON(!data))
		return -ENODEV;

	if(_kstrtoul(buf, 10, &val) || val > 1)
		return -EINVAL;

	if (atomic_cmpxchg(&data->operational, 1-val, val) == 1-val) {
		int ret;
		dev_dbg(dev, "%s: switching to %soperational state\n",
			__func__, (val ? "" : "non-"));
		if (val)
			ret = create_pipeline(data);
		else
			ret = teardown_pipeline(data);
		if (ret < 0) {
			dev_dbg(dev, "%s: failed to change operational state\n",
				__func__);
			atomic_xchg(&data->operational, 1-val);
			return ret;
		}
	} else
		dev_dbg(dev, "%s: no change in operational state\n", __func__);

	return count;
}

static ssize_t mxc_pipeline_operational_show(struct device *dev,
					     struct device_attribute *attr, char *buf)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->operational));

}
static DEVICE_ATTR(operational, 0666, (void *)mxc_pipeline_operational_show, (void *)mxc_pipeline_operational_store);

static ssize_t mxc_pipeline_subdevs_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct mxc_pipeline_data* data = dev_to_mxc_pipeline(dev);
	struct media_entity *ent, *source;
	int count = 0;

	if (atomic_read(&data->operational) == 0)
		return -EINVAL;

	source = find_source(data);
	if (source == NULL)
		return -ENODEV;
	for (ent = source; ent != NULL; ent = next_sink(data, ent))
		count += snprintf(&buf[count], PAGE_SIZE-count,"%s\n",
				  media_entity_to_v4l2_subdev(ent)->name);

	return count;
}
static DEVICE_ATTR(subdevs, 0444, (void *)mxc_pipeline_subdevs_show, NULL);

static struct attribute *attributes[] = {
	&dev_attr_operational.attr,
	&dev_attr_subdevs.attr,
	NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int mxc_pipeline_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxc_pipeline_data *data;
	struct device_node *mxc_node;
	struct platform_device *mxc_plat;
	struct v4l2_subdev *mxc_plat_subdev;
	u32 csi_id, mclk_source;
	int ret;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	/*
	 * Find the "sink" end of the pipeline in the device tree.
	 * This is the node closest to the CPU.
	 */
	mxc_node = of_parse_phandle(dev->of_node, "mxc-node", 0);
	if (mxc_node == NULL) {
		dev_err(dev, "Missing mxc-endpoint node\n");
		return -EINVAL;
	}
	mxc_plat = of_find_device_by_node(mxc_node);
	if (mxc_plat == NULL) {
		dev_err(dev, "mxc-node is not a platform device\n");
		return -EINVAL;
	}
	device_lock(&mxc_plat->dev);
	if (!mxc_plat->dev.driver || !try_module_get(mxc_plat->dev.driver->owner)) {
		ret = -EPROBE_DEFER;
		goto cleanup_unlock;
	}
	ret = of_property_read_u32(mxc_node, "csi_id", &csi_id);
	if (ret < 0) {
		dev_err(dev, "csi id missing or invalid: %d", ret);
		goto cleanup_mod_put;
	}

	mxc_plat_subdev = platform_get_drvdata(mxc_plat);
	if (mxc_plat_subdev == NULL) {
		ret = -EPROBE_DEFER;
		goto cleanup_mod_put;
	}

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto cleanup_mod_put;
	}
	atomic_set(&data->running, 0);
	atomic_set(&data->operational, 0);
	data->dev = dev;
	data->sensor.csi = csi_id;
	data->sink = &mxc_plat_subdev->entity;
	data->sink_node = mxc_node;

	platform_set_drvdata(pdev, data);

	data->sensor.sensor_clk = devm_clk_get(&mxc_plat->dev, "csi_mclk");
	if (IS_ERR(data->sensor.sensor_clk)) {
		dev_err(dev, "clock frequency missing or invalid: %d\n", ret);
		goto cleanup_free_mem;
	}
	ret = of_property_read_u32(mxc_node, "mclk", &data->sensor.mclk);
	if (ret < 0) {
		dev_err(dev, "mclk property missing or invalid: %d\n", ret);
		goto cleanup_sensor_clk;
	}
	ret = of_property_read_u32(mxc_node, "mclk_source", &mclk_source);
	if (ret < 0) {
		dev_err(dev, "mclk_source property missing or invalid: %d\n", ret);
		goto cleanup_sensor_clk;
	}
	data->sensor.mclk_source = mclk_source;
	ret = of_property_read_u32(mxc_node, "vdev", &data->sensor.vdev);
	if (ret < 0) {
		dev_err(dev, "vdev property missing or invalid: %d\n", ret);
		goto cleanup_sensor_clk;
	}

	/* Don't know if capability is meaningful. */
	data->sensor.streamcap.capability = (V4L2_MODE_HIGHQUALITY |
					     V4L2_CAP_TIMEPERFRAME);

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if (ret == 0)
		goto cleanup_mod_put;
	dev_err(dev, "Cannot create sysfs group\n");

cleanup_sensor_clk:
	devm_clk_put(dev, data->sensor.sensor_clk);
cleanup_free_mem:
	devm_kfree(dev, data);
cleanup_mod_put:
	module_put(mxc_plat->dev.driver->owner);
cleanup_unlock:
	device_unlock(&mxc_plat->dev);
	return ret;
}

static int mxc_pipeline_remove(struct platform_device *pdev)
{
	struct mxc_pipeline_data *data = platform_get_drvdata(pdev);
	sysfs_remove_group(&data->dev->kobj, &attr_group);
	if (atomic_read(&data->operational))
		teardown_pipeline(data);
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
