/*
 * Copyright (C) 2014-2015 Sensity Systems, Inc. All Rights Reserved.
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/media.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/media-entity.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>
#include <linux/mipi_csi2.h>

struct mxc_subdev_mipi_cam {
	struct device *dev;
	struct regmap *gpr;
	struct v4l2_subdev subdev;
	struct media_pad pad;
	void *mipi_csi2_info;
	struct clk *sensor_clk;
};


static struct mxc_subdev_mipi_cam *to_mxc_subdev_mipi_from_v4l2(const struct v4l2_subdev *sd)
{
	return  container_of(sd, struct mxc_subdev_mipi_cam, subdev);
}

static struct mxc_subdev_mipi_cam *to_mxc_subdev_mipi_from_dev(const struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	return container_of(sd, struct mxc_subdev_mipi_cam, subdev);
}


static ssize_t err1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_dev(dev);
	return snprintf(buf,PAGE_SIZE,"%04x\n",mipi_csi2_get_error1(data->mipi_csi2_info));
}
static DEVICE_ATTR(err1, 0444, (void *)err1_show, NULL);

static ssize_t err2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_dev(dev);
	return snprintf(buf,PAGE_SIZE,"%04x\n",mipi_csi2_get_error2(data->mipi_csi2_info));
}
static DEVICE_ATTR(err2, 0444, (void *)err2_show, NULL);

static struct attribute *attributes[] = {
		&dev_attr_err1.attr,
		&dev_attr_err2.attr,
		NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

#define MIPI_CSI2_PHY_STATE_CLK_STOP (1 << 10)
#define MIPI_CSI2_PHY_STATE_DDR_CLK (1 << 8)
#define MIPI_CSI2_PHY_STATE_D1_STOP  (1 << 5)
#define MIPI_CSI2_PHY_STATE_D0_STOP  (1 << 4)
#define MIPI_CSI2_PHY_STATE_STOPPED \
	(MIPI_CSI2_PHY_STATE_CLK_STOP | \
	 MIPI_CSI2_PHY_STATE_D1_STOP | \
	 MIPI_CSI2_PHY_STATE_D0_STOP)

static bool mipi_dphy_receiving_clock(struct mxc_subdev_mipi_cam *cam)
{
	int i;
	u32 mipi_reg;

	mipi_reg = mipi_csi2_dphy_status(cam->mipi_csi2_info);
	for(i = 0;i < 40; i++) {
		dev_dbg(cam->dev, "%s phy state: 0x%x\n", __func__, mipi_reg);
		if((mipi_reg & MIPI_CSI2_PHY_STATE_DDR_CLK) == MIPI_CSI2_PHY_STATE_DDR_CLK)
			return true;
		msleep(5);
		mipi_reg = mipi_csi2_dphy_status(cam->mipi_csi2_info);
	}
	return false;
}

#define IMX6DL_GPR13_IPU_CSI0_MUX (0x07 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0 (0x0 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0 (0x4 << 0)

static void mxc_csi1_mipicsi0_input_enable(struct mxc_subdev_mipi_cam *data, int enable)
{
	dev_dbg(data->dev, "%s: enable %d", __func__, enable);

	if (!enable) {
		if (of_machine_is_compatible("fsl,imx6q"))
			regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<19),(1 << 19));
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0);
	}
	else {
		if (of_machine_is_compatible("fsl,imx6q"))
			regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<19),(0 << 19));
		else if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0);
	}
}

static int mxc_subdev_mipi_s_stream_on(struct mxc_subdev_mipi_cam *data)
{
	struct device *dev = data->dev;
	u32 mipi_reg;
	unsigned int i;

	clk_prepare_enable(data->sensor_clk);
	mxc_csi1_mipicsi0_input_enable(data, 1);

	/* wait for mipi sensor ready */

	if(!mipi_dphy_receiving_clock(data)) {
		dev_err(dev, "mipi csi2 can not receive sensor clk!\n");
		return -EIO;
	}
	dev_dbg(dev, "mipi dphy status OK\n");

	/* wait for mipi stable */
	mipi_reg = mipi_csi2_get_error1(data->mipi_csi2_info);
	for(i = 0; i < 10 && (mipi_reg != 0x0); i++) {
		dev_dbg(dev, "%s err1 state: 0x%x\n", __func__, mipi_reg);
		mipi_reg = mipi_csi2_get_error1(data->mipi_csi2_info);
		msleep(10);
	}

	if(mipi_reg != 0x0) {
		dev_err(dev, "mipi csi2 can not receive data correctly!\n");
		return -EIO;
	}
	dev_dbg(dev,"mipi dphy no error\n");
	return 0;
}

static int mxc_subdev_mipi_s_stream_off(struct mxc_subdev_mipi_cam *data);

int mxc_subdev_mipi_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_v4l2(sd);
	if(enable) {
		return mxc_subdev_mipi_s_stream_on(data);
	}
	return mxc_subdev_mipi_s_stream_off(data);
}

static int mxc_subdev_mipi_s_stream_off(struct mxc_subdev_mipi_cam *data)
{
	struct device *dev = data->dev;
	u32 mipi_reg;
	unsigned int i;

	if(mipi_dphy_receiving_clock(data))
		return 0;

	dev_dbg(dev, "mipi_csi2_set_lanes, mipi_csi2_reset");
	mipi_csi2_set_lanes(data->mipi_csi2_info);
	mipi_csi2_reset(data->mipi_csi2_info);

	mipi_reg = mipi_csi2_dphy_status(data->mipi_csi2_info);
	for (i=0;i < 20;i++) {
		dev_dbg(dev, "%s phy state: 0x%x\n", __func__ , mipi_reg);
		if((mipi_reg & MIPI_CSI2_PHY_STATE_STOPPED) == MIPI_CSI2_PHY_STATE_STOPPED)
			break;
		msleep(10);
		mipi_reg = mipi_csi2_dphy_status(data->mipi_csi2_info);
	}
	if((mipi_reg & MIPI_CSI2_PHY_STATE_STOPPED) != MIPI_CSI2_PHY_STATE_STOPPED) {
		dev_err(dev, "%s: phy did not enter stop state: %03x", __func__, mipi_reg);
		return -EIO;
	}
	return 0;
}

struct mbus_mipi_data_mapping {
	enum v4l2_mbus_pixelcode code;
	int datatype;
};

static const int mbus_framefmt_to_mipi_data_type(struct v4l2_mbus_framefmt *fmt)
{
	static const struct mbus_mipi_data_mapping mapping[] = {
		{V4L2_MBUS_FMT_UYVY8_2X8, MIPI_DT_YUV422},

	};
	int i;
	for(i = 0; i < ARRAY_SIZE(mapping); i++) {
		if(mapping[i].code == fmt->code)
			return mapping[i].datatype;
	}
	return -EINVAL;
}

static	int mxc_subdev_mipi_try_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	int ret = mbus_framefmt_to_mipi_data_type(fmt);
	return ret < 0 ? ret : 0;
}

static	int mxc_subdev_mipi_s_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_v4l2(sd);
	int ret = mxc_subdev_mipi_try_mbus_fmt(sd, fmt);
	int datatype;
	if(ret < 0)
		return ret;
	datatype = mbus_framefmt_to_mipi_data_type(fmt);
	if(datatype < 0)
		return datatype;

	mipi_csi2_set_datatype(data->mipi_csi2_info, datatype);
	return 0;
}

int mxc_subdev_mipi_core_init(struct v4l2_subdev *sd, u32 val)
{
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_v4l2(sd);
	clk_prepare_enable(data->sensor_clk);
	mxc_csi1_mipicsi0_input_enable(data, 1);
	return 0;
}

static struct v4l2_subdev_video_ops mxc_subdev_mipi_subdev_video_ops = {
	.s_stream = mxc_subdev_mipi_video_s_stream,
	.try_mbus_fmt = mxc_subdev_mipi_try_mbus_fmt,
	.s_mbus_fmt = mxc_subdev_mipi_s_mbus_fmt,
};

static struct v4l2_subdev_core_ops  mxc_subdev_mipi_subdev_core_ops = {
		.init =  mxc_subdev_mipi_core_init,
};

static struct v4l2_subdev_ops mxc_subdev_mipi_subdev_ops = {
	.video	= &mxc_subdev_mipi_subdev_video_ops,
	.core = &mxc_subdev_mipi_subdev_core_ops,
};


static int mxc_subdev_mipi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev	*subdev;
	struct mxc_subdev_mipi_cam *data;
	int retval;
	int csi;

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	data->dev = dev;

	data->mipi_csi2_info = mipi_csi2_get_info();

	if (!data->mipi_csi2_info) {
		dev_err(dev, "%s: Fail to get mipi_csi2_info!\n",
		       __func__);
		return -ENODEV;
	}

	if (!mipi_csi2_get_status(data->mipi_csi2_info)) {
		dev_dbg(dev, "mipi_csi2_enable");
		mipi_csi2_enable(data->mipi_csi2_info);
	}
	if (!mipi_csi2_get_status(data->mipi_csi2_info)) {
		dev_err(dev, "Can not enable mipi csi2 driver!\n");
		return -EPERM;
	}

	data->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(data->sensor_clk)) {
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(data->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&csi);
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	if(csi != 0) {
		dev_err(dev, "csi other than 0 not supported.\n");
		return -EINVAL;
	}


	data->gpr = syscon_regmap_lookup_by_phandle(dev->of_node,
				"gpr");
	if (IS_ERR(data->gpr))	{
		dev_err(dev, "missing gpr\n");
		return -1;
	}

	retval = sysfs_create_group(&dev->kobj, &attr_group);
	if (retval < 0) {
	   dev_err(dev,"Could not create sysfs file.\n");
	   return retval;
	}

	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &mxc_subdev_mipi_subdev_ops);
	subdev->owner = THIS_MODULE;
	v4l2_set_subdevdata(subdev, data);
	platform_set_drvdata(pdev, subdev);
	snprintf(subdev->name, sizeof(subdev->name), "%s-%d",
		dev->driver->name,csi);
	subdev->name[sizeof(subdev->name)-1] = 0;
	data->pad.flags = MEDIA_PAD_FL_SINK;
	retval = media_entity_init(&subdev->entity, 1, &data->pad, 0);
	if (retval < 0)
		dev_err(dev, "Error initializing media entity: %d\n", retval);

	return retval;
}

static int mxc_subdev_mipi_remove(struct platform_device *pdev)
{
	struct v4l2_subdev	*sd = platform_get_drvdata(pdev);
	struct mxc_subdev_mipi_cam *data = to_mxc_subdev_mipi_from_v4l2(sd);

	v4l2_device_unregister_subdev(&data->subdev);
	media_entity_cleanup(&data->subdev.entity);
	sysfs_remove_group(&data->dev->kobj, &attr_group);

	mxc_csi1_mipicsi0_input_enable(data, 0);
	if(mipi_csi2_get_status(data->mipi_csi2_info))
		mipi_csi2_disable(data->mipi_csi2_info);

	return 0;
}

static struct of_device_id mxc_subdev_mipi_dt_ids[] = {
	{ .compatible = "fsl,mxc-subdev-mipi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_subdev_mipi_dt_ids);


static struct platform_driver mxc_subdev_mipi_driver = {
	.driver = {
		   .name = "mxc_subdev_mipi",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mxc_subdev_mipi_dt_ids),
		   },
	.probe = mxc_subdev_mipi_probe,
	.remove = mxc_subdev_mipi_remove,
};

module_platform_driver(mxc_subdev_mipi_driver);

MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("mxc mipi v4l2_subdev driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
