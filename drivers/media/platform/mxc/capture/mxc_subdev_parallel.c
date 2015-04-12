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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>


struct mxc_subdev_parallel_cam {
	struct device *dev;
	struct regmap *gpr;
	struct v4l2_subdev	subdev;
	struct clk *sensor_clk;
	int ipu;
	int csi;
};

static struct mxc_subdev_parallel_cam *to_mxc_subdev_parallel_from_v4l2(const struct v4l2_subdev *sd)
{
	return  container_of(sd, struct mxc_subdev_parallel_cam, subdev);
}

#define IMX6DL_GPR13_IPU_CSI0_MUX (0x07 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0 (0x0 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0 (0x4 << 0)

static void mxc_subdev_parallel_powerdown(struct mxc_subdev_parallel_cam *data, int powerdown)
{
	pr_debug("ssmn_camera_parallel: powerdown %d\n", powerdown);
	if (powerdown) {
		if (of_machine_is_compatible("fsl,imx6q")) {
			if (data->ipu == 0)
				regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<19),(0<<19));
			else
				regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<20),(0<<20));
		}
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0);
	}
	else {
		if (of_machine_is_compatible("fsl,imx6q")) {
			if (data->ipu == 0)
				regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<19),(1<<19));
			else
				regmap_update_bits(data->gpr,IOMUXC_GPR1,(1<<20),(1<<20));
		}
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0);
	}
	msleep(1100);
}

int mxc_subdev_parallel_core_init(struct v4l2_subdev *sd, u32 val)
{
	struct mxc_subdev_parallel_cam *data = to_mxc_subdev_parallel_from_v4l2(sd);
	clk_prepare_enable(data->sensor_clk);
	mxc_subdev_parallel_powerdown(data, 0);
	return 0;
}

int mxc_subdev_parallel_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_subdev_parallel_cam *data = to_mxc_subdev_parallel_from_v4l2(sd);
	if(!enable) {
		return 0;
	}
	clk_prepare_enable(data->sensor_clk);
	mxc_subdev_parallel_powerdown(data, 0);
	return 0;
}

static const struct v4l2_subdev_core_ops mxc_subdev_parallel_subdev_core_ops = {
		.init =  mxc_subdev_parallel_core_init,
};

static struct v4l2_subdev_video_ops mxc_subdev_parallel_subdev_video_ops = {
	.s_stream = mxc_subdev_parallel_video_s_stream,
};

static struct v4l2_subdev_ops mxc_subdev_parallel_subdev_ops = {
		.video = &mxc_subdev_parallel_subdev_video_ops,
		.core = &mxc_subdev_parallel_subdev_core_ops,
};

static int mxc_subdev_parallel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_subdev	*subdev;
	struct mxc_subdev_parallel_cam *data;
	int ret;
	int csi;
	int ipu;

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	data->dev = dev;

	data->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(data->sensor_clk)) {
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(data->sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&csi);
	if (ret < 0) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}

	if(csi != 0 && csi != 1) {
		dev_err(dev, "csi other than 0 or 1 not supported.\n");
		return -EINVAL;
	}

	data->csi = csi;

	ret = of_property_read_u32(dev->of_node, "ipu_id",
							   &ipu);
	if (ret < 0) {
		dev_err(dev, "ipu id missing or invalid\n");
		return ret;
	}

	if(ipu != 0 && ipu != 1) {
		dev_err(dev, "ipu other than 0 or 1 not supported.\n");
		return -EINVAL;
	}

	data->ipu = ipu;

	if (of_machine_is_compatible("fsl,imx6q")) {
		// Quad/Dual: ipu/csi mapping is fixed.
		if ((ipu == 0 && csi == 1) ||
			(ipu == 1 && csi == 0)) {
			dev_err(dev, "ipu/csi combination not supported.\n");
			return -EINVAL;
		}
	} else {
		// Dual-lite and others: only one IPU
		if (ipu != 0) {
			dev_err(dev, "invalid ipu specified.\n");
			return -EINVAL;
		}
	}

	data->gpr = syscon_regmap_lookup_by_phandle(dev->of_node,
				"gpr");
	if (IS_ERR(data->gpr))	{
		dev_err(dev, "missing gpr\n");
		return -1;
	}

	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &mxc_subdev_parallel_subdev_ops);
	subdev->owner = pdev->dev.driver->owner;
	v4l2_set_subdevdata(subdev, data);
	platform_set_drvdata(pdev, subdev);
	snprintf(subdev->name, sizeof(subdev->name), "%s-%d",
		dev->driver->name,csi);
	subdev->name[sizeof(subdev->name)-1] = 0;

	pr_debug("ssmn_camera_parallel: clk_prepare_enable\n");
	return ret;
}

static int mxc_subdev_parallel_remove(struct platform_device *pdev)
{
	struct v4l2_subdev	*sd = platform_get_drvdata(pdev);
	struct mxc_subdev_parallel_cam *data = container_of(sd, struct mxc_subdev_parallel_cam, subdev);

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(&pdev->dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
	}
	mxc_subdev_parallel_powerdown(data,1);
	return 0;
}


static struct of_device_id mxc_subdev_parallel_dt_ids[] = {
	{ .compatible = "fsl,mxc-subdev-parallel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_subdev_parallel_dt_ids);

static struct platform_driver mxc_subdev_parallel_driver = {
	.driver = {
		   .name = "mxc_subdev_parallel",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mxc_subdev_parallel_dt_ids),
		   },
	.probe = mxc_subdev_parallel_probe,
	.remove = mxc_subdev_parallel_remove,
};

module_platform_driver(mxc_subdev_parallel_driver);

MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("mxc parallel v4l2_subdev driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
