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

#define DEBUG

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


struct mxc_parallel_cam {
	struct device *dev;
	struct regmap *gpr;
	struct v4l2_subdev	subdev;
};

#define IMX6DL_GPR13_IPU_CSI0_MUX (0x07 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0 (0x0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0 (0x4)
static void mxc_parallel_powerdown(struct mxc_parallel_cam *data, int powerdown)
{
	pr_debug("ssmn_camera_parallel: powerdown %d\n", powerdown);
	if (powerdown) {
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0);
	}
	else {
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(data->gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0);
	}
	msleep(1100);
}

static struct v4l2_subdev_ops mxc_parallel_subdev_ops = {
};

static int mxc_parallel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *sensor_clk;
	struct v4l2_subdev	*subdev;
	struct mxc_parallel_cam *data;
	int ret;
	int csi;

	sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor_clk)) {
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&csi);
	if (ret < 0) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}

	if(csi != 0) {
		dev_err(dev, "csi other than 0 not supported.\n");
		return -EINVAL;
	}

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	data->dev = dev;

	data->gpr = syscon_regmap_lookup_by_phandle(dev->of_node,
				"gpr");
	if (IS_ERR(data->gpr))	{
		dev_err(dev, "missing gpr\n");
		return -1;
	}

	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &mxc_parallel_subdev_ops);
	subdev->owner = pdev->dev.driver->owner;
	v4l2_set_subdevdata(subdev, data);
	platform_set_drvdata(pdev, subdev);
	snprintf(subdev->name, sizeof(subdev->name), "%s-%d",
		dev->driver->name,csi);
	subdev->name[sizeof(subdev->name)-1] = 0;

	pr_debug("ssmn_camera_parallel: clk_prepare_enable\n");
	clk_prepare_enable(sensor_clk);
	mxc_parallel_powerdown(data,0);
	return ret;
}

static int mxc_parallel_remove(struct platform_device *pdev)
{
	struct v4l2_subdev	*sd = platform_get_drvdata(pdev);
	struct mxc_parallel_cam *data = container_of(sd, struct mxc_parallel_cam, subdev);

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(&pdev->dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
	}
	mxc_parallel_powerdown(data,1);
	return 0;
}


static struct of_device_id mxc_parallel_dt_ids[] = {
	{ .compatible = "fsl,mxc_parallel_cam" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_parallel_dt_ids);

static struct platform_driver mxc_parallel_driver = {
	.driver = {
		   .name = "mxc_parallel",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mxc_parallel_dt_ids),
		   },
	.probe = mxc_parallel_probe,
	.remove = mxc_parallel_remove,
};

module_platform_driver(mxc_parallel_driver);

MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_DESCRIPTION("mxc parallel v4l2_subdev driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
