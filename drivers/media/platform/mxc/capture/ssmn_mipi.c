/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include "../arch/arm/mach-imx/hardware.h"
#include "../arch/arm/mach-imx/iomux-v3.h"
#include "max_ap0100_func.h"
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>

#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__);
#endif

#define SSMN_VOLTAGE_ANALOG               2800000
#define SSMN_VOLTAGE_DIGITAL_CORE         1500000
#define SSMN_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define SSMN_XCLK_MIN 6000000
#define SSMN_XCLK_MAX 67500000 // 24000000

#define SSMN_CHIP_ID_HIGH_BYTE	0x300A
#define SSMN_CHIP_ID_LOW_BYTE	0x300B

#define I2C_MUX_CHAN_CSI0	1
#define I2C_MUX_CHAN_CSI1	0

#define I2C_MUX_CHAN	I2C_MUX_CHAN_CSI0
#define SSMN_CHANNEL "mipi"
#include "ap0100_param_sysfs.h"

enum ssmn_mipi_mode {
	ssmn_mipi_mode_MIN = 0,
	ssmn_mipi_mode_720P_1280_720 = 0,
	ssmn_mipi_mode_960P_1280_960 = 1,
	ssmn_mipi_mode_VGA_640_480 = 2,
	ssmn_mipi_mode_TEST_1280_720 = 3,
	ssmn_mipi_mode_I2C_TEST_1 = 4,
	ssmn_mipi_mode_I2C_TEST_2 = 5,
	ssmn_mipi_mode_SENSOR_TEST_MODE = 6,
	ssmn_mipi_mode_I2C_TEST_3 = 7,
	ssmn_mipi_mode_I2C_TEST_4 = 8,
	ssmn_mipi_mode_MAX = 8,
	ssmn_mipi_mode_INIT = 0xff, /*only for sensor init*/
};

enum ssmn_mipi_frame_rate {
	ssmn_mipi_15_fps,
	ssmn_mipi_30_fps
};

/* image size under 1280 * 960 are SUBSAMPLING
 * image size upper 1280 * 960 are SCALING
 */
enum ssmn_mipi_downsize_mode {
	SUBSAMPLING,
	SCALING,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ssmn_mipi_mode_info {
	enum ssmn_mipi_mode mode;
	enum ssmn_mipi_downsize_mode dn_mode;
	u32 width;
	u32 height;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ssmn_mipi_data;

static struct ssmn_mipi_mode_info ssmn_mipi_mode_info_data[2][ssmn_mipi_mode_MAX + 1] = {
	{
		{ssmn_mipi_mode_720P_1280_720, SUBSAMPLING, 1280, 720},
		{ssmn_mipi_mode_960P_1280_960, SUBSAMPLING, 1280, 960},
		{ssmn_mipi_mode_VGA_640_480, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_TEST_1280_720, SUBSAMPLING, 1280,  720},
		{ssmn_mipi_mode_I2C_TEST_1, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_I2C_TEST_2, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_SENSOR_TEST_MODE, SUBSAMPLING, 1280,  720},
		{ssmn_mipi_mode_I2C_TEST_3, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_I2C_TEST_4, SUBSAMPLING, 640,  480},
	},
	{
		{ssmn_mipi_mode_720P_1280_720, SUBSAMPLING, 1280, 720},
		{ssmn_mipi_mode_960P_1280_960, SUBSAMPLING, 1280, 960},
		{ssmn_mipi_mode_VGA_640_480, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_TEST_1280_720, SUBSAMPLING, 1280,  720},
		{ssmn_mipi_mode_I2C_TEST_1, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_I2C_TEST_2, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_SENSOR_TEST_MODE, SUBSAMPLING, 1280,  720},
		{ssmn_mipi_mode_I2C_TEST_3, SUBSAMPLING, 640,  480},
		{ssmn_mipi_mode_I2C_TEST_4, SUBSAMPLING, 640,  480},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct regmap *gpr;

static int ssmn_mipi_probe(struct i2c_client *adapter,
                      const struct i2c_device_id *device_id);
static int ssmn_mipi_remove(struct i2c_client *client);

/* if sensor changes inside scaling or subsampling
 * change mode directly
 * */
static int ssmn_mipi_change_mode_direct(enum ssmn_mipi_frame_rate frame_rate,
				enum ssmn_mipi_mode mode)
{
	int retval = 0;

	ssmn_mipi_data.pix.width =
		ssmn_mipi_mode_info_data[frame_rate][mode].width;
	ssmn_mipi_data.pix.height =
		ssmn_mipi_mode_info_data[frame_rate][mode].height;

	if (ssmn_mipi_data.pix.width == 0 || ssmn_mipi_data.pix.height == 0)
		return -EINVAL;

	//TODO: change mode here

	return retval;
}

static void ssmn_mipi_powerdown(int powerdown)
{
	if (powerdown) {
		gpio_direction_output(MEDIANODE_CAM_MIPI_PWR, 0);
		if (of_machine_is_compatible("fsl,imx6q"))
			regmap_update_bits(gpr,IOMUXC_GPR1,(1<<19),1);
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(gpr,IOMUXC_GPR13,0x38,4);
	}
	else {
		gpio_direction_input(MEDIANODE_CAM_MIPI_PWR);
		if (of_machine_is_compatible("fsl,imx6q"))
			regmap_update_bits(gpr,IOMUXC_GPR1,(1<<19),0);
		else if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(gpr,IOMUXC_GPR13,0x38,0);
	}
	msleep(1100);
}

// Toshiba Mipi-bridge chip
static void ssmn_mipi_tc_reset(int reset)
{
	if (reset) {
		gpio_direction_output(MEDIANODE_MIPI_TC_RESET, 0);
	}
	else {
		gpio_direction_input(MEDIANODE_MIPI_TC_RESET);
	}
	mdelay(5);
}

static void ssmn_mipi_sensor_io_init(void)
{
#if 0
	if (of_machine_is_compatible("fsl,imx6q"))
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_csi0_sensor_pads));
	else if (of_machine_is_compatible("fsl,imx6dl"))
		mxc_iomux_v3_setup_multiple_pads(mx6dl_medianode_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_medianode_mipi_sensor_pads));
#endif
	/* For mx6dl, mipi virtual channel 0 connects to csi 1 */
	if (of_machine_is_compatible("fsl,imx6dl"))
		regmap_update_bits(gpr,IOMUXC_GPR13,0x38,0);
}

static int ssmn_mipi_init_mode(enum ssmn_mipi_frame_rate frame_rate,
			    enum ssmn_mipi_mode mode, enum ssmn_mipi_mode orig_mode)
{
	int retval = 0, init_retry;
	void *mipi_csi2_info;
	u32 mipi_reg;
	s32 ret=0;
	int i2c_test_cycles = 1000000, i2c_r_fail_cnt, i2c_w_fail_cnt;
	int i2c_w_retry_cnt, i2c_r_retry_cnt;
	enum ssmn_mipi_downsize_mode dn_mode, orig_dn_mode;

	pr_debug("%s, mode = %d\n", __func__, mode);

	if ((mode > ssmn_mipi_mode_MAX || mode < ssmn_mipi_mode_MIN)
		&& (mode != ssmn_mipi_mode_INIT)) {
		pr_err("Wrong ssmn mode detected!\n");
		return -1;
	}

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
        if (!mipi_csi2_info) {
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
		       __func__, __FILE__);
                return -1;
        }

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

        if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	mipi_csi2_set_lanes(mipi_csi2_info);

	/*Only reset MIPI CSI2 HW at sensor initialize*/
	//if (mode == ssmn_mipi_mode_INIT)
		mipi_csi2_reset(mipi_csi2_info);

	if (ssmn_mipi_data.pix.pixelformat == V4L2_PIX_FMT_UYVY)
		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_YUV422);
	else if (ssmn_mipi_data.pix.pixelformat == V4L2_PIX_FMT_RGB565)
		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RGB565);
	else
		pr_err("currently this sensor format can not be supported!\n");

	dn_mode = ssmn_mipi_mode_info_data[frame_rate][mode].dn_mode;
	orig_dn_mode = ssmn_mipi_mode_info_data[frame_rate][orig_mode].dn_mode;
	if (mode == ssmn_mipi_mode_INIT) {
		ssmn_mipi_data.pix.width = 640;
		ssmn_mipi_data.pix.height = 480;
	} else {
		/* change inside subsampling or scaling
		 * download firmware directly */
		retval = ssmn_mipi_change_mode_direct(frame_rate, mode);
	}

	if (retval < 0)
		goto err;

	camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);

	pca954x_select_channel(I2C_MUX_CHAN);
	// within select channel, we need to call pca954x_release_channel() after I2C use is done
	if (mode < ssmn_mipi_mode_TEST_1280_720 ||mode == ssmn_mipi_mode_SENSOR_TEST_MODE) {
		init_retry = 0;
		while ( init_retry < INIT_RETRY) {
			if (max_ap0100_init(mode, 0) == 0)
				break;
			init_retry++;
			camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
			pca954x_release_channel();
			pca954x_select_channel(I2C_MUX_CHAN);
		}
		if (init_retry == INIT_RETRY) {
			pr_err("init mode failed after %d times retry\n", init_retry);
			retval = -1;
			goto err;
		}

		tc_mipi_bridge_dev_init(0); // 0: mipi output
	} else if ( mode == ssmn_mipi_mode_TEST_1280_720) {
		tc_mipi_bridge_dev_init(1); // 1: mipi test output
	} else if ( mode != ssmn_mipi_mode_INIT) { // I2C tests
		max927x_init();
		if ( mode == ssmn_mipi_mode_I2C_TEST_3) {
			// max9272 I2C test
			pr_debug("max9272 i2c test running ...... \n");
			ret = max9272_I2C_test(i2c_test_cycles, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
			if ( ret == 0) {
				pr_debug("max9272 i2c test passed !total test count = %d\n", i2c_test_cycles);
			} else {
				pr_debug("max9272 i2c test FAILED!write failure count = %d, read failure count = %d, total test count = %d\n",
				i2c_w_fail_cnt,  i2c_r_fail_cnt,  i2c_test_cycles);
				pca954x_release_channel();
				return -1;
			}
		} else if ( mode == ssmn_mipi_mode_I2C_TEST_1) {
			// max9271 I2C test
			pr_debug("max9271 i2c test running ...... \n");
			ret = max9271_I2C_test(i2c_test_cycles, &i2c_w_retry_cnt, & i2c_r_retry_cnt, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
			if ( ret == 0) {
				pr_debug("max9271 i2c test passed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
			} else {
				pr_debug("max9271 i2c test failed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
				pca954x_release_channel();
				return -1;
			}
		} else if ( mode == ssmn_mipi_mode_I2C_TEST_2) {
			// ap0100 I2C test
			pr_debug("AP0100  i2c test running ...... \n");
			ret = ap0100_m034_I2C_test(i2c_test_cycles, &i2c_w_retry_cnt, & i2c_r_retry_cnt, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
			if ( ret == 0) {
				pr_debug("ap0100 i2c test passed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
			} else {
				pr_debug("ap0100 i2c test failed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
				pca954x_release_channel();
				return -1;
			}

		} else if ( mode == ssmn_mipi_mode_I2C_TEST_4) {
			// max927x I2C test
			pr_debug("max927x i2c test running ...... \n");
			ret = max927x_I2C_test(i2c_test_cycles, &i2c_w_retry_cnt, & i2c_r_retry_cnt, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
			if ( ret == 0) {
				pr_debug("max927x i2c test passed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
			} else {
				pr_debug("max927x i2c test failed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
				pca954x_release_channel();
				return -1;
			}
		}

		tc_mipi_bridge_dev_init(1); // 1: mipi test output

	}

	pca954x_release_channel();

	mipi_csi2_dump_reg(mipi_csi2_info);

	if (mipi_csi2_info) {
		unsigned int i;

		for (i=0;i<20;i++) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			//pr_debug("mipi_csi2 state reg: 0x%x\n", mipi_reg);
		}

		i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while ((mipi_reg  != 0x300) && (i < 20)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			//pr_debug("mipi_csi2 state reg: 0x%x\n", mipi_reg);
			i++;
			msleep(10);
		}

		if (i >= 20) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			return -1;
		} else {
			pr_debug("mipi dphy status OK\n");
			}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			return -1;
		} else {
			pr_debug("mipi dphy no error\n");
		}

		mipi_csi2_dump_reg(mipi_csi2_info);

	}
err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ssmn_mipi_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ssmn_mipi_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = SSMN_XCLK_MIN;
	p->u.bt656.clock_max = SSMN_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;

		// wait for the POR delay
		//mdelay(1000);
		//pca954x_select_channel(I2C_MUX_CHAN);

	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ssmn_mipi_frame_rate frame_rate;
	enum ssmn_mipi_mode orig_mode;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = ssmn_mipi_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ssmn_mipi_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
		ret = ssmn_mipi_init_mode(frame_rate,
				(u32)a->parm.capture.capturemode, orig_mode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ssmn_mipi_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ssmn_mipi_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ssmn_mipi_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ssmn_mipi_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ssmn_mipi_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ssmn_mipi_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ssmn_mipi_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In ssmn:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ssmn_mipi_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ssmn_mipi_data.pix.pixelformat;
	fsize->discrete.width =
			max(ssmn_mipi_mode_info_data[0][fsize->index].width,
			    ssmn_mipi_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ssmn_mipi_mode_info_data[0][fsize->index].height,
			    ssmn_mipi_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"ssmn_mipi_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ssmn_mipi_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ssmn_mipi_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ssmn_mipi_frame_rate frame_rate;
	void *mipi_csi2_info;

	ssmn_mipi_data.on = true;

	/* mclk */
	tgt_xclk = ssmn_mipi_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)SSMN_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)SSMN_XCLK_MIN);
	ssmn_mipi_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	//set_mclk_rate(&ssmn_mipi_data.mclk, ssmn_mipi_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ssmn_mipi_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ssmn_mipi_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	mipi_csi2_info = mipi_csi2_get_info();

	/* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "Fail to get mipi_csi2_info!\n");
		return -EPERM;
	}

	return 0;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ssmn_mipi_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave ssmn_mipi_slave = {
	.ioctls = ssmn_mipi_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ssmn_mipi_ioctl_desc),
};

static struct v4l2_int_device ssmn_mipi_int_device = {
	.module = THIS_MODULE,
	.name = "ssmn_mipi",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ssmn_mipi_slave,
	},
};

/*!
 * ssmn I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ssmn_mipi_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval, init_retry;
#if 0
	u8 chip_id_high, chip_id_low;
#endif

	pr_debug("ssmn_mipi_probe\n");

	/* Set initial values for the sensor struct. */
	memset(&ssmn_mipi_data, 0, sizeof(ssmn_mipi_data));
	ssmn_mipi_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ssmn_mipi_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ssmn_mipi_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ssmn_mipi_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ssmn_mipi_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ssmn_mipi_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ssmn_mipi_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	gpr = syscon_regmap_lookup_by_phandle(dev->of_node,
				"gpr");
	if (IS_ERR(gpr))	{
		dev_err(dev, "missing gpr\n");
		return -1;
	}
	retval = gpio_request(MEDIANODE_CAM_MIPI_PWR, "camera mipi pwr");
	if(retval < 0)
		return retval;

	retval = gpio_request(MEDIANODE_MIPI_TC_RESET, "camera mipi tc rst");
	if(retval < 0)
		goto error1;

	clk_prepare_enable(ssmn_mipi_data.sensor_clk);

	ssmn_mipi_data.io_init = ssmn_mipi_sensor_io_init;
	ssmn_mipi_data.i2c_client = client;
	ssmn_mipi_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ssmn_mipi_data.pix.width = 640;
	ssmn_mipi_data.pix.height = 480;
	ssmn_mipi_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ssmn_mipi_data.streamcap.capturemode = 0;
	ssmn_mipi_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ssmn_mipi_data.streamcap.timeperframe.numerator = 1;

	//clk_disable_unprepare(ssmn_mipi_data.sensor_clk);

	camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);

	pca954x_select_channel(I2C_MUX_CHAN);
	if (1) { // within select channel, we need to call pca954x_release_channel() after I2C use is done
		init_retry = 0;
		while (init_retry < INIT_RETRY) {
			if (max_ap0100_init(0, 0) == 0)
				break;
			init_retry++;
			camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
			pca954x_release_channel();
			pca954x_select_channel(I2C_MUX_CHAN);
		}
		if (init_retry == INIT_RETRY) {
			pr_err("mipi probe: init mode failed after %d times retry\n", init_retry);
			retval = -1;
		}

		tc_mipi_bridge_dev_init(0); // 0: mipi output
	}
	pca954x_release_channel();

	ssmn_mipi_int_device.priv = &ssmn_mipi_data;
	retval = v4l2_int_device_register(&ssmn_mipi_int_device);

	add_ap0100_param((&client->dev));

	printk("ssmn_mipi_probe done\n");
	return retval;
error1:
	gpio_free(MEDIANODE_CAM_MIPI_PWR);
	return retval;
}

/*!
 * ssmn I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ssmn_mipi_remove(struct i2c_client *client)
{
	remove_ap0100_param(&client->dev);

	v4l2_int_device_unregister(&ssmn_mipi_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	gpio_free(MEDIANODE_MIPI_TC_RESET);
	gpio_free(MEDIANODE_CAM_MIPI_PWR);
	return 0;
}

static struct of_device_id ssmn_mipi_dt_ids[] = {
	{ .compatible = "sensity,ssmn_mipi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ssmn_mipi_dt_ids);

static const struct i2c_device_id ssmn_mipi_i2c_id[] = {
	{ "ssmn_mipi", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ssmn_mipi_i2c_id);

static struct i2c_driver ssmn_mipi_driver = {
	.driver = {
		   .name = "ssmn_mipi",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ssmn_mipi_dt_ids),
		   },
	.probe = ssmn_mipi_probe,
	.remove = ssmn_mipi_remove,
	.id_table = ssmn_mipi_i2c_id,
};

module_i2c_driver(ssmn_mipi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SSMN MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
