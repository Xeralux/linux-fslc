/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define SSMN_PARALLEL_VOLTAGE_ANALOG               2800000
#define SSMN_PARALLEL_VOLTAGE_DIGITAL_CORE         1500000
#define SSMN_PARALLEL_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define SSMN_PARALLEL_XCLK_MIN 6000000
#define SSMN_PARALLEL_XCLK_MAX 67500000 //24000000

#define I2C_MUX_CHAN_CSI0	1 // MIPI-CSI2 => CSI#1 (i.MX6DL)
#define I2C_MUX_CHAN_CSI1	0 // MIPI-CSI2 => Parallel->CSI#0 (i.MX6DL)

#define I2C_MUX_CHAN	I2C_MUX_CHAN_CSI1
#define SSMN_CHANNEL 	"parallel"
#include "ap0100_param_sysfs.h"

enum ssmn_parallel_mode {
	ssmn_parallel_mode_MIN = 0,
	ssmn_parallel_mode_720P_1280_720 = 0,
	ssmn_parallel_mode_960P_1280_960 = 1,
	ssmn_parallel_mode_VGA_640_480 = 2,
	ssmn_parallel_mode_TEST_1280_720 = 3,
	ssmn_parallel_mode_I2C_TEST_1 = 4,
	ssmn_parallel_mode_I2C_TEST_2 = 5,
	ssmn_parallel_mode_SENSOR_TEST_MODE = 6,
	ssmn_parallel_mode_MAX = 6,
	ssmn_parallel_mode_INIT = 0xff, /*only for sensor init*/
};

enum ssmn_parallel_frame_rate {
	ssmn_parallel_15_fps,
	ssmn_parallel_30_fps
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ssmn_parallel_mode_info {
	enum ssmn_parallel_mode mode;
	u32 width;
	u32 height;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ssmn_parallel_data;

static struct ssmn_parallel_mode_info ssmn_parallel_mode_info_data[2][ssmn_parallel_mode_MAX + 1] = {
	{
		{ssmn_parallel_mode_720P_1280_720, 1280, 720},
		{ssmn_parallel_mode_960P_1280_960, 1280, 960},
		{ssmn_parallel_mode_VGA_640_480, 640,  480},
		{ssmn_parallel_mode_TEST_1280_720, 1280, 720},
		{ssmn_parallel_mode_I2C_TEST_1, 640, 480},
		{ssmn_parallel_mode_I2C_TEST_2, 640, 480},
		{ssmn_parallel_mode_SENSOR_TEST_MODE, 1280, 720},
	},
	{
		{ssmn_parallel_mode_720P_1280_720, 1280, 720},
		{ssmn_parallel_mode_960P_1280_960, 1280, 960},
		{ssmn_parallel_mode_VGA_640_480, 640,  480},
		{ssmn_parallel_mode_TEST_1280_720, 1280, 720},
		{ssmn_parallel_mode_I2C_TEST_1, 640, 480},
		{ssmn_parallel_mode_I2C_TEST_2, 640, 480},
		{ssmn_parallel_mode_SENSOR_TEST_MODE, 1280, 720},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct regmap *gpr;

static int ssmn_parallel_probe(struct i2c_client *adapter,
                               const struct i2c_device_id *device_id);
static int ssmn_parallel_remove(struct i2c_client *client);

#define IMX6DL_GPR13_IPU_CSI0_MUX (0x07 << 0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0 (0x0)
#define IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0 (0x4)

static void ssmn_parallel_powerdown(int powerdown)
{

	if (powerdown) {
		gpio_direction_output(MEDIANODE_CAM_PARALLEL_PWR, 0);
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_MIPI_CSI0);
	}
	else {
		gpio_direction_input(MEDIANODE_CAM_PARALLEL_PWR);
		if (of_machine_is_compatible("fsl,imx6dl"))
			regmap_update_bits(gpr,IOMUXC_GPR13,
					IMX6DL_GPR13_IPU_CSI0_MUX, IMX6DL_GPR13_IPU_CSI0_MUX_IPU_CSI0);
	}
	//mdelay(5);
	msleep(1100);
}

// Toshiba Mipi-bridge chip
static void ssmn_parallel_tc_reset(int reset)
{
	if (reset) {
		gpio_direction_output(MEDIANODE_PARALLEL_TC_RESET, 0);
	}
	else {
		gpio_direction_input(MEDIANODE_PARALLEL_TC_RESET);
	}
	//mdelay(5);
	mdelay(500);
}

static void ssmn_parallel_sensor_io_init(void)
{
#if 0
	if (of_machine_is_compatible("fsl,imx6q"))
		mxc_iomux_v3_setup_multiple_pads(mx6dl_medianode_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_csi0_sensor_pads));
	else if (of_machine_is_compatible("fsl,imx6dl"))
		mxc_iomux_v3_setup_multiple_pads(mx6dl_medianode_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_medianode_mipi_sensor_pads));
#endif

	ssmn_parallel_powerdown(1);
	ssmn_parallel_powerdown(0);
}

static int ssmn_parallel_init_mode(enum ssmn_parallel_frame_rate frame_rate,
			    enum ssmn_parallel_mode mode)
{
	int retval = 0, init_retry;
	s32 ret=0;
	int i2c_test_cycles = 1000000, i2c_r_fail_cnt, i2c_w_fail_cnt;
	int i2c_w_retry_cnt, i2c_r_retry_cnt;

	pr_debug("%s, mode = %d\n", __func__, mode);

	if (mode > ssmn_parallel_mode_MAX || mode < ssmn_parallel_mode_MIN) {
		pr_err("Wrong ssmn_parallel mode detected!\n");
		return -1;
	}

	ssmn_parallel_data.pix.width = ssmn_parallel_mode_info_data[frame_rate][mode].width;
	ssmn_parallel_data.pix.height = ssmn_parallel_mode_info_data[frame_rate][mode].height;

	camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);

	pca954x_select_channel(I2C_MUX_CHAN);
	// within select channel, we need to call pca954x_release_channel() after I2C use is done
	if (mode < ssmn_parallel_mode_TEST_1280_720  || mode == ssmn_parallel_mode_SENSOR_TEST_MODE) {
		init_retry = 0;
		while ( init_retry < INIT_RETRY) {
			if (max_ap0100_init(mode, 0) == 0)
				break;
			init_retry++;
			camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
			pca954x_release_channel();
			pca954x_select_channel(I2C_MUX_CHAN);
		}
		if (init_retry == INIT_RETRY) {
			pr_err("init_mode: init mode failed after %d times retry\n", init_retry);
			retval = -1;
		}

		tc_mipi_bridge_dev_init(0); // 0: mipi output
	} else if ( mode == ssmn_parallel_mode_TEST_1280_720) {
		tc_mipi_bridge_dev_init(3); // 3: mipi test output for parallel
	}else if ( mode != ssmn_parallel_mode_INIT) { // I2C tests
		max927x_init();
		//ap0100_hw_reset();

		// max9272 I2C test
/*		pr_debug("max9272 i2c test running ...... \n");
		ret = max9272_I2C_test(i2c_test_cycles, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
		if ( ret == 0) {
			pr_debug("max9272 i2c test passed !, total test count = %d\n", i2c_test_cycles);
		} else {
			pr_debug("max9272 i2c test FAILED!write failure count = %d, read failure count = %d, total test count = %d\n",
				i2c_w_fail_cnt,  i2c_r_fail_cnt,  i2c_test_cycles);
			pca954x_release_channel();
			return -1;
		}
*/
		if (mode == ssmn_parallel_mode_I2C_TEST_1) {
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
		} else if (mode == ssmn_parallel_mode_I2C_TEST_2) {
			// ap0100 I2C test
			pr_debug("AP0100  i2c test running ...... \n");
			ret = ap0100_m034_I2C_test(i2c_test_cycles, &i2c_w_retry_cnt, & i2c_r_retry_cnt, &i2c_w_fail_cnt, &i2c_r_fail_cnt);
			if (ret == 0) {
				pr_debug("ap0100 i2c test passed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
			} else {
				pr_debug("ap0100 i2c test failed !, total test count = %d, w_retry_cnt = %d, r_retry_cnt = %d, w_failure = %d, r_failure = %d\n", \
					i2c_test_cycles, i2c_w_retry_cnt, i2c_r_retry_cnt, i2c_w_fail_cnt, i2c_r_fail_cnt);
				pca954x_release_channel();
				return -1;
			}
		}

		tc_mipi_bridge_dev_init(3); // 3: mipi test output for parallel

	}

	pca954x_release_channel();

	pr_debug("%s done: width = %d, height = %d\n", __func__, ssmn_parallel_data.pix.width, ssmn_parallel_data.pix.height);

	if (ssmn_parallel_data.pix.width == 0 || ssmn_parallel_data.pix.height == 0)
		return -EINVAL;

	pr_debug("%s done: frame-rate=%d, mode=%d\n", __func__, frame_rate, mode);

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
	p->u.bt656.clock_curr = ssmn_parallel_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ssmn_parallel_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = SSMN_PARALLEL_XCLK_MIN;
	p->u.bt656.clock_max = SSMN_PARALLEL_XCLK_MAX;
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
		//msleep(2000);
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
	enum ssmn_parallel_frame_rate frame_rate;
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
			frame_rate = ssmn_parallel_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ssmn_parallel_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = ssmn_parallel_init_mode(frame_rate,
				       sensor->streamcap.capturemode);
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
		vc->value = ssmn_parallel_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ssmn_parallel_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ssmn_parallel_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ssmn_parallel_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ssmn_parallel_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ssmn_parallel_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ssmn_parallel_data.ae_mode;
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

	pr_debug("In ssmn_parallel:ioctl_s_ctrl %d\n",
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
	if (fsize->index > ssmn_parallel_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ssmn_parallel_data.pix.pixelformat;
	fsize->discrete.width =
			max(ssmn_parallel_mode_info_data[0][fsize->index].width,
			    ssmn_parallel_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ssmn_parallel_mode_info_data[0][fsize->index].height,
			    ssmn_parallel_mode_info_data[1][fsize->index].height);
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
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ssmn_parallel_camera");

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
	if (fmt->index > ssmn_parallel_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ssmn_parallel_data.pix.pixelformat;

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
	enum ssmn_parallel_frame_rate frame_rate;

	ssmn_parallel_data.on = true;

	/* mclk */
	tgt_xclk = ssmn_parallel_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)SSMN_PARALLEL_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)SSMN_PARALLEL_XCLK_MIN);
	ssmn_parallel_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz for source %d\n",
	             tgt_xclk / 1000000, ssmn_parallel_data.mclk_source);
	//set_mclk_rate(&ssmn_parallel_data.mclk, ssmn_parallel_data.mclk_source);

	/* Default camera frame rate is set in probe */
	pr_debug("   denominator=%d, numerator=%d\n",
	         sensor->streamcap.timeperframe.denominator,
	         sensor->streamcap.timeperframe.numerator);
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;
	pr_debug("   %s: frame rate=%dfps\n", __func__, tgt_fps);

	if (tgt_fps == 15)
		frame_rate = ssmn_parallel_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ssmn_parallel_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

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
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ssmn_parallel_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave ssmn_parallel_slave = {
	.ioctls = ssmn_parallel_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ssmn_parallel_ioctl_desc),
};

static struct v4l2_int_device ssmn_parallel_int_device = {
	.module = THIS_MODULE,
	.name = "ssmn_parallel",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ssmn_parallel_slave,
	},
};

/*!
 * ssmn_parallel I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ssmn_parallel_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval, init_retry;
#if 0
	u8 chip_id_high, chip_id_low;
#endif

        pr_debug("ssmn_parallel_probe: i2c_client.addr=0x%02X, i2c_client.name=%s\n",
		 client->addr, client->name);

	/* Set initial values for the sensor struct. */
	memset(&ssmn_parallel_data, 0, sizeof(ssmn_parallel_data));
	ssmn_parallel_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ssmn_parallel_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ssmn_parallel_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ssmn_parallel_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ssmn_parallel_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ssmn_parallel_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ssmn_parallel_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	gpr = syscon_regmap_lookup_by_phandle(dev->of_node,
			"gpr");
	if (IS_ERR(gpr)) {
		dev_err(dev, "missing gpr\n");
		return -1;
	}

	retval = gpio_request(MEDIANODE_CAM_PARALLEL_PWR, "camera parallel pwr");
	if(retval < 0)
		return retval;

	retval = gpio_request(MEDIANODE_PARALLEL_TC_RESET, "camera parallel tc rst");
	if(retval < 0)
		goto error1;

	clk_prepare_enable(ssmn_parallel_data.sensor_clk);

	ssmn_parallel_data.io_init = ssmn_parallel_sensor_io_init;
	ssmn_parallel_data.i2c_client = client;
	ssmn_parallel_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	ssmn_parallel_data.pix.width = 640;
	ssmn_parallel_data.pix.height = 480;
	ssmn_parallel_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ssmn_parallel_data.streamcap.capturemode = 0;
	ssmn_parallel_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ssmn_parallel_data.streamcap.timeperframe.numerator = 1;

	//clk_disable_unprepare(ssmn_parallel_data.sensor_clk);

	camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);

	pca954x_select_channel(I2C_MUX_CHAN);

	init_retry = 0;
	while (init_retry < INIT_RETRY) {
		if (max_ap0100_init(0, 0) == 0)
			break;
		init_retry++;
		camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
		pca954x_release_channel();
		pca954x_select_channel(I2C_MUX_CHAN);
	}
	if (init_retry == INIT_RETRY) {
		pr_err("parallel probe: init mode failed after %d retries\n", init_retry);
		retval = -1;
	}

	tc_mipi_bridge_dev_init(3); // 3: mipi test pattern for parallel

	pca954x_release_channel();

	ssmn_parallel_int_device.priv = &ssmn_parallel_data;
	retval = v4l2_int_device_register(&ssmn_parallel_int_device);

	add_ap0100_param((&client->dev));

	printk("ssmn_parallel_probe done\n");
	return retval;

error1:
	gpio_free(MEDIANODE_CAM_PARALLEL_PWR);
	return retval;
}

/*!
 * ssmn_parallel I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ssmn_parallel_remove(struct i2c_client *client)
{
	remove_ap0100_param(&client->dev);

	v4l2_int_device_unregister(&ssmn_parallel_int_device);

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
	gpio_free(MEDIANODE_PARALLEL_TC_RESET);
	gpio_free(MEDIANODE_CAM_PARALLEL_PWR);
	return 0;
}

static struct of_device_id ssmn_parallel_dt_ids[] = {
	{ .compatible = "sensity,ssmn_parallel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ssmn_parallel_dt_ids);

static const struct i2c_device_id ssmn_parallel_i2c_id[] = {
	{ "ssmn_parallel", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ssmn_parallel_i2c_id);

static struct i2c_driver ssmn_parallel_driver = {
	.driver = {
		   .name = "ssmn_parallel",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ssmn_parallel_dt_ids),
		   },
	.probe = ssmn_parallel_probe,
	.remove = ssmn_parallel_remove,
	.id_table = ssmn_parallel_i2c_id,
};

module_i2c_driver(ssmn_parallel_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SSMN_PARALLEL Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
