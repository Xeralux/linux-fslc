#ifndef __ap000_param_sysfs__
#define __ap000_param_sysfs__

#include "max_ap0100_func.h"

#ifdef CONFIG_SYSFS
#define SENSOR_UPDATE_UUID 		"2c3fc110-ab3c-11e3-a5e2-0800200c9a66"
#define SENSOR_READ_TEMPERATURE_UUID 	"ec3d5630-ab46-11e3-a5e2-0800200c9a66"
#define SENSOR_READ_ERROR_CNT_UUID	"13f071e0-b91a-11e3-a5e2-0800200c9a66"
#define SENSOR_READ_UUID		"941b5580-b21a-11e3-a5e2-0800200c9a66"

enum SENSOR_SYSFS_STATUS {
	SSS_UPDATE = 0,
	SSS_READ_TEMPERATURE,
	SSS_READ_ERROR_CNT,
	SSS_SENSOR_READ,
	SSS_IDLE,
};

static enum SENSOR_SYSFS_STATUS cur_sss_status = SSS_IDLE;
static signed char cur_temp, min_temp, max_temp, detected_err_cnt, corrected_err_cnt;
static int sensor_read_len = 0;
static int update_init = 0;
static ssize_t sensor_sysfs_read(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	s32 ret;
	u16 cmd_status;
	int retry;

	switch (cur_sss_status) {
		case SSS_UPDATE:
			pca954x_select_channel(I2C_MUX_CHAN);
			if (!update_init) {
				//camera_power_cycle(camera_plat);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
				camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
				msleep(1000);
				max927x_init();
				update_init = 1;
			}
			retry = 0;
			while (retry < 5) {
				cmd_status = ap0100_m034_cmd_status();

				if ( cmd_status != -1)
					break;
				printk("%s, cmd retry\n", __func__);
				retry++;
			}
			pca954x_release_channel();
			return snprintf(buf, PAGE_SIZE, "%d\n", cmd_status);
			break;
		case SSS_READ_TEMPERATURE:
			pca954x_select_channel(I2C_MUX_CHAN);
			retry = 0;
			while (retry < 5) {
				ret = ap0100_m034_read_temperature(&cur_temp, &min_temp, &max_temp);
				if ( ret != -1)
					break;
				printk("%s, cmd retry\n", __func__);
				retry++;
			}

			pca954x_release_channel();

			if (ret != 0) {
				return snprintf(buf, PAGE_SIZE, "SSMN camera (%s), read temperature error\n\n", SSMN_CHANNEL);
			}
			else
				return snprintf(buf, PAGE_SIZE,
					"SSMN camera (%s)\n"
					"Current Temperature: %d\n"
					"Min Temperature: %d\n"
					"Max Temperatuer: %d\n\n",
					SSMN_CHANNEL,
					cur_temp, min_temp, max_temp);
			break;
		case SSS_READ_ERROR_CNT:
			pca954x_select_channel(I2C_MUX_CHAN);
			ret = max9272_read_error_cnt(&detected_err_cnt, &corrected_err_cnt);
			pca954x_release_channel();

			if (ret != 0) {
				return snprintf(buf, PAGE_SIZE, "SSMN camera (%s), read error count error\n\n", SSMN_CHANNEL);
			}
			else
				return snprintf(buf, PAGE_SIZE,
					"SSMN camera (%s)\n"
					"Detected error count: %d\n"
					"Corrected error count %d\n\n",
					SSMN_CHANNEL,
					detected_err_cnt, corrected_err_cnt);
			break;
		case SSS_SENSOR_READ:
			break;
		case SSS_IDLE:
			break;
	}

	return 0;

}

static enum SENSOR_SYSFS_STATUS check_buf_command(const char * buf)
{
	int i;
	char *sensor_update_uuid = SENSOR_UPDATE_UUID ;
	char *sensor_read_temperature_uuid = SENSOR_READ_TEMPERATURE_UUID;
	char *sensor_read_error_cnt_uuid = SENSOR_READ_ERROR_CNT_UUID;
	char *sensor_read_uuid = SENSOR_READ_UUID;

	for (i=0; i<sizeof(sensor_update_uuid)-1; i++) {
		if (buf[i] != sensor_update_uuid[i]) {
			break;
		}
	}
	if ( i == sizeof(sensor_update_uuid)-1)
		return SSS_UPDATE;

	for (i=0; i<sizeof(sensor_read_uuid)-1; i++) {
		if (buf[i] != sensor_read_uuid[i]) {
			break;
		}
	}
	if ( i == sizeof(sensor_read_uuid)-1)
		return SSS_SENSOR_READ;

	for (i=0; i<sizeof(sensor_read_temperature_uuid)-1; i++) {
		if (buf[i] != sensor_read_temperature_uuid[i]) {
			break;
		}
	}
	if ( i == sizeof(sensor_read_temperature_uuid)-1)
		return SSS_READ_TEMPERATURE;

	for (i=0; i<sizeof(sensor_read_error_cnt_uuid)-1; i++) {
		if (buf[i] != sensor_read_error_cnt_uuid[i]) {
			break;
		}
	}
	if ( i == sizeof(sensor_read_error_cnt_uuid)-1)
		return SSS_READ_ERROR_CNT;

	return SSS_IDLE;

}

static ssize_t sensor_sysfs_write(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	u8  read_buf[256];
	u16 reg_addr;
	int retry;

	cur_sss_status  = check_buf_command(buf);

	switch (cur_sss_status  ) {
		case SSS_UPDATE :
			pca954x_select_channel(I2C_MUX_CHAN);
			mdelay(1);
			if (!update_init) {
				//camera_power_cycle(camera_plat);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
				camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
				msleep(1000);
				max927x_init();
				update_init = 1;
			}

			retry = 0;
			while (retry < 5) {
				if ( 0 ==ap0100_m034_cmd_write((char*)(buf + sizeof(SENSOR_UPDATE_UUID) -1),
						count-sizeof(SENSOR_UPDATE_UUID)+1 ) )
					break;
				printk("%s, cmd retry\n", __func__);
				retry++;
			}
			pca954x_release_channel();
			break;
		case SSS_READ_TEMPERATURE:
			printk("received SSS_READ_TEMPRATURE cmd\n");
			break;
		case SSS_READ_ERROR_CNT:
			printk("received SSS_READ_ERROR_CNT cmd\n");
			break;
		case SSS_SENSOR_READ:
			sensor_read_len = (buf[sizeof(SENSOR_READ_UUID)-1]) & 0x00ff;
			reg_addr = ((buf[sizeof(SENSOR_READ_UUID)-1 + 1] << 8) & 0xff00) \
				| (buf[sizeof(SENSOR_READ_UUID)-1 + 2] & 0x00ff);

			pca954x_select_channel(I2C_MUX_CHAN);
			if (!update_init) {
				//camera_power_cycle(camera_plat);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
				camera_power_cycle(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				camera_power_cycle(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
				msleep(1000);
				max927x_init();
				update_init = 1;
			}

			retry = 0;
			while (retry < 5) {
				if ( 0 == ap0100_m034_cmd_read(reg_addr, read_buf, sensor_read_len) )
					break;
				printk("%s, cmd retry\n", __func__);
				retry++;
			}
			pca954x_release_channel();
			break;
		case SSS_IDLE:
			update_init = 0;
			printk("sensor sysfs command idle!\n");
			break;
	}

	return count;
}

static DEVICE_ATTR(ap0100_param, 0666, (void *)sensor_sysfs_read, (void *)sensor_sysfs_write);

static int add_ap0100_param(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dev_attr_ap0100_param);
	if (err < 0)
		goto fail;

fail:
	return err;
}

static void remove_ap0100_param(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ap0100_param);
}

#else
#define add_ap0100_param(dev) 0
#define remove_ap0100_param(dev) do {} while (0)
#endif /* CONFIG_SYSFS */

#endif
