#ifndef __ap000_param_sysfs__
#define __ap000_param_sysfs__

#include "max_ap0100_func.h"

#ifdef CONFIG_SYSFS
#define SENSOR_UPDATE_UUID 		"2c3fc110-ab3c-11e3-a5e2-0800200c9a66"
#define SENSOR_READ_UUID		"941b5580-b21a-11e3-a5e2-0800200c9a66"
#define SENSOR_SET_CMD_UUID					"23917fa2-bc3a-11e3-a5e2-0800200c9a66"

enum AP0100_SET_CMD {
	CMD_NO_FLIP_NO_MIRROR 		= 0,
	CMD_FLIP_IMAGE 	= 1,
	CMD_MIRROR_IMAGE	= 2,
	CMD_FLIP_N_MIRROR	= 3,
	CMD_WDR_MODE		= 4,
	CMD_SDR_MODE		= 5,
	CMD_INIT_MODE_BUF	= 0xFF,
};

enum SENSOR_SYSFS_STATUS {
	SSS_UPDATE = 0,
	SSS_SENSOR_READ,
	SSS_SENSOR_SET_CMD,
	SSS_IDLE,
};

static enum SENSOR_SYSFS_STATUS cur_sss_status = SSS_IDLE;
static int sensor_read_len = 0;
static int update_init = 0;
static u8  read_buf[256];

static ssize_t max9272_rev_trf_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	unsigned long val;
	if(_kstrtoul(buf, 10, &val) || val > 3) {
		dev_err(dev,"Must supply value between 0-3.");
		return count;
	}
	max9272_magic &= ~MAX9272_REG_15_REV_TRF_MASK;
	max9272_magic |= val << MAX9272_REG_15_REV_TRF_SHIFT;

	pca954x_select_channel(I2C_MUX_CHAN);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
	max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
	max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
	pca954x_release_channel();
	return count;
}

static ssize_t max9272_rev_trf_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	u8 val = (max9272_magic & MAX9272_REG_15_REV_TRF_MASK);
	val >>= MAX9272_REG_15_REV_TRF_SHIFT;

	switch(val) {
	case 0:
		return sprintf(buf, "0:211ns,100ns\n");
	case 1:
		return sprintf(buf, "1:281ns,200ns\n");
	case 2:
		return sprintf(buf, "2:422ns,300ns\n");
	case 3:
		return sprintf(buf, "3:563ns,400ns\n");
	};
	return 0;
}
static DEVICE_ATTR(magic_rev_trf, 0666, (void *)max9272_rev_trf_show, (void *)max9272_rev_trf_store);

static ssize_t max9271_rev_logain_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	unsigned long val;
	if(_kstrtoul(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.");
		return count;
	}
	max9271_magic &= ~MAX9271_REG_08_REV_LOGAIN_MASK;
	max9271_magic |= val << MAX9271_REG_08_REV_LOGAIN_SHIFT;

	pca954x_select_channel(I2C_MUX_CHAN);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
	max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
	max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
	pca954x_release_channel();
	return count;
}

static ssize_t max9271_rev_logain_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	u8 val = (max9271_magic & MAX9271_REG_08_REV_LOGAIN_MASK);
	val >>= MAX9271_REG_08_REV_LOGAIN_SHIFT;

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR(magic_rev_logain, 0666, (void *)max9271_rev_logain_show, (void *)max9271_rev_logain_store);

static ssize_t max927x_i2c_slvsh_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	unsigned long val;
	if(_kstrtoul(buf, 10, &val) || val > 3) {
		dev_err(dev,"Must supply value between 0-3.");
		return count;
	}
	max927x_i2c &=       ~MAX927X_REG_0D_I2CSLVSH_MASK;
	max927x_i2c |= val << MAX927X_REG_0D_I2CSLVSH_SHIFT;

	pca954x_select_channel(I2C_MUX_CHAN);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
	max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
	max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
	pca954x_release_channel();
	return count;
}

static ssize_t max927x_i2c_slvsh_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	u8 val = (max927x_i2c & MAX927X_REG_0D_I2CSLVSH_MASK);
	val >>=                 MAX927X_REG_0D_I2CSLVSH_SHIFT;

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR(i2c_slvsh, 0666, (void *)max927x_i2c_slvsh_show, (void *)max927x_i2c_slvsh_store);

static ssize_t max927x_i2c_mstbt_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	unsigned long val;
	if(_kstrtoul(buf, 10, &val) || val > 7) {
		dev_err(dev,"Must supply value between 0-7.");
		return count;
	}
	max927x_i2c &=       ~MAX927X_REG_0D_I2CMSTBT_MASK;
	max927x_i2c |= val << MAX927X_REG_0D_I2CMSTBT_SHIFT;

	pca954x_select_channel(I2C_MUX_CHAN);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
	max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
	max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
	pca954x_release_channel();
	return count;
}

static ssize_t max927x_i2c_mstbt_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	u8 val = (max927x_i2c & MAX927X_REG_0D_I2CMSTBT_MASK);
	val >>=                 MAX927X_REG_0D_I2CMSTBT_SHIFT;

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR(i2c_mstbt, 0666, (void *)max927x_i2c_mstbt_show, (void *)max927x_i2c_mstbt_store);

static ssize_t max9272_revtxamp_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	unsigned long val;
	if(_kstrtoul(buf, 10, &val) || val > 7) {
		dev_err(dev,"Must supply value between 0-7.\n");
		return count;
	}

	max9272_magic &= ~MAX9272_REG_15_REVTXAMP_MASK;
	max9272_magic |= val << MAX9272_REG_15_REVTXAMP_SHIFT;

	pca954x_select_channel(I2C_MUX_CHAN);
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
	max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
	max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
	pca954x_release_channel();
	return count;
}

static ssize_t max9272_revtxamp_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

	u8 val = (max9272_magic & MAX9272_REG_15_REVTXAMP_MASK);
	val >>= MAX9272_REG_15_REVTXAMP_SHIFT;

	return sprintf(buf, "%d:%dmV\n",val,val*10+30);
}
static DEVICE_ATTR(magic_revtxamp, 0666, (void *)max9272_revtxamp_show, (void *)max9272_revtxamp_store);

static ssize_t max9272_show_regs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	u8 i;
	u8 val;
	int err;
	pca954x_select_channel(I2C_MUX_CHAN);
	for(i=0; i < 0x20 && cnt < PAGE_SIZE; i++) {
		err = max9272_read_reg(i, &val);
		if(err)
			continue;
		cnt += snprintf(buf+cnt,PAGE_SIZE-cnt,"%02x:%02x\n", i, val);
	}
	pca954x_release_channel();
	return cnt;
}
static DEVICE_ATTR(max9272_regs, 0444, (void *)max9272_show_regs, NULL);

static ssize_t max9271_show_regs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	u8 i;
	u8 val;
	int err;
	pca954x_select_channel(I2C_MUX_CHAN);
	for(i=0; i < 0x20 && cnt < PAGE_SIZE; i++) {
		err = max9271_read_reg(i, &val);
		if(err)
			continue;
		cnt += snprintf(buf+cnt,PAGE_SIZE-cnt,"%02x:%02x\n", i, val);
	}
	pca954x_release_channel();
	return cnt;
}
static DEVICE_ATTR(max9271_regs, 0444, (void *)max9271_show_regs, NULL);

static ssize_t max9272_show_errors(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	static signed char detected_err_cnt, corrected_err_cnt;
	s32 ret;
	pca954x_select_channel(I2C_MUX_CHAN);
	ret = max9272_read_error_cnt(&detected_err_cnt, &corrected_err_cnt);
	pca954x_release_channel();

	if (ret != 0) {
		return snprintf(buf, PAGE_SIZE, "SSMN camera (%s), read error count error\n\n", SSMN_CHANNEL);
	}

	return snprintf(buf, PAGE_SIZE,
			"SSMN camera (%s)\n"
			"Detected error count: %d\n"
			"Corrected error count %d\n\n",
			SSMN_CHANNEL,
			detected_err_cnt, corrected_err_cnt);
}
static DEVICE_ATTR(max9272_errors, 0444, (void *)max9272_show_errors, NULL);

static ssize_t ap0100_show_temp(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, retry;
	static signed char cur_temp, min_temp, max_temp;
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
	return snprintf(buf, PAGE_SIZE,
			"SSMN camera (%s)\n"
			"Current Temperature: %d\n"
			"Min Temperature: %d\n"
			"Max Temperature: %d\n\n",
				SSMN_CHANNEL,
				cur_temp, min_temp, max_temp);
}
static DEVICE_ATTR(ap0100_temp, 0444, (void *)ap0100_show_temp, NULL);

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
				max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
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
		case SSS_SENSOR_READ:
			if ( sensor_read_len > 0) {
				memcpy(buf, read_buf, sensor_read_len);
				ret = sensor_read_len;
				sensor_read_len = 0;
				return ret;
			} else
				return 0;
			break;
		case SSS_SENSOR_SET_CMD:
			printk("%s, SSS_SENSOR_SET_CMD \n", __func__);
			pca954x_select_channel(I2C_MUX_CHAN);
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
		case SSS_IDLE:
			break;
	}

	return 0;

}

static enum SENSOR_SYSFS_STATUS check_buf_command(const char * buf)
{
	int i;
	char *sensor_update_uuid = SENSOR_UPDATE_UUID ;
	char *sensor_read_uuid = SENSOR_READ_UUID;
	char *sensor_set_cmd_uuid = SENSOR_SET_CMD_UUID;

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

	for (i=0; i<sizeof(sensor_set_cmd_uuid)-1; i++) {
		if (buf[i] != sensor_set_cmd_uuid[i]) {
			break;
		}
	}
	if ( i == sizeof(sensor_set_cmd_uuid)-1)
		return SSS_SENSOR_SET_CMD;
	
	return SSS_IDLE;

}

static ssize_t sensor_sysfs_write(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	u16 reg_addr;
	unsigned char sensor_cmd;
	int retry;

	cur_sss_status  = check_buf_command(buf);

	switch (cur_sss_status  ) {
		case SSS_UPDATE :
			pca954x_select_channel(I2C_MUX_CHAN);
			//mdelay(1);
			if (!update_init) {
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
				max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
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
		case SSS_SENSOR_READ:
			sensor_read_len = (buf[sizeof(SENSOR_READ_UUID)-1]) & 0x00ff;
			reg_addr = ((buf[sizeof(SENSOR_READ_UUID)-1 + 1] << 8) & 0xff00) \
				| (buf[sizeof(SENSOR_READ_UUID)-1 + 2] & 0x00ff);

			pca954x_select_channel(I2C_MUX_CHAN);
			if (!update_init) {
#if (I2C_MUX_CHAN == I2C_MUX_CHAN_CSI0)
				max927x_init(ssmn_mipi_powerdown, ssmn_mipi_tc_reset);
#else
				max927x_init(ssmn_parallel_powerdown, ssmn_parallel_tc_reset);
#endif
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
		case SSS_SENSOR_SET_CMD:
			if ( count < sizeof(SENSOR_READ_UUID))
				break;
			
			sensor_cmd = (buf[sizeof(SENSOR_READ_UUID)-1 + 1]);

			printk("%s, SSS_SENSOR_SET_CMD, read cmd %d\n", __func__, sensor_cmd );

			if (sensor_cmd == CMD_INIT_MODE_BUF) {
				ap0100_m034_sensor_update_init_buf((unsigned char *)buf + sizeof(SENSOR_READ_UUID) + 1, count - sizeof(SENSOR_READ_UUID)-1);
			} else {
				pca954x_select_channel(I2C_MUX_CHAN);

				retry = 0;
				while (retry < 5) {
					if ( 0 == ap0100_m034_sensor_set_cmd(sensor_cmd) )
						break;
					printk("%s, cmd retry\n", __func__);
					retry++;
				}
				pca954x_release_channel();
			}
			break;
		case SSS_IDLE:
			update_init = 0;
			printk("sensor sysfs command idle!\n");
			break;
	}

	return count;
}

static DEVICE_ATTR(ap0100_param, 0666, (void *)sensor_sysfs_read, (void *)sensor_sysfs_write);

static struct attribute *attributes[] = {
		&dev_attr_ap0100_param.attr,
		&dev_attr_max9272_regs.attr,
		&dev_attr_max9271_regs.attr,
		&dev_attr_ap0100_temp.attr,
		&dev_attr_max9272_errors.attr,
		&dev_attr_magic_rev_trf.attr,
		&dev_attr_magic_revtxamp.attr,
		&dev_attr_magic_rev_logain.attr,
		&dev_attr_i2c_slvsh.attr,
		&dev_attr_i2c_mstbt.attr,
		NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};

static int add_ap0100_param(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &attr_group);
}

static void remove_ap0100_param(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &attr_group);
}

#else
#define add_ap0100_param(dev) 0
#define remove_ap0100_param(dev) do {} while (0)
#endif /* CONFIG_SYSFS */

#endif
