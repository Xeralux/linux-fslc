/*
 * Maxim 9721 seralizer/9272 deserializer driver
 *
 * Copyright (c) 2013-2014 Leopard Imaging, Inc.
 * Copyright (c) 2015 Sensity Systems, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The 9271/9272 operate as a pair, with the CPU controlling the pair
 * through the 9272 deserializer for inbound (camera) applications, or
 * through the 9271 serializer for outbound (display) applications.
 *
 * Besides the video stream, the serial link carries a control channel
 * that acts as an extension of the I2C bus.  A GPI-to-GPO signal may
 * also be carried; that functionality is not implemented here.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/media.h>
#include <linux/mutex.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>
#include "max927x.h"

/*
 * Device settings
 */
#define MAX9271_SETTING(name_, regno_, pos_, wid_) MAX9271_##name_,
typedef enum {
	MAX9271_SETTINGS
	MAX9271_NUM_SETTINGS
} max9271_setting_t;
#undef MAX9271_SETTING
#define MAX9271_SETTING(name_, regno_, pos_, wid_) \
	[ MAX9271_##name_ ] = { .name = #name_, .reg = regno_, .pos = pos_, .width = wid_, .idx = MAX9271_##name_  },
static const struct max927x_setting max9271_settings[MAX9271_NUM_SETTINGS] = { MAX9271_SETTINGS };
#undef MAX9271_SETTING

#define MAX9272_SETTING(name_, regno_, pos_, wid_) MAX9272_##name_,
typedef enum {
	MAX9272_SETTINGS
	MAX9272_NUM_SETTINGS
} max9272_setting_t;
#undef MAX9272_SETTING
#define MAX9272_SETTING(name_, regno_, pos_, wid_) \
	[ MAX9272_##name_ ] = { .name = #name_, .reg = regno_, .pos = pos_, .width = wid_, .idx = MAX9272_##name_  },
static const struct max927x_setting max9272_settings[MAX9272_NUM_SETTINGS] = { MAX9272_SETTINGS };
#undef MAX9272_SETTING

/*
 * Device tree and sysfs-accessible configuration controls, named
 * for backward compatibility with old driver.	ORDER IS IMPORTANT
 * to maintain that compatibility.
 *
 * MAX927x_PROPERTY(<name>, <attribute-name>, <setting-name>)
 */
#define MAX9272_PROPERTIES \
	MAX9272_PROPERTY(REV_AMP, magic_rev_amp, REV_AMP) \
	MAX9272_PROPERTY(REV_TRF, magic_rev_trf, REV_TRF)
#define MAX9271_PROPERTIES \
	MAX9271_PROPERTY(REV_DIG_FLT, magic_rev_dig_flt, DIGFLT) \
	MAX9271_PROPERTY(REV_LOGAIN, magic_rev_logain, LOGAIN) \
	MAX9271_PROPERTY(REV_HIGAIN, magic_rev_higain, HIGAIN) \
	MAX9271_PROPERTY(REV_HIBW, magic_rev_hibw, HIBW) \
	MAX9271_PROPERTY(REV_HIVTH, magic_rev_hivth, HIVTH) \
	MAX9271_PROPERTY(I2C_SLVSH, i2c_slvsh, I2CSLVSH) \
	MAX9271_PROPERTY(I2C_MSTBT, i2c_mstbt, I2CMSTBT) \
	MAX9271_PROPERTY(CMLLVL, cmllvl, CMLLVL) \
	MAX9271_PROPERTY(PREEMP, preemp, PREEMP) \
	MAX9271_PROPERTY(SPREAD, spread, SS)

#define MAX9272_PROPERTY(tag_, name_, setting_) CFG_##tag_,
#define MAX9271_PROPERTY(tag_, name_, setting_) CFG_##tag_,
typedef enum {
	MAX9272_PROPERTIES
	MAX9271_PROPERTIES
	NUM_CFG_PROPS
} property_id_t;
#undef MAX9272_PROPERTY
#undef MAX9271_PROPERTY

struct max927x_property {
	int for_deserializer;
	property_id_t propid;
	const struct max927x_setting *setting;
	struct device_attribute dev_attr;
};

/*
 * We maintain an array of counts of I2C transactions
 * that were successful after <n> retries, where n ranges
 * from 0 to I2C_RETRIES_MAX.
 */
#define I2C_RETRIES_MAX 15
#define I2C_RETRIES_DEFAULT 10
static unsigned long i2c_retries = I2C_RETRIES_DEFAULT;
module_param(i2c_retries, ulong, 0644);
MODULE_PARM_DESC(i2c_retries, "Default number of times to retry I2C xfers");

/*
 * Most register updates work on the first try, except
 * in the case where the register is on the remote and
 * communication hasn't been fully established yet.
 */
#define REG_RETRIES 5

static int training_cycles = 1000;
module_param(training_cycles, int, 0644);
MODULE_PARM_DESC(training_cycles, "Number of read cycles to perform during link training");

static int training_threshold = 5;
module_param(training_threshold, int, 0644);
MODULE_PARM_DESC(training_threshold, "Error/retry threshold during link training, in percent");

/*
 * Driver-private data structures
 */

struct i2c_test_data {
	unsigned int cycles;
	unsigned int failures;
	unsigned int write_retries;
	unsigned int read_retries;
	unsigned int mismatches;
	unsigned int max_retries;
	unsigned int transaction_count;
	char tested_device[32];
};

struct max927x;

typedef int (*link_init_fn)(struct max927x *me);

typedef enum {
	STATE_UNINITIALIZED,
	STATE_READY,
	STATE_OPERATIONAL,
} max927x_state_t;

#define I2C_XFER_NORETRIES	(1<<0)
#define I2C_XFER_NOCOUNT	(1<<1)
struct max927x {
	struct device *dev;
	struct i2c_adapter *parent;
	struct i2c_client *local, *remote;
	struct i2c_client *ser, *des;
	struct mutex lock;
	max927x_state_t curstate;
	int training;
	struct work_struct training_work;
	link_init_fn ctrl_link_init;
	struct regulator *remote_power;
	atomic_t remote_power_enabled;
	u32 power_on_mdelay, power_off_mdelay;
	atomic_t ser_gpios_enabled;
	atomic_t ser_gpios_set;
	struct gpio_chip gc;
	struct i2c_adapter dummy_adapter;
	struct i2c_adapter adap;
	struct v4l2_subdev subdev;
	struct device_node *i2c_node;
	struct media_pad pads[2]; /* 0=serializer, 1=deserializer */
	atomic_t i2c_error_count;
	atomic_t i2c_retry_counts[I2C_RETRIES_MAX+1];
	u32 of_cfg_settings[NUM_CFG_PROPS];
	atomic_t i2c_test_in_progress;
	atomic_t i2c_xfer_flags;
	u8 current_logain, current_rev_amp;
	struct i2c_test_data i2c_test_results;
};

/*
 * Pointer translation
 */
static struct max927x *to_max927x_from_dev(struct device *dev)
{
	return container_of(dev_get_drvdata(dev), struct max927x, subdev);
}

static struct max927x *to_max927x_from_v4l2_subdev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max927x, subdev);
}

/*
 * Basic I2C register access
 */
static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 regnum = reg;
	int ntries, ret;

	msgs[0].addr = msgs[1].addr = client->addr;
	msgs[0].flags = 0;
	msgs[1].flags = I2C_M_RD;
	msgs[0].len = msgs[1].len = 1;
	msgs[0].buf = &regnum;
	msgs[1].buf = val;
	for (ntries = 0, ret = -EIO; ntries < REG_RETRIES && ret < 0;
	     ret = i2c_transfer(client->adapter, msgs, 2), ntries += 1);
	return (ret < 0 ? ret : 0);
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[2] = {reg, val};
	int ntries, ret;
	for (ntries = 0, ret = -EIO; ntries < REG_RETRIES && ret < 0;
	     ret = i2c_master_send(client, buf, 2), ntries += 1);
	return (ret < 0 ? ret : 0);
}

/*
 * Serializer settings
 */

/*
 * ser_set_logain:
 *
 * Just writes the LOGAIN setting into the appropriate register,
 * with no checks.  For use during early setup, when we don't have
 * the reverse channel available for reading back settings.  We
 * write multiple times, to try to make sure the setting succeeds.
 */
static int ser_set_logain(struct max927x *me)
{
	const struct max927x_setting *s = &max9271_settings[MAX9271_LOGAIN];
	u8 val = (me->current_logain << s->pos);
	int i;
	for (i = 0; i < REG_RETRIES-1; i++)
		i2c_reg_write(me->ser, s->reg, val);
	return i2c_reg_write(me->ser, s->reg, val);
}

static int ser_read(struct max927x *me, max9271_setting_t which, u8 *valp)
{
	const struct max927x_setting *s = &max9271_settings[which];
	u8 val;
	int ret = i2c_reg_read(me->ser, s->reg, &val);

	if (ret < 0)
		dev_dbg(me->dev, "%s: which=%s, read status=%d\n", __func__, s->name, ret);
	else
		*valp = (val >> s->pos) & ~(0xFF << s->width);
	return ret;
}

static int ser_update_internal(struct max927x *me, max9271_setting_t which, u8 newval, bool docheck)
{
	const struct max927x_setting *s = &max9271_settings[which];
	u8 val,	 mask, updval, checkval = 0;
	int ret = i2c_reg_read(me->ser, s->reg, &val);

	dev_dbg(me->dev, "%s: %s = %d\n", __func__, s->name, newval);
	if (ret < 0) {
		dev_warn(me->dev, "error %d reading register 0x%x for updating %s\n",
			 ret, s->reg, s->name);
		return ret;
	}
	mask = ~(~(0xFF << s->width) << s->pos);
	updval = (val & mask) | ((newval << s->pos) & ~mask);
	if (val == updval) {
		dev_dbg(me->dev, "%s: skipping update of %s (reg 0x%x): no value change\n",
			__func__, s->name, s->reg);
		return 0;
	}
	ret = i2c_reg_write(me->ser, s->reg, updval);
	if (ret >= 0) {
		if (!docheck)
			return ret;
		/*
		 * If the serializer is remote, and we're touching register 4,
		 * we may not be able to access the remote for some time after
		 * updating SEREN and/or CLINKEN, so pause first.  If the read-back
		 * still fails, put back the old value before returning.
		 *
		 * The read-back can fail if we're enabling SEREN on a remote
		 * serializer but there is no input for the serializer to process.
		 * So restore the old value so we can reestablish control communication
		 * with the remote.
		 */
		if (me->ser == me->remote && s->reg == 4)
			msleep(5);
		ret = i2c_reg_read(me->ser, s->reg, &checkval);
		if (ret < 0) {
			dev_warn(me->dev, "error %d reading back register 0x%x for %s update\n",
				 ret, s->reg, s->name);
			if (me->ser == me->remote && s->reg == 4)
				i2c_reg_write(me->ser, s->reg, val);
		}
	} else {
		dev_warn(me->dev, "error %d updating %s in register 0x%x\n", ret, s->name, s->reg);
	}
	if (ret >=0 && checkval != updval) {
		dev_warn(me->dev, "%s: read back val 0x%x mismatch with desired 0x%x updating %s\n",
			 __func__, val, updval, s->name);
		ret = -EIO;
	}
	return ret;
}

static int ser_update_nocheck(struct max927x *me, max9272_setting_t which, u8 newval)
{
	return ser_update_internal(me, which, newval, false);
}

static int ser_update(struct max927x *me, max9272_setting_t which, u8 newval)
{
	return ser_update_internal(me, which, newval, true);
}

/*
 * Deserializer settings
 */
static int des_read(struct max927x *me, max9272_setting_t which, u8 *valp)
{
	const struct max927x_setting *s = &max9272_settings[which];
	u8 val;
	int ret = i2c_reg_read(me->des, s->reg, &val);

	if (ret < 0) {
		dev_dbg(me->dev, "%s: which=%s, read status=%d\n", __func__, s->name, ret);
		return ret;
	}
	*valp = (val >> s->pos) & ~(0xFF << s->width);
	return ret;
}

static int des_update_internal(struct max927x *me, max9272_setting_t which, u8 newval, bool docheck)
{
	const struct max927x_setting *s = &max9272_settings[which];
	u8 val, mask, updval;
	int ret = i2c_reg_read(me->des, s->reg, &val);

	dev_dbg(me->dev, "%s: %s = %d\n", __func__, s->name, newval);
	if (ret < 0)
		return ret;
	mask = ~(~(0xFF << s->width) << s->pos);
	updval = (val & mask) | ((newval << s->pos) & ~mask);
	updval = (val & mask) | ((newval << s->pos) & ~mask);
	if (val == updval) {
		dev_dbg(me->dev, "%s: skipping update of %s (reg 0x%x): no value change\n",
			 __func__, s->name, s->reg);
		return 0;
	}
	ret = i2c_reg_write(me->des, s->reg, updval);
	if (ret >= 0) {
		if (!docheck)
			return ret;
		ret = i2c_reg_read(me->des, s->reg, &val);
	}
	if (ret >=0 && val != updval) {
		dev_warn(me->dev, "%s: read back val 0x%x mismatch with desired 0x%x updating %s\n",
			 __func__, val, updval, s->name);
		ret = -EIO;
	}
	return ret;
}

static int des_update_nocheck(struct max927x *me, max9272_setting_t which, u8 newval)
{
	return des_update_internal(me, which, newval, false);
}

static int des_update(struct max927x *me, max9272_setting_t which, u8 newval)
{
	return des_update_internal(me, which, newval, true);
}

/*
 * Our I2C transfer routine for devices hanging off the remote, which
 * adds some retries-after-delay in the event of a failure.
 */
static int remote_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg msgs[], int num)
{
	struct max927x *me = adap->algo_data;
	unsigned int xferflags  = atomic_read(&me->i2c_xfer_flags);
	unsigned ntries;
	int ret = 0;


	for (ntries = 0; ntries < i2c_retries; ntries += 1) {
		ret = me->parent->algo->master_xfer(me->parent, msgs, num);
		if ((xferflags & I2C_XFER_NORETRIES) != 0)
			return ret;
		if (ret >= 0)
			break;
	}

	if ((xferflags & I2C_XFER_NOCOUNT) == 0) {
		atomic_inc(&me->i2c_retry_counts[(ntries > i2c_retries ? i2c_retries : ntries)]);

		if (ret < 0)
			atomic_inc(&me->i2c_error_count);
	}

	return ret;

}

/*
 * Power on/off remote
 */
static void power_up_remote(struct max927x *me)
{
	int ret;
	if (me->remote_power == NULL)
		return;
	ret = regulator_enable(me->remote_power);
	dev_dbg(me->dev, "powering up remote %s (%d), now waiting %u msec\n",
		(ret < 0 ? "failed" : "succeeded"), ret, me->power_on_mdelay);
	if (me->power_on_mdelay != 0)
		msleep(me->power_on_mdelay);
	atomic_xchg(&me->remote_power_enabled, 1);
}

static void power_down_remote(struct max927x *me)
{
	int ret;
	if (me->remote_power == NULL)
		return;

	if (atomic_cmpxchg(&me->remote_power_enabled, 1, 0)) {
		ret = regulator_disable(me->remote_power);
		if (ret < 0)
			dev_dbg(me->dev, "powering down remote failed: %d\n", ret);
		if (me->power_off_mdelay != 0)
			msleep(me->power_off_mdelay);
	}
}

/*
 * Device attribute handling.  If you need to make another setting accessible through
 * sysfs or the array in the device tree, add it to the list of property definitions above.
 */
static ssize_t show_property(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	struct max927x_property *prop = container_of(devattr, struct max927x_property, dev_attr);

	if (me == NULL)
		return -ENODEV;
	if (prop == NULL)
		return -EINVAL;
	dev_dbg(dev, "%s: getting %s from init-time configuration\n",
		__func__, prop->setting->name);

	return scnprintf(buf, PAGE_SIZE, "%u\n", me->of_cfg_settings[prop->propid]);
}

static ssize_t store_property(struct device *dev, struct device_attribute *devattr,
			      const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	struct max927x_property *prop = container_of(devattr, struct max927x_property, dev_attr);
	unsigned val;

	if (me == NULL)
		return -ENODEV;
	if (prop == NULL || kstrtouint(buf, 10, &val) != 0 || val >= (1 << prop->setting->width))
		return -EINVAL;

	dev_dbg(dev, "%s: setting %s to 0x%x in init-time configuration\n",
		__func__, prop->setting->name, val);

	me->of_cfg_settings[prop->propid] = val;

	return count;
}

#define MAX9272_PROPERTY(tag_, name_, setting_) [CFG_##tag_] = { \
	.for_deserializer = 1, \
	.propid = CFG_##tag_, \
	.setting = &max9272_settings[MAX9272_##setting_], \
	.dev_attr = { .attr = { .name = #name_, .mode = 0666, }, \
		      .show = show_property, .store = store_property }},
#define MAX9271_PROPERTY(tag_, name_, setting_) [CFG_##tag_] = { \
	.for_deserializer = 0, \
	.propid = CFG_##tag_, \
	.setting = &max9271_settings[MAX9271_##setting_], \
	.dev_attr = { .attr = { .name = #name_, .mode = 0666, },\
		      .show = show_property, .store = store_property }},
static struct max927x_property properties[NUM_CFG_PROPS] = { MAX9272_PROPERTIES MAX9271_PROPERTIES };
#undef MAX9272_PROPERTY
#undef MAX9271_PROPERTY

#define MAX9272_PROPERTY(tag_, name_, setting_) &properties[CFG_##tag_].dev_attr.attr,
#define MAX9271_PROPERTY(tag_, name_, setting_) &properties[CFG_##tag_].dev_attr.attr,
static struct attribute *properties_attributes[] = {
	MAX9272_PROPERTIES
	MAX9271_PROPERTIES
	NULL
};
#undef MAX9272_PROPERTY
#undef MAX9271_PROPERTY

static struct attribute_group properties_attr_group = {
	.attrs = properties_attributes,
};

static const property_id_t ser_init_properties[] = {
	CFG_REV_DIG_FLT, CFG_REV_HIGAIN, CFG_REV_HIBW, CFG_REV_HIVTH
};

/*
 * Control channel initialization when serializer is local and deserializer
 * is remote
 */
static int ser_control_init(struct max927x *me)
{
	int i, ret;

	/*
	 * Reset so we're starting from a known state
	 */
	device_reset(me->dev);
	msleep(5);
	i2c_recover_bus(me->parent);
	power_down_remote(me);

	/*
	 * Start by disabling SEREN and enabling CLINKEN in one register access.
	 */
	ret = i2c_reg_write(me->ser, 0x04, 0x47);
	if (ret < 0) {
		dev_err(me->dev, "error initializing control channel: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ser_init_properties); i++) {
		property_id_t which = ser_init_properties[i];
		ret = ser_update(me, properties[which].setting->idx,
				 me->of_cfg_settings[which]);
		if (ret < 0) {
			dev_err(me->dev, "could not set initial value for %s\n",
				properties[which].dev_attr.attr.name);
		}
	}
	if (ret == 0)
		ret = ser_update(me, MAX9271_INVVS, 0);
	if (ret == 0)
		ret = ser_update(me, MAX9271_INVHS, 0);
	if (ret == 0)
		ret = ser_update(me, MAX9271_GPIO_EN, atomic_read(&me->ser_gpios_enabled));
	if (ret == 0)
		ret = ser_update(me, MAX9271_GPIO_SET, 0);
	if (ret == 0)
		ret = ser_update(me, MAX9271_GPIO_SET, atomic_read(&me->ser_gpios_set));
	if (ret < 0)
		dev_err(me->dev, "error initializing serializer: %d\n", ret);

	/*
	 * Prepare for I2C communication over the serial link
	 */
	ret = ser_update(me, MAX9271_I2CLOCACK, 1);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CSLVSH, me->of_cfg_settings[CFG_I2C_SLVSH]);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CMSTBT, me->of_cfg_settings[CFG_I2C_MSTBT]);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CSLVTO, 2);
	if (ret < 0)
		dev_err(me->dev, "serializer I2C setup error: %d\n", ret);

	power_up_remote(me);

	ret = des_update(me, MAX9272_REV_AMP, me->of_cfg_settings[CFG_REV_AMP]);
	if (ret == 0)
		ret = des_update(me, MAX9272_REV_TRF, me->of_cfg_settings[CFG_REV_TRF]);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CLOCACK, 1);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CSLVSH, me->of_cfg_settings[CFG_I2C_SLVSH]);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CMSTBT, me->of_cfg_settings[CFG_I2C_MSTBT]);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CSLVTO, 2);
	if (ret == 0)
		ret = des_update(me, MAX9272_DBL, 1);
	if (ret == 0)
		ret = des_update(me, MAX9272_DRS, MAX927X_DRS_HIGH);
	if (ret == 0)
		ret = des_update(me, MAX9272_BWS, MAX927X_BWS_24BIT);
	if (ret == 0)
		ret = des_update(me, MAX9272_ES, MAX927X_ES_FALLING);
	if (ret == 0)
		ret = des_update(me, MAX9272_HVTRACK, 0);
	if (ret == 0)
		ret = des_update(me, MAX9272_HVEN, 1);
	/*
	 * Changing EDC on the remote causes loss of communication
	 * until we update the local side to match, so update both,
	 * then re-update the remote to test that the change was made.
	 * Also need to wait before trying to update the local side.
	 */
	if (ret == 0) {
		des_update_nocheck(me, MAX9272_EDC, MAX927X_EDC_HAMMING);
		msleep(5);
		for (i = 1; i < 3; i++) {
			ret = ser_update(me, MAX9271_EDC, MAX927X_EDC_HAMMING);
			if (ret >= 0)
				break;
			msleep(5);
		}
		if (ret >= 0)
			ret = des_update(me, MAX9272_EDC, MAX927X_EDC_HAMMING);
		else
			dev_err(me->dev, "could not update EDC on serializer: %d\n", ret);
	}
	if (ret < 0) {
		dev_err(me->dev, "error initializing deserializer: %d\n", ret);
		power_down_remote(me);
		return ret;
	}

	/*
	 * Don't care about I2C retries or errors during setup
	 */
	atomic_xchg(&me->i2c_error_count, 0);
	for (i = 0; i < ARRAY_SIZE(me->i2c_retry_counts); i++)
		atomic_xchg(&me->i2c_retry_counts[i], 0);

	ret = ser_update(me, MAX9271_DBL, 1);
	if (ret == 0)
		ret = ser_update(me, MAX9271_DRS, MAX927X_DRS_HIGH);
	if (ret == 0)
		ret = ser_update(me, MAX9271_BWS, MAX927X_BWS_24BIT);
	if (ret == 0)
		ret = ser_update(me, MAX9271_ES, MAX927X_ES_RISING);
	if (ret == 0)
		ret = ser_update(me, MAX9271_HVEN, 1);
	if (ret < 0) {
		dev_err(me->dev, "error finalizing serializer config: %d\n", ret);
		power_down_remote(me);
		return ret;
	}

	/*
	 * Don't need automatic I2C acks any longer
	 */
	return ser_update(me, MAX9271_I2CLOCACK, 0);

}

/*
 * Control channel initialization for when deserializer is local and
 * serializer is remote
 */
static int des_control_init(struct max927x *me)
{
	int i, ret;
	/*
	 * Reset so we're starting from a known state
	 */
	device_reset(me->dev);
	msleep(10);
	i2c_recover_bus(me->parent);
	power_down_remote(me);

	/*
	 * These settings (which are not documented in the data sheet) need to be performed early,
	 * as they affect the reverse communications channel between the deserializer and the
	 * serializer.
	 */
	ret = des_update(me, MAX9272_REV_AMP, me->current_rev_amp);
	if (ret == 0)
		ret = des_update(me, MAX9272_REV_TRF, me->of_cfg_settings[CFG_REV_TRF]);
	if (ret == 0)
		ret = des_update(me, MAX9272_SLEEP, 0);

	/*
	 * Prepare for I2C communication with the remote
	 */
	ret = des_update(me, MAX9272_I2CLOCACK, 1);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CSLVSH, me->of_cfg_settings[CFG_I2C_SLVSH]);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CMSTBT, me->of_cfg_settings[CFG_I2C_MSTBT]);
	if (ret == 0)
		ret = des_update(me, MAX9272_I2CSLVTO, 2);
	if (ret == 0)
		ret = des_update(me, MAX9272_FWDCCEN, 1);
	if (ret == 0)
		ret = des_update(me, MAX9272_REVCCEN, 1);
	if (ret < 0)
		dev_err(me->dev, "deserializer I2C setup error: %d\n", ret);

	power_up_remote(me);

	if (max9272_rev_amp_to_millivolts(me->current_rev_amp) == 60) {
		me->current_logain = 1;
		ret = ser_set_logain(me);
		dev_notice(me->dev, "resetting LOGAIN to %d: ret=%d\n",
			   me->current_logain, ret);
		max9272_millivolts_to_rev_amp(80, &me->current_rev_amp);
		ret = des_update(me, MAX9272_REV_AMP, me->current_rev_amp);
		dev_notice(me->dev, "resetting REV_AMP to %u mVolts: ret=%d\n",
			   max9272_rev_amp_to_millivolts(me->current_rev_amp), ret);
	} else {
		ret = ser_set_logain(me);
		if (ret < 0)
			dev_err(me->dev, "could not initialize LOGAIN\n");
	}

	/*
	 * Disable SEREN and enable CLINKEN on the serializer in a single register
	 * write.  If we can read back the setting, we know the control channel is good.
	 * Retry/delay loop here is just in case - it shouldn't really be needed.
	 */
	for (i = 1; i <= REG_RETRIES; i++) {
		dev_dbg(me->dev, "setting up control link on serializer (attempt #%d)\n", i);
		ret = i2c_reg_write(me->ser, 0x04, 0x47);
		if (ret == 0) {
			int j;
			u8 val = 0;
			for (j = 1; j <= REG_RETRIES; j++) {
				msleep(10);
				ret = i2c_reg_read(me->ser, 0x04, &val);
				if (ret == 0) {
					if (val == 0x47)
						break;
					else
						ret = -EIO;
				}
			}
			if (ret == 0)
				break;
		}
	}
	if (ret < 0) {
		dev_err(me->dev, "could not initialize control link in serializer, ret=%d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ser_init_properties); i++) {
		property_id_t which = ser_init_properties[i];
		ret = ser_update(me, properties[which].setting->idx,
				 me->of_cfg_settings[which]);
		if (ret < 0) {
			dev_err(me->dev, "could not set initial value for %s\n",
				properties[which].dev_attr.attr.name);
		}
	}
	if (ret == 0)
		ret = ser_update(me, MAX9271_INVVS, 0);
	if (ret == 0)
		ret = ser_update(me, MAX9271_INVHS, 0);
	if (ret == 0)
		ret = ser_update(me, MAX9271_GPIO_EN, atomic_read(&me->ser_gpios_enabled));
	if (ret == 0)
		ret = ser_update(me, MAX9271_GPIO_SET, atomic_read(&me->ser_gpios_set));
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CLOCACK, 1);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CSLVSH, me->of_cfg_settings[CFG_I2C_SLVSH]);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CMSTBT, me->of_cfg_settings[CFG_I2C_MSTBT]);
	if (ret == 0)
		ret = ser_update(me, MAX9271_I2CSLVTO, 2);
	if (ret == 0)
		ret = ser_update(me, MAX9271_DBL, 1);
	if (ret == 0)
		ret = ser_update(me, MAX9271_DRS, MAX927X_DRS_HIGH);
	if (ret == 0)
		ret = ser_update(me, MAX9271_BWS, MAX927X_BWS_24BIT);
	if (ret == 0)
		ret = ser_update(me, MAX9271_ES, MAX927X_ES_RISING);
	if (ret == 0)
		ret = ser_update(me, MAX9271_HVEN, 1);

	/*
	 * Changing EDC on the remote causes loss of communication
	 * until we update the local side to match, so update both,
	 * then re-update the remote to test that the change was made.
	 * Must also wait (and possibly retry) to update the local side.
	 */
	if (ret == 0) {
		ser_update_nocheck(me, MAX9271_EDC, MAX927X_EDC_HAMMING);
		msleep(5);
		for (i = 1; i <= 3; i++) {
			ret = des_update(me, MAX9272_EDC, MAX927X_EDC_HAMMING);
			if (ret >= 0)
				break;
			msleep(5);
		}
		if (ret >= 0)
			ret = ser_update(me, MAX9271_EDC, MAX927X_EDC_HAMMING);
		else
			dev_err(me->dev, "could not update EDC on deserializer: %d\n", ret);
	}
	if (ret < 0) {
		dev_err(me->dev, "error initializing serializer: %d\n", ret);
		power_down_remote(me);
		return ret;
	}

	/*
	 * Don't care about I2C retries during setup, so clear the counts
	 */
	atomic_xchg(&me->i2c_error_count, 0);
	for (i = 0; i < ARRAY_SIZE(me->i2c_retry_counts); i++)
		atomic_xchg(&me->i2c_retry_counts[i], 0);

	ret = des_update(me, MAX9272_DBL, 1);
	if (ret == 0)
		ret = des_update(me, MAX9272_DRS, MAX927X_DRS_HIGH);
	if (ret == 0)
		ret = des_update(me, MAX9272_BWS, MAX927X_BWS_24BIT);
	if (ret == 0)
		ret = des_update(me, MAX9272_ES, MAX927X_ES_FALLING);
	if (ret == 0)
		ret = des_update(me, MAX9272_HVTRACK, 0);
	if (ret == 0)
		ret = des_update(me, MAX9272_HVEN, 1);
	if (ret < 0) {
		dev_err(me->dev, "error finalizing deserializer config: %d\n", ret);
		power_down_remote(me);
		return ret;
	}

	/*
	 * Don't need automatic I2C acks any longer
	 */
	return des_update(me, MAX9272_I2CLOCACK, 0);
}

/*
 * GPIO support - normally only available after we have initialized
 * the control link, as some GPIOs are on the remote device.
 *
 * 8 GPIOs provided (but GPIO 0 is the special ser-GPO/des-GPI
 * and should not be used - not implemented).
 * GPIOs 1-5 are on the serializer, GPIOs 6 and 7 are on the
 * deserializer (numbered 0 & 1 in the MAX9272 data sheet).
 *
 * Per the data sheet, all of the GPIOs are open drain and
 * should be set when used for input.
 *
 * The serializer's GPIO state is tracked in our private data
 * structure and re-played into the hardware when we reset the
 * control link (toggling the GPIOs off/on).
 *
 * XXX - should do similar tracking for deserializer
 */

static int gpio_dir_in(struct gpio_chip *gc, unsigned num)
{
	struct max927x *me = container_of(gc, struct max927x, gc);
	int ret;

	dev_dbg(me->dev, "%s(%d)\n", __func__, num);

	if (num < 1 || num > 7)
		return -EINVAL;

	if (num < 6) {
		u8 gpio_en, gpio_set;

		/*
		 * Switch from 1-based to zero-based to match the regs
		 */
		num -= 1;
		ret = ser_read(me, MAX9271_GPIO_EN, &gpio_en);
		if (ret < 0)
			return ret;
		ret = ser_read(me, MAX9271_GPIO_SET, &gpio_set);
		if (ret < 0)
			return ret;
		dev_dbg(me->dev, "%s: old gpio_set=0x%x, gpio_en=0x%x\n",
			__func__, gpio_set, gpio_en);
		gpio_set |= (1 << num);
		gpio_en |= (1 << num);
		dev_dbg(me->dev, "%s: new gpio_set=0x%x, gpio_en=0x%x\n",
			__func__, gpio_set, gpio_en);
		ret = ser_update(me, MAX9271_GPIO_SET, gpio_set);
		if (ret == 0)
			ret = ser_update(me, MAX9271_GPIO_EN, gpio_en);
		if (ret == 0) {
			atomic_xchg(&me->ser_gpios_set, gpio_set);
			atomic_xchg(&me->ser_gpios_enabled, gpio_en);
		}
	} else {
		max9272_setting_t which = (num == 6 ? MAX9272_GPIO0_SET : MAX9272_GPIO1_SET);
		ret = des_update(me, which, 1);
	}

	return ret;
}

static int gpio_dir_out(struct gpio_chip *gc, unsigned num, int outval)
{
	struct max927x *me = container_of(gc, struct max927x, gc);
	int ret;

	dev_dbg(me->dev, "%s(%d)\n", __func__, num);

	if (num < 1 || num > 7)
		return -EINVAL;

	if (num < 6) {
		u8 gpio_en, gpio_set;
		/*
		 * Switch from 1-based to zero-based to match the regs
		 */
		num -= 1;
		ret = ser_read(me, MAX9271_GPIO_EN, &gpio_en);
		if (ret < 0)
			return ret;
		ret = ser_read(me, MAX9271_GPIO_SET, &gpio_set);
		if (ret < 0)
			return ret;
		dev_dbg(me->dev, "%s: old gpio_set=0x%x, gpio_en=0x%x\n",
			__func__, gpio_set, gpio_en);
		if (outval)
			gpio_set |= (1 << num);
		else
			gpio_set &= ~(1 << num);
		gpio_en |= (1 << num);
		dev_dbg(me->dev, "%s: new gpio_set=0x%x, gpio_en=0x%x\n",
			__func__, gpio_set, gpio_en);
		ret = ser_update(me, MAX9271_GPIO_SET, gpio_set);
		if (ret == 0)
			ret = ser_update(me, MAX9271_GPIO_EN, gpio_en);
		if (ret == 0) {
			atomic_xchg(&me->ser_gpios_set, gpio_set);
			atomic_xchg(&me->ser_gpios_enabled, gpio_en);
		}
	} else {
		max9272_setting_t which = (num == 6 ? MAX9272_GPIO0_SET : MAX9272_GPIO1_SET);
		ret = des_update(me, which, (outval ? 1 : 0));
	}

	return ret;
}

static void gpio_set(struct gpio_chip *gc, unsigned num, int setval)
{
	struct max927x *me = container_of(gc, struct max927x, gc);
	int ret;

	dev_dbg(me->dev, "%s(%d)\n", __func__, num);

	if (num < 1 || num > 7) {
		dev_err(me->dev, "invalid GPIO index: %u\n", num);
		return;
	}

	if (num < 6) {
		u8 gpio_en, gpio_set;
		/*
		 * Switch from 1-based to zero-based to match the regs
		 */
		num -= 1;
		ret = ser_read(me, MAX9271_GPIO_EN, &gpio_en);
		if (ret == 0) {
			if ((gpio_en & (1 << num)) == 0)
				ret = -EINVAL;
		}
		if (ret == 0) {
			ret = ser_read(me, MAX9271_GPIO_SET, &gpio_set);
			if (ret == 0) {
				dev_dbg(me->dev, "%s: old gpio_set=0x%x\n",
					__func__, gpio_set);
				if (setval)
					gpio_set |= (1 << num);
				else
					gpio_set &= ~(1<<num);
				dev_dbg(me->dev, "%s: new gpio_set=0x%x\n",
					__func__, gpio_set);
				ret = ser_update(me, MAX9271_GPIO_SET, gpio_set);
				if (ret == 0)
					atomic_xchg(&me->ser_gpios_set, gpio_set);
			}
		}
	} else {
		max9272_setting_t which = (num == 6 ? MAX9272_GPIO0_SET : MAX9272_GPIO1_SET);
		ret = des_update(me, which, (setval ? 1 : 0));
	}

	if (ret < 0)
		dev_err(me->dev, "error setting GPIO %u: %d\n", num, ret);
}

static int gpio_read(struct gpio_chip *gc, unsigned num)
{
	struct max927x *me = container_of(gc, struct max927x, gc);
	int ret;
	u8 gpio_val;

	dev_dbg(me->dev, "%s(%d)\n", __func__, num);

	if (num < 1 || num > 7)
		return -EINVAL;

	if (num < 6) {
		u8 gpio_en;
		/*
		 * Switch from 1-based to zero-based to match the regs
		 */
		num -= 1;
		ret = ser_read(me, MAX9271_GPIO_EN, &gpio_en);
		if (ret < 0)
			return ret;
		if ((gpio_en & (1 << num)) == 0)
			return -EINVAL;
		ret = ser_read(me, MAX9271_GPIO_READ, &gpio_val);
		if (ret < 0)
			return ret;
		ret = (gpio_val & (1 << num)) ? 1 : 0;
		dev_dbg(me->dev, "%s: gpio_set=0x%x, returning %d\n",
			__func__, gpio_val, ret);
	} else {
		max9272_setting_t which = (num == 6 ? MAX9272_GPIO0_READ : MAX9272_GPIO1_READ);
		ret = des_read(me, which, &gpio_val);
		if (ret == 0)
			ret = gpio_val;
	}

	return ret;
}

static int setup_gpiochip(struct max927x *me)
{
	struct gpio_chip *gc = &me->gc;

	gc->direction_input = gpio_dir_in;
	gc->direction_output = gpio_dir_out;
	gc->get = gpio_read;
	gc->set = gpio_set;
	gc->can_sleep = 1;
	gc->ngpio = 8;
	gc->label = me->local->name;
	gc->dev = &me->local->dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	return gpiochip_add(gc);
}

static void run_i2c_test(struct max927x *me, unsigned int which, unsigned int cycles)
{
	struct i2c_client *target;
	struct i2c_test_data *d = &me->i2c_test_results;
	const struct max927x_setting *s;
	char *device_name;
	unsigned int i, j, percent_complete;
	u8 reg, mask, origval;
	int ret;

	if (which == MAX9272_CHIPID) {
		s = &max9272_settings[MAX9272_I2CDSTA];
		target = me->des;
		ret = des_read(me, MAX9272_I2CDSTA, &origval);
		device_name = "max9272";
	} else {
		s = &max9271_settings[MAX9271_I2CDSTA];
		target = me->ser;
		ret = ser_read(me, MAX9271_I2CDSTA, &origval);
		device_name = "max9271";
	}
	if (ret < 0) {
		dev_err(me->dev, "I2C test aborted: could not save original register value\n");
		return;
	}
	reg = s->reg;
	mask = ~(0xFF << s->width) << s->pos;

	memset(d, 0, sizeof(*d));
	strlcpy(d->tested_device, device_name, sizeof(d->tested_device));

	dev_notice(me->dev, "I2C test (%s): start\n", device_name);
	for (i = 1; i <= cycles; i++) {
		u8 writeval, readval;

		d->cycles += 1;
		writeval = (i << s->pos) & mask;
		for (j = 0; j < i2c_retries; j++) {
			ret = i2c_reg_write(target, reg, writeval);
			d->transaction_count += 1;
			if (ret >= 0)
				break;
			d->write_retries += 1;
		}

		if (d->write_retries > d->max_retries)
			d->max_retries = d->write_retries;

		if (j >= i2c_retries) {
			dev_warn(me->dev, "I2C test (%s): write failure on cycle %u\n",
				 device_name, i);
			d->failures += 1;
			continue;
		}

		for (j = 0; j < i2c_retries; j++) {
			ret = i2c_reg_read(target, reg, &readval);
			d->transaction_count += 1;
			if (ret >= 0)
				break;
			d->read_retries += 1;
		}

		if (d->read_retries > d->max_retries)
			d->max_retries = d->read_retries;

		if (j >= i2c_retries) {
			dev_warn(me->dev, "I2C test (%s): read failure on cycle %u\n",
				 device_name, i);
			d->failures += 1;
			continue;
		}

		readval &= mask;
		if (readval != writeval) {
			dev_warn(me->dev, "I2C test (%s): write 0x%x read 0x%x mismatch on cycle %u\n",
				 device_name, writeval, readval, i);
			d->mismatches += 1;
		}
		percent_complete = (i * 100) / cycles;
		if (percent_complete > 0 && percent_complete % 10 == 0 && percent_complete * cycles / 100 == i)
			dev_notice(me->dev, "I2C test (%s): %u cycles (%u%%) complete with %u failures\n",
				   device_name, i, percent_complete, d->failures);
	}
	dev_notice(me->dev, "I2C test (%s): complete, %u failures\n", device_name, d->failures);
	if (which == MAX9272_CHIPID)
		ret = des_update(me, MAX9272_I2CDSTA, origval);
	else
		ret = ser_update(me, MAX9271_I2CDSTA, origval);
	if (ret < 0)
		dev_err(me->dev, "I2C test: could not restore original register value\n");
}

static int link_test(struct max927x *me, int count_singles)
{
	struct i2c_client *target = me->remote;
	const struct max927x_setting *s = &max9271_settings[MAX9271_ID];
	unsigned int oldflags = atomic_read(&me->i2c_xfer_flags);
	unsigned int j;
	unsigned int retries[I2C_RETRIES_MAX+1];
	u8 reg, desired;
	int ret, i;

	reg = s->reg;
	desired = (me->remote == me->ser ? MAX9271_CHIPID : MAX9272_CHIPID);

	memset(retries, 0, sizeof(retries));

	atomic_or(I2C_XFER_NORETRIES|I2C_XFER_NOCOUNT, &me->i2c_xfer_flags);

	for (i = 1; i <= training_cycles; i++) {
		u8 readval;

		for (j = 0; j < i2c_retries; j++) {
			ret = i2c_reg_read(target, reg, &readval);
			if (ret >= 0 && readval == desired)
				break;
			if (ret == -ETIMEDOUT) {
				dev_warn(me->dev, "%s: timeout error\n", __func__);
				atomic_xchg(&me->i2c_xfer_flags, oldflags);
				return ret;
			}
		}
		if (j >= i2c_retries) {
			dev_warn(me->dev, "%s: too many retries on cycle %d\n", __func__, i);
			atomic_xchg(&me->i2c_xfer_flags, oldflags);
			return -EIO;
		} else
			retries[j] += 1;
	}

	/*
	 * Compute weighted score as sum of:
	 *   for each retry count > 1, retry count * number of occurrences
	 */
	for (ret = 0, i = (count_singles ? 1 : 2); i < i2c_retries; i++) {
		ret += i * retries[i];
		if (retries[i] > 0)
			dev_dbg(me->dev, "%s: %u occurrences of %u retries\n",
				__func__, retries[i], i);
	}

	dev_notice(me->dev, "%s: exiting, ret=%d", __func__, ret);
	atomic_xchg(&me->i2c_xfer_flags, oldflags);
	return ret;
}

/*
 * run_link_training expects lock mutex to be held
 */
static int run_link_training(struct max927x *me, int live)
{
	int threshold = training_threshold * training_cycles / 100;
	int counter, ret, score[2];

	dev_notice(me->dev, "begin link training (link %sactive), REV_AMP=%u mVolts, LOGAIN=%d, threshold=%d\n",
		   (live ? "" : "in"),
		   max9272_rev_amp_to_millivolts(me->current_rev_amp),
		   me->current_logain, threshold);

	me->training = 1;
	for (counter = 0; counter < 2; counter++) {
		mutex_unlock(&me->lock);
		ret = link_test(me, 0);
		mutex_lock(&me->lock);
		if (ret < 0) {
			if (live) {
				dev_warn(me->dev, "%s: link test failed while live, aborting\n",
					 __func__);
				me->training = 0;
				return ret;
			}
			max9272_millivolts_to_rev_amp(60, &me->current_rev_amp);
			dev_notice(me->dev, "error during link training, resetting control link with REV_AMP at %u mVolts\n",
				   max9272_rev_amp_to_millivolts(me->current_rev_amp));
			ret = me->ctrl_link_init(me);
			dev_notice(me->dev, "early exit from link training, REV_AMP=%u mVolts, LOGAIN=%d, result=%d\n",
				   max9272_rev_amp_to_millivolts(me->current_rev_amp),
				   me->current_logain, ret);
			me->training = 0;
			return ret;
		}
		if (ret < threshold)
			break;
		score[me->current_logain] = ret;
		me->current_logain = 1 - me->current_logain;
		dev_notice(me->dev, "threshold exceeded (%d), switching LOGAIN to %d\n",
			   ret, me->current_logain);
		ret = ser_set_logain(me);
		if (ret < 0) {
			dev_err(me->dev, "could not update LOGAIN, ret=%d\n", ret);
			break;
		}
	}

	if (counter >= 2) {
		me->current_logain = (score[0] < score[1]) ? 0 : 1;
		dev_notice(me->dev, "choosing lower of (%d, %d) to set LOGAIN=%d\n",
			   score[0], score[1], me->current_logain);
		ret = ser_set_logain(me);
		if (ret < 0)
			dev_err(me->dev, "could not update LOGAIN, ret=%d\n", ret);
	}

	me->training = 0;

	dev_notice(me->dev, "exit link training, REV_AMP=%u mVolts, LOGAIN=%d, result=%d\n",
		   max9272_rev_amp_to_millivolts(me->current_rev_amp),
		   me->current_logain, ret);

	return ret;
}


/*
 * Enabling or disabling the serial link for data should
 * be sufficient for stream on/off control.
 * XXX - May want to see about also enabling
 * and disabling outputs from the deserializer.
 */
static int stream_control(struct v4l2_subdev *sd, int enable)
{
	struct max927x *me = to_max927x_from_v4l2_subdev(sd);
	int ret;
	u8 newval = (enable ? 1 : 0);

	ret = ser_update(me, MAX9271_SEREN, newval);
	if (ret == 0) {
		u8 val = 1 - newval;
		int i;
		for (i = 0; i < 3; i++) {
			ret = des_read(me, MAX9272_LOCKED, &val);
			if (ret == 0 && val == newval)
				break;
			msleep(5);
		}
		if (ret == 0 && val != newval) {
			dev_err(me->dev, "video lock status did not change\n");
			ret = -EBUSY;
		}
	}
	return ret;
}

/*
 * More device attributes, for compatibility with old driver
 */
static ssize_t clear_i2c_retry_counts(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	int i;
	for (i = 0; i < ARRAY_SIZE(me->i2c_retry_counts); i++)
		atomic_xchg(&me->i2c_retry_counts[i], 0);
	return count;
}

static ssize_t show_i2c_retry_counts(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	ssize_t count = 0;
	int i;

	for (i = 0; i <= i2c_retries; i++)
		count += scnprintf(buf+count, PAGE_SIZE-count, "%u:%u\n",
				   i, atomic_read(&me->i2c_retry_counts[i]));
	return count;
}
static DEVICE_ATTR(i2c_retry_counts, 0666, show_i2c_retry_counts, clear_i2c_retry_counts);

static ssize_t deserializer_i2c_test(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	unsigned int cycles;

	if (kstrtouint(buf, 10, &cycles) || cycles < 100)
		return -EINVAL;
	if (atomic_cmpxchg(&me->i2c_test_in_progress, 0, 1))
		return -EBUSY;
	run_i2c_test(me, MAX9272_CHIPID, cycles);
	atomic_xchg(&me->i2c_test_in_progress, 0);
	return count;
}

static ssize_t serializer_i2c_test(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	unsigned int cycles;

	if (kstrtouint(buf, 10, &cycles) || cycles < 100)
		return -EINVAL;
	if (atomic_cmpxchg(&me->i2c_test_in_progress, 0, 1))
		return -EBUSY;
	run_i2c_test(me, MAX9271_CHIPID, cycles);
	atomic_xchg(&me->i2c_test_in_progress, 0);
	return count;
}

static ssize_t i2c_test_results(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	struct i2c_test_data *d = &me->i2c_test_results;
	ssize_t count = 0;

	if (atomic_cmpxchg(&me->i2c_test_in_progress, 0, 1))
		return -EBUSY;
	count = scnprintf(buf, PAGE_SIZE,
			  "Tested:             %s\n"
			  "Test cycles run:    %u\n"
			  "Failures:           %u\n"
			  "Retries (write):    %u\n"
			  "Retries (read):     %u\n"
			  "Mismatches:         %u\n"
			  "Max retries:        %u\n"
			  "Total transactions: %u\n"
			  "Result:             %s\n",
			  d->tested_device, d->cycles, d->failures,
			  d->write_retries, d->read_retries, d->mismatches,
			  d->max_retries, d->transaction_count,
			  (d->cycles == 0 ? "<none>" : (d->failures == 0 ? "PASS" : "FAIL")));
	atomic_xchg(&me->i2c_test_in_progress, 0);
	return count;
}

static DEVICE_ATTR(max9272_i2c_test, 0666, i2c_test_results, deserializer_i2c_test);
static DEVICE_ATTR(max9271_i2c_test, 0666, i2c_test_results, serializer_i2c_test);

static ssize_t set_operational(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	unsigned int enable;
	const char *bufp = buf;
	size_t remain = count;
	int ret, try = 0;

	if (count > 4 && memcmp(buf, "try:", 4) == 0) {
		bufp += 4;
		remain -= 4;
		try = 1;
	}
	ret = kstrtouint(bufp, 10, &enable);
	if (ret < 0)
		return ret;
	if (try) {
		if (!mutex_trylock(&me->lock))
			return -EBUSY;
	} else
		mutex_lock(&me->lock);

	if (me->curstate < STATE_READY) {
		mutex_unlock(&me->lock);
		return -EBUSY;
	}
	me->curstate = (enable ? STATE_OPERATIONAL : STATE_READY);
	mutex_unlock(&me->lock);

	return count;
}

static ssize_t show_operational(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	int operstate;

	mutex_lock(&me->lock);
	operstate = (me->curstate == STATE_OPERATIONAL) ? 1 : 0;
	mutex_unlock(&me->lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", operstate);
}
static DEVICE_ATTR(operational, 0666, show_operational, set_operational);

static int unregister_remote_i2c_device(struct device *dev, void *meptr)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct max927x *me = meptr;

	if (client && client->adapter == &me->adap && strcmp(client->name, "dummy") != 0) {
		dev_notice(me->dev, "unregistering I2C client %s\n", client->name);
		i2c_unregister_device(client);
	}
	return 0;
}

static ssize_t do_reset(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	int ret;

	// buf is non-NULL when we're called directly via sysfs handlers,
	// or NULL when we're called internally from do_training() and already
	// hold the mutex.
	if (buf != NULL) {
		if (count >= 3 && memcmp(buf, "try", 3) == 0) {
			if (!mutex_trylock(&me->lock))
				return -EBUSY;
		} else
			mutex_lock(&me->lock);
		if (me->training) {
			mutex_unlock(&me->lock);
			return -EBUSY;
		}
	}

	dev_dbg(dev, "device reset requested\n");

	mutex_unlock(&me->lock);
	cancel_work_sync(&me->training_work);
	mutex_lock(&me->lock);
	me->curstate = STATE_UNINITIALIZED;
	me->training = 0;
	atomic_xchg(&me->i2c_xfer_flags, I2C_XFER_NOCOUNT);

	device_for_each_child(&me->adap.dev, me, unregister_remote_i2c_device);

	max9272_millivolts_to_rev_amp(80, &me->current_rev_amp);
	me->current_logain = 0;

	ret = me->ctrl_link_init(me);

	if (ret < 0) {
		if (me->local == me->des) {
			max9272_millivolts_to_rev_amp(60, &me->current_rev_amp);
			dev_notice(dev, "%s: setting REV_AMP to %u mVolts to retry link setup\n",
				   __func__, max9272_rev_amp_to_millivolts(me->current_rev_amp));
			ret = me->ctrl_link_init(me);
			if (ret == 0) {
				dev_notice(dev, "%s: skipping link training due to REV_AMP reset\n",
					   __func__);
				goto finish_reset;
			}
		}
		dev_err(dev, "%s: link initialization failed\n", __func__);
		mutex_unlock(&me->lock);
		return ret;
	}

	ret = run_link_training(me, 0);
	if (ret < 0) {
		mutex_unlock(&me->lock);
		return ret;
	}
finish_reset:
	me->adap.dev.of_node = me->i2c_node;
	of_i2c_register_devices(&me->adap);
	atomic_xchg(&me->i2c_xfer_flags, 0);
	me->curstate = STATE_READY;
	mutex_unlock(&me->lock);

	return count;
}
static DEVICE_ATTR(reset, 0222, NULL, do_reset);

static ssize_t do_training(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	int ret;

	if (count >= 3 && memcmp(buf, "try", 3) == 0) {
		if (!mutex_trylock(&me->lock))
			return -EBUSY;
	} else
		mutex_lock(&me->lock);

	if (me->curstate < STATE_READY || me->training) {
		mutex_unlock(&me->lock);
		return -EBUSY;
	}
	ret = run_link_training(me, 1);
	if (ret < 0)
		return do_reset(dev, attr, NULL, count);
	mutex_unlock(&me->lock);
	return count;
}
static DEVICE_ATTR(retrain_link, 0222, NULL, do_training);

static ssize_t show_corrected_errs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	u8 val;
	int ret;

	ret = des_read(me, MAX9272_CORRERR, &val);
	if (ret < 0)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR(corrected_link_errors, 0444, show_corrected_errs, NULL);

static ssize_t show_detected_errs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	u8 val;
	int ret;

	ret = des_read(me, MAX9272_DETERR, &val);
	if (ret < 0)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR(detected_link_errors, 0444, show_detected_errs, NULL);

static ssize_t show_remote_i2c_errs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", atomic_read(&me->i2c_error_count));
}

static ssize_t clear_remote_i2c_errs(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	atomic_xchg(&me->i2c_error_count, 0);
	return count;
}

static DEVICE_ATTR(i2c_error_count, 0666, show_remote_i2c_errs, clear_remote_i2c_errs);


static ssize_t show_current_logain(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", me->current_logain);
}

static ssize_t set_current_logain(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	const struct max927x_setting *s = &max9271_settings[MAX9271_LOGAIN];
	unsigned int val;
	int ret;

	if (kstrtouint(buf, 10, &val) || val >= (1 << s->width))
		return EINVAL;
	ret = ser_update(me, MAX9271_LOGAIN, val);
	if (ret == 0)
		me->current_logain = val;
	return (ret < 0) ? ret : count;
}
static DEVICE_ATTR(current_logain, 0666, show_current_logain, set_current_logain);

static ssize_t show_current_rev_amp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x *me = to_max927x_from_dev(dev);

	return scnprintf(buf, PAGE_SIZE, "%u (%u mVolts)\n", me->current_rev_amp,
			 max9272_rev_amp_to_millivolts(me->current_rev_amp));
}

static ssize_t set_current_rev_amp(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max927x *me = to_max927x_from_dev(dev);
	unsigned int mvolts;
	u8 val;
	int ret;

	if (kstrtouint(buf, 10, &mvolts))
		return EINVAL;
	ret = max9272_millivolts_to_rev_amp(mvolts, &val);
	if (ret == 0)
		ret = des_update(me, MAX9272_REV_AMP, val);
	if (ret == 0)
		me->current_rev_amp = val;
	return (ret < 0) ? ret : count;
}
static DEVICE_ATTR(current_rev_amp, 0666, show_current_rev_amp, set_current_rev_amp);

static struct attribute *misc_attributes[] = {
	&dev_attr_i2c_retry_counts.attr,
	&dev_attr_max9272_i2c_test.attr,
	&dev_attr_max9271_i2c_test.attr,
	&dev_attr_corrected_link_errors.attr,
	&dev_attr_detected_link_errors.attr,
	&dev_attr_i2c_error_count.attr,
	&dev_attr_current_logain.attr,
	&dev_attr_current_rev_amp.attr,
	&dev_attr_retrain_link.attr,
	NULL,
};

static const struct attribute_group misc_attr_group = {
	.attrs = misc_attributes,
};

/*
 * These attributes should always be available after
 * the device is probed, to allow for reset if the
 * link doesn't come up.
 */
static struct attribute *static_attributes[] = {
	&dev_attr_operational.attr,
	&dev_attr_reset.attr,
	NULL,
};

static const struct attribute_group static_attr_group = {
	.attrs = static_attributes,
};

/*
 * I2C adapter data
 */
static u32 remote_i2c_functionality(struct i2c_adapter *adap) {
	return (I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL);
};

static struct i2c_algorithm remote_i2c_algo = {
	.master_xfer = remote_i2c_xfer,
	.functionality = remote_i2c_functionality,
};

/*
 * All attribute groups (except static_attr_group) collected here
 */
static const struct attribute_group *attr_groups[] = {
	&properties_attr_group,
	&misc_attr_group,
	NULL,
};

/*
 * V4L2 supported operations
 */
static const struct v4l2_subdev_video_ops subdev_video_ops = {
	.s_stream = stream_control,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.video = &subdev_video_ops,
};

/*
 * Driver initialization and shutdown
 */

/*
 * max927x_finish_probe expects to be called with the lock mutex
 * held, and releases it when complete.
 */
static void max927x_finish_probe(struct max927x *me)
{
	int ret;

	atomic_xchg(&me->i2c_xfer_flags, I2C_XFER_NOCOUNT);
	/*
	 * Now that the serializer/deserializer pair is fully up
	 * and running, we can enable the GPIOs and I2C communication
	 * to the devices that may be attached to the other side of
	 * the link.
	 */
	ret = setup_gpiochip(me);
	if (ret < 0) {
		dev_err(me->dev, "GPIO setup failed\n");
		power_down_remote(me);
		i2c_del_adapter(&me->dummy_adapter);
		me->curstate = STATE_UNINITIALIZED;
		mutex_unlock(&me->lock);
		return;
	}

	ret = i2c_add_numbered_adapter(&me->adap);
	if (ret < 0) {
		int gcret = gpiochip_remove(&me->gc);
		dev_err(me->dev, "failed to register I2C adapter: %d\n", ret);
		if (gcret < 0)
			dev_warn(me->dev, "error %d removing gpiochip\n", ret);
		i2c_del_adapter(&me->dummy_adapter);
		me->curstate = STATE_UNINITIALIZED;
		mutex_unlock(&me->lock);
		return;
	}

	ret = media_entity_init(&me->subdev.entity, 2, me->pads, 0);
	if (ret < 0)
		dev_err(me->dev, "could not initialize media entity: %d\n", ret);
	ret = sysfs_create_groups(&me->dev->kobj, attr_groups);
	if (ret < 0)
		dev_err(me->dev, "could not create sysfs groups\n");

	me->curstate = STATE_READY;
	atomic_xchg(&me->i2c_xfer_flags, 0);
	mutex_unlock(&me->lock);
	return;
}

static void initial_training(struct work_struct *work)
{
	struct max927x *me = container_of(work, struct max927x, training_work);
	unsigned int score[2];
	int ret, logain;

	mutex_lock(&me->lock);
	me->training = 1;
	atomic_xchg(&me->i2c_xfer_flags, I2C_XFER_NOCOUNT);

	for (logain = 0; logain < 2; logain++) {
		dev_notice(me->dev,
			   "%s: testing with REV_AMP=%u mVolts, LOGAIN=%d\n",
			   __func__, max9272_rev_amp_to_millivolts(me->current_rev_amp), logain);
		me->current_logain = logain;
		ret = ser_set_logain(me);
		if (ret >= 0) {
			mutex_unlock(&me->lock);
			ret = link_test(me, 1);
			mutex_lock(&me->lock);
		}
		if (ret < 0) {
			dev_err(me->dev, "%s: link test failed, err=%d\n", __func__, ret);
			if (ret == -ETIMEDOUT) {
				score[0] = score[1] = -ret;
				break;
			}
		}
		score[logain] = ret;
	}

	me->training = 0;

	if (score[0] < 0 && score[1] < 0) {
		max9272_millivolts_to_rev_amp(60, &me->current_rev_amp);
		dev_notice(me->dev, "%s: resetting control link with REV_AMP at %u mVolts\n",
			   __func__, max9272_rev_amp_to_millivolts(me->current_rev_amp));
		ret = me->ctrl_link_init(me);
		if (ret < 0) {
			dev_err(me->dev, "%s: could not establish control link, ret=%d\n",
				__func__, ret);
			power_down_remote(me);
			i2c_del_adapter(&me->dummy_adapter);
			me->curstate = STATE_UNINITIALIZED;
			mutex_unlock(&me->lock);
			return;
		} else {
			dev_notice(me->dev, "%s: leaving LOGAIN at %d after resetting control link\n",
				   __func__, me->current_logain);
			/*
			 * Call finish_probe before switching to TRAINING_OFF, so we
			 * don't end up recording spurious I2C retries during probe of the adapter
			 * we add there.
			 */
			max927x_finish_probe(me);
			return;
		}
	}
	logain = (score[0] < score[1] ? 0 : 1);
	dev_notice(me->dev, "%s: retry counts (%d, %d), selecting LOGAIN %d\n",
		   __func__, score[0], score[1], logain);
	me->current_logain = logain;
	ret = ser_set_logain(me);
	if (ret < 0) {
		dev_err(me->dev, "could not update LOGAIN, ret=%d\n", ret);
		power_down_remote(me);
		i2c_del_adapter(&me->dummy_adapter);
		mutex_unlock(&me->lock);
	} else
		max927x_finish_probe(me);

}

static int max927x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct i2c_driver *i2cdrv = to_i2c_driver(dev->driver);
	struct device_node *child;
	struct max927x *me;
	u32 remote_reg;
	u8 chipid = 0;
	int i, ret, skip_training = 0;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;
	if (ret < 0)
		dev_dbg(dev, "device_reset returned %d\n", ret);
	msleep(5);

	/* Chip ID is in same register for both 9271 & 9272 */
	ret = i2c_reg_read(client, max9271_settings[MAX9271_ID].reg, &chipid);
	if (ret < 0 || (chipid != MAX9271_CHIPID &&
			chipid != MAX9272_CHIPID)) {
		dev_err(dev, "chip id read failed, ret=%d, id=0x%x\n",
			ret, chipid);
		return -ENODEV;
	}
	dev_dbg(dev, "found %s\n", (chipid == MAX9271_CHIPID ? "max9271" : "max9272"));

	me = devm_kzalloc(dev, sizeof(*me), GFP_KERNEL);
	if (me == NULL)
		return -ENOMEM;

	/*
	 * XXX
	 * We may get probed before our driver's I2C client list
	 * gets initialized, so if we encounter an error and
	 * need to delete our I2C adapter structures, we need
	 * to do the list initialization ourselves to prevent
	 * a crash.
	 * XXX
	 */
	if (i2cdrv->clients.next == NULL && i2cdrv->clients.prev == NULL)
		INIT_LIST_HEAD(&i2cdrv->clients);

	atomic_set(&me->i2c_error_count, 0);
	for (i = 0; i < ARRAY_SIZE(me->i2c_retry_counts); i++)
		atomic_set(&me->i2c_retry_counts[i], 0);
	atomic_set(&me->ser_gpios_enabled, 0);
	atomic_set(&me->ser_gpios_set, 0);
	atomic_set(&me->remote_power_enabled, 0);
	atomic_set(&me->i2c_test_in_progress, 0);
	atomic_set(&me->i2c_xfer_flags, I2C_XFER_NOCOUNT);
	mutex_init(&me->lock);
	INIT_WORK(&me->training_work, initial_training);
	me->curstate = STATE_UNINITIALIZED;

	me->dev = dev;
	me->local = client;
	me->parent = to_i2c_adapter(client->dev.parent);
	snprintf(me->adap.name, sizeof(me->adap.name), "max927x-%d", i2c_adapter_id(me->parent));
	snprintf(me->dummy_adapter.name, sizeof(me->dummy_adapter.name), "max927x-d-%d",
		 i2c_adapter_id(me->parent));
	me->dummy_adapter.owner = me->adap.owner = THIS_MODULE;
	me->dummy_adapter.algo = me->adap.algo = &remote_i2c_algo;
	me->dummy_adapter.algo_data = me->adap.algo_data = me;
	me->dummy_adapter.dev.parent = me->adap.dev.parent = &me->parent->dev;
	me->dummy_adapter.class = me->adap.class = 0;
	ret = of_property_read_u32(np, "i2c-nr", &me->adap.nr);
	if (ret < 0)
		dev_err(dev, "could not read i2c-nr property\n");

	ret = of_property_read_u32(np, "remote-reg", &remote_reg);
	if (ret < 0) {
		dev_warn(dev, "missing remote-reg property, assuming default\n");
		remote_reg = (chipid == MAX9271_CHIPID ? 0x48 : 0x40);
	}
	ret = of_property_read_u32(np, "dummy-i2c-nr", &me->dummy_adapter.nr);
	if (ret < 0) {
		me->dummy_adapter.nr = me->adap.nr + 5;
		dev_warn(dev, "could not read dummy-i2c-nr property, using default: %d\n",
			 me->dummy_adapter.nr);
	}

	me->remote_power = devm_regulator_get(dev, "slave");
	of_property_read_u32(np, "slave-on-delay-ms", &me->power_on_mdelay);
	of_property_read_u32(np, "slave-off-delay-ms", &me->power_off_mdelay);

	for_each_child_of_node(np, child) {
		if (of_property_read_bool(child, "i2c")) {
			me->i2c_node = me->adap.dev.of_node = child;
			break;
		}
	}

	/*
	 * Register settings from the device tree.  See the property definitions
	 * at the top of this module.
	 */
	ret = of_property_read_u32_array(np, "cfg-0", me->of_cfg_settings, NUM_CFG_PROPS);
	if (ret < 0) {
		dev_err(dev, "missing required cfg-0 property\n");
		return -EINVAL;
	}
	for (i = 0; i < NUM_CFG_PROPS; i++) {
		if (me->of_cfg_settings[i] >= (1 << properties[i].setting->width)) {
			dev_err(dev, "cfg setting for %s too large\n",
				properties[i].dev_attr.attr.name);
			return -EINVAL;
		}
	}

	me->current_logain = 0;
	max9272_millivolts_to_rev_amp(80, &me->current_rev_amp);

	/*
	 * Create a separate i2c_adapter just for addressing the
	 * remote end of the link, to prevent premature probing
	 * of other devices that might be attached to the remote
	 * device.
	 */
	ret = i2c_add_numbered_adapter(&me->dummy_adapter);
	if (ret < 0) {
		dev_err(dev, "failed to register dummy I2C adapter: %d\n", ret);
		return ret;
	}
	dev_dbg(dev, "dummy adapter registered, number %d\n", me->dummy_adapter.nr);
	me->remote = i2c_new_dummy(&me->dummy_adapter, remote_reg);
	if (me->remote == NULL) {
		dev_err(dev, "could not create I2C client for remote\n");
		i2c_del_adapter(&me->dummy_adapter);
		return -EINVAL;
	}

	if (chipid == MAX9271_CHIPID) {
		me->ser = me->local;
		me->des = me->remote;
		me->ctrl_link_init = ser_control_init;
		me->pads[0].flags = MEDIA_PAD_FL_SOURCE;
		me->pads[1].flags = MEDIA_PAD_FL_SINK;
	} else {
		me->des = me->local;
		me->ser = me->remote;
		me->ctrl_link_init = des_control_init;
		me->pads[0].flags = MEDIA_PAD_FL_SINK;
		me->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	}

	ret = me->ctrl_link_init(me);
	if (ret < 0) {
		if (me->local == me->des) {
			max9272_millivolts_to_rev_amp(60, &me->current_rev_amp);
			dev_notice(dev, "%s: setting REV_AMP to %u mVolts to retry link setup\n",
				   __func__, max9272_rev_amp_to_millivolts(me->current_rev_amp));
			ret = me->ctrl_link_init(me);
			if (ret == 0)
				skip_training = 1;
		}
	}

	if (ret < 0) {
		dev_err(dev, "control link initialization/training failed, ret=%d\n", ret);
		i2c_del_adapter(&me->dummy_adapter);
		return -ENODEV;
	}

	/*
	 * And now set up our V4L2 subdevice.  The
	 * mxc_subdev_pipeline driver makes assumptions
	 * about the subdevdata and I2C client data,
	 * so set that up here.
	 */
	v4l2_subdev_init(&me->subdev, &subdev_ops);
	me->subdev.owner = THIS_MODULE;
	v4l2_set_subdevdata(&me->subdev, client);
	i2c_set_clientdata(client, &me->subdev);
	scnprintf(me->subdev.name, sizeof(me->subdev.name), "%s %d-%04x",
		  me->dev->driver->name, i2c_adapter_id(client->adapter), client->addr);

	ret = sysfs_create_group(&dev->kobj, &static_attr_group);
	if (ret < 0)
		dev_err(me->dev, "could not create static sysfs group, err=%d\n", ret);

	if (skip_training) {
		dev_notice(dev, "%s: skipping initial training due to REV_AMP reset\n", __func__);
		mutex_lock(&me->lock);
		max927x_finish_probe(me);
	} else
		schedule_work(&me->training_work);
	return 0;
}

static int max927x_remove(struct i2c_client *client)
{
	struct max927x *me = to_max927x_from_v4l2_subdev(i2c_get_clientdata(client));
	int i, ret;

	mutex_lock(&me->lock);
	for (i = 0; me->training && i < 30; i++) {
		mutex_unlock(&me->lock);
		dev_notice(me->dev, "%s: waiting for training to complete before removing\n", __func__);
		msleep(1000);
		mutex_lock(&me->lock);
	}
	cancel_work_sync(&me->training_work);
	if (me->curstate >= STATE_READY) {
		stream_control(&me->subdev, 0);
		sysfs_remove_groups(&me->dev->kobj, attr_groups);
		v4l2_device_unregister_subdev(&me->subdev);
		media_entity_cleanup(&me->subdev.entity);
		i2c_del_adapter(&me->adap);
		ret = gpiochip_remove(&me->gc);
		if (ret < 0)
			dev_err(me->dev, "error %d removing gpiochip\n", ret);
	}
	i2c_del_adapter(&me->dummy_adapter);
	power_down_remote(me);
	mutex_unlock(&me->lock);
	sysfs_remove_group(&me->dev->kobj, &static_attr_group);

	return 0;
}

static struct of_device_id max927x_dt_ids[] = {
	{ .compatible = "maxim,max927x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max927x_dt_ids);

static const struct i2c_device_id max927x_i2c_id[] = {
	{ "max927x", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max927x_i2c_id);

static struct i2c_driver max927x_driver = {
	.driver = {
		.name = "max927x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max927x_dt_ids),
	},
	.probe = max927x_probe,
	.remove = max927x_remove,
	.id_table = max927x_i2c_id,
};

module_i2c_driver(max927x_driver);

MODULE_AUTHOR("Leopard Imaging, Inc.");
MODULE_AUTHOR("Sarah Newman <sarah.newman@computer.org>");
MODULE_AUTHOR("Matt Madison <mmadison@sensity.com>");
MODULE_DESCRIPTION("max927x Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
