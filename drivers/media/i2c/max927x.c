/*
 * Copyright (C) 2013-2014 Leopard Imaging, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/i2c-mux.h>
#include <linux/gpio.h>
#include <linux/of_i2c.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include <linux/moduleparam.h>
#define I2C_RETRY_CNT_HIST 11
static unsigned long i2c_retries = I2C_RETRY_CNT_HIST-1;
module_param(i2c_retries, ulong, 0444);

/*Magic number, either higher or lower seems to lead to many bit errors,
* think very hard about changing
*/
static unsigned long i2c_udelay = 3000;
module_param(i2c_udelay, ulong, 0444);

#define dev_warn_ratelimit(...) do{ if(printk_ratelimit()) dev_warn( __VA_ARGS__); } while(0)

struct max927x_data {
	struct mutex data_lock;
	struct mutex gpio_lock;
	struct gpio_chip gpio_chip;
	bool gpio_chip_initialized;
	struct device *dev;
	struct i2c_adapter *parent;
	struct i2c_adapter adap;
	struct i2c_client* master;
	struct i2c_client* slave;
	struct regulator *slave_supply;
	bool deserializer_master;
	u8 gpio_en;
	u8 gpio_set;
	u8 des_gpio;
	unsigned rev_amp;
	unsigned rev_trf;
	unsigned rev_dig_flt;
	unsigned rev_logain;
	unsigned rev_higain;
	unsigned rev_hibw;
	unsigned rev_hivth;
	unsigned i2c_slvsh;
	unsigned i2c_mstbt;
	unsigned cmllvl;
	unsigned preemp;
	unsigned spread;
	unsigned slave_off_ms;
	unsigned slave_on_ms;
	struct v4l2_subdev	subdev;
	bool operational;
	unsigned error_count;
	unsigned i2c_retry_counts[I2C_RETRY_CNT_HIST];
};

static struct max927x_data *to_max927x_from_i2c(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct max927x_data, subdev);
}

static struct max927x_data *to_max927x_from_v4l2(const struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return to_max927x_from_i2c(client);
}

static struct max927x_data *to_max927x_from_dev(const struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	return container_of(sd, struct max927x_data, subdev);
}

static struct max927x_data *to_max927x_from_gpio(struct gpio_chip *gc)
{
	return container_of(gc, struct max927x_data, gpio_chip);
}

#define MAX9271_REG_02_SS_SHIFT (5)
#define MAX9271_REG_02_SS_MASK (0x7 << MAX9271_REG_02_SS_SHIFT)

#define MAX9271_REG_04_SEREN_SHIFT (7)
#define MAX9271_REG_04_CLINKEN_SHIFT (6)

#define MAX9272_REG_04_LOCKED_SHIFT (7)
#define MAX9272_REG_04_OUTENB_SHIFT (6)

#define MAX927X_REG_04_PRBSEN_SHIFT (5)
#define MAX927X_REG_04_SLEEP_SHIFT (4)
#define MAX927X_REG_04_INTTYPE_SHIFT (2)
#define MAX927X_REG_04_REVCCEN_SHIFT (1)
#define MAX927X_REG_04_FWDCCEN_SHIFT (0)


#define MAX9272_REG_04_SEREN_SHIFT (7)
#define MAX9272_REG_04_CLINKEN_SHIFT (6)
#define MAX9272_REG_04_PRBSEN_SHIFT (5)
#define MAX9272_REG_04_SLEEP_SHIFT (4)
#define MAX9272_REG_04_INTTYPE_SHIFT (2)
#define MAX9272_REG_04_REVCCEN_SHIFT (1)
#define MAX9272_REG_04_FWDCCEN_SHIFT (0)

#define MAX927X_REG_05_I2CMETHOD_SHIFT (7)

#define MAX9272_REG_05_DCS_SHIFT (6)
#define MAX9272_REG_05_HVTRMODE_SHIFT (5)
#define MAX9272_REG_05_ENEQ_SHIFT (4)
#define MAX9272_REG_05_EQTUNE_SHIFT (0)

#define MAX9271_REG_05_ENJITFILT_SHIFT (6)
#define MAX9271_REG_05_PRBSLEN_SHIFT (4)
#define MAX9271_REG_05_ENWAKEN_SHIFT (1)
#define MAX9271_REG_05_ENWAKEP_SHIFT (0)

#define MAX9271_REG_06_CMLLVL_SHIFT (4)
#define MAX9271_REG_06_CMLLVL_MASK (0xf << MAX9271_REG_06_CMLLVL_SHIFT)
#define MAX9271_REG_06_PREEMP_SHIFT (0)
#define MAX9271_REG_06_PREEMP_MASK (0xf << MAX9271_REG_06_PREEMP_SHIFT)

#define MAX927X_REG_07_DBL_SHIFT  (7)
#define MAX927X_REG_07_DRS_SHIFT  (6)
#define MAX927X_REG_07_BWS_SHIFT (5)
#define MAX927X_REG_07_ES_SHIFT (4)
#define MAX9272_REG_07_HVTRACK_SHIFT (3)
#define MAX927X_REG_07_HVEN_SHIFT (2)
#define MAX927X_REG_07_EDC_SHIFT (0)

#define MAX927X_REG_08_INVVS_SHIFT (7)
#define MAX927X_REG_08_INVHS_SHIFT (6)
#define MAX9272_REG_08_UNEQDBL_SHIFT (4)
#define MAX9272_REG_08_DISSTAG_SHIFT (3)
#define MAX9272_REG_08_AUTORST_SHIFT (2)
#define MAX9272_REG_08_ERRSEL_SHIFT (0)

#define MAX9271_REG_08_REV_DIG_FLT_SHIFT (4)
#define MAX9271_REG_08_REV_DIG_FLT_MASK (3 << MAX9271_REG_08_REV_DIG_FLT_SHIFT)
#define MAX9271_REG_08_REV_LOGAIN_SHIFT (3)
#define MAX9271_REG_08_REV_LOGAIN_MASK (1 << MAX9271_REG_08_REV_LOGAIN_SHIFT)
#define MAX9271_REG_08_REV_HIGAIN_SHIFT (2)
#define MAX9271_REG_08_REV_HIGAIN_MASK (1 << MAX9271_REG_08_REV_HIGAIN_SHIFT)
#define MAX9271_REG_08_REV_HIBW_SHIFT (1)
#define MAX9271_REG_08_REV_HIBW_MASK (1 << MAX9271_REG_08_REV_HIBW_SHIFT)
#define MAX9271_REG_08_REV_HIVTH_SHIFT (0)
#define MAX9271_REG_08_REV_HIVTH_MASK (1 << MAX9271_REG_08_REV_HIVTH_SHIFT)

#define MAX927X_REG_0D_I2CLOCACK_SHIFT (7)
#define MAX927X_REG_0D_I2CSLVSH_SHIFT  (5)
#define MAX927X_REG_0D_I2CSLVSH_MASK   (0x3 << MAX927X_REG_0D_I2CSLVSH_SHIFT)
#define MAX927X_REG_0D_I2CMSTBT_SHIFT  (2)
#define MAX927X_REG_0D_I2CMSTBT_MASK   (0x7 << MAX927X_REG_0D_I2CMSTBT_SHIFT)
#define MAX927X_REG_0D_I2CSLVTO_SHIFT  (0)

#define MAX9271_REG_0E_DIS_REV_P_SHIFT (7)
#define MAX9271_REG_0E_DIS_REV_N_SHIFT (6)
#define MAX9271_REG_0E_DIS_GPIO5EN_SHIFT (5)
#define MAX9271_REG_0E_DIS_GPIO4EN_SHIFT (4)
#define MAX9271_REG_0E_DIS_GPIO3EN_SHIFT (3)
#define MAX9271_REG_0E_DIS_GPIO2EN_SHIFT (2)
#define MAX9271_REG_0E_DIS_GPIO1EN_SHIFT (1)

#define MAX9271_REG_0F_GPIO5OUT_SHIFT (5)
#define MAX9271_REG_0F_GPIO4OUT_SHIFT (4)
#define MAX9271_REG_0F_GPIO3OUT_SHIFT (3)
#define MAX9271_REG_0F_GPIO2OUT_SHIFT (2)
#define MAX9271_REG_0F_GPIO1OUT_SHIFT (1)
#define MAX9271_REG_0F_SETGP0_SHIFT (0)

#define MAX9272_REG_15_REV_TRF_SHIFT (5)
#define MAX9272_REG_15_REV_TRF_MASK (0x3 << MAX9272_REG_15_REV_TRF_SHIFT)
#define MAX9272_REG_15_REV_TRF_211_100 (0 << MAX9272_REG_15_REV_TRF_SHIFT)
#define MAX9272_REG_15_REV_TRF_281_200 (1 << MAX9272_REG_15_REV_TRF_SHIFT)
#define MAX9272_REG_15_REV_TRF_422_300 (2 << MAX9272_REG_15_REV_TRF_SHIFT)
#define MAX9272_REG_15_REV_TRF_563_400 (3 << MAX9272_REG_15_REV_TRF_SHIFT)
#define MAX9272_REG_15_REV_AMP_SHIFT (0)
#define MAX9272_REG_15_REV_AMP_MASK (0xF << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_30  (0 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_40  (1 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_50  (2 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_60  (3 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_70  (4 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_80  (5 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_90  (6 << MAX9272_REG_15_REV_AMP_SHIFT)
#define MAX9272_REG_15_REV_AMP_100 (7 << MAX9272_REG_15_REV_AMP_SHIFT)

/*
 * Max9272 undocumented setting: REV_AMP
 * Reverse-channel transmitter pulse amplitude
 */
static inline __attribute__((unused)) unsigned int max9272_rev_amp_to_millivolts(u8 val) {
	return (val < 8 ? (30 + 10 * val) : (90 + 10 * (val - 8)));
}
static inline __attribute__((unused)) int max9272_millivolts_to_rev_amp(unsigned int mvolts, u8 *valp) {
	unsigned int centivolts = (mvolts + 5) / 10;
	if (centivolts < 3 || centivolts > 16)
		return -EINVAL;
	*valp = (centivolts < 10 ? centivolts - 3 : (centivolts - 9) + 8);
	return 0;
}
static int _max927x_try_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	u8 au8RegBuf;
	struct i2c_msg msgs[2];

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	au8RegBuf = reg;
	msgs[0].buf = &au8RegBuf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;
	return i2c_transfer(client->adapter, msgs, 2);
}

static int _max927x_read_reg(struct i2c_client *client, u8 reg, u8 *val, unsigned *error_count)
{
	int err = -EIO;

	err =  _max927x_try_read_reg(client, reg, val);

	if (err < 0) {
		dev_err(&client->dev,"%s: error:reg=%x,val=%x,err=%d\n",
					__func__, reg, *val, err);
		if(error_count)
			*error_count += 1;
		return err;
	}

	return 0;
}

static int _max927x_try_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	u8 au8Buf[2];

	au8Buf[0] = reg;
	au8Buf[1] = val;

	ret = i2c_master_send(client, au8Buf, 2);
	return ret;
}


static int _max927x_write_reg(struct i2c_client *client, u8 reg, u8 val, u8 mask, unsigned *error_count)
{
	int err = -EIO;
	int i;
	u8 check;

	for(i = 0; i < i2c_retries; i++) {
		msleep(5);
		/*Write data*/
		err = _max927x_try_write_reg(client, reg, val);
		if (err < 0) {
			dev_warn_ratelimit(&client->dev,"%s warn:reg=%x,val=%x,err=%d\n",
					__func__, reg, val, err);
			continue;
		}

		/*Read back data just written*/
		err = _max927x_read_reg(client, reg, &check, NULL);
		if(err < 0)
			continue;

		if((check & mask) != (val & mask)) {
			dev_warn_ratelimit(&client->dev,"%s:check warn:reg=%x,val=%x,check=%x\n", __func__,
				reg, val, check);
			err = -EIO;
			continue;
		}

		/*All conditions satisfied (data written, read back
		 * and verified to be the same value
		 */
		return 0;
	}
	if(error_count)
		*error_count += 1;

	dev_err(&client->dev,"%s warn:reg=%x,val=%x,err=%d,check=%02hhx\n",
					__func__, reg, val, err, check);

	return err;
}

#define SLAVE_WRITE(reg,val,error) _max927x_write_reg(data->slave, reg, val, 0xff, error)
#define MASTER_WRITE(reg,val,error) _max927x_write_reg(data->master, reg, val, 0xff, error)
#define SER_WRITE(reg,val,error) \
	_max927x_write_reg(data->deserializer_master ? data->slave : data->master, \
			reg, val, 0xff, error)
#define SER_READ(reg,val,error) \
	_max927x_read_reg(data->deserializer_master ? data->slave : data->master, \
			reg, val, error)

#define DES_WRITE(reg,val,error) \
	_max927x_write_reg(data->deserializer_master ? data->master : data->slave, \
			reg, val, 0xff, error)
#define DES_WRITE_MASK(reg,val,mask,error) \
	_max927x_write_reg(data->deserializer_master ? data->master : data->slave, \
			reg, val, mask, error)
#define DES_READ(reg,val,error) \
	_max927x_read_reg(data->deserializer_master ? data->master : data->slave, \
			reg, val, error)

static inline u8 _max9272_magic(struct max927x_data* data)
{
	return (data->rev_amp << MAX9272_REG_15_REV_AMP_SHIFT) |
			(data->rev_trf << MAX9272_REG_15_REV_TRF_SHIFT);
}

static inline u8 _max9271_magic(struct max927x_data* data)
{
	return (0 << MAX927X_REG_08_INVVS_SHIFT) |
			(0 << MAX927X_REG_08_INVHS_SHIFT) |
			(data->rev_dig_flt << MAX9271_REG_08_REV_DIG_FLT_SHIFT) |
			(data->rev_logain << MAX9271_REG_08_REV_LOGAIN_SHIFT) |
			(data->rev_higain << MAX9271_REG_08_REV_HIGAIN_SHIFT) |
			(data->rev_hibw << MAX9271_REG_08_REV_HIBW_SHIFT) |
			(data->rev_hivth << MAX9271_REG_08_REV_HIVTH_SHIFT);
}

static inline u8 _max9271_preemp(struct max927x_data* data)
{
	return (data->preemp << MAX9271_REG_06_PREEMP_SHIFT) |
			(data->cmllvl << MAX9271_REG_06_CMLLVL_SHIFT);
}

static inline u8 _max9271_spread(struct max927x_data* data)
{
	return (data->spread << MAX9271_REG_02_SS_SHIFT) | 0x1f;
}

static inline u8 _max927x_i2c(struct max927x_data* data)
{
	return (1 << MAX927X_REG_0D_I2CLOCACK_SHIFT) |
			 (data->i2c_slvsh << MAX927X_REG_0D_I2CSLVSH_SHIFT) |
			 (data->i2c_mstbt << MAX927X_REG_0D_I2CMSTBT_SHIFT) |
			 (0x2 << MAX927X_REG_0D_I2CSLVTO_SHIFT);
}

#define MAX9271_REG04 (0 << MAX9271_REG_04_SEREN_SHIFT) | \
			 (1 << MAX9271_REG_04_CLINKEN_SHIFT) | \
			 (0 << MAX927X_REG_04_PRBSEN_SHIFT) | \
			 (0 << MAX927X_REG_04_SLEEP_SHIFT) | \
			 (0x1 << MAX927X_REG_04_INTTYPE_SHIFT) | \
			 (1 << MAX927X_REG_04_REVCCEN_SHIFT) | \
			 (1 << MAX927X_REG_04_FWDCCEN_SHIFT)

#define MAX9271_REG07 (1 << MAX927X_REG_07_DBL_SHIFT) | \
			 (0 << MAX927X_REG_07_DRS_SHIFT) | \
			 (0 << MAX927X_REG_07_BWS_SHIFT) | \
			 (0 << MAX927X_REG_07_ES_SHIFT) | \
			 (1 << MAX927X_REG_07_HVEN_SHIFT) | \
			 (0x2 << MAX927X_REG_07_EDC_SHIFT)

#define MAX9272_REG07 (1 << MAX927X_REG_07_DBL_SHIFT) | \
			 (0 << MAX927X_REG_07_DRS_SHIFT) | \
			 (0 << MAX927X_REG_07_BWS_SHIFT) | \
			 (1 << MAX927X_REG_07_ES_SHIFT) | \
			 (0 << MAX9272_REG_07_HVTRACK_SHIFT) | \
			 (1 << MAX927X_REG_07_HVEN_SHIFT) | \
			 (0x2 << MAX927X_REG_07_EDC_SHIFT)

#define MAX9272_REG08 (0 << MAX927X_REG_08_INVVS_SHIFT) | \
			 (0 << MAX927X_REG_08_INVHS_SHIFT) | \
			 (0 << MAX9272_REG_08_UNEQDBL_SHIFT ) | \
			 (1 << MAX9272_REG_08_DISSTAG_SHIFT) | \
			 (1 << MAX9272_REG_08_AUTORST_SHIFT) | \
			 (0x2 << MAX9272_REG_08_ERRSEL_SHIFT)

static int _max927x_video_enable(struct max927x_data* data, int enable)
{
	return SER_WRITE(0x04,
			((enable ? 1 : 0) << MAX9271_REG_04_SEREN_SHIFT) | MAX9271_REG04, &data->error_count);
}

#define LOCK_MAX_RETRIES (5)
static int _max9272_link_locked(struct max927x_data* data)
{
	int i;
	int err;
	u8 val;

	for(i = 0; i < LOCK_MAX_RETRIES; i++) {
		err = SER_READ(0x04, &val, NULL);
		if(err == 0 && (val & (1 << MAX9272_REG_04_LOCKED_SHIFT)))
			break;
		msleep(5);
	}
	if(i == LOCK_MAX_RETRIES) {
		return -EIO;
	}
	return 0;
}

static int max9271_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct max927x_data* data = to_max927x_from_gpio(gc);
	u8 val;
	int ret;

	printk(KERN_INFO "%s: off=%d\n", __func__, off);

	if (!data->operational)
		return -ENODEV;

	if (off < 6) {
		/* Serializer GPIOs: 1-5 */
		mutex_lock(&data->gpio_lock);
		ret = SER_READ(0x10, &val, &data->error_count);
		mutex_unlock(&data->gpio_lock);
		if(ret < 0) {
			dev_err(data->dev,"error in %s\n",__func__);
			return 0;
		}

		return (val & (1u << off)) ? 1 : 0;
	} else {
		/* De-serializer GPIOs: 6-7 */
		mutex_lock(&data->gpio_lock);
		ret = DES_READ(0x0E, &val, &data->error_count);
		mutex_unlock(&data->gpio_lock);
		if(ret < 0) {
			dev_err(data->dev,"error in %s\n",__func__);
			return 0;
		}

		/* GPIO6 - bit 0, GPIO7 - bit 2*/
		off -= 6;
		off *= 2;
		return (val >> off) & 1;
	}

}

static void max9271_gpio_set_value(struct gpio_chip *gc, unsigned off, int output_val)
{
	struct max927x_data* data = to_max927x_from_gpio(gc);
	int ret;

	printk(KERN_INFO "%s: off=%d, output=%d\n", __func__, off, output_val);

	if (!data->operational)
		return;

	mutex_lock(&data->gpio_lock);

	if(off < 6) {
		if(output_val)
			data->gpio_set |= (1 << off);
		else
			data->gpio_set &= ~(1 << off);

		ret = SER_WRITE(0x0F, data->gpio_set, &data->error_count);
	} else {
		u8 bits = 0;
		/* GPIO6 - bit 1, GPIO7 - bit 3 */
		off -= 6; // 0 or 1
		off *= 2; // 0 or 2
		off += 1; // 1 or 3
		bits = (1 << off);
		data->des_gpio &= ~bits;
		if(output_val)
			data->des_gpio |= bits;

		ret = DES_WRITE_MASK(0x0E, data->des_gpio, 0x6a, &data->error_count);
	}

	mutex_unlock(&data->gpio_lock);
	if(ret < 0) {
		dev_err(data->dev,"error in %s\n",__func__);
	}

}

static int max9721_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct max927x_data* data = to_max927x_from_gpio(gc);
	int ret = 0;

	printk(KERN_INFO "%s: off=%d\n", __func__, off);

	if (!data->operational)
		return 0;

	if(off == 0) {
		dev_err(data->dev, "Bad gpio offset\n");
		return -EINVAL;
	}

	/* max9272 GPIOs are OD only, so just set them 'high'. */
	if (off > 5) {
		max9271_gpio_set_value(gc, off, 1);
		return ret;
	}

	mutex_lock(&data->gpio_lock);
	data->gpio_en &= ~(1 << off);
	ret = SER_WRITE(0x0E, data->gpio_en, &data->error_count);
	mutex_unlock(&data->gpio_lock);
	if(ret < 0) {
		dev_err(data->dev,"error in %s\n",__func__);
	}

	return ret;
}

static int max9721_gpio_direction_output(struct gpio_chip *gc, unsigned off, int value)
{
	struct max927x_data* data = to_max927x_from_gpio(gc);
	int ret = 0;

	printk(KERN_INFO "%s: off=%d value=0x%x\n", __func__, off, value);

	if (!data->operational)
		return 0;

	max9271_gpio_set_value(gc, off, value);
	if (off > 5)
		return ret;

	mutex_lock(&data->gpio_lock);
	data->gpio_en |= 1 << off;
	ret = SER_WRITE(0x0E, data->gpio_en, &data->error_count);
	mutex_unlock(&data->gpio_lock);
	if(ret < 0) {
		dev_err(data->dev,"error in %s\n",__func__);
	}
	return ret;
}

static int _max927x_slave_power_on(struct max927x_data* data)
{
	int retval = 0;
	if(!data->slave_supply)
		return 0;

	retval = regulator_enable(data->slave_supply);
	if(retval < 0)
		return retval;
	msleep(data->slave_on_ms);
	return 0;

}

static int _max927x_slave_power_off(struct max927x_data* data)
{
	int retval = 0;
	if(!data->slave_supply)
		return 0;

	retval = regulator_disable(data->slave_supply);
	if(retval < 0)
		return retval;
	msleep(data->slave_off_ms);
	return 0;

}

static int _max927x_link_configure(struct max927x_data* data)
{
	struct device *dev = data->dev;
	int retval=0;
	int counter;

	retval = _max927x_slave_power_off(data);
	if(retval < 0)
		goto error;

	/*Write magic before powering slave*/
	if(data->deserializer_master)
		retval = MASTER_WRITE(0x15, _max9272_magic(data), &data->error_count);
	else
		retval = MASTER_WRITE(0x8, _max9271_magic(data), &data->error_count);
	if(retval < 0) {
		_max927x_slave_power_on(data);
		goto error;
	}
	dev_dbg(dev, "max927x init\n");

	/*Allow magic to take*/
	msleep(1000);

	/*Set local i2c config*/
	retval = MASTER_WRITE(0x0d, _max927x_i2c(data), &data->error_count);
	if(retval < 0) {
		_max927x_slave_power_on(data);
		goto error;
	}

	/*Power slave back on*/
	retval = _max927x_slave_power_on(data);
	if(retval < 0)
		goto error;

	for (counter = 1; counter <= 5; counter++) {

		/*Enable reverse channel so we know if remote writes succeed*/
		retval  = SER_WRITE(0x04, MAX9271_REG04, &data->error_count);
		if (retval == 0)
			break;
		dev_dbg(dev, "%s: enable reverse channel try %d err %d",
			__func__, counter, retval);
		/*Allow some additional time for power on*/
		msleep(10);
	}

	if (retval < 0)
		goto error;

	/*Write slaves magic*/
	if(data->deserializer_master) {
		retval = SLAVE_WRITE(0x8, _max9271_magic(data), &data->error_count);
		if (retval >= 0)
			retval = SLAVE_WRITE(0x6, _max9271_preemp(data), &data->error_count);

		if (retval >= 0)
			retval = SLAVE_WRITE(0x2, _max9271_spread(data), &data->error_count);

	} else {
		retval = SLAVE_WRITE(0x15, _max9272_magic(data), &data->error_count);
	}

	if(retval < 0)
		goto error;

	/*Set remote i2c config*/
	retval = SLAVE_WRITE(0x0d, _max927x_i2c(data), &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d setting remote I2C config", retval);
		goto error;
	}

	/*Set slave value for 0x07 first because we lose contact when the common settings mismatch*/
	SLAVE_WRITE(0x07, data->deserializer_master ? MAX9271_REG07 : MAX9272_REG07, NULL);
	i2c_lock_adapter(&data->adap);
	memset(data->i2c_retry_counts, 0, sizeof(data->i2c_retry_counts));
	i2c_unlock_adapter(&data->adap);

	/*Set local to match slave*/
	retval = MASTER_WRITE(0x07, data->deserializer_master ?
			MAX9272_REG07 : MAX9271_REG07, &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d setting local to match slave", retval);
		goto error;
	}

	/*Set deserializer video settings for generating local bus*/
	retval = DES_WRITE(0x08, MAX9272_REG08, &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d setting deserializer video settings", retval);
		goto error;
	}

	/*Clear automatic acks since we shouldn't need them anymore*/
	retval = MASTER_WRITE(0x0d,
			_max927x_i2c(data) & ~(1 << MAX927X_REG_0D_I2CLOCACK_SHIFT),
			&data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d clearing automatic ACKs", retval);
		goto error;
	}

	data->gpio_en = 0x42;
	data->gpio_set = 0xFE;
	data->des_gpio = 0x6a;

	retval = SER_WRITE(0x0F, data->gpio_set, &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d writing gpio_set", retval);
		goto error;
	}

	retval = SER_WRITE(0x0E, data->gpio_en, &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d writing gpio_en", retval);
		goto error;
	}

	retval = DES_WRITE_MASK(0x0E, data->des_gpio, 0x6a, &data->error_count);
	if(retval < 0) {
		dev_err(dev, "err %d writing des_gpio", retval);
		goto error;
	}

	dev_dbg(dev, "max927x init done\n");
	return retval;

error:
	dev_err(dev, "max927x init failed\n");
	return retval;
}

static ssize_t max9272_rev_trf_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 3) {
		dev_err(dev,"Must supply value between 0-3.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_trf = val;
	mutex_unlock(&data->data_lock);

	return count;
}

static ssize_t max9272_rev_trf_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	switch(data->rev_trf) {
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

static ssize_t max9271_rev_dig_flt_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 3) {
		dev_err(dev,"Must supply value between 0-3.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_dig_flt = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_rev_dig_flt_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);

	return sprintf(buf, "%d\n", data->rev_dig_flt);
}
static DEVICE_ATTR(magic_rev_dig_flt, 0666, (void *)max9271_rev_dig_flt_show, (void *)max9271_rev_dig_flt_store);

static ssize_t max9271_rev_logain_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_logain = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_rev_logain_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);

	return sprintf(buf, "%d\n", data->rev_logain);
}
static DEVICE_ATTR(magic_rev_logain, 0666, (void *)max9271_rev_logain_show, (void *)max9271_rev_logain_store);

static ssize_t max9271_rev_higain_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_higain = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_rev_higain_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);

	return sprintf(buf, "%d\n", data->rev_higain);
}
static DEVICE_ATTR(magic_rev_higain, 0666, (void *)max9271_rev_higain_show, (void *)max9271_rev_higain_store);

static ssize_t max9271_rev_hibw_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_hibw = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_rev_hibw_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);

	return sprintf(buf, "%d\n", data->rev_hibw);
}
static DEVICE_ATTR(magic_rev_hibw, 0666, (void *)max9271_rev_hibw_show, (void *)max9271_rev_hibw_store);

static ssize_t max9271_rev_hivth_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_hivth = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_rev_hivth_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);

	return sprintf(buf, "%d\n", data->rev_hivth);
}
static DEVICE_ATTR(magic_rev_hivth, 0666, (void *)max9271_rev_hivth_show, (void *)max9271_rev_hivth_store);

static ssize_t max927x_i2c_slvsh_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 3) {
		dev_err(dev,"Must supply value between 0-3.");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->i2c_slvsh = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max927x_i2c_slvsh_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n", data->i2c_slvsh);
}
static DEVICE_ATTR(i2c_slvsh, 0666, (void *)max927x_i2c_slvsh_show, (void *)max927x_i2c_slvsh_store);

static ssize_t max927x_i2c_mstbt_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 7) {
		dev_err(dev,"Must supply value between 0-7.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->i2c_mstbt = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max927x_i2c_mstbt_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n", data->i2c_mstbt);
}
static DEVICE_ATTR(i2c_mstbt, 0666, (void *)max927x_i2c_mstbt_show, (void *)max927x_i2c_mstbt_store);

static ssize_t max9272_rev_amp_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 15) {
		dev_err(dev,"Must supply value between 0-7.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->rev_amp = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9272_rev_amp_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d:%dmV\n",data->rev_amp,data->rev_amp*10+30);
}
static DEVICE_ATTR(magic_rev_amp, 0666, (void *)max9272_rev_amp_show, (void *)max9272_rev_amp_store);

static ssize_t max9271_cmllvl_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 15) {
		dev_err(dev,"Must supply value between 0-15.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->cmllvl = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_cmllvl_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n",data->cmllvl);
}
static DEVICE_ATTR(cmllvl, 0666, (void *)max9271_cmllvl_show, (void *)max9271_cmllvl_store);

static ssize_t max9271_preemp_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 15) {
		dev_err(dev,"Must supply value between 0-15.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->preemp = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_preemp_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n",data->preemp);
}
static DEVICE_ATTR(preemp, 0666, (void *)max9271_preemp_show, (void *)max9271_preemp_store);

static ssize_t max9271_spread_store(struct device *dev,
					struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;
	if(kstrtouint(buf, 10, &val) || val > 7) {
		dev_err(dev,"Must supply value between 0-7.\n");
		return count;
	}
	mutex_lock(&data->data_lock);
	data->spread = val;
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max9271_spread_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n",data->spread);
}
static DEVICE_ATTR(spread, 0666, (void *)max9271_spread_show, (void *)max9271_spread_store);

static ssize_t max9272_show_regs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	int cnt = 0;
	u8 i;
	u8 val;
	int err;
	mutex_lock(&data->data_lock);
	for(i=0; i < 0x20 && cnt < PAGE_SIZE; i++) {
		err = DES_READ(i, &val, NULL);
		if(err)
			continue;
		cnt += snprintf(buf+cnt,PAGE_SIZE-cnt,"%02x:%02x\n", i, val);
	}
	mutex_unlock(&data->data_lock);
	return cnt;
}
static DEVICE_ATTR(max9272_regs, 0444, (void *)max9272_show_regs, NULL);

static ssize_t max9271_show_regs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	int cnt = 0;
	u8 i;
	u8 val;
	int err;
	mutex_lock(&data->data_lock);
	for(i=0; i < 0x20 && cnt < PAGE_SIZE; i++) {
		err = SER_READ(i, &val, NULL);
		if(err)
			continue;
		cnt += snprintf(buf+cnt,PAGE_SIZE-cnt,"%02x:%02x\n", i, val);
	}
	mutex_unlock(&data->data_lock);
	return cnt;
}
static DEVICE_ATTR(max9271_regs, 0444, (void *)max9271_show_regs, NULL);

static ssize_t max9272_store_retry_count(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	i2c_lock_adapter(&data->adap);
	memset(data->i2c_retry_counts, 0, sizeof(data->i2c_retry_counts));
	i2c_unlock_adapter(&data->adap);
	return count;
}

static ssize_t max9272_show_retry_count(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	int i, cnt = 0;
	unsigned i2c_retry_counts[sizeof(data->i2c_retry_counts)];
	i2c_lock_adapter(&data->adap);
	memcpy(i2c_retry_counts, data->i2c_retry_counts,sizeof(i2c_retry_counts));
	//memset(data->i2c_retry_counts, 0, sizeof(data->i2c_retry_counts));
	i2c_unlock_adapter(&data->adap);
	for(i = 0; i < I2C_RETRY_CNT_HIST; i++) {
		cnt += snprintf(buf+cnt,PAGE_SIZE-cnt,"%u:%u\n", i, i2c_retry_counts[i]);
	}
	return cnt;
}
static DEVICE_ATTR(i2c_retry_counts, 0644, (void *)max9272_show_retry_count, (void*)max9272_store_retry_count);

static ssize_t max9272_show_link_errors(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	u8 corrected_err_cnt;
	int ret = 0;
	mutex_lock(&data->data_lock);
	ret = DES_READ(0x12,&corrected_err_cnt, &data->error_count);
	mutex_unlock(&data->data_lock);
	if (ret != 0) {
		dev_err(dev,"%s:read error\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE,"%d\n", corrected_err_cnt);
}
static DEVICE_ATTR(corrected_link_errors, 0444, (void *)max9272_show_link_errors, NULL);

static ssize_t max9272_show_detected_link_errors(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	u8 detected_err_cnt;
	int ret = 0;
	mutex_lock(&data->data_lock);
	ret = DES_READ(0x10,&detected_err_cnt, &data->error_count);
	mutex_unlock(&data->data_lock);
	if (ret != 0) {
		dev_err(dev,"%s:read error\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE,"%d\n", detected_err_cnt);
}
static DEVICE_ATTR(detected_link_errors, 0444, (void *)max9272_show_detected_link_errors, NULL);

static void _max927x_i2c_test(struct i2c_client * client, struct device *dev, const char *name, unsigned cycles)
{
	static const u8 reg =  0x0A; // a register for testing
	u8 val, w_val;
	int w_retry_cnt, r_retry_cnt, check_cnt;
	int i, j, k;
	int fail_cnt;
	int ret=0;
	int max_retry_cnt = 0;
	int total_transactions = 0;

	dev_err(dev, "%s i2c test running ......", name);

	w_retry_cnt = 0;
	r_retry_cnt = 0;
	check_cnt = 0;
	fail_cnt = 0;
	for (i=0; i< cycles; i++) {
		if ( i % (cycles / 100) == 0) {
			dev_err(dev, "test in progress : %d%%, w_retry_cnt = %d, r_retry_cnt = %d, check_cnt=%d", \
				i / (cycles / 100) , w_retry_cnt, r_retry_cnt, check_cnt);
			dev_err(dev, "                         fail_cnt = %d",  \
				fail_cnt);
		}

		w_val = (i << 1) & 0x0FFE;

		ret = -1;

		for(j = 0; j <= i2c_retries && (ret < 0 || w_val != val); j++) {
			u8 au8Buf[2];
			au8Buf[0] = reg;
			au8Buf[1] = w_val;
			ret = i2c_master_send(client, au8Buf, 2);
			total_transactions++;
			if(ret < 0) {
				dev_warn(&client->dev,"%s:write reg warn:reg=%x,val=%x\n",
						__func__, reg, w_val);
				w_retry_cnt++;
				continue;
			}

			ret = -1;
			for(k = 0; k < i2c_retries && ret < 0; k++) {
				ret = _max927x_try_read_reg(client, reg, &val);
				total_transactions++;
				if(ret < 0) {
					r_retry_cnt++;
					msleep(5);
				}
			}
			if(ret >= 0 && val != w_val) {
				dev_warn(&client->dev,"%s:check reg warn:reg=%x,val=%x,check=%x\n",
						__func__, reg, w_val, val);
				check_cnt++;
			}
		}
		if (ret < 0 || w_val != val) {
			fail_cnt++;
			dev_err(dev, "write failed: i = %d, write = 0x%x, w_retry_cnt=%d, r_retry_cnt=%d, check_cnt=%d\n",
					i, w_val, w_retry_cnt, r_retry_cnt, check_cnt);
			goto out;
		} else if(j > max_retry_cnt) {
			max_retry_cnt = j;
		}
	}
out:
	_max927x_write_reg(client, reg, 0x0, 0xff, NULL);

	dev_err(dev, "%s i2c test %s !, total test count = %d, fail_cnt=%d, w_retry_cnt = %d, "
			"r_retry_cnt = %d, check_cnt = %d, max retries=%d, total transactions=%d\n",
				name, fail_cnt == 0 ? "passed" : "failed", cycles, fail_cnt, w_retry_cnt,
						r_retry_cnt, check_cnt, max_retry_cnt,total_transactions);
}


static ssize_t max9272_i2c_test(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned val;
	if(kstrtouint(buf, 10, &val) || val < 100) {
		dev_err(dev,"Must supply test cycles > 100 in decimal.");
		return count;
	}
	mutex_lock(&data->data_lock);
	_max927x_i2c_test(data->deserializer_master ? data->master : data->slave, dev, "max9272", val);
	mutex_unlock(&data->data_lock);
	return count;
}
static DEVICE_ATTR(max9272_i2c_test, 0222, NULL, (void *)max9272_i2c_test);

static ssize_t max9271_i2c_test(struct device *dev,
				   struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned val;
	if(kstrtouint(buf, 10, &val) || val < 100) {
		dev_err(dev,"Must supply test cycles > 100 in decimal.");
		return count;
	}
	mutex_lock(&data->data_lock);
	_max927x_i2c_test(data->deserializer_master ? data->slave : data->master, dev, "max9271", val);
	mutex_unlock(&data->data_lock);
	return count;
}
static DEVICE_ATTR(max9271_i2c_test, 0222, NULL, (void *)max9271_i2c_test);

static ssize_t show_current_logain(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x_data *data = to_max927x_from_dev(dev);
	u8 val;
	int ret;

	mutex_lock(&data->data_lock);
	ret = SER_READ(0x08, &val, &data->error_count);
	mutex_unlock(&data->data_lock);
	if (ret < 0)
		return ret;
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 (val & MAX9271_REG_08_REV_LOGAIN_MASK) >> MAX9271_REG_08_REV_LOGAIN_SHIFT);
}

static ssize_t set_current_logain(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max927x_data *data = to_max927x_from_dev(dev);
	unsigned int val;
	u8 reg08val;
	int ret;

	if (kstrtouint(buf, 10, &val) || (val != 0 && val != 1))
		return EINVAL;
	mutex_lock(&data->data_lock);
	reg08val = _max9271_magic(data) & ~MAX9271_REG_08_REV_LOGAIN_MASK;
	reg08val |= val << MAX9271_REG_08_REV_LOGAIN_SHIFT;
	ret = SER_WRITE(0x08, reg08val, &data->error_count);
	mutex_unlock(&data->data_lock);
	return (ret < 0) ? ret : count;
}
static DEVICE_ATTR(current_logain, 0666, show_current_logain, set_current_logain);

static ssize_t show_current_rev_amp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max927x_data *data = to_max927x_from_dev(dev);
	u8 val;
	int ret;

	mutex_lock(&data->data_lock);
	ret = DES_READ(0x15, &val, &data->error_count);
	mutex_unlock(&data->data_lock);
	if (ret < 0)
		return ret;
	val &= MAX9272_REG_15_REV_AMP_MASK;
	val >>= MAX9272_REG_15_REV_AMP_SHIFT;
	return scnprintf(buf, PAGE_SIZE, "%u (%u mVolts)\n", val,
			 max9272_rev_amp_to_millivolts(val));
}

static ssize_t set_current_rev_amp(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct max927x_data *data = to_max927x_from_dev(dev);
	unsigned int mvolts;
	u8 val, reg15val;
	int ret;

	if (kstrtouint(buf, 10, &mvolts))
		return EINVAL;
	ret = max9272_millivolts_to_rev_amp(mvolts, &val);
	if (ret < 0)
		return ret;
	mutex_lock(&data->data_lock);
	reg15val = _max9272_magic(data) & ~MAX9272_REG_15_REV_AMP_MASK;
	reg15val |= val << MAX9272_REG_15_REV_AMP_SHIFT;
	ret = DES_WRITE(0x15, reg15val, &data->error_count);
	mutex_unlock(&data->data_lock);
	return (ret < 0) ? ret : count;
}
static DEVICE_ATTR(current_rev_amp, 0666, show_current_rev_amp, set_current_rev_amp);

static int _max927x_of_get_config(struct max927x_data* data, struct device_node	*np, unsigned index)
{
	enum {
		CFG_REV_AMP,
		CFG_REV_TRF,
		CFG_REV_DIG_FLT,
		CFG_REV_LOGAIN,
		CFG_REV_HIGAIN,
		CFG_REV_HIBW,
		CFG_REV_HIVTH,
		CFG_I2C_SLVSH,
		CFG_I2C_MSTBT,
		CFG_CMLLVL,
		CFG_PREEMP,
		CFG_SPREAD,
		NUM_CFG_PROPS
	};
	int i, ret;
	u8 limits[NUM_CFG_PROPS] = {15,3,3,1,1,1,1,3,7,15,15,7};
	u32 cfg[NUM_CFG_PROPS];
	char buf[sizeof("cfg-")+2];
	if(index > 99) {
		dev_dbg(data->dev, "%s:index too big\n",__func__);
		return -EINVAL;
	}
	snprintf(buf,sizeof(buf),"cfg-%d",index);

	ret = of_property_read_u32_array(np,buf,cfg,NUM_CFG_PROPS);
	if(ret < 0) {
		dev_dbg(data->dev, "%s:could not read %s, code %d\n", __func__, buf, ret);
		return -EINVAL;
	}
	for(i=0; i < NUM_CFG_PROPS; i++) {
		if(cfg[i] > limits[i]) {
			dev_dbg(data->dev, "%s:prop %d: cfg %d exceeds %d\n",
					__func__, i, cfg[i], limits[i]);
			return -EINVAL;
		}
	}
	data->rev_amp = cfg[CFG_REV_AMP];
	data->rev_trf = cfg[CFG_REV_TRF];
	data->rev_dig_flt = cfg[CFG_REV_DIG_FLT];
	data->rev_logain = cfg[CFG_REV_LOGAIN];
	data->rev_higain = cfg[CFG_REV_HIGAIN];
	data->rev_hibw = cfg[CFG_REV_HIBW];
	data->rev_hivth = cfg[CFG_REV_HIVTH];
	data->i2c_slvsh = cfg[CFG_I2C_SLVSH];
	data->i2c_mstbt = cfg[CFG_I2C_MSTBT];
	data->cmllvl = cfg[CFG_CMLLVL];
	data->preemp = cfg[CFG_PREEMP];
	data->spread = cfg[CFG_SPREAD];
	return 0;
}

static struct attribute *attributes[] = {
		&dev_attr_max9271_regs.attr,
		&dev_attr_detected_link_errors.attr,
		&dev_attr_corrected_link_errors.attr,
		&dev_attr_max9271_i2c_test.attr,
		&dev_attr_i2c_retry_counts.attr,
		&dev_attr_current_logain.attr,
		&dev_attr_current_rev_amp.attr,
		NULL,
};

static const struct attribute_group attr_group = {
	.attrs	= attributes,
};


static int i2c_max927x_xfer(struct i2c_adapter *adap,
			       struct i2c_msg msgs[], int num)
{
	struct max927x_data *data = adap->algo_data;
	int j, ret = 0;
	unsigned delay = i2c_udelay;
	/*this is delay rather than sleep so that it is more
	 * predictable, meaning it is easier to guarantee that
	 * what works under one workload will continue to work
	 * under a different workload.
    */
	while(delay > 1000) {
		udelay(delay);
		delay -= 1000;
	}
	udelay(delay);
	/*The messages must be sent all at once so that there is a repeated
	 * start condition instead of start/stop in between the messages.  This
	 * is required by one of the drivers using this adapter.
	 */
	for(j = 0; j < i2c_retries; j++) {
		ret = data->parent->algo->master_xfer(data->parent, msgs,num);
		if(ret >= 0)
			break;
	}
	if(j < I2C_RETRY_CNT_HIST)
		data->i2c_retry_counts[j]++;
	else
		data->i2c_retry_counts[I2C_RETRY_CNT_HIST-1]++;
	return ret;
}

static u32 i2c_max927x_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm i2c_max927x_algo = {
	.master_xfer	= i2c_max927x_xfer,
	.functionality	= i2c_max927x_func,
};


int max927x_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max927x_data *data = to_max927x_from_v4l2(sd);
	int ret;
	if(!data->operational)
		return -ENODEV;

	/*TODO can't turn off once it has been turned on(?)*/
	if(!enable)
		return 0;

	mutex_lock(&data->data_lock);
	ret = _max927x_video_enable(data, enable);
	if(ret < 0) {
		dev_err(data->dev,"Unable to turn %s video\n", enable ? "on" : "off");
		goto out;
	}
	ret = _max9272_link_locked(data);

	if(!enable) {
		if(ret == 0) {
			dev_err(data->dev, "Video link did not drop\n");
			ret = -EIO;
		}
	} else {
		if(ret < 0) {
			dev_err(data->dev, "Video link did not start\n");
		}
	}
out:
	mutex_unlock(&data->data_lock);
	return ret;
}

static void _max927x_subdev_init(struct max927x_data* data);
static int _max927x_probe(struct max927x_data* data)
{
	struct device *dev = data->dev;
	struct i2c_client *client = data->master;
	int ret;
	struct gpio_chip *gc;
	int i2c_nr = 0;
	struct device_node	*np = data->dev->of_node;
	struct device_node *child;
	int remote_reg;

	if(data->operational)
		return 0;

	dev_dbg(dev, "%s: client (master)=%p", __func__, client);
	if (!client)
		return -EPROBE_DEFER;
	dev_dbg(dev, "%s: adapter=%p", __func__, client->dev.parent);
	if (!client->dev.parent)
		return -EPROBE_DEFER;

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if(ret<0) {
		dev_err(dev,"Cannot create sysfs group\n");
		return ret;
	}

	of_property_read_u32(np,"i2c-nr", &i2c_nr);
	data->parent = to_i2c_adapter(client->dev.parent);
	snprintf(data->adap.name, sizeof(data->adap.name),
		 "max927x-%d", i2c_adapter_id(data->parent));
	data->adap.owner = THIS_MODULE;
	data->adap.algo = &i2c_max927x_algo;
	data->adap.algo_data = data;
	data->adap.dev.parent = &data->parent->dev;
	data->adap.class = 0;
	for_each_child_of_node(np, child) {
		ret = of_property_read_bool(child, "i2c");
		if(ret) {
			data->adap.dev.of_node = child;
			break;
		}
	}

	data->adap.nr = i2c_nr;

	memset(data->i2c_retry_counts, 0, sizeof(data->i2c_retry_counts));
	ret = i2c_add_numbered_adapter(&data->adap);
	if(ret < 0) {
		ret = -ENODEV;
		dev_err(dev, "Failed to register i2c adapter\n");
		goto error_sysfs;
	}

	ret = of_property_read_u32(np,"remote-reg",&remote_reg);
	if(ret < 0) {
		dev_err(dev,"Missing remote-reg property\n");
		ret = -EINVAL;
		goto error_i2c_adap;
	}
	data->slave = i2c_new_dummy(&data->adap, remote_reg);
	if(!data->slave) {
		dev_err(dev, "Could not create slave i2c client\n");
		ret = -EINVAL;
		goto error_i2c_adap;
	}

	/*Take GPIO lock while it is unsafe to use gpio.*/
	mutex_lock(&data->gpio_lock);
	ret = device_reset(dev);
	if (ret == -ENODEV) {
		mutex_unlock(&data->gpio_lock);
		ret = -EPROBE_DEFER;
		goto error_i2c_adap;
	}
	/*power up time is 6ms, double*/
	msleep(12);

	ret = _max927x_link_configure(data);
	mutex_unlock(&data->gpio_lock);

	if (ret < 0) {
		dev_err(dev, "Could not initialize max927x link\n");
		goto error_i2c_adap;
	}


	if(!data->gpio_chip_initialized) {
		gc = &data->gpio_chip;
		gc->direction_output = max9721_gpio_direction_output;
		gc->direction_input = max9721_gpio_direction_input;
		gc->get = max9271_gpio_get_value;
		gc->set = max9271_gpio_set_value;
		gc->can_sleep = 1;
		gc->ngpio = 8;
		gc->label = client->name;
		gc->dev = &client->dev;
		gc->owner = THIS_MODULE;
		gc->base = -1;
		ret = gpiochip_add(gc);
		if(ret) {
			dev_err(dev,"Failed to add gpio\n");
			goto error_i2c_adap;
		}
		data->gpio_chip_initialized = true;
	}

	of_i2c_register_devices(&data->adap);
	dev_err(dev,"initialization complete\n");
	data->error_count = 0;
	data->operational = true;
	_max927x_subdev_init(data);
	return 0;

error_i2c_adap:
	i2c_del_adapter(&data->adap);
	memset(&data->adap, 0, sizeof(data->adap));

error_sysfs:
	sysfs_remove_group(&dev->kobj, &attr_group);
	return ret;
}

static void _max927x_subdev_clear(struct max927x_data* data);
static int _max927x_remove(struct max927x_data* data)
{
	int ret = 0;

	dev_dbg(data->dev, "in _max927x_remove");

	_max927x_subdev_clear(data);

	data->operational = false;
	i2c_del_adapter(&data->adap);
	memset(&data->adap, 0, sizeof(data->adap));

	sysfs_remove_group(&data->dev->kobj, &attr_group);
	return ret;
}

static ssize_t max927x_operational_store(struct device *dev,
                                  struct device_attribute *attr, const char *buf, int count)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	unsigned int val;

	if(kstrtouint(buf, 10, &val) || val > 1) {
		dev_err(dev,"Must supply value between 0-1.\n");
		return count;
	}

	mutex_lock(&data->data_lock);
	if(!!val == data->operational)
		goto out;

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
		goto out;
	}

	if(data->operational)
		_max927x_remove(data);

	if(val)
		_max927x_probe(data);

out:
	mutex_unlock(&data->data_lock);
	return count;
}

static ssize_t max927x_operational_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct max927x_data* data = to_max927x_from_dev(dev);
	return sprintf(buf, "%d\n",data->operational);
}
static DEVICE_ATTR(operational, 0666, (void *)max927x_operational_show, (void *)max927x_operational_store);

static ssize_t max927x_error_count_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
       struct max927x_data* data = to_max927x_from_dev(dev);
       return sprintf(buf, "%d\n",data->error_count);
}
static DEVICE_ATTR(i2c_error_count, 0444, (void *)max927x_error_count_show, NULL);

static struct attribute *init_attributes[] = {
	&dev_attr_operational.attr,
	&dev_attr_i2c_error_count.attr,
	&dev_attr_magic_rev_trf.attr,
	&dev_attr_magic_rev_amp.attr,
	&dev_attr_magic_rev_dig_flt.attr,
	&dev_attr_magic_rev_logain.attr,
	&dev_attr_magic_rev_higain.attr,
	&dev_attr_magic_rev_hibw.attr,
	&dev_attr_magic_rev_hivth.attr,
	&dev_attr_i2c_slvsh.attr,
	&dev_attr_i2c_mstbt.attr,
	&dev_attr_cmllvl.attr,
	&dev_attr_preemp.attr,
	&dev_attr_spread.attr,
	&dev_attr_max9272_i2c_test.attr,
	&dev_attr_max9272_regs.attr,
	NULL,
};

static const struct attribute_group init_attr_group = {
	.attrs	= init_attributes,
};

static const struct v4l2_subdev_video_ops max927x_subdev_video_ops = {
	.s_stream = max927x_video_s_stream,
};

static const struct v4l2_subdev_ops max927x_subdev_ops = {
	.video	= &max927x_subdev_video_ops,
};


static void _max927x_subdev_init(struct max927x_data* data)
{
	struct i2c_client *client = data->master;
	struct v4l2_subdev *subdev = &data->subdev;
	snprintf(subdev->name, sizeof(subdev->name), "%s %d-%04x",
		client->driver->driver.name, i2c_adapter_id(client->adapter),
		client->addr);
	subdev->name[sizeof(subdev->name)-1] = 0;
}

static void _max927x_subdev_clear(struct max927x_data* data)
{
	struct v4l2_subdev *subdev = &data->subdev;
	subdev->name[0] = 0;
}

static int max927x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_driver *drv = dev->driver;
	struct device_node	*np = dev->of_node;
	struct max927x_data* data;
	bool deserializer_master;
	u8 client_id;
	struct v4l2_subdev *subdev;
	int ret;

	dev_dbg(dev, "max927x probe, drv=%p", drv);

	if (!drv)
		return -EPROBE_DEFER;

	ret = device_reset(dev);
	if (ret == -ENODEV)
		return -EPROBE_DEFER;

	if (ret < 0)
		dev_dbg(dev, "device_reset returned %d", ret);

	msleep(5);

	data =  devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;


	ret = of_property_read_u32(np,"slave-on-delay-ms", &data->slave_on_ms);
	if(ret < 0) {
		dev_err(dev,"Missing %s\n","slave-on-delay-ms");
		return ret;
	}

	ret = of_property_read_u32(np,"slave-off-delay-ms", &data->slave_off_ms);
	if(ret < 0) {
		dev_err(dev,"Missing %s\n","slave-off-delay-ms");
		return ret;
	}

	data->slave_supply = devm_regulator_get(dev, "slave");
	if(!data->slave_supply) {
		dev_err(dev,"Missing slave supply\n");
		return ret;
	}

	ret = _max927x_of_get_config(data,np,0);
	if(ret < 0) {
		dev_err(dev,"Missing or incorrect cfg-0 property\n");
		return ret;
	}

	data->master = client;
	data->dev = dev;

	mutex_init(&data->data_lock);
	mutex_init(&data->gpio_lock);

	ret = _max927x_read_reg(client, 0x1e, &client_id, NULL);
	if(ret < 0) {
		dev_err(dev,"Missing client\n");
		return ret;
	}
	switch(client_id) {
	case 0x09:
		deserializer_master = false;
		break;
	case 0x0A:
		deserializer_master = true;
		break;
	default:
		dev_err(dev,"Unknown client id 0x%02x", (unsigned)client_id);
		return -ENODEV;
	};

	data->deserializer_master = deserializer_master;

	ret = _max927x_slave_power_on(data);
	if(ret < 0)
		return ret;
	/*Not using v4l2_i2c_subdev_init because we don't want i2c devices to be
	 * automatically removed
	 */
	subdev = &data->subdev;
	v4l2_subdev_init(subdev, &max927x_subdev_ops);
	subdev->owner = to_i2c_driver(drv)->driver.owner;
	v4l2_set_subdevdata(subdev, client);
	i2c_set_clientdata(client, subdev);

	ret = sysfs_create_group(&dev->kobj, &init_attr_group);
	if(ret < 0) {
	   dev_err(dev,"Could not create sysfs file.\n");
	   return ret;
   }

	return 0;
}

static int max927x_remove(struct i2c_client *client)
{
	int ret = 0;
	struct max927x_data* data = to_max927x_from_i2c(client);

	dev_dbg(data->dev, "in max927x_remove");

	if(data->subdev.v4l2_dev != NULL) {
		dev_err(data->dev,"v4l2 subdev still in use; please shut down %s.\n",
				data->subdev.v4l2_dev->name);
		return -EBUSY;
	}

	if(data->operational) {
		if (data->gpio_chip_initialized) {
			dev_dbg(data->dev, "calling gpiochip_remove");
			ret = gpiochip_remove(&data->gpio_chip);
			if(ret < 0) {
				dev_err(data->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
				return ret;
			}
			memset(&data->gpio_chip, 0, sizeof(data->gpio_chip));
			data->gpio_chip_initialized = false;
		}
		ret = _max927x_remove(data);
	}
	if(ret < 0)
		return ret;

	_max927x_slave_power_off(data);
	sysfs_remove_group(&data->dev->kobj, &init_attr_group);
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
MODULE_DESCRIPTION("max927x Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
