/*
 * I2C multiplexer
 *
 * Copyright (c) 2008-2009 Rodolfo Giometti <giometti@linux.it>
 * Copyright (c) 2008-2009 Eurotech S.p.A. <info@eurotech.it>
 *
 * This module supports the PCA954x series of I2C multiplexer/switch chips
 * made by Philips Semiconductors.
 * This includes the:
 *	 PCA9540, PCA9542, PCA9543, PCA9544, PCA9545, PCA9546, PCA9547
 *	 and PCA9548.
 *
 * These chips are all controlled via the I2C bus itself, and all have a
 * single 8-bit register. The upstream "parent" bus fans out to two,
 * four, or eight downstream busses or channels; which of these
 * are selected is determined by the chip type and register contents. A
 * mux can select only one sub-bus at a time; a switch can select any
 * combination simultaneously.
 *
 * Based on:
 *	pca954x.c from Kumar Gala <galak@kernel.crashing.org>
 * Copyright (C) 2006
 *
 * Based on:
 *	pca954x.c from Ken Harrenstien
 * Copyright (C) 2004 Google, Inc. (Ken Harrenstien)
 *
 * Based on:
 *	i2c-virtual_cb.c from Brian Kuschak <bkuschak@yahoo.com>
 * and
 *	pca9540.c from Jean Delvare <jdelvare@suse.de>.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/i2c/pca954x.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#define PCA954X_MAX_NCHANS 8

enum pca_type {
	pca_9540,
	pca_9542,
	pca_9543,
	pca_9544,
	pca_9545,
	pca_9546,
	pca_9547,
	pca_9548,
};

struct pca954x {
	enum pca_type type;
	struct i2c_adapter *virt_adaps[PCA954X_MAX_NCHANS];
	struct mutex transaction_lock;
	u8 last_chan;		/* last register value */
};

struct chip_desc {
	u8 nchans;
	u8 enable;	/* used for muxes only */
	enum muxtype {
		pca954x_ismux = 0,
		pca954x_isswi
	} muxtype;
	enum pca_type type;
};

/* Provide specs for the PCA954x types we know about */
static const struct chip_desc chips[] = {
	[pca_9540] = {
		.nchans = 2,
		.enable = 0x4,
		.muxtype = pca954x_ismux,
		.type = pca_9540,
	},
	[pca_9543] = {
		.nchans = 2,
		.muxtype = pca954x_isswi,
		.type = pca_9543,
	},
	[pca_9544] = {
		.nchans = 4,
		.enable = 0x4,
		.muxtype = pca954x_ismux,
		.type = pca_9544,
	},
	[pca_9545] = {
		.nchans = 4,
		.muxtype = pca954x_isswi,
		.type = pca_9545,
	},
	[pca_9547] = {
		.nchans = 8,
		.enable = 0x8,
		.muxtype = pca954x_ismux,
		.type = pca_9547,
	},
	[pca_9548] = {
		.nchans = 8,
		.muxtype = pca954x_isswi,
		.type = pca_9548,
	},
};

#ifdef CONFIG_OF
static const struct of_device_id pca954x_of_match[] = {
	{ .compatible = "nxp,pca9540", &chips[pca_9540] },
	{ .compatible = "nxp,pca9542", &chips[pca_9542] },
	{ .compatible = "nxp,pca9543", &chips[pca_9543] },
	{ .compatible = "nxp,pca9544", &chips[pca_9544] },
	{ .compatible = "nxp,pca9545", &chips[pca_9545] },
	{ .compatible = "nxp,pca9546", &chips[pca_9545] },
	{ .compatible = "nxp,pca9547", &chips[pca_9547] },
	{ .compatible = "nxp,pca9548", &chips[pca_9547] },
	{ },
};
MODULE_DEVICE_TABLE(of, pca954x_of_match);
#endif

static const struct i2c_device_id pca954x_id[] = {
	{ "pca9540", pca_9540 },
	{ "pca9542", pca_9540 },
	{ "pca9543", pca_9543 },
	{ "pca9544", pca_9544 },
	{ "pca9545", pca_9545 },
	{ "pca9546", pca_9545 },
	{ "pca9547", pca_9547 },
	{ "pca9548", pca_9548 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca954x_id);

static int pca954x_recover(struct i2c_adapter *adap);

static struct i2c_bus_recovery_info pca954x_recovery_info = {
	.recover_bus = pca954x_recover,
};

/* Write to mux register. Don't use i2c_transfer()/i2c_smbus_xfer()
   for this as they will try to lock adapter a second time */
static int pca954x_reg_write(struct i2c_adapter *adap,
			     struct i2c_client *client, u8 val)
{
	int ret = -ENODEV;

	if (adap->algo->master_xfer) {
		struct i2c_msg msg;
		char buf[1];

		msg.addr = client->addr;
		msg.flags = 0;
		msg.len = 1;
		buf[0] = val;
		msg.buf = buf;
		ret = adap->algo->master_xfer(adap, &msg, 1);
	} else {
		union i2c_smbus_data data;
		ret = adap->algo->smbus_xfer(adap, client->addr,
					     client->flags,
					     I2C_SMBUS_WRITE,
					     val, I2C_SMBUS_BYTE, &data);
	}

	return ret;
}

static int pca954x_select_chan(struct i2c_adapter *adap,
			       void *client, u32 chan)
{
	struct pca954x *data = i2c_get_clientdata(client);
	const struct chip_desc *chip = &chips[data->type];
	u8 regval;
	int ret = 0;

	mutex_lock(&data->transaction_lock);

	/* we make switches look like muxes, not sure how to be smarter */
	if (chip->muxtype == pca954x_ismux)
		regval = chan | chip->enable;
	else
		regval = 1 << chan;

	/* Only select the channel if its different from the last channel */
	if (data->last_chan != regval) {
		ret = pca954x_reg_write(adap, client, regval);
		data->last_chan = regval;
	}

	return ret;
}

static int pca954x_unlock_mux(struct i2c_adapter *adap,
			      void *client, u32 chan)
{
	struct pca954x *data = i2c_get_clientdata(client);
	mutex_unlock(&data->transaction_lock);
	return 0;
}

static int pca954x_deselect_mux(struct i2c_adapter *adap,
				void *client, u32 chan)
{
	struct pca954x *data = i2c_get_clientdata(client);

	pca954x_unlock_mux(adap, client, chan);
	/* Deselect active channel */
	data->last_chan = 0;
	return pca954x_reg_write(adap, client, data->last_chan);
}

/*
 * I2C init/probing/exit functions
 */

#ifdef CONFIG_OF
static int pca954x_probe_dt(struct i2c_client *client, struct pca954x_platform_data **pdatap)
{
	struct pca954x *data = i2c_get_clientdata(client);
	struct pca954x_platform_data *pdata;
	struct device_node *node = client->dev.of_node;
	struct device_node *child;
	const struct of_device_id *matched_id;
	const struct chip_desc *chip;

	if (node == NULL)
		return -ENODEV;

	matched_id = of_match_node(pca954x_of_match, node);
	if (matched_id == NULL)
		return -ENODEV;
	chip = matched_id->data;
	pdata = devm_kzalloc(&client->dev, (sizeof(struct pca954x_platform_data) + 
					    chip->nchans * sizeof(struct pca954x_platform_mode)),
			     GFP_KERNEL);
	if (pdata == NULL)
		return -ENOMEM;
	pdata->modes = (struct pca954x_platform_mode *)(pdata + 1);

	for_each_available_child_of_node(node, child) {
		u32 chan, adap_id;
		if (!of_device_is_compatible(child, "nxp,pca954x-bus"))
			continue;
		if (of_property_read_u32(child, "reg", &chan) != 0)
			continue;
		if (chan >= chip->nchans) {
			dev_err(&client->dev, "invalid mux channel %u at %s\n",
				chan, of_node_full_name(child));
			continue;
		}
		if (chan >= pdata->num_modes)
			pdata->num_modes = chan + 1;
		if (of_property_read_u32(child, "nr", &adap_id) == 0)
			pdata->modes[chan].adap_id = adap_id;
		pdata->modes[chan].deselect_on_exit =
			of_property_read_bool(child, "nxp,deselect-on-exit");
		dev_dbg(&client->dev, "DT: found reg=%d, nr=%d, deselect=%s\n", chan, adap_id,
			(pdata->modes[chan].deselect_on_exit ? "yes" : "no"));
	}

	if (pdata->num_modes == 0) {
		dev_err(&client->dev, "no busses found\n");
		devm_kfree(&client->dev, pdata);
		return -ENODEV;
	}

	*pdatap = pdata;
	data->type = chip->type;

	return 0;
}
#endif

static int pca954x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	struct pca954x_platform_data *pdata = dev_get_platdata(&client->dev);
	struct device_node *np = client->dev.of_node;
	int num, force, class;
	struct pca954x *data;
	int ret;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(struct pca954x), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

#ifdef CONFIG_OF
	ret = pca954x_probe_dt(client, &pdata);
	if (ret < 0) {
		dev_err(&client->dev, "DT parsing error\n");
		return ret;
	}
#endif
	if (IS_ENABLED(CONFIG_OF) && np) {
		enum of_gpio_flags flags;
		int gpio;

		/* Get the mux out of reset if a reset GPIO is specified. */
		gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, &flags);
		if (gpio_is_valid(gpio)) {
			ret = devm_gpio_request_one(&client->dev, gpio,
					flags & OF_GPIO_ACTIVE_LOW ?
					GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
					"pca954x reset");
			if (ret < 0)
				return ret;
		}
	}

	/* Write the mux register at addr to verify
	 * that the mux is in fact present. This also
	 * initializes the mux to disconnected state.
	 */
	if (i2c_smbus_write_byte(client, 0) < 0) {
		dev_warn(&client->dev, "probe failed\n");
		return -ENODEV;
	}

#ifndef CONFIG_OF
	data->type = id->driver_data;
#endif
	mutex_init(&data->transaction_lock);
	data->last_chan = 0;		   /* force the first selection */

	/* Now create an adapter for each channel */
	for (num = 0; num < chips[data->type].nchans; num++) {
		force = 0;			  /* dynamic adap number */
		class = 0;			  /* no class by default */
		if (pdata) {
			if (num < pdata->num_modes) {
				/* force static number */
				force = pdata->modes[num].adap_id;
				class = pdata->modes[num].class;
			} else
				/* discard unconfigured channels */
				break;
		}

		data->virt_adaps[num] =
			i2c_add_mux_adapter(adap, &client->dev, client,
				force, num, class, pca954x_select_chan,
				(pdata && pdata->modes[num].deselect_on_exit)
					? pca954x_deselect_mux : pca954x_unlock_mux);

		if (data->virt_adaps[num] == NULL) {
			ret = -ENODEV;
			dev_err(&client->dev,
				"failed to register multiplexed adapter"
				" %d as bus %d\n", num, force);
			goto virt_reg_failed;
		}
		data->virt_adaps[num]->bus_recovery_info = &pca954x_recovery_info;
	}

	dev_info(&client->dev,
		 "registered %d multiplexed busses for I2C %s %s\n",
		 num, chips[data->type].muxtype == pca954x_ismux
				? "mux" : "switch", client->name);

	return 0;

virt_reg_failed:
	for (num--; num >= 0; num--)
		i2c_del_mux_adapter(data->virt_adaps[num]);
	return ret;
}

static int pca954x_remove(struct i2c_client *client)
{
	struct pca954x *data = i2c_get_clientdata(client);
	const struct chip_desc *chip = &chips[data->type];
	int i;

	for (i = 0; i < chip->nchans; ++i)
		if (data->virt_adaps[i]) {
			i2c_del_mux_adapter(data->virt_adaps[i]);
			data->virt_adaps[i] = NULL;
		}

	return 0;
}

static int pca954x_recover(struct i2c_adapter *adap)
{
	struct i2c_client *client = i2c_mux_pdata(adap);
	struct pca954x *data = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s: resetting last channel\n", __func__);
	data->last_chan = 0;
	return i2c_smbus_write_byte(client, 0);
}

static struct i2c_driver pca954x_driver = {
	.driver		= {
		.name	= "pca954x",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pca954x_of_match,
#endif
	},
	.probe		= pca954x_probe,
	.remove		= pca954x_remove,
	.id_table	= pca954x_id,
};

module_i2c_driver(pca954x_driver);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("PCA954x I2C mux/switch driver");
MODULE_LICENSE("GPL v2");
