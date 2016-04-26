/*
 * adv761x Analog Devices ADV761X HDMI receiver driver
 *
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ADV761X_H_
#define _ADV761X_H_

struct adv761x_platform_data {
	/* INT1 GPIO IRQ */
	int gpio;

	/* I2C addresses: 0 == use default */
	u8 i2c_cec;
	u8 i2c_inf;
	u8 i2c_dpll;
	u8 i2c_rep;
	u8 i2c_edid;
	u8 i2c_hdmi;
	u8 i2c_cp;
};

/* Notify events */
#define ADV761X_HOTPLUG		1
#define ADV761X_FMT_CHANGE	2

#endif	/* _ADV761X_H_ */
