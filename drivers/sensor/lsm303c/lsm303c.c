/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <misc/byteorder.h>
#include <misc/__assert.h>

#include "lsm303c.h"

int lsm303c_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_ACCEL_XYZ ||
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
			chan == SENSOR_CHAN_DIE_TEMP ||
#endif
			chan == SENSOR_CHAN_MAGN_XYZ);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm303c_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		lsm303c_sample_fetch_magn(dev);
		break;
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		lsm303c_sample_fetch_temp(dev);
		break;
#endif
	case SENSOR_CHAN_ALL:
		lsm303c_sample_fetch_accel(dev);
		lsm303c_sample_fetch_magn(dev);
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
		lsm303c_sample_fetch_temp(dev);
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

int lsm303c_channel_get(struct device *dev,
	enum sensor_channel chan,
	struct sensor_value *val)
{
	struct lsm303c_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm303c_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		lsm303c_magn_channel_get(chan, val, data);
		break;
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		lsm303c_magn_channel_get_temp(val, data);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

const struct sensor_driver_api lsm303c_api_funcs = {
	.sample_fetch = lsm303c_sample_fetch,
	.channel_get = lsm303c_channel_get,
};

int lsm303c_init_chip(struct device *dev)
{
	int rc = 0;

	/* Reboot chip */
	rc = lsm303c_accel_reboot(dev);
	if(rc < 0) {
		SYS_LOG_DBG("Failed to reboot accelerometer");
		return -EIO;
	}

	rc = lsm303c_magn_reboot(dev);
	if(rc < 0) {
		SYS_LOG_DBG("Failed to reboot magnetometer");
		return -EIO;
	}

	/* Init Accelerometer */
	rc = lsm303c_accel_init(dev);
	if(rc < 0) {
		SYS_LOG_DBG("Failed to init accelerometer");
		return -EIO;
	}

	/* Init Magnetometer */
	rc = lsm303c_magn_init(dev);
	if(rc < 0) {
		SYS_LOG_DBG("Failed to init magnetometer");
		return -EIO;
	}

	return 0;
}

int lsm303c_init(struct device *dev)
{
	const struct lsm303c_config * const config = dev->config->config_info;
	struct lsm303c_data *data = dev->driver_data;

	data->i2c_master = device_get_binding(config->i2c_master_dev_name);
	if (!data->i2c_master) {
		SYS_LOG_DBG("I2C master not found: %s",
			    config->i2c_master_dev_name);
		return -EINVAL;
	}

	/* TODO: When and where should the I2C bus get initialized? */
	u32_t dev_config = (I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if(i2c_configure(data->i2c_master, dev_config) != 0) {
		SYS_LOG_DBG("I2C config failed\n");
		return -EIO;
	}

	if (lsm303c_init_chip(dev) < 0) {
		SYS_LOG_DBG("Failed to initialize chip");
		return -EIO;
	}

	/* TODO: Power-on self test */

	return 0;
}

const struct lsm303c_config lsm303c_config = {
	.i2c_master_dev_name = CONFIG_LSM303C_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_LSM303C_I2C_ADDR,
	.i2c_m_slave_addr = CONFIG_LSM303C_M_I2C_ADDR,
};

struct lsm303c_data lsm303c_data;

DEVICE_AND_API_INIT(lsm303c, CONFIG_LSM303C_DEV_NAME, lsm303c_init,
		    &lsm303c_data, &lsm303c_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &lsm303c_api_funcs);
