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

LOG_MODULE_REGISTER(st_lsm303c);

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
	if (rc < 0) {
		LOG_DBG("Failed to reboot accelerometer");
		return -EIO;
	}

	rc = lsm303c_magn_reboot(dev);
	if (rc < 0) {
		LOG_DBG("Failed to reboot magnetometer");
		return -EIO;
	}

	/* Init Accelerometer */
	rc = lsm303c_accel_init(dev);
	if (rc < 0) {
		LOG_DBG("Failed to init accelerometer");
		return -EIO;
	}

	/* Init Magnetometer */
	rc = lsm303c_magn_init(dev);
	if (rc < 0) {
		LOG_DBG("Failed to init magnetometer");
		return -EIO;
	}

	return 0;
}

int lsm303c_init(struct device *dev)
{
	const struct lsm303c_config * const config = dev->config->config_info;
	struct lsm303c_data *data = dev->driver_data;
	u32_t dev_config = 0;

	data->i2c_master = device_get_binding(config->i2c_master_dev_name);
	if (!data->i2c_master) {
		LOG_DBG("I2C master not found: %s",
			config->i2c_master_dev_name);
		return -EINVAL;
	}

	if (lsm303c_init_chip(dev) < 0) {
		LOG_DBG("Failed to initialize chip");
		return -EIO;
	}

	/* Power-on self test */
#if CONFIG_LSM303C_SELFTEST
	if (lsm303c_selftest(dev) < 0) {
		LOG_DBG("Self-test failed");
		return -EIO;
	}
#endif

	return 0;
}

const struct lsm303c_config lsm303c_config = {
	.i2c_master_dev_name = DT_LSM303C_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = DT_LSM303C_I2C_ADDR,
	.i2c_m_slave_addr = CONFIG_LSM303C_M_I2C_ADDR,
};

struct lsm303c_data lsm303c_data;

DEVICE_AND_API_INIT(lsm303c, DT_LSM303C_DEV_NAME, lsm303c_init,
		    &lsm303c_data, &lsm303c_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &lsm303c_api_funcs);
