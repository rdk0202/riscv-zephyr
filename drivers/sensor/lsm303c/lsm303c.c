/* lsm303c.c - Driver for LSM303C accelerometer, magnetometer, and temperature
 * sensor
 */

/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <misc/byteorder.h>
#include <misc/__assert.h>

#include "lsm303c.h"

static inline int lsm303c_reboot(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	/* Write the accelerometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG6_A,
				LSM303C_REG_CTRL_REG6_A_BOOT,
				LSM303C_REG_CTRL_REG6_A_BOOT) < 0) {
		return -EIO;
	}

	/* Poll the accelerometer reboot bit to detect reboot completes */
	int rc = 0;
	u8_t boot_reg;
	while(1) {
		rc = i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG6_A, &boot_reg);
		if(rc < 0) {
			SYS_LOG_DBG("I2C failed to read 0x%02X@0x%02X",
					LSM303C_REG_CTRL_REG6_A, config->i2c_slave_addr);
		}

		if((boot_reg & LSM303C_REG_CTRL_REG6_A_BOOT) == 0) {
			break;
		}
	}

	return 0;
}

static inline int lsm303c_accel_axis_ctrl(struct device *dev, int x_en,
					  int y_en, int z_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t state = 0;
	if(x_en)
		state |= LSM303C_REG_CTRL_REG1_A_XEN;
	if(y_en)
		state |= LSM303C_REG_CTRL_REG1_A_YEN;
	if(z_en)
		state |= LSM303C_REG_CTRL_REG1_A_ZEN;

	return i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				   LSM303C_REG_CTRL_REG1_A,
				   LSM303C_REG_CTRL_REG1_A_XEN |
				   LSM303C_REG_CTRL_REG1_A_YEN |
				   LSM303C_REG_CTRL_REG1_A_ZEN,
				   state);
}

static int lsm303c_accel_set_fs_raw(struct device *dev, u8_t fs)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG4_A,
				LSM303C_REG_CTRL_REG4_A_FS_MASK,
				fs << LSM303C_REG_CTRL_REG4_A_FS_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_accel_set_odr_raw(struct device *dev, u8_t odr)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG1_A,
				LSM303C_REG_CTRL_REG1_A_ODR_MASK,
				odr << LSM303C_REG_CTRL_REG1_A_ODR_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static inline int lsm303c_magn_axis_ctrl(struct device *dev, int x_en, int y_en,
					 int z_en)
{
	/* TODO: Can these be enabled/disabled? */
	return 0;
}

static int lsm303c_magn_set_fs_raw(struct device *dev, u8_t fs)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG2_M,
				LSM303C_REG_CTRL_REG2_M_FS_MASK,
				fs << LSM303C_REG_CTRL_REG2_M_FS_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_magn_set_odr_raw(struct device *dev, u8_t odr)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_DO_MASK,
				odr << LSM303C_REG_CTRL_REG1_M_DO_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_sample_fetch_accel(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t buf[6];
	u8_t addr = LSM303C_REG_OUT_X_L_A;

	for(int i = 0; i < 6; i++) {
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				   addr + i, buf + i) < 0) {
			SYS_LOG_DBG("failed to read sample");
			return -EIO;
		}
	}

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
	data->accel_sample_x = (s16_t)((u16_t)(buf[0]) |
				((u16_t)(buf[1]) << 8));
	SYS_LOG_DBG("Fetched accel sample x: %d", data->accel_sample_x);
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
	data->accel_sample_y = (s16_t)((u16_t)(buf[2]) |
				((u16_t)(buf[3]) << 8));
	SYS_LOG_DBG("Fetched accel sample y: %d", data->accel_sample_y);
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
	data->accel_sample_z = (s16_t)((u16_t)(buf[4]) |
				((u16_t)(buf[5]) << 8));
	SYS_LOG_DBG("Fetched accel sample z: %d", data->accel_sample_z);
#endif

	return 0;
}

static int lsm303c_sample_fetch_magn(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t buf[6];

	if (i2c_burst_read(data->i2c_master, config->i2c_m_slave_addr,
			   LSM303C_REG_OUT_X_L_M, buf, sizeof(buf)) < 0) {
		SYS_LOG_DBG("failed to read sample");
		return -EIO;
	}

#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
	data->magn_sample_x = (s16_t)((u16_t)(buf[0]) |
				((u16_t)(buf[1]) << 8));
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
	data->magn_sample_y = (s16_t)((u16_t)(buf[2]) |
				((u16_t)(buf[3]) << 8));
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
	data->magn_sample_z = (s16_t)((u16_t)(buf[4]) |
				((u16_t)(buf[5]) << 8));
#endif

	return 0;
}

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
static int lsm303c_sample_fetch_temp(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t buf[2];

	if (i2c_burst_read(data->i2c_master, config->i2c_slave_addr,
			   LSM303C_REG_OUT_TEMP_L, buf, sizeof(buf)) < 0) {
		SYS_LOG_DBG("failed to read sample");
		return -EIO;
	}

	data->temp_sample = (s16_t)((u16_t)(buf[0]) |
				((u16_t)(buf[1]) << 8));

	return 0;
}
#endif

static int lsm303c_sample_fetch(struct device *dev, enum sensor_channel chan)
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

#ifdef CONFIG_FLOAT
static inline void lsm303c_accel_convert(struct sensor_value *val, int raw_val,
					 float scale)
{
	double dval;

	dval = (double)(raw_val) * scale / 32767.0;
	val->val1 = (s32_t)dval;
	val->val2 = ((s32_t)(dval * 1000000)) % 1000000;
}
#else
static inline void lsm303c_accel_convert(struct sensor_value *val, int raw_val,
					 s32_t scale)
{
	long int dval;

	dval = (long int)(raw_val) * scale / 32767;
	val->val1 = (s32_t)dval;
	val->val2 = ((s32_t)(dval * 1000000)) % 1000000;
}
#endif

#ifdef CONFIG_FLOAT
static inline int lsm303c_accel_get_channel(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct lsm303c_data *data,
					    float scale)
#else
static inline int lsm303c_accel_get_channel(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct lsm303c_data *data,
					    s32_t scale)
#endif
{
	switch (chan) {
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
	case SENSOR_CHAN_ACCEL_X:
		lsm303c_accel_convert(val, data->accel_sample_x, scale);
		break;
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
	case SENSOR_CHAN_ACCEL_Y:
		lsm303c_accel_convert(val, data->accel_sample_y, scale);
		break;
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
	case SENSOR_CHAN_ACCEL_Z:
		lsm303c_accel_convert(val, data->accel_sample_z, scale);
		break;
#endif
	case SENSOR_CHAN_ACCEL_XYZ:
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
		lsm303c_accel_convert(val, data->accel_sample_x, scale);
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
		lsm303c_accel_convert(val + 1, data->accel_sample_y, scale);
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
		lsm303c_accel_convert(val + 2, data->accel_sample_z, scale);
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm303c_accel_channel_get(enum sensor_channel chan,
				     struct sensor_value *val,
				     struct lsm303c_data *data)
{
	return lsm303c_accel_get_channel(chan, val, data,
					LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR);
}

#ifdef CONFIG_FLOAT
static inline void lsm303c_magn_convert(struct sensor_value *val, int raw_val,
					float numerator)
{
	double dval;

	dval = (double)(raw_val) * numerator / 1000.0 * SENSOR_DEG2RAD_DOUBLE;
	val->val1 = (s32_t)dval;
	val->val2 = ((s32_t)(dval * 1000000)) % 1000000;
}
#else
static inline void lsm303c_magn_convert(struct sensor_value *val, int raw_val,
					s32_t numerator)
{
	long int dval;

	dval = (long int)(raw_val) * numerator / 1000 * SENSOR_DEG2RAD_INT;
	val->val1 = (s32_t)dval;
	val->val2 = ((s32_t)(dval * 1000000)) % 1000000;
}
#endif

#ifdef CONFIG_FLOAT
static inline int lsm303c_magn_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct lsm303c_data *data,
					   float numerator)
#else
static inline int lsm303c_magn_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct lsm303c_data *data,
					   s32_t numerator)
#endif
{
	switch (chan) {
#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
	case SENSOR_CHAN_MAGN_X:
		lsm303c_magn_convert(val, data->magn_sample_x, numerator);
		break;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
	case SENSOR_CHAN_MAGN_Y:
		lsm303c_magn_convert(val, data->magn_sample_y, numerator);
		break;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
	case SENSOR_CHAN_MAGN_Z:
		lsm303c_magn_convert(val, data->magn_sample_z, numerator);
		break;
#endif
	case SENSOR_CHAN_MAGN_XYZ:
#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
		lsm303c_magn_convert(val, data->magn_sample_x, numerator);
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
		lsm303c_magn_convert(val + 1, data->magn_sample_y, numerator);
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
		lsm303c_magn_convert(val + 2, data->magn_sample_z, numerator);
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm303c_magn_channel_get(enum sensor_channel chan,
				    struct sensor_value *val,
				    struct lsm303c_data *data)
{
	return lsm303c_magn_get_channel(chan, val, data,
					LSM303C_DEFAULT_MAGN_FULLSCALE_FACTOR);
}

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
static void lsm303c_magn_channel_get_temp(struct sensor_value *val,
					  struct lsm303c_data *data)
{
	/* val = temp_sample / 16 + 25 */
	val->val1 = data->temp_sample / 16 + 25;
	val->val2 = (data->temp_sample % 16) * (1000000 / 16);
}
#endif

static int lsm303c_channel_get(struct device *dev,
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

static const struct sensor_driver_api lsm303c_api_funcs = {
	.sample_fetch = lsm303c_sample_fetch,
	.channel_get = lsm303c_channel_get,
};

static int lsm303c_init_chip(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t chip_id;

	/* Reboot chip */
	if (lsm303c_reboot(dev) < 0) {
		SYS_LOG_DBG("failed to reboot device");
		return -EIO;
	}

	/* Verify chip identity */
	if (i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
			      LSM303C_REG_WHO_AM_I_A, &chip_id) < 0) {
		SYS_LOG_DBG("failed reading chip id");
		return -EIO;
	}
	if (chip_id != LSM303C_VAL_WHO_AM_I_A) {
		SYS_LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}
	SYS_LOG_DBG("chip id 0x%x", chip_id);

	/* Configure accelerometer axis enables */
	if (lsm303c_accel_axis_ctrl(dev, LSM303C_ACCEL_ENABLE_X_AXIS,
				    LSM303C_ACCEL_ENABLE_Y_AXIS,
				    LSM303C_ACCEL_ENABLE_Z_AXIS) < 0) {
		SYS_LOG_DBG("failed to set accelerometer axis");
		return -EIO;
	}

	/* Configure accelerometer full scale */
	if (lsm303c_accel_set_fs_raw(dev, LSM303C_DEFAULT_ACCEL_FULLSCALE)
				     < 0) {
		SYS_LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	/* Configure accelerometer sample rate */
	if (lsm303c_accel_set_odr_raw(dev, LSM303C_DEFAULT_ACCEL_SAMPLING_RATE)
				      < 0) {
		SYS_LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	if (lsm303c_magn_axis_ctrl(dev, LSM303C_MAGN_ENABLE_X_AXIS,
				   LSM303C_MAGN_ENABLE_Y_AXIS,
				   LSM303C_MAGN_ENABLE_Z_AXIS) < 0) {
		SYS_LOG_DBG("failed to set magnetometer axis");
		return -EIO;
	}

	if (lsm303c_magn_set_fs_raw(dev, LSM303C_DEFAULT_MAGN_FULLSCALE)
				    < 0) {
		SYS_LOG_DBG("failed to set magnetometer full-scale");
		return -EIO;
	}

	if (lsm303c_magn_set_odr_raw(dev, LSM303C_DEFAULT_MAGN_SAMPLING_RATE)
				     < 0) {
		SYS_LOG_DBG("failed to set magnetometer sampling rate");
		return -EIO;
	}

	/* Set accelerometer block data update */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG1_A,
				LSM303C_REG_CTRL_REG1_A_BDU,
				LSM303C_REG_CTRL_REG1_A_BDU) < 0) {
		SYS_LOG_DBG("failed to set accel BDU");
		return -EIO;
	}

	/* Set accelerometer bust read mode */
	/*if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG4_A,
				LSM303C_REG_CTRL_REG4_A_IF_ADD_INC,
				LSM303C_REG_CTRL_REG4_A_IF_ADD_INC) < 0) {
		SYS_LOG_DBG("failed to set accel burst");
		return -EIO;
	}*/

	return 0;
}

static int lsm303c_init(struct device *dev)
{
	const struct lsm303c_config * const config = dev->config->config_info;
	struct lsm303c_data *data = dev->driver_data;

	data->i2c_master = device_get_binding(config->i2c_master_dev_name);
	if (!data->i2c_master) {
		SYS_LOG_DBG("i2c master not found: %s",
			    config->i2c_master_dev_name);
		return -EINVAL;
	}

	/* TODO: When and where should the I2C bus get initialized? */
	u32_t dev_config = (I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if(i2c_configure(data->i2c_master, dev_config) != 0) {
		SYS_LOG_DBG("i2c config failed\n");
		return -EIO;
	}

	if (lsm303c_init_chip(dev) < 0) {
		SYS_LOG_DBG("failed to initialize chip");
		return -EIO;
	}

	return 0;
}

static const struct lsm303c_config lsm303c_config = {
	.i2c_master_dev_name = CONFIG_LSM303C_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_LSM303C_I2C_ADDR,
	.i2c_m_slave_addr = CONFIG_LSM303C_M_I2C_ADDR,
};

static struct lsm303c_data lsm303c_data;

DEVICE_AND_API_INIT(lsm303c, CONFIG_LSM303C_DEV_NAME, lsm303c_init,
		    &lsm303c_data, &lsm303c_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &lsm303c_api_funcs);
