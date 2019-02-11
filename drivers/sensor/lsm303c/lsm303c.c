/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <sensor.h>
#include <i2c.h>
#include <init.h>

#include <zephyr/types.h>
#include <misc/byteorder.h>
#include <misc/__assert.h>
#include <misc/util.h>

#define LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/log.h>

#include "lsm303c.h"

LOG_MODULE_REGISTER(st_lsm303c);

static int lsm303c_accel_axis_ctrl(struct device *dev, int x_en, int y_en, int z_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t state = 0;

	if (x_en) {
		state |= LSM303C_REG_CTRL_REG1_A_XEN;
	}
	if (y_en) {
		state |= LSM303C_REG_CTRL_REG1_A_YEN;
	}
	if (z_en) {
		state |= LSM303C_REG_CTRL_REG1_A_ZEN;
	}

	return i2c_reg_update_byte(data->i2c_master,
				   config->i2c_slave_addr,
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

	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_slave_addr,
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

	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG1_A,
				LSM303C_REG_CTRL_REG1_A_ODR_MASK,
				odr << LSM303C_REG_CTRL_REG1_A_ODR_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_sample_fetch_accel(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t buf[6] = {0};
	u8_t addr = LSM303C_REG_OUT_X_L_A;

	if (i2c_burst_read(data->i2c_master,
			   config->i2c_slave_addr,
			   addr,
			   buf,
			   6) < 0) {
		LOG_DBG("Failed to read accelerometer sample");
		return -EIO;
	}

#if defined(CONFIG_LSM303C_ACCEL_ENABLE_X_AXIS)
	data->accel_sample_x = (s16_t)((u16_t)(buf[0]) |
				((u16_t)(buf[1]) << 8));
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Y_AXIS)
	data->accel_sample_y = (s16_t)((u16_t)(buf[2]) |
				((u16_t)(buf[3]) << 8));
#endif
#if defined(CONFIG_LSM303C_ACCEL_ENABLE_Z_AXIS)
	data->accel_sample_z = (s16_t)((u16_t)(buf[4]) |
				((u16_t)(buf[5]) << 8));
#endif

	return 0;
}

static void lsm303c_accel_convert(struct sensor_value *val,
			   s16_t raw_val,
			   s64_t scale)
{
	s64_t value_ug = 0;
	s64_t value_umgss = 0;
	/*
	 * The value is stored as a 16-bit signed integer with extrema of
	 * +/- the full-scale setting.
	 */

	/* Compute the value in micro-gravitys */
	value_ug = ((s64_t) raw_val) * scale * 1000000LL / 32767LL;

	/* Convert from micro-gravitys to micro-meters per square second */
	value_umgss = value_ug * SENSOR_G / 1000000LL;

	/* Divide by 1000000LL to get meters per square second */
	val->val1 = (s32_t) (value_umgss / 1000000LL);

	/* Modulo by 1000000LL to get the remainder */
	val->val2 = (s32_t) (value_umgss % 1000000LL);

	/* Compute the proper sign of the remainder */
	if (value_umgss < 0) {
		val->val2 *= -1;
	}
}

static int lsm303c_accel_get_channel(enum sensor_channel chan,
			      struct sensor_value *val,
			      struct lsm303c_data *data,
			      s64_t scale)
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
	return lsm303c_accel_get_channel(chan,
					 val,
					 data,
					 LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR);
}

static int lsm303c_accel_reboot(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	int rc = 0;
	u8_t boot_reg = 0;

	/* Write the accelerometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG6_A,
				LSM303C_REG_CTRL_REG6_A_BOOT,
				LSM303C_REG_CTRL_REG6_A_BOOT) < 0) {
		return -EIO;
	}

	/* Poll the accelerometer reboot bit to detect reboot completes */
	rc = 0;
	while (1) {
		rc = i2c_reg_read_byte(data->i2c_master,
				       config->i2c_slave_addr,
				       LSM303C_REG_CTRL_REG6_A, &boot_reg);
		if (rc < 0) {
			LOG_DBG("I2C failed to read 0x%02X@0x%02X",
				LSM303C_REG_CTRL_REG6_A,
				config->i2c_slave_addr);
		}

		if ((boot_reg & LSM303C_REG_CTRL_REG6_A_BOOT) == 0) {
			break;
		}
	}

	return 0;
}

static int lsm303c_accel_init(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t chip_id = 0;

	/* Verify accelerometer identity */
	if (i2c_reg_read_byte(data->i2c_master,
			      config->i2c_slave_addr,
			      LSM303C_REG_WHO_AM_I_A,
			      &chip_id) < 0) {
		LOG_DBG("Failed reading accelerometer ID");
		return -EIO;
	}
	if (chip_id != LSM303C_VAL_WHO_AM_I_A) {
		LOG_DBG("Invalid accelerometer ID 0x%x", chip_id);
		return -EIO;
	}
	LOG_DBG("Accelerometer ID 0x%x", chip_id);

	/* Configure accelerometer axis enables */
	if (lsm303c_accel_axis_ctrl(dev,
				    LSM303C_ACCEL_ENABLE_X_AXIS,
				    LSM303C_ACCEL_ENABLE_Y_AXIS,
				    LSM303C_ACCEL_ENABLE_Z_AXIS) < 0) {
		LOG_DBG("Failed to enable accelerometer axes");
		return -EIO;
	}

	/* Configure accelerometer full scale */
	if (lsm303c_accel_set_fs_raw(dev, LSM303C_DEFAULT_ACCEL_FULLSCALE)
				     < 0) {
		LOG_DBG("Failed to set accelerometer full-scale");
		return -EIO;
	}

	/* Configure accelerometer sample rate */
	if (lsm303c_accel_set_odr_raw(dev, LSM303C_DEFAULT_ACCEL_SAMPLING_RATE)
				      < 0) {
		LOG_DBG("Failed to set accelerometer sample rate");
		return -EIO;
	}

	/* Set accelerometer block data update */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG1_A,
				LSM303C_REG_CTRL_REG1_A_BDU,
				LSM303C_REG_CTRL_REG1_A_BDU) < 0) {
		LOG_DBG("Failed to set accelerometer block data update");
		return -EIO;
	}

	return 0;
}

static int lsm303c_magn_axis_ctrl(struct device *dev, int x_en, int y_en, int z_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	int rc = 0;
	u8_t om_state = 0;

	/* Enable the X & Y axes */
	if (x_en || y_en) {
		om_state = LSM303C_REG_CTRL_REG1_M_OM_ULTRAHIGH;
		om_state <<= LSM303C_REG_CTRL_REG1_M_OM_SHIFT;

		rc = i2c_reg_update_byte(data->i2c_master,
					 config->i2c_m_slave_addr,
					 LSM303C_REG_CTRL_REG1_M,
					 LSM303C_REG_CTRL_REG1_M_OM_MASK,
					 om_state);
		if (rc < 0) {
			LOG_DBG("Failed to enable x & y magn axes\n");
			return -EIO;
		}
	}

	/* Enable the Z axis */
	if (z_en) {
		om_state = LSM303C_REG_CTRL_REG4_M_OMZ_ULTRAHIGH;
		om_state <<= LSM303C_REG_CTRL_REG4_M_OMZ_SHIFT;

		rc = i2c_reg_update_byte(data->i2c_master,
					 config->i2c_m_slave_addr,
					 LSM303C_REG_CTRL_REG4_M,
					 LSM303C_REG_CTRL_REG4_M_OMZ_MASK,
					 om_state);
		if (rc < 0) {
			LOG_DBG("Failed to enable z magn axis\n");
			return -EIO;
		}
	}

	return 0;
}

static int lsm303c_magn_set_fs_raw(struct device *dev, u8_t fs)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
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
	u8_t md_state = 0;

	/* Set the device to continuous conversion mode */
	md_state = LSM303C_REG_CTRL_REG3_M_MD_CONTINUOUS;
	md_state <<= LSM303C_REG_CTRL_REG3_M_MD_SHIFT;

	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG3_M,
				LSM303C_REG_CTRL_REG3_M_MD_MASK,
				md_state) < 0) {
		return -EIO;
	}

	/* Set sample frequency */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_DO_MASK,
				odr << LSM303C_REG_CTRL_REG1_M_DO_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_sample_fetch_magn(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t buf[6] = {0};
	u8_t addr = LSM303C_REG_OUT_X_L_M;

	if (i2c_burst_read(data->i2c_master,
			   config->i2c_m_slave_addr,
			   addr,
			   buf,
			   6) < 0) {
		LOG_DBG("Failed to read magnetometer sample");
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
	u8_t buf[2] = {0};
	u8_t addr = LSM303C_REG_TEMP_L_M;

	if (i2c_burst_read(data->i2c_master,
			   config->i2c_m_slave_addr,
			   addr,
			   buf,
			   2) < 0) {
		LOG_DBG("Failed to read temperature sample");
		return -EIO;
	}

	data->temp_sample = (s16_t) ((((u16_t) buf[1]) << 8) |
				     ((u16_t) buf[0]));

	return 0;
}
#endif

static void lsm303c_magn_convert(struct sensor_value *val, int raw_val, s32_t scale)
{
	s64_t value_ugauss = 0;
	/*
	 * The value is stored as a 16-bit signed integer with extrema of +/-
	 * the full-scale setting.
	 */

	value_ugauss = ((s64_t) raw_val) * scale * 1000000LL / 32767LL;

	/* Divide by 1000000LL to get gauss */
	val->val1 = (s32_t) (value_ugauss / 1000000LL);

	/* Modulo by 1000000LL to get the remainder */
	val->val2 = (s32_t) (value_ugauss % 1000000LL);

	/* Compute the proper sign of the remainder */
	if (value_ugauss < 0) {
		val->val2 *= -1;
	}
}

static int lsm303c_magn_get_channel(enum sensor_channel chan,
			     struct sensor_value *val,
			     struct lsm303c_data *data,
			     s32_t scale)
{
	switch (chan) {
#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
	case SENSOR_CHAN_MAGN_X:
		lsm303c_magn_convert(val, data->magn_sample_x, scale);
		break;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
	case SENSOR_CHAN_MAGN_Y:
		lsm303c_magn_convert(val, data->magn_sample_y, scale);
		break;
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
	case SENSOR_CHAN_MAGN_Z:
		lsm303c_magn_convert(val, data->magn_sample_z, scale);
		break;
#endif
	case SENSOR_CHAN_MAGN_XYZ:
#if defined(CONFIG_LSM303C_MAGN_ENABLE_X_AXIS)
		lsm303c_magn_convert(val, data->magn_sample_x, scale);
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Y_AXIS)
		lsm303c_magn_convert(val + 1, data->magn_sample_y, scale);
#endif
#if defined(CONFIG_LSM303C_MAGN_ENABLE_Z_AXIS)
		lsm303c_magn_convert(val + 2, data->magn_sample_z, scale);
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
	return lsm303c_magn_get_channel(chan,
					val,
					data,
					LSM303C_DEFAULT_MAGN_FULLSCALE_FACTOR);
}

static int lsm303c_temp_enable(struct device *dev, int t_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t state = 0;

	if (t_en) {
		state |= LSM303C_REG_CTRL_REG1_M_TEMP_EN;
	}

	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_TEMP_EN,
				state) < 0) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
static void lsm303c_magn_channel_get_temp(struct sensor_value *val,
				   struct lsm303c_data *data)
{
	s32_t raw_value = 0;

	/*
	 * The raw value is stored in 8ths of a degree C with a zero point at
	 * -25 degrees C.
	 */

	/* Convert to 32-bit from the 16-bit stored sample */
	raw_value = (s32_t) data->temp_sample;

	/* Get the integer part by ofsetting by 8 * 25 and then dividing by 8 */
	val->val1 = (raw_value + 200) / 8;

	/*
	 * Get the fractional part in micro-degrees C (10e-6) by multiplying by
	 * 10e6 and then modulo 10e6
	 */
	val->val2 = ((1000000 * (raw_value + 200)) / 8) % 1000000;
}
#endif

static int lsm303c_magn_reboot(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	int rc = 0;
	u8_t boot_reg = 0;

	/* Write the magnetometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG2_M,
				LSM303C_REG_CTRL_REG2_M_REBOOT,
				LSM303C_REG_CTRL_REG2_M_REBOOT) < 0) {
		return -EIO;
	}

	/* Poll the magnetometer reboot bit to detect reboot completes */
	while (1) {
		rc = i2c_reg_read_byte(data->i2c_master,
				       config->i2c_m_slave_addr,
				       LSM303C_REG_CTRL_REG2_M,
				       &boot_reg);
		if (rc < 0) {
			LOG_DBG("I2C failed to read 0x%02X@0x%02X",
				LSM303C_REG_CTRL_REG2_M,
				config->i2c_m_slave_addr);
		}

		if ((boot_reg & LSM303C_REG_CTRL_REG2_M_REBOOT) == 0) {
			break;
		}
	}
	return 0;
}

static int lsm303c_magn_init(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t chip_id = 0;

	/* Verify magnetometer identity */
	if (i2c_reg_read_byte(data->i2c_master,
			      config->i2c_m_slave_addr,
			      LSM303C_REG_WHO_AM_I_M,
			      &chip_id) < 0) {
		LOG_DBG("Failed to read magnetometer ID");
		return -EIO;
	}
	if (chip_id != LSM303C_VAL_WHO_AM_I_M) {
		LOG_DBG("Invalid magnetometer ID 0x%x", chip_id);
		return -EIO;
	}
	LOG_DBG("Magnetometer ID 0x%x", chip_id);

	/* Enable magnetometer axes */
	if (lsm303c_magn_axis_ctrl(dev,
				   LSM303C_MAGN_ENABLE_X_AXIS,
				   LSM303C_MAGN_ENABLE_Y_AXIS,
				   LSM303C_MAGN_ENABLE_Z_AXIS) < 0) {
		LOG_DBG("Failed to enable magnetometer axes");
		return -EIO;
	}

	/* Set magnetometer fullscale */
	if (lsm303c_magn_set_fs_raw(dev, LSM303C_DEFAULT_MAGN_FULLSCALE)
				    < 0) {
		LOG_DBG("Failed to set magnetometer full-scale");
		return -EIO;
	}

	/* Set magnetometer sample rate */
	if (lsm303c_magn_set_odr_raw(dev, LSM303C_DEFAULT_MAGN_SAMPLING_RATE)
				     < 0) {
		LOG_DBG("Failed to set magnetometer sample rate");
		return -EIO;
	}

	/* Set magnetometer block data update */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG5_M,
				LSM303C_REG_CTRL_REG5_M_BDU,
				LSM303C_REG_CTRL_REG5_M_BDU) < 0) {
		LOG_DBG("Failed to set magnetometer block data update");
		return -EIO;
	}

	/* Enable thermometer channel */
	if (lsm303c_temp_enable(dev, LSM303C_ENABLE_TEMP) < 0) {
		LOG_DBG("Failed to enable temperature");
		return -EIO;
	}

	return 0;
}

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

static int lsm303c_init_chip(struct device *dev)
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

static const struct sensor_driver_api lsm303c_api_funcs = {
	.sample_fetch = lsm303c_sample_fetch,
	.channel_get = lsm303c_channel_get,
};

static int lsm303c_init(struct device *dev)
{
	const struct lsm303c_config * const config = dev->config->config_info;
	struct lsm303c_data *data = dev->driver_data;
    
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

	return 0;
}

static const struct lsm303c_config lsm303c_config = {
	.i2c_master_dev_name = DT_ST_LSM303C_0_BUS_NAME,
	.i2c_slave_addr = DT_ST_LSM303C_0_BASE_ADDRESS,
	.i2c_m_slave_addr = CONFIG_LSM303C_M_I2C_ADDR,
};

static struct lsm303c_data lsm303c_data;

DEVICE_AND_API_INIT(lsm303c, DT_ST_LSM303C_0_LABEL, lsm303c_init,
		    &lsm303c_data, &lsm303c_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &lsm303c_api_funcs);
