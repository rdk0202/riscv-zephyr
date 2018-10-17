/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lsm303c.h"

int lsm303c_accel_reboot(struct device *dev)
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

int lsm303c_accel_init(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	/* Verify accelerometer identity */
	u8_t chip_id;
	if (i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
			      LSM303C_REG_WHO_AM_I_A, &chip_id) < 0) {
		SYS_LOG_DBG("Failed reading accelerometer ID");
		return -EIO;
	}
	if (chip_id != LSM303C_VAL_WHO_AM_I_A) {
		SYS_LOG_DBG("Invalid accelerometer ID 0x%x", chip_id);
		return -EIO;
	}
	SYS_LOG_DBG("Accelerometer ID 0x%x", chip_id);

	/* Configure accelerometer axis enables */
	if (lsm303c_accel_axis_ctrl(dev, LSM303C_ACCEL_ENABLE_X_AXIS,
				    LSM303C_ACCEL_ENABLE_Y_AXIS,
				    LSM303C_ACCEL_ENABLE_Z_AXIS) < 0) {
		SYS_LOG_DBG("Failed to enable accelerometer axes");
		return -EIO;
	}

	/* Configure accelerometer full scale */
	if (lsm303c_accel_set_fs_raw(dev, LSM303C_DEFAULT_ACCEL_FULLSCALE)
				     < 0) {
		SYS_LOG_DBG("Failed to set accelerometer full-scale");
		return -EIO;
	}

	/* Configure accelerometer sample rate */
	if (lsm303c_accel_set_odr_raw(dev, LSM303C_DEFAULT_ACCEL_SAMPLING_RATE)
				      < 0) {
		SYS_LOG_DBG("Failed to set accelerometer sample rate");
		return -EIO;
	}

	/* Set accelerometer block data update */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG1_A,
				LSM303C_REG_CTRL_REG1_A_BDU,
				LSM303C_REG_CTRL_REG1_A_BDU) < 0) {
		SYS_LOG_DBG("Failed to set accelerometer block data update");
		return -EIO;
	}

	return 0;
}

int lsm303c_accel_axis_ctrl(struct device *dev, int x_en, int y_en, int z_en)
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

int lsm303c_accel_set_fs_raw(struct device *dev, u8_t fs)
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

int lsm303c_accel_set_odr_raw(struct device *dev, u8_t odr)
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

int lsm303c_sample_fetch_accel(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t buf[6];
	u8_t addr = LSM303C_REG_OUT_X_L_A;

	for(int i = 0; i < 6; i++) {
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				   addr + i, buf + i) < 0) {
			SYS_LOG_DBG("Failed to read accelerometer sample");
			return -EIO;
		}
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

void lsm303c_accel_convert(struct sensor_value *val,
	s16_t raw_val, s32_t scale)
{
	/* The value is stored as a 16-bit signed integer with extrema of +/- the
	 * full-scale setting. */

	/* Convert the value to a long int to make sure we don't lose precision
	 * during arithmetic. We'll try to order operations to make the value
	 * bigger before we make it smaller. */
	long int dval = (long int) raw_val;

	/* Scale such that the maximum value is the full-scale value */
	val->val1 = (s32_t) (dval * scale / 32767);

	/* Get the fractional part in micro-meters/(s^2) by multiplying by 10e6
	 * and then taking the modulo 10e6 */
	val->val2 = (s32_t)((dval * scale * 1000000) / 32767) % 1000000;
}

int lsm303c_accel_get_channel(enum sensor_channel chan,
	struct sensor_value *val,
	struct lsm303c_data *data,
	s32_t scale)
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

int lsm303c_accel_channel_get(enum sensor_channel chan,
	struct sensor_value *val,
	struct lsm303c_data *data)
{
	return lsm303c_accel_get_channel(chan, val, data,
					LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR);
}
