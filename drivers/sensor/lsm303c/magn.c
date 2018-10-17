/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lsm303c.h"

int lsm303c_magn_reboot(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	/* Write the magnetometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG2_M,
				LSM303C_REG_CTRL_REG2_M_REBOOT,
				LSM303C_REG_CTRL_REG2_M_REBOOT) < 0) {
		return -EIO;
	}

	/* Poll the magnetometer reboot bit to detect reboot completes */
	int rc = 0;
	u8_t boot_reg;
	while(1) {
		rc = i2c_reg_read_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG2_M, &boot_reg);
		if(rc < 0) {
			SYS_LOG_DBG("I2C failed to read 0x%02X@0x%02X",
					LSM303C_REG_CTRL_REG2_M, config->i2c_m_slave_addr);
		}

		if((boot_reg & LSM303C_REG_CTRL_REG2_M_REBOOT) == 0) {
			break;
		}
	}
	return 0;
}

int lsm303c_magn_init(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t chip_id;
	/* Verify magnetometer identity */
	if (i2c_reg_read_byte(data->i2c_master, config->i2c_m_slave_addr,
			      LSM303C_REG_WHO_AM_I_M, &chip_id) < 0) {
		SYS_LOG_DBG("Failed to read magnetometer ID");
		return -EIO;
	}
	if (chip_id != LSM303C_VAL_WHO_AM_I_M) {
		SYS_LOG_DBG("Invalid magnetometer ID 0x%x", chip_id);
		return -EIO;
	}
	SYS_LOG_DBG("Magnetometer ID 0x%x", chip_id);

	/* Enable magnetometer axes */
	if (lsm303c_magn_axis_ctrl(dev, LSM303C_MAGN_ENABLE_X_AXIS,
				   LSM303C_MAGN_ENABLE_Y_AXIS,
				   LSM303C_MAGN_ENABLE_Z_AXIS) < 0) {
		SYS_LOG_DBG("Failed to enable magnetometer axes");
		return -EIO;
	}

	/* Set magnetometer fullscale */
	if (lsm303c_magn_set_fs_raw(dev, LSM303C_DEFAULT_MAGN_FULLSCALE)
				    < 0) {
		SYS_LOG_DBG("Failed to set magnetometer full-scale");
		return -EIO;
	}

	/* Set magnetometer sample rate */
	if (lsm303c_magn_set_odr_raw(dev, LSM303C_DEFAULT_MAGN_SAMPLING_RATE)
				     < 0) {
		SYS_LOG_DBG("Failed to set magnetometer sample rate");
		return -EIO;
	}

	/* Set magnetometer block data update */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG5_M,
				LSM303C_REG_CTRL_REG5_M_BDU,
				LSM303C_REG_CTRL_REG5_M_BDU) < 0) {
		SYS_LOG_DBG("Failed to set magnetometer block data update");
		return -EIO;
	}

	/* Enable thermometer channel */
	if (lsm303c_temp_enable(dev, LSM303C_ENABLE_TEMP) < 0) {
		SYS_LOG_DBG("Failed to enable temperature");
		return -EIO;
	}

	return 0;
}

int lsm303c_magn_axis_ctrl(struct device *dev, int x_en, int y_en, int z_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	int rc = 0;

	/* Enable the X & Y axes */
	if(x_en || y_en) {
		u8_t om_state = LSM303C_REG_CTRL_REG1_M_OM_ULTRAHIGH;
		rc = i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_OM_MASK,
				om_state << LSM303C_REG_CTRL_REG1_M_OM_SHIFT);
		if(rc < 0) {
			SYS_LOG_DBG("Failed to enable x & y magn axes\n");
			return -EIO;
		}
	}

	/* Enable the Z axis */
	if(z_en) {
		u8_t om_state = LSM303C_REG_CTRL_REG4_M_OMZ_ULTRAHIGH;
		rc = i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG4_M,
				LSM303C_REG_CTRL_REG4_M_OMZ_MASK,
				om_state << LSM303C_REG_CTRL_REG4_M_OMZ_SHIFT);
		if(rc < 0) {
			SYS_LOG_DBG("Failed to enable z magn axis\n");
			return -EIO;
		}
	}

	return 0;
}

int lsm303c_magn_set_fs_raw(struct device *dev, u8_t fs)
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

int lsm303c_magn_set_odr_raw(struct device *dev, u8_t odr)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	/* Set the device to continuous conversion mode */
	u8_t md_state = LSM303C_REG_CTRL_REG3_M_MD_CONTINUOUS;
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG3_M,
				LSM303C_REG_CTRL_REG3_M_MD_MASK,
				md_state << LSM303C_REG_CTRL_REG3_M_MD_SHIFT) < 0) {
		return -EIO;
	}

	/* Set sample frequency */
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_DO_MASK,
				odr << LSM303C_REG_CTRL_REG1_M_DO_SHIFT) < 0) {
		return -EIO;
	}

	return 0;
}

int lsm303c_sample_fetch_magn(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t buf[6];
	u8_t addr = LSM303C_REG_OUT_X_L_M;

	for(int i = 0; i < 6; i++) {
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_m_slave_addr,
				   addr + i, buf + i) < 0) {
			SYS_LOG_DBG("Failed to read magnetometer sample");
			return -EIO;
		}
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
int lsm303c_sample_fetch_temp(struct device *dev)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t buf[2];
	u8_t addr = LSM303C_REG_TEMP_L_M;

	for(int i = 0; i < 2; i++) {
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_m_slave_addr,
				   addr + i, buf + i) < 0) {
			SYS_LOG_DBG("Failed to read temperature sample");
			return -EIO;
		}
	}

	data->temp_sample = (s16_t) ((((u16_t) buf[1]) << 8) | ((u16_t) buf[0]));

	return 0;
}
#endif

void lsm303c_magn_convert(struct sensor_value *val, int raw_val, s32_t scale)
{
	/* The value is stored as a 16-bit signed integer with extrema of +/- the
	 * full-scale setting. */

	/* Convert the value to a long int to make sure we don't lose precision
	 * during arithmetic. We'll try to order operations to make the value
	 * bigger before we make it smaller. */
	long int dval = (long int) raw_val;

	/* Scale such that the maximum value is the full-scale value */
	val->val1 = (s32_t) (dval * scale / 32767);

	/* Get the fractional part in micro-gauss by multiplying by 10e6
	 * and then taking the modulo 10e6 */
	val->val2 = (s32_t)((dval * scale * 1000000) / 32767) % 1000000;
}

int lsm303c_magn_get_channel(enum sensor_channel chan,
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

int lsm303c_magn_channel_get(enum sensor_channel chan,
	struct sensor_value *val,
	struct lsm303c_data *data)
{
	return lsm303c_magn_get_channel(chan, val, data,
					LSM303C_DEFAULT_MAGN_FULLSCALE_FACTOR);
}

int lsm303c_temp_enable(struct device *dev, int t_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;

	u8_t state = 0;
	if(t_en) {
		state |= LSM303C_REG_CTRL_REG1_M_TEMP_EN;
	}
	if (i2c_reg_update_byte(data->i2c_master, config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_TEMP_EN,
				state) < 0) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_LSM303C_ENABLE_TEMP)
void lsm303c_magn_channel_get_temp(struct sensor_value *val,
	struct lsm303c_data *data)
{
	/* The raw value is stored in 8ths of a degree C with a zero point at
	 * -25 degrees C. */

	/* Convert to 32-bit from the 16-bit stored sample */
	s32_t raw_value = (s32_t) data->temp_sample;
	
	/* Get the integer part by ofsetting by 8 * 25 and then dividing by 8 */
	val->val1 = (raw_value + 200) / 8;

	/* Get the fractional part in micro-degrees C (10e-6) by multiplying by
	 * 10e6 and then modulo 10e6 */
	val->val2 = ((1000000 * (raw_value + 200)) / 8) % 1000000;
}
#endif
