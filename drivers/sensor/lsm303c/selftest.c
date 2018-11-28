/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lsm303c.h"

LOG_MODULE_DECLARE(st_lsm303c);

/* Average a number of samples with delay_us inter-sample spacing */
static int lsm303c_selftest_average(struct device *dev,
				    enum sensor_channel chan,
				    u32_t count,
				    u32_t delay_us,
				    s16_t *raw_avg)
{
	struct lsm303c_data *data = dev->driver_data;
	int rc = 0;
	s32_t sum = 0;
	s16_t *raw_sample;

	/* Get a pointer to the raw sample value for a given channel */
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		raw_sample = &(data->accel_sample_x);
	case SENSOR_CHAN_ACCEL_Y:
		raw_sample = &(data->accel_sample_y);
	case SENSOR_CHAN_ACCEL_Z:
		raw_sample = &(data->accel_sample_z);
		break;
	case SENSOR_CHAN_MAGN_X:
		raw_sample = &(data->magn_sample_x);
	case SENSOR_CHAN_MAGN_Y:
		raw_sample = &(data->magn_sample_y);
	case SENSOR_CHAN_MAGN_Z:
		raw_sample = &(data->magn_sample_z);
		break;
#if defined(CONFIG_LSM303C_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		raw_sample = &(data->temp_sample);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	/* Collect and accumulate samples */
	for (int i = 0; i < count; i++) {
		/* Fetch sample */
		rc = sensor_sample_fetch(dev);
		if (rc < 0) {
			LOG_DBG("Failed to fetch sample");
			return -EIO;
		}

		sum += *raw_sample;

		k_busy_wait(delay_us);
	}

	/* Compute and return average */
	*raw_avg = (s16_t) (sum / count);

	return 0;
}

/* Enable or disable the accelerometer self-test */
static int lsm303c_accel_selftest_set(struct device *dev, int st_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t st_state = 0;

	if (st_en) {
		st_state |= (0x01 << LSM303C_REG_CTRL_REG5_A_ST_SHIFT);
	}

	/* Write the accelerometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_slave_addr,
				LSM303C_REG_CTRL_REG5_A,
				LSM303C_REG_CTRL_REG5_A_ST_MASK,
				st_state) < 0) {
		return -EIO;
	}

	return 0;
}

/* Enable or disable the magnetometer self-test */
static int lsm303c_magn_selftest_set(struct device *dev, int st_en)
{
	struct lsm303c_data *data = dev->driver_data;
	const struct lsm303c_config *config = dev->config->config_info;
	u8_t st_state = 0;

	if (st_en) {
		st_state |= LSM303C_REG_CTRL_REG1_M_ST;
	}

	/* Write the accelerometer reboot bit */
	if (i2c_reg_update_byte(data->i2c_master,
				config->i2c_m_slave_addr,
				LSM303C_REG_CTRL_REG1_M,
				LSM303C_REG_CTRL_REG1_M_ST,
				st_state) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm303c_accel_selftest(struct device *dev)
{
	int rc = 0;
	s16_t raw_avg_ctrl[3] = {0};
	s16_t raw_avg_st[3] = {0};
	s16_t delta = 0;
	s32_t mag = 0;
	struct sensor_value val;

	/* Collect a control average without the self-test on */
	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_X,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_ctrl);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_Y,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_ctrl + 1);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_Z,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_ctrl + 2);
	if (rc < 0) {
		return rc;
	}

	/* Enable the self-test */
	rc = lsm303c_accel_selftest_set(dev, 1);
	if (rc < 0) {
		LOG_DBG("Failed to enable accel self-test");
		return -EIO;
	}

	/* Collect an average with the self-test on */
	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_X,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_st);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_Y,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_st + 1);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_ACCEL_Z,
				      CONFIG_LSM303C_ACCEL_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_ACCEL_SELFTEST_DELAY_US,
				      raw_avg_st + 2);
	if (rc < 0) {
		return rc;
	}

	/* Disable the self-test */
	rc = lsm303c_accel_selftest_set(dev, 0);
	if (rc < 0) {
		LOG_DBG("Failed to disable accel self-test");
		return -EIO;
	}

	/*
	 * Verify that the difference is within the datasheet tolerance of
	 * 0.070-1.500 g
	 */
	for (int i = 0; i < 3; i++) {
		/* Calculate delta */
		delta = raw_avg_st[i] - raw_avg_ctrl[i];

		/* Convert delta to value */
		lsm303c_accel_convert(&val,
				      delta,
				      LSM303C_DEFAULT_ACCEL_FULLSCALE_FACTOR);

		/*
		 * Check delta is between 686,465 um/s^2 and 14,709,975 um/s^2
		 */
		mag = val.val1 * 1000000 + val.val2;
		if ((mag < 686465) || (mag > 14709975)) {
			LOG_DBG("Magnitude outside of tolerances: %d", mag);
			return -EIO;
		}
	}

	return 0;
}

static int lsm303c_magn_selftest(struct device *dev)
{
	int rc = 0;
	s16_t raw_avg_ctrl[3] = {0};
	s16_t raw_avg_st[3] = {0};
	s16_t delta = 0;
	s32_t mag = 0;
	struct sensor_value val;

	/* Collect a control average without the self-test on */
	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_ctrl);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_ctrl + 1);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_ctrl + 2);
	if (rc < 0) {
		return rc;
	}

	/* Enable the self-test */
	rc = lsm303c_magn_selftest_set(dev, 1);
	if (rc < 0) {
		LOG_DBG("Failed to enable magn self-test");
		return -EIO;
	}

	/* Collect an average with the self-test on */
	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_st);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_st + 1);
	if (rc < 0) {
		return rc;
	}

	rc = lsm303c_selftest_average(dev,
				      SENSOR_CHAN_MAGN_X,
				      CONFIG_LSM303C_MAGN_SELFTEST_SAMPLES,
				      CONFIG_LSM303C_MAGN_SELFTEST_DELAY_US,
				      raw_avg_st + 2);
	if (rc < 0) {
		return rc;
	}

	/* Disable the self-test */
	rc = lsm303c_magn_selftest_set(dev, 0);
	if (rc < 0) {
		LOG_DBG("Failed to disable magn self-test");
		return -EIO;
	}

	/* Verify that the difference is within the datasheet tolerances */
	for (int i = 0; i < 3; i++) {
		/* Calculate delta */
		delta = raw_avg_st[i] - raw_avg_ctrl[i];

		/* Convert delta to value */
		lsm303c_magn_convert(&val,
				     delta,
				     LSM303C_DEFAULT_MAGN_FULLSCALE_FACTOR);

		mag = val.val1 * 1000000 + val.val2;

		/* X & Y should deflect by -1 to -3 gauss */
		if (i < 2) {
			if ((mag > -1000000) || (mag < -3000000)) {
				LOG_DBG("X/Y Magnitude outside of tolerances: %d", mag);
				return -EIO;
			}
		} else {
			/* Z should deflect by -0.1 to -1 gauss */
			if ((mag > -100000) || (mag < -1000000)) {
				LOG_DBG("Z Magnitude outside of tolerances: %d", mag);
				return -EIO;
			}
		}
	}

	return 0;
}

int lsm303c_selftest(struct device *dev)
{
	int rc = 0;

	rc = lsm303c_accel_selftest(dev);
	if (rc < 0) {
		LOG_DBG("Accelerometer self-test failed");
		return rc;
	}

	rc = lsm303c_magn_selftest(dev);
	if (rc < 0) {
		LOG_DBG("Magnetometer self-test failed");
		return rc;
	}

	return 0;
}
