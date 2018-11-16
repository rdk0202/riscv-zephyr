/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sensor.h>
#include <i2c.h>

/*
 * Logging Configuration
  */

#define SYS_LOG_DOMAIN "LTR329"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>

/* Control Register */
#define REG_ALS_CONTR		0x80

#define CONTR_GAIN_SHIFT	2
#define CONTR_GAIN_MASK		(7 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_1X		(0 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_2X		(1 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_4X		(2 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_8X		(3 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_48X		(6 << CONTR_GAIN_SHIFT)
#define CONTR_GAIN_96X		(7 << CONTR_GAIN_SHIFT)

#if CONFIG_LTR329_GAIN == 1
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_1X
#elif CONFIG_LTR329_GAIN == 2
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_2X
#elif CONFIG_LTR329_GAIN == 4
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_4X
#elif CONFIG_LTR329_GAIN == 8
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_8X
#elif CONFIG_LTR329_GAIN == 48
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_48X
#elif CONFIG_LTR329_GAIN == 96
	#define CONFIG_LTR329_GAIN_VAL CONTR_GAIN_96X
#else
	#error CONFIG_LTR329_GAIN set to unsupported value
#endif

#define CONTR_RESET_SHIFT	1
#define CONTR_RESET			(1 << CONTR_RESET_SHIFT)

#define CONTR_MODE_SHIFT	0
#define CONTR_MODE_MASK		(1 << 0)
#define CONTR_MODE_STANDBY	(0 << CONTR_MODE_SHIFT)
#define CONTR_MODE_ACTIVE	(1 << CONTR_MODE_SHIFT)

/* Measurement Rate Register */
#define REG_ALS_MEAS_RATE	0x85

#define RATE_INTEG_SHIFT	3
#define RATE_INTEG_MASK		(7 << RATE_INTEG_SHIFT)
#define RATE_INTEG_50MS		(1 << RATE_INTEG_SHIFT)
#define RATE_INTEG_100MS	(0 << RATE_INTEG_SHIFT)
#define RATE_INTEG_150MS	(4 << RATE_INTEG_SHIFT)
#define RATE_INTEG_200MS	(2 << RATE_INTEG_SHIFT)
#define RATE_INTEG_250MS	(5 << RATE_INTEG_SHIFT)
#define RATE_INTEG_300MS	(6 << RATE_INTEG_SHIFT)
#define RATE_INTEG_350MS	(7 << RATE_INTEG_SHIFT)
#define RATE_INTEG_400MS	(3 << RATE_INTEG_SHIFT)

#if CONFIG_LTR329_INTEG_RATE == 50
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_50MS
#elif CONFIG_LTR329_INTEG_RATE == 100
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_100MS
#elif CONFIG_LTR329_INTEG_RATE == 150
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_150MS
#elif CONFIG_LTR329_INTEG_RATE == 200
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_200MS
#elif CONFIG_LTR329_INTEG_RATE == 250
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_250MS
#elif CONFIG_LTR329_INTEG_RATE == 300
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_300MS
#elif CONFIG_LTR329_INTEG_RATE == 350
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_350MS
#elif CONFIG_LTR329_INTEG_RATE == 400
	#define CONFIG_LTR329_INTEG_RATE_VAL RATE_INTEG_400MS
#else
	#error CONFIG_LTR329_INTEG_RATE set to unsupported value
#endif

#define RATE_MEAS_SHIFT		0
#define RATE_MEAS_MASK		(7 << RATE_MEAS_SHIFT)
#define RATE_MEAS_50MS		(0 << RATE_MEAS_SHIFT)
#define RATE_MEAS_100MS		(1 << RATE_MEAS_SHIFT)
#define RATE_MEAS_200MS		(2 << RATE_MEAS_SHIFT)
#define RATE_MEAS_500MS		(3 << RATE_MEAS_SHIFT)
#define RATE_MEAS_1000MS	(4 << RATE_MEAS_SHIFT)
#define RATE_MEAS_2000MS	(5 << RATE_MEAS_SHIFT)

#if CONFIG_LTR329_MEAS_RATE == 50
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_50MS
#elif CONFIG_LTR329_MEAS_RATE == 100
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_100MS
#elif CONFIG_LTR329_MEAS_RATE == 200
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_200MS
#elif CONFIG_LTR329_MEAS_RATE == 500
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_500MS
#elif CONFIG_LTR329_MEAS_RATE == 1000
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_1000MS
#elif CONFIG_LTR329_MEAS_RATE == 2000
	#define CONFIG_LTR329_MEAS_RATE_VAL RATE_MEAS_2000MS
#else
	#error CONFIG_LTR329_MEAS_RATE set to unsupported value
#endif

/* Part and Manufacturer ID Registers */
#define REG_PART_ID			0x86
#define VAL_PART_ID			0xA0
#define REG_MANUFAC_ID		0x87
#define VAL_MANUFAC_ID		0x05

/* Data Registers -- read in-order to get single-measurement */
#define REG_ALS_DATA_CH1_0	0x88 /* Low byte */
#define REG_ALS_DATA_CH1_1	0x89 /* High byte */
#define REG_ALS_DATA_CH0_0	0x8A
#define REG_ALS_DATA_CH0_1	0x8B

/* Status Register */
#define REG_ALS_STATUS		0x8C

#define STATUS_VALID_SHIFT	7
#define STATUS_VALID		(0 << STATUS_VALID_SHIFT)
#define STATUS_INALID		(1 << STATUS_VALID_SHIFT)

#define STATUS_GAIN_SHIFT	4
#define STATUS_GAIN_MASK	(0x7 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_1X		(0 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_2X		(1 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_4X		(2 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_8X		(3 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_48X		(6 << STATUS_GAIN_SHIFT)
#define STATUS_GAIN_96X		(7 << STATUS_GAIN_SHIFT)

#define STATUS_DATA_SHIFT	2
#define STATUS_DATA_OLD		(0 << STATUS_DATA_SHIFT)
#define STATUS_DATA_NEW		(1 << STATUS_DATA_SHIFT)

/*
 * Struct Definitions
 */

struct ltr329_config {
	char *i2c_master_dev_name;
	u16_t i2c_slave_addr;
};

struct ltr329_data {
	struct device *i2c_master;
/* Sensor Sample for Channel 0 (Visible + IR) */
	u16_t channel_0_sample;
/* Sensor Sample for Channel 1 (IR) */
	u16_t channel_1_sample;
};

/*
 * Helper Functions
 */

static void ltr329_convert_lux(u16_t sample,
		s64_t gain,
		u32_t channel,
		struct sensor_value *val)
{
	s64_t value = 0;

	switch(channel) {
	case 0:
		/* Channel 0 sees 200 Lux at a value of ~1190 with 96X gain */
		value = ((s64_t) sample) * 96LL * 200LL * 1000000LL / (1190LL * gain);
		break;
	case 1:
		/* Channel 1 sees 200 Lux at a value of ~4675 with 96X gain */
		value = ((s64_t) sample) * 96LL * 200LL * 1000000LL / (4675LL * gain);
		break;
	default:
		value = 0;
	}

	val->val1 = (s32_t) (value / 1000000LL);
	val->val2 = (s32_t) (value % 1000000LL);
}

/*
 * API Functions
 */

static int ltr329_sample_fetch(struct device *dev, enum sensor_channel chan) {
	const struct ltr329_config * const config = dev->config->config_info;
	struct ltr329_data *data = dev->driver_data;

	u8_t reg = 0;

	switch(chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_LIGHT:
	case SENSOR_CHAN_IR:
		/* Fetch channel 1 */
		data->channel_1_sample = 0;
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				REG_ALS_DATA_CH1_0, &reg) < 0) {
			SYS_LOG_DBG("Failed to read 0x%02X@0x%02X",
					REG_ALS_DATA_CH1_0, config->i2c_slave_addr);
			return -EIO;
		}
		data->channel_1_sample |= (u16_t) reg;
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				REG_ALS_DATA_CH1_1, &reg) < 0) {
			SYS_LOG_DBG("Failed to read 0x%02X@0x%02X",
					REG_ALS_DATA_CH1_1, config->i2c_slave_addr);
			return -EIO;
		}
		data->channel_1_sample |= (((u16_t) reg) << 8);

		/* Fetch channel 0 */
		data->channel_0_sample = 0;
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				REG_ALS_DATA_CH0_0, &reg) < 0) {
			SYS_LOG_DBG("Failed to read 0x%02X@0x%02X",
					REG_ALS_DATA_CH0_0, config->i2c_slave_addr);
			return -EIO;
		}
		data->channel_0_sample |= (u16_t) reg;
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				REG_ALS_DATA_CH0_1, &reg) < 0) {
			SYS_LOG_DBG("Failed to read 0x%02X@0x%02X",
					REG_ALS_DATA_CH0_1, config->i2c_slave_addr);
			return -EIO;
		}
		data->channel_0_sample |= (((u16_t) reg) << 8);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ltr329_channel_get(struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct ltr329_data *data = dev->driver_data;

	switch(chan) {
	case SENSOR_CHAN_ALL:
		/* Convert samples */
		ltr329_convert_lux(data->channel_0_sample,
				CONFIG_LTR329_GAIN,
				0,
				&(val[0]));
		ltr329_convert_lux(data->channel_1_sample,
				CONFIG_LTR329_GAIN,
				1,
				&(val[1]));
		break;
	case SENSOR_CHAN_LIGHT:
		/* Convert sample */
		ltr329_convert_lux(data->channel_0_sample,
				CONFIG_LTR329_GAIN,
				0,
				val);
		break;
	case SENSOR_CHAN_IR:
		/* Convert sample */
		ltr329_convert_lux(data->channel_1_sample,
				CONFIG_LTR329_GAIN,
				1,
				val);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

const struct sensor_driver_api ltr329_api_funcs = {
	.sample_fetch = ltr329_sample_fetch,
	.channel_get = ltr329_channel_get,
};

/*
 * Device Initialization
 */

static int ltr329_init(struct device *dev) {
	const struct ltr329_config * const config = dev->config->config_info;
	struct ltr329_data *data = dev->driver_data;

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

	/* Read and check part and manufacturer ID */
	u8_t part_id;
	if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
			REG_PART_ID, &part_id) < 0) {
		SYS_LOG_DBG("Failed to read part id");
		return -EIO;
	}
	if(part_id != VAL_PART_ID) {
		SYS_LOG_DBG("Invalid part ID 0x%x", part_id);
		return -EIO;
	}
	SYS_LOG_DBG("LTR329 Part ID: 0x%0x", part_id);
	u8_t manufac_id;
	if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
			REG_MANUFAC_ID, &manufac_id) < 0) {
		SYS_LOG_DBG("Failed to read manufacturer id");
		return -EIO;
	}
	if(manufac_id != VAL_MANUFAC_ID) {
		SYS_LOG_DBG("Invalid manufacturer ID 0x%x", manufac_id);
		return -EIO;
	}
	SYS_LOG_DBG("LTR329 Manufacturer ID: 0x%0x", manufac_id);

	/* Perform Reset */
	if(i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
			REG_ALS_CONTR,
			CONTR_RESET,
			CONTR_RESET) < 0) {
		SYS_LOG_DBG("Failed to reset part");
		return -EIO;
	}
	/* Wait for reset to complete */
	while(1) {
		u8_t contr_reg;
		if(i2c_reg_read_byte(data->i2c_master, config->i2c_slave_addr,
				REG_ALS_CONTR, &contr_reg) < 0) {
			SYS_LOG_DBG("Failed to read 0x%02X@0x%02X",
					REG_ALS_CONTR, config->i2c_slave_addr);
			return -EIO;
		}
		if((contr_reg & CONTR_RESET) == 0)
			break;
	}		

	/* Configure Gain */
	if(i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
			REG_ALS_CONTR,
			CONTR_GAIN_MASK,
			CONFIG_LTR329_GAIN_VAL) < 0) {
		SYS_LOG_DBG("Failed to set gain");
		return -EIO;
	}

	/* Configure Integration and Measurement Rates */
	if(i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
			REG_ALS_MEAS_RATE,
			RATE_INTEG_MASK,
			CONFIG_LTR329_INTEG_RATE_VAL) < 0) {
		SYS_LOG_DBG("Failed to set integration rate");
		return -EIO;
	}
	if(i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
			REG_ALS_MEAS_RATE,
			RATE_MEAS_MASK,
			CONFIG_LTR329_MEAS_RATE_VAL) < 0) {
		SYS_LOG_DBG("Failed to set sample rate");
		return -EIO;
	}

	/* Leave Stand-by Mode */
	if(i2c_reg_update_byte(data->i2c_master, config->i2c_slave_addr,
			REG_ALS_CONTR,
			CONTR_MODE_MASK,
			CONTR_MODE_ACTIVE) < 0) {
		SYS_LOG_DBG("Failed activate part");
		return -EIO;
	}

	return 0;
}

/*
 * Device Instantiation
 */

const struct ltr329_config ltr329_config = {
	.i2c_master_dev_name = CONFIG_LTR329_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_LTR329_I2C_ADDR,
};

static struct ltr329_data ltr329_data;

DEVICE_AND_API_INIT(ltr329, CONFIG_LTR329_DEV_NAME, ltr329_init,
		    &ltr329_data, &ltr329_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &ltr329_api_funcs);
