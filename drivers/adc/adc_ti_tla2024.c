/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <misc/util.h>
#include <init.h>

#include <adc.h>
#include <i2c.h>

#define SYS_LOG_DOMAIN "TLA2024"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_ADC_LEVEL
#include <logging/sys_log.h>

/* The conversion register contains the left-aligned signed 12-bit
 * result of the last ADC conversion */
#define REG_CONVERSION 0x00

/* Configuration register */
#define REG_CONFIG 0x01

#define CONFIG_OS_SHIFT	15
#define CONFIG_OS		(1 << CONFIG_OS_SHIFT)

#define CONFIG_MUX_SHIFT	12
#define CONFIG_MUX_MASK		(7 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_0_1		(0 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_0_3		(1 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_1_3		(2 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_2_3		(3 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_0_G		(4 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_1_G		(5 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_2_G		(6 << CONFIG_MUX_SHIFT)
#define CONFIG_MUX_3_G		(7 << CONFIG_MUX_SHIFT)
#define CONFIG_NUM_CHAN		8

#define CONFIG_PGA_SHIFT	9
#define CONFIG_PGA_MASK		(7 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_6000MV	(0 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_4000MV	(1 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_2000MV	(2 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_1000MV	(3 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_500MV	(4 << CONFIG_PGA_SHIFT)
#define CONFIG_PGA_125MV	(5 << CONFIG_PGA_SHIFT)

#if CONFIG_TI_TLA2024_FS_RANGE == 6000
	#define CONFIG_PGA_VAL CONFIG_PGA_6000MV
#elif CONFIG_TI_TLA2024_FS_RANGE == 4000
	#define CONFIG_PGA_VAL CONFIG_PGA_4000MV
#elif CONFIG_TI_TLA2024_FS_RANGE == 2000
	#define CONFIG_PGA_VAL CONFIG_PGA_2000MV
#elif CONFIG_TI_TLA2024_FS_RANGE == 1000
	#define CONFIG_PGA_VAL CONFIG_PGA_1000MV
#elif CONFIG_TI_TLA2024_FS_RANGE == 500
	#define CONFIG_PGA_VAL CONFIG_PGA_500MV
#elif CONFIG_TI_TLA2024_FS_RANGE == 125
	#define CONFIG_PGA_VAL CONFIG_PGA_125MV
#else
	#error CONFIG_TI_TLA2024_FS_RANGE set to unsupported value
#endif

#define CONFIG_MODE_SHIFT		8
#define CONFIG_MODE_MASK		(1 << CONFIG_MODE_SHIFT)
#define CONFIG_MODE_CONTINUOUS	(0 << CONFIG_MODE_SHIFT)
#define CONFIG_MODE_ONE_SHOT	(1 << CONFIG_MODE_SHIFT)

#define CONFIG_DR_SHIFT		5
#define CONFIG_DR_MASK		(7 << CONFIG_DR_SHIFT)
#define CONFIG_DR_128		(0 << CONFIG_DR_SHIFT)
#define CONFIG_DR_250		(1 << CONFIG_DR_SHIFT)
#define CONFIG_DR_490		(2 << CONFIG_DR_SHIFT)
#define CONFIG_DR_920		(3 << CONFIG_DR_SHIFT)
#define CONFIG_DR_1600		(4 << CONFIG_DR_SHIFT)
#define CONFIG_DR_2400		(5 << CONFIG_DR_SHIFT)
#define CONFIG_DR_3300		(6 << CONFIG_DR_SHIFT)

/* Reset the part using I2C "General Call" */
#define REG_GEN_CALL	0x00
#define GEN_CALL_RESET	0x06

/*
 * Data Structures
 */

struct tla2024_config {
	char *i2c_master_dev_name;
	u16_t i2c_slave_addr;
};

struct tla2024_data {
	struct device *i2c_master;
};

/*
 * Helper Functions
 */

/* Swap the high and low bytes of a 16-bit value */
static void swap_bytes(u16_t *value) {
	u16_t temp = *value & 0xFF;
	*value = *value >> 8;
	*value |= temp << 8;
}

/* Read a 16-bit register */
static int tla2024_i2c_read16(struct device *dev,
		const u16_t dev_addr,
		const u8_t reg_addr,
		u16_t *reg_val)
{
	/* Cast the 16-bit reg_val to an 8-bit array */
	int rc = i2c_burst_read(dev, dev_addr, reg_addr, (u8_t *)reg_val, 2);
	if(rc < 0) {
		SYS_LOG_DBG("Failed to read 16-bit register");
		return rc;	
	}

	/* Fix the byte order */
	swap_bytes(reg_val);

	return 0;
}

/* Update a 16-bit register */
static int tla2024_i2c_update16(struct device *dev,
		const u16_t dev_addr,
		const u8_t reg_addr,
		u16_t reg_mask,
		u16_t reg_val)
{
	u16_t buf;
	int rc;

	rc = tla2024_i2c_read16(dev, dev_addr, reg_addr, &buf);
	if(rc < 0) {
		return rc;
	}

	if((reg_mask & CONFIG_OS) == 0) {
		/* If we don't mean to write something to the OS bit, make sure
		 * we're writing a 0 to it */
		reg_mask |= CONFIG_OS;
		reg_val &= ~(CONFIG_OS);
	}

	/* Mask the buffer */
	buf &= ~(reg_mask);

	/* If the buffer needs update to match the updated value */
	if(buf != (reg_val & reg_mask)) {
		buf |= (reg_val & reg_mask);

		/* Build transmit buffer */
		u8_t tx_buf[3];
		tx_buf[0] = reg_addr;
		tx_buf[1] = (u8_t) (buf >> 8);
		tx_buf[2] = (u8_t) (buf & 0xFF);

		rc = i2c_write(dev, tx_buf, 3, dev_addr);
		if(rc < 0) {
			SYS_LOG_DBG("Failed to write 16-bit register");
			return rc;
		}
	}

	return 0;
}

static int tla2024_reset(struct device *dev) {
	struct tla2024_data *data = dev->driver_data;

	u8_t buf[] = { GEN_CALL_RESET };

	/* Write just the RESET general call value to the general call address */
	int rc = i2c_write(data->i2c_master, buf, 1, REG_GEN_CALL);
	if(rc < 0) {
		return rc;
	}

	return 0;
}

static int tla2024_wait(struct device *dev) {
	const struct tla2024_config * const config = dev->config->config_info;
	struct tla2024_data *data = dev->driver_data;

	u16_t config_val = 0;
	do {
		if(tla2024_i2c_read16(data->i2c_master,
				config->i2c_slave_addr,
				REG_CONFIG,
				&config_val) < 0) {
			SYS_LOG_DBG("Failed to read config register");
			return -EIO;
		}
	} while((config_val & CONFIG_OS) == 0);

	return 0;
}

/*
 * API Functions
 */

static int tla2024_channel_setup(struct device *dev,
		const struct adc_channel_cfg *channel_cfg)
{
	/* The TLA2024 does not support configurable gain, reference signal,
	 * acquisition time, or reconfigurable channels. There are exactly 8
	 * available channels, so merely switch the mux to the requested channel */

	if(dev == NULL || channel_cfg == NULL)
		return -EINVAL;

	const struct tla2024_config * const config = dev->config->config_info;
	struct tla2024_data *data = dev->driver_data;

	const u8_t channel = channel_cfg->channel_id;
	if(channel > CONFIG_MUX_MASK) {
		return -ENOTSUP;
	}

	if(tla2024_i2c_update16(data->i2c_master,
			config->i2c_slave_addr,
			REG_CONFIG,
			CONFIG_MUX_MASK,
			(channel << CONFIG_MUX_SHIFT)) < 0) {
		SYS_LOG_DBG("Failed to set input multiplexer");
		return -EIO;
	}

	return 0;
}

#define CHAN_BIT(_n) (1 << (_n))

static int tla2024_read(struct device *dev,
		const struct adc_sequence *sequence)
{
	if(dev == NULL || sequence == NULL)
		return -EINVAL;

	const struct tla2024_config * const config = dev->config->config_info;
	struct tla2024_data *data = dev->driver_data;

	/* If the request is for channels we don't have, error */
	if(sequence->channels & ~((2 << CONFIG_NUM_CHAN) - 1))
		return -ENOTSUP;

	/* Get a 16-bit pointer to the buffer */
	u16_t *sample_buffer = (u16_t *)sequence->buffer;

	/* Perform 1 + extra_samplings number of sample collections */
	u32_t num_rounds = 1;
	if(sequence->options != NULL) {
		num_rounds += sequence->options->extra_samplings;	
	}

	/* Count of the number of 16-bit samples collected */
	u32_t samples_collected = 0;

	/* For each sampling round */
	for(u32_t rounds = 0; rounds < num_rounds; rounds++) {
		/* For each available channel */
		for(int chan = 0; chan < CONFIG_NUM_CHAN; chan++) {
			/* If the channel has been requested, sample it */
			if(sequence->channels & CHAN_BIT(chan)) {
				/* Check sample buffer array bounds */
				size_t space = (sizeof(u16_t) * (samples_collected + 1));
				if(space > sequence->buffer_size)
					return -ENOMEM;

				/* Set mux */
				struct adc_channel_cfg chan_cfg = {
					.channel_id = chan,
				};
				if(tla2024_channel_setup(dev, &chan_cfg) < 0)
					return -EIO;

				/* Wait for previous conversion */
				if(tla2024_wait(dev) < 0)
					return -EIO;

				/* Start one-shot */
				if(tla2024_i2c_update16(data->i2c_master,
						config->i2c_slave_addr,
						REG_CONFIG,
						CONFIG_OS,
						CONFIG_OS) < 0) {
					SYS_LOG_DBG("Failed to start conversion");
					return -EIO;
				}

				/* Wait for conversion */
				if(tla2024_wait(dev) < 0)
					return -EIO;

				/* Read and store value */
				if(tla2024_i2c_read16(data->i2c_master,
						config->i2c_slave_addr,
						REG_CONVERSION,
						&(sample_buffer[samples_collected])) < 0) {
					SYS_LOG_DBG("Failed to read sample value");
					return -EIO;
				}

				/* Increment index into sample buffer */
				samples_collected += 1;
			}
		}

		/* After every round, run the callback and wait the requested
		 * interval */
		if(sequence->options != NULL) {
			/* Execute callback */
			if(sequence->options->callback != NULL) {
				(sequence->options->callback)(dev,
						sequence,
						rounds);
			}

			/* Wait sampling interval */
			k_busy_wait(sequence->options->interval_us);
		}
	}

	return 0;
}

const struct adc_driver_api tla2024_api = {
	.channel_setup = tla2024_channel_setup,
	.read = tla2024_read,
};

static int tla2024_init(struct device *dev) {
	const struct tla2024_config * const config = dev->config->config_info;
	struct tla2024_data *data = dev->driver_data;

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

	/* Reset the part */
	if(tla2024_reset(dev) < 0) {
		SYS_LOG_DBG("Failed to reset part");
		return -EIO;
	}

	/* Set the full-scale range */
	if(tla2024_i2c_update16(data->i2c_master, config->i2c_slave_addr,
			REG_CONFIG,
			CONFIG_PGA_MASK,
			CONFIG_PGA_VAL) < 0) {
		SYS_LOG_DBG("Failed to set the full-scale range");
		return -EIO;
	}

	SYS_LOG_DBG("Part reset and initialized");

	return 0;
}

/*
 * Device Instantiation
 */

const struct tla2024_config tla2024_config = {
	.i2c_master_dev_name = CONFIG_TI_TLA2024_I2C_MASTER_DEV_NAME,
	.i2c_slave_addr = CONFIG_TI_TLA2024_I2C_ADDR,
};

static struct tla2024_data tla2024_data;

DEVICE_AND_API_INIT(ti_tla2024, CONFIG_TI_TLA2024_DEV_NAME, tla2024_init,
			&tla2024_data, &tla2024_config, POST_KERNEL,
			CONFIG_ADC_INIT_PRIORITY, &tla2024_api);
