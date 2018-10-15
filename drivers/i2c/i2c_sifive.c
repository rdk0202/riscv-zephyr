/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <device.h>
#include <i2c.h>
#include <board.h>

/* Macros */

#define _REG8(base, offset) (*(volatile u8_t *) ((base) + (offset)))
#define I2C_REG(config, reg) _REG8(config->base, reg)

#define IS_SET(config, reg, value)	(I2C_REG(config, reg) & (value))

/* Registers */
#define REG_PRESCALE_LOW	0x00
#define REG_PRESCALE_HIGH	0x04

#define REG_CONTROL			0x08

#define REG_TRANSMIT		0x0c
#define REG_RECEIVE			0x0c

#define REG_COMMAND			0x10
#define REG_STATUS			0x10

/* Values */
#define SF_CONTROL_EN	(1 << 7)
#define SF_CONTROL_IE	(1 << 6)

#define SF_TX_WRITE		(0 << 0)
#define SF_TX_READ		(1 << 0)

#define SF_CMD_START	(1 << 7)
#define SF_CMD_STOP		(1 << 6)
#define SF_CMD_READ		(1 << 5)
#define SF_CMD_WRITE	(1 << 4)
#define SF_CMD_ACK		(1 << 3)
#define SF_CMD_IACK		(1 << 0)

#define SF_STATUS_RXACK	(1 << 7)
#define SF_STATUS_BUSY	(1 << 6)
#define SF_STATUS_AL	(1 << 5)
#define SF_STATUS_TIP	(1 << 1)
#define SF_STATUS_IP	(1 << 0)

/* Values */

/* Structure declarations */
struct i2c_sifive_data {

};

struct i2c_sifive_cfg {
	u32_t base;
	u32_t f_sys;
};

/* Helper functions */

static bool inline i2c_sifive_busy(struct device *dev) {
	const struct i2c_sifive_cfg *config = dev->config->config_info;
	return IS_SET(config, REG_STATUS, SF_STATUS_TIP);
}

int i2c_sifive_send_addr(struct device *dev,
		u16_t addr,
		u16_t rw_flag)
{
	const struct i2c_sifive_cfg *config = dev->config->config_info;

	/* Wait for a previous transfer to complete */
	while(i2c_sifive_busy(dev)) ;

	/* Set transmit register to address with read/write flag */
	I2C_REG(config, REG_TRANSMIT) = (addr | rw_flag);

	/* Addresses are always written */
	u8_t command = SF_CMD_WRITE | SF_CMD_START;

	/* Write the command register to start the transfer */
	I2C_REG(config, REG_COMMAND) = command;

	while(i2c_sifive_busy(dev));

	if(IS_SET(config, REG_STATUS, SF_STATUS_RXACK)) {
		printk("I2C Rx failed to acknowledge\n");
		return -EIO;
	}

	return 0;
}

int i2c_sifive_write_msg(struct device *dev,
		struct i2c_msg *msg,
		u16_t addr)
{
	const struct i2c_sifive_cfg *config = dev->config->config_info;

	int rc = 0;

	rc = i2c_sifive_send_addr(dev, addr, SF_TX_WRITE);
	if(rc != 0) {
		printk("I2C failed to write message\n");
		return rc;
	}

	for(int i = 0; i < msg->len; i++) {
		/* Wait for a previous transfer */
		while(i2c_sifive_busy(dev));

		/* Put data in transmit reg */
		I2C_REG(config, REG_TRANSMIT) = (msg->buf)[i];

		/* Generate command byte */
		u8_t command = SF_CMD_WRITE;

		/* On the last byte of the message */
		if(i == (msg->len - 1)) {
			/* If the stop bit is requested, set it */
			if(msg->flags & I2C_MSG_STOP)
				command |= SF_CMD_STOP;
		}
		
		/* Write command reg */
		I2C_REG(config, REG_COMMAND) = command;

		/* Wait for a previous transfer */
		while(i2c_sifive_busy(dev));

		if(IS_SET(config, REG_STATUS, SF_STATUS_RXACK)) {
			printk("I2C Rx failed to acknowledge\n");
			return -EIO;
		}
	}

	return 0;
}

int i2c_sifive_read_msg(struct device *dev,
		struct i2c_msg *msg,
		u16_t addr)
{
	const struct i2c_sifive_cfg *config = dev->config->config_info;

	i2c_sifive_send_addr(dev, addr, SF_TX_READ);

	while(i2c_sifive_busy(dev));

	for(int i = 0; i < msg->len; i++) {
		/* Generate command byte */
		u8_t command = SF_CMD_READ | SF_CMD_ACK;

		/* On the last byte of the message */
		if(i == (msg->len - 1)) {
			/* If the stop bit is requested, set it */
			if(msg->flags & I2C_MSG_STOP)
				command |= SF_CMD_STOP;
		}
		
		/* Write command reg */
		I2C_REG(config, REG_COMMAND) = command;

		/* Wait for the read to complete */
		while(i2c_sifive_busy(dev));

		/* Store the received byte */
		(msg->buf)[i] = I2C_REG(config, REG_RECEIVE);
	}

	return 0;
}

/* API Functions */

int i2c_sifive_init(struct device *dev) {
	ARG_UNUSED(dev);

	return 0;
}

int i2c_sifive_configure(struct device *dev, u32_t dev_config) {
	const struct i2c_sifive_cfg *config = dev->config->config_info;

	/* Disable the I2C peripheral */
	I2C_REG(config, REG_CONTROL) = 0;

	/* Configure bus frequency */
	const u32_t speed_config = I2C_SPEED_GET(dev_config);
	u32_t i2c_speed = 0;
	switch(speed_config) {
		case I2C_SPEED_STANDARD:
			i2c_speed = 100000; /* 100 KHz */
			break;
		case I2C_SPEED_FAST:
			i2c_speed = 400000; /* 400 KHz */
			break;
		case I2C_SPEED_FAST_PLUS:
		case I2C_SPEED_HIGH:
		case I2C_SPEED_ULTRA:
		default:
			return -EIO;
	}

	const u16_t prescale = (config->f_sys / (5 * i2c_speed)) - 1;
	I2C_REG(config, REG_PRESCALE_LOW) = (u8_t) (0xFF & prescale);
	I2C_REG(config, REG_PRESCALE_HIGH) = (u8_t) (0xFF & (prescale >> 8));

	printk("Using prescale value of %d\n", prescale);

	/* We support I2C Master mode only */
	if(!(dev_config & I2C_MODE_MASTER))
		return -EIO;

	/* We don't support 10-bit addressing */
	if(dev_config & I2C_ADDR_10_BITS)
		return -EIO;

	/* Ensable the I2C peripheral */
	I2C_REG(config, REG_CONTROL) = SF_CONTROL_EN;

	return 0;
}

int i2c_sifive_transfer(struct device *dev,
		struct i2c_msg *msgs,
		u8_t num_msgs,
		u16_t addr)
{
	int rc = 0;

	if(msgs == NULL)
		return -EIO;

	for(int i = 0; i < num_msgs; i++) {
		if(msgs[i].flags & I2C_MSG_READ) {
			rc = i2c_sifive_read_msg(dev, &(msgs[i]), addr);
		} else {
			rc = i2c_sifive_write_msg(dev, &(msgs[i]), addr);
		}

		if(rc != 0) {
			printk("I2C failed to transfer messages\n");
			return rc;
		}
	}

	return 0;
};

/* Device instantiation */
static struct i2c_driver_api i2c_sifive_api = {
	.configure = i2c_sifive_configure,
	.transfer = i2c_sifive_transfer
};

static struct i2c_sifive_data i2c_sifive_data_0;

static struct i2c_sifive_cfg i2c_sifive_cfg_0 = {
	.base = CONFIG_SIFIVE_I2C_0_BASE_ADDR,
	.f_sys = i2c_sifive_port_0_clk_freq,
};

DEVICE_AND_API_INIT(i2c_0,
		CONFIG_SIFIVE_I2C_0_LABEL,
		i2c_sifive_init,
		&i2c_sifive_data_0,
		&i2c_sifive_cfg_0,
		POST_KERNEL,
		CONFIG_I2C_INIT_PRIORITY,
		&i2c_sifive_api);

