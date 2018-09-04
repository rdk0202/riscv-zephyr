/*
 * Copyright (c) 2018 SiFive Inc.
 */

#include <device.h>
#include <i2c.h>

/* Structure declarations */
struct i2c_sifive_data {

};

struct i2c_sifive_config {
	u32_t base_addr;
};

/* Function declarations */
int i2c_sifive_init(struct device *dev);

int i2c_sifive_configure(struct device *dev, u32_t dev_config);

int i2c_sifive_transfer(
		struct device * dev,
		struct i2c_msg *msgs,
		u8_t num_msgs,
		u16_t addr);

/* Function implementations */
int i2c_sifive_init(struct device *dev) {
	return 0;
}

int i2c_sifive_configure(struct device *dev, u32_t dev_config) {
	return 0;
}

int i2c_sifive_transfer(
		struct device * dev,
		struct i2c_msg *msgs,
		u8_t num_msgs,
		u16_t addr) {
	return 0;
};

/* Device instantiation */
static struct i2c_driver_api i2c_sifive_api = {
	.configure = i2c_sifive_configure,
	.transfer = i2c_sifive_transfer
};

static struct i2c_sifive_data i2c_sifive_data_0;

static struct i2c_sifive_config i2c_sifive_config_0 = {
	.base_addr = CONFIG_SIFIVE_I2C_0_BASE_ADDR
};

DEVICE_AND_API_INIT(i2c_0,
		CONFIG_SIFIVE_I2C_0_LABEL,
		i2c_sifive_init,
		&i2c_sifive_data_0,
		&i2c_sifive_config_0,
		POST_KERNEL,
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		&i2c_sifive_api);

