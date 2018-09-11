/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <pwm.h>
#include <device.h>

/* Macros */

#define _REG32(_base, _offset)	(*(volatile u32_t *) ((_base) + (_offset)))	
#define PWM_REG(_config, _offset)	_REG32((_config)->base, _offset)

/* Register Offsets */

#define REG_PWMCFG		0x00
#define REG_PWMCOUNT	0x08
#define REG_PWMS		0x10
#define REG_PWMCMP0		0x20
#define REG_PWMCMP1		0x24
#define REG_PWMCMP2		0x28
#define REG_PWMCMP3		0x2C

/* Structure Declarations */

struct pwm_sifive_data {};

struct pwm_sifive_cfg {
	u32_t base;
	u32_t f_sys;
};

/* Helper Functions */

/* API Functions */

int sifive_pin_set(struct device *dev,
		u32_t pwm,
		u32_t period_cycles,
		u32_t pulse_cycles)
{
	struct pwm_sifive_cfg *config = dev->config->config_info;
	return 0;
}

int sifive_get_cycles_per_sec(struct device *dev, u32_t pwm, u64_t *cycles) {
	struct pwm_sifive_cfg *config = dev->config->config_info;
	return 0;
}

/* Device Instantiation */

static struct pwm_driver_api pwm_sifive_api = {
	.pin_set = sifive_pin_set,
	.get_cycles_per_sec = sifive_get_cycles_per_sec,
};

#define PWM_SIFIVE_INIT(n)	\
	static struct pwm_sifive_data pwm_sifive_data_##n;	\
	static struct pwm_sifive_cfg pwm_sifive_cfg_##n = {	\
		.base = CONFIG_SIFIVE_PWM_##n##_BASE_ADDR,	\
		.f_sys = pwm_sifive_port_##n##_clk_freq,	\
		};	\
	DEVICE_AND_API_INIT(pwm_##n,	\
			CONFIG_SIFIVE_PWM_##n##_LABEL,	\
			pwm_sifive_init,	\
			&pwm_sifive_data_##n,	\
			&pwm_sifive_cfg_##n,	\
			POST_KERNEL,	\
			CONFIG_PWM_INIT_PRIORITY,	\
			&pwm_sifive_api)

#ifdef CONFIG_SIFIVE_PWM_0_LABEL
PWM_SIFIVE_INIT(0);
#endif /* CONFIG_SIFIVE_PWM_0_LABEL */

#ifdef CONFIG_SIFIVE_PWM_1_LABEL
PWM_SIFIVE_INIT(1);
#endif /* CONFIG_SIFIVE_PWM_1_LABEL */

#ifdef CONFIG_SIFIVE_PWM_2_LABEL
PWM_SIFIVE_INIT(2);
#endif /* CONFIG_SIFIVE_PWM_2_LABEL */

