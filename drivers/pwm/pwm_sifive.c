/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <pwm.h>
#include <device.h>
#include <board.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_PWM_LEVEL
#include <logging/sys_log.h>

/* Macros */

#define _REG32(_base, _offset)	(*(volatile u32_t *) ((_base) + (_offset)))	
#define PWM_REG(_config, _offset)	_REG32((_config)->base, _offset)

#define BIT_SET(_reg, _offset)	((_reg) |= (1 << (_offset)))
#define BIT_CLR(_reg, _offset)	((_reg) &= ~(1 << (_offset)))

/* Register Offsets */
#define REG_PWMCFG		0x00
#define REG_PWMCOUNT	0x08
#define REG_PWMS		0x10
#define REG_PWMCMP0		0x20
#define REG_PWMCMP(_channel)	(REG_PWMCMP0 + ((_channel) * 0x4))

/* Number of PWM Channels */
#define SF_NUMCHANNELS	4

/* pwmcfg Bit Offsets */
#define SF_PWMSTICKY	8
#define SF_PWMZEROCMP	9
#define SF_PWMDEGLITCH	10
#define SF_PWMENALWAYS	12
#define SF_PWMENONESHOT	13
#define SF_PWMCMPCENTER(_channel)	(16 + (_channel))
#define SF_PWMCMPGANG(_channel)		(24 + (_channel))
#define SF_PWMCMPIP(_channel)		(28 + (_channel))

/* pwmcount scale factor */
#define SF_PWMSCALEMASK	0xF
#define SF_PWMSCALE(_val)	(SF_PWMSCALEMASK & (_val))

#define SF_PWMCOUNT_MIN_WIDTH	15

/* Structure Declarations */

struct pwm_sifive_data {};

struct pwm_sifive_cfg {
	u32_t base;
	u32_t f_sys;
	u32_t cmpwidth;
};

/* Helper Functions */

/* API Functions */

int pwm_sifive_init(struct device *dev) {
	const struct pwm_sifive_cfg *config = dev->config->config_info;

	/* When pwms == pwmcmp0, reset the counter */
	BIT_SET(PWM_REG(config, REG_PWMCFG), SF_PWMZEROCMP);
	
	/* Enable continuous operation */
	BIT_SET(PWM_REG(config, REG_PWMCFG), SF_PWMENALWAYS);

	/* Clear IP config bits */
	BIT_CLR(PWM_REG(config, REG_PWMCFG), SF_PWMSTICKY);
	BIT_CLR(PWM_REG(config, REG_PWMCFG), SF_PWMDEGLITCH);

	/* Clear all channels */
	for(int i = 0; i < SF_NUMCHANNELS; i++) {
		PWM_REG(config, REG_PWMCMP(i)) = 0;
		BIT_CLR(PWM_REG(config, REG_PWMCFG), SF_PWMCMPCENTER(i));
		BIT_CLR(PWM_REG(config, REG_PWMCFG), SF_PWMCMPGANG(i));
	}

	return 0;
}

/*
 * Set the period and pulse width for a single PWM output.
 *
 * Parameters:
 * dev		- Pointer to the driver instance
 * pwm		- PWM channel
 * period	- Period in clock cycles
 * pulse	- Pulse width in clock cycles
 *
 * Return values:
 * 0		- Upon success
 * -EFAULT	- If the device instance was NULL
 * -EINVAL	- If an invalid PWM channel was requested
 * -ENOTSUP - If PWM channel 0 was requested
 * -EIO		- If the requested period or pulse length cannot be supported
 */
int pwm_sifive_pin_set(struct device *dev,
		u32_t pwm,
		u32_t period_cycles,
		u32_t pulse_cycles)
{
	if(dev == NULL) {
		SYS_LOG_ERR("The device instance pointer was NULL\n");
		return -EFAULT;
	}

	const struct pwm_sifive_cfg *config = dev->config->config_info;

	if(pwm >= SF_NUMCHANNELS) {
		SYS_LOG_ERR("The requested PWM channel %d is invalid\n", pwm);
		return -EINVAL;
	}

	/* Channel 0 sets the period, we can't output PWM with it */
	if((pwm == 0)) {
		SYS_LOG_ERR("PWM channel 0 cannot be configured\n");
		return -ENOTSUP;
	}

	/* We can't support periods greater than we can store in pwmcount */
	u32_t pwmcount_max = (1 << (config->cmpwidth + SF_PWMCOUNT_MIN_WIDTH)) - 1;
	if(period_cycles > pwmcount_max) {
		SYS_LOG_ERR("Requested period is %d but maximum allowable is %d\n",
				period_cycles, pwmcount_max);
		return -EIO;
	}

	/* Calculate the maximum value that pwmcmpX can be set to */
	u32_t max_cmp_val = ((1 << config->cmpwidth) - 1);

	/* Find the minimum value of pwmscale that will allow us to set the
	 * requested period */
	u32_t pwmscale = 0;
	while((period_cycles >> pwmscale) > max_cmp_val)
		pwmscale++;

	/* Make sure that we can scale that much */
	if(pwmscale > SF_PWMSCALEMASK) {
		SYS_LOG_ERR("Requested period is %d but maximum allowable is %d\n",
				period_cycles, max_cmp_val << pwmscale);
		return -EIO;
	}

	if(pulse_cycles > period_cycles) {
		SYS_LOG_ERR("Requested pulse %d is longer than requested period %d\n",
				pulse_cycles, period_cycles);	
		return -EIO;
	}

	/* Set the pwmscale field */
	PWM_REG(config, REG_PWMCFG) &= ~(SF_PWMSCALEMASK);
	PWM_REG(config, REG_PWMCFG) |= SF_PWMSCALE(pwmscale);

	/* Set the period by setting pwmcmp0 */
	PWM_REG(config, REG_PWMCMP0) = (period_cycles >> pwmscale);

	/* Set the duty cycle by setting pwmcmpX */
	PWM_REG(config, REG_PWMCMP(pwm)) = (pulse_cycles >> pwmscale);

	SYS_LOG_DBG("channel: %d, pwmscale: %d, pwmcmp0: %d, pwmcmp%d: %d",
			pwm, pwmscale, (period_cycles >> pwmscale),
			pwm, (pulse_cycles >> pwmscale));

	return 0;
}

/*
 * Get the clock frequency driving the PWM peripheral
 *
 * Parameters:
 * dev		- Pointer to the driver instance
 * pwm		- PWM channel
 * cycles	- Pointer to variable to hold clock frequency
 *
 * Return values:
 * 0		- Upon success
 * -EFAULT	- If the device instance was NULL
 * -EINVAL	- If an invalid PWM channel was requested
 */
int pwm_sifive_get_cycles_per_sec(struct device *dev,
		u32_t pwm,
		u64_t *cycles)
{
	if(dev == NULL)
		return -EFAULT;

	const struct pwm_sifive_cfg *config = dev->config->config_info;

	if(pwm >= SF_NUMCHANNELS)
		return -EINVAL;

	*cycles = config->f_sys;

	return 0;
}

/* Device Instantiation */

static struct pwm_driver_api pwm_sifive_api = {
	.pin_set = pwm_sifive_pin_set,
	.get_cycles_per_sec = pwm_sifive_get_cycles_per_sec,
};

#define PWM_SIFIVE_INIT(n)	\
	static struct pwm_sifive_data pwm_sifive_data_##n;	\
	static struct pwm_sifive_cfg pwm_sifive_cfg_##n = {	\
		.base = CONFIG_SIFIVE_PWM_##n##_BASE_ADDR,	\
		.f_sys = pwm_sifive_port_##n##_clk_freq,	\
		.cmpwidth = CONFIG_PWM_SIFIVE_CMPWIDTH,	\
		};	\
	DEVICE_AND_API_INIT(pwm_##n,	\
			CONFIG_SIFIVE_PWM_##n##_LABEL,	\
			pwm_sifive_init,	\
			&pwm_sifive_data_##n,	\
			&pwm_sifive_cfg_##n,	\
			POST_KERNEL,	\
			CONFIG_PWM_SIFIVE_INIT_PRIORITY,	\
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

