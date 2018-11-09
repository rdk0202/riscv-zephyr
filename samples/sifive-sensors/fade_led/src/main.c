/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <pwm.h>
#include <board.h>

#define PERIOD 1000

#define UP 1
#define DOWN 0

void main(void)
{
	struct device *gpio = device_get_binding(CONFIG_GPIO_SIFIVE_GPIO_NAME);
	if(!gpio) {
		printk("Failed to get GPIO dev\n");
		return;
	}

	gpio_pin_configure(gpio, 16, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, 17, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, 18, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, 19, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, 23, GPIO_DIR_OUT);

	gpio_pin_write(gpio, 16, 1);
	gpio_pin_write(gpio, 17, 1);
	gpio_pin_write(gpio, 18, 1);
	gpio_pin_write(gpio, 19, 1);
	gpio_pin_write(gpio, 23, 1);

	struct device *pwm0 = device_get_binding("pwm_0");
	if(!pwm0) {
		printk("Failed to get PWM 0 dev\n");
		return;
	}
	struct device *pwm1 = device_get_binding("pwm_1");
	if(!pwm1) {
		printk("Failed to get PWM 1 dev\n");
		return;
	}

	u32_t duty = PERIOD / 2;
	u32_t dir = UP;

	while(1) {
		if(pwm_pin_set_usec(pwm0, 1, PERIOD, duty) < 0)
			printk("Failed to set PWM0_1\n");
		if(pwm_pin_set_usec(pwm0, 2, PERIOD, duty) < 0)
			printk("Failed to set PWM0_2\n");
		if(pwm_pin_set_usec(pwm1, 1, PERIOD, duty) < 0)
			printk("Failed to set PWM1_1\n");
		if(pwm_pin_set_usec(pwm1, 2, PERIOD, duty) < 0)
			printk("Failed to set PWM1_2\n");
		if(pwm_pin_set_usec(pwm1, 3, PERIOD, duty) < 0)
			printk("Failed to set PWM1_3\n");

		if(dir == UP) {
			duty += PERIOD / 10;
			if(duty >= PERIOD) {
				duty = PERIOD;
				dir = DOWN;
			}
		} else {
			if(duty <= PERIOD / 10) {
				duty = 0;
				dir = UP;
			} else {
				duty -= PERIOD / 10;
			}
		}

		k_sleep(100);
	}
}
