/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <pwm.h>
#include <board.h>

#define PWM_DRIVER	"pwm_1"
#define PERIOD (USEC_PER_SEC / 200)
#define PWM_CHANNEL 3

#define DOWN 0
#define UP 1

void main(void)
{
	struct device *pwm_dev;

	int channels[3] = {1, 2, 3};
	int duty[3] = {PERIOD, PERIOD/2, 0};
	u8_t dir[3] = {DOWN, DOWN, UP};

	printk("PWM demo app-blink LED\n");

	pwm_dev = device_get_binding(PWM_DRIVER);
	if (!pwm_dev) {
		printk("Cannot find %s!\n", PWM_DRIVER);
		return;
	}

	while (1) {
		for(int i = 0; i < 3; i++) {
			if (pwm_pin_set_usec(pwm_dev, channels[i], PERIOD, (u32_t) duty[i])) {
				printk("pwm pin set fails\n");
				return;
			}

			if (dir[i] == DOWN) {
				duty[i] -= PERIOD / 10;
				if(duty[i] <= 0) {
					duty[i] = 0;
					dir[i] = UP;
				}

			} else {
				duty[i] += PERIOD / 10;

				if(duty[i] >= PERIOD) {
					duty[i] = PERIOD;
					dir[i] = DOWN;
				}
			}
		}

		k_sleep(100);
	}
}
