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

#include <display/sifive_display.h>

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
	gpio_pin_configure(gpio, 20, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, 23, GPIO_DIR_OUT);

	u32_t brightness = 50;
	u32_t direction = 1;

	while(1) {
		sifive_display_setleds(gpio, 0x50AAC544, 10, brightness);

		if(direction == 1) {
			brightness += 10;
			if(brightness == 100)
				direction = 0;
		} else {
			if(brightness == 0) {
				direction = 1;
				brightness = 10;
			} else {
				brightness -= 10;
			}
		}
	}
}
