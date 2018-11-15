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

enum {
	DEMO_TEXT,
	DEMO_FADE
} demo_state = DEMO_TEXT;

void fade(struct device *gpio, u32_t times) {
	for(int i = 0; i < times; i++) {
		/* Fade in */
		for(u32_t brightness = 0; brightness <= 100; brightness += 10) {
			sifive_display_setleds(gpio, 0x1FFFFFF, 10, brightness);
		}
		/* Fade out */
		for(u32_t brightness = 100; brightness > 0; brightness -= 10) {
			sifive_display_setleds(gpio, 0x1FFFFFF, 10, brightness);
		}
	}
}

void main(void)
{
	struct device *gpio = device_get_binding(CONFIG_GPIO_SIFIVE_GPIO_NAME);
	if(!gpio) {
		printk("Failed to get GPIO dev\n");
		return;
	}

	gpio_pin_configure(gpio, CONFIG_SIFIVE_DISPLAY_ROW_0_GPIO_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, CONFIG_SIFIVE_DISPLAY_ROW_1_GPIO_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, CONFIG_SIFIVE_DISPLAY_ROW_2_GPIO_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, CONFIG_SIFIVE_DISPLAY_ROW_3_GPIO_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, CONFIG_SIFIVE_DISPLAY_ROW_4_GPIO_PIN, GPIO_DIR_OUT);

	u32_t brightness = 50;
	u32_t direction = 1;

	while(1) {
		switch(demo_state) {
			case DEMO_TEXT:
				/* Add spaces to the front so that the text scrolls onto the screen */
				sifive_display_string(gpio, "  Hello SiFive!", 15, 100);
				demo_state = DEMO_FADE;
				break;
			case DEMO_FADE:
				/* Fade a square in and out four times */
				fade(gpio, 4);
				demo_state = DEMO_TEXT;
				break;
		}
	}
}
