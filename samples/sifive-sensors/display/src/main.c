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
	DEMO_FADE,
	DEMO_DIM_TEXT,
	DEMO_SYMBOLS,
} demo_state = DEMO_TEXT;

void fade(u32_t times) {
	for(int i = 0; i < times; i++) {
		/* Fade in */
		for(u32_t brightness = 0; brightness <= 100; brightness += 10) {
			sifive_display_setleds(0x1FFFFFF, 10, brightness);
		}
		/* Fade out */
		for(u32_t brightness = 100; brightness > 0; brightness -= 10) {
			sifive_display_setleds(0x1FFFFFF, 10, brightness);
		}
	}
}

void symbols() {
	for(int i = 1; i <= 11; i++) {
		sifive_display_setleds(sifive_font[i], 200, 100);
	}
}

void main(void)
{
	u32_t brightness = 50;
	u32_t direction = 1;

	while(1) {
		switch(demo_state) {
			case DEMO_TEXT:
				/* Add spaces to the front so that the text scrolls onto the screen */
				sifive_display_string("  Hello SiFive!", 15, 100);
				demo_state = DEMO_FADE;
				break;
			case DEMO_FADE:
				/* Fade a square in and out four times */
				fade(4);
				demo_state = DEMO_DIM_TEXT;
				break;
			case DEMO_DIM_TEXT:
				sifive_display_string("  Dim text", 15, 25);
				demo_state = DEMO_SYMBOLS;
				break;
			case DEMO_SYMBOLS:
				symbols();
				demo_state = DEMO_TEXT;
				break;
		}
	}
}
