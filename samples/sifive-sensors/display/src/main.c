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

void fade(struct sifive_display *disp, u32_t times) {
	for(int i = 0; i < times; i++) {
		/* Fade in */
		for(u32_t brightness = 0; brightness <= 100; brightness += 10) {
			sifive_display_canvas(disp, 0x1FFFFFF, 10, brightness);
		}
		/* Fade out */
		for(u32_t brightness = 100; brightness > 0; brightness -= 10) {
			sifive_display_canvas(disp, 0x1FFFFFF, 10, brightness);
		}
	}
}

void symbols(struct sifive_display *disp) {
	for(int i = 1; i <= 11; i++) {
		sifive_display_canvas(disp, sifive_font[i], 200, 100);
	}
}

void main(void)
{
	struct sifive_display *disp = sifive_display_get();
	while(1) {
		switch(demo_state) {
			case DEMO_TEXT:
				/* Add spaces to the front so that the text scrolls onto the screen */
				printk("Scrolling \"Hello SiFive!\"\n");
				sifive_display_string(disp, "  Hello SiFive!", 15, 100);
				demo_state = DEMO_FADE;
				break;
			case DEMO_FADE:
				/* Fade a square in and out four times */
				printk("Fading a square 4 times\n");
				fade(disp, 4);
				demo_state = DEMO_DIM_TEXT;
				break;
			case DEMO_DIM_TEXT:
				printk("Displaying dim text\n");
				sifive_display_string(disp, "  Dim text", 15, 25);
				demo_state = DEMO_SYMBOLS;
				break;
			case DEMO_SYMBOLS:
				printk("Displaying symbols\n");
				symbols(disp);
				demo_state = DEMO_TEXT;
				break;
		}
	}
}
