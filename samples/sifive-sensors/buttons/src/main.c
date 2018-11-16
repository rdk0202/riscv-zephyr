/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <board.h>
#include <display/sifive_display.h>

#define BUTTON_A_PIN 11
#define BUTTON_B_PIN 0

#define DEBOUNCE_PERIOD K_MSEC(100)

volatile enum {
	DISPLAY_BLANK,
	DISPLAY_A,
	DISPLAY_B
} display_state = DISPLAY_BLANK;

/* Create kernel timers to debounce button inputs */
K_TIMER_DEFINE(a_debounce_timer, NULL, NULL);
K_TIMER_DEFINE(b_debounce_timer, NULL, NULL);

void button_pressed_callback(struct device *gpio,
		struct gpio_callback *cb,
		u32_t pins)
{
	if(pins & (1 << BUTTON_A_PIN)) {
		/* Is the debounce timer expired? */
		if(k_timer_remaining_get(&a_debounce_timer) == 0) {
			/* Start the debounce timer */
			k_timer_start(&a_debounce_timer, DEBOUNCE_PERIOD, 0);

			display_state = DISPLAY_A;
		}
	}
	else if(pins & (1 << BUTTON_B_PIN)) {
		/* Is the debounce timer expired? */
		if(k_timer_remaining_get(&b_debounce_timer) == 0) {
			/* Start the debounce timer */
			k_timer_start(&b_debounce_timer, DEBOUNCE_PERIOD, 0);

			display_state = DISPLAY_B;
		}
	}
}

static struct gpio_callback gpio_cb;

void main(void) {
	struct device *gpio = device_get_binding(CONFIG_GPIO_SIFIVE_GPIO_NAME);
	if(!gpio) {
		printk("Failed to get GPIO binding\n");
		return;
	}

	/* Configure the button GPIO pins for input and falling edge interrupts */
	gpio_pin_configure(gpio, BUTTON_A_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);
	gpio_pin_configure(gpio, BUTTON_B_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);

	/* Configure the callback struct */
	gpio_init_callback(&gpio_cb, button_pressed_callback, (1 << BUTTON_A_PIN) | (1 << BUTTON_B_PIN));

	/* Add the callback struct to the gpio device */
	gpio_add_callback(gpio, &gpio_cb);

	/* Enable the pin callbacks in the gpio device */
	gpio_pin_enable_callback(gpio, BUTTON_A_PIN);
	gpio_pin_enable_callback(gpio, BUTTON_B_PIN);

	struct sifive_display *display = sifive_display_get();

	while(1) {
		switch(display_state) {
		case DISPLAY_A:
			sifive_display_canvas(display, sifive_font['A'], 10, 100);
			break;
		case DISPLAY_B:
			sifive_display_canvas(display, sifive_font['B'], 10, 100);
			break;
		}
	}
}
