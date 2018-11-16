/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <string.h>
#include <board.h>
#include <gpio.h>
#include <pwm.h>
#include <pinmux.h>

#include <display/sifive_display.h>

#define LED_ROW_4	16
#define LED_ROW_3	17
#define LED_ROW_2	18
#define LED_ROW_1	20
#define LED_ROW_0	23

/*
 * Display data
 */

static struct sifive_display display;

/*
 * Helper functions to set rows and columns
 */

/* Set the row to val */
static int sifive_display_setrow(struct sifive_display *disp,
		const u32_t pin,
		const u32_t val)
{
	/* Make sure the GPIO is configured as output */
	gpio_pin_configure(disp->gpio, pin, GPIO_DIR_OUT);
	/* Set the GPIO pin to val */
	return gpio_pin_write(disp->gpio, pin, val);
}

/* Turn on column col for duty cycles out of CONFIG_SIFIVE_DISPLAY_PERIOD cycles */
static int sifive_display_setcol(struct sifive_display *disp,
		const u32_t col,
		const u32_t duty)
{
	int rc = 0;

	switch(col) {
	case 4:
		rc = pwm_pin_set_cycles(disp->pwm[CONFIG_SIFIVE_DISPLAY_COL_0_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_0_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 3:
		rc = pwm_pin_set_cycles(disp->pwm[CONFIG_SIFIVE_DISPLAY_COL_1_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_1_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 2:
		rc = pwm_pin_set_cycles(disp->pwm[CONFIG_SIFIVE_DISPLAY_COL_2_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_2_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 1:
		rc = pwm_pin_set_cycles(disp->pwm[CONFIG_SIFIVE_DISPLAY_COL_3_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_3_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 0:
		rc = pwm_pin_set_cycles(disp->pwm[CONFIG_SIFIVE_DISPLAY_COL_4_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_4_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	}

	return rc;
}

static u32_t sifive_display_rstring(const char *msg, const int pixel_idx) {
	int chr = 0;
	int pixel_count = 0;

	uint32_t canvas = 0;

	/* Index through the string until we are in the leftmost character */
	while(msg[chr] != 0) {
		pixel_count += (sifive_font[(int)msg[chr++]] >> 28) & 0x0F;

		if(pixel_count > pixel_idx) {
			/* Reset to the beginning of the previous character */
			chr--;
			pixel_count -= (sifive_font[(int)msg[chr]] >> 28) & 0x0F;      
			break;
		}
	}

	if(msg[chr] == 0)
		return canvas;

	/* Width of the current character */
	int width = (sifive_font[(int)msg[chr]] >> 28) & 0x0F;

	// idx is the index from the LHS of the character
	// pixel_idx will always be >= pixel_count
	int idx = pixel_idx - pixel_count;

	uint32_t imask, ibits;

	// Generate the canvas
	for(u32_t i = 0; i < 5; i++) {
		imask=0b1000010000100001000010000;
		if(idx)
			imask = imask >> idx;

		ibits = sifive_font[(int)msg[chr]] & imask;

		if(i>idx)
			ibits = ibits >> (i - idx);
		if(i<idx)
			ibits = ibits << (idx - i);

		canvas |= ibits;

		idx++;

		if(idx >= width) {
			idx = 0;
			chr++;

			if(msg[chr] == 0)
				break;

			width = (sifive_font[(int) msg[chr]] >> 28) & 0x0F;      
		}
	}
	return(canvas);
}

/*
 * API Functions
 */

/* Get the display data struct */
struct sifive_display *sifive_display_get(void) {
	return &display;
}

int sifive_display_canvas(struct sifive_display *disp,
		const u32_t canvas,
		const u32_t hangtime,
		const u32_t brightness)
{
	if(IS_ENABLED(CONFIG_SIFIVE_DISPLAY_TOGGLE_UART_0)) {
		pinmux_pin_set(disp->pinmux, CONFIG_SIFIVE_DISPLAY_UART_0_TX, SIFIVE_PINMUX_DISABLE);
		pinmux_pin_set(disp->pinmux, CONFIG_SIFIVE_DISPLAY_UART_0_RX, SIFIVE_PINMUX_DISABLE);
	}

	const u32_t rowary[] = {
		CONFIG_SIFIVE_DISPLAY_ROW_0_GPIO_PIN,
		CONFIG_SIFIVE_DISPLAY_ROW_1_GPIO_PIN,
		CONFIG_SIFIVE_DISPLAY_ROW_2_GPIO_PIN,
		CONFIG_SIFIVE_DISPLAY_ROW_3_GPIO_PIN,
		CONFIG_SIFIVE_DISPLAY_ROW_4_GPIO_PIN};

	for(u32_t i = 0; i < hangtime; i++) {
		// Run through each ROW from the TOP to the BOTTOM
		for(u32_t row = 0; row < 5; row++) {
			// Turn off all the COLUMNS
			for(u32_t col = 0; col < 5; col++) {
				sifive_display_setcol(disp, col, 0);
			}

			// Get the chunk of bits for the row
			u32_t rowbits = (canvas >> (20 - (5 * row))) & 0x1F;

			// Turn on the selected columns
			for(u32_t col = 0; col < 5; col++) {
				if (rowbits % 2) {
					u32_t duty = brightness * CONFIG_SIFIVE_DISPLAY_PERIOD / 100;
					sifive_display_setcol(disp, col, duty);
				}
				rowbits = rowbits >> 1;
			}

			// Flash the row
			sifive_display_setrow(disp, rowary[row], 1);
			k_busy_wait(CONFIG_SIFIVE_DISPLAY_PERIOD);
			sifive_display_setrow(disp, rowary[row], 0);
		}

		// Turn off all the COLUMNS
		for(u32_t col = 0; col < 5; col++) {
			sifive_display_setcol(disp, col, 0);
		}
	}

	if(IS_ENABLED(CONFIG_SIFIVE_DISPLAY_TOGGLE_UART_0)) {
		pinmux_pin_set(disp->pinmux, CONFIG_SIFIVE_DISPLAY_UART_0_TX, SIFIVE_PINMUX_IOF0);
		pinmux_pin_set(disp->pinmux, CONFIG_SIFIVE_DISPLAY_UART_0_RX, SIFIVE_PINMUX_IOF0);
	}
	return 0;
}

void sifive_display_string(struct sifive_display *disp,
		const char *msg,
		const u32_t hangtime,
		const u32_t brightness)
{
	u32_t canvas, pixel_idx = 0;
	do {
		/* Get canvas */
		canvas = sifive_display_rstring(msg, pixel_idx);

		/* Render it to the display */
		sifive_display_canvas(disp, canvas, hangtime, brightness);

		pixel_idx++;
	} while(canvas != 0);
}

/*
 * Driver Initialization
 */
static int sifive_display_init(struct device *dev) {
	ARG_UNUSED(dev);

	/* Get device bindings */
	display.gpio = device_get_binding(CONFIG_GPIO_SIFIVE_GPIO_NAME);
	display.pwm[0] = device_get_binding(CONFIG_SIFIVE_PWM_0_LABEL);
	display.pwm[1] = device_get_binding(CONFIG_SIFIVE_PWM_1_LABEL);
	display.pinmux = device_get_binding(CONFIG_PINMUX_SIFIVE_0_NAME);
}

SYS_INIT(sifive_display_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
