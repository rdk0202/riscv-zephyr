/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <gpio.h>
#include <pwm.h>

#define LED_WAIT_US	1000
#define PERIOD LED_WAIT_US

#define LED_COL_4	1
#define LED_COL_3	2
#define LED_COL_2	19
#define LED_COL_1	21
#define LED_COL_0	22

#define LED_ROW_4	16
#define LED_ROW_3	17
#define LED_ROW_2	18
#define LED_ROW_1	20
#define LED_ROW_0	23

static struct device *pwm0;
static struct device *pwm1;

int sifive_display_setcol(u32_t col, u32_t duty) {
	if(!pwm0)
		pwm0 = device_get_binding("pwm_0");
	if(!pwm1)
		pwm1 = device_get_binding("pwm_1");

	int rc = 0;

	switch(col) {
	case 0:
		rc = pwm_pin_set_cycles(pwm1, 3, PERIOD, duty);
		break;
	case 1:
		rc = pwm_pin_set_cycles(pwm1, 2, PERIOD, duty);
		break;
	case 2:
		rc = pwm_pin_set_cycles(pwm1, 1, PERIOD, duty);
		break;
	case 3:
		rc = pwm_pin_set_cycles(pwm0, 2, PERIOD, duty);
		break;
	case 4:
		rc = pwm_pin_set_cycles(pwm0, 1, PERIOD, duty);
		break;
	}

	return rc;
}

int sifive_display_setleds(struct device *gpio, u32_t ledpat, int hangtime, u32_t brightness) {
	const u32_t rowary[] = {LED_ROW_0, LED_ROW_1, LED_ROW_2, LED_ROW_3, LED_ROW_4};  

	for(u32_t i = 0; i < hangtime; i++) {
		// Run through each ROW from the TOP to the BOTTOM
		for(u32_t row = 0; row < 5; row++) {
			// Turn off all the COLUMNS
			for(u32_t col = 0; col < 5; col++) {
				sifive_display_setcol(col, 0);
			}

			// Get the chunk of bits for the row
			u32_t rowbits = (ledpat >> (20 - (5 * row))) & 0x1F;

			// Turn on the selected columns
			for(u32_t col = 0; col < 5; col++) {
				if (rowbits % 2)
					sifive_display_setcol(col, brightness * PERIOD / 100);
				rowbits = rowbits >> 1;
			}

			// Flash the row
			gpio_pin_write(gpio, rowary[row], 1);
			k_busy_wait(LED_WAIT_US);
			gpio_pin_write(gpio, rowary[row], 0);
		}

		// Turn off all the COLUMNS
		for(u32_t col = 0; col < 5; col++) {
			sifive_display_setcol(col, 0);
		}
	}
	return 0;
}

