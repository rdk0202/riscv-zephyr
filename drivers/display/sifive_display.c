/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <gpio.h>
#include <pwm.h>

#define LED_ROW_4	16
#define LED_ROW_3	17
#define LED_ROW_2	18
#define LED_ROW_1	20
#define LED_ROW_0	23

/* Turn on column col for duty cycles out of CONFIG_SIFIVE_DISPLAY_PERIOD cycles */
int sifive_display_setcol(u32_t col, u32_t duty) {
	static struct device *pwm[2];
	if(!pwm[0])
		pwm[0] = device_get_binding(CONFIG_SIFIVE_PWM_0_LABEL);
	if(!pwm[1])
		pwm[1] = device_get_binding(CONFIG_SIFIVE_PWM_1_LABEL);

	int rc = 0;

	switch(col) {
	case 0:
		rc = pwm_pin_set_cycles(pwm[CONFIG_SIFIVE_DISPLAY_COL_0_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_0_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 1:
		rc = pwm_pin_set_cycles(pwm[CONFIG_SIFIVE_DISPLAY_COL_1_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_1_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 2:
		rc = pwm_pin_set_cycles(pwm[CONFIG_SIFIVE_DISPLAY_COL_2_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_2_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 3:
		rc = pwm_pin_set_cycles(pwm[CONFIG_SIFIVE_DISPLAY_COL_3_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_3_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	case 4:
		rc = pwm_pin_set_cycles(pwm[CONFIG_SIFIVE_DISPLAY_COL_4_PWM_DEV],
				CONFIG_SIFIVE_DISPLAY_COL_4_PWM_CHAN,
				CONFIG_SIFIVE_DISPLAY_PERIOD, duty);
		break;
	}

	return rc;
}

int sifive_display_setleds(struct device *gpio,
		u32_t ledpat,
		u32_t hangtime,
		u32_t brightness)
{
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
				sifive_display_setcol(col, 0);
			}

			// Get the chunk of bits for the row
			u32_t rowbits = (ledpat >> (20 - (5 * row))) & 0x1F;

			// Turn on the selected columns
			for(u32_t col = 0; col < 5; col++) {
				if (rowbits % 2) {
					u32_t duty = brightness * CONFIG_SIFIVE_DISPLAY_PERIOD / 100;
					sifive_display_setcol(col, duty);
				}
				rowbits = rowbits >> 1;
			}

			// Flash the row
			gpio_pin_write(gpio, rowary[row], 1);
			k_busy_wait(CONFIG_SIFIVE_DISPLAY_PERIOD);
			gpio_pin_write(gpio, rowary[row], 0);
		}

		// Turn off all the COLUMNS
		for(u32_t col = 0; col < 5; col++) {
			sifive_display_setcol(col, 0);
		}
	}
	return 0;
}

