/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SIFIVE_DISPLAY__H
#define _SIFIVE_DISPLAY__H

#include <zephyr.h>

extern const u32_t sifive_font[];

struct sifive_display {
	/* GPIO, PWM, and Pinmux devices */
	struct device *gpio;
	struct device *pwm[2];
	struct device *pinmux;
};

/* Get a handle to the display device */
struct sifive_display *sifive_display_get(void);

/* Render a canvas to the display for a hangtime multiple of
 * CONFIG_SIFIVE_DISPLAY_PERIOD with a brightness out of 100 */
int sifive_display_canvas(struct sifive_display *disp,
		const u32_t canvas,
		const u32_t hangtime,
		const u32_t brightness);

/* Scroll a string across the display, advancing a pixel after a hangtime
 * multiple of CONFIG_SIFIVE_DISPLAY_PERIOD with a brightness out of 100 */
void sifive_display_string(struct sifive_display *disp,
		const char *msg,
		const u32_t hangtime,
		const u32_t brightness);

#endif /* _SIFIVE_DISPLAY__H */

