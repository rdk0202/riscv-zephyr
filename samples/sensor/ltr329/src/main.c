/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sensor.h>
#include <misc/printk.h>

void set_cursor_pos(int y, int x)
{
	printk("\x1b[%d;%dH", y, x);
}

void draw_header(void)
{
	/* Clear screen */
	printk("\033c");

	set_cursor_pos(1, 0);
	printk("LTR329 Light Sensor Demo\n");
}

void main(void)
{
	int rc = 0;
	struct sensor_value vis, ir;
	struct device *ltr329 = device_get_binding("ltr329");

	if (ltr329 == NULL) {
		printk("Failed to get LTR329 binding!\n");
		return;
	}

	k_sleep(3000);

	while (1) {
		draw_header();

		/* Fetch sensor values */
		rc = sensor_sample_fetch(ltr329);
		if (rc != 0) {
			printk("Fetching sensor values failed with code %d\n",
			       rc);
			continue;
		}

		/* Get sensor values */
		rc = sensor_channel_get(ltr329, SENSOR_CHAN_LIGHT, &vis);
		if (rc != 0) {
			printk("Reading sensor failed %d\n", rc);
			continue;
		}

		rc = sensor_channel_get(ltr329, SENSOR_CHAN_IR, &ir);
		if (rc != 0) {
			printk("Reading sensor failed %d\n", rc);
			continue;
		}

		set_cursor_pos(2, 0);
		printk("Visible light: %6d.%06d\n", vis.val1, vis.val2);
		set_cursor_pos(3, 0);
		printk("IR light:      %6d.%06d\n", ir.val1, ir.val2);

		k_sleep(100);
	}
}

