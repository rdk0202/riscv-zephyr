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

	set_cursor_pos(0, 0);
	printk("                 Zephyr LSM303C Accelerometer and Magnetometer Demo\n\n");
	printk(" Accel X   | Accel Y    | Accel Z    | Mag X      | Mag Y      | Mag Z      | Temp\n");
	printk("------------------------------------------------------------------------------------------\n\n");
}

void main(void)
{
	int rc = 0;
	struct sensor_value accel[3];
	struct sensor_value magn[3];
	struct sensor_value temp;
	int redraw_header_count = 0;
	struct device *lsm303c = device_get_binding("lsm303c");

	if (lsm303c == NULL) {
		printk("Failed to get LSM303C binding!\n");
		return;
	}

	k_sleep(3000);

	draw_header();

	if (lsm303c == NULL) {
		printk("Failed to get LSM303C binding!\n");
		return;
	}

	while (1) {
		/* Redraw header periodically */
		if (redraw_header_count >= 200) {
			draw_header();
			redraw_header_count = 0;
		}
		redraw_header_count += 1;

		set_cursor_pos(5, 0);

		/* Fetch sensor values */
		rc = sensor_sample_fetch(lsm303c);
		if (rc != 0) {
			printk("Fetching sensor values failed with code %d\n",
			       rc);
			continue;
		}

		/* Get sensor values */
		rc = sensor_channel_get(lsm303c, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (rc != 0) {
			printk("Reading sensor failed %d\n", rc);
			continue;
		}

		rc = sensor_channel_get(lsm303c, SENSOR_CHAN_MAGN_XYZ, magn);
		if (rc != 0) {
			printk("Reading sensor failed %d\n", rc);
			continue;
		}

		rc = sensor_channel_get(lsm303c, SENSOR_CHAN_DIE_TEMP, &temp);
		if (rc != 0) {
			printk("Reading sensor failed %d\n", rc);
			continue;
		}

		for (int i = 0; i < 3; i++) {
			printk("%3d.%06d | ", accel[i].val1, accel[i].val2);
		}

		for (int i = 0; i < 3; i++) {
			printk("%3d.%06d | ", magn[i].val1, magn[i].val2);
		}

		printk("%3d.%06d", temp.val1, temp.val2);

		/* Short sleep to deflicker */
		k_sleep(10);
	}
}

