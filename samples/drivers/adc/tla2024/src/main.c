/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <adc.h>
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
	printk("TLA2024 ADC Demo\n");
	printk("Channel | Value\n");
	printk("---------------------------\n");
}

void print_adc(s16_t value)
{
	/* At a FSR of 6.144 V, the LSB is equal to 3 mV */
	s32_t mvolts = (value >> 4) * 3;

	printk("%5d mV", mvolts);
}

void main(void)
{
	int rc = 0;
	s16_t buffer[4];
	int redraw_count = 0;
	struct device *tla2024 = device_get_binding("tla2024");

	if (tla2024 == NULL) {
		printk("Failed to get TLA2024 binding!\n");
		return;
	}

	k_sleep(3000);

	struct adc_sequence seq = {
		/* Read channels 4-7 */
		.channels = 0xF0,
		.buffer = (void *) buffer,
		.buffer_size = sizeof(buffer),
	};

	draw_header();

	while (1) {
		if (redraw_count >= 100) {
			draw_header();
			redraw_count = 0;
		}
		redraw_count += 1;

		/* Read ADC */
		rc = adc_read(tla2024, &seq);
		if (rc < 0) {
			printk("Failed to read ADC");
		}

		/* Display ADC values */
		set_cursor_pos(4, 0);

		printk("AIN0:     ");
		print_adc(buffer[0]);
		printk("\n");

		printk("AIN1:     ");
		print_adc(buffer[1]);
		printk("\n");

		printk("AIN2:     ");
		print_adc(buffer[2]);
		printk("\n");

		printk("AIN3:     ");
		print_adc(buffer[3]);
		printk("\n");
	}
}

