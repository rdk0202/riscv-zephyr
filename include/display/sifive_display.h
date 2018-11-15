/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SIFIVE_DISPLAY__H
#define _SIFIVE_DISPLAY__H

#include <zephyr.h>

extern const u32_t sifive_font[];

int sifive_display_setleds(u32_t ledpat, u32_t hangtime, u32_t brightness);
void sifive_display_string(const char *msg, u32_t hangtime, u32_t brightness);

#endif /* _SIFIVE_DISPLAY__H */

