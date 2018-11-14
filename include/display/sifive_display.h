/*
 * Copyright (c) 2018 Sifive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SIFIVE_DISPLAY__H
#define _SIFIVE_DISPLAY__H

int sifive_display_setleds(struct device *gpio, u32_t ledpat, int hangtime, u32_t brightness);

#endif /* _SIFIVE_DISPLAY__H */

