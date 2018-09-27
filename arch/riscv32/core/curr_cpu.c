/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <kernel_structs.h>

_cpu_t *_arch_curr_cpu(void)
{
	/* Read CSR mhartid */
	u32_t hart_id;

	__asm__ volatile("csrr %0, mhartid" : "=r" (hart_id));

	return &_kernel.cpus[hart_id];
}

