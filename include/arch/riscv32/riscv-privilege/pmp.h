/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_PMP_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_PMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

#include <misc/util.h>

/* PMP RWX flags */
#define RISCV_PRIV_PMPCFG_R         BIT(0)
#define RISCV_PRIV_PMPCFG_W         BIT(1)
#define RISCV_PRIV_PMPCFG_X         BIT(2)

/* PMP address matching modes */
#define RISCV_PRIV_PMPCFG_A_SHIFT   3
#define RISCV_PRIV_PMPCFG_A_MASK    (BIT(3) | BIT(4))
#define RISCV_PRIV_PMPCFG_A_OFF     0
#define RISCV_PRIV_PMPCFG_A_TOR     1
#define RISCV_PRIV_PMPCFG_A_NA4     2
#define RISCV_PRIV_PMPCFG_A_NAPOT   3

/* PMP region lock */
#define RISCV_PRIV_PMPCFG_L         BIT(7)

typedef u8_t k_mem_partition_attr_t;

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_PMP_H_ */
