/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_SYSCALL_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_SYSCALL_H_

#ifdef CONFIG_USERSPACE
#ifndef _ASMLANGUAGE

static inline int _arch_is_user_context(void)
{
    /* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke0(u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke1(u32_t arg1, u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke2(u32_t arg1, u32_t arg2, u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke3(u32_t arg1, u32_t arg2, u32_t arg3, u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke4(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke5(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t arg5, u32_t call_id)
{
	/* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke6(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t arg5, u32_t arg6, u32_t call_id)
{
	/* TODO */
    return 0;
}

#endif /* _ASMLANGUAGE */
#endif /* CONFIG_USERSPACE */

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_SYSCALL_H_ */
