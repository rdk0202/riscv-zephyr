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
    u32_t mstatus;
    __asm__ volatile (
            "csrr %[out], mstatus"
            : [out] "=r" (mstatus));
    /* TODO */
    return 0;
}

static inline u32_t _arch_syscall_invoke0(u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id));
    return rc;
}

static inline u32_t _arch_syscall_invoke1(u32_t arg1, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1));
    return rc;
}

static inline u32_t _arch_syscall_invoke2(u32_t arg1, u32_t arg2, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "mv a3, %[arg2]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2));
    return rc;
}

static inline u32_t _arch_syscall_invoke3(u32_t arg1, u32_t arg2, u32_t arg3, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "mv a3, %[arg2]"
        "mv a4, %[arg3]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2),
          [arg3] "r" (arg3));
    return rc;
}

static inline u32_t _arch_syscall_invoke4(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "mv a3, %[arg2]"
        "mv a4, %[arg3]"
        "mv a5, %[arg4]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2),
          [arg3] "r" (arg3),
          [arg4] "r" (arg4));
    return rc;
}

static inline u32_t _arch_syscall_invoke5(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t arg5, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "mv a3, %[arg2]"
        "mv a4, %[arg3]"
        "mv a5, %[arg4]"
        "mv a6, %[arg5]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2),
          [arg3] "r" (arg3),
          [arg4] "r" (arg4),
          [arg5] "r" (arg5));
    return rc;
}

static inline u32_t _arch_syscall_invoke6(u32_t arg1, u32_t arg2, u32_t arg3, u32_t arg4, u32_t arg5, u32_t arg6, u32_t call_id)
{
	u32_t rc;
    __asm__ volatile (
        "li a0, 1"
        "mv a1, %[call_id]"
        "mv a2, %[arg1]"
        "mv a3, %[arg2]"
        "mv a4, %[arg3]"
        "mv a5, %[arg4]"
        "mv a6, %[arg5]"
        "mv a7, %[arg6]"
        "ecall"
        "mv %[rc], a0"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2),
          [arg3] "r" (arg3),
          [arg4] "r" (arg4),
          [arg5] "r" (arg5),
          [arg6] "r" (arg6));
    return rc;
}

#endif /* _ASMLANGUAGE */
#endif /* CONFIG_USERSPACE */

#endif /* ZEPHYR_INCLUDE_ARCH_RISCV32_RISCV_PRIVILEGE_SYSCALL_H_ */
