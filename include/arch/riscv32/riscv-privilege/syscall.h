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
    /* TODO: cleanup */
    u32_t mstatus;
    __asm__ volatile (
            "csrr %[out], mstatus"
            : [out] "=r" (mstatus) ::);

    u32_t mpp = ((mstatus >> 11) & 0x3);

    if(mpp == 0)
        return 1;
    else
        return 0;
}

static inline u32_t _arch_syscall_invoke0(u32_t call_id)
{
    u32_t rc;
    __asm__ volatile (
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "ecall\n"
        "mv %[rc], a0\n"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id));
    return rc;
}

static inline u32_t _arch_syscall_invoke1(u32_t arg1, u32_t call_id)
{
    u32_t rc;
    __asm__ volatile (
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "ecall\n"
        "mv %[rc], a0\n"
        : [rc] "=r" (rc)
        : [call_id] "r" (call_id),
          [arg1] "r" (arg1));
    return rc;
}

static inline u32_t _arch_syscall_invoke2(u32_t arg1, u32_t arg2, u32_t call_id)
{
    u32_t rc;
    __asm__ volatile (
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "mv a3, %[arg2]\n"
        "ecall\n"
        "mv %[rc], a0\n"
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
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "mv a3, %[arg2]\n"
        "mv a4, %[arg3]\n"
        "ecall\n"
        "mv %[rc], a0\n"
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
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "mv a3, %[arg2]\n"
        "mv a4, %[arg3]\n"
        "mv a5, %[arg4]\n"
        "ecall\n"
        "mv %[rc], a0\n"
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
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "mv a3, %[arg2]\n"
        "mv a4, %[arg3]\n"
        "mv a5, %[arg4]\n"
        "mv a6, %[arg5]\n"
        "ecall\n"
        "mv %[rc], a0\n"
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
        "li a0, 1\n"
        "mv a1, %[call_id]\n"
        "mv a2, %[arg1]\n"
        "mv a3, %[arg2]\n"
        "mv a4, %[arg3]\n"
        "mv a5, %[arg4]\n"
        "mv a6, %[arg5]\n"
        "mv a7, %[arg6]\n"
        "ecall\n"
        "mv %[rc], a0\n"
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
