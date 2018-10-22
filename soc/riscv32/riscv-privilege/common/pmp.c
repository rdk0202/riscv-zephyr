/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <soc.h>
#include <arch/riscv32/riscv-privilege/pmp.h>

/**
 * @brief Get the maximum number of partitions for a memory domain
 *
 * A memory domain is a container data structure containing some number of
 * memory partitions, where each partition represents a memory range with
 * access policies.
 *
 * MMU-based systems don't have a limit here, but MPU-based systems will
 * have an upper bound on how many different regions they can manage
 * simultaneously.
 *
 * @return Max number of free regions, or -1 if there is no limit
 */
int _arch_mem_domain_max_partitions_get(void)
{
	/* We use two regions per partition because we want to use top-of-region
	 * (TOR) mode. */
	return CONFIG_RISCV_PMP_NUM_ENTRIES / 2;
}

void set_pmpcfg(u8_t region, u32_t attr)
{
	u8_t cfg_num = region / 4;
	u8_t subcfg_num = region % 4;

	/* All regions are readable if configured and use TOR addressing */
	u32_t cfg_value = RISCV_PRIV_PMPCFG_R | RISCV_PRIV_PMPCFG_A_TOR;

	/* If the region is writeable, set the write bit */
	/* TODO */

	/* If the region is executable, set the execute bit */
	/* TODO */

	/* Shift the config to the proper pmpcfg CSR offset */
	cfg_value = cfg_value << (sizeof(u8_t) * subcfg_num);
	u32_t cfg_mask = 0xFF << (sizeof(u8_t) * subcfg_num);

	/* Write the config to the proper mpmcfg0 */
	u32_t pmpcfg;
	switch(cfg_num)
	{
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 0
	case 0:
		__asm__ volatile("csrr %[oldcfg], pmpcfg0"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		pmpcfg |= cfg_value;
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 5
	case 1:
		__asm__ volatile("csrr %[oldcfg], pmpcfg1"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		pmpcfg |= cfg_value;
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 9
	case 2:
		__asm__ volatile("csrr %[oldcfg], pmpcfg2"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		pmpcfg |= cfg_value;
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 13
	case 3:
		__asm__ volatile("csrr %[oldcfg], pmpcfg3"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		pmpcfg |= cfg_value;
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
	default:
		return;
	}
}

void clr_pmpcfg(u8_t region)
{
	u8_t cfg_num = region / 4;
	u8_t subcfg_num = region % 4;

	u32_t cfg_mask = 0xFF << (sizeof(u8_t) * subcfg_num);

	/* Write the config to the proper mpmcfg0 */
	u32_t pmpcfg;
	switch(cfg_num)
	{
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 0
	case 0:
		__asm__ volatile("csrr %[oldcfg], pmpcfg0"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 5
	case 1:
		__asm__ volatile("csrr %[oldcfg], pmpcfg1"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 9
	case 2:
		__asm__ volatile("csrr %[oldcfg], pmpcfg2"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES >= 13
	case 3:
		__asm__ volatile("csrr %[oldcfg], pmpcfg3"
				: [oldcfg] "=r" (pmpcfg));
		pmpcfg &= ~(cfg_mask);
		__asm__ volatile("csrw pmpcfg0, %[cfg]"
				:: [cfg] "r" (pmpcfg));
#endif
	default:
		return;
	}
}

void set_pmpaddr(u8_t region, u32_t addr)
{
	switch(region)
	{
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 0
	case 0:
		__asm__ volatile("csrw pmpaddr0, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 1
	case 1:
		__asm__ volatile("csrw pmpaddr1, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 2
	case 2:
		__asm__ volatile("csrw pmpaddr2, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 3
	case 3:
		__asm__ volatile("csrw pmpaddr3, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 4
	case 4:
		__asm__ volatile("csrw pmpaddr4, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 5
	case 5:
		__asm__ volatile("csrw pmpaddr5, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 6
	case 6:
		__asm__ volatile("csrw pmpaddr6, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 7
	case 7:
		__asm__ volatile("csrw pmpaddr7, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 8
	case 8:
		__asm__ volatile("csrw pmpaddr8, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 9
	case 9:
		__asm__ volatile("csrw pmpaddr9, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 10
	case 10:
		__asm__ volatile("csrw pmpaddr10, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 11
	case 11:
		__asm__ volatile("csrw pmpaddr11, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 12
	case 12:
		__asm__ volatile("csrw pmpaddr12, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 13
	case 13:
		__asm__ volatile("csrw pmpadd13, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 14
	case 14:
		__asm__ volatile("csrw pmpaddr14, %[addr]"
				:: [addr] "r" (addr));
#endif
#if CONFIG_RISCV_PMP_NUM_ENTRIES > 15
	case 15:
		__asm__ volatile("csrw pmpaddr15, %[addr]"
				:: [addr] "r" (addr));
#endif
	default:
		return;
	}
}

int configure_pmp(u8_t partition, u32_t start, u32_t end, u32_t attr)
{
	if(partition >= _arch_mem_domain_max_partitions_get())
		return 1;

	u8_t lower_region = partition * 2;
	u8_t upper_region = partition * 2 + 1;

	/* Set lower bound */
	set_pmpaddr(lower_region, start);

	/* Set upper bound */
	set_pmpaddr(upper_region, end);

	/* Configure region */
	set_pmpcfg(upper_region, attr);

	return 0;
}

/**
 * @brief Configure the memory domain of the thread.
 *
 * A memory domain is a container data structure containing some number of
 * memory partitions, where each partition represents a memory range with
 * access policies. This api will configure the appropriate hardware
 * registers to make it work.
 *
 * @param thread Thread which needs to be configured.
 */
void _arch_mem_domain_configure(struct k_thread *thread)
{
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;

	for(int i = 0; i < domain->num_partitions; i++)
	{
		struct k_mem_partition part = domain->partitions[i];

		configure_pmp(i, part.start, part.start + part.size, part.attr);
	}
}

/**
 * @brief Remove a partition from the memory domain
 *
 * A memory domain contains multiple partitions and this API provides the
 * freedom to remove a particular partition while keeping others intact.
 * This API will handle any arch/HW specific changes that needs to be done.
 *
 * @param domain The memory domain structure
 * @param partition_id The partition that needs to be deleted
 */
void _arch_mem_domain_partition_remove(struct k_mem_domain *domain,
		u32_t partition_id)
{
	clr_pmpcfg(partition_id * 2 + 1);
}

/**
 * @brief Remove the memory domain
 *
 * A memory domain contains multiple partitions and this API will traverse
 * all these to reset them back to default setting.
 * This API will handle any arch/HW specific changes that needs to be done.
 *
 * @param domain The memory domain structure which needs to be deleted.
 */
void _arch_mem_domain_destroy(struct k_mem_domain *domain)
{
	for(int i = 0; i < domain->num_partitions; i++)
	{
		clr_pmpcfg(i * 2 + 1);
	}
}

int _arch_buffer_validate(void *addr, size_t size, int write)
{
	/* TODO */
	return 0;
}

