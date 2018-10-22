/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <soc.h>

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
	/* TODO */
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
	/* TODO */
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
	/* TODO */
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
	/* TODO */
}

int _arch_buffer_validate(void *addr, size_t size, int write)
{
	/* TODO */
	return 1;
}

