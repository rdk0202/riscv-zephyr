zephyr_include_directories(.)

zephyr_sources(
  idle.c
  soc_irq.S
  soc_common_irq.c
  )

zephyr_sources_ifdef(CONFIG_USERSPACE   pmp.c)

if(NOT(CONFIG_SOC_SERIES_RISCV32_QEMU))
  zephyr_sources(vector.S)
endif()
