zephyr_library()
zephyr_library_sources(pinmux.c)
zephyr_library_sources(${ZEPHYR_BASE}/soc/riscv32/riscv-privilege/sifive-freedom/fe310_clock.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
