zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LSM303C lsm303c.c)
