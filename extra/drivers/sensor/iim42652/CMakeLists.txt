# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(iim42652.c)
zephyr_library_sources(iim42652_setup.c)
zephyr_library_sources_ifdef(CONFIG_SPI iim42652_spi.c)
zephyr_library_sources_ifdef(CONFIG_IIM42652_TRIGGER iim42652_trigger.c)
