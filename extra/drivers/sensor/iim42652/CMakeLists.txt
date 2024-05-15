# SPDX-License-Identifier: Apache-2.0
#
# CHANGELOG:
# 2024-04-02: created IIM42652 driver based on ICM42605 driver by Takuro Tsujikawa (takuro.tsujikawa@lexxpluss.com)
# 	- changed definition from ICM42605 to IIM42652

zephyr_library()

zephyr_library_sources(iim42652.c)
zephyr_library_sources(iim42652_setup.c)
zephyr_library_sources_ifdef(CONFIG_SPI iim42652_spi.c)
zephyr_library_sources_ifdef(CONFIG_IIM42652_TRIGGER iim42652_trigger.c)
