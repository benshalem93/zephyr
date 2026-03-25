# Kionix KX132-1211 3-axis accelerometer driver
#
# Copyright (c) 2024 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(kx132.c)
zephyr_library_sources_ifdef(CONFIG_KX132_TRIGGER kx132_trigger.c)
