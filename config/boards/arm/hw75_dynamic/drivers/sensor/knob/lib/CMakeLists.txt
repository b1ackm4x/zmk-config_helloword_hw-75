# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_library_sources(knob.c)

add_subdirectory_ifdef(CONFIG_KNOB_PORT port)
