# Copyright (c) 2022-2023 XiNGRZ
# SPDX-License-Identifier: MIT

zephyr_library_sources(knob.c)
zephyr_library_sources(encoder_state.c)

add_subdirectory_ifdef(CONFIG_KNOB_PORT port)
