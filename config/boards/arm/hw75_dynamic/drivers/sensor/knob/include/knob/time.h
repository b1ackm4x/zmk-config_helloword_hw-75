/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#ifndef KNOB_INCLUDE_TIME_H_
#define KNOB_INCLUDE_TIME_H_

#include <stdint.h>
#include <kernel.h>

static inline uint32_t time_us()
{
	return k_cyc_to_us_floor32(k_cycle_get_32());
}

#endif /* KNOB_INCLUDE_TIME_H_ */
