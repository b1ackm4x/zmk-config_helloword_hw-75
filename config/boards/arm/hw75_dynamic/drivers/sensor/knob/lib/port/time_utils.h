#pragma once

#include <stdint.h>
#include <kernel.h>

static inline uint32_t micros()
{
	return k_cyc_to_us_floor32(k_cycle_get_32());
}

static inline uint32_t millis()
{
	return k_cyc_to_ms_floor32(k_cycle_get_32());
}

static inline void delay_micro_seconds(uint32_t us)
{
	k_usleep(us);
}
