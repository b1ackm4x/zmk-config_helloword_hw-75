#pragma once

#include <stdint.h>

struct lowpass_filter {
	uint32_t timestamp;
	float output_last;
	float time_constant;
};

void lpf_create(struct lowpass_filter *lpf, float time_constant);
float lpf_input(struct lowpass_filter *lpf, float input);
