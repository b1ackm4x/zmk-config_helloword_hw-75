#include "lowpass_filter.h"

#include <knob/time.h>

void lpf_create(struct lowpass_filter *lpf, float time_constant)
{
	lpf->time_constant = time_constant;
	lpf->output_last = 0.0f;
	lpf->timestamp = time_us();
}

float lpf_input(struct lowpass_filter *lpf, float input)
{
	uint32_t time = time_us();
	float dt = ((float)time - (float)lpf->timestamp) * 1e-6f;

	if (dt < 0.0f)
		dt = 1e-3f;
	else if (dt > 0.3f) {
		lpf->output_last = input;
		lpf->timestamp = time;
		return input;
	}

	float alpha = lpf->time_constant / (lpf->time_constant + dt);
	float output = alpha * lpf->output_last + (1.0f - alpha) * input;
	lpf->output_last = output;
	lpf->timestamp = time;

	return output;
}
