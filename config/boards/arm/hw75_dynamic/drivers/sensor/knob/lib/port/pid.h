#pragma once

#include <stdint.h>

struct pid_controller {
	float p;
	float i;
	float d;
	float output_ramp;
	float limit;
	float error_last;
	float output_last;
	float integral_last;
	uint32_t timestamp;
};

void pid_create(struct pid_controller *pid, float p, float i, float d, float ramp, float limit);
void pid_set(struct pid_controller *pid, float p, float i, float d);
float pid_error(struct pid_controller *pid, float error);
