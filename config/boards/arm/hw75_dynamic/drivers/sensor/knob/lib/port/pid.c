#include "pid.h"

#include <sys/util_macro.h>

#include <knob/time.h>

void pid_create(struct pid_controller *pid, float p, float i, float d, float ramp, float limit)
{
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->output_ramp = ramp;
	pid->limit = limit;
	pid->error_last = 0;
	pid->output_last = 0;
	pid->integral_last = 0;
	pid->timestamp = time_us();
}

void pid_set(struct pid_controller *pid, float p, float i, float d)
{
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

float pid_error(struct pid_controller *pid, float error)
{
	uint32_t time = time_us();
	float dt = (float)(time - pid->timestamp) * 1e-6f;
	// Quick fix for strange cases (micros overflow)
	if (dt <= 0 || dt > 0.5f)
		dt = 1e-3f;

	float pTerm = pid->p * error;
	float iTerm = pid->integral_last + pid->i * dt * 0.5f * (error + pid->error_last);
	iTerm = CLAMP(iTerm, -pid->limit, pid->limit);
	float dTerm = pid->d * (error - pid->error_last) / dt;

	float output = pTerm + iTerm + dTerm;
	output = CLAMP(output, -pid->limit, pid->limit);

	// If output ramp defined
	if (pid->output_ramp > 0) {
		// Limit the acceleration by ramping the output
		float outputRate = (output - pid->output_last) / dt;
		if (outputRate > pid->output_ramp)
			output = pid->output_last + pid->output_ramp * dt;
		else if (outputRate < -pid->output_ramp)
			output = pid->output_last - pid->output_ramp * dt;
	}

	pid->integral_last = iTerm;
	pid->output_last = output;
	pid->error_last = error;
	pid->timestamp = time;

	return output;
}
