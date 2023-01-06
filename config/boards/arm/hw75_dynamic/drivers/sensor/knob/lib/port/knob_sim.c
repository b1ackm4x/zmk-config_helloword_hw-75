#include "knob_sim.h"

#include "pid.h"
#include "motor.h"
#include "math_utils.h"

#include <math.h>

#include <knob/math.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(port_knob, CONFIG_ZMK_LOG_LEVEL);

void knob_sim_create(struct knob_sim *knob)
{
	knob->mode = MODE_DISABLE;
	knob->zero_position = 0.0f;
	knob->limit_position_max = 5.1f;
	knob->limit_position_min = 3.3f;
	knob->encoder_divides = 12;
	knob->encoder_position = 0;
	knob->last_angle = 0.0f;
	knob->last_velocity = 0.0f;
}

void knob_sim_init(struct knob_sim *knob, struct motor *motor)
{
	knob->motor = motor;
	motor->config.control_mode = TORQUE;
	motor->config.voltage_limit = 1.5f;
	motor->config.velocity_limit = 100.0f;
	pid_set(&motor->config.pid_velocity, 0.1f, 0.0f, 0.0f);
	pid_set(&motor->config.pid_angle, 80.0f, 0.0f, 0.7f);

	if (motor_init(motor, NOT_SET, CW)) {
		LOG_INF("ZeroElectricAngleOffset: %d | Encoder direction: %d",
			(int)motor->zero_electric_angle_offset,
			motor->encoder->count_direction == CW);
		motor->target = 0;
		motor_set_enable(motor, false);
	} else {
		LOG_ERR("Error[%d]", motor->error);
	}
}

void knob_sim_set_mode(struct knob_sim *knob, enum knob_sim_mode mode)
{
	knob->mode = mode;

	knob->last_angle = knob_sim_get_position(knob);
	knob->last_velocity = knob_sim_get_velocity(knob);

	switch (mode) {
	case MODE_DISABLE:
		motor_set_enable(knob->motor, false);
		break;
	case MODE_INERTIA: {
		motor_set_enable(knob->motor, true);
		motor_set_torque_limit(knob->motor, 0.5f);
		knob->motor->config.control_mode = VELOCITY;
		pid_set(&knob->motor->config.pid_velocity, 0.1f, 0.0f, 0.0f);
		pid_set(&knob->motor->config.pid_angle, 20.0f, 0.0f, 0.7f);
	} break;
	case MODE_ENCODER: {
		motor_set_enable(knob->motor, true);
		motor_set_torque_limit(knob->motor, 1.5f);
		knob->motor->config.control_mode = ANGLE;
		pid_set(&knob->motor->config.pid_velocity, 0.1f, 0.0f, 0.0f);
		pid_set(&knob->motor->config.pid_angle, 10.0f, 0.0f, 3.5f);
		knob->motor->target = 4.2f;
		knob->last_angle = 4.2f;
	} break;
	case MODE_SPRING: {
		motor_set_enable(knob->motor, true);
		motor_set_torque_limit(knob->motor, 1.5f);
		knob->motor->config.control_mode = ANGLE;
		pid_set(&knob->motor->config.pid_velocity, 0.1f, 0.0f, 0.0f);
		pid_set(&knob->motor->config.pid_angle, 100.0f, 0.0f, 3.5f);
		knob->motor->target = 4.2f;
	} break;
	case MODE_DAMPED: {
		motor_set_enable(knob->motor, true);
		motor_set_torque_limit(knob->motor, 1.5f);
		knob->motor->config.control_mode = VELOCITY;
		pid_set(&knob->motor->config.pid_velocity, 0.1f, 0.0f, 0.0f);
		knob->motor->target = 0.0f;
	} break;
	case MODE_SPIN: {
		motor_set_enable(knob->motor, true);
		motor_set_torque_limit(knob->motor, 1.5f);
		knob->motor->config.control_mode = VELOCITY;
		pid_set(&knob->motor->config.pid_velocity, 0.3f, 0.0f, 0.0f);
		knob->motor->target = 20.0f;
	} break;
	}
}

void knob_sim_tick(struct knob_sim *knob)
{
	switch (knob->mode) {
	case MODE_INERTIA: {
		float v = knob_sim_get_velocity(knob);
		if (v > 1.0f || v < -1.0f) {
			if (fabsf(v - knob->last_velocity) > 0.3f)
				knob->motor->target = v;
		} else {
			knob->motor->target = 0.0f;
		}
		knob->last_velocity = v;
	} break;
	case MODE_ENCODER: {
		float a = knob_sim_get_position(knob);
		if (a - knob->last_angle > PI / (float)knob->encoder_divides) {
			knob->motor->target += PI2 / (float)knob->encoder_divides;
			knob->last_angle = knob->motor->target;
			knob->encoder_position++;
		} else if (a - knob->last_angle < -PI / (float)knob->encoder_divides) {
			knob->motor->target -= PI2 / (float)knob->encoder_divides;
			knob->last_angle = knob->motor->target;
			knob->encoder_position--;
		}
	} break;
	case MODE_DAMPED: {
		if (knob->limit_position_max != 0 && knob->limit_position_min != 0) {
			float a = knob_sim_get_position(knob);
			if (a > knob->limit_position_max) {
				knob->motor->config.control_mode = ANGLE;
				knob->motor->target = knob->limit_position_max;
			} else if (a < knob->limit_position_min) {
				knob->motor->config.control_mode = ANGLE;
				knob->motor->target = knob->limit_position_min;
			} else {
				knob->motor->config.control_mode = VELOCITY;
				knob->motor->target = 0;
			}
		}
	} break;
	case MODE_DISABLE:
	case MODE_SPRING:
	case MODE_SPIN:
		break;
	}

	motor_tick(knob->motor);
}

void knob_sim_set_limit_pos(struct knob_sim *knob, float min, float max)
{
	knob->limit_position_min = min;
	knob->limit_position_max = max;
}

void knob_sim_apply_zero_pos(struct knob_sim *knob, float angle)
{
	if (angle != 0.0f) {
		knob->zero_position = angle;
	} else {
		knob->zero_position = motor_get_estimate_angle(knob->motor);
	}
}

float knob_sim_get_position(struct knob_sim *knob)
{
	return motor_get_estimate_angle(knob->motor) - knob->zero_position;
}

float knob_sim_get_velocity(struct knob_sim *knob)
{
	return motor_get_estimate_velocity(knob->motor);
}

int knob_sim_get_encoder_mode_pos(struct knob_sim *knob)
{
	return lround(knob_sim_get_position(knob) / (PI2 / (float)knob->encoder_divides));
}

void knob_sim_set_enable(struct knob_sim *knob, bool en)
{
	motor_set_enable(knob->motor, en);
}
