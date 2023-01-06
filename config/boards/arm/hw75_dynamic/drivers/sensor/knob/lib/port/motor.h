#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "lowpass_filter.h"
#include "pid.h"
#include "math_utils.h"
#include "driver.h"
#include "encoder.h"

enum motor_control_mode {
	TORQUE,
	VELOCITY,
	ANGLE,
};

enum motor_error {
	NO_ERROR,
	FAILED_TO_NOTICE_MOVEMENT,
	POLE_PAIR_MISMATCH,
};

struct motor_config {
	float voltage_limit;
	float velocity_limit;
	float voltage_used_for_sensor_align;
	enum motor_control_mode control_mode;
	struct lowpass_filter lpf_velocity;
	struct lowpass_filter lpf_angle;
	struct pid_controller pid_velocity;
	struct pid_controller pid_angle;
};

struct motor_state {
	float raw_angle;
	float est_angle;
	float raw_velocity;
	float est_velocity;
};

struct motor_voltage {
	float d;
	float q;
};

struct motor {
	float target;
	enum motor_error error;
	struct motor_config config;
	struct motor_state state;
	struct motor_voltage voltage;
	float zero_electric_angle_offset;
	struct driver *driver;
	struct encoder *encoder;

	bool enabled;
	int pole_pairs;
	float voltage_a, voltage_b, voltage_c;
	float estimate_angle;
	float electrical_angle;
	float estimate_velocity;
	float set_point_voltage;
	float set_point_velocity;
	float set_point_angle;
};

void motor_create(struct motor *motor, int pole_pairs, struct driver *driver,
		  struct encoder *encoder);
bool motor_init(struct motor *motor, float zero_electric_offset,
		enum encoder_direction encoder_dir);
void motor_set_enable(struct motor *motor, bool enable);
float motor_get_estimate_angle(struct motor *motor);
float motor_get_estimate_velocity(struct motor *motor);
float motor_get_electrical_angle(struct motor *motor);
void motor_tick(struct motor *motor);
void motor_set_torque_limit(struct motor *motor, float val);
