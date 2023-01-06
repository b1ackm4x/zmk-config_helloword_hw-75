#include "motor.h"

#include <string.h>
#include <math.h>
#include <arm_math.h>

#include <knob/math.h>
#include <knob/drivers/inverter.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(port_motor, CONFIG_ZMK_LOG_LEVEL);

#define MOTOR_VOLTAGE (12.0f)

bool motor_init_foc(struct motor *motor, float zero_electric_offset,
		    enum encoder_direction sensor_direction);
bool motor_align_sensor(struct motor *motor);
void motor_close_loop_control_tick(struct motor *motor);
void motor_foc_output_tick(struct motor *motor);
void motor_set_phase_voltage(struct motor *motor, float voltage_q, float voltage_d,
			     float angle_electrical);

void motor_create(struct motor *motor, int pole_pairs, const struct device *inverter,
		  const struct device *encoder)
{
	memset(motor, 0, sizeof(struct motor));
	motor->target = 0;
	motor->error = NO_ERROR;
	motor->config.voltage_limit = MOTOR_VOLTAGE;
	motor->config.velocity_limit = 20.0f;
	motor->config.voltage_used_for_sensor_align = 1.0f;
	motor->config.control_mode = ANGLE;
	lpf_create(&motor->config.lpf_velocity, 0.1f);
	lpf_create(&motor->config.lpf_angle, 0.03f);
	pid_create(&motor->config.pid_velocity, 0.5f, 10.0f, 0.0f, 1000.0f, MOTOR_VOLTAGE);
	pid_create(&motor->config.pid_angle, 20.0f, 0, 0, 0, 20.0f);
	motor->zero_electric_angle_offset = NOT_SET;
	motor->inverter = inverter;
	motor->encoder = encoder;
	motor->enabled = false;
	motor->pole_pairs = pole_pairs;

	motor->encoder_dir = UNKNOWN;
	encoder_init(&motor->encoder_state, encoder);
}

bool motor_init(struct motor *motor, float zero_electric_offset, enum encoder_direction encoder_dir)
{
	if (motor->config.voltage_limit > MOTOR_VOLTAGE) {
		motor->config.voltage_limit = MOTOR_VOLTAGE;
	}

	if (motor->config.voltage_used_for_sensor_align > motor->config.voltage_limit) {
		motor->config.voltage_used_for_sensor_align = motor->config.voltage_limit;
	}

	motor->config.pid_velocity.limit = motor->config.velocity_limit;
	motor->config.pid_angle.limit = motor->config.velocity_limit;

	return motor_init_foc(motor, zero_electric_offset, encoder_dir);
}

void motor_set_enable(struct motor *motor, bool enable)
{
	motor->enabled = enable;
	if (enable) {
		inverter_start(motor->inverter);
	} else {
		inverter_stop(motor->inverter);
	}
}

float motor_get_estimate_angle(struct motor *motor)
{
	motor->state.raw_angle =
		(float)motor->encoder_dir * encoder_get_full_angle(&motor->encoder_state);
	motor->state.est_angle = lpf_input(&motor->config.lpf_angle, motor->state.raw_angle);

	return motor->state.est_angle;
}

float motor_get_estimate_velocity(struct motor *motor)
{
	motor->state.raw_velocity =
		(float)motor->encoder_dir * encoder_get_velocity(&motor->encoder_state);
	motor->state.est_velocity =
		lpf_input(&motor->config.lpf_velocity, motor->state.raw_velocity);

	return motor->state.est_velocity;
}

float motor_get_electrical_angle(struct motor *motor)
{
	return norm_rad((float)(motor->encoder_dir * motor->pole_pairs) *
				encoder_get_lap_angle(&motor->encoder_state) -
			motor->zero_electric_angle_offset);
}

void motor_tick(struct motor *motor)
{
	motor_close_loop_control_tick(motor);
	motor_foc_output_tick(motor);
}

bool motor_init_foc(struct motor *motor, float zero_electric_offset,
		    enum encoder_direction sensor_direction)
{
	if (ASSERT(zero_electric_offset)) {
		motor->zero_electric_angle_offset = zero_electric_offset;
		motor->encoder_dir = sensor_direction;
	}

	if (!motor_align_sensor(motor))
		return false;

	encoder_update(&motor->encoder_state, motor->encoder);
	motor->estimate_angle = motor_get_estimate_angle(motor);

	return true;
}

bool motor_align_sensor(struct motor *motor)
{
	// TODO
	motor->zero_electric_angle_offset = 0;
	motor->encoder_dir = CW;
	return true;
}

void motor_close_loop_control_tick(struct motor *motor)
{
	motor->estimate_angle = motor_get_estimate_angle(motor);
	motor->estimate_velocity = motor_get_estimate_velocity(motor);

	if (!motor->enabled)
		return;

	switch (motor->config.control_mode) {
	case TORQUE:
		motor->voltage.q = motor->target;
		motor->voltage.d = 0;
		motor->set_point_voltage = motor->voltage.q;
		break;
	case ANGLE:
		motor->set_point_angle = motor->target;
		motor->set_point_velocity = pid_error(
			&motor->config.pid_angle, motor->set_point_angle - motor->estimate_angle);
		motor->set_point_voltage =
			pid_error(&motor->config.pid_velocity,
				  motor->set_point_velocity - motor->estimate_velocity);
		break;
	case VELOCITY:
		motor->set_point_velocity = motor->target;
		motor->set_point_voltage =
			pid_error(&motor->config.pid_velocity,
				  motor->set_point_velocity - motor->estimate_velocity);
		break;
	}
}

void motor_foc_output_tick(struct motor *motor)
{
	encoder_update(&motor->encoder_state, motor->encoder);

	if (!motor->enabled)
		return;

	motor->electrical_angle = motor_get_electrical_angle(motor);

	motor->voltage.q = motor->set_point_voltage;
	motor->voltage.d = 0;

	motor_set_phase_voltage(motor, motor->voltage.q, motor->voltage.d, motor->electrical_angle);
}

void motor_set_phase_voltage(struct motor *motor, float voltage_q, float voltage_d,
			     float angle_electrical)
{
	float uOut;

	if (voltage_d != 0) {
		float mod;
		arm_sqrt_f32(voltage_d * voltage_d + voltage_q * voltage_q, &mod);
		uOut = mod / MOTOR_VOLTAGE;
		angle_electrical = norm_rad(angle_electrical + atan2(voltage_q, voltage_d));
	} else {
		uOut = voltage_q / MOTOR_VOLTAGE;
		angle_electrical = norm_rad(angle_electrical + PI_2);
	}
	uint8_t sec = (int)(floor(angle_electrical / PI_3)) + 1;
	float t1 = SQRT3 * arm_sin_f32(((float)(sec)) * PI_3 - angle_electrical) * uOut;
	float t2 = SQRT3 * arm_sin_f32(angle_electrical - (((float)(sec)) - 1.0f) * PI_3) * uOut;
	float t0 = 1 - t1 - t2;

	float tA, tB, tC;
	switch (sec) {
	case 1:
		tA = t1 + t2 + t0 / 2.0f;
		tB = t2 + t0 / 2.0f;
		tC = t0 / 2.0f;
		break;
	case 2:
		tA = t1 + t0 / 2.0f;
		tB = t1 + t2 + t0 / 2.0f;
		tC = t0 / 2.0f;
		break;
	case 3:
		tA = t0 / 2.0f;
		tB = t1 + t2 + t0 / 2.0f;
		tC = t2 + t0 / 2.0f;
		break;
	case 4:
		tA = t0 / 2.0f;
		tB = t1 + t0 / 2.0f;
		tC = t1 + t2 + t0 / 2.0f;
		break;
	case 5:
		tA = t2 + t0 / 2.0f;
		tB = t0 / 2.0f;
		tC = t1 + t2 + t0 / 2.0f;
		break;
	case 6:
		tA = t1 + t2 + t0 / 2.0f;
		tB = t0 / 2.0f;
		tC = t1 + t0 / 2.0f;
		break;
	default:
		tA = 0.0f;
		tB = 0.0f;
		tC = 0.0f;
	}

	inverter_set_powers(motor->inverter, tA, tB, tC);
}

void motor_set_torque_limit(struct motor *motor, float val)
{
	motor->config.voltage_limit = val;

	if (motor->config.voltage_limit > MOTOR_VOLTAGE) {
		motor->config.voltage_limit = MOTOR_VOLTAGE;
	}

	if (motor->config.voltage_used_for_sensor_align > motor->config.voltage_limit) {
		motor->config.voltage_used_for_sensor_align = motor->config.voltage_limit;
	}

	motor->config.pid_velocity.limit = motor->config.voltage_limit;
	motor->config.pid_angle.limit = motor->config.voltage_limit;
}
