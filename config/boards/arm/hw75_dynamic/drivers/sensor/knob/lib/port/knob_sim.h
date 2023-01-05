#pragma once

#include "motor.h"

#include <stdbool.h>

enum knob_sim_mode {
	MODE_DISABLE = 0,
	MODE_INERTIA,
	MODE_ENCODER,
	MODE_SPRING,
	MODE_DAMPED,
	MODE_SPIN,
};

struct knob_sim {
	struct motor *motor;
	enum knob_sim_mode mode;
	float zero_position;
	float limit_position_max;
	float limit_position_min;
	int encoder_divides;
	int encoder_position;
	float last_angle;
	float last_velocity;
};

void knob_sim_create(struct knob_sim *knob);
void knob_sim_init(struct knob_sim *knob, struct motor *motor);
void knob_sim_tick(struct knob_sim *knob);
void knob_sim_set_enable(struct knob_sim *knob, bool en);
void knob_sim_apply_zero_pos(struct knob_sim *knob, float angle);
void knob_sim_set_mode(struct knob_sim *knob, enum knob_sim_mode mode);
void knob_sim_set_limit_pos(struct knob_sim *knob, float min, float max);
float knob_sim_get_position(struct knob_sim *knob);
float knob_sim_get_velocity(struct knob_sim *knob);
int knob_sim_get_encoder_mode_pos(struct knob_sim *knob);
