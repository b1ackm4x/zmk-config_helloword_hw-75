/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#ifndef KNOB_INCLUDE_ENCODER_STATE_H_
#define KNOB_INCLUDE_ENCODER_STATE_H_

#include <device.h>
#include <stdint.h>

enum encoder_direction {
	CW = 1,
	CCW = -1,
	UNKNOWN = 0,
};

struct encoder_state {
	float angle_last;
	uint32_t angle_time;

	float velocity_last;
	uint32_t velocity_time;

	int32_t rotation_count;
	int32_t rotation_count_last;
};

void encoder_init(struct encoder_state *state, const struct device *dev);
void encoder_update(struct encoder_state *state, const struct device *dev);
float encoder_get_lap_angle(struct encoder_state *state);
float encoder_get_full_angle(struct encoder_state *state);
float encoder_get_velocity(struct encoder_state *state);

#endif /* KNOB_INCLUDE_ENCODER_STATE_H_ */
