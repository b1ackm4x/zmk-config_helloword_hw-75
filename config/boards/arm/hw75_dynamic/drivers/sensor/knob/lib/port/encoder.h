#pragma once

#include <device.h>
#include <stdint.h>

enum encoder_direction {
	CW = 1,
	CCW = -1,
	UNKNOWN = 0,
};

struct encoder {
	const struct device *dev;
	enum encoder_direction count_direction;
	float angle_last;
	uint32_t angle_timestamp;
	float velocity_last;
	uint32_t velocity_timestamp;
	int32_t rotation_count;
	int32_t rotation_count_last;
};

void encoder_create(struct encoder *encoder, const struct device *dev);
void encoder_init(struct encoder *encoder);
float encoder_get_raw_angle(struct encoder *encoder);
void encoder_update(struct encoder *encoder);
float encoder_get_lap_angle(struct encoder *encoder);
float encoder_get_full_angle(struct encoder *encoder);
float encoder_get_velocity(struct encoder *encoder);
int32_t encoder_rotation_count(struct encoder *encoder);
