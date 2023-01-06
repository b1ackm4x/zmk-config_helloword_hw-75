#include "encoder.h"
#include "math_utils.h"

#include <kernel.h>

#include <knob/time.h>
#include <knob/math.h>
#include <knob/drivers/encoder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(port_encoder, CONFIG_ZMK_LOG_LEVEL);

void encoder_create(struct encoder *encoder, const struct device *dev)
{
	encoder->dev = dev;
	encoder->count_direction = UNKNOWN;
	encoder->angle_last = 0;
	encoder->angle_timestamp = 0;
	encoder->velocity_last = 0;
	encoder->velocity_timestamp = 0;
	encoder->rotation_count = 0;
	encoder->rotation_count_last = 0;
}

void encoder_init(struct encoder *encoder)
{
	// Initialize all the internal variables of EncoderBase
	// to ensure a "smooth" startup (without a 'jump' from zero)
	encoder_get_raw_angle(encoder);
	k_usleep(1);

	encoder->velocity_last = encoder_get_raw_angle(encoder);
	encoder->velocity_timestamp = time_us();
	k_msleep(1);

	encoder_get_raw_angle(encoder);
	k_usleep(1);

	encoder->angle_last = encoder_get_raw_angle(encoder);
	encoder->angle_timestamp = time_us();
}

float encoder_get_raw_angle(struct encoder *encoder)
{
	return encoder_get_radian(encoder->dev);
}

void encoder_update(struct encoder *encoder)
{
	float angle = encoder_get_raw_angle(encoder);
	encoder->angle_timestamp = time_us();

	float deltaAngle = angle - encoder->angle_last;
	// If overflow happened track it as full rotation
	if (fabsf(deltaAngle) > (0.8f * PI2))
		encoder->rotation_count += (deltaAngle > 0) ? -1 : 1;

	encoder->angle_last = angle;
}

float encoder_get_lap_angle(struct encoder *encoder)
{
	return encoder->angle_last;
}

float encoder_get_full_angle(struct encoder *encoder)
{
	return (float)encoder->rotation_count * PI2 + encoder->angle_last;
}

float encoder_get_velocity(struct encoder *encoder)
{
	float time = (float)(encoder->angle_timestamp - encoder->velocity_timestamp) * 1e-6f;
	// Quick fix for strange cases (micros overflow)
	if (time <= 0)
		time = 1e-3f;

	// velocity calculation
	float vel = ((float)(encoder->rotation_count - encoder->rotation_count_last) * PI2 +
		     (encoder->angle_last - encoder->velocity_last)) /
		    time;

	// save variables for future pass
	encoder->velocity_last = encoder->angle_last;
	encoder->rotation_count_last = encoder->rotation_count;
	encoder->velocity_timestamp = encoder->angle_timestamp;

	return vel;
}

int32_t encoder_rotation_count(struct encoder *encoder)
{
	return encoder->rotation_count;
}
