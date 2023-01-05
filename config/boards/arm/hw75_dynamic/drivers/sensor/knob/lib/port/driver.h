#pragma once

#include <device.h>
#include <stdbool.h>

struct driver {
	const struct device *dev;
	float voltage_power_supply;
};

void driver_create(struct driver *driver, const struct device *dev);
void driver_set_enable(struct driver *driver, bool enable);
void driver_set_voltage(struct driver *driver, float voltage_a, float voltage_b, float voltage_c);
