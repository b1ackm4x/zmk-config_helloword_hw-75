#include "driver.h"

#include <knob/drivers/inverter.h>

void driver_create(struct driver *driver, const struct device *dev)
{
	driver->dev = dev;
	driver->voltage_power_supply = 12;
}

void driver_set_enable(struct driver *driver, bool enable)
{
	if (enable) {
		inverter_start(driver->dev);
	} else {
		inverter_stop(driver->dev);
	}
}

void driver_set_voltage(struct driver *driver, float voltage_a, float voltage_b, float voltage_c)
{
	inverter_set_powers(driver->dev, voltage_a / driver->voltage_power_supply,
			    voltage_b / driver->voltage_power_supply,
			    voltage_c / driver->voltage_power_supply);
}
