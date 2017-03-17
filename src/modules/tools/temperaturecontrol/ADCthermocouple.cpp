/*
 This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
 Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
 */

#include "ADCthermocouple.h"

#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "libs/Median.h"
#include "utils.h"
#include "StreamOutputPool.h"

#include <fastmath.h>

#include "MRI_Hooks.h"

#define UNDEFINED -1

#define ADCthermocouple_pin_checksum		CHECKSUM("adcthermocouple_pin")
#define ADCthermocouple_offset_checksum		CHECKSUM("adcthermocouple_offset")   //Temperature offset
#define ADCthermocouple_gain_checksum		CHECKSUM("adcthermocouple_gain")     //Voltage gain between sensor and ADC
#define ADCthermocouple_mv_per_c_checksum	CHECKSUM("adcthermocouple_mv_per_c") //Output from sensor

ADCthermocouple::ADCthermocouple()
{
	min_temp = 999;
	max_temp = 0;
}

ADCthermocouple::~ADCthermocouple()
{
}

// Get configuration from the config file
void ADCthermocouple::UpdateConfig(uint16_t module_checksum,
		uint16_t name_checksum)
{
	// Thermistor pin for ADC readings
	this->ADCthermocouple_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, ADCthermocouple_pin_checksum)->required()->as_string());
	this->ADCthermocouple_offset = THEKERNEL->config->value(module_checksum, name_checksum, ADCthermocouple_offset_checksum)->by_default(0)->as_number();
	this->ADCthermocouple_gain = THEKERNEL->config->value(module_checksum, name_checksum, ADCthermocouple_gain_checksum)->by_default(1)->as_number();
	this->ADCthermocouple_mv_per_c = THEKERNEL->config->value(module_checksum, name_checksum, ADCthermocouple_mv_per_c_checksum)->by_default(10)->as_number();

	THEKERNEL->adc->enable_pin(&ADCthermocouple_pin);
}

float ADCthermocouple::get_temperature()
{
	float t = adc_value_to_temperature(new_ADCthermocouple_reading());
	// keep track of min/max for M305
	if (t > max_temp)
		max_temp = t;
	if (t < min_temp)
		min_temp = t;
	return t;
}

void ADCthermocouple::get_raw()
{
	int adc_value = new_ADCthermocouple_reading();
	const uint32_t max_adc_value = THEKERNEL->adc->get_max_value();
	float temp = adc_value_to_temperature(adc_value);

	THEKERNEL->streams->printf("adc = %d, max_adc = %lu, temp = %f, offset = %f, gain = %f, mv per c = %f\n", adc_value, max_adc_value, temp, ADCthermocouple_offset, ADCthermocouple_gain, ADCthermocouple_mv_per_c);

	// reset the min/max
	min_temp = max_temp = temp;
}

float ADCthermocouple::adc_value_to_temperature(uint32_t adc_value)
{
	const uint32_t max_adc_value = THEKERNEL->adc->get_max_value();
	if ((adc_value >= max_adc_value))
	return infinityf();

	float vin = (adc_value * 3.3) / max_adc_value;
	float temp = (vin * 1000) / ADCthermocouple_mv_per_c * ADCthermocouple_gain - ADCthermocouple_offset;

	return temp;
}

int ADCthermocouple::new_ADCthermocouple_reading()
{
	// filtering now done in ADC
	return THEKERNEL->adc->read(&ADCthermocouple_pin);
}
