/****************************************************************************
 *
 *   Copyright (c) 2025 Mauricio Diaz. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tfmini_i2c.cpp
 *
 * @author Mauricio Diaz <mauricio.diaz9036@gmail.com>
 *
 * Driver for the TFMini i2c lidar range finder series.
 */

#include "tfmini_i2c.hpp"

//List of commands
const uint8_t TFMiniI2C::TFMiniCMD::ReadMeasurement[5]  = { 0x5A, 0x05, 0x00, 0x01, 0x60 };
const uint8_t TFMiniI2C::TFMiniCMD::GetFwVersion[4]     = { 0x5A, 0x04, 0x01, 0x5F };
const uint8_t TFMiniI2C::TFMiniCMD::TriggerDetection[4] = { 0x5A, 0x04, 0x04, 0x62 };
const uint8_t TFMiniI2C::TFMiniCMD::OutputFormatCm[5]   = { 0x5A, 0x05, 0x05, 0x01, 0x65 };
const uint8_t TFMiniI2C::TFMiniCMD::EnableOutput[5]     = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
const uint8_t TFMiniI2C::TFMiniCMD::SaveSettings[4]     = { 0x5A, 0x04, 0x11, 0x6F };

TFMiniI2C::TFMiniI2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TFMINI);
}

TFMiniI2C::~TFMiniI2C()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFMiniI2C::init()
{
	int ret = PX4_ERROR;
	//updateParams();
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
	const int32_t hw_model = _param_sens_tfmini_i2c.get();

	switch (hw_model) {
	case 1: // TFMINI (12m, 100 Hz)
		// Note:
		// Sensor specification shows 0.3m as minimum, but in practice
		// 0.3 is too close to minimum so chattering of invalid sensor decision
		// is happening sometimes. this cause EKF to believe inconsistent range readings.
		// So we set 0.4 as valid minimum.
		_min_distance_m=0.4f;
		_max_distance_m=12.0f;
		_px4_rangefinder.set_fov(math::radians(2.3f));
		break;

	case 2: // ISTRA24 (50m, 100 Hz)
		_min_distance_m=0.3f;
		_max_distance_m=50.0f;
		_px4_rangefinder.set_fov(math::radians(20));
		break;

	case 3: // ISTRA24 (100m, 100 Hz)
		_min_distance_m=0.3f;
		_max_distance_m=100.0f;
		_px4_rangefinder.set_fov(math::radians(20));
		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return -1;
	}
	_px4_rangefinder.set_min_distance(_min_distance_m);
	_px4_rangefinder.set_max_distance(_max_distance_m);

	//Wait a little for the sensor to be initialized
	px4_usleep(100_ms);

	/* do I2C init (and probe) first */
	ret = I2C::init();

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int TFMiniI2C::probe()
{
	uint8_t response[7];
	int ret;

	ret = transfer(TFMiniCMD::GetFwVersion, sizeof(TFMiniCMD::GetFwVersion), nullptr, 0);

	if (ret == PX4_ERROR) {
		PX4_DEBUG("Failed to send FW_VERSION command");
		return PX4_ERROR;
	}

	//Wait a little to get the response
	px4_usleep(10_ms);

	ret = transfer(nullptr, 0, response, 7);

	//From manual: response should be 5A 07 01 V1 V2 V3 SU
	if (ret == PX4_ERROR || response[0] != 0x5A || response[1] != 0x07 || response[2] != 0x01 ||
	    !checksum(response, sizeof(response))) {
		PX4_DEBUG("Checksum failed");
		return PX4_ERROR;
	}

	if (response[5] * 10000 + response[4] * 100 + response[3] < 20003) {
		PX4_DEBUG("Need a newer firmware version (2.0.3 or higher)");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int TFMiniI2C::configure()
{
	int ret;

	ret = transfer(TFMiniCMD::OutputFormatCm, sizeof(TFMiniCMD::OutputFormatCm), nullptr, 0);
	if (ret == PX4_ERROR) {
		PX4_DEBUG("Couldn't set output format");
		return PX4_ERROR;
	}

	px4_usleep(10_ms);

	ret = transfer(TFMiniCMD::EnableOutput, sizeof(TFMiniCMD::EnableOutput), nullptr, 0);
	if (ret == PX4_ERROR) {
		PX4_DEBUG("Couldn't enable output");
		return PX4_ERROR;
	}

	px4_usleep(10_ms);

	ret = transfer(TFMiniCMD::SaveSettings, sizeof(TFMiniCMD::SaveSettings), nullptr, 0);
	if (ret == PX4_ERROR) {
		PX4_DEBUG("Couldn't save settings");
		return PX4_ERROR;
	}

	px4_usleep(100_ms);

	ret = transfer(TFMiniCMD::TriggerDetection, sizeof(TFMiniCMD::TriggerDetection), nullptr, 0);
	if (ret == PX4_ERROR) {
		PX4_DEBUG("Couldn't trigger detection");
		return PX4_ERROR;
	}

	px4_usleep(10_ms);

	return PX4_OK;

}

bool TFMiniI2C::checksum(uint8_t *arr, int len)
{
	/*
	According to manual: Checksum is the sum of all bytes form head to payload
	*/
	if(len==0) return false;
	uint8_t check = 0;
	for (int i = 0; i < len-1; i++) {
		check += arr[i];
	}
	return check == arr[len - 1];
}

void TFMiniI2C::RunImpl()
{
	switch (_state) {
	case State::Configuring: {
			if (configure() == PX4_OK) {
				_state = State::Running;
				ScheduleDelayed(7_ms);

			} else {
				// retry after a while
				PX4_DEBUG("Retrying...");
				ScheduleDelayed(300_ms);
			}

			break;
		}

	case State::Running:

		_px4_rangefinder.set_mode(distance_sensor_s::MODE_ENABLED);

		if (PX4_OK != collect()) {
			PX4_DEBUG("collection error");
			if (++_consecutive_errors > 3) {
				_state = State::Configuring;
				_consecutive_errors = 0;
			}
		}

		ScheduleDelayed(7_ms);
		break;
	}
}

int TFMiniI2C::collect()
{

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	TFminiFrame frame;
	int ret;
	float distance_m;

	int8_t signal_quality = -1;

	ret = transfer(TFMiniCMD::ReadMeasurement, sizeof(TFMiniCMD::ReadMeasurement), nullptr, 0);

	if (ret == PX4_ERROR || transfer(nullptr, 0, (uint8_t *)&frame, sizeof(frame))) {
		PX4_DEBUG("No measurement was caught");
		return PX4_ERROR;
	}

	if (frame.header1 != 0x59 || frame.header2 != 0x59 || !checksum((uint8_t *)&frame, sizeof(frame))) {
		PX4_DEBUG("Checksum failed");
		return PX4_ERROR;
	}

	distance_m = frame.distance/100.0f;
	if (frame.strength < 100 || frame.strength == 0xFFFF) {
		//Strenght not enough to ensure an accurate measurement
		//Set distance as out of range
		signal_quality = 0;
		distance_m = _max_distance_m + 1;
	}

	//To do: calculate signal_quality

	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);

	perf_end(_sample_perf);

	return PX4_OK;
}

void TFMiniI2C::start()
{
	//Schedule a cycle to start things (100Hz but running a little faster to avoid loosing data)
	ScheduleOnInterval(7_ms);
}

void TFMiniI2C::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void TFMiniI2C::print_usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified address using the SENS_TFMINI_I2C parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/tfmini.html

### Examples

Attempt to start driver on a specified address and rotation.
$ tfmini_i2c start -a 82 -P 0
Stop driver
$ tfmini stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfmini_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x10);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('A', 0, 0, 127, "I2C Address parameter (set the I2C to a specific parameter i.e. p:SENS_TFMINI0_ADD)", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
