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
 * @file tfmini_i2c.hpp
 * @author Mauricio Diaz <mauricio.diaz9036@gmail.com>
 *
 * Driver for the Benewake TFmini I2C laser rangefinder series
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/cli.h>

#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>


#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/topics/distance_sensor.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

#define TFMINI_I2C_DEFAULT_ADDRESS	0x10
#define TFMINI_I2C_BUS_CLOCK            400000 // 400kHz bus speed

using namespace time_literals;

class TFMiniI2C : public device::I2C, public I2CSPIDriver<TFMiniI2C>, public ModuleParams
{
public:
	TFMiniI2C(const I2CSPIDriverConfig &config);

	~TFMiniI2C() override;

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:

	int probe() override;

	int collect();

	int configure();

	bool checksum(uint8_t *arr, int len);

	void start();

	enum class State {
		Configuring,
		Running
	};

	struct TFMiniCMD {
		static const uint8_t ReadMeasurement[5];
		static const uint8_t GetFwVersion[4];
		static const uint8_t TriggerDetection[4];
		static const uint8_t OutputFormatCm[5];
		static const uint8_t EnableOutput[5];
		static const uint8_t SaveSettings[4];
	};

	struct __attribute__((packed)) TFminiFrame {
		uint8_t header1;
		uint8_t header2;
		uint16_t distance;
		uint16_t strength;
		uint16_t temperature;
		uint8_t checksum;
	};

	State _state{State::Configuring};
	int _consecutive_errors{0};

	float _min_distance_m;
	float _max_distance_m;

	PX4Rangefinder _px4_rangefinder;

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_TFMINI_I2C>) _param_sens_tfmini_i2c
	)

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
};
