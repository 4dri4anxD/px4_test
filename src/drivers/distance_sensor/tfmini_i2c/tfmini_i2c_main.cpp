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

#include "tfmini_i2c.hpp"

extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[])
{
	using ThisDriver = TFMiniI2C;
	BusCLIArguments cli{true, false};

	int ch = 0;
	cli.rotation =  (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.i2c_address = TFMINI_I2C_DEFAULT_ADDRESS;
	cli.default_i2c_frequency = TFMINI_I2C_BUS_CLOCK;


	while ((ch = cli.getOpt(argc, argv, "R:A:")) != EOF) {
		switch (ch) {
		case 'R': {
			int rot = -1;
			if (px4_get_parameter_value(cli.optArg(), rot) != 0) {
				PX4_ERR("Rotation parsing failed");
				return -1;
			}
			cli.rotation = (Rotation)rot;
			break;
		}
		case 'A': {
			int add = 0;
			if (px4_get_parameter_value(cli.optArg(), add) != 0) {
				PX4_ERR("Address from parameter parsing failed");
				break;
			}
			cli.i2c_address = (uint8_t)add;
			break;
		}

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;

		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINI);


	if (!strcmp(verb, "start")) {
		if (cli.i2c_address > 0) {
			return ThisDriver::module_start(cli, iterator);
		} else {
			PX4_WARN("Please specify device address!");
			ThisDriver::print_usage();
			return -1;
		}
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

