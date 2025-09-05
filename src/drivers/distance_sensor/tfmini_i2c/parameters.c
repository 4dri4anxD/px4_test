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
 * TFMini rangefinder (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 * @value 0 Disabled
 * @value 1 TFMINI
 * @value 2 ISTRA24
 * @value 3 ISTRA24_100m
 */
PARAM_DEFINE_INT32(SENS_TFMINI_I2C, 0);

/**
 * TFMini Sensor 0 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI0_ROT, 0);

/**
 * TFMini Sensor 1 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI1_ROT, 0);

/**
 * TFMini Sensor 2 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI2_ROT, 0);

/**
 * TFMini Sensor 3 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI3_ROT, 0);

/**
 * TFMini Sensor 4 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI4_ROT, 0);

/**
 * TFMini Sensor 5 Rotation
 *
 * This parameter defines the rotation of the TFMini sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(SENS_TFMINI5_ROT, 0);

/**
 * TFMini Sensor 0 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI0_ADD, 0);

/**
 * TFMini Sensor 1 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI1_ADD, 0);

/**
 * TFMini Sensor 2 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI2_ADD, 0);

/**
 * TFMini Sensor 3 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI3_ADD, 0);

/**
 * TFMini Sensor 4 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI4_ADD, 0);

/**
 * TFMini Sensor 5 Address
 *
 * This parameter defines the I2C address of the TFMini sensor.
 *
 * @reboot_required true
 * @min 0
 * @max 127
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_TFMINI5_ADD, 0);
