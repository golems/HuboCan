/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HUBOCAN_HUBOCANID_H
#define HUBOCAN_HUBOCANID_H

namespace HuboCan {

// Values used in the CAN protocol by can_id
typedef enum {
    
    CMD_BYTE            = 0x01,
    SENSOR_REQUEST      = 0x02,
    REFERENCE_CMD       = 0x10,
    SENSOR_INSTRUCTION  = 0x2F,

    FT_REPLY            = 0x40,
    IMU_REPLY           = 0x50,
    ENCODER_REPLY       = 0x60,
    STATUS_REPORT       = 0x150

} can_base_id_t;

// Values used in the CAN protocol by data[] entries
typedef enum {
    
    GET_BOARD_INFO      = 0x01,
    GET_STATUS          = 0x02,
    GET_ENCODER         = 0x03,
    GET_CURRENT         = 0x04, // electrical current
    // SendPm = 0x05??
    SET_ENC_ZERO        = 0x06, // set the current encoder value to zero
    SET_POS_GAIN_0      = 0x07,
    SET_POS_GAIN_1      = 0x08,
    SET_CUR_GAIN_0      = 0x09,
    SET_CUR_GAIN_1      = 0x0A,
    // SWITCH_DRIVER = 0x0B ??
    // GoHome = 0x0C ??
    SET_OPENLOOP_PWM    = 0x0D,
    SET_MOTOR_CTRL_ON   = 0x0E,
    SET_MOTOR_CTRL_OFF  = 0x0F,
    SET_CTRL_MODE       = 0x10,
    
    GOTO_HOME           = 0x11,
    
    SET_DEADZONE        = 0x20,
    GET_PARAMETERS      = 0x24,
    
    SET_HOME_PARAM      = 0x30,
    SET_ENC_RES         = 0x38, // set encoder resolution
    SET_MAX_ACC_VEL     = 0x40,
    SET_LOW_POS_LIM     = 0x50,
    SET_UPP_POS_LIM     = 0x56,
    SET_HOME_VEL_ACC    = 0x60,
    
    GAIN_OVERRIDE       = 0x6F,
    SET_BOARD_NUM       = 0xF0,
    SET_JAM_SAT_LIM     = 0xF2, // set the "jam" saturation limit
    SET_ERR_BOUND       = 0xF3,
    INITIALIZE_BOARD    = 0xFA,

    // Sensor commands
    NULL_SENSOR         = 0x81,
    NULL_FT             = 0x00,
    NULL_TILT           = 0x04,

    GET_FT_ACC_DIGITAL        = 0x00,
    GET_FT_ACC_SCALED         = 0x02,
    GET_FT_SCALED_ACC_DIGITAL = 0x03, // Get scaled force-torques and digital accelerations
    GET_FT_DIGITAL_ACC_SCALED = 0x04, // Get digital force-torques and scaled accelerations

    GET_FT_DIGITAL      = 0x11,
    GET_FT_SCALED       = 0x12,

    GET_ACC_DIGITAL     = 0x21,
    GET_ACC_SCALED      = 0x22,

    GET_GYRO_TEMP       = 0x13
    
} can_cmd_id_t;

typedef enum {
    
    PARAM_A = 1,
    PARAM_B = 2,
    PARAM_C = 3,
    PARAM_D = 4,
    PARAM_E = 5,
    PARAM_F = 6,
    
    PARAM_G = 20,
    PARAM_H = 21,
    PARAM_I = 22
    
} can_param_type_t;

inline uint8_t long_to_bytes(unsigned long value, size_t index)
{
    return (uint8_t)( (value >> (index*8) ) & 0xFF );
}

} // namespace HuboCan

#endif // HUBOCAN_HUBOCANID_H
