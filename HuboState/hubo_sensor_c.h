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

#ifndef HUBOSTATE_HUBO_SENSOR_C_H
#define HUBOSTATE_HUBO_SENSOR_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "HuboCan/hubo_info_c.h"
#include "HuboCmd/hubo_cmd_c.h"
#include "HuboCan/AchIncludes.h"

#define HUBO_JOINT_SENSOR_CHANNEL   "hubo_joint_sensors"
#define HUBO_IMU_SENSOR_CHANNEL     "hubo_imu_sensors"
#define HUBO_FT_SENSOR_CHANNEL      "hubo_ft_sensors"

#define HUBO_DATA_HEADER_CODE "DATAHEADER_0.01"
#define HUBO_DATA_HEADER_CODE_SIZE 16 /* including null-terminator \0 */

typedef uint8_t hubo_data;

typedef struct hubo_data_header {

    char code[HUBO_DATA_HEADER_CODE_SIZE];
    uint8_t array_size;
    double time;

}__attribute__((packed)) hubo_data_header_t;

typedef struct hubo_joint_error {

    uint8_t jam;
    uint8_t pwm_saturated;
    uint8_t big;
    uint8_t encoder;
    uint8_t driver_fault;
    uint8_t motor_fail_0;
    uint8_t motor_fail_1;

    uint8_t min_position;
    uint8_t max_position;
    uint8_t velocity;
    uint8_t acceleration;
    uint8_t temperature;

}__attribute__((packed)) hubo_joint_error_t;

typedef struct hubo_joint_status {

    uint8_t driver_on;
    uint8_t control_on;
    uint8_t control_mode;
    uint8_t limit_switch;

    uint8_t home_flag;

    hubo_joint_error_t error;

}__attribute__((packed)) hubo_joint_status_t; // 17 bytes total

typedef struct hubo_joint_state {

    double position;
    double duty;
    double current;
    double temperature;

    double reference;
    hubo_cmd_mode_t mode;

    hubo_joint_status_t status;

}__attribute__((packed)) hubo_joint_state_t;

typedef struct hubo_imu_state {

//    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];

    double angular_position[3];
    double angular_velocity[3];

}__attribute__((packed)) hubo_imu_state_t;

typedef struct hubo_ft_state {

//    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];

    double force[3];
    double torque[3];

}__attribute__((packed)) hubo_ft_state_t;


hubo_data_error_t hubo_data_header_check(const hubo_data* data);

#endif // HUBOSTATE_HUBO_SENSOR_C_H
