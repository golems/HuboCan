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

#ifndef HUBOCMD_HUBO_AUX_CMD_C_H
#define HUBOCMD_HUBO_AUX_CMD_C_H

#include "hubo_cmd_c.h"

#define HUBO_AUX_CMD_CHANNEL "hubo_aux_cmd"

/*                                123456789012345                       */
#define HUBO_AUX_CMD_HEADER_CODE "AUXHEADER_V0.01"
#define HUBO_AUX_CMD_HEADER_CODE_SIZE 16 // including null-terminator \0

typedef enum {

    // TODO: Put in all the different kinds of board commands
    HOME_JOINT,
    HOME_ALL_JOINTS,

    INIT_ALL_SENSORS,
    INIT_ALL_IMUS,
    INIT_ALL_FTS,
    INIT_SENSOR

} hubo_aux_cmd_id_t;

typedef enum {

    DISABLE,
    ENABLE,
    UPDATE,
    IGNORE,

    CLOCKWISE,
    COUNTERCLOCKWISE,

    // Homing type parameters
    SWITCH_AND_INDEX,
    SWITCH,
    JAM_LIMIT

} hubo_aux_cmd_param_t;

typedef struct hubo_aux_cmd {

    char code[HUBO_AUX_CMD_HEADER_CODE_SIZE];
    hubo_aux_cmd_id_t cmd_id;
    uint32_t device_id;
    uint32_t component_id;
    hubo_aux_cmd_param_t params[8];
    double values[8];

}__attribute__((packed)) hubo_aux_cmd_t;

hubo_data_error_t hubo_aux_cmd_header_check(const hubo_aux_cmd_t* cmd);

#endif // HUBOCMD_HUBO_AUX_CMD_C_H
