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

#ifndef HUBOPATH_HUBO_PATH_C_H
#define HUBOPATH_HUBO_PATH_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "HuboCan/AchIncludes.h"
#include "HuboCan/hubo_info_c.h"

#define HUBO_PATH_INSTRUCTION_CHANNEL "hubo_path_instruction"
#define HUBO_PATH_INPUT_CHANNEL "hubo_path_input"
#define HUBO_PATH_FEEDBACK_CHANNEL "hubo_path_feedback"
#define HUBO_PATH_PLAYER_STATE_CHANNEL "hubo_path_player_state"

//                             123456789012345
#define HUBO_PATH_HEADER_CODE "PATHHEADERv0.01"
#define HUBO_PATH_HEADER_CODE_SIZE 16 // including null-terminator \0

// Maximum number of joints supported in HuboPath messages.
// Note: Theoretically, this may need to be increased if Hubo
// (or another robot using this codebase) has more than 50
// joints, but this seems unlikely.
#define HUBO_PATH_JOINT_MAX_SIZE 64
// TODO: ^^^ Consider using variable-sized data structures for this based
// on the HuboDescription. For right now, this is more involved than I think
// it's worth. Also, this fixed-size format makes it easier to dump into files.

// Maximum number of waypoints transmitted in each message over Ach.
// This value could be adjusted if 100 turns out to not be optimal.
#define HUBO_PATH_CHUNK_MAX_SIZE 100

// A stream of const-sized messages are being used for paths, unlike the
// single-shoot variable-sized messages for state and command data. This
// is because those other message types have a predictable max-size for 
// a given HuboDescription of the robot. Paths, on the other hand, can be
// arbitrarily long, so we cannot make any assumptions about a max size.

typedef enum hubo_path_interp {
    
    HUBO_PATH_SPLINE=0, /*! Go from waypoint to waypoint based on a series of splines,
                            coming to a stop at each waypoint                               */
    HUBO_PATH_OPTIMAL,  /*! Minimize the time spent travelling along the path, but without
                            violating the joints' nominal speed and acceleration settings   */
    HUBO_PATH_DENSIFY,  /*! NOT YET IMPLEMENTED. Add points based on input frequency        */
    HUBO_PATH_RAW       /*! Use the waypoints exactly as they are given                     */
    
} hubo_path_interp_t;
// Note: No matter which option is chosen, paths will be rejected if they violate maximum
// speed or acceleration settings.

typedef enum hubo_path_instruction {
    
    HUBO_PATH_QUIT = 0, /*! Quit the current trajectory. The next command will prompt the
                            trajectory runner to load a new trajectory                              */
    HUBO_PATH_RUN,      /*! Continue running the current trajectory. If the trajectory runner
                            is not currently in a trajectory, it will attempt to load a new one
                            and then immediately start running it                                   */
    HUBO_PATH_PAUSE,    /*! Pause the current trajectory. If the trajectory runner is not
                            currently in a trajectory, it will attempt to load a new one and then
                            pause at the first timestep                                             */
    HUBO_PATH_REVERSE,  /*! Run backwards through the current trajectory. If the trajectory runner
                            is not currently in a trajectory, it will behave the same as pause      */
    HUBO_PATH_LOAD,     /*! Quit the current trajectory and attempt to load a new one. Then pause.  */
    HUBO_PATH_LOAD_N_GO /*! Quit the current trajectory and attempt to load a new one. Then run.    */
    
    
} hubo_path_instruction_t;

typedef struct hubo_path_command {

    hubo_path_instruction_t instruction;

    // TODO: Add in flags for parameters that we might want to change during
    // trajectory execution. For example, overriding the control scheme

}__attribute__((packed)) hubo_path_command_t;

typedef struct hubo_player_state {

    hubo_path_instruction_t current_instruction;
    uint64_t current_index;
    uint64_t trajectory_size;

}__attribute__((packed)) hubo_player_state_t;

typedef struct hubo_path_element {
    
    double references[HUBO_PATH_JOINT_MAX_SIZE];

    uint64_t phase_index;
    
    // TODO: Add controller-related parameters in here
    
    
}__attribute__((packed)) hubo_path_element_t; // Size estimate: 400 bytes

typedef struct hubo_path_header {
    
    char code[HUBO_PATH_HEADER_CODE_SIZE];

    // TODO: Add any useful meta-data in here, e.g. the model
    // of hubo that the trajectory is meant for
    
}__attribute__((packed)) hubo_path_header_t; // Size estimate: 16 bytes

typedef struct hubo_path_params {
    
    double frequency;
    hubo_path_interp_t interp;
    double tolerance;
    uint64_t bitmap;
    
    uint8_t use_custom_limits;
    hubo_joint_limits_t limits[HUBO_PATH_JOINT_MAX_SIZE];
    
    // TODO: Add any parameters which apply to all elements in the path
    
}__attribute__((packed)) hubo_path_params_t; // Size estimate: ~24 bytes

typedef struct hubo_path_chunk {

    hubo_path_header_t header; // Size estimate: 16 bytes
    
    hubo_path_params_t params; // Size estimate: 24 bytes
    hubo_path_element_t elements[HUBO_PATH_CHUNK_MAX_SIZE]; // Size estimate: 40000 bytes
    
    uint32_t chunk_size;    /*! Number of relevant steps in this chunk      */
    int32_t chunk_id;       /*! ID of this chunk                            */
    uint32_t total_chunks;  /*! Total number of chunks to be streamed in    */
    
}__attribute__((packed)) hubo_path_chunk_t; // Size estimate: ~40000 bytes

void clear_hubo_path_chunk(hubo_path_chunk_t* chunk);
int  check_hubo_path_chunk(const hubo_path_chunk_t* chunk);

enum {
    PATH_TX_CANCEL = -2
};

typedef enum {
    
    PATH_RX_IGNORING = 0,   /*! There is no point in sending any path chunks right now  */
    PATH_RX_READ_READY,     /*! Ready to start reading a series of path chunks          */
    PATH_RX_LISTENING,      /*! Currently reading a series of path chunks               */
    PATH_RX_TIMEOUT,        /*! Timed out while waiting for the next chunk  -- quitting!*/
    PATH_RX_DISCONTINUITY,  /*! Received a discontinuous chunk              -- quitting!*/
    PATH_RX_ACH_ERROR,      /*! Ran into an ach error                       -- quitting!*/
    PATH_RX_FINISHED,       /*! Finished receiving the incoming path chunks -- good quit*/
    PATH_RX_CANCELED        /*! The user has canceled the chunk sending     -- quitting!*/
    
} hubo_path_rx_state_t;

typedef struct path_reception {
    
    hubo_path_rx_state_t state;
    
    uint32_t chunk_id;
    uint32_t expected_size;
    
}__attribute__((packed)) hubo_path_rx_t;

#endif // HUBOPATH_HUBO_PATH_C_H
