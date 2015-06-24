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

#ifndef HUBOPATH_HUBO_PATH_HPP
#define HUBOPATH_HUBO_PATH_HPP

#include "Trajectory.hpp"

namespace HuboPath {

HuboCan::error_result_t send_trajectory(ach_channel_t& output_channel,
                                        ach_channel_t& feedback_channel,
                                        const Trajectory& trajectory,
                                        int max_wait_time = 5);

HuboCan::error_result_t receive_trajectory(ach_channel_t& input_channel,
                                           ach_channel_t& feedback_channel,
                                           Trajectory& new_trajectory,
                                           int max_wait_time = 5);

} // namespace HuboPath

const char* hubo_path_interp_to_string(const hubo_path_interp_t& type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_interp_t& type);

const char* hubo_path_instruction_to_string(const hubo_path_instruction_t& type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_instruction_t& type);

//const char* hubo_path_element_to_string(const hubo_path_element_t& elem);
//std::ostream& operator<<(std::ostream& stream, const hubo_path_element_t& elem);

//const char* hubo_path_params_to_string(const hubo_path_params_t& params);
std::ostream& operator<<(std::ostream& stream, const hubo_path_params_t& params);


const char* hubo_path_rx_state_to_string(const hubo_path_rx_state_t& state);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_state_t& state);

const char* hubo_path_rx_to_string(const hubo_path_rx_t& rx);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_t& rx);

#endif // HUBOPATH_HUBO_PATH_HPP
