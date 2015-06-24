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

#ifndef HUBOCAN_DEVICESTRINGS_HPP
#define HUBOCAN_DEVICESTRINGS_HPP

#include <string>

namespace HuboCan {

//-----------------------------------------------------------------------------
// -- Joint string
//-----------------------------------------------------------------------------
const std::string joint_device_string = "Joint";


//-----------------------------------------------------------------------------
// -- JMC strings
//-----------------------------------------------------------------------------
const std::string jmc_device_string = "JMC";

// Hub2Plus JMC types
const std::string hubo2plus_1ch_type_string = "H2P_1CH";
const std::string hubo2plus_2ch_type_string = "H2P_2CH";
const std::string hubo2plus_nck_type_string = "H2P_NCK"; // Neck JMC
const std::string hubo2plus_5ch_type_string = "H2P_5CH";

// DrcHubo JMC types
const std::string drchubo_2ch_type_string = "DRC_2CH";
const std::string drchubo_3ch_type_string = "DRC_3CH";


//-----------------------------------------------------------------------------
// -- IMU strings
//-----------------------------------------------------------------------------
const std::string imu_sensor_string = "IMU";

// Normal IMU types
const std::string hubo2plus_imu_sensor_type_string = "H2P_IMU";
const std::string drchubo_imu_sensor_type_string = "DRC_IMU";

// Tilt sensor for feet
const std::string hubo2plus_tilt_sensor_type_string = "H2P_TILT";


//-----------------------------------------------------------------------------
// -- Force-Torque strings
//-----------------------------------------------------------------------------
const std::string ft_sensor_string = "FT";

const std::string hubo2plus_ft_sensor_type_string = "H2P_FT";
const std::string drchubo_ft_sensor_type_string = "DRC_FT";


//-----------------------------------------------------------------------------
// -- Meta string
//-----------------------------------------------------------------------------
// Used to specify the name of the robot, the nominal control frequency,
// and the number of CAN buses available
const std::string meta_device_string = "meta";

} // namespace HuboCan

#endif // HUBOCAN_DEVICESTRINGS_HPP
