/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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

#ifndef HUBOCOMMANDER_HPP
#define HUBOCOMMANDER_HPP

#include "AchIncludes.h"
#include "hubo_cmd_c.h"
#include "InfoReceiver.hpp"

namespace HuboCmd {

typedef std::vector<hubo_cmd_mode_t> ModeArray;

class Commander
{
public:

    Commander();
    ~Commander();

    HuboState::error_result_t set_mode(JointIndex joint, hubo_cmd_mode_t mode);
    HuboState::error_result_t set_modes(IndexArray joints, ModeArray modes);

    HuboState::error_result_t position(JointIndex joint, float value);
    HuboState::error_result_t positions(IndexArray joints, ValueArray values);

    HuboState::error_result_t base_torque(JointIndex joint, double value);
    HuboState::error_result_t base_torques(IndexArray joints, ValueArray values);

    HuboState::error_result_t kP_gain(JointIndex joint, double kP_value);
    HuboState::error_result_t kP_gains(IndexArray joints, ValueArray kP_values);

    HuboState::error_result_t kD_gain(JointIndex joint, double kD_value);
    HuboState::error_result_t kD_gains(IndexArray joints, ValueArray kD_values);

    JointIndex getIndex(std::string joint_name);
    IndexArray getIndices(StringArray joint_names);


    HuboState::error_result_t send_commands();

protected:

    void _initialize();
    hubo_cmd_data* cmd_data;

    HuboState::InfoReceiver _info;

};

} // HuboCmd

#endif // HUBOCOMMANDER_HPP
