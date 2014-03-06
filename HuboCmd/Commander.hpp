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

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "HuboCmd/hubo_cmd_c.h"
}
#include "HuboCan/HuboDescription.hpp"

namespace HuboCmd {

typedef std::vector<hubo_cmd_mode_t> ModeArray;

class Commander
{
public:

    Commander(double timeout=1);
    Commander(const HuboCan::HuboDescription& description);
    ~Commander();

    bool receive_description(double timeout=2);
    void load_description(const HuboCan::HuboDescription& description);

    HuboCan::error_result_t set_mode(size_t joint_index, hubo_cmd_mode_t mode);
    HuboCan::error_result_t set_modes(const IndexArray& joints, const ModeArray& modes);

    hubo_cmd_mode_t get_mode(size_t joint_index);
    ModeArray       get_modes(const IndexArray& joints);

    HuboCan::error_result_t set_position(size_t joint_index, float value);
    HuboCan::error_result_t set_positions(const IndexArray& joints, const ValueArray& values);

    float       get_position_cmd(size_t joint_index);
    ValueArray  get_position_cmds(const IndexArray& joints);

    HuboCan::error_result_t set_base_torque(size_t joint_index, double value);
    HuboCan::error_result_t set_base_torques(const IndexArray& joints, const ValueArray& values);

    double      get_base_torque_cmd(size_t joint_index);
    ValueArray  get_base_torque_cmds(const IndexArray& joints);

    HuboCan::error_result_t set_kp_gain(size_t joint_index, double kP_value);
    HuboCan::error_result_t set_kp_gains(const IndexArray& joints, const ValueArray& kP_values);

    double      get_kp_gain_cmd(size_t joint_index);
    ValueArray  get_kp_gain_cmds(const IndexArray& joints);

    HuboCan::error_result_t set_kd_gain(size_t joint_index, double kD_value);
    HuboCan::error_result_t set_kd_gains(const IndexArray& joints, const ValueArray& kD_values);

    double      get_kd_gain_cmd(size_t joint_index);
    ValueArray  get_kd_gain_cmds(const IndexArray& joints);

    size_t      get_index(const std::string& joint_name);
    IndexArray  get_indices(const StringArray& joint_names);

    inline size_t get_joint_count()
    {
        return _desc.getJointCount();
    }

    HuboCan::error_result_t send_commands();

    hubo_cmd_data* cmd_data;

protected:

    bool _has_been_updated;

    void _initialize();
    void _create_memory();

    HuboCan::error_result_t _register_joint(size_t joint_index);
    void _fill_container(size_t joint_index);

    hubo_cmd_data* _compressed_data;

    hubo_joint_cmd_t _container;

    HuboCan::HuboDescription _desc;

    ach_channel_t _cmd_chan;
};

} // HuboCmd

#endif // HUBOCOMMANDER_HPP
