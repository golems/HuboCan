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

extern "C"{
#include <stdlib.h>
}

#include "HuboState/State.hpp"
#include "HuboCan/DeviceStrings.hpp"

namespace HuboState {

State::State(double timeout)
{
    _initialize();
    receive_description(timeout);
}

State::State(const HuboCan::HuboDescription &description)
{
    _initialize();
    load_description(description);
}

State::~State()
{
    free(_last_cmd_data);
}

bool State::receive_description(double timeout_sec)
{
    HuboCan::error_result_t result = _desc.receiveInfo(timeout_sec);
    _initialized = (result == HuboCan::OKAY);
    _create_memory();
    return _initialized;
}

void State::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
    _initialized = true;
}

void State::_initialize()
{
    _last_cmd_data = NULL;
}

void State::_create_memory()
{
    free(_last_cmd_data);

    _last_cmd_data = hubo_cmd_init_data(_desc.getJointCount());
    joints.initialize(_desc.getJointNames(), HUBO_JOINT_SENSOR_CHANNEL);

    std::vector<std::string> imu_names;
    std::vector<std::string> ft_names;
    for(size_t i=0; i<_desc.sensors.size(); ++i)
    {
        const hubo_sensor_info_t& info = _desc.sensors[i]->info;
        if(info.sensor == HuboCan::imu_sensor_string)
            imu_names.push_back(info.name);
        else if(info.sensor == HuboCan::ft_sensor_string)
            ft_names.push_back(info.name);
    }

    imus.initialize(imu_names, HUBO_IMU_SENSOR_CHANNEL);
    force_torques.initialize(ft_names, HUBO_FT_SENSOR_CHANNEL);
}

HuboCan::error_result_t State::update(double timeout_sec)
{
    HuboCan::error_result_t result = HuboCan::OKAY;
    result |= joints.receive_data(timeout_sec);
    result |= imus.receive_data(0);
    result |= force_torques.receive_data(0);

    if( joints.get_time() != imus.get_time()
     || joints.get_time() != force_torques.get_time())
    {
        double  jointTime = joints.get_time(),
                imuTime = imus.get_time(),
                ftTime = force_torques.get_time();

        std::cout << HuboCan::error_result_t(HuboCan::SYNCH_ERROR)
                  << " | Published time stamps: Joints " << jointTime
                  << " | IMU " << imuTime << " (diff " << jointTime - imuTime << ")"
                  << " | FT " << ftTime << " (diff " << jointTime - ftTime << ")" << std::endl;
    }

    return result;
}

HuboCan::error_result_t State::publish()
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double timestamp = (double)(time.tv_sec);
    timestamp += (double)(time.tv_nsec)/1.0e9;

    bool success = true;
    success &= force_torques.send_data(timestamp);
    success &= imus.send_data(timestamp);
    success &= joints.send_data(timestamp);

    if(success)
        return HuboCan::OKAY;
    else
        return HuboCan::ACH_ERROR;
}

double State::get_time()
{
    return joints.get_time();
}

State::State(const State&) { }

State& State::operator=(const State&) { return *this; }

} // namespace HuboState
