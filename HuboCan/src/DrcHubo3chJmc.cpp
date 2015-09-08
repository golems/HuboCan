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

#include "HuboCan/HuboJmc.hpp"
#include "HuboState/State.hpp"
#include "HuboCan/HuboCanId.hpp"
#include "HuboCmd/Aggregator.hpp"

namespace HuboCan {

void DrcHubo3chJmc::_send_reference_commands()
{
    // TODO: Implement this

    for(size_t i=0; i < joints.size(); ++i)
    {
        const hubo_joint_cmd_t& cmd = _agg->joint(joints[i]->info.software_index);

        _state->joints[joints[i]->info.software_index].reference = cmd.position;
    }
}

bool DrcHubo3chJmc::_decode_encoder_reading(const can_frame_t& frame)
{
    // TODO: Decide if frame.can_dlc should be checked
    if(joints.size() > 3)
    {
        std::cout << "WARNING: Expected 3 joints in the DrcHubo3chJmc named " << info.name
                  << " but instead there are " << joints.size() << "!" << std::endl;
        return false;
    }

    for(size_t i=0; i<joints.size(); ++i)
    {
        int16_t encoder = 0;
        encoder = (encoder << 8) + frame.data[1 + i*2];
        encoder = (encoder << 8) + frame.data[0 + i*2];

        size_t joint_index = joints[i]->info.software_index;
        _state->joints[joint_index].position =
                    joints[i]->encoder2radian(encoder);

        joints[i]->updated = true;
        ++joints[i]->received_replies;
    }

    return true;
}

bool DrcHubo3chJmc::_decode_status_reading(const can_frame_t& frame)
{
    for(size_t i=0; i<joints.size(); ++i)
    {
        size_t jnt = joints[i]->info.software_index;
        hubo_joint_status_t& status = _state->joints[jnt].status;

        uint8_t byte = frame.data[i];
        status.driver_on    = (byte >> 0) & 0x01;
        status.control_on   = (byte >> 1) & 0x01;
        status.control_mode = (byte >> 2) & 0x01;
        status.limit_switch = (byte >> 3) & 0x01;

        status.error.jam            = (byte >> 4) & 0x01;
        status.error.pwm_saturated  = (byte >> 5) & 0x01;
        status.error.big            = (byte >> 6) & 0x01;
        status.error.encoder        = (byte >> 7) & 0x01;
    }

    return true;
}

} // namespace HuboCan
