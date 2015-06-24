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

#include <sstream>

#include "HuboCan/HuboJmc.hpp"

namespace HuboCan {

HuboJmc::HuboJmc()
    : _agg(NULL),
      _state(NULL)
{
    memset(&_frame, 0, sizeof(_frame));
}

void HuboJmc::assign_pointers(HuboCmd::Aggregator* agg, HuboState::State* state)
{
    _agg = agg;
    _state = state;
}

bool HuboJmc::addJoint(HuboJoint *joint, std::string &error_report)
{
    joints.clear();

    HuboJointPtrMap::iterator check = _tempJointMap.find(joint->info.hardware_index);
    if(check != _tempJointMap.end())
    {
        std::stringstream report;
        report << "The JMC named '" << info.name << "' already has a joint in index #"
                  << joint->info.hardware_index << " named '" << check->second->info.name << "'";
        report << "\n -- Therefore we cannot add the joint named '" << joint->info.name << "' to that index.";
        error_report = report.str();
        return false;
    }

    _tempJointMap[joint->info.hardware_index] = joint;
    return true;
}

bool HuboJmc::sortJoints(std::string& error_report)
{
    HuboJointPtrMap::iterator it = _tempJointMap.begin();
    for(size_t i=0; i < _tempJointMap.size(); ++i)
    {
        if(it->first != i)
        {
            std::stringstream report;
            report << "Missing joint index #" << i
                   << " for the JMC named '" << info.name << "'!";

            error_report = report.str();

            return false;
        }

        joints.push_back(it->second);

        ++it;
    }

    return true;
}

void HuboJmc::auxiliary_command(const hubo_aux_cmd_t& command)
{
    _aux_commands.push_back(command);
}

std::string HuboJmc::header()
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(8);
    str << "JMC Name";
    str.width(12);
    str << "JMC Type";
    str.width(7);
    str << "Index";
    str.width(13);
    str << "CAN Channel";

    return str.str();
}

std::string HuboJmc::table() const
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(8);   // JMC Name
    str << info.name;
    str.width(12);  // JMC Type
    str << info.type;
    str.width(7);   // Index
    str << info.hardware_index;
    str.width(13);  // CAN Channel
    str << info.can_channel;

    return str.str();
}

} // namespace HuboCan
