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

#include <iostream>

#include "HuboCan/HuboDescription.hpp"

int main(int, char* [])
{
    HuboCan::HuboDescription desc;
//    desc.parseFile("../HuboCan/devices/Hubo2Plus.dd");
    desc.parseFile("../HuboCan/devices/DrcHubo.dd");
    desc.broadcastInfo();

//    desc.receiveInfo();

//    std::cout << desc.getJointTable() << std::endl << std::endl;

//    std::cout << desc.getJmcTable() << std::endl;

//    desc.broadcastInfo();

//    HuboCan::HuboDescription copy_desc(desc);

//    std::cout << "Copy-constructed:" << std::endl;
//    std::cout << copy_desc.getJointTable() << std::endl << std::endl;

//    HuboCan::HuboDescription assign_desc;
//    assign_desc = desc;
//    std::cout << "Copy-assigned:" << std::endl;
//    std::cout << assign_desc.getJointTable() << std::endl;

    StringArray joint_names;
    joint_names.push_back("RSP");
    joint_names.push_back("REP");

    IndexArray indices = desc.getJointIndices(joint_names);
    for(size_t i=0; i<indices.size(); ++i)
        std::cout << indices[i] << "\t";
    std::cout << std::endl;

    std::cout << "Name: " << desc.params.name
              << ", Frequency: " << desc.params.frequency
              << ", CAN buses: " << desc.params.can_bus_count << std::endl;

    return 0;
}
