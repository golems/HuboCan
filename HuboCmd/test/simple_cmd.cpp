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

#include <math.h>

#include <iostream>

#include "HuboCmd/Commander.hpp"
#include "HuboRT/Daemonizer.hpp"

int main(int argc, char* argv[])
{
    HuboCmd::Commander cmd;
    HuboRT::Daemonizer rt;
    rt.redirect_signals();

    if(!cmd.initialized())
    {
        std::cout << "Commander was not initialized successfully!" << std::endl;
        return 1;
    }

    if(argc != 3)
    {
        std::cout << "Usage: ./simple_cmd <joint_name> <joint_value>" << std::endl;
        return 2;
    }

    std::string joint_name(argv[1]);
    size_t joint_index = cmd.get_index(joint_name);
    double joint_value = atof(argv[2]);

    cmd.update();

    std::cout << "Claiming joint" << std::endl;
    cmd.claim_joint(joint_index);
    cmd.send_commands();

    cmd.update();

    std::cout << "Setting mode" << std::endl;
    cmd.set_mode(joint_index, HUBO_CMD_RIGID);
    std::cout << "Setting position" << std::endl;
    cmd.set_position(joint_index, joint_value);
    cmd.send_commands();


//    while( fabs(cmd.joints[joint_index].position - joint_value) > 1e-3 )
//    {
//        cmd.update();
//    }

    int counter = 0;
    while(rt.good())
    {
        cmd.update();
        if(counter > 200)
        {
            std::cout << cmd.joints[joint_index] << std::endl;
            counter = 0;
        }
        ++counter;
    }

    return 0;
}
