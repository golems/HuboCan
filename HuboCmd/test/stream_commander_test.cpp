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

int main(int, char* [])
{
    HuboCmd::Commander cmd;
    if(!cmd.initialized())
    {
        std::cout << "Commander was not initialized successfully!" << std::endl;
        return 1;
    }

    StringArray joint_names;
    ValueArray joint_values;
    joint_names.push_back("RSP");
    joint_names.push_back("REP");
    joint_values.resize(joint_names.size());

    IndexArray indices = cmd.get_indices(joint_names);
    
    cmd.claim_joints(indices);
    cmd.send_commands();
    cmd.update();

    cmd.set_modes(indices, HUBO_CMD_RIGID);

    cmd.update();

    for(size_t i=0; i<indices.size(); ++i)
    {
        if(fabs(cmd.joints[indices[i]].position) > 5e-3)
        {
            std::cout << "Joint " << cmd.description().joints[indices[i]]->info.name
                      << " is at " << cmd.joints[indices[i]].position
                      << ", so we will not execute this test" << std::endl;
            return 1;
        }
    }

    double T = 10, start = cmd.get_time();
    double elapsed = 0;
    while(elapsed <= T)
    {
        joint_values[0] = M_PI/4.0*sin(2*M_PI*elapsed/T);
        joint_values[1] = -M_PI/2.0*(1.0/2.0)*(1-cos(2*M_PI*elapsed/T));
        
        std::cout << joint_values[0]
                  << "\t" << joint_values[1]
                  << std::endl;

        cmd.set_positions(indices, joint_values);

        cmd.send_commands();
        
        HuboCan::error_result_t result = cmd.update();
        if(result != HuboCan::OKAY)
        {
            std::cout << "Update threw an error: " << result << std::endl;
            return 1;
        }
        elapsed = cmd.get_time() - start;
    }
    joint_values[0] = 0;
    joint_values[1] = 0;
    cmd.set_positions(indices, joint_values);
    cmd.send_commands();



    return 0;
}
