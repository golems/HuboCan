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

#include "HuboPath/hubo_path.hpp"

int main(int, char* [])
{
    HuboPath::Trajectory traj;
    hubo_path_element_t elem;
    
    IndexArray indices;
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(4);
    indices.push_back(7);
    indices.push_back(10);

    traj.claim_joints(indices);

    for(size_t i=0; i<50; ++i)
    {
        elem.references[1] = i;
        traj.push_back(elem);
    }

    std::cout << traj << std::endl;

    ach_channel_t output_chan;
    ach_status_t result = ach_open(&output_chan, HUBO_PATH_INPUT_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Ach error: " << ach_result_to_string(result) << std::endl;
        return 1;
    }

    ach_channel_t feedback_chan;
    result = ach_open(&feedback_chan, HUBO_PATH_FEEDBACK_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Feedback channel Ach error: " << ach_result_to_string(result) << std::endl;
        return 2;
    }
    report_ach_errors(ach_flush(&feedback_chan), "main", "ach_flush", HUBO_PATH_FEEDBACK_CHANNEL);

    HuboCan::error_result_t sent = HuboPath::send_trajectory(output_chan,
                                                             feedback_chan,
                                                             traj, 10);

    std::cout << "Sending status: " << sent << std::endl;
    
    return 0;
}
