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

#include "HuboState/HuboData.hpp"

using namespace HuboState;

int main(int, char* [])
{
    HuboData<hubo_joint_state_t> rec_data;
    rec_data.verbose = true;
    std::vector<std::string> joint_names;
    joint_names.push_back("RSP");
    joint_names.push_back("RSR");
    joint_names.push_back("RSY");
    joint_names.push_back("REP");
    joint_names.push_back("RWY");
    joint_names.push_back("LSP");
    joint_names.push_back("LSR");
    joint_names.push_back("LSY");
    joint_names.push_back("LEP");
    joint_names.push_back("LWY");

    rec_data.initialize(joint_names, HUBO_JOINT_SENSOR_CHANNEL);
    
    std::vector<hubo_joint_state_t> vec;
    for(size_t i=0; i<10; ++i)
    {
        hubo_joint_state_t js;
        js.position = 10-i;
        vec.push_back(js);
//        rec_data[i].position = 10-i;
    }
    
    rec_data.set_data(vec);
    
    
    
    
    rec_data.receive_data(5);
    vec = rec_data.get_data(false);
    
    
    for(size_t i=0; i<vec.size(); ++i)
    {
        std::cout << vec[i].position << "\t";
    }
    std::cout << std::endl;
    
    for(size_t i=0; i<rec_data.size(); ++i)
    {
        std::cout << rec_data[i].position << "\t";
    }
    std::cout << std::endl;
    
    for(size_t i=0; i<get_data_component_count(rec_data._raw_data); ++i)
    {
        hubo_joint_state_t* jsp = get_data_component<hubo_joint_state_t>(rec_data._raw_data, i);
        std::cout << jsp->position << "\t";
    }
    std::cout << std::endl;
    
    std::cout << "Bytes:" << std::endl;
    for(size_t i=0; i<get_data_size<hubo_joint_state_t>(rec_data._raw_data); ++i)
    {
        std::cout << (unsigned int)(rec_data._raw_data[i]) << "\t";
        if( (i+1)%10 == 0 )
        {
            std::cout << std::endl;
        }
    }
    
    return 0;
}
