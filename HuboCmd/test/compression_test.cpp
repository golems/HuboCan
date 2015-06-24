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

extern "C" {
#include "HuboCmd/hubo_cmd_c.h"
}

#include <iostream>

int main(int, char* [])
{
    size_t num_joints = 10;
    hubo_cmd_data* cx = hubo_cmd_init_data(num_joints);

    hubo_cmd_data* test_byte = cx;

    size_t byte_count = 0;
    std::cout << "Header:" << std::endl;
    for(size_t i=0; i < 16; ++i)
    {
        std::cout << i << ":\t" << *(test_byte+i) << std::endl;
        ++byte_count;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)test_byte;

    std::cout << "pid:\t" << (unsigned int)header->pid << std::endl;
    std::cout << "bitmap:\t" << (unsigned int)header->bitmap << std::endl;


    hubo_joint_cmd_t jc;
    jc.mode = HUBO_CMD_RIGID;
    jc.position = 1;
    jc.base_torque = 2;

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data_set_joint_cmd(cx, &jc, 3);

    hubo_cmd_data_get_joint_cmd(&jc, cx, 4);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data_get_joint_cmd(&jc, cx, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data* comp = hubo_cmd_init_data(num_joints);
    std::cout << "About to compress..." << std::endl;
    hubo_cmd_data_compressor(comp, cx);
    std::cout << "...compressed!" << std::endl;

    hubo_cmd_data_get_joint_cmd(&jc, comp, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    if(hubo_cmd_data_check_if_joint_is_set(comp, 4))
    {
        hubo_cmd_data_get_joint_cmd(&jc, comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;
    }

    jc.position = 10;
    jc.base_torque = 15;

    hubo_cmd_data_set_joint_cmd(cx, &jc, 4);

    hubo_cmd_data_compressor(comp, cx);

    if(hubo_cmd_data_check_if_joint_is_set(comp, 4))
    {
        hubo_cmd_data_get_joint_cmd(&jc, comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;
    }

    return 0;
}
