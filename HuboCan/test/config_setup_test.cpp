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

#include "HuboRT/Manager.hpp"
#include "HuboCmd/Commander.hpp"
#include "HuboCmd/Aggregator.hpp"
#include "HuboState/State.hpp"
#include "HuboPath/hubo_path.hpp"
#include "HuboRT/LogRelay.hpp"

int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.register_new_chan(std::string("meta:")+HUBO_INFO_META_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("info:")+HUBO_INFO_DATA_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");

    mgr.register_new_chan(std::string("command:")+HUBO_CMD_CHANNEL
                          +":10:4096:"+ACHD_INTERNAL_STRING+":");
    mgr.register_new_chan(std::string("aggregate:")+HUBO_AGG_CHANNEL
                          +":20:4096:"+ACHD_INTERNAL_STRING+":");
    mgr.register_new_chan(std::string("auxiliary:")+HUBO_AUX_CMD_CHANNEL
                          +":100:64:"+ACHD_PUSH_STRING+":");

    mgr.register_new_chan(std::string("joint_state:")+HUBO_JOINT_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("imu_state:")+HUBO_IMU_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("ft_state:")+HUBO_FT_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");

    mgr.register_new_chan(std::string("instruction:")+HUBO_PATH_INSTRUCTION_CHANNEL
                          +":5:64:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("trajectory:")+HUBO_PATH_INPUT_CHANNEL
                          +":3:65536:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("traj_rx_feedback:")+HUBO_PATH_FEEDBACK_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("player:")+HUBO_PATH_PLAYER_STATE_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");

    mgr.register_new_chan(std::string("log:")+HUBO_RT_LOG_RELAY_CHAN
                          +":10:4608:"+ACHD_PULL_STRING+":");


    mgr.register_new_proc(std::string("player")
                          + ":/usr/bin/huboplayer::");
    mgr.register_new_proc(std::string("log_publisher")
                          + ":/usr/bin/hubo_log_publisher::");


    std::string robot_type = "Hubo2Plus";
    for(int i=1; i<argc; ++i)
    {
        robot_type = argv[i];
    }

    std::string args;
    if(robot_type == "virtual")
        args = "virtual";

    mgr.register_new_proc(std::string("socketcan_interface")
                          +":/usr/bin/hubo_socketcan_interface:robot "+"DrcHubo "+args+":");
    mgr.save_current_config("DrcHubo");

    mgr.register_new_proc(std::string("socketcan_interface")
                          +":/usr/bin/hubo_socketcan_interface:robot "+"Hubo2Plus "+args+":");
    mgr.save_current_config("Hubo2Plus");

    if(robot_type != "virtual")
        mgr.load_config(robot_type);

    return 0;
}
