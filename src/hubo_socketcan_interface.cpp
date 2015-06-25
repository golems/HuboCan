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

#include "HuboCan/SocketCanPump.hpp"
#include "HuboCan/HuboDescription.hpp"
#include "HuboState/State.hpp"
#include "HuboCmd/Aggregator.hpp"
#include "HuboCmd/AuxReceiver.hpp"

using namespace HuboCan;

int main(int argc, char* argv[])
{
    bool terminal = false;
    for(int i=1; i<argc; ++i)
    {
        if(strcmp(argv[i], "terminal") == 0)
        {
            std::cout << "terminal flag noticed -- will run in terminal mode" << std::endl;
            terminal = true;
        }
    }

    HuboRT::Daemonizer rt;
    if(!terminal)
    {
        if(!rt.begin("socketcan_interface", 49))
        {
            return 1;
        }
    }

    bool virtual_can = false;
    double frequency_override = 0;
    std::string robot_name = "Hubo2Plus";
    for(int i=1; i<argc; ++i)
    {
        if(strcmp(argv[i],"virtual")==0)
        {
            std::cout << "virtual flag noticed -- will run in virtual can mode" << std::endl;
            virtual_can = true;
        }
        else if(strcmp(argv[i],"robot")==0)
        {
            if(i+1 >= argc)
            {
                std::cerr << "The 'robot' argument must be followed by a robot name!" << std::endl;
            }
            else
            {
                robot_name = argv[i+1];
            }
        }
        else if(strcmp(argv[i],"frequency")==0)
        {
            if(i+1 >= argc)
            {
                std::cout << "The 'frequency' argument must be followed by a value!" << std::endl;
            }
            else
            {
                frequency_override = atof(argv[i+1]);
            }
        }
    }

    HuboDescription desc;
    std::string file_name = "/opt/hubo/devices/" + robot_name + ".dd";
    if(!desc.parseFile(file_name))
    {
        std::cout << "Description for '" << robot_name << "' (" << file_name << ") "
                  << "could not be correctly parsed! Quitting!" << std::endl;
        return 2;
    }

    if(frequency_override > 0)
        desc.params.frequency = frequency_override;

    desc.broadcastInfo();


    SocketCanPump can(desc.params.frequency, 1e6, desc.params.can_bus_count, 1000, virtual_can);

    can.load_description(desc);

    HuboState::State state(desc);
    HuboCmd::Aggregator agg(desc);
    HuboCmd::AuxReceiver aux(&desc);

    if(!state.initialized())
    {
        std::cout << "State was not initialized correctly, so we are quitting.\n"
                  << " -- Either your ach channels are not open"
                  << " or your HuboDescription was not valid!\n" << std::endl;
        return 3;
    }

    agg.run();

    std::cout << "Beginning control loop" << std::endl;
    while(can.pump() && rt.good())
    {
        state.publish();
        aux.update();
        agg.update();
    }

    std::cout << "Shutting down Socket CAN Pump" << std::endl;
}
