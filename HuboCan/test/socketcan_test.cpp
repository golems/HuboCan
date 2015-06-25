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
#include "HuboRT/Daemonizer.hpp"

using namespace HuboCan;

int main(int argc, char* argv[])
{
    bool virtual_can = false;
    for(int i=1; i<argc; ++i)
    {
        if(strcmp(argv[i],"virtual")==0)
        {
            virtual_can = true;
        }
    }

    HuboDescription desc;
    if(!desc.parseFile("../HuboCan/devices/DrcHubo.dd"))
    {
        std::cout << "Description could not be correctly parsed! Quitting!" << std::endl;
        return 1;
    }
    desc.broadcastInfo();
    
    double frequency = 200;
    SocketCanPump can(frequency, 1e6, 2, 1000, virtual_can);

    HuboRT::Daemonizer rt;
    rt.redirect_signals();
    rt.prioritize(45);

    can.load_description(desc);

    HuboState::State state(desc);
    HuboCmd::Aggregator agg(desc);
    HuboCmd::AuxReceiver aux(&desc);

    std::cout << state.joints << "\n\n" << std::endl;

    if(!state.initialized())
    {
        std::cout << "State was not initialized correctly, so we are quitting.\n"
                  << " -- Either your ach channels are not open"
                  << " or your HuboDescription was not valid!\n" << std::endl;
        return 2;
    }

    agg.run();

    size_t iter=0, count=1;
    while(can.pump() && rt.good())
    {
        state.publish();
        aux.update();
        agg.update();
//        if(iter > frequency*10)
//        {
//            bool missed_one = false;
//            std::cout << " ----------------------------------- " << std::endl;
//            for(size_t i=0; i<desc.joints.size(); ++i)
//            {
//                if(desc.joints[i]->dropped_count > 0 && !virtual_can)
//                {
//                    std::cout << desc.joints[i]->info.name << ":"
//                              << desc.joints[i]->dropped_count
////                              << "(" << (double)(desc.joints[i]->dropped_count)/(double)(count)*100
//                              << "(" << desc.joints[i]->expected_replies - desc.joints[i]->received_replies
//                              << ")" << "\t";
//                    missed_one = true;
//                }
//            }
//            if(missed_one)
//                std::cout << std::endl;
//            iter = 0;
//        }

        ++iter;
        ++count;
    }

    return 0;
}
