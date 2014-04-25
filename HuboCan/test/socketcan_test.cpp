
#include "../SocketCanPump.hpp"
#include "../HuboDescription.hpp"
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

    for(size_t i=0; i<desc.jmcs.size(); ++i)
    {
        desc.jmcs[i]->assign_pointers(&agg, &state);
    }

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
