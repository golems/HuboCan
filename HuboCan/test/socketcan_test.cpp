
#include "../SocketCanPump.hpp"
#include "../HuboDescription.hpp"
#include "HuboState/State.hpp"
#include "HuboCmd/Aggregator.hpp"
#include "HuboCmd/AuxReceiver.hpp"

using namespace HuboCan;

int main(int argc, char* argv[])
{
    SocketCanPump can(200, 1e6, 2);

    HuboDescription desc;
    desc.parseFile("../HuboCan/misc/DrcHubo.dd");
    desc.broadcastInfo();

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
        return 1;
    }

    agg.run();

    size_t iter=0, count=1;
    while(can.pump())
    {
        state.publish();
        aux.update();
        agg.update();
        if(iter > 600)
        {
            bool missed_one = false;
            for(size_t i=0; i<desc.joints.size(); ++i)
            {
                if(desc.joints[i]->dropped_count > 0)
                {
                    std::cout << "Dropped " << desc.joints[i]->info.name << ":"
                              << desc.joints[i]->dropped_count << "\t";
                    missed_one = true;
                }
            }
            if(missed_one)
                std::cout << std::endl;
            iter = 0;
        }
//        bool missed_one = false;
//        for(size_t i=0; i<desc.joints.size(); ++i)
//        {
//            if(!desc.joints[i]->updated)
//            {
//                std::cout << "Dropped " << desc.joints[i]->info.name << ":"
//                          << desc.joints[i]->dropped_count << "\t";
//                missed_one = true;
//            }
//        }
//        if(missed_one)
//            std::cout << std::endl;

        ++iter;
        ++count;
    }

    return 0;
}
