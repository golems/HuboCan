
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

    size_t iter=0, count=1;
    while(can.pump())
    {
        aux.update();
        if(iter > 200)
        {
            std::cout << "lost: " << can.channel(0).net_lost_replies << ", "
                      << can.channel(1).net_lost_replies << std::endl;
            std::cout << "per iteration: " << (double)(can.channel(0).net_lost_replies)/count << ", "
                      << (double)(can.channel(1).net_lost_replies)/count << std::endl;
            std::cout << state.joints << "\n\n" << std::endl;
            iter = 0;
        }
        ++iter;
        ++count;
    }

    return 0;
}
