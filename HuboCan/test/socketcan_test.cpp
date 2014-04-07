
#include "../SocketCanPump.hpp"
#include "../HuboDescription.hpp"
#include "HuboState/State.hpp"
#include "HuboCmd/Aggregator.hpp"

using namespace HuboCan;

int main(int argc, char* argv[])
{
    SocketCanPump can(200, 1e6, 2);

    HuboDescription desc;
    desc.parseFile("../HuboCan/misc/DrcHubo.dd");

    can.load_description(desc);

    HuboState::State state(desc);
    HuboCmd::Aggregator agg(desc);

    std::cout << state.joints << "\n\n" << std::endl;

    for(size_t i=0; i<desc.jmcs.size(); ++i)
    {
        desc.jmcs[i]->assign_pointers(&agg, &state);
    }

    size_t iter=0;
    while(can.pump())
    {
        if(iter > 200)
        {
            std::cout << "lost: " << can.channel(0).net_lost_replies << ", "
                      << can.channel(1).net_lost_replies << std::endl;
            std::cout << state.joints << "\n\n" << std::endl;
            iter = 0;
        }
        ++iter;
    }

    return 0;
}
