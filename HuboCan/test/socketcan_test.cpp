
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

    for(size_t i=0; i<desc.jmcs.size(); ++i)
    {
        desc.jmcs[i]->assign_pointers(&agg, &state);
    }

    while(can.pump())
    {
        std::cout << state.joints << "\n\n" << std::endl;
    }

    return 0;
}
