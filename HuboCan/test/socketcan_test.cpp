
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

    std::cout << "About to pump once" << std::endl;
    while(can.pump())
    {
        std::cout << "About to cout states" << std::endl;
        std::cout << state.joints << "\n\n" << std::endl;
        std::cout << "Did the cout on states" << std::endl;
    }

    return 0;
}
