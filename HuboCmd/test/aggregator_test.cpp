
#include "../Aggregator.hpp"
#include <iostream>


size_t simple_min(size_t x, size_t y)
{
    return (x < y) ? x : y;
}

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");
    HuboCmd::Aggregator agg(desc);

    agg.run();

    while(true)
    {
        sleep(1);
        const HuboCmd::JointCmdArray& cmds = agg.update();
        size_t stop = simple_min(10, desc.getJointCount());
        for(size_t i=0; i < stop; ++i)
        {
            std::cout.width(12);
            std::cout << cmds[i].position;
        }
        std::cout << std::endl;
    }

    return 0;
}
