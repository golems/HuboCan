
#include "../Commander.hpp"
#include "math.h"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("../HuboCan/misc/Hubo2Plus.dd");
    HuboCmd::Commander cmd(desc);

    for(size_t i=0; i<cmd.get_joint_count()/4; ++i)
    {
        cmd.set_position(i, (float)(i)*M_PI/180.0);
        std::cout.width(10);
        std::cout << (float)(i)*M_PI/180.0;
    }

    std::cout << std::endl;

    for(size_t i=0; i<cmd.get_joint_count()/4; ++i)
    {
        std::cout.width(10);
        std::cout << cmd.get_position_cmd(i);
    }

    cmd.update();
    cmd.send_commands();

    std::cout << std::endl;

    return 0;
}
