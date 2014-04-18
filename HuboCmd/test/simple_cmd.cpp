
#include "../Commander.hpp"
#include "math.h"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCmd::Commander cmd;
    if(!cmd.initialized())
    {
        std::cout << "Commander was not initialized successfully!" << std::endl;
        return 1;
    }

    if(argc != 3)
    {
        std::cout << "Usage: ./simple_cmd <joint_name> <joint_value>" << std::endl;
        return 2;
    }

    std::string joint_name(argv[1]);
    size_t joint_index = cmd.get_index(joint_name);
    double joint_value = atof(argv[2]);

    cmd.update();

    cmd.claim_joint(joint_index);
    cmd.send_commands();

    cmd.update();

    cmd.set_mode(joint_index, HUBO_CMD_RIGID);
    cmd.set_position(joint_index, joint_value);
    cmd.send_commands();

    return 0;
}
