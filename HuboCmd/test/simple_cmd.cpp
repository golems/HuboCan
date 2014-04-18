
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

    std::cout << "Claiming joint" << std::endl;
    cmd.claim_joint(joint_index);
    cmd.send_commands();

    cmd.update();

    std::cout << "Setting mode" << std::endl;
    cmd.set_mode(joint_index, HUBO_CMD_RIGID);
    std::cout << "Setting position" << std::endl;
    cmd.set_position(joint_index, joint_value);
    cmd.send_commands();

    cmd.update();

    return 0;
}
