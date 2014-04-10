
#include "../Commander.hpp"
#include "math.h"
#include <iostream>

int main(int argc, char* argv[])
{
    HuboCmd::Commander cmd;

    StringArray joint_names;
    ValueArray joint_values;
    joint_names.push_back("RSP");
    joint_names.push_back("REP");
    joint_values.resize(joint_names.size());

    IndexArray indices = cmd.get_indices(joint_names);

    cmd.set_modes(indices, HUBO_CMD_RIGID);

    cmd.update();

    for(size_t i=0; i<indices.size(); ++i)
    {
        if(cmd.joints[indices[i]].position != 0)
        {
            std::cout << "Joint " << cmd.description().joints[indices[i]]->info.name
                      << " is not at 0, so we will not execute this test" << std::endl;
            return 1;
        }
    }

    double T = 10, start = cmd.get_time();
    double elapsed = 0;
    while(elapsed <= T)
    {
        joint_values[0] = M_PI/4.0*sin(elapsed/T);
        joint_values[1] = -M_PI/2.0*(1.0/2.0)*(1-cos(elapsed/T));

        cmd.set_positions(indices, joint_values);

        cmd.send_commands();
        HuboCan::error_result_t result = cmd.update();
        if(result != HuboCan::OKAY)
        {
            std::cout << "Update threw an error: " << result << std::endl;
            return 1;
        }
        elapsed = cmd.get_time() - start;
    }




    return 0;
}