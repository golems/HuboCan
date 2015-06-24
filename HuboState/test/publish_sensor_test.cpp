
#include "HuboState/State.hpp"
#include "unistd.h"

int main(int , char* [])
{
    HuboCan::HuboDescription desc;
    desc.parseFile("/home/grey/projects/HuboCan/devices/Hubo2Plus.dd");
    desc.broadcastInfo();

    HuboState::State state(desc);

    if(state.initialized())
    {
        std::cout << "Joint count: " << state.joints.size() << std::endl
                  << "IMU count:   " << state.imus.size() << std::endl
                  << "FT count:    " << state.force_torques.size() << std::endl;
    }

    if(!desc.okay())
        return 1;

    while(state.initialized())
    {
        for(size_t i=0; i<state.imus.size(); ++i)
        {
            hubo_imu_state_t& imu = state.imus[i];
            imu.angular_position[0] = 0.1;
            imu.angular_position[1] = 1.2;
            imu.angular_position[2] = 2.3;

            imu.angular_velocity[0] = 1.0;
            imu.angular_velocity[1] = 2.1;
            imu.angular_position[2] = 3.2;
        }

        for(size_t i=0; i<state.force_torques.size(); ++i)
        {
            hubo_ft_state_t& ft = state.force_torques[i];
            ft.force[0] = 0.12;
            ft.force[1] = 1.23;
            ft.force[2] = 2.34;

            ft.torque[0] = 4.32;
            ft.torque[1] = 3.21;
            ft.torque[2] = 2.10;
        }

        state.publish();
        usleep(1e6/200);
    }
}
