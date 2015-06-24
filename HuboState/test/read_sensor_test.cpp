
#include "HuboState/State.hpp"
#include "HuboCmd/AuxSender.hpp"

int main(int, char* [])
{
    HuboState::State state;
    HuboCmd::AuxSender sender;

    std::cout << "Joint count: " << state.joints.size() << std::endl
              << "IMU count:   " << state.imus.size() << std::endl
              << "FT  count:   " << state.force_torques.size() << std::endl;

    if(sender.ready())
    {
        sender.initialize_all_sensors();

        while(state.initialized())
        {
            state.update();

            std::cout << state.imus << std::endl;
            std::cout << state.force_torques << std::endl;
        }
    }
}
