
#include "HuboState/State.hpp"
#include "HuboCmd/AuxSender.hpp"

int main(int, char* [])
{
    HuboState::State state;
    HuboCmd::AuxSender sender;

    if(sender.ready())
    {
        sender.initialize_all_sensors();

        while(state.initialized())
        {
            state.update();

            std::cout << state.imus << std::endl;
        }
    }
}
