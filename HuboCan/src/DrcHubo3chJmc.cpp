
#include "../HuboJmc.hpp"
#include "HuboState/State.hpp"
#include "../HuboCanId.hpp"

using namespace HuboCan;

void DrcHubo3chJmc::_send_reference_commands()
{

}

bool DrcHubo3chJmc::_decode_encoder_reading(const can_frame_t& frame)
{
    // TODO: Decide if frame.can_dlc should be checked
    for(size_t i=0; i<2; ++i)
    {
        int16_t encoder = 0;
        encoder = (encoder << 8) + frame.data[1 + i*2];
        encoder = (encoder << 8) + frame.data[0 + i*2];

        size_t joint_index = joints[i]->info.software_index;
        _state->joints[joint_index].position =
                    joints[i]->encoder2radian(encoder);
    }

    return true;
}
