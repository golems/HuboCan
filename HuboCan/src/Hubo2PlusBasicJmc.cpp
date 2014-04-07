
#include "../HuboJmc.hpp"
#include "../HuboCanId.hpp"
#include "HuboState/State.hpp"

using namespace HuboCan;

void Hubo2PlusBasicJmc::update()
{
    if(NULL == _pump)
        return;

    _request_encoder_readings();
    _send_reference_commands();
}

void Hubo2PlusBasicJmc::_request_encoder_readings()
{
    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GET_ENCODER;
    _frame.data[2] = 0;

    _frame.can_dlc = 3;

    _pump->add_frame(_frame, info.can_channel, 1);
}

void Hubo2PlusBasicJmc::_send_reference_commands()
{
    // TODO
}

bool Hubo2PlusBasicJmc::decode(const can_frame_t &frame, size_t channel)
{
    if( channel != info.can_channel )
        return false;

    if( frame.can_id - ENCODER_REPLY == info.hardware_index )
    {
        if(frame.can_dlc == 8)
        {
            std::cout << "jmc name: " << info.name << " | joint count: " << joints.size() << std::endl;

            for(size_t i=0; i < joints.size(); ++i)
            {
                int32_t encoder = 0;
                for(int j=3; j >= 0; --j)
                {
                    std::cout << "frame index: " << j + i*4 << std::endl;
                    encoder = (encoder << 8) + frame.data[j + i*4];
                }

                std::cout << "checking joint index..." << std::endl;
                size_t joint_index = joints[i]->info.software_index;
                std::cout << "joint_index: " << joint_index << std::endl;
                _state->joints[joint_index].position =
                            joints[i]->encoder2radian(encoder);

                // TODO: Decide if velocity should be computed here
            }
            return true;
        }
        return false;
    }



    return false;
}
