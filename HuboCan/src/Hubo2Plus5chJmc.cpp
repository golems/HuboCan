
#include "../HuboJmc.hpp"
#include "../HuboCanId.hpp"
#include "HuboState/State.hpp"

using namespace HuboCan;

void Hubo2Plus5chJmc::_request_encoder_readings()
{
    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GET_ENCODER;
    _frame.data[2]  = 1; // An extra request frame to get the last two fingers on the hand
    _frame.can_dlc  = 3;

    _pump->add_frame(_frame, info.can_channel, 1);
    // Minor gripe: The frames that get returned by this extra encoder request have the
    // same ID as frames which are returned by the standard encoder request, which means
    // the order in which they are received matters for interpreting them correctly. This
    // is very bad CAN protocol and could easily result in botched encoder readings for
    // the fingers.

    // UNLESS they correctly report their DLCs, in which case they can be distinguished.
    // This needs to be investigated.
    // Note: Based on the existing hubo-ach code, it seems the DLC might be set correctly.

    // The CanPump runs through the frames opposite of the order in which they're added, so
    // we're adding the extra frame first and expecting it to arrive second.


    // After we've sent our extra frame for the last two fingers, send the standard encoder
    // request frame
    Hubo2PlusBasicJmc::_request_encoder_readings();
}

void Hubo2Plus5chJmc::_send_reference_commands()
{

}

bool Hubo2Plus5chJmc::decode(const can_frame_t &frame, size_t channel)
{
    if( channel != info.can_channel )
        return false;

    if( frame.can_id - ENCODER_REPLY == info.hardware_index )
    {
        size_t start = 0;
        size_t end = 0;
        if(frame.can_dlc == 6)
        {
            start = 0;
            end = 3;
        }
        else if(frame.can_dlc == 4)
        {
            start = 3;
            end = 5;
        }

        if(joints.size() < end)
        {
            std::cout << "Expected 5 joints in a Hubo2Plus5chJmc, but it only had "
                      << joints.size() << "!" << std::endl;
            end = joints.size();
        }

        for(size_t i=start; i<end; ++i)
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

    return Hubo2PlusBasicJmc::decode(frame, channel);
}
