
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

    Hubo2PlusBasicJmc::_request_encoder_readings();
}

void Hubo2Plus5chJmc::_send_reference_commands()
{

}

bool Hubo2Plus5chJmc::_decode_encoder_reading(const can_frame_t &frame)
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

bool Hubo2Plus5chJmc::_decode_status_reading(const can_frame_t &frame)
{
    // TODO: Check can_dlc?
    for(size_t i=0; i<joints.size(); ++i)
    {
        size_t jnt = joints[i]->info.software_index;
        hubo_joint_status_t& status = _state->joints[jnt].status;

        uint8_t byte = frame.data[i];
        status.driver_on    = (byte>>0) & 0x01;
        status.control_on   = (byte>>1) & 0x01;
        status.control_mode = (byte>>2) & 0x01;
        status.limit_switch = (byte>>3) & 0x01;

        status.error.jam            = (byte>>4) & 0x01;
        status.error.pwm_saturated  = (byte>>5) & 0x01;
        status.error.big            = (byte>>6) & 0x01;
        status.error.encoder        = (byte>>7) & 0x01;
    }

    return true;
}




