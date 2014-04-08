
#include "../HuboJmc.hpp"
#include "../HuboCanId.hpp"
#include "HuboState/State.hpp"

using namespace HuboCan;

Hubo2PlusBasicJmc::Hubo2PlusBasicJmc()
{
    _encoders_requested = false;
}

void Hubo2PlusBasicJmc::update()
{
    if(NULL == _pump)
        return;

    if(!_encoders_requested)
    {
        _request_encoder_readings();
        _encoders_requested = true;
    }

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
        return _decode_encoder_reading(frame);
    }
    else if( frame.can_id - STATUS_REPORT == info.hardware_index )
    {
        return _decode_status_reading(frame);
    }


    return false;
}

bool Hubo2PlusBasicJmc::_decode_encoder_reading(const can_frame_t& frame)
{
    if(frame.can_dlc == 8)
    {
        for(size_t i=0; i < joints.size(); ++i)
        {
            int32_t encoder = 0;
            for(int j=3; j >= 0; --j)
            {
                encoder = (encoder << 8) + frame.data[j + i*4];
            }

            size_t joint_index = joints[i]->info.software_index;
            _state->joints[joint_index].position =
                        joints[i]->encoder2radian(encoder);

            // TODO: Decide if velocity should be computed here
        }
        return true;
    }
    return false;
}

bool Hubo2PlusBasicJmc::_decode_status_reading(const can_frame_t &frame)
{
    // TODO: Check can_dlc?
    for(size_t i=0; i<joints.size(); ++i)
    {
        size_t jnt = joints[i]->info.software_index;
        hubo_joint_status_t& status = _state->joints[jnt].status;

        uint8_t byte = frame.data[4*i+0];
        status.driver_on    = (byte>>0) & 0x01;
        status.control_on   = (byte>>1) & 0x01;
        status.control_mode = (byte>>2) & 0x01;
        status.limit_switch = (byte>>3) & 0x01;
        status.home_flag    = (byte>>4) & 0x0F;

        byte = frame.data[4*i+1];
        status.error.jam            = (byte>>0) & 0x01;
        status.error.pwm_saturated  = (byte>>1) & 0x01;
        status.error.big            = (byte>>2) & 0x01;
        status.error.encoder        = (byte>>3) & 0x01;
        status.error.driver_fault   = (byte>>4) & 0x01;
        status.error.motor_fail_0   = (byte>>5) & 0x01;
        status.error.motor_fail_1   = (byte>>6) & 0x01;

        byte = frame.data[4*i+2];
        status.error.min_position   = (byte>>0) & 0x01;
        status.error.max_position   = (byte>>1) & 0x01;
        status.error.velocity       = (byte>>2) & 0x01;
        status.error.acceleration   = (byte>>3) & 0x01;
        status.error.temperature    = (byte>>4) & 0x01;
    }

    return true;
}











