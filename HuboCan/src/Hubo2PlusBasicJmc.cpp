
#include "../HuboJmc.hpp"
#include "../HuboCanId.hpp"
#include "HuboState/State.hpp"

using namespace HuboCan;

Hubo2PlusBasicJmc::Hubo2PlusBasicJmc()
{
    _startup = true;
}

void Hubo2PlusBasicJmc::update()
{
    if(NULL == _pump)
        return;

    _cycle_reset();

    if(_startup)
    {
        // TODO: handle any startup routines here
        // such as grabbing the first status reading for the JMC

        _startup = false;
    }

    if(_aux_commands.size() > 0)
    {
        _process_auxiliary_commands();
        // If there are auxiliary commands that need to be handled, we put off requesting
        // encoder readings and sending position commands in order to reduce the load on
        // the CAN bus
    }
    else
    {
        _request_encoder_readings();
        _send_reference_commands();
    }
}

void Hubo2PlusBasicJmc::_cycle_reset()
{
    for(size_t i=0; i<joints.size(); ++i)
    {
        joints[i]->updated = false;
    }
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

            joints[i]->updated = true;
        }
        return true;
    }
    return false;
}

bool Hubo2PlusBasicJmc::_decode_status_reading(const can_frame_t& frame)
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


void Hubo2PlusBasicJmc::_process_auxiliary_commands()
{
    while(_aux_commands.size() > 0)
    {
        _handle_auxiliary_command(_aux_commands.back());
        _aux_commands.pop_back();
    }
}

void Hubo2PlusBasicJmc::_handle_auxiliary_command(const hubo_aux_cmd_t& cmd)
{
    switch(cmd.id)
    {
        case HOME_JOINT:
            _handle_home_joint(cmd); break;
        case HOME_ALL_JOINTS:
            _handle_home_all_joints(); break;

        default:
            std::cerr << "Unknown/Unsupported auxiliary command type: " << cmd.id << std::endl;
            break;
    }
}

void Hubo2PlusBasicJmc::_handle_home_joint(const hubo_aux_cmd_t& cmd)
{
    if(cmd.joint >= joints.size())
    {
        std::cerr << "Requested homing for invalid joint on " << info.name << " ("
                  << cmd.joint << ". Max joint size is " << joints.size() << std::endl;
        return;
    }
    std::cout << "Sending home command for joint " << joints[cmd.joint]->info.name
    << " (" << joints[cmd.joint]->info.software_index << ") " << " on board " << info.name
    << " (" << info.hardware_index << ")" << std::endl;

    memset(&_frame, 0, sizeof(_frame));

    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GOTO_HOME;
    _frame.data[2]  = ((cmd.joint+1) << 4) | ( 0x01 << 1 );
    // Why do we add 1 to the joint index? This does not seem necessary according to
    // the CAN documentation. Does the firmware index the joints starting at 1 instead
    // of 0? According to experimental observation, IT DOES!

    // ( 0x01 << 1 ) is used to specify that we want the joint to home according to
    // the settings which are stored in the firmware and to ignore all remaining bytes
    // in the CAN frame.

    // All other bytes are ignored, so we will leave them as 0.
    _frame.can_dlc = 8;

    _pump->add_frame(_frame, info.can_channel);

    // TODO: Decide what other bookkeeping should be done when a joint gets homed.
}

void Hubo2PlusBasicJmc::_handle_home_all_joints()
{
    for(size_t i=0; i<joints.size(); ++i)
    {
        std::cout << "Sending home command for joint " << joints[i]->info.name
        << " (" << joints[i]->info.software_index << ") " << " \ton board " << info.name
        << " (" << info.hardware_index << ")" << std::endl;
    }

    memset(&_frame, 0, sizeof(_frame));

    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GOTO_HOME;
    _frame.data[2]  = ( 0x0F << 4 ) | ( 0x01 << 1 );

    // TODO: According to the CAN documentation 0x0F will instruct all joints to home,
    // but this has never been tested by us Linux users.

    // ( 0x01 << 1 ) is used to specify that we want the joint to home according to
    // the settings which are stored in the firmware and to ignore all remaining bytes
    // in the CAN frame.

    // All other bytes are ignored, so we will leave them as 0.
    _frame.can_dlc = 8;

    _pump->add_frame(_frame, info.can_channel);

    // TODO: Decide what other bookkeeping should be done when a joint gets homed.
}




