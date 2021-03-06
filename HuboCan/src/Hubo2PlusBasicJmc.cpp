/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "HuboCan/HuboJmc.hpp"
#include "HuboCan/HuboCanId.hpp"
#include "HuboState/State.hpp"
#include "HuboCmd/Aggregator.hpp"

namespace HuboCan {

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
        if(joints[i]->updated == false)
        {
            ++(joints[i]->dropped_count);
        }
    }
}

void Hubo2PlusBasicJmc::_request_encoder_readings()
{
    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GET_ENCODER;
    _frame.data[2] = 0;

    _frame.can_dlc = 3;

    for(size_t i=0; i<joints.size(); ++i)
    {
        joints[i]->updated = false;
        ++joints[i]->expected_replies;
    }

    _pump->add_frame(_frame, info.can_channel, 1);
}

void Hubo2PlusBasicJmc::_send_reference_commands()
{
    for(size_t i=0; i<joints.size(); ++i)
    {
        const hubo_joint_cmd_t& cmd = _agg->joint(joints[i]->info.software_index);
        if(cmd.mode == HUBO_CMD_RIGID)
        {
            _handle_rigid_reference_cmd();
            return; // All joints' reference commands get sent out with a single CAN frame
                    // so we quit as soon as a frame has been sent out
        }
    }
}

void Hubo2PlusBasicJmc::_handle_rigid_reference_cmd()
{
    if(joints.size() > 2)
    {
        std::cout << "Hubo2PlusBasicJmc named '" << info.name
                  << "' expected at most two joints, but instead has "
                  << joints.size() << std::endl;
        _pump->report_error();
        return;
    }

    can_frame_t frame; memset(&frame, 0, sizeof(frame));
    frame.can_id = REFERENCE_CMD + info.hardware_index;

    for(size_t i=0; i<joints.size(); ++i)
    {
        hubo_joint_cmd_t& cmd = _agg->joint(joints[i]->info.software_index);
        unsigned long reference = sign_convention_converter(
                    joints[i]->radian2encoder(cmd.position));

        for(size_t j=0; j<3; ++j)
        {
            frame.data[j + 3*i] = long_to_bytes(reference, j);
        }
        
        _state->joints[joints[i]->info.software_index].reference = cmd.position;
    }

    frame.can_dlc = 6;
    _pump->add_frame(frame, info.can_channel);
}

unsigned long Hubo2PlusBasicJmc::sign_convention_converter(int encoder_value)
{
    if(encoder_value < 0)
        return (unsigned long)( ((-encoder_value)&0x7FFFFF) | (1<<23) );

    return (unsigned long)encoder_value;
}

bool Hubo2PlusBasicJmc::decode(const can_frame_t& frame, size_t channel)
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
            ++joints[i]->received_replies;
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
    // TODO: Consider only doing one per cycle?
    while(_aux_commands.size() > 0)
    {
        _handle_auxiliary_command(_aux_commands.back());
        _aux_commands.pop_back();
    }
}

void Hubo2PlusBasicJmc::_handle_auxiliary_command(const hubo_aux_cmd_t& cmd)
{
    switch(cmd.cmd_id)
    {
        case HOME_JOINT:
            _handle_home_joint(cmd);
            break;

        case JOINT_CTRL_SWITCH:
            _handle_ctrl_switch(cmd);
            break;

        default:
            std::cerr << "[Hubo2PlusBasicJmc] Unknown/Unsupported auxiliary command type: "
                      << cmd.cmd_id << std::endl;
            break;
    }
}

void Hubo2PlusBasicJmc::_handle_home_joint(const hubo_aux_cmd_t& cmd)
{
    if(1 == cmd.all_devices)
    {
        _handle_home_all_joints();
        return;
    }

    if(cmd.component_id >= joints.size())
    {
        std::cerr << "Requested homing for invalid joint on " << info.name << " ("
                  << cmd.component_id << ". Max joint size is " << joints.size() << std::endl;
        return;
    }

    // Log the homing so we know that the command arrived
    std::cout << "Sending home command for joint " << joints[cmd.component_id]->info.name
    << " (" << joints[cmd.component_id]->info.software_index << ") " << " on board " << info.name
    << " (" << info.hardware_index << ")" << std::endl;

    memset(&_frame, 0, sizeof(_frame));

    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GOTO_HOME;
    _frame.data[2]  = ((cmd.component_id+1) << 4) | ( 0x01 << 1 );
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

    // ( 0x01 << 1 ) is used to specify that we want the joint to home according to
    // the settings which are stored in the firmware and to ignore all remaining bytes
    // in the CAN frame.

    // All other bytes are ignored, so we will leave them as 0.
    _frame.can_dlc = 8;

    _pump->add_frame(_frame, info.can_channel);

    // TODO: Decide what other bookkeeping should be done when a joint gets homed.
}

void Hubo2PlusBasicJmc::_handle_ctrl_switch(const hubo_aux_cmd_t& cmd)
{
    _frame.can_id   = CMD_BYTE;

    _frame.data[0]  = info.hardware_index;

    if(ENABLE == cmd.params[0])
    {
        std::cout << "Turning on motor control for joints:";
        for(size_t i=0; i<joints.size(); ++i)
            std::cout << " " << joints[i]->info.name;
        std::cout << std::endl;

        _frame.data[1] = SET_MOTOR_CTRL_ON;
    }
    else if(DISABLE == cmd.params[0])
    {
        std::cout << "Turning off motor control for joints:";
        for(size_t i=0; i<joints.size(); ++i)
            std::cout << " " << joints[i]->info.name;
        std::cout << std::endl;

        _frame.data[1] = SET_MOTOR_CTRL_OFF;
    }
    else
    {
        std::cerr << "[Hubo2PlusBasicJmc::_handle_ctrl_switch] Invalid param[0] value for cmd_id ("
                  << cmd.cmd_id << ": " << cmd.params[0] << "\n Must be equal to "
                  << ENABLE << " or " << DISABLE << std::endl;
        return;
    }

    _frame.can_dlc = 2;

    _pump->add_frame(_frame, info.can_channel);
}

} // namespace HuboCan
