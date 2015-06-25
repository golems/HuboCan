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

#include <stdio.h>
#include <iostream>

#include "HuboCmd/AuxReceiver.hpp"
#include "HuboCan/DeviceStrings.hpp"

namespace HuboCmd {

AuxReceiver::AuxReceiver(HuboCan::HuboDescription* description)
{
    _desc = description;
    _channels_opened = false;
    std::cout << "[AuxReceiver] Opening channels" << std::endl;
    open_channels();
    std::cout << "[AuxReceiver] Organizing sensors" << std::endl;
    _organize_sensors();
}

bool AuxReceiver::ready() { return _channels_opened; }

bool AuxReceiver::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_aux_cmd_chan, HUBO_AUX_CMD_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        fprintf(stderr, "Error opening auxiliary command channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
    }
    else
    {
        report_ach_errors(ach_flush(&_aux_cmd_chan), "AuxReceiver::open_channels",
                          "ach_flush", HUBO_AUX_CMD_CHANNEL);
    }

    return _channels_opened;
}

bool AuxReceiver::update()
{
    if(!_channels_opened)
    {
        std::cout << "Attempting to receive auxiliary commands when the channel was not "
                  << "successfully opened!" << std::endl;
        return false;
    }

    size_t fs;
    ach_status_t result = ACH_OK;
    while( ACH_OK == result || ACH_MISSED_FRAME == result )
    {
        result = ach_get(&_aux_cmd_chan, &_cmd, sizeof(_cmd), &fs, NULL, 0);
        if(ACH_STALE_FRAMES == result)
        {
            break;
        }
        else if( ACH_OK != result && ACH_MISSED_FRAME != result )
        {
            fprintf(stdout, "Unexpected Ach Result in auxiliary command channel: %s (%d)\n",
                    ach_result_to_string(result), (int)result); fflush(stdout);
            return false;
        }
        else
        {
            if( hubo_aux_cmd_header_check(&_cmd) != HUBO_DATA_OKAY )
            {
                continue;
            }

            _register_command();
        }
    }

    return true;
}

void AuxReceiver::_organize_sensors()
{
    if(NULL == _desc)
        return;

    for(size_t i=0; i<_desc->sensors.size(); ++i)
    {
        HuboCan::HuboSensor* sensor = _desc->sensors[i];
        if(     HuboCan::imu_sensor_string == sensor->info.sensor)
        {
            if(HuboCan::HuboImu* imu = dynamic_cast<HuboCan::HuboImu*>(sensor))
            {
                _imus.push_back(imu);
            }
            else
            {
                std::cerr << "[AuxReceiver::_organize_sensors] Could not convert sensor named ["
                          << sensor->info.name << "] into an IMU even though it is labelled as "
                          << "an IMU of type [" << sensor->info.type << "]. Please report this as "
                          << "a bug!" << std::endl;
            }
        }
        else if(HuboCan::ft_sensor_string == sensor->info.sensor)
        {
            if(HuboCan::HuboFt* ft = dynamic_cast<HuboCan::HuboFt*>(sensor))
            {
                _fts.push_back(ft);
            }
            else
            {
                std::cerr << "[AuxReceiver::_organize_sensors] Could not convert sensor named ["
                          << sensor->info.name << "] into a force-torque sensor even though it is "
                          << "labelled as an FT sensor of type [" << sensor->info.type << "]. "
                          << "Please report this as a bug!" << std::endl;
            }
        }
    }
}

void AuxReceiver::_register_command()
{
    switch(_cmd.cmd_id)
    {
        case HOME_ALL_JOINTS:

            _register_with_all_jmcs();
            break;

        case HOME_JOINT:

            _desc->jmcs[_cmd.device_id]->auxiliary_command(_cmd);
            break;

        case INIT_ALL_SENSORS:

            std::cout << "received INIT_ALL_SENSORS command!" << std::endl;
            _register_with_all_sensors();
            break;

        case INIT_ALL_IMUS:

            _register_with_all_imus();
            break;

        case INIT_ALL_FTS:

            _register_with_all_fts();
            break;

        case INIT_SENSOR:

            _desc->sensors[_cmd.device_id]->auxiliary_command(_cmd);
            break;
    }
}

void AuxReceiver::_register_with_all_jmcs()
{
    for(size_t i=0; i<_desc->jmcs.size(); ++i)
    {
        _desc->jmcs[i]->auxiliary_command(_cmd);
    }
}

void AuxReceiver::_register_with_jmc(size_t index)
{
    if(index < _desc->jmcs.size())
    {
        _desc->jmcs[index]->auxiliary_command(_cmd);
    }
    else
    {
        if(_desc->getJmcCount() > 0)
        {
            std::cerr << "Requested an auxiliary command on an out-of-bounds JMC: "
                      << _cmd.device_id << " (max JMC index is " << _desc->getJmcCount()-1
                      << ")" << std::endl;
        }
        else
        {
            std::cerr << "Requested an auxiliary command on a JMC (" << _cmd.device_id
                      << "), but there are not any JMCs available!" << std::endl;
        }
    }
}

void AuxReceiver::_register_with_all_sensors()
{
    for(size_t i=0; i<_desc->sensors.size(); ++i)
    {
        _desc->sensors[i]->auxiliary_command(_cmd);
    }
}

void AuxReceiver::_register_with_sensor(size_t index)
{
    if(index < _desc->sensors.size())
    {
        _desc->sensors[index]->auxiliary_command(_cmd);
    }
    else
    {
        if(_desc->sensors.size() > 0)
        {
            std::cerr << "Requested an auxiliary command on an out-of-bounds Sensor: "
                      << _cmd.device_id << " (max Sensor index is " << _desc->sensors.size() - 1
                      << ")" << std::endl;
        }
        else
        {
            std::cerr << "Requested an auxiliary command on a Sensor (" << _cmd.device_id
                      << "), but there are not any Sensors available!" << std::endl;
        }
    }
}

void AuxReceiver::_register_with_all_imus()
{
    for(size_t i=0; i<_imus.size(); ++i)
    {
        _imus[i]->auxiliary_command(_cmd);
    }
}

void AuxReceiver::_register_with_all_fts()
{
    for(size_t i=0; i<_fts.size(); ++i)
    {
        _fts[i]->auxiliary_command(_cmd);
    }
}

} // namespace HuboCmd
