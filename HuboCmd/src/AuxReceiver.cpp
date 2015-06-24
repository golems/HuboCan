
#include <stdio.h>
#include <iostream>

#include "../AuxReceiver.hpp"
#include "HuboCan/DeviceStrings.hpp"

namespace HuboCmd {

AuxReceiver::AuxReceiver(HuboCan::HuboDescription* description)
{
    _desc = description;
    _channels_opened = false;
    open_channels();
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

            if(_cmd.device_id < _desc->getJmcCount())
            {
                _register_command();
            }
            else
            {
                std::cout << "Requested an auxiliary command on an out-of-bounds JMC: "
                          << _cmd.device_id << " (max JMC index is " << _desc->getJmcCount()-1
                          << ")" << std::endl;
                continue;
            }
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

void AuxReceiver::_register_with_all_sensors()
{
    for(size_t i=0; i<_desc->sensors.size(); ++i)
    {
        _desc->sensors[i]->auxiliary_command(_cmd);
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
