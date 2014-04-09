
#include "../AuxReceiver.hpp"
#include <stdio.h>
#include <iostream>

using namespace HuboCmd;

AuxReceiver::AuxReceiver(HuboCan::HuboDescription *description)
{
    _desc = description;
    _channels_opened = false;
    open_channels();
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
        ach_flush(&_aux_cmd_chan);
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

            if(_cmd.jmc < _desc->getJmcCount())
            {
                _register_command();
            }
            else
            {
                std::cout << "Requested an auxiliary command on an out-of-bounds JMC: "
                          << _cmd.jmc << " (max JMC index is " << _desc->getJmcCount()-1
                          << ")" << std::endl;
                continue;
            }
        }

    }

    return true;
}

void AuxReceiver::_register_command()
{
    switch(_cmd.id)
    {
        case HOME_ALL_JOINTS:

            _register_with_all_jmcs();
            break;
        default: _desc->jmcs[_cmd.jmc]->auxiliary_command(_cmd); break;
    }
}

void AuxReceiver::_register_with_all_jmcs()
{
    for(size_t i=0; i<_desc->jmcs.size(); ++i)
    {
        _desc->jmcs[i]->auxiliary_command(_cmd);
    }
}
