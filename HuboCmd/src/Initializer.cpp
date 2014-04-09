
#include "../Initializer.hpp"
#include <stdio.h>

using namespace HuboCmd;

Initializer::Initializer(double timeout_sec)
{
    _channels_opened = false;
    receive_description(timeout_sec);
}

Initializer::Initializer(const HuboCan::HuboDescription& description)
{
    _channels_opened = false;
    load_description(description);
}

bool Initializer::ready() { return _description_loaded && _channels_opened; }

bool Initializer::receive_description(double timeout_sec)
{
    HuboCan::error_result_t result = _desc.receiveInfo(timeout_sec);
    _description_loaded = ( result == HuboCan::OKAY );
    return _description_loaded;
}

void Initializer::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _description_loaded = true;
}



bool Initializer::open_channels()
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

    return _channels_opened;
}

void Initializer::_clear_command()
{
    memset(&_cmd, 0, sizeof(_cmd));
}

bool Initializer::_send_command()
{
    ach_status_t result = ach_put(&_aux_cmd_chan, &_cmd, sizeof(_cmd));
    if( ACH_OK != result)
    {
        fprintf(stderr, "Error in sending auxiliary command: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        return false;
    }
    return true;
}

size_t Initializer::_jmc(size_t joint)
{
    if(joint >= _desc.joints.size())
        return InvalidIndex;
    return _desc.getJmcIndex(_desc.joints[joint]->info.jmc_name);
}

size_t Initializer::_hw_index(size_t joint)
{
    if(joint >= _desc.joints.size())
        return InvalidIndex;
    return _desc.getJointInfo(joint).hardware_index;
}

void Initializer::_set_jmc_info(size_t joint)
{
    _cmd.jmc = _jmc(joint);
    _cmd.joint = _hw_index(joint);
}

void Initializer::home_joint(size_t joint)
{
    _clear_command();
    _cmd.id = HOME_JOINT;
    _set_jmc_info(joint);
    _send_command();
}

void Initializer::home_all_joints()
{
    _clear_command();
    _cmd.id = HOME_ALL_JOINTS;
    _send_command();
}









