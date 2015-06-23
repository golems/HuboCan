
#include "../AuxSender.hpp"
#include <stdio.h>

using namespace HuboCmd;

AuxSender::AuxSender(bool initialize, double timeout_sec)
{
    _channels_opened = false;
    _clear_command();
    
    if(!initialize)
        return;
    
    receive_description(timeout_sec);
    open_channels();
}

AuxSender::AuxSender(const HuboCan::HuboDescription& description)
{
    _channels_opened = false;
    _clear_command();
    
    load_description(description);
    open_channels();
}

bool AuxSender::initialize(double timeout_sec)
{
    if(!_description_loaded)
        receive_description(timeout_sec);
    
    open_channels();
    
    return ready();
}

bool AuxSender::receive_description(double timeout_sec)
{
    HuboCan::error_result_t result = _desc.receiveInfo(timeout_sec);
    _description_loaded = ( result == HuboCan::OKAY );
    return _description_loaded;
}

void AuxSender::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _description_loaded = true;
}

bool AuxSender::open_channels()
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
    ach_flush(&_aux_cmd_chan);

    return _channels_opened;
}

void AuxSender::_clear_command()
{
    memset(&_cmd, 0, sizeof(_cmd));
    strcpy(_cmd.code, HUBO_AUX_CMD_HEADER_CODE);
}

bool AuxSender::_send_command()
{
    if(!_channels_opened)
    {
        std::cout << "WARNING: Cannot send auxiliary commands because the Initializer's ach channel "
                  << "has not been opened!" << std::endl;
    }

    ach_status_t result = ach_put(&_aux_cmd_chan, &_cmd, sizeof(_cmd));
    if( ACH_OK != result)
    {
        _channels_opened = false;
        fprintf(stderr, "Error in sending auxiliary command: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        return false;
    }
    return true;
}

size_t AuxSender::_jmc(size_t joint)
{
    if(joint >= _desc.joints.size())
        return InvalidIndex;
    return _desc.getJmcIndex(_desc.joints[joint]->info.jmc_name);
}

size_t AuxSender::_hw_index(size_t joint)
{
    if(joint >= _desc.joints.size())
        return InvalidIndex;
    return _desc.getJointInfo(joint).hardware_index;
}

void AuxSender::_set_jmc_info(size_t joint)
{
    _cmd.device_id = _jmc(joint);
    _cmd.component_id = _hw_index(joint);
}

void AuxSender::home_joint(size_t joint)
{
    _clear_command();
    _cmd.cmd_id = HOME_JOINT;
    _set_jmc_info(joint);
    _send_command();
}

void AuxSender::home_all_joints()
{
    _clear_command();
    _cmd.cmd_id = HOME_ALL_JOINTS;
    _send_command();
}

void AuxSender::initialize_sensor(size_t sensor)
{
    _clear_command();
    _cmd.cmd_id = INIT_SENSOR;
    _cmd.device_id = sensor;
    _send_command();
}

void AuxSender::initialize_all_imus()
{
    _clear_command();
    _cmd.cmd_id = INIT_ALL_IMUS;
    _send_command();
}

void AuxSender::initialize_all_fts()
{
    _clear_command();
    _cmd.cmd_id = INIT_ALL_FTS;
    _send_command();
}

void AuxSender::initialize_all_sensors()
{
    _clear_command();
    _cmd.cmd_id = INIT_ALL_SENSORS;
    _send_command();
}

bool AuxSender::ready() { return _description_loaded && _channels_opened; }
