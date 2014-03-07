
#include "../Commander.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace HuboCmd;

Commander::Commander(double timeout)
{
    _initialize();
    receive_description(timeout);
}

Commander::Commander(const HuboCan::HuboDescription& description)
{
    _initialize();
    load_description(description);
}

void Commander::_initialize()
{
    cmd_data = NULL;
    _compressed_data = NULL;

    _channel_opened = false;
    open_channel();

    _has_been_updated = false;
}

bool Commander::open_channel()
{
    if(_channel_opened)
        return true;

    ach_status_t result = ach_open(&_cmd_chan, HUBO_CMD_CHANNEL, NULL);
    if(ACH_OK != result)
    {
        fprintf(stderr, "Error opening command channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channel_opened = false;
    }
    else
    {
        _channel_opened = true;
    }

    return false;
}

void Commander::_create_memory()
{
    free(cmd_data);
    free(_compressed_data);
    if(_desc.getJointCount() > 0)
    {
        cmd_data = hubo_cmd_init_data( _desc.getJointCount() );
        _compressed_data = hubo_cmd_init_data( _desc.getJointCount() );
    }
    else
    {
        cmd_data = NULL;
        _compressed_data = NULL;
    }
}

bool Commander::receive_description(double timeout)
{
    int result = _desc.receiveInfo(timeout);
    _create_memory();
    return result;
}

void Commander::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
}

Commander::~Commander()
{
    ach_close(&_cmd_chan);
    free(cmd_data);
    free(_compressed_data);
}

HuboCan::error_result_t Commander::_register_joint(size_t joint_index)
{
    hubo_cmd_error_t result = hubo_cmd_data_register_joint(cmd_data, joint_index);
    if(CMD_ERR_OUT_OF_BOUNDS == result)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    else if(CMD_ERR_READ_ONLY == result)
    {
        return HuboCan::READ_ONLY;
    }

    return HuboCan::OKAY;
}

void Commander::_fill_container(size_t joint_index)
{
    memset(&_container, 0, sizeof(hubo_joint_cmd_t));
    hubo_cmd_data_get_joint_cmd(&_container, cmd_data, joint_index);
}

HuboCan::error_result_t Commander::set_mode(size_t joint_index, hubo_cmd_mode_t mode)
{
    hubo_joint_cmd_t* cptr = hubo_cmd_data_access_joint_cmd(cmd_data, joint_index);
    if(NULL == cptr)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    cptr->mode = mode;

    return _register_joint(joint_index);
}

HuboCan::error_result_t Commander::set_modes(const IndexArray& joints, const ModeArray& modes)
{
    if(joints.size() != modes.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i<joints.size(); ++i)
    {
        result |= set_mode(joints[i], modes[i]);
    }

    return result;
}

hubo_cmd_mode_t Commander::get_mode(size_t joint_index)
{
    _fill_container(joint_index);
    return _container.mode;
}

ModeArray Commander::get_modes(const IndexArray &joints)
{
    ModeArray modes;
    for(size_t i=0; i<joints.size(); ++i)
    {
        modes.push_back(get_mode(joints[i]));
    }
    return modes;
}

HuboCan::error_result_t Commander::set_position(size_t joint_index, float value)
{
    hubo_joint_cmd_t* cptr = hubo_cmd_data_access_joint_cmd(cmd_data, joint_index);
    if(NULL == cptr)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    cptr->position = value;

    return _register_joint(joint_index);
}

HuboCan::error_result_t Commander::set_positions(const IndexArray& joints,
                                                 const ValueArray& values)
{
    if(joints.size() != values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= set_position(joints[i], values[i]);
    }

    return result;
}

float Commander::get_position_cmd(size_t joint_index)
{
    _fill_container(joint_index);
    return _container.position;
}

ValueArray Commander::get_position_cmds(const IndexArray &joints)
{
    ValueArray positions;
    for(size_t i=0; i < joints.size(); ++i)
    {
        positions.push_back(get_position_cmd(joints[i]));
    }
    return positions;
}

HuboCan::error_result_t Commander::set_base_torque(size_t joint_index, double value)
{
    hubo_joint_cmd_t* cptr = hubo_cmd_data_access_joint_cmd(cmd_data, joint_index);
    if(NULL == cptr)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    cptr->base_torque = value;

    return _register_joint(joint_index);
}

HuboCan::error_result_t Commander::set_base_torques(const IndexArray& joints,
                                                    const ValueArray& values)
{
    if(joints.size() != values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= set_base_torque(joints[i], values[i]);
    }

    return result;
}

double Commander::get_base_torque_cmd(size_t joint_index)
{
    _fill_container(joint_index);
    return _container.base_torque;
}

ValueArray Commander::get_base_torque_cmds(const IndexArray &joints)
{
    ValueArray torques;
    for(size_t i=0; i<joints.size(); ++i)
    {
        torques.push_back(get_base_torque_cmd(joints[i]));
    }
    return torques;
}

HuboCan::error_result_t Commander::set_kp_gain(size_t joint_index, double kP_value)
{
    hubo_joint_cmd_t* cptr = hubo_cmd_data_access_joint_cmd(cmd_data, joint_index);
    if(NULL == cptr)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    cptr->kP_gain = kP_value;

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::set_kp_gains(const IndexArray& joints,
                                            const ValueArray& kP_values)
{
    if(joints.size() != kP_values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= set_kp_gain(joints[i], kP_values[i]);
    }

    return result;
}

double Commander::get_kp_gain_cmd(size_t joint_index)
{
    _fill_container(joint_index);
    return _container.kP_gain;
}

ValueArray Commander::get_kp_gain_cmds(const IndexArray &joints)
{
    ValueArray gains;
    for(size_t i=0; i<joints.size(); ++i)
    {
        gains.push_back(get_kp_gain_cmd(joints[i]));
    }
    return gains;
}

HuboCan::error_result_t Commander::set_kd_gain(size_t joint_index, double kD_value)
{
    hubo_joint_cmd_t* cptr = hubo_cmd_data_access_joint_cmd(cmd_data, joint_index);
    if(NULL == cptr)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    cptr->kD_gain = kD_value;

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::set_kd_gains(const IndexArray& joints, const ValueArray& kD_values)
{
    if(joints.size() != kD_values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= set_kd_gain(joints[i], kD_values[i]);
    }

    return result;
}

double Commander::get_kd_gain_cmd(size_t joint_index)
{
    _fill_container(joint_index);
    return _container.kD_gain;
}

ValueArray Commander::get_kd_gain_cmds(const IndexArray &joints)
{
    ValueArray gains;
    for(size_t i=0; i<joints.size(); ++i)
    {
        gains.push_back(get_kd_gain_cmd(joints[i]));
    }
    return gains;
}

size_t Commander::get_index(const std::string& joint_name)
{
    return _desc.getJointIndex(joint_name);
}

IndexArray Commander::get_indices(const StringArray& joint_names)
{
    return _desc.getJointIndices(joint_names);
}

HuboCan::error_result_t Commander::update()
{
    _has_been_updated = true;

    // TODO: Inherit the state class, and call its update function here
    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::send_commands()
{
    if(!_has_been_updated)
    {
        fprintf(stderr, "Your Commander has not been updated since the last time you sent a command!");
        // TODO: Decide if I should terminate here or allow the command to proceed anyway
    }

    hubo_cmd_data_compressor(_compressed_data, cmd_data);

    ach_put(&_cmd_chan, _compressed_data, hubo_cmd_data_get_min_data_size(_compressed_data));

    _has_been_updated = false;

    return HuboCan::OKAY;
}


















