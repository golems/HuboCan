
#include "../Commander.hpp"
#include <stdlib.h>
#include <stdio.h>

#include <iostream>

using namespace HuboCmd;

Commander::Commander(double timeout) :
    HuboState::State(timeout)
{
    _initialize();
    _create_memory();
}

Commander::Commander(const HuboCan::HuboDescription& description) :
    HuboState::State(description)
{
    _initialize();
    _create_memory();
}

void Commander::_initialize()
{
    _construction = true;

    cmd_data = NULL;
    _compressed_data = NULL;

    _channels_opened = false;
    open_channels();

    _has_been_updated = false;
}

bool Commander::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_cmd_chan, HUBO_CMD_CHANNEL, NULL);
    if(ACH_OK != result)
    {
        fprintf(stderr, "Error opening command channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
    }
    ach_flush(&_cmd_chan);

    return _channels_opened;
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

    if(_construction)
    {
        _construction = false;
    }
    else
    {
        HuboState::State::_create_memory();
    }
}

Commander::~Commander()
{
    update(0);

    release_joints();
    send_commands();

    ach_close(&_cmd_chan);
    free(cmd_data);
    free(_compressed_data);
}

void Commander::release_joints()
{
    for(size_t i=0; i < get_joint_count(); ++i)
    {
        if(hubo_cmd_data_check_if_joint_is_set(cmd_data, i) == 1)
        {
            release_joint(i);
        }
    }
}

HuboCan::error_result_t Commander::release_joints(const IndexArray &joints)
{
    HuboCan::error_result_t result = HuboCan::OKAY;

    for(size_t i=0; i<joints.size(); ++i)
    {
        result |= release_joint(joints[i]);
    }

    return result;
}

HuboCan::error_result_t Commander::release_joint(size_t joint_index)
{
    if(hubo_cmd_data_check_if_joint_is_set(cmd_data, joint_index) == 1)
    {
        return set_mode(joint_index, HUBO_CMD_RELEASE);
    }
    else
    {
        std::cerr << "Warning: Attempting to release a joint which this process does not own. This will not do anything." << std::endl;
        return HuboCan::INCOMPATIBLE_JOINT;
    }

    return HuboCan::UNDEFINED_ERROR;
}

HuboCan::error_result_t Commander::claim_joints(const IndexArray &joints)
{
    HuboCan::error_result_t result = HuboCan::OKAY;

    for(size_t i=0; i<joints.size(); ++i)
    {
        result |= claim_joint(joints[i]);
    }

    return result;
}

HuboCan::error_result_t Commander::claim_joint(size_t joint_index)
{
    return set_mode(joint_index, HUBO_CMD_CLAIM);
}

HuboCan::error_result_t Commander::_register_joint(size_t joint_index)
{
    hubo_data_error_t result = hubo_cmd_data_register_joint(cmd_data, joint_index);
    if(HUBO_DATA_OUT_OF_BOUNDS == result)
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    else if(HUBO_DATA_READ_ONLY == result)
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

HuboCan::error_result_t Commander::set_modes(const IndexArray &joints, hubo_cmd_mode_t mode)
{
    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i<joints.size(); ++i)
    {
        result |= set_mode(joints[i], mode);
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

HuboCan::error_result_t Commander::update(double timeout_sec)
{
    _has_been_updated = true;

    return HuboState::State::update(timeout_sec);
}

HuboCan::error_result_t Commander::send_commands()
{
    if(!_has_been_updated)
    {
        fprintf(stderr, "Your Commander has not been updated since the last time you sent a command!\n");
        // TODO: Decide if I should terminate here or allow the command to proceed anyway
    }

    hubo_cmd_data_compressor(_compressed_data, cmd_data);

    ach_put(&_cmd_chan, _compressed_data, hubo_cmd_data_get_min_data_size(_compressed_data));

    hubo_cmd_data_unregister_released_joints(cmd_data);

    _has_been_updated = false;

    return HuboCan::OKAY;
}


















