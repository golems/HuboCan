
#include "../Commander.hpp"
#include <stdlib.h>

using namespace HuboCmd;

Commander::Commander(double timeout)
{
    _initialize();
    getDescription(timeout);
}

Commander::Commander(const HuboCan::HuboDescription& description)
{
    _initialize();
    getDescription(description);
}

void Commander::_initialize()
{
    cmd_data = NULL;
    _compressed_data = NULL;
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

bool Commander::getDescription(double timeout)
{
    int result = _desc.receiveInfo(timeout);
    _create_memory();
    return result;
}

void Commander::getDescription(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
}

Commander::~Commander()
{
    free(cmd_data);
}

HuboCan::error_result_t Commander::_setup_container(JointIndex joint)
{
    int joint_check = hubo_cmd_data_check_if_joint_is_set(cmd_data, joint);
    if( 1 == joint_check )
    {
        hubo_cmd_data_get_joint_cmd(&_container, cmd_data, joint);
    }
    else if( -1 == joint_check )
    {
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    else
    {
        memset(&_container, 0, sizeof(hubo_joint_cmd_t)); // TODO: Have this fill in current state information
    }

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::set_mode(JointIndex joint, hubo_cmd_mode_t mode)
{
    HuboCan::error_result_t result = _setup_container(joint);
    if(result != HuboCan::OKAY)
        return result;

    _container.mode = mode;
    hubo_cmd_data_set_joint_cmd(cmd_data, &_container, joint);

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::set_modes(IndexArray joints, ModeArray modes)
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

HuboCan::error_result_t Commander::position(JointIndex joint, float value)
{
    HuboCan::error_result_t result = _setup_container(joint);
    if(result != HuboCan::OKAY)
        return result;

    _container.position = value;
    hubo_cmd_data_set_joint_cmd(cmd_data, &_container, joint);

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::positions(IndexArray joints, ValueArray values)
{
    if(joints.size() != values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= position(joints[i], values[i]);
    }

    return result;
}

HuboCan::error_result_t Commander::base_torque(JointIndex joint, double value)
{
    HuboCan::error_result_t result = _setup_container(joint);
    if(result != HuboCan::OKAY)
        return result;

    _container.base_torque = value;
    hubo_cmd_data_set_joint_cmd(cmd_data, &_container, joint);

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::base_torques(IndexArray joints, ValueArray values)
{
    if(joints.size() != values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= base_torque(joints[i], values[i]);
    }

    return result;
}

HuboCan::error_result_t Commander::kP_gain(JointIndex joint, double kP_value)
{
    HuboCan::error_result_t result = _setup_container(joint);
    if(result != HuboCan::OKAY)
        return result;

    _container.kP_gain = kP_value;
    hubo_cmd_data_set_joint_cmd(cmd_data, &_container, joint);

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::kP_gains(IndexArray joints, ValueArray kP_values)
{
    if(joints.size() != kP_values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= kP_gain(joints[i], kP_values[i]);
    }

    return result;
}

HuboCan::error_result_t Commander::kD_gain(JointIndex joint, double kD_value)
{
    HuboCan::error_result_t result = _setup_container(joint);
    if(result != HuboCan::OKAY)
        return result;

    _container.kD_gain = kD_value;
    hubo_cmd_data_set_joint_cmd(cmd_data, &_container, joint);

    return HuboCan::OKAY;
}

HuboCan::error_result_t Commander::kD_gains(IndexArray joints, ValueArray kD_values)
{
    if(joints.size() != kD_values.size())
        return HuboCan::ARRAY_MISMATCH;

    HuboCan::error_result_t result = HuboCan::OKAY;
    for(size_t i=0; i < joints.size(); ++i)
    {
        result |= kD_gain(joints[i], kD_values[i]);
    }

    return result;
}

JointIndex Commander::getIndex(std::string joint_name)
{
    return _desc.getJointIndex(joint_name);
}

IndexArray Commander::getIndices(StringArray joint_names)
{
    return _desc.getJointIndices(joint_names);
}

HuboCan::error_result_t Commander::send_commands()
{


    return HuboCan::OKAY;
}


















