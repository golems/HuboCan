
#include "../State.hpp"

extern "C"{
#include <stdlib.h>
}

using namespace HuboState;


State::State(double timeout)
{
    _initialize();
    receive_description(timeout);
}

State::State(const HuboCan::HuboDescription &description)
{
    _initialize();
    load_description(description);
}

State::~State()
{
    free(_last_cmd_data);
}

bool State::receive_description(double timeout_sec)
{
    HuboCan::error_result_t result = _desc.receiveInfo(timeout_sec);
    _initialized = result == HuboCan::OKAY;
    _create_memory();
    return _initialized;
}

void State::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
    _initialized = true;
}

void State::_initialize()
{
    _channels_opened = false;
    _last_cmd_data = NULL;
}

void State::_create_memory()
{
    free(_last_cmd_data);

    _last_cmd_data = hubo_cmd_init_data(_desc.getJointCount());
    joints.initialize(_desc.getJointNames(), HUBO_JOINT_SENSOR_CHANNEL);

    std::vector<std::string> imu_names;
    std::vector<std::string> ft_names;
    for(size_t i=0; i < _desc.sensors.size(); ++i)
    {
        std::string type(_desc.sensors[i]->info.type);

        if(type == imu_sensor_type_string)
        {
            imu_names.push_back(_desc.sensors[i]->info.name);
        }
        else if(type == ft_sensor_type_string)
        {
            ft_names.push_back(_desc.sensors[i]->info.name);
        }
    }

    imus.initialize(imu_names, HUBO_IMU_SENSOR_CHANNEL);
    force_torques.initialize(ft_names, HUBO_FT_SENSOR_CHANNEL);
}

HuboCan::error_result_t State::update(double timeout_sec)
{
    bool success = true;
    success &= joints.receive_data(timeout_sec);
    success &= imus.receive_data(0);
    success &= force_torques.receive_data(0);

    if( joints.get_time() != imus.get_time()
     || joints.get_time() != force_torques.get_time())
    {
        return HuboCan::SYNCH_ERROR;
    }

    if(success)
        return HuboCan::OKAY;
    else
        return HuboCan::ACH_ERROR;
}

HuboCan::error_result_t State::publish()
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double timestamp = (double)(time.tv_sec);
    timestamp += (double)(time.tv_nsec)/1.0e9;

    bool success = true;
    success &= force_torques.send_data(timestamp);
    success &= imus.send_data(timestamp);
    success &= joints.send_data(timestamp);

    if(success)
        return HuboCan::OKAY;
    else
        return HuboCan::ACH_ERROR;
}

double State::get_time()
{
    return joints.get_time();
}
