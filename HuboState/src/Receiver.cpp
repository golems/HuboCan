
#include "../Receiver.hpp"

extern "C"{
#include <stdlib.h>
}

using namespace HuboState;


Receiver::Receiver(double timeout)
{
    _initialize();
    receive_description(timeout);
}

Receiver::Receiver(const HuboCan::HuboDescription &description)
{
    _initialize();
    load_description(description);
}

Receiver::~Receiver()
{
    free(_last_cmd_data);
}

bool Receiver::receive_description(double timeout_sec)
{
    int result = _desc.receiveInfo(timeout_sec);
    _create_memory();
    return result;
}

void Receiver::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
}

void Receiver::_initialize()
{
    _channels_opened = false;
    _last_cmd_data = NULL;
}

void Receiver::_create_memory()
{
    free(_last_cmd_data);

    _last_cmd_data = hubo_cmd_init_data(_desc.getJointCount());
    joints.initialize(_desc.getJointCount(), HUBO_JOINT_SENSOR_CHANNEL);

    size_t imu_count = 0;
    size_t ft_count = 0;

    for(size_t i=0; i < _desc.sensors.size(); ++i)
    {
        std::string type(_desc.sensors[i]->info.name);

        if(type == imu_sensor_type_string)
        {
            ++imu_count;
        }
        else if(type == ft_sensor_type_string)
        {
            ++ft_count;
        }
    }

    imus.initialize(imu_count, HUBO_IMU_SENSOR_CHANNEL);
    force_torques.initialize(ft_count, HUBO_FT_SENSOR_CHANNEL);
}

HuboCan::error_result_t Receiver::update(double timeout_sec)
{
    bool success = true;
    success &= joints.receive_data(timeout_sec);
    success &= imus.receive_data(0);
    success &= force_torques.receive_data(0);

    if(success)
        return HuboCan::OKAY;
    else
        return HuboCan::ACH_ERROR;
}
