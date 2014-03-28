
#include "../Receiver.hpp"
#include "../HuboData.hpp"

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
    free(_joint_data);
    free(_ft_data);
    free(_imu_data);
}

bool Receiver::receive_description(double timeout)
{
    int result = _desc.receiveInfo(timeout);
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
    open_channels();

    _joint_data = NULL;
    _ft_data = NULL;
    _imu_data = NULL;
}

void Receiver::_create_memory()
{
    free(_joint_data);
    free(_ft_data);
    free(_imu_data);

    _joint_data = initialize_data<hubo_joint_state_t>(_desc.getJointCount());

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

    _imu_data = initialize_data<hubo_imu_state_t>(imu_count);
    _ft_data  = initialize_data<hubo_imu_state_t>(ft_count);
}

bool Receiver::open_channels()
{
    // TODO: Open channels

    return true;
}

HuboCan::error_result_t Receiver::update()
{
    // TODO: Update

    return HuboCan::OKAY;
}
