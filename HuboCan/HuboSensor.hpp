#ifndef HUBOSENSOR_HPP
#define HUBOSENSOR_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include "CanDevice.hpp"
#include <vector>
#include <string>

const std::string imu_sensor_type_string = "imu";
const std::string ft_sensor_type_string = "force_torque";

namespace HuboCan {

class HuboSensor : public CanDevice
{
public:

    hubo_sensor_info_t info;

protected:

};

typedef std::vector<HuboSensor*> HuboSensorPtrArray;

} // namespace HuboCan

#endif // HUBOSENSOR_HPP
