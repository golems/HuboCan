#ifndef HUBOSENSOR_HPP
#define HUBOSENSOR_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include "CanDevice.hpp"
#include <vector>

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
