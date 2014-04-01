#ifndef HUBO_SENSOR_STREAM_HPP
#define HUBO_SENSOR_STREAM_HPP

extern "C" {
#include "hubo_sensor_c.h"
}

#include <iostream>

// TODO
//inline std::ostream& operator<<(std::ostream& stream, const hubo_joint_status& status)
//{

//}

std::ostream& operator<<(std::ostream& stream, const hubo_joint_state_t& state);
std::ostream& operator<<(std::ostream& stream, const hubo_imu_state_t& imu);
std::ostream& operator<<(std::ostream& stream, const hubo_ft_state_t& ft);


#endif // HUBO_SENSOR_STREAM_HPP
