#ifndef DEVICESTRINGS_HPP
#define DEVICESTRINGS_HPP

#include <string>

namespace HuboCan {

//-----------------------------------------------------------------------------
// -- Joint string
//-----------------------------------------------------------------------------
const std::string joint_device_string = "Joint";


//-----------------------------------------------------------------------------
// -- JMC strings
//-----------------------------------------------------------------------------
const std::string jmc_device_string = "JMC";

// Hub2Plus JMC types
const std::string hubo2plus_1ch_type_string = "H2P_1CH";
const std::string hubo2plus_2ch_type_string = "H2P_2CH";
const std::string hubo2plus_nck_type_string = "H2P_NCK"; // Neck JMC
const std::string hubo2plus_5ch_type_string = "H2P_5CH";

// DrcHubo JMC types
const std::string drchubo_2ch_type_string = "DRC_2CH";
const std::string drchubo_3ch_type_string = "DRC_3CH";


//-----------------------------------------------------------------------------
// -- IMU strings
//-----------------------------------------------------------------------------
const std::string imu_sensor_string = "IMU";

// Normal IMU types
const std::string hubo2plus_imu_sensor_type_string = "H2P_IMU";
const std::string drchubo_imu_sensor_type_string = "DRC_IMU";

// Tilt sensor for feet
const std::string hubo2plus_tilt_sensor_type_string = "H2P_TILT";


//-----------------------------------------------------------------------------
// -- Force-Torque strings
//-----------------------------------------------------------------------------
const std::string ft_sensor_string = "FT";

const std::string hubo2plus_ft_sensor_type_string = "H2P_FT";
const std::string drchubo_ft_sensor_type_string = "DRC_FT";


//-----------------------------------------------------------------------------
// -- Meta string
//-----------------------------------------------------------------------------
// Used to specify the name of the robot, the nominal control frequency,
// and the number of CAN buses available
const std::string meta_device_string = "meta";

} // namespace HuboCan

#endif // DEVICESTRINGS_HPP
