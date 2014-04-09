
#include "../Manager.hpp"
#include <iostream>

#include "HuboCmd/Commander.hpp"
#include "HuboCmd/Aggregator.hpp"

extern "C"{
#include "HuboState/hubo_sensor_c.h"
}

int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.register_new_chan(std::string("meta:")+HUBO_INFO_META_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("info:")+HUBO_INFO_DATA_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("command:")+HUBO_CMD_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("aggregate:")+HUBO_AGG_CHANNEL+":20:4096:");
    mgr.register_new_chan(std::string("auxiliary:")+HUBO_AUX_CMD_CHANNEL+":100:64");
    
    mgr.register_new_chan(std::string("joint_state:")+HUBO_JOINT_SENSOR_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("imu_state:")+HUBO_IMU_SENSOR_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("ft_state:")+HUBO_FT_SENSOR_CHANNEL+":10:4096:");

    return 0;
}
