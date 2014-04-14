
#include "HuboRT/Manager.hpp"
#include "HuboCmd/Commander.hpp"
#include "HuboCmd/Aggregator.hpp"
#include "HuboState/State.hpp"


#include <iostream>


int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.register_new_chan(std::string("meta:")+HUBO_INFO_META_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("info:")+HUBO_INFO_DATA_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("command:")+HUBO_CMD_CHANNEL
                          +":10:4096:"+ACHD_INTERNAL_STRING+":");
    mgr.register_new_chan(std::string("aggregate:")+HUBO_AGG_CHANNEL
                          +":20:4096:"+ACHD_INTERNAL_STRING+":");
    mgr.register_new_chan(std::string("auxiliary:")+HUBO_AUX_CMD_CHANNEL
                          +":100:64:"+ACHD_PUSH_STRING+":");

    mgr.register_new_chan(std::string("joint_state:")+HUBO_JOINT_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("imu_state:")+HUBO_IMU_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("ft_state:")+HUBO_FT_SENSOR_CHANNEL
                          +":10:4096:"+ACHD_PULL_STRING+":");

    std::string robot_type = "Hubo2Plus";
    for(int i=1; i<argc; ++i)
    {
        robot_type = argv[i];
    }

    mgr.register_new_proc(std::string("socketcan_interface")
                            +":/usr/bin/hubo_socketcan_interface:robot "+robot_type+":");

    return 0;
}
