
#include "../Manager.hpp"
#include <iostream>

#include "HuboCmd/Commander.hpp"
#include "HuboCmd/Aggregator.hpp"

extern "C"{
#include "HuboState/hubo_sensor_c.h"
#include "HuboPath/hubo_path_c.h"
}

// Note: This is really only meant for testing the format

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
    
    mgr.register_new_chan(std::string("instruction:")+HUBO_PATH_INSTRUCTION_CHANNEL
                          +":5:64:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("trajectory:")+HUBO_PATH_INPUT_CHANNEL
                          +":3:65536:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("traj_rx_feedback:")+HUBO_PATH_FEEDBACK_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("player:")+HUBO_PATH_PLAYER_STATE_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");

    return 0;
}
