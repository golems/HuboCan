
#include "HuboRT/Manager.hpp"
#include "HuboCmd/Commander.hpp"
#include "HuboCmd/Aggregator.hpp"
#include "HuboState/State.hpp"
#include "HuboPath/hubo_path.hpp"
#include "HuboRT/LogRelay.hpp"

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

    mgr.register_new_chan(std::string("instruction:")+HUBO_PATH_INSTRUCTION_CHANNEL
                          +":5:64:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("trajectory:")+HUBO_PATH_INPUT_CHANNEL
                          +":3:65536:"+ACHD_PUSH_STRING+":");
    mgr.register_new_chan(std::string("traj_rx_feedback:")+HUBO_PATH_FEEDBACK_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");
    mgr.register_new_chan(std::string("player:")+HUBO_PATH_PLAYER_STATE_CHANNEL
                          +":5:64:"+ACHD_PULL_STRING+":");

    mgr.register_new_chan(std::string("log:")+HUBO_RT_LOG_RELAY_CHAN
                          +":10:4608:"+ACHD_PULL_STRING+":");


    mgr.register_new_proc(std::string("player")
                          + ":/usr/bin/huboplayer::");
    mgr.register_new_proc(std::string("log_publisher")
                          + ":/usr/bin/hubo_log_publisher::");


    std::string robot_type = "Hubo2Plus";
    for(int i=1; i<argc; ++i)
    {
        robot_type = argv[i];
    }

    std::string args;
    if(robot_type == "virtual")
        args = "virtual";

    mgr.register_new_proc(std::string("socketcan_interface")
                          +":/usr/bin/hubo_socketcan_interface:robot "+"DrcHubo "+args+":");
    mgr.save_current_config("DrcHubo");

    mgr.register_new_proc(std::string("socketcan_interface")
                          +":/usr/bin/hubo_socketcan_interface:robot "+"Hubo2Plus "+args+":");
    mgr.save_current_config("Hubo2Plus");

    if(robot_type != "virtual")
        mgr.load_config(robot_type);

    return 0;
}