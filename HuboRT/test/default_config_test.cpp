
#include "../Manager.hpp"
#include <iostream>

#include "HuboCmd/Commander.hpp"

int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.register_new_chan(std::string("meta:")+HUBO_INFO_META_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("info:")+HUBO_INFO_DATA_CHANNEL+":10:4096:");
    mgr.register_new_chan(std::string("command:")+HUBO_CMD_CHANNEL+":10:4096:");

    return 0;
}
