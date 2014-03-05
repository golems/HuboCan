
#include "HuboRT/Manager.hpp"
#include <iostream>

extern "C" {
#include "HuboRT/Daemonizer_C.h"
#include "HuboCan/hubo_info_c.h"
}



int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;
    
//    mgr.register_new_proc("bogus:/home/grey/projects/HuboCan/bin/daemon_test::");
//    mgr.register_new_chan(std::string("meta_channel:")+HUBO_INFO_META_CHANNEL+":10:4096:");
//    mgr.register_new_chan(std::string("info_channel:")+HUBO_INFO_DATA_CHANNEL+":10:4096:");
    mgr.create_all_ach_chans();
    
//    mgr.run_all_processes();
    
//    mgr.launch();
    
    return 0;
}
