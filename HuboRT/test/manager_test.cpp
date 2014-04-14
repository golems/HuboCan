
#include "HuboRT/Manager.hpp"
#include <iostream>

extern "C" {
#include "HuboRT/Daemonizer_C.h"
#include "HuboCan/hubo_info_c.h"
}



int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;

    mgr.run();

//    mgr.create_all_ach_chans();
//    mgr.stop_all_processes();
    
//    mgr.register_new_proc("bogus:/home/grey/projects/HuboCan/bin/daemon_test::");
    
//    mgr.run_all_processes();
    
//    mgr.launch();
    
    return 0;
}
