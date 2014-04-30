
#include "HuboRT/Manager.hpp"
#include <iostream>

extern "C" {
#include "HuboRT/Daemonizer_C.h"
#include "HuboCan/hubo_info_c.h"
}



int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;
    HuboRT::Daemonizer rt;
    rt.redirect_signals();

    std::string command = "";
    for(int i=1; i<argc; ++i)
    {
        command = argv[i];
    }

    if(command == "create")
    {
        mgr.create_all_ach_chans();
    }
    else if(command == "kill")
    {
        mgr.stop_all_processes();
    }
    else
    {
        mgr.run();
    }

//    mgr.create_all_ach_chans();
//    mgr.stop_all_processes();
    
//    mgr.register_new_proc("bogus:/home/grey/projects/HuboCan/bin/daemon_test::");
    
//    mgr.run_all_processes();
    
//    mgr.launch();
    
    return 0;
}
