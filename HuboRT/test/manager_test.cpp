
#include "HuboRT/Manager.hpp"
#include <iostream>

extern "C" {
#include "HuboRT/Daemonizer_C.h"
}

int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;
    
    mgr.register_new_proc("bogus:/home/grey/projects/HuboCan/bin/daemon_test::");
    
    mgr.run_all_processes();
    
//    mgr.launch();
    
    return 0;
}
