
#include "Manager.hpp"
#include <iostream>

extern "C" {
#include "Daemonizer_C.h"
}

int main(int argc, char* argv[])
{
    HuboRT::Manager mgr;
    
    mgr.launch();
    
//    hubo_rt_safe_make_directory("/opt/hubo/rt/log");
    
    return 0;
}
