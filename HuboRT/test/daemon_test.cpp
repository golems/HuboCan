

#include "HuboRT/Daemonizer.hpp"

int main(int, char* [])
{
    HuboRT::Daemonizer rt;
    
    rt.daemonize("bogus_daemon");

    return 0;
}
