
#include "HuboRT/Daemonizer.hpp"

int main(int argc, char* argv[])
{
    HuboRT::Daemonizer rt;
    
    rt.daemonize("bogus_daemon");

    return 0;
}
