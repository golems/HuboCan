
#include "HuboRT/Manager.hpp"
#include <syslog.h>

int main(int argc, char* argv[])
{
    syslog( LOG_NOTICE, "About to instantiate manager" );
    HuboRT::Manager mgr;

    syslog( LOG_NOTICE, "About to launch manager" );
//    mgr.launch();
    mgr.run();

    return 0;
}
