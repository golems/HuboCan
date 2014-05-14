
#include "HuboRT/Manager.hpp"
#include <syslog.h>

int main(int, char* [])
{
    HuboRT::Manager mgr;

    mgr.launch();

    return 0;
}
