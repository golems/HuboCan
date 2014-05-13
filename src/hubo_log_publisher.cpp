
#include "HuboRT/Daemonizer.hpp"
#include "HuboRT/LogRelay.hpp"

int main(int, char* [])
{
    HuboRT::Daemonizer rt;
    if(!rt.begin("log_publisher", 30))
    {
        return 1;
    }

    HuboRT::LogRelay relay;

    while(relay.send() && rt.good())
    {

    }

    return 0;
}
