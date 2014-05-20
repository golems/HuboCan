
#include "HuboRT/Daemonizer.hpp"
#include "HuboRT/LogRelay.hpp"

#include <iostream>

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

    if(!rt.good())
    {
        std::cout << "RT IS NOT GOOD" << std::endl;
    }

    return 0;
}
