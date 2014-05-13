
#include "../LogRelay.hpp"
#include "../Daemonizer.hpp"
#include <iostream>

int main( int, char* [] )
{
    HuboRT::LogRelay relay;
    HuboRT::Daemonizer rt;

    rt.redirect_signals();

    while(relay.send() && rt.good())
    {
        std::cout << "Cycle" << std::endl;
    }


    return 0;
}
