
#include "../LogRelay.hpp"
#include "../Daemonizer.hpp"
#include <iostream>

int main( int, char* [] )
{
    HuboRT::LogRelay relay;
    HuboRT::Daemonizer rt;

    rt.redirect_signals();

    size_t cycle = 0;
    while(relay.send() && rt.good())
    {
        std::cout << "Cycle " << cycle++ << " | fd count: " << relay.fd_count() << std::endl;
    }


    return 0;
}
