
#include "../LogRelay.hpp"
#include "../Daemonizer.hpp"
#include <iostream>

int main( int, char* [] )
{
    HuboRT::LogRelay relay;
    HuboRT::Daemonizer rt;

    rt.redirect_signals();

    std::string log_name;
    std::string log_contents;
    while( rt.good() )
    {
        if(relay.receive(log_name, log_contents))
        {
            std::cout << log_name << " | " << log_contents << std::endl;
        }
    }

    return 0;
}
