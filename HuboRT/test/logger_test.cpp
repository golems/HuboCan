
#include "../Daemonizer.hpp"
#include <iostream>

int main( int argc, char* argv[] )
{
    std::string message = "log test message";

    if(argc > 1)
    {
        message = std::string(argv[1]);
    }

    std::cout << "writing to log: " << message << std::endl;

    HuboRT::Daemonizer rt;
    rt.redirect_logs("logger_test");

    std::cout << message << std::endl;

    return 0;
}
