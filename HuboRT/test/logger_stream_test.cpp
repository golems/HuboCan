
#include "../Daemonizer.hpp"
#include <iostream>
#include <unistd.h>

int main( int argc, char* argv[] )
{
    std::string log_name = "logger_stream_test";
    if(argc > 1)
    {
        log_name = std::string(argv[1]);
    }

    HuboRT::Daemonizer rt;

    rt.redirect_logs(log_name);

    size_t message_num = 0;
    while(true)
    {
        std::cout << "Logging message #" << message_num++ << std::endl;

        sleep(1);
    }


    return 0;
}
