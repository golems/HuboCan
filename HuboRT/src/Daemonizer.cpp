
#include "Daemonizer.hpp"
#include "Daemonizer_C.h"
#include <syslog.h>
#include <stdlib.h>

using namespace HuboRT;

Daemonizer::Daemonizer(size_t safe_stack_size)
{
    stack_prefault_size = safe_stack_size;
    lock_directory = "/opt/hubo/rt/lock";
}

Daemonizer::~Daemonizer()
{
    close();
}

bool Daemonizer::daemonize(std::string daemon_name)
{
    _daemon_name = daemon_name;
    int result = hubo_rt_daemonize(daemon_name.c_str(), lock_directory.c_str());
    hubo_rt_stack_prefault(stack_prefault_size);
    return result == 0;
}

bool Daemonizer::prioritize(int priority)
{
    return hubo_rt_prioritize(priority) == 0;
}

bool Daemonizer::good() const
{
    return hubo_rt_sig_quit == 0;
}

bool Daemonizer::usr1()
{
    bool _usr1 = hubo_rt_sig_usr1 != 0;
    hubo_rt_sig_usr2 = 0;
    return _usr1;
}

bool Daemonizer::usr2()
{
    bool _usr2 = hubo_rt_sig_usr2 != 0;
    hubo_rt_sig_usr2 = 0;
    return _usr2;
}

size_t Daemonizer::alarm() const
{
    return hubo_rt_sig_alarm;
}

bool Daemonizer::check(bool condition, std::string message, bool quit_immediately)
{
    if(condition)
    {
        std::string output = "Condition failed in process " + _daemon_name + ": " + message;
        syslog(LOG_ERR, "%s", output.c_str());
        hubo_rt_sig_quit = 1;

        if(quit_immediately)
        {
            close();
            exit(1);
        }
    }

    return condition;
}

bool Daemonizer::close()
{
    return hubo_rt_daemon_close(_daemon_name.c_str()) == 0;
}
