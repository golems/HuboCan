/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

extern "C" {
#include <syslog.h>
#include <stdlib.h>

#include "HuboRT/Daemonizer_C.h"
#include "HuboRT/HuboRtParams.h"
} // extern "C"

#include "HuboRT/Daemonizer.hpp"

namespace HuboRT {

Daemonizer::Daemonizer(size_t safe_stack_size)
{
    _d_status = 0;
    _successful_launch = false;
    stack_prefault_size = safe_stack_size;
    _lock_directory = hubo_rt_default_lock_dir;
    _log_directory = hubo_rt_default_log_dir;
}

Daemonizer::~Daemonizer()
{
    if(_successful_launch)
        close();
}

bool Daemonizer::begin(std::string daemon_name, int priority)
{
    prioritize(priority);
    return daemonize(daemon_name);
}

bool Daemonizer::daemonize(const std::string& daemon_name)
{
    _daemon_name = daemon_name;
    _d_status = hubo_rt_daemonize(daemon_name.c_str(), _lock_directory.c_str(),
                                   _log_directory.c_str());
    hubo_rt_stack_prefault(stack_prefault_size);
    if(_d_status == 1)
    {
        _successful_launch = true;
    }
    else
    {
        switch(_d_status)
        {
            case -1: std::cout << "Requested priority is too high"; break;
            case -2: std::cout << "Could not set scheduling for real-time prioritization"; break;
            case -3: std::cout << "Could not set user to root"; break;
            case -4: std::cout << "Could not find root account??"; break;
            case -5: std::cout << "Could not create new session"; break;
            case -6: std::cout << "Could not change current directory"; break;
            case -7: std::cout << "Could not open lockfile -- try running with sudo"; break;
            case -8: std::cout << "Could not create log files"; break;
            case -9: std::cout << "Could not stream output"; break;
            case 17: std::cout << "Lockfile already exists!"; break;
        }

        std::cout << " (" << _d_status << ")" << "\n -- Check syslog for details" << std::endl;
    }

    return _d_status == 1;
}

bool Daemonizer::redirect_logs(const std::string& daemon_name)
{
    return (hubo_rt_redirect_logs(daemon_name.c_str(), _log_directory.c_str())==0);
}

bool Daemonizer::prioritize(int priority)
{
    hubo_rt_stack_prefault(stack_prefault_size);
    hubo_rt_lock_memory();
    return hubo_rt_prioritize(priority) == 1;
}

int Daemonizer::daemonization_status() const { return _d_status; }

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

size_t Daemonizer::child_processes_exited() const
{
    return hubo_rt_sig_child;
}

bool Daemonizer::check(bool condition, std::string message, bool quit_immediately)
{
    if(!condition)
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

void Daemonizer::close()
{
    hubo_rt_daemon_close(_daemon_name.c_str(), _lock_directory.c_str());
}

void Daemonizer::redirect_signals()
{
    hubo_rt_redirect_signals();
}

} // namespace HuboRT
