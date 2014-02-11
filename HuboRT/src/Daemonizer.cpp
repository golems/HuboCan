/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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

#include "Daemonizer.hpp"
#include "Daemonizer_C.h"
#include <syslog.h>
#include <stdlib.h>

using namespace HuboRT;

Daemonizer::Daemonizer(size_t safe_stack_size)
{
    stack_prefault_size = safe_stack_size;
    _lock_directory = "/opt/hubo/rt/lock";
    _log_directory = "/opt/hubo/rt/log";
}

Daemonizer::~Daemonizer()
{
    close();
}

bool Daemonizer::begin(std::string daemon_name, int priority)
{
    return daemonize(daemon_name) && prioritize(priority);
}

bool Daemonizer::daemonize(std::string daemon_name)
{
    _daemon_name = daemon_name;
    int result = hubo_rt_daemonize(daemon_name.c_str(), _lock_directory.c_str(),
                                   _log_directory.c_str());
    hubo_rt_stack_prefault(stack_prefault_size);
    return result == 0;
}

bool Daemonizer::prioritize(int priority)
{
    hubo_rt_stack_prefault(stack_prefault_size);
    hubo_rt_lock_memory();
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

void Daemonizer::close()
{
    hubo_rt_daemon_close(_daemon_name.c_str(), _lock_directory.c_str());
}
