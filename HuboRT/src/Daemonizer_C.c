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

#include "Daemonizer_C.h"
#include <stdio.h>
#include <syslog.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pwd.h>
#include <signal.h>


int hubo_rt_sig_quit = 0;
int hubo_rt_sig_usr1 = 0;
int hubo_rt_sig_usr2 = 0;
int hubo_rt_sig_alarm = 0;
int hubo_rt_sig_child = 0;

static void hubo_rt_daemon_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGALRM: hubo_rt_sig_alarm++; break;
        case SIGCHLD: hubo_rt_sig_child++; break;
        case SIGUSR1: hubo_rt_sig_usr1 = 1; break;
        case SIGUSR2: hubo_rt_sig_usr2 = 1; break;
        case SIGINT:
        case SIGTERM:
            hubo_rt_sig_quit = 1; break;
    }
}

void hubo_rt_redirect_signals()
{
    signal(SIGALRM, hubo_rt_daemon_sig_handler);
    signal(SIGCHLD, hubo_rt_daemon_sig_handler);
    signal(SIGUSR1, hubo_rt_daemon_sig_handler);
    signal(SIGUSR2, hubo_rt_daemon_sig_handler);
    signal(SIGINT, hubo_rt_daemon_sig_handler);
    signal(SIGTERM, hubo_rt_daemon_sig_handler);
}

static void hubo_rt_forking_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGUSR1: exit(0); break;
        case SIGALRM: exit(1); break;
        case SIGCHLD: exit(1); break;
    }
}

static void hubo_rt_set_fork_signals()
{
    signal(SIGUSR1, hubo_rt_forking_sig_handler);
    signal(SIGALRM, hubo_rt_forking_sig_handler);
    signal(SIGCHLD, hubo_rt_forking_sig_handler);
}

static pid_t hubo_rt_daemon_fork()
{
    pid_t child;
    if( child < 0 ) // Quit and report error if a child could not be made
    {
        syslog(LOG_ERR, "Unable to fork daemon, code=%d (%s)",
               errno, strerror(errno));
        exit(1);
    }

    if( child>0 ) // Quit silently if we get a good Process ID for the child
    {
        // Wait for confirmation from the child
        //  -- The child should send SIGUSR1 which tells us to quit silently
        // Or quit forcibly after 2 seconds
        //  -- alarm(2) will trigger a SIGALRM after 2 seconds which we are
        //     handling by quitting with an error
        alarm(2);
        pause();

        exit(1);
    }

    return child;
}

int hubo_rt_daemonize(const char *daemon_name, const char *lock_directory)
{
    syslog(LOG_NOTICE, "Starting daemonization for %s", daemon_name);

    hubo_rt_sig_quit = 0;
    hubo_rt_sig_usr1 = 0;
    hubo_rt_sig_usr2 = 0;

    if( getppid() == 1 ) return 1;

    struct stat st = {0};
    if( stat(lock_directory, &st) == -1 )
        mkdir(lock_directory, 0700);

    char lockfile[512];
    sprintf(lockfile, "%s/%s", lock_directory, daemon_name);
    int lfp = open(lockfile, O_RDWR|O_CREAT|O_EXCL, 0640); // lockfile pointer
    if( lfp < 0 )
    {
        syslog( LOG_ERR, "Unable to create lock file %s, code=%d (%s)"
                " -- Check if daemon already exists!",
                lockfile, errno, strerror(errno));
        return -1;
    }

    // Drop the user if one exists
    if( getuid()==0 || geteuid()==0 )
    {
        struct passwd *pw = getpwnam("root");
        if( pw )
        {
            syslog( LOG_NOTICE, "Setting user to root");
            setuid( pw->pw_uid );
        }
    }

    hubo_rt_set_fork_signals();

    pid_t pid, child, sid, parent;

    child = hubo_rt_daemon_fork();






    return 0;
}


int hubo_rt_prioritize(int priority)
{

    return 0;
}
