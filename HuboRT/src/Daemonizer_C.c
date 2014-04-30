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

#include "HuboRT/Daemonizer_C.h"
#include "HuboRT/HuboRtParams.h"
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pwd.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/wait.h>


int hubo_rt_sig_quit = 0;
int hubo_rt_sig_usr1 = 0;
int hubo_rt_sig_usr2 = 0;
int hubo_rt_sig_alarm = 0;
int hubo_rt_sig_child = 0;
int hubo_rt_last_child_pid = 0;
int hubo_rt_last_child_status = 0;

static void hubo_rt_daemon_sig_handler(int signum)
{
    switch(signum)
    {
        case SIGALRM: hubo_rt_sig_alarm++; break;
        case SIGUSR1: hubo_rt_sig_usr1 = 1; break;
        case SIGUSR2: hubo_rt_sig_usr2 = 1; break;
        case SIGCHLD:
            hubo_rt_last_child_pid = wait(&hubo_rt_last_child_status);
            hubo_rt_sig_child++; break;
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
    signal(SIGINT,  hubo_rt_daemon_sig_handler);
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

static int hubo_rt_make_directory_component(const char* subdirectory)
{
    struct stat st = {0};
    if( stat(subdirectory, &st) == -1 )
    {
        if( mkdir(subdirectory, S_IRUSR | S_IXUSR | S_IWUSR |
                                S_IRGRP | S_IXGRP | S_IWGRP |
                                S_IROTH | S_IXOTH | S_IWOTH ) != 0 )
        {
            fprintf( stdout, "Unable to create directory %s, code=%d (%s)\n",
                    subdirectory, errno, strerror(errno) ); fflush(stdout);
            return -1;
        }

        // We do a chmod after creating because POSIX seems to be obstinate
        // about creating a directory with different permissions than its parent.
        if(chmod(subdirectory, S_IRUSR | S_IXUSR | S_IWUSR |
                               S_IRGRP | S_IXGRP | S_IWGRP |
                               S_IROTH | S_IXOTH | S_IWOTH ) != 0)
        {
            fprintf( stdout, "Unable to set universal permissions for directory %s, "
                     "code=%d (%s)\n", subdirectory, errno, strerror(errno)); fflush(stdout);
            return -2;
        }
    }

    return 0;
}

int hubo_rt_safe_make_directory(const char* directory_name)
{
    size_t i;
    for(i=1; i<strlen(directory_name); ++i)
    {
        if(directory_name[i] == '/')
        {
            char subdirectory[MAX_FILENAME_SIZE];
            strncpy(subdirectory, directory_name, i);
            subdirectory[i] = '\0';
            int result = hubo_rt_make_directory_component(subdirectory);
            if(result != 0)
                return result;
        }
    }
    
    hubo_rt_make_directory_component(directory_name);
    
    return 0;
}

static pid_t hubo_rt_daemon_fork()
{
    pid_t child = fork();
    hubo_rt_set_fork_signals();
    if( child < 0 ) // Quit and report error if a child could not be made
    {
        syslog(LOG_ERR, "Unable to fork daemon, code=%d (%s)",
               errno, strerror(errno));
        exit(1);
    }

    if( child > 0 ) // Quit silently if we get a good Process ID for the child
    {
        // (1) Wait for confirmation from the child
        //  -- The child should send SIGUSR1 which tells us to quit silently
        //
        // Or (2) quit forcibly after 2 seconds
        //  -- alarm(2) will trigger a SIGALRM after 2 seconds which we are
        //     handling by quitting with an error
        alarm(2);
        pause();

        exit(1);
    }

    return child;
}

int hubo_rt_daemonize(const char* daemon_name, const char* lock_directory,
                      const char* log_directory)
{
    syslog(LOG_NOTICE, "Starting daemonization for '%s'", daemon_name);

    hubo_rt_sig_quit = 0;
    hubo_rt_sig_usr1 = 0;
    hubo_rt_sig_usr2 = 0;
    hubo_rt_sig_alarm = 0;
    hubo_rt_sig_child = 0;

    if( getppid() == 1 ) return 1;

    int make_dir_error = hubo_rt_safe_make_directory(lock_directory);
    if(make_dir_error != 0)
        return make_dir_error;
    
    char my_log_directory[MAX_FILENAME_SIZE];
    sprintf(my_log_directory, "%s/%s", log_directory, daemon_name);
    make_dir_error = hubo_rt_safe_make_directory(my_log_directory);
    if(make_dir_error != 0)
        return make_dir_error;

    
    char lockfile[MAX_FILENAME_SIZE];
    sprintf(lockfile, "%s/%s", lock_directory, daemon_name);
    int lfp = open(lockfile, O_RDWR|O_CREAT|O_EXCL, S_IRUSR | S_IRGRP | S_IROTH); // lockfile pointer
    if( lfp < 0 )
    {
        syslog( LOG_ERR, "Unable to create lock file '%s', code=%d (%s)\n",
                lockfile, errno, strerror(errno));
        return errno;
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

    hubo_rt_daemon_fork();
    pid_t parent = getppid();
    kill(parent, SIGUSR1); // Kill the useless parent
    
    // Supposedly escaping the terminal might require a double-fork
    hubo_rt_daemon_fork();
    parent = getppid();

    // --- From here on out, we're executing as the final (daemonized) process ---
    
    hubo_rt_redirect_signals();
    
    // Set user permissions for file creation
    umask(0);
    
    pid_t sid = setsid();
    if( sid < 0 )
    {
        syslog( LOG_ERR, "Unable to create new session, code=%d (%s)",
               errno, strerror(errno) );
        return -3;
    }
    
    if( chdir("/") < 0 )
    {
        syslog( LOG_ERR, "Unable to change directory, code=%d (%s)",
                errno, strerror(errno) );
        return -4;
    }
    
    FILE* fp;
    fp = fopen(lockfile, "w");
    fprintf(fp, "%d", sid);
    fclose(fp);
    
    char output_file[MAX_FILENAME_SIZE];
    sprintf(output_file, "%s/output", my_log_directory);
    char error_file[MAX_FILENAME_SIZE];
    sprintf(error_file, "%s/error", my_log_directory);
    if(     !fopen(output_file, "w") ||
            !fopen(error_file, "w") )
    {
        syslog( LOG_ERR, "Unable to create log files, code=%d (%s)",
                errno, strerror(errno));
        return -5;
    }
    
    if(     !freopen(output_file, "w", stdout) ||
            !freopen(error_file, "w", stderr) )
    {
        syslog(LOG_ERR, "Unable to stream output, code=%d (%s)",
               errno, strerror(errno) );
        return -6;
    }
    
    kill(parent, SIGUSR1); // Let the parent process know that we're okay
    
    syslog( LOG_NOTICE, "Finished daemonization for '%s'", daemon_name);

    return 1;
}


int hubo_rt_prioritize(int priority)
{
    if(priority >= 50)
    {
        fprintf(stderr, "You requested an unreasonably high priority of %d\n"
               " -- The maximum you should use is 49\n"
               " -- Most real-time applications should be from 30 - 40\n",
               priority);
        return -1;
    }
    
    if(priority >= 0)
    {
        struct sched_param param;
        param.sched_priority = priority;
        if( sched_setscheduler(0, SCHED_FIFO, &param) == -1 )
        {
            fprintf(stderr, "Failed to set the scheduling for real-time prioritization!\n");
            return -2;
        }
    }
    return 1;
}

int hubo_rt_lock_memory()
{
    return mlockall( MCL_CURRENT | MCL_FUTURE );
}

void hubo_rt_stack_prefault(size_t stack_size)
{
    unsigned char dummy[stack_size];
    memset(dummy, 0, stack_size);
}

void hubo_rt_daemon_close(const char *daemon_name, const char *lock_directory)
{
    char lockfile[MAX_FILENAME_SIZE];
    sprintf(lockfile, "%s/%s", lock_directory, daemon_name);
    remove( lockfile );
    fclose( stdout );
    fclose( stderr );
    syslog( LOG_NOTICE, "Terminated daemon %s gracefully", daemon_name);
}
