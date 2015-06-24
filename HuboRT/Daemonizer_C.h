/*
 * Copyright (c) 2015-2015, Georgia Tech Research Corporation
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

#ifndef HUBORT_DAEMONIZER_C_H
#define HUBORT_DAEMONIZER_C_H

#include <stddef.h>

extern int hubo_rt_sig_quit;
extern int hubo_rt_sig_usr1;
extern int hubo_rt_sig_usr2;
extern int hubo_rt_sig_alarm;
extern int hubo_rt_sig_child;
extern int hubo_rt_last_child_pid;
extern int hubo_rt_last_child_status;

int hubo_rt_daemonize(const char* daemon_name, const char *lock_directory, const char *log_directory);

int hubo_rt_redirect_logs(const char* daemon_name, const char* log_directory);

void hubo_rt_redirect_signals();

int hubo_rt_prioritize(int priority);

int hubo_rt_lock_memory();

void hubo_rt_stack_prefault(size_t stack_size);

void hubo_rt_remove_lockfile(const char* daemon_name, const char* lock_directory);

void hubo_rt_daemon_close(const char* daemon_name, const char *lock_directory);

int hubo_rt_safe_make_directory(const char* directory_name);

#endif // HUBORT_DAEMONIZER_C_H
