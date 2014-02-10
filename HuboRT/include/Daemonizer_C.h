#ifndef HUBORT_C_H
#define HUBORT_C_H

#include <stddef.h>

extern int hubo_rt_sig_quit;
extern int hubo_rt_sig_usr1;
extern int hubo_rt_sig_usr2;
extern int hubo_rt_sig_alarm;
extern int hubo_rt_sig_child;

int hubo_rt_daemonize(const char* daemon_name, const char *lock_directory);

void hubo_rt_redirect_signals();

int hubo_rt_prioritize(int priority);

int hubo_rt_daemon_close(const char* daemon_name);

void hubo_rt_stack_prefault(size_t stack_size);

void hubo_rt_daemon_assert(int result, int line);

#endif // HUBORT_C_H
