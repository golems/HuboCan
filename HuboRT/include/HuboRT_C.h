#ifndef HUBORT_C_H
#define HUBORT_C_H

int hubo_rt_daemonize(const char* daemon_name);
int hubo_rt_prioritize(int priority);

int hubo_rt_daemon_close(const char* daemon_name);

void hubo_rt_stack_prefault(size_t stack_size);

#endif // HUBORT_C_H
