#ifndef MAXFILENAMESIZE_H
#define MAXFILENAMESIZE_H

enum {
    MAX_FILENAME_SIZE = 512
};

const char hubo_rt_default_lock_dir[] = "/opt/hubo/rt/lock";
const char hubo_rt_default_log_dir[] = "/opt/hubo/rt/log";

// These pertain to the stamp placed in a log at the moment that stdout and stderr get redirected
#define TIMESTAMP_SIZE 24
#define STAMP_PRECURSOR_SIZE 18
#define LOG_STAMP_SIZE STAMP_PRECURSOR_SIZE + TIMESTAMP_SIZE

#endif // MAXFILENAMESIZE_H
