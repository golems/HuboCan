
#include "../LogRelay.hpp"
#include "../utils.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>

using namespace HuboRT;

LogRelay::LogRelay()
{
    _initialize();
}

LogRelay::_initialize()
{
    _channels_opened = false;
    _log_directory = hubo_rt_default_log_dir;
    open_channels();
}

bool LogRelay::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;
    ach_status_t result = ach_open(&_log_chan, HUBO_RT_LOG_RELAY_CHAN, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening log relay channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }

    return _channels_opened;
}

bool LogRelay::send(int timeout)
{
    _check_for_new_logs();

    fd_set set;
    FD_ZERO(&set);
    int nfds = 0;
    for(size_t i=0; i<_fds.size(); ++i)
    {
        int fd = _fds[i];
        FD_SET(fd, &set);
        if(fd > nfds)
            nfds = fd;
    }
    ++nfds;

    struct timespec t;
    memset(&t, 0, sizeof(t));
    t.tv_sec += timeout;

    int err = pselect(nfds, &set, NULL, NULL, &t, NULL);
    if(err!=0)
    {
        std::cout << "Error while trying to read logs ("
                  << strerror(err) << ")" << std::endl;
        return false;
    }

    for(size_t i=0; i<_fds.size(); ++i)
    {
        if(FD_ISSET(_fds[i], &set)!=0)
        {

        }
    }

    return true;
}

void LogRelay::_check_for_new_logs()
{
    StringArray root_logs = grab_files_in_dir(_log_directory);
    for(size_t i=0; i<root_logs.size(); ++i)
    {
        _add_file_descriptor(_log_directory+"/"+root_logs[i]);
    }

    StringArray log_dirs = grab_dirs_in_dir(_log_directory);

    StringArray logs;
    for(size_t i=0; i<log_dirs.size(); ++i)
    {
        logs = grab_files_in_dir(_log_directory+"/"+log_dirs[i]);
        for(size_t j=0; j<logs.size(); ++j)
        {
            _add_file_descriptor(_log_directory+"/"+log_dirs[i]+"/"+logs[j]);
        }
    }
}

void LogRelay::_add_file_descriptor(const std::string &log_name)
{
    std::map<std::string,size_t>::iterator check = _fd_tracker.find(log_name);
    if(check == _fd_tracker.end())
    {
        int new_fd = open(log_name.c_str(), O_RDONLY);
        if(new_fd >= 0)
        {
            _fds.push_back(new_fd);
            _fd_tracker[log_name] = _fds.size()-1;
        }
        else
        {
            std::cout << "Error while trying to open log file '" << log_name
                      << "' (" << strerror(errno) << ") " << errno << std::endl;
        }
    }
}
