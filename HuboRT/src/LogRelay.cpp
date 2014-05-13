
#include "../LogRelay.hpp"
#include "../utils.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <unistd.h>

using namespace HuboRT;

LogRelay::LogRelay()
{
    _initialize();
}

void LogRelay::_initialize()
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

bool LogRelay::receive(std::string &log_name, std::string &contents, int timeout)
{
    if(!_channels_opened)
    {
        std::cout << "ERROR: Attempting to receive log data before Ach channels are opened!"
                  << std::endl;
        return false;
    }

    bool new_message = false;

    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    t.tv_sec += timeout;

    size_t fs;
    ach_status_t r = ach_get(&_log_chan, &_buffer, sizeof(_buffer), &fs, &t, 0);
    if( ACH_OK == r || ACH_MISSED_FRAME == r )
    {
        new_message = true;
        log_name = std::string(_buffer.log_name);
        if(_buffer.content_size > LOG_MESSAGE_CONTENT_SIZE)
        {
            std::cout << "LogRelay received invalid message size: "
                      << _buffer.content_size << ". Cannot exceed "
                      << LOG_MESSAGE_CONTENT_SIZE << std::endl;
            return false;
        }

        strncpy(_transition, _buffer.contents, _buffer.content_size);
        _transition[_buffer.content_size] = '\0';
        contents = std::string(_transition);

        std::cout << "size received: " << _buffer.content_size << std::endl;
    }

    return new_message;
}

bool LogRelay::send(int timeout)
{
    if(!_channels_opened)
    {
        std::cout << "ERROR: Attempting to send log data before Ach channels are opened!"
                  << std::endl;
        return false;
    }

    _check_for_new_logs();

    fd_set set;
    FD_ZERO(&set);
    int nfds = 0;
    for(size_t i=0; i<_handles.size(); ++i)
    {
        int fd = _handles[i].fd;
        FD_SET(fd, &set);
        if(fd > nfds)
            nfds = fd;
    }
    ++nfds;

    struct timespec t;
    memset(&t, 0, sizeof(t));
    t.tv_sec += timeout;

    int err = pselect(nfds, &set, NULL, NULL, &t, NULL);
    if(err==-1)
    {
        std::cout << "Error while trying to read logs ("
                  << strerror(err) << ")" << std::endl;
        return false;
    }

    for(size_t i=0; i<_handles.size(); ++i)
    {
        if(FD_ISSET(_handles[i].fd, &set)!=0)
        {
            _read_through_fd(_handles[i]);
        }
    }

    return true;
}

void LogRelay::_read_through_fd(const FileHandler& fh)
{
    strcpy(_buffer.log_name, fh.filename.c_str());

    ssize_t s = LOG_MESSAGE_CONTENT_SIZE;
    while( s == LOG_MESSAGE_CONTENT_SIZE )
    {
        s = read(fh.fd, _buffer.contents, LOG_MESSAGE_CONTENT_SIZE);
        if( s < 0 )
        {
            std::cout << "Error in attempting to read log of '" << fh.filename
                      << "': " << strerror(errno) << std::endl;
        }
        else if( s == 0 )
            break;

        _buffer.content_size = s;
        ach_put(&_log_chan, &_buffer, sizeof(_buffer));
    }
}

void LogRelay::_check_for_new_logs()
{
    StringArray root_logs = grab_files_in_dir(_log_directory);
    for(size_t i=0; i<root_logs.size(); ++i)
    {
        _add_file_descriptor(_log_directory, root_logs[i]);
    }

    StringArray log_dirs = grab_dirs_in_dir(_log_directory);

    StringArray logs;
    for(size_t i=0; i<log_dirs.size(); ++i)
    {
        logs = grab_files_in_dir(_log_directory+"/"+log_dirs[i]);
        for(size_t j=0; j<logs.size(); ++j)
        {
            _add_file_descriptor(_log_directory, log_dirs[i]+"/"+logs[j]);
        }
    }
}

void LogRelay::_add_file_descriptor(const std::string& directory, const std::string &log_name)
{
    std::string path = directory+"/"+log_name;
    std::map<std::string,size_t>::iterator check = _fd_tracker.find(path);
    if(check == _fd_tracker.end())
    {
        int new_fd = open(path.c_str(), O_RDONLY);
        if(new_fd >= 0)
        {
            FileHandler fh;
            fh.fd = new_fd;
            fh.filename = log_name;
            _handles.push_back(fh);
            _fd_tracker[path] = _handles.size()-1;
        }
        else
        {
            std::cout << "Error while trying to open log file '" << path
                      << "' (" << strerror(errno) << ") " << errno << std::endl;
        }
    }
}
