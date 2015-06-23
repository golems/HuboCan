
#include "../LogRelay.hpp"
#include "../utils.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include "../Manager.hpp"

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
    clock_gettime(ACH_DEFAULT_CLOCK, &t);
    t.tv_sec += timeout;

    size_t fs;
    ach_status_t r = ach_get(&_log_chan, &_buffer, sizeof(_buffer), &fs, &t, ACH_O_WAIT);
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
    }

    return new_message;
}

bool LogRelay::send(double timeout)
{
    if(!_channels_opened)
    {
        std::cout << "ERROR: Attempting to send log data before Ach channels are opened!"
                  << std::endl;
        return false;
    }

    _check_for_new_logs();
    _check_for_log_reset();

//    fd_set set;
//    FD_ZERO(&set);
//    int nfds = 0;
//    for(size_t i=0; i<_handles.size(); ++i)
//    {
//        int fd = _handles[i].fd;
//        FD_SET(fd, &set);
//        if(fd > nfds)
//            nfds = fd;
//    }
//    ++nfds;

//    struct timespec t;
//    memset(&t, 0, sizeof(t));
//    t.tv_sec += timeout;

//    int err = pselect(nfds, &set, NULL, NULL, &t, NULL);
//    if(err==-1)
//    {
//        std::cout << "Error while trying to read logs ("
//                  << strerror(err) << ")" << std::endl;
//        return false;
//    }

    for(size_t i=0; i<_handles.size(); ++i)
    {
//        if(FD_ISSET(_handles[i].fd, &set)!=0)
//        {
            _read_through_fd(_handles[i]);
//        }
    }

    usleep((int)(timeout*1e6));

    return true;
}

void LogRelay::_read_through_fd(FileHandle& fh)
{
    strcpy(_buffer.log_name, fh.filename.c_str());

    ssize_t s = LOG_MESSAGE_CONTENT_SIZE;
    while( s == LOG_MESSAGE_CONTENT_SIZE )
    {
        s = read(fh.fd, _buffer.contents, LOG_MESSAGE_CONTENT_SIZE);
        if( s < 0 && errno != EINTR )
        {
            std::cout << "Error in attempting to read log of '" << fh.filename
                      << "': " << strerror(errno) << std::endl;
            break;
        }
        else if( s == 0 )
            break;

        fh.read_so_far += s;
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

void LogRelay::_check_for_log_reset()
{
//    struct stat stats;
    char stamp_test[LOG_STAMP_SIZE];
    for(size_t i=0; i<_handles.size(); ++i)
    {
        FileHandle& handle = _handles[i];

        // Get the header and check that it's the correct size
        lseek(handle.fd, 0, SEEK_SET);
        ssize_t bytes_read = read(handle.fd, stamp_test, LOG_STAMP_SIZE);
        if( bytes_read != LOG_STAMP_SIZE )
        {
            if(!handle.header_errored)
            {
                std::cerr << "Error checking the stamp of log '" << handle.filename
                          << "'. Bytes read was " << bytes_read << " when it should be"
                          << LOG_STAMP_SIZE;
                if( bytes_read < 0 )
                {
                    std::cerr << ". Error: " << strerror(errno);
                }
                std::cerr << std::endl;
                handle.header_errored = true;
            }
            continue;
        }

        handle.header_errored = false;

        if( strncmp(stamp_test, handle.last_stamp, LOG_STAMP_SIZE) != 0 )
        {
            // If the header has changed, start over from the beginning of the file
            strncpy(handle.last_stamp, stamp_test, LOG_STAMP_SIZE);

            strcpy(_buffer.log_name, handle.filename.c_str());
            strcpy(_buffer.contents, "\n .::. ===== LOG RESTARTED ===== .::. \n");
            _buffer.content_size = strlen(_buffer.contents);
            ach_put(&_log_chan, &_buffer, sizeof(_buffer));

            lseek(handle.fd, 0, SEEK_SET);
            handle.read_so_far = 0;
        }
        else
        {
            // If the header is unchanged, continue reading where we left off
            lseek(handle.fd, handle.read_so_far, SEEK_SET);
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
            FileHandle fh;
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

size_t LogRelay::fd_count() const
{
    return _handles.size();
}
