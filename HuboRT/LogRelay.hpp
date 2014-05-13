#ifndef LOGRELAY_HPP
#define LOGRELAY_HPP

#include "HuboCan/AchIncludes.h"

#include <string>
#include <vector>
#include <map>

#define HUBO_RT_LOG_RELAY_CHAN "log_relay"

#include "HuboRtParams.h"

#define LOG_MESSAGE_CONTENT_SIZE 4096

typedef struct hubo_rt_log_message {

    char log_name[MAX_FILENAME_SIZE];
    char contents[LOG_MESSAGE_CONTENT_SIZE];
    uint32_t content_size;

}__attribute__((packed)) hubo_rt_log_message_t;

namespace HuboRT{

class FileHandle
{
public:
    int fd;
    std::string filename;
    long int read_so_far;
};

class LogRelay
{
public:

    LogRelay();

    bool open_channels();

    bool send(int timeout = 2);

    bool receive(std::string& log_name, std::string& contents, int timeout=2);

protected:

    void _add_file_descriptor(const std::string& directory, const std::string& log_name);
    void _read_through_fd(FileHandle &fh);

    void _initialize();

    void _check_for_new_logs();
    void _check_for_truncation();

    std::string _log_directory;

    std::vector<FileHandle> _handles;
    std::map<std::string,size_t> _fd_tracker;


    hubo_rt_log_message_t _buffer;
    char _transition[LOG_MESSAGE_CONTENT_SIZE+1];
    ach_channel_t _log_chan;
    bool _channels_opened;


};

} // namespace HuboRT

#endif // LOGRELAY_HPP
