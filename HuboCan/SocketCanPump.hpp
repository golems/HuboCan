#ifndef SOCKETCANPUMP_HPP
#define SOCKETCANPUMP_HPP

#include "CanPump.hpp"

namespace HuboCan {

class SocketCanPump : public CanPump
{
public:
    SocketCanPump(double nominal_frequency, double bitrate, size_t channels,
                  size_t nominal_pump_size=1000, bool virtual_can=false);
    
    ~SocketCanPump();
    
    bool initialize_devices(bool virtual_can=false);
    
protected:
    
    bool _is_virtual;
    
    bool _initialize_device(const char* device_name, size_t index);
    bool _deactivate_device(const char* device_name);
    
    bool _send_frame(const can_frame_t &frame, size_t channel);
    bool _wait_on_frame(const timespec_t &relative_timeout);
    
    std::vector<int> _sockets;
    
    int _nfds;
};

} // namespace HuboCan

#endif // SOCKETCANPUMP_HPP
