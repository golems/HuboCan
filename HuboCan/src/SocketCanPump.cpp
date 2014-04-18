
#include "../SocketCanPump.hpp"
#include <iostream>

#include <stdio.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace HuboCan;

const char* can_device_names[] = { "can0", "can1", "can2", "can3" };
const char* pcan_device_names[] = { "/dev/pcan0", "/dev/pcan1", "/dev/pcan2", "/dev/pcan3" };
const char* virtual_can_names[] = { "vcan0", "vcan1", "vcan2", "vcan3" };

SocketCanPump::SocketCanPump(double nominal_frequency,
                             double bitrate, size_t channels,
                             size_t nominal_pump_size, bool virtual_can) :
    CanPump(nominal_frequency, bitrate, channels, nominal_pump_size)
{
    initialize_devices(virtual_can);
}

SocketCanPump::~SocketCanPump()
{
    size_t channels = channel_count();
    for(size_t i=0; i<channels; ++i)
    {
        if(_is_virtual)
        {
            _deactivate_device(virtual_can_names[i]);
        }
        else
        {
            _deactivate_device(can_device_names[i]);
        }
    }
}

bool SocketCanPump::initialize_devices(bool virtual_can)
{
    _is_virtual = virtual_can;
    _nfds = 0;
    _can_initialized = false;
    
    size_t channels = channel_count();
    // TODO: Decide if the interface should be put up here
    // seems like the best idea
    if(channels > 4)
    {
        std::cout << "Unsupported number of channels for SocketCan (" << channels << ")!\n"
                  << " -- Maximum size is 4" << std::endl;
        return false;
    }
    
    _sockets.resize(channels);
    
    for(size_t i=0; i<channels; ++i)
    {
        if(virtual_can)
        {
            if(!_initialize_device(virtual_can_names[i], i))
                return false;
        }
        else
        {
            // Instruct the pcan device to operate at 1Mb/s
            int result = system( (std::string("echo \"i 0x0014 e\" > ")
                                  +pcan_device_names[i]).c_str());
            if(result != 0)
            {
                perror("setting pcan to 1Mb/s"); fflush(stderr);
            }

            if(!_initialize_device(can_device_names[i], i))
                return false;
        }
    }
    
    for(size_t i=0; i<_sockets.size(); ++i)
    {
        if(_sockets[i] > _nfds)
            _nfds = _sockets[i];
    }
    ++_nfds;
    
    _can_initialized = true;
    return true;
}

bool SocketCanPump::_initialize_device(const char *device_name, size_t index)
{
    int& s = _sockets[index];
    
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        std::cout << "Error while opening a socket for " << device_name
                  << " (" << index << ")!\n"
                  << " -- " << strerror(errno) << " (" << errno << ")" << std::endl;
        return false;
    }
    
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    strcpy(ifr.ifr_name, device_name);
    
    // Get the current flags
    ioctl(s, SIOCGIFFLAGS, &ifr);
    // Make sure the device is up
    ifr.ifr_flags |= IFF_UP;
    // Return the flags with the device being up
    ioctl(s, SIOCSIFFLAGS, &ifr);
    
    // Get the device's interface index
    ioctl(s, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    std::cout << device_name << " was found to have interface index "
              << ifr.ifr_ifindex << std::endl;
    
    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cout << "Error while binding a socket for " << device_name
                  << " (" << index << ")!\n"
                  << " -- " << strerror(errno) << " (" << errno << ")" << std::endl;
//        perror("error while binding a socket");
        return false;
    }
    
    return true;
}

bool SocketCanPump::_deactivate_device(const char *device_name)
{
    // TODO: Decide if device should really be deactivated
    // Would this accomplish anything?
    return false;
}

bool SocketCanPump::_send_frame(const can_frame_t &frame, size_t channel)
{
    ssize_t bytes_written = send(_sockets[channel], &frame, sizeof(frame), MSG_DONTWAIT);
    if(bytes_written != sizeof(frame))
    {
//        if( errno != ENOBUFS && errno != EAGAIN ) // Why not report this?
        {
            perror("send frame over SocketCan");
            std::cout << "Error found on CAN bus " << channel << ", we will quit pumping" << std::endl;
            std::cout << "Check to make sure hardware power is on." << std::endl;
            std::cout << "If the problem persists, you may need to restart the computer :(" << std::endl;
            // TODO: Write a print out that explains appropriate usage
            _can_error = true;
        }
    }
    
    return false;
}

bool SocketCanPump::_wait_on_frame(const timespec_t &relative_timeout)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    
    for(size_t i=0; i<_sockets.size(); ++i)
    {
        FD_SET(_sockets[i], &read_fds);
    }
    
    int result = pselect(_nfds, &read_fds, NULL, NULL, &relative_timeout, NULL);
    if( result < 0 )
    {
        perror("error in pselect attempt");
        return false;
    }
    else if(result == 0)
    {
        return true;
    }
    
    can_frame_t frame;
    for(size_t i=0; i<_sockets.size(); ++i)
    {
        if(FD_ISSET(_sockets[i], &read_fds))
        {
            ssize_t bytes_read = recv(_sockets[i], &frame, sizeof(frame), MSG_DONTWAIT);
            if( bytes_read != sizeof(frame) )
            {
//                if(recv_errno != EAGAIN && recv_errno != EWOULDBLOCK) // Why not report this?
                {
                    perror("recv error in SocketCanPump");
                }
                return false;
            }
            
            _decode_frame(frame, i);
        }
    }
    
    return true;
}
