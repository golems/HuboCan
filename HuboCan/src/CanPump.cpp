
#include "../CanPump.hpp"
#include "../CanDevice.hpp"
#include <iostream>
#include <errno.h>
#include <string.h>
#include <math.h>

using namespace HuboCan;

CanPump::CanPump(double nominal_frequency, double bitrate, size_t channels)
{
    _timestep = 1.0/nominal_frequency;
    _can_initialized = false;
    _deadline.tv_nsec = 0;
    _deadline.tv_sec  = 0;
    _first_tick = true;
    
    _bitrate = bitrate;
    
    period_threshold = 1E-3;
    
    _channels.resize(channels);
    
}

bool CanPump::_send_frame()
{
    std::cout << "WARNING: Attempting to send CAN frames using an instance of an abstract CAN"
              << " Pump!" << std::endl;
    return false;
}

bool CanPump::_wait_on_frames()
{
    std::cout << "WARNING: Attempting to read CAN frames using an instance of an abstract CAN"
              << " Pump!" << std::endl;
    return false;
}

void CanPump::addFrame(const can_frame_t &frame, size_t channel, size_t expected_replies)
{
    if(channel >= _channels.size())
    {
        std::cout << "ERROR: Attempting to add a frame to the queue for channel #"
                  << channel << ", but the max channel is " << _channels.size() << std::endl;
        return;
    }
    
    ChannelHandle& handle = _channels[channel];
    
    if(expected_replies==0)
    {
        handle.command_frames.push_back(frame);
    }
    else
    {
        handle.reply_expectation += expected_replies;
        handle.request_frames.push_back(frame);
    }
}

void CanPump::_increment_clock(timespec_t& clock, double seconds)
{
    int nano_dt = clock.tv_nsec + (int)(seconds*1E9);
    clock.tv_sec += (int)(nano_dt/1E9);
    clock.tv_nsec = (int)(nano_dt%((int)1E9));
}

double CanPump::_clock_diff(const timespec_t& last, const timespec_t& first)
{
    double last_sec  = (double)(last.tv_sec)+(double)(last.tv_nsec)/1E9;
    double first_sec = (double)(first.tv_sec)+(double)(first.tv_nsec)/1E9;
    return last_sec - first_sec;
}

bool CanPump::pump()
{
    if(!_can_initialized)
    {
        std::cout << "WARNING: Attempting to pump the CanPump before the CAN"
                  << "communication has been initialized!" << std::endl;
        return false;
    }
    
    if(_first_tick)
    {
        if(clock_gettime(CLOCK_MONOTONIC, &_deadline) != 0)
        {
            std::cout << "ERROR: clock_gettime triggered error '"
                      << strerror(errno) << "' (" << errno << ")" << std::endl;
            return false;
        }
        _first_tick = false;
    }
    
    _increment_clock(_deadline, _timestep);
    
    for(size_t i=0; i<_devices.size(); ++i)
    {
        _devices[i]->update();
    }
    
//    int frame_count = _reply_expectation + _request_frames.size() + _command_frames.size();
    timespec_t time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double diff = _clock_diff(_deadline, time);
    
    if(fabs(_timestep-diff) > period_threshold)
    {
        std::cout << "WARNING: CanPump is off schedule by " << _timestep-diff << " seconds" << std::endl;
    }
    
    
    
    return true;
}

void CanPump::_decode_frame(const can_frame_t& frame, size_t channel)
{
    for(size_t i=0; i<_devices.size(); ++i)
    {
        _devices[i]->decode(frame, channel);
    }
    --_channels[channel].reply_expectation;
}

















