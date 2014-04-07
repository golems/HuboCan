
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
    _can_error = false;
    _first_tick = true;
    _zero_clock(_deadline);
    
    _bitrate = bitrate;
    
    period_threshold = 1E-3;
    
    _channels.resize(channels);
    
}

bool CanPump::_send_frame(const can_frame_t& frame, size_t channel)
{
    std::cout << "WARNING: Attempting to send CAN frames using an instance of an abstract CAN"
              << " Pump!" << std::endl;
    return false;
}

bool CanPump::_wait_on_frame(const timespec_t& relative_timeout)
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
    long nano_dt = clock.tv_nsec + (long)(seconds*1E9);
    clock.tv_sec += (long)(nano_dt/1E9);
    clock.tv_nsec = (long)(nano_dt%((long)1E9));
}

void CanPump::_zero_clock(timespec_t &clock)
{
    clock.tv_sec = 0; clock.tv_nsec = 0;
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
    
    if(_can_error)
        return false;
    
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
    
    timespec_t time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double diff = _clock_diff(_deadline, time);
    
    if(fabs(_timestep-diff) > period_threshold)
    {
        std::cout << "WARNING: CanPump is off schedule by " << _timestep-diff << " seconds" << std::endl;
    }
    
    int expectation = _get_max_frame_expectation();
    if(expectation * can_frame_bit_size > _bitrate * _timestep) // TODO: Use diff here instead of _timestep?
    {
        std::cout << "WARNING: Expected size of CAN frame transfer exceeds the bitrate setting\n"
                  << " -- This may result in frames being dropped!" << std::endl;
    }
    
    double frame_dt = diff/(double)(_get_max_frame_count());
    while(_clock_diff(_deadline, time) > 0)
    {
        _send_next_frames();
        
        if(_can_error)
            return false;
        
        _increment_clock(time, frame_dt);
        _wait_on_next_frames(time);
        
        if(_can_error)
            return false;
    }
    
    for(size_t i=0; i<_channels.size(); ++i)
    {
        _channels[i].reset_counter();
    }
    
    return true;
}

void CanPump::_send_next_frames()
{
    for(size_t i=0; i < _channels.size(); ++i)
    {
        ChannelHandle& handle = _channels[i];
        if(handle.request_frames.size() > 0)
        {
            _send_frame(handle.request_frames.back(), i);
            handle.request_frames.pop_back();
        }
        else if(handle.command_frames.size() > 0)
        {
            _send_frame(handle.command_frames.back(), i);
            handle.command_frames.pop_back();
        }
    }
}

void CanPump::_wait_on_next_frames(const timespec_t &timeout)
{
    timespec_t current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    timespec_t relative_timeout;
    while(_clock_diff(timeout, current_time) > 0)
    {
        _zero_clock(relative_timeout);
        _increment_clock(relative_timeout, _clock_diff(timeout, current_time));
        bool okay = _wait_on_frame(relative_timeout);
        if(!okay)
        {
            break;
        }
        
        clock_gettime(CLOCK_MONOTONIC, &current_time);
    }
}

void CanPump::_decode_frame(const can_frame_t& frame, size_t channel)
{
    bool decoded = false;
    for(size_t i=0; i<_devices.size(); ++i)
    {
        decoded |= _devices[i]->decode(frame, channel);
    }
    --_channels[channel].reply_expectation;

    if(!decoded)
    {
        std::cout << "Could not decode frame! ID:" << std::hex << frame.can_id << " Data: ";
        for(size_t i=0; i<8; ++i)
        {
            std::cout << std::hex << frame.data[i] << " ";
        }
        std::cout << " DLC:" << std::dec << frame.can_dlc << std::endl;
    }
}

int CanPump::_get_max_frame_count()
{
    int max_frames = 0;
    for(size_t i=0; i < _channels.size(); ++i)
    {
        int frame_count = _channels[i].frame_count();
        if( frame_count > max_frames )
            max_frames = frame_count;
    }
    return max_frames;
}

int CanPump::_get_max_frame_expectation()
{
    int max_expectation = 0;
    for(size_t i=0; i < _channels.size(); ++i)
    {
        int frame_expectation = _channels[i].frame_expectation();
        if( frame_expectation > max_expectation )
            max_expectation = frame_expectation;
    }
    return max_expectation;
}













