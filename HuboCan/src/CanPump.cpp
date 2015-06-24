/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <errno.h>
#include <string.h>
#include <math.h>

#include "HuboCan/CanPump.hpp"
#include "HuboCan/CanDevice.hpp"
#include "HuboCan/HuboDescription.hpp"

namespace HuboCan {

CanPump::CanPump(double nominal_frequency, double bitrate, size_t channels, size_t nominal_pump_size)
{
    _timestep = 1.0/nominal_frequency;
    _can_initialized = false;
    _can_error = false;
    _first_tick = true;
    zero_clock(_deadline);
    
    _bitrate = bitrate;
    
    period_threshold = 3E-3;
    
    _channels.resize(channels);
    for(size_t i=0; i<_channels.size(); ++i)
    {
        _channels[i].request_frames.reserve(nominal_pump_size);
        _channels[i].command_frames.reserve(nominal_pump_size);
    }
}

void CanPump::load_description(HuboDescription& desc)
{
    for(size_t i=0; i<desc.jmcs.size(); ++i)
    {
        desc.jmcs[i]->registerPump(*this);
    }

    for(size_t i=0; i<desc.sensors.size(); ++i)
    {
        desc.sensors[i]->registerPump(*this);
    }
}

bool CanPump::_send_frame(const can_frame_t&, size_t)
{
    std::cout << "WARNING: Attempting to send CAN frames using an instance of an abstract CAN"
              << " Pump!" << std::endl;
    return false;
}

bool CanPump::_wait_on_frame(const timespec_t&)
{
    std::cout << "WARNING: Attempting to read CAN frames using an instance of an abstract CAN"
              << " Pump!" << std::endl;
    return false;
}

void CanPump::add_frame(const can_frame_t &frame, size_t channel, size_t expected_replies)
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

void CanPump::increment_clock(timespec_t& clock, double seconds)
{
    long nano_dt = clock.tv_nsec + (long)(seconds*1E9);
    clock.tv_sec += (long)(nano_dt/1E9);
    clock.tv_nsec = (long)(nano_dt%((long)1E9));
}

void CanPump::zero_clock(timespec_t &clock)
{
    clock.tv_sec = 0; clock.tv_nsec = 0;
}

double CanPump::clock_diff(const timespec_t& last, const timespec_t& first)
{
    double last_sec  = (double)(last.tv_sec)+(double)(last.tv_nsec)/1E9;
    double first_sec = (double)(first.tv_sec)+(double)(first.tv_nsec)/1E9;
    return last_sec - first_sec;
}

void CanPump::clock_add(timespec_t &value, const timespec_t &add)
{
    long nano_wait = value.tv_nsec + add.tv_nsec + add.tv_sec*(long)(1e9);
    value.tv_sec += (long)(nano_wait/1e9);
    value.tv_nsec = (long)(nano_wait%((long)1e9));
}

bool CanPump::pump()
{
    if(!_can_initialized)
    {
        std::cout << "WARNING: Attempting to pump the CanPump before the CAN "
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
    
    increment_clock(_deadline, _timestep);
    
    timespec_t time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    double diff = clock_diff(_deadline, time);
    
    if(fabs(_timestep-diff) > period_threshold)
    {
        std::cout << "WARNING: CanPump is off schedule by " << _timestep-diff << " seconds" << std::endl;
    }
    
    int expectation = _get_max_frame_expectation();
    if(expectation * can_frame_bit_size > _bitrate * diff)
    {
        std::cout << "WARNING: Expected size of CAN frame transfer (" << expectation*can_frame_bit_size
                  << ") exceeds the bitrate setting (" << _bitrate * diff << ")\n"
                  << " -- This may result in frames being dropped!" << std::endl;

        if( diff < 0 )
        {
            std::cout << "Skipping this cycle" << std::endl;
            return true;
        }
    }

    for(size_t i=0; i<_devices.size(); ++i)
    {
        _devices[i]->update();
    }
    
    if(_get_max_frame_count() > 0)
    {
        double frame_dt = diff/(double)(_get_max_frame_count());
        while(clock_diff(_deadline, time) > 0)
        {
            _send_next_frames();

            if(_can_error)
                return false;

            increment_clock(time, frame_dt);
            _wait_on_next_frames(time);

            if(_can_error)
                return false;
        }
    }
    else
    {
        _wait_on_next_frames(_deadline);
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
    while(clock_diff(timeout, current_time) > 0)
    {
        zero_clock(relative_timeout);
        increment_clock(relative_timeout, clock_diff(timeout, current_time));
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
        std::cout << "Could not decode frame on Channel " << channel << "! ID:"
                  << /*std::hex <<*/ frame.can_id << " Data: ";
        for(size_t i=0; i<8; ++i)
        {
            std::cout << (int)frame.data[i] << " ";
        }
        std::cout << " DLC:" << /*std::dec <<*/ (int)frame.can_dlc << std::endl;
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

} // namespace HuboCan
