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

extern "C" {
#include <stdio.h>
} // extern "C"

#include "HuboCan/VirtualPump.hpp"

namespace HuboCan {

VirtualPump::VirtualPump(double nominal_frequency,
                         double bitrate,
                         size_t channels,
                         size_t nominal_pump_size) :
    CanPump(nominal_frequency, bitrate, channels, nominal_pump_size)
{
    open_channels();
}

bool VirtualPump::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_write_channel, HUBO_VIRTUAL_PUMP_WRITE, NULL);
    if(ACH_OK != result)
    {
        fprintf(stderr, "Error opening vpump write channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
    }

    result = ach_open(&_read_channel, HUBO_VIRTUAL_PUMP_READ, NULL);
    if(ACH_OK != result)
    {
        fprintf(stderr, "Error opening vpump read channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
    }

    _can_initialized = _channels_opened;
    return _channels_opened;
}

bool VirtualPump::_send_frame(const can_frame_t& frame, size_t )
{
    return ach_put(&_write_channel, &frame, sizeof(frame)) == ACH_OK;
}

bool VirtualPump::_wait_on_frame(const timespec_t &relative_timeout)
{
    timespec_t abs_timeout;
    clock_gettime(ACH_DEFAULT_CLOCK, &abs_timeout);
    clock_add(abs_timeout, relative_timeout);

    can_frame_t frame; memset(&frame, 0, sizeof(frame));
    size_t fs = 0;

    ach_status_t result = ACH_OK;
    while( result == ACH_OK )
    {
        result = ach_get(&_read_channel, &frame, sizeof(frame), &fs, &abs_timeout, ACH_O_WAIT);

        if( ACH_TIMEOUT == result )
            return true;

        if(result == ACH_OK)
            _decode_frame(frame, 0);
        // TODO: What should I do about the channel number?
        // Idea: Make a C-Struct for Ach which contains the
        //       frame data in addition to a channel number
    }

    return false;
}

} // namespace HuboCan
