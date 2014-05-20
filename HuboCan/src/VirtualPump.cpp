
#include "../VirtualPump.hpp"
#include <stdio.h>

using namespace HuboCan;

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
