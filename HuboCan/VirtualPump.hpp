#ifndef VIRTUALPUMP_HPP
#define VIRTUALPUMP_HPP

#include "CanPump.hpp"
#include "AchIncludes.h"

#define HUBO_VIRTUAL_PUMP_WRITE "hubo_vpump_write"
#define HUBO_VIRTUAL_PUMP_READ  "hubo_vpump_read"

namespace HuboCan {

class VirtualPump : public CanPump
{
public:

    VirtualPump(double nominal_frequency,
                double bitrate,
                size_t channels,
                size_t nominal_pump_size=1000);

    bool open_channels();

protected:

    ach_channel_t _write_channel;
    ach_channel_t _read_channel;

    bool _channels_opened;

    bool _send_frame(const can_frame_t &frame, size_t channel);
    bool _wait_on_frame(const timespec_t &relative_timeout);

};


} // namespace HuboCan

#endif // VIRTUALPUMP_HPP
