#ifndef CANPUMP_HPP
#define CANPUMP_HPP

#include <linux/can.h>
#include <vector>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

namespace HuboCan {

class CanDevice;

typedef struct can_frame can_frame_t;
typedef std::vector<can_frame_t> FrameArray;
typedef std::vector<CanDevice*> CanDevicePtrArray;
typedef struct timespec timespec_t;

class ChannelHandle
{
public:
    int net_lost_replies;
    int reply_expectation;
    FrameArray request_frames;
    FrameArray command_frames;
};

typedef std::vector<ChannelHandle> ChannelArray;

class CanPump
{
public:
    
    CanPump(double nominal_frequency, double bitrate, size_t channels);
    
    void addFrame(const can_frame_t& frame, size_t channel,
                  size_t expected_replies=0);

    virtual bool pump();

    inline void addDevice(CanDevice* new_device)
    {
        _devices.push_back(new_device);
    }
    
    double period_threshold;

protected:
    
    bool _can_initialized;
    
    virtual bool _send_frame();
    virtual bool _wait_on_frames();
    
    virtual void _decode_frame(const can_frame_t& frame, size_t channel);

    ChannelArray _channels;
    
    CanDevicePtrArray _devices;
    
    double _bitrate;

    bool _first_tick;
    double _timestep;
    timespec_t _deadline;
    
    void _increment_clock(timespec_t& clock, double seconds);
    double _clock_diff(const timespec_t& last, const timespec_t&first);
};

} // namespace HuboCan

#endif // CANPUMP_HPP
