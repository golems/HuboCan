#ifndef CANPUMP_HPP
#define CANPUMP_HPP

#include <linux/can.h>
#include <vector>
#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include <iostream>

namespace HuboCan {

class CanDevice;
class HuboDescription;

typedef struct can_frame can_frame_t;
typedef std::vector<can_frame_t> FrameArray;
typedef std::vector<CanDevice*> CanDevicePtrArray;
typedef struct timespec timespec_t;

const int can_frame_bit_size = 108;

class ChannelHandle
{
public:
    int net_lost_replies;
    int reply_expectation;
    FrameArray request_frames;
    FrameArray command_frames;
    
    inline int frame_count()
    {
        return request_frames.size()
            + command_frames.size();
    }
    
    inline int frame_expectation()
    {
        return reply_expectation + frame_count();
    }
    
    inline void reset_counter()
    {
        net_lost_replies += reply_expectation;
        reply_expectation = 0;
    }
};

typedef std::vector<ChannelHandle> ChannelArray;

class CanPump
{
public:
    
    CanPump(double nominal_frequency, double bitrate, size_t channels,
            size_t nominal_pump_size);

    void load_description(HuboDescription& desc);
    
    void add_frame(const can_frame_t& frame, size_t channel,
                  size_t expected_replies=0);

    bool pump();

    inline void add_device(CanDevice* new_device)
    {
        _devices.push_back(new_device);
    }
    
    double period_threshold;
    
    inline bool error() { return _can_error; }
    inline void report_error() { _can_error = true; }
    
    int channel_count() { return _channels.size(); }
    
    inline const timespec_t& last_deadline() { return _deadline; }
    
    inline double last_deadline_value()
    {
        return (double)(_deadline.tv_sec)+(double)(_deadline.tv_nsec)/1E9;
    }

    inline const ChannelHandle& channel(size_t num)
    {
        if(num < _channels.size())
            return _channels[num];
        else
        {
            std::cout << "Trying to retrieve out-of-bounds channel handle (" << num << "), "
                         << "max is " << _channels.size()-1 << std::endl;
            return _channels[0];
        }
    }

    // TODO: Make these utility functions instead of CanPump functions
    static void increment_clock(timespec_t& clock, double seconds);
    static void zero_clock(timespec_t& clock);
    static double clock_diff(const timespec_t& last, const timespec_t&first);
    static void clock_add(timespec_t& value, const timespec_t& add);

protected:
    
    bool _can_initialized;
    bool _can_error;
    
    void _send_next_frames();
    void _wait_on_next_frames(const timespec_t& timeout);
    
    virtual bool _send_frame(const can_frame_t& frame, size_t channel);
    virtual bool _wait_on_frame(const timespec_t& relative_timeout);
    
    void _decode_frame(const can_frame_t& frame, size_t channel);

    ChannelArray _channels;
    
    CanDevicePtrArray _devices;
    
    double _bitrate;

    bool _first_tick;
    double _timestep;
    timespec_t _deadline;
    
    int _get_max_frame_count();
    int _get_max_frame_expectation();
};

} // namespace HuboCan

#endif // CANPUMP_HPP
