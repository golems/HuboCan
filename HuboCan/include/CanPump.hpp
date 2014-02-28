#ifndef CANPUMP_HPP
#define CANPUMP_HPP

#include <linux/can.h>
#include <vector>
#include <stddef.h>
#include <stdint.h>

namespace HuboCan {

class CanDevice;

typedef struct can_frame can_frame_t;
typedef std::vector<CanDevice*> CanDevicePtrArray;


class CanPump
{
public:

    void addFrame(const can_frame_t& frame, size_t expected_replies=0);

    void pump();

    inline void addDevice(CanDevice* new_device)
    {
        _devices.push_back(new_device);
    }

protected:

    CanDevicePtrArray _devices;

};

} // namespace HuboCan

#endif // CANPUMP_HPP
