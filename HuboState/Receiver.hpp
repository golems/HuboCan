#ifndef RECEIVER_HPP
#define RECEIVER_HPP

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "hubo_sensor_c.h"
#include "HuboCmd/hubo_cmd_c.h"
}
#include "HuboCan/HuboDescription.hpp"

namespace HuboState {

class Receiver
{
public:
    Receiver(double timeout=1);
    Receiver(const HuboCan::HuboDescription& description);
    ~Receiver();

    bool receive_description(double timeout=2);
    void load_description(const HuboCan::HuboDescription& description);

    virtual bool open_channels();


protected:

    bool _channels_opened;

    virtual void _initialize();
    virtual void _create_memory();

    HuboCan::HuboDescription _desc;

    inline Receiver(const Receiver& doNotCopy) { }
    inline Receiver& operator=(const Receiver& doNotCopy) { return *this; }

};

} // namespace HuboState

#endif // RECEIVER_HPP
