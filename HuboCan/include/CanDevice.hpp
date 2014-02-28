#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include "CanPump.hpp"

namespace HuboCan {

class CanDevice
{
public:

    inline virtual void registerPump(CanPump& pump)
    {
        pump.addDevice(this);
        _pump = &pump;
    }

    inline virtual void update() { }
    inline virtual void decode(const can_frame_t& frame) { }

protected:

    CanPump* _pump;

};

}

#endif // CANDEVICE_HPP
