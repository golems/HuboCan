#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include "CanPump.hpp"

namespace HuboCan {

class CanDevice
{
public:
    
    inline CanDevice()
    {
        _pump = NULL;
    }

    inline virtual void registerPump(CanPump& pump)
    {
        pump.addDevice(this);
        _pump = &pump;
    }

    inline virtual void update() { }
    inline virtual bool decode(const can_frame_t& frame, size_t channel) { return false; }

protected:

    CanPump* _pump;

};

}

#endif // CANDEVICE_HPP
