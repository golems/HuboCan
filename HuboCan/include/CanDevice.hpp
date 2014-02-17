#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include "CanPump.hpp"

namespace HuboCan {

class CanDevice
{
public:

    virtual void registerPump(CanPump& pump);

    virtual void update() = 0;
    virtual void decode(const can_frame_t& frame) = 0;

protected:

    CanPump* _pump;

};

}

#endif // CANDEVICE_HPP
