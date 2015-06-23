#ifndef CANDEVICE_HPP
#define CANDEVICE_HPP

#include "CanPump.hpp"

namespace HuboCan {

class CanDevice
{
public:
    
    virtual ~CanDevice();
    
    CanDevice();

    virtual void registerPump(CanPump& pump);

    virtual void startup(); // TODO: Currently not being used
    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    CanPump* _pump;

};

}

#endif // CANDEVICE_HPP
