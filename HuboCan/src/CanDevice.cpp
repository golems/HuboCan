
#include "../CanDevice.hpp"

// Not a very exciting file...

HuboCan::CanDevice::CanDevice()
{
    _pump = NULL;
}

void HuboCan::CanDevice::registerPump(CanPump &pump)
{
    pump.add_device(this);
    _pump = &pump;
}

void HuboCan::CanDevice::startup() { }

void HuboCan::CanDevice::update() { }

bool HuboCan::CanDevice::decode(const can_frame_t&, size_t) { return false; }

HuboCan::CanDevice::~CanDevice()
{
    
}
