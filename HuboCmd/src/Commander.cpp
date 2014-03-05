
#include "../Commander.hpp"
#include <stdlib.h>

using namespace HuboCmd;

Commander::Commander(double timeout)
{
    cmd_data = NULL;
    getDescription(timeout);
}

Commander::Commander(const HuboCan::HuboDescription& description)
{
    cmd_data = NULL;
    getDescription(description);
}

void Commander::_initialize()
{
    free(cmd_data);
    if(_desc.getJointCount() > 0)
        cmd_data = hubo_cmd_init_data( _desc.getJointCount() );
    else
        cmd_data = NULL;
}

bool Commander::getDescription(double timeout)
{
    int result = _desc.receiveInfo(timeout);
    _initialize();
    return result;
}

void Commander::getDescription(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _initialize();
}

Commander::~Commander()
{

}
