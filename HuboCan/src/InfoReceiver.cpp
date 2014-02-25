
#include "InfoReceiver.hpp"
#include <stdlib.h>

using namespace HuboCan;

InfoReceiver::InfoReceiver(bool receive, double timeout)
{
    _data = NULL;
    if(receive)
        receiveInfo(timeout);
}

InfoReceiver::~InfoReceiver()
{
    free(_data);
}

int InfoReceiver::receiveInfo(double timeout)
{

    return 0;
}

