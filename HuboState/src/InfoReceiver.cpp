
#include "InfoReceiver.hpp"
#include <stdlib.h>

using namespace HuboState;

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
    free(_data);
    _data = hubo_info_receive_data(timeout);

    if(_data == NULL)
        return -1;

    size_t joint_count = hubo_info_get_joint_count(_data);

    for(size_t i=0; i < joint_count; ++i)
    {
        _array.push_back(*hubo_info_get_joint_info(_data, i));
    }

    free(_data);
    _data = NULL;

    return 0;
}

