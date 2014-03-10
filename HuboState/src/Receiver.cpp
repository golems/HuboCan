
#include "../Receiver.hpp"
#include "../HuboDataParsers.hpp"

using namespace HuboState;


Receiver::Receiver(double timeout)
{
    _initialize();
    receive_description(timeout);
}

Receiver::Receiver(const HuboCan::HuboDescription &description)
{
    _initialize();
    load_description(description);
}

Receiver::~Receiver()
{

}

bool Receiver::receive_description(double timeout)
{
    int result = _desc.receiveInfo(timeout);
    _create_memory();
    return result;
}

void Receiver::load_description(const HuboCan::HuboDescription &description)
{
    _desc = description;
    _create_memory();
}

void Receiver::_initialize()
{
    _channels_opened = false;
    open_channels();


}

void Receiver::_create_memory()
{

}

bool Receiver::open_channels()
{
    return true;
}
