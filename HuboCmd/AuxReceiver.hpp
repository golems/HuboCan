#ifndef HUBOAUXRECEIVER_HPP
#define HUBOAUXRECEIVER_HPP

extern "C" {
#include "hubo_aux_cmd_c.h"
}

#include "HuboCan/HuboDescription.hpp"

namespace HuboCmd {

class AuxReceiver
{
public:

    AuxReceiver(HuboCan::HuboDescription* description);

    bool open_channels();

    bool update();

    bool ready();

protected:

    void _register_command();
    void _register_with_all_jmcs();

    bool _channels_opened;
    ach_channel_t _aux_cmd_chan;

    hubo_aux_cmd_t _cmd;

    HuboCan::HuboDescription* _desc;

};

} // namespace HuboCmd

#endif // HUBOAUXRECEIVER_HPP
