#ifndef HUBOAUXRECEIVER_HPP
#define HUBOAUXRECEIVER_HPP

extern "C" {
#include "hubo_aux_cmd_c.h"
}

#include "HuboCan/HuboDescription.hpp"

namespace HuboCmd {

/// AuxReceiver
class AuxReceiver
{
public:

    AuxReceiver(HuboCan::HuboDescription* description);

    bool open_channels();

    bool update();

    bool ready();

protected:

    void _organize_sensors();

    void _register_command();

    void _register_with_all_jmcs();

    void _register_with_all_sensors();

    void _register_with_all_imus();

    void _register_with_all_fts();

    std::vector<HuboCan::HuboImu*> _imus;
    std::vector<HuboCan::HuboFt*> _fts;

    bool _channels_opened;
    ach_channel_t _aux_cmd_chan;

    hubo_aux_cmd_t _cmd;

    HuboCan::HuboDescription* _desc;

};

} // namespace HuboCmd

#endif // HUBOAUXRECEIVER_HPP
