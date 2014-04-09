#ifndef HUBOINITIALIZER_HPP
#define HUBOINITIALIZER_HPP

extern "C" {
#include "hubo_aux_cmd_c.h"
}

#include "HuboCan/HuboDescription.hpp"

namespace HuboCmd {

class Initializer
{
public:

    Initializer(double timeout_sec=1);
    Initializer(const HuboCan::HuboDescription& description);

    bool receive_description(double timeout_sec=1);
    void load_description(const HuboCan::HuboDescription& description);

    bool open_channels();

    void home_joint(size_t joint);
    void home_all_joints();

    bool ready();

protected:

    bool _description_loaded;
    HuboCan::HuboDescription _desc;

    bool _channels_opened;
    ach_channel_t _aux_cmd_chan;

    hubo_aux_cmd_t _cmd;
    bool _send_command();
    void _clear_command();

    void _set_jmc_info(size_t joint);
    size_t _jmc(size_t joint);
    size_t _hw_index(size_t joint);
};

} // namespace HuboCmd

#endif // HUBOINITIALIZER_HPP
