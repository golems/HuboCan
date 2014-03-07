#ifndef AGGREGATOR_HPP
#define AGGREGATOR_HPP

#define HUBO_AGG_CHANNEL "hubo_agg"

#include <vector>
#include "HuboCan/HuboDescription.hpp"

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "HuboCmd/hubo_cmd_c.h"
}

namespace HuboCan {

typedef std::vector<uint16_t> PidArray;
typedef std::vector<hubo_joint_cmd_t> JointCmdArray;

class Aggregator
{
public:

    Aggregator(const HuboCan::HuboDescription& description);
    ~Aggregator();

    void load_description(const HuboCan::HuboDescription& desc);

    bool open_channels();

    bool run();
    void stop();

    JointCmdArray& getCommands();

protected:

    bool _channels_opened;

    void _initialize();
    void _create_memory();

    void _aggregator_loop();

    bool _is_launched;

    PidArray _pids;

    hubo_cmd_data* _input_data;
    hubo_cmd_data* _output_data;

    hubo_cmd_data* _final_data;
    JointCmdArray _aggregate;

    HuboCan::HuboDescription _desc;

    ach_channel_t _cmd_chan;
    ach_channel_t _agg_chan;

    pid_t _child;

    inline Aggregator(const Aggregator& doNotCopy) { }
    inline Aggregator& operator=(const Aggregator& doNotCopy) { return *this; }

};

} // namespace HuboCan


#endif // AGGREGATOR_HPP
