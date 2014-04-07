#ifndef AGGREGATOR_HPP
#define AGGREGATOR_HPP

#define HUBO_AGG_CHANNEL "hubo_agg"

#include <vector>
#include <map>
#include "HuboCan/HuboDescription.hpp"
#include "HuboRT/Daemonizer.hpp"

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "HuboCmd/hubo_cmd_c.h"
}

namespace HuboCmd {

typedef std::vector<pid_t> PidArray;
typedef std::map<pid_t,bool> PidBoolMap;
typedef std::vector<hubo_joint_cmd_t> JointCmdArray;

class Aggregator
{
public:

    Aggregator(const HuboCan::HuboDescription& description);
    ~Aggregator();

    void load_description(const HuboCan::HuboDescription& desc);

    bool open_channels();
    void close_channels();

    bool run();

    const JointCmdArray& update();
    
    inline const JointCmdArray& last_commands()
    {
        return _aggregated_cmds;
    }

protected:

    bool _memory_set;
    bool _channels_opened;
    bool _is_launched;

    void _initialize();
    void _create_memory();

    void _init_aggregator();
    void _aggregator_loop();
    void _quit_aggregator();

    void _check_hubocan_state();
    void _collate_input();
    bool _resolve_ownership(size_t joint_index);
    void _accept_command(size_t joint_index);
    void _send_output();

    PidArray _pids;
    PidBoolMap _reception_check;

    hubo_joint_cmd_t _container;

    hubo_cmd_data* _input_data;
    hubo_cmd_data* _output_data;

    hubo_cmd_data* _final_data;
    JointCmdArray _aggregated_cmds;
    void _copy_final_data_to_array();

    HuboCan::HuboDescription _desc;

    ach_channel_t _cmd_chan;
    ach_channel_t _agg_chan;

    pid_t _child;
    size_t _child_death_count;

    HuboRT::Daemonizer _rt;

    inline Aggregator(const Aggregator& doNotCopy) { }
    inline Aggregator& operator=(const Aggregator& doNotCopy) { return *this; }

};

} // namespace HuboCan


#endif // AGGREGATOR_HPP
