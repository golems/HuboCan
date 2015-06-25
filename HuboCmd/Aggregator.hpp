/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HUBOCMD_AGGREGATOR_HPP
#define HUBOCMD_AGGREGATOR_HPP

#include <vector>
#include <map>

extern "C" {
#include "HuboCmd/hubo_cmd_c.h"
}

#include "HuboCan/HuboDescription.hpp"
#include "HuboCan/AchIncludes.hpp"
#include "HuboRT/Daemonizer.hpp"

#define HUBO_AGG_CHANNEL "hubo_agg"

namespace HuboCmd {

typedef std::vector<pid_t> PidArray;
typedef std::map<pid_t,bool> PidBoolMap;
typedef std::vector<hubo_joint_cmd_t> JointCmdArray;

class Aggregator
{
public:

    Aggregator(HuboCan::HuboDescription& description);
    ~Aggregator();

    void load_description(const HuboCan::HuboDescription& desc);

    bool open_channels();
    void close_channels();

    bool run();

    const JointCmdArray& update();
    
    inline JointCmdArray& last_commands()
    {
        return _aggregated_cmds;
    }

    inline hubo_joint_cmd_t& joint(size_t index)
    {
        if(index >= _aggregated_cmds.size())
            return _dummy;

        return _aggregated_cmds[index];
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

    Aggregator(const Aggregator& doNotCopy);
    Aggregator& operator=(const Aggregator& doNotCopy);

    hubo_joint_cmd_t _dummy;

};

} // namespace HuboCan

#endif // HUBOCMD_AGGREGATOR_HPP
