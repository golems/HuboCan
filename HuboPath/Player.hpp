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

#ifndef HUBOPATH_PLAYER_HPP
#define HUBOPATH_PLAYER_HPP

#include "HuboCmd/Commander.hpp"
#include "hubo_path.hpp"

namespace HuboPath {

class Player : public HuboCmd::Commander
{
public:
    Player(double timeout=1);
    Player(const HuboCan::HuboDescription& description);

    virtual bool receive_description(double timeout_sec);
    virtual void load_description(const HuboCan::HuboDescription& description);

    virtual bool open_channels();

    bool step();

    const hubo_path_element_t& current_element();
    
    void report_state();

protected:

    double _last_time;

    bool _receive_incoming_trajectory();
    void _send_element_commands(const hubo_path_element_t& elem);


    bool _first_step;
    bool _new_trajectory;
    bool _new_instruction;
    void _check_for_instructions();

    bool _channels_opened;
    void _initialize_player();

    Trajectory _trajectory;
    int _current_index;
    hubo_path_element_t _current_elem;
    hubo_path_element_t _last_elem;

    hubo_path_command_t _incoming_cmd;
    hubo_path_command_t _current_cmd;

    ach_channel_t _instruction_chan;
    ach_channel_t _input_chan;
    ach_channel_t _feedback_chan;
    ach_channel_t _state_chan;
};

} // namespace HuboPath


#endif // HUBOPATH_PLAYER_HPP
