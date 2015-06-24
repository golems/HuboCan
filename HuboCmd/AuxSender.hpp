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

#ifndef HUBOCMD_HUBOAUXSENDER_HPP
#define HUBOCMD_HUBOAUXSENDER_HPP

extern "C" {
#include "hubo_aux_cmd_c.h"
}

#include "HuboCan/HuboDescription.hpp"

namespace HuboCmd {

// TODO: Come up with a better name for this class

class AuxSender
{
public:
    
    AuxSender(bool initialize=true, double timeout_sec=1);
    AuxSender(const HuboCan::HuboDescription& description);
    
    bool initialize(double timeout_sec=1);

    bool receive_description(double timeout_sec=1);
    void load_description(const HuboCan::HuboDescription& description);

    bool open_channels();

    void home_joint(size_t joint);
    void home_all_joints();

    void initialize_sensor(size_t sensor);
    void initialize_all_imus();
    void initialize_all_fts();
    void initialize_all_sensors();

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

#endif // HUBOCMD_HUBOAUXSENDER_HPP
