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

#ifndef HUBOCMD_HUBOAUXRECEIVER_HPP
#define HUBOCMD_HUBOAUXRECEIVER_HPP

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

    void _register_with_jmc(size_t index);

    void _register_with_all_sensors();

    void _register_with_sensor(size_t index);

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

#endif // HUBOCMD_HUBOAUXRECEIVER_HPP
