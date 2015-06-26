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

#ifndef HUBOCAN_HUBOJMC_HPP
#define HUBOCAN_HUBOJMC_HPP

extern "C" {
#include "hubo_info_c.h"
#include "HuboCmd/hubo_aux_cmd_c.h"
} // extern "C"

#include "CanDevice.hpp"
#include "HuboJoint.hpp"

#include <string>
#include <string.h>

namespace HuboCmd {
class Aggregator;
} // namespace HuboCmd

namespace HuboState {

class State;

} // namespace HuboState

namespace HuboCan {

class HuboJmc : public CanDevice
{
public:

    HuboJmc();
    
    void assignAggregator(HuboCmd::Aggregator* agg);

    void assignState(HuboState::State* state);

    hubo_jmc_info_t info;
    HuboJointPtrArray joints;

    bool addJoint(HuboJoint* joint, std::string& error_report);
    bool sortJoints(std::string& error_report);

    void auxiliary_command(const hubo_aux_cmd_t& command);
    
    static std::string header();

    std::string table() const;

protected:

    std::vector<hubo_aux_cmd_t>  _aux_commands;

    HuboCmd::Aggregator* _agg;
    HuboState::State* _state;
    
    HuboJointPtrMap _tempJointMap;
    can_frame_t _frame;
    
    inline bool _is_type(const char* type) { return strcmp(info.type, type) == 0; }

};

typedef std::vector<HuboJmc*> HuboJmcPtrArray;

class Hubo2PlusBasicJmc : public HuboJmc
{
public:

    Hubo2PlusBasicJmc();
    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

    virtual unsigned long sign_convention_converter(int encoder_value);

protected:

    bool _startup;
    virtual void _cycle_reset();
    virtual void _process_auxiliary_commands();
    virtual void _request_encoder_readings();
    virtual void _send_reference_commands();

    virtual bool _decode_encoder_reading(const can_frame_t& frame);
    virtual bool _decode_status_reading(const can_frame_t& frame);

    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);

    // Joint homing
    virtual void _handle_home_joint(const hubo_aux_cmd_t& cmd);
    virtual void _handle_home_all_joints();

    // Motor ctrl switching
    virtual void _handle_ctrl_switch(const hubo_aux_cmd_t& cmd);


    virtual void _handle_rigid_reference_cmd();


};

class Hubo2Plus2chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2PlusNckJmc : public Hubo2PlusBasicJmc
{
public:

    // TODO: The neck has a different protocol than the rest of the joints

protected:

    virtual void _send_reference_commands();
    virtual bool _decode_encoder_reading(const can_frame_t& frame);

};

class Hubo2Plus5chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

    void _request_encoder_readings();
    void _send_reference_commands();

    bool _decode_encoder_reading(const can_frame_t& frame);
    bool _decode_status_reading(const can_frame_t &frame);

};

class DrcHubo2chJmc : public Hubo2Plus2chJmc
{
public:

    // TODO: Handle compliance control

protected:

};

class DrcHubo3chJmc : public Hubo2PlusBasicJmc
{
public:


protected:

    void _send_reference_commands();

    bool _decode_encoder_reading(const can_frame_t& frame);

};

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

} // namespace HuboCan

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

#endif // HUBOCAN_HUBOJMC_HPP
