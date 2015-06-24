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

#ifndef HUBOCAN_HUBOSENSOR_HPP
#define HUBOCAN_HUBOSENSOR_HPP

#include <vector>
#include <string>

extern "C" {
#include "hubo_info_c.h"
#include "HuboCmd/hubo_aux_cmd_c.h"
} // extern "C"

#include "CanDevice.hpp"

namespace HuboCmd {
class Aggregator;
} // namespace HuboCmd

namespace HuboState {
class State;
} // namespace HuboState

namespace HuboCan {

class HuboSensor : public CanDevice
{
public:

    HuboSensor();

    void assign_pointers(HuboCmd::Aggregator* agg, HuboState::State* state);

    hubo_sensor_info_t info;

    void auxiliary_command(const hubo_aux_cmd_t& command);

protected:

    std::vector<hubo_aux_cmd_t> _aux_commands;

    HuboCmd::Aggregator* _agg;
    HuboState::State* _state;

    can_frame_t _frame;

};

class HuboImu : public HuboSensor
{
public:

    HuboImu(size_t index);

    size_t getImuIndex() const;

protected:

    size_t _index;
};

class Hubo2PlusImu : public HuboImu
{
public:

    Hubo2PlusImu(size_t index);

    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_imu_readings();
    virtual void _process_auxiliary_commands();
    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);
    virtual void _initialize_imu();
};

class Hubo2PlusTilt : public Hubo2PlusImu
{
public:

    Hubo2PlusTilt(size_t index);

    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_imu_readings();
    virtual void _initialize_imu();
};

class DrcHuboImu : public Hubo2PlusImu
{
public:

    DrcHuboImu(size_t index);

    // This class is identical to the Hubo2PlusImu. We create a separate class
    // for this type of IMU just in case its protocol gets changed in the future.
};

class HuboFt : public HuboSensor
{
public:

    HuboFt(size_t index);

    size_t getFtIndex() const;

protected:

    size_t _index;
};

class Hubo2PlusFt : public HuboFt
{
public:

    Hubo2PlusFt(size_t index);

    virtual void update();
    virtual bool decode(const can_frame_t& frame, size_t channel);

protected:

    virtual void _request_ft_readings();
    virtual void _process_auxiliary_commands();
    virtual void _handle_auxiliary_command(const hubo_aux_cmd_t& cmd);
    virtual void _initialize_ft();
};

class DrcHuboFt : public Hubo2PlusFt
{
public:

    DrcHuboFt(size_t index);

    // This class is identical to the Hubo2PlusFt. We create a separate class
    // for this type of FT just in case its protocol gets changed in the future.

};

typedef std::vector<HuboSensor*> HuboSensorPtrArray;

} // namespace HuboCan

#endif // HUBOCAN_HUBOSENSOR_HPP
