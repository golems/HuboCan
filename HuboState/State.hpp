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

#ifndef HUBOSTATE_STATE_HPP
#define HUBOSTATE_STATE_HPP

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "hubo_sensor_c.h"
#include "HuboCmd/hubo_cmd_c.h"
}
#include "HuboCan/HuboDescription.hpp"

#include "HuboData.hpp"

namespace HuboState {

class State
{
public:

    /*!
     * \fn State(double timeout=1)
     * \brief Standard constructor for the State class
     * \param timeout
     *
     * This default version of the constructor will check the HuboDescription Ach channel to see if
     * a description has been published. If one has, it will use that description to initialize all
     * of the state data structures.
     *
     * Initialization of the data structures is necessary before the State class can interact with
     * anything. If for some reason you need to use the State class without a description being
     * published in the usual way, you will need to use the other constructor
     * State(const HuboCan::HuboDescription &description).
     */
    State(double timeout=1);

    /*!
     * \fn State(const HuboCan::HuboDescription& description)
     * \brief Constructor for loading a specific HuboDescription
     * \param description
     *
     * This allows you to pass a description directly into the State class so that it can
     * initialize all of the state data structures without relying on a HuboDescription being
     * published.
     *
     * Recommended use of the State class is to use the default State(double timeout=1) constructor
     * to grab the HuboDescription which has been broadcasted. This constructor is for use in
     * special cases where you do not want to use the broadcasted description for some reason.
     */
    State(HuboCan::HuboDescription& description);
    virtual ~State();

    /*!
     * \fn receive_description
     * \brief Attempt to receive a new description
     * \param timeout_sec
     * \return
     *
     * This function allows you to continue attempting to receive a description in case the
     * initial attempt by the constructor failed.
     *
     * You can check whether the state has successfully been initialized by checking the value of
     * the initialized() function.
     *
     * This function can also be used to clear away the current description and load a new one in
     * the event that the description has been changed (but this is not recommended behavior, since
     * your physical Hubo should not be able to change spontaneously during runtime).
     */
    virtual bool receive_description(double timeout_sec=2);

    /*!
     * \fn load_description
     * \brief Load a specific HuboDescription
     * \param description
     *
     * This function allows you to load a specific HuboDescription in case the initial attempt by
     * the default constructor failed.
     *
     * You can check whether the state has successfully been initialized by checking the value of
     * the initalized() function.
     *
     * This function can also be used to clear away the current description and load a new one in
     * the event that you want a different description to be used (but this is not recommended
     * behavior, since your physical Hubo should not be able to change spontaneously during
     * runtime).
     */
    virtual void load_description(const HuboCan::HuboDescription& description);

    /*!
     * \fn update
     * \brief Grab the latest state data that has been published.
     * \param timeout_sec
     * \return
     *
     * You can specify how long you want to wait for a new frame to be published before giving up.
     * It is strongly recommended that this time be at least the expected duration of a single
     * control cycle, otherwise your program will probably end up busy-waiting.
     *
     * The argument report_sync should be set to false if the channel is communicating over a
     * network where you cannot reasonably expect the states to be updated in real time.
     */
    virtual HuboCan::error_result_t update(double timeout_sec=1, bool report_sync = true);

    /*!
     * \fn publish
     * \brief Broadcast the current data in this State instance
     * \return
     *
     * Note: This function should only be used by the single process which is responsible for
     * interpreting the sensor readings. So either the CAN interface or the simulation interface,
     * depending on whether your system is running physically or in simulation.
     */
    virtual HuboCan::error_result_t publish();

    HuboData<hubo_joint_state_t>    joints;
    HuboData<hubo_imu_state_t>      imus;
    HuboData<hubo_ft_state_t>       force_torques;

    /*!
     * \fn get_time()
     * \brief Returns the timestamp of the latest joint data
     * \return
     *
     * The timestamp of the latest joint data should be equivalent to the timestamp of the
     * rest of the sensor data, unless there is a major timing issue happening.
     */
    double get_time();

    /*!
     * \fn initialized()
     * \brief Indicates whether this State instance has been successfully initialized.
     * \return
     */
    inline bool initialized() { return _initialized && joints.is_initialized()
                && imus.is_initialized() && force_torques.is_initialized(); }
    
    inline const HuboCan::HuboDescription& get_description() const { return _desc; }

protected:

    bool _initialized;
//    bool _channels_opened;

    virtual void _initialize();
    virtual void _create_memory();

    hubo_cmd_data*    _last_cmd_data;
    HuboCan::HuboDescription _desc;

    State(const State& doNotCopy);
    State& operator=(const State& doNotCopy);

};

} // namespace HuboState

#endif // HUBOSTATE_STATE_HPP
