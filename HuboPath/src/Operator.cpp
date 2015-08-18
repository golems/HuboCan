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

#include "HuboPath/Operator.hpp"

namespace HuboPath {

Operator::Operator(double timeout) :
    HuboState::State(timeout)
{
    _initialize_operator();
}

Operator::Operator(HuboCan::HuboDescription& description) :
    HuboState::State(description)
{
    _initialize_operator();
}

void Operator::_initialize_operator()
{
    _mapping_set = false;
    _channels_opened = false;
    _constructed = false;
    _trajectory.clear();
    _trajectory.desc = _desc;
    memset(&command, 0, sizeof(command));
    memset(&params, 0, sizeof(params));

    open_channels();
    
    if(_desc.okay())
        _trajectory.desc = _desc;
}

bool Operator::receive_description(double timeout_sec)
{
    bool success = HuboState::State::receive_description(timeout_sec);

    if(success)
        _trajectory.desc = _desc;

    return success;
}

void Operator::load_description(const HuboCan::HuboDescription& description)
{
    HuboState::State::load_description(description);

    _trajectory.desc = _desc;
}

bool Operator::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;
    ach_status_t result = ach_open(&_instruction_chan, HUBO_PATH_INSTRUCTION_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path instruction channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    report_ach_errors(ach_flush(&_instruction_chan), "Operator::open_channels",
                      "ach_flush", HUBO_PATH_INSTRUCTION_CHANNEL);

    result = ach_open(&_output_chan, HUBO_PATH_INPUT_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path input channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    report_ach_errors(ach_flush(&_output_chan), "Operator::open_channels",
                      "ach_flush", HUBO_PATH_INPUT_CHANNEL);

    result = ach_open(&_feedback_chan, HUBO_PATH_FEEDBACK_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening path feedback channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    report_ach_errors(ach_flush(&_feedback_chan), "Operator::open_channels",
                      "ach_flush", HUBO_PATH_FEEDBACK_CHANNEL);

    result = ach_open(&_state_chan, HUBO_PATH_PLAYER_STATE_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        std::cout << "Error opening player state channel: "
                  << ach_result_to_string(result) << std::endl;
        _channels_opened = false;
    }
    report_ach_errors(ach_flush(&_state_chan), "Operator::open_channels",
                      "ach_flush", HUBO_PATH_PLAYER_STATE_CHANNEL);

    return _channels_opened;
}

HuboCan::error_result_t Operator::setJointIndices(const StringArray &joint_names)
{
    if(!_initialized)
    {
        std::cout << "This operator does not have a valid HuboDescription loaded yet!\n"
                  << " -- We cannot set joint indices until a description is loaded"
                  << std::endl;
        return HuboCan::UNINITIALIZED;
    }

    _constructed = false;
    
    _index_map = _desc.getJointIndices(joint_names);
    
    IndexArray invalids;
    for(size_t i=0; i < _index_map.size(); ++i)
    {
        if( _index_map[i] == InvalidIndex )
        {
            invalids.push_back(i);
        }
    }
    
    if(invalids.size() > 0)
    {
        std::cout << "The following joint names were invalid: ";
        for(size_t i=0; i<invalids.size(); ++i)
        {
            std::cout << joint_names[invalids[i]] << " (" << i << ")";
            if(i+1 < invalids.size())
                std::cout << ", ";
        }
        std::cout << std::endl;
        _index_map.clear();
        _mapping_set = false;
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _mapping_set = true;
    
    return HuboCan::OKAY;
}

HuboCan::error_result_t Operator::setJointIndices(const IndexArray &joint_indices)
{
    if(!_initialized)
    {
        std::cout << "This operator does not have a valid HuboDescription loaded yet!\n"
                  << " -- We cannot set joint indices until a description is loaded"
                  << std::endl;
        return HuboCan::UNINITIALIZED;
    }

    _constructed = false;
    
    IndexArray invalids;
    for(size_t i=0; i<joint_indices.size(); ++i)
    {
        if(joint_indices[i] > _desc.joints.size())
        {
            invalids.push_back(i);
        }
    }
    
    if(invalids.size() > 0)
    {
        std::cout << "The following joint indices exceeded the maximum ("
                  << _desc.joints.size() << "): ";
        for(size_t i=0; i<invalids.size(); ++i)
        {
            std::cout << joint_indices[invalids[i]] << " (" << i << ")";
            if(i+1 < invalids.size())
                std::cout << ", ";
        }
        std::cout << std::endl;
        _index_map.clear();
        _mapping_set = false;
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _index_map = joint_indices;
    _mapping_set = true;
    
    return HuboCan::OKAY;
}

const IndexArray& Operator::getIndexMap() const
{
    return _index_map;
}

bool Operator::_check_mapping_set(std::string calling_function)
{
    if(!_mapping_set)
    {
        std::cout << "Error: Cannot use Operator function (" << calling_function
                  << ") until the joint index mapping has been set.\n"
                  << " -- Use setJointIndices() before using "
                  << calling_function << std::endl;
        return false;
    }
    
    return true;
}

HuboCan::error_result_t Operator::addWaypoint(const Eigen::VectorXd &waypoint)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }

    _constructed = false;
    
    if((size_t)waypoint.size() != _index_map.size())
    {
        std::cout << "Invalid number of components in this waypoint ("
                  << waypoint.size() << "). Must be equal to " << _index_map.size()
                  << std::endl;
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _input_path.push_back(waypoint);
    
    return HuboCan::OKAY;
}

HuboCan::error_result_t Operator::addWaypoints(const std::list<Eigen::VectorXd> &waypoints)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }
    
    HuboCan::error_result_t result = HuboCan::OKAY;
    std::list<Eigen::VectorXd>::const_iterator it;
    size_t count = 0;
    for(it = waypoints.begin(); it != waypoints.end(); ++it)
    {
        result |= addWaypoint(*(it));
        
        if(result != HuboCan::OKAY)
        {
            std::cout << "Found item #" << count << " in the std::list to contain an error: "
                      << result << ". We will stop adding waypoints!" << std::endl;
            return result;
        }
        ++count;
    }
    
    return result;
}

HuboCan::error_result_t Operator::addWaypoints(const std::vector<Eigen::VectorXd> &waypoints)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }
    
    HuboCan::error_result_t result = HuboCan::OKAY;
    
    for(size_t i=0; i<waypoints.size(); ++i)
    {
        result |= addWaypoint(waypoints[i]);
        
        if(result != HuboCan::OKAY)
        {
            std::cout << "Found item #" << i << " in the std::vector to contain an error: "
                      << result << ". We will stop adding waypoints!" << std::endl;
            return result;
        }
    }
    
    return result;
}

HuboCan::error_result_t Operator::removeLast()
{
    if(_input_path.size() == 0)
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    
    _input_path.pop_back();
    
    return HuboCan::OKAY;
}

void Operator::clearWaypoints()
{
    _input_path.clear();
}

void Operator::_update_state()
{
    size_t fs;
    report_ach_errors(ach_get(&_state_chan, &_state, sizeof(_state), &fs, NULL, ACH_O_LAST),
                      "Operator::_update_state", "ach_get", HUBO_PATH_PLAYER_STATE_CHANNEL);
}

const hubo_player_state_t& Operator::getPlayerState()
{
    _update_state();
    return _state;
}

void Operator::setInterpolationMode(hubo_path_interp_t mode)
{
    params.interp = mode;
}

bool Operator::interpolate()
{
    _construct_trajectory();
    return _trajectory.interpolate();
}

void Operator::_construct_trajectory()
{
    if(_constructed)
        return;

    _trajectory.clear();
    _trajectory.params = params;

    _trajectory.claim_joints(_index_map);
    hubo_path_element_t elem;
    memset(&elem, 0, sizeof(elem));

    update(0);
    for(size_t j=0; j<_desc.joints.size(); ++j)
    {
        elem.references[j] = joints[j].reference;
    }
    _trajectory.push_back(elem);

    for(size_t i=0; i<_input_path.size(); ++i)
    {
        for(size_t j=0; j<_index_map.size(); ++j)
        {
            elem.references[_index_map[j]] = _input_path[i][j];

            // TODO: Give the element a control scheme
        }
        _trajectory.push_back(elem);
    }

    _constructed = true;
}

const Trajectory& Operator::getCurrentTrajectory()
{
    _construct_trajectory();
    return _trajectory;
}

HuboCan::error_result_t Operator::sendInstruction(hubo_path_instruction_t instruction)
{
    command.instruction = instruction;
    
    ach_status_t result = ach_put(&_instruction_chan, &command, sizeof(command));
    if( ACH_OK != result )
    {
        std::cout << "Ach error on player command channel: " << ach_result_to_string(result)
                  << std::endl;
        return HuboCan::ACH_ERROR;
    }
    
    return HuboCan::OKAY;
}

void Operator::_outgoing_instruction_state_machine(hubo_path_instruction_t &s)
{
    switch(s)
    {
        case HUBO_PATH_RUN:
        case HUBO_PATH_LOAD_N_GO:
            s = HUBO_PATH_LOAD_N_GO; break;
        case HUBO_PATH_PAUSE:
        case HUBO_PATH_REVERSE:
        case HUBO_PATH_LOAD:
            s = HUBO_PATH_LOAD; break;
        case HUBO_PATH_QUIT:
        default:
            s = HUBO_PATH_QUIT;
    }
}

HuboCan::error_result_t Operator::sendNewTrajectory(const Trajectory &premade_trajectory,
                                                    hubo_path_instruction_t instruction,
                                                    int timeout_sec)
{
    if(!_channels_opened)
    {
        return HuboCan::UNINITIALIZED;
    }

    if(!_check_mapping_set("sendNewTrajectory"))
    {
        return HuboCan::UNINITIALIZED;
    }

    if(premade_trajectory.size() < 2)
    {
        std::cout << "Trajectory size is " << premade_trajectory.size()
                  << ", but it must be at least size 2 before we send it!" << std::endl;
        return HuboCan::ARRAY_MISMATCH;
    }

    _update_state();
    
    _outgoing_instruction_state_machine(instruction);

    if(HUBO_PATH_QUIT == instruction)
    {
        std::cout << "Invalid instruction for a new trajectory: " << instruction << std::endl;
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }

    report_ach_errors(ach_flush(&_feedback_chan), "Operator::sendNewTrajectory",
                      "ach_flush", HUBO_PATH_FEEDBACK_CHANNEL);
    sendInstruction(instruction);

    return send_trajectory(_output_chan, _feedback_chan, premade_trajectory, timeout_sec);
}

HuboCan::error_result_t Operator::sendNewTrajectory(hubo_path_instruction_t instruction,
                                                    int timeout_sec)
{
    _construct_trajectory();

    _constructed = false;
    return sendNewTrajectory(_trajectory, instruction, timeout_sec);
}

} // namespace HuboPath
