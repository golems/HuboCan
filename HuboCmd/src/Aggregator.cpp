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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>

#include "HuboCmd/Aggregator.hpp"

using namespace HuboCan;

namespace HuboCmd {

Aggregator::Aggregator(const HuboDescription& description)
{
    _initialize();
    load_description(description);
}

Aggregator::~Aggregator()
{
    free(_input_data);
    free(_output_data);
    free(_final_data);

    close_channels();
}

void Aggregator::_initialize()
{
    _memory_set = false;
    _input_data = NULL;
    _output_data = NULL;
    _final_data = NULL;

    _channels_opened = false;
    _is_launched = false;

    open_channels();
}

bool Aggregator::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_cmd_chan, HUBO_CMD_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        fprintf(stderr, "Error opening command channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
        return false;
    }

    result = ach_open(&_agg_chan, HUBO_AGG_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        fprintf(stderr, "Error opening aggregated channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;

        report_ach_errors(ach_close(&_cmd_chan), "Aggregator::open_channels",
                          "ach_close", HUBO_CMD_CHANNEL);
        return false;
    }

    report_ach_errors(ach_flush(&_cmd_chan), "Aggregator::open_channels",
                      "ach_flush", HUBO_CMD_CHANNEL);

    report_ach_errors(ach_flush(&_agg_chan), "Aggregator::open_channels",
                      "ach_flush", HUBO_AGG_CHANNEL);

    return true;
}

void Aggregator::close_channels()
{
    report_ach_errors(ach_close(&_cmd_chan), "Aggregator::close_channels",
                      "ach_close", HUBO_CMD_CHANNEL);

    report_ach_errors(ach_close(&_agg_chan), "Aggregator::close_channels",
                      "ach_close", HUBO_AGG_CHANNEL);
}

void Aggregator::_create_memory()
{
    memset(&_dummy, 0, sizeof(_dummy));
    free(_input_data);
    free(_output_data);
    free(_final_data);

    if(_desc.getJointCount() > 0)
    {
        _input_data  = hubo_cmd_init_data( _desc.getJointCount() );
        _output_data = hubo_cmd_init_data( _desc.getJointCount() );
        _final_data  = hubo_cmd_init_data( _desc.getJointCount() );
        _aggregated_cmds.resize(_desc.getJointCount());
        _pids.resize(_desc.getJointCount(), 0);
        _memory_set = true;
    }
    else
    {
        _input_data  = NULL;
        _output_data = NULL;
        _final_data  = NULL;
        _aggregated_cmds.resize(0);
        _pids.resize(0);
        _memory_set = false;
    }
}

void Aggregator::load_description(const HuboDescription &desc)
{
    _desc = desc;
    _create_memory();
}

bool Aggregator::run()
{
    if(!_memory_set)
    {
        std::cerr << "Trying to launch the aggregator before a valid Description has been loaded!" << std::endl;
        return false;
    }

    pid_t child = fork();
    if( child < 0 ) // Something failed
    {
        std::cerr << "Unable to fork the aggregator, code=" << errno << " ("
                     << strerror(errno) << ")" << std::endl;
        _is_launched = false;
        return false;
    }
    else if( child > 0 ) // We are in the parent process
    {
        _child = child;
        _child_death_count = _rt.child_processes_exited();
        _is_launched = true;
        return true;
    }
    else // We are in the aggregator process
    {
        _init_aggregator();
        _aggregator_loop();
        _quit_aggregator();
    }

    return false;
}

void Aggregator::_init_aggregator()
{
    if(!_rt.begin("hubo_cmd_aggregator"))
    {
        if(_rt.daemonization_status() == 13)
        {
            std::cout << "Did not have permissions to create the aggregator. Run with sudo!" << std::endl;
        }
        else if(_rt.daemonization_status() == 17)
        {
            std::cout << "Not launching a new aggregator, because one seems to already exist." << std::endl;
        }
        else
        {
            std::cout << "Could not properly daemonize the aggregator. Uncommon error ("
                      << _rt.daemonization_status() << "). Check the syslog!" << std::endl;
        }
        exit(0);
    }
    if(!open_channels())
    {
        std::cout << "Could not open the ach channels for aggregation: quitting!" << std::endl;
        _quit_aggregator();
    }
}

void Aggregator::_aggregator_loop()
{
    size_t max_expected_size = hubo_cmd_data_get_size(_input_data);
    while(_rt.good())
    {
        size_t fs;
        double quit_check = 1;
        struct timespec wait_time;
        clock_gettime( ACH_DEFAULT_CLOCK, &wait_time);
        long nano_wait = wait_time.tv_nsec + (long)(quit_check*1E9);
        wait_time.tv_sec += (long)(nano_wait/1E9);
        wait_time.tv_nsec = (long)(nano_wait%((long)1E9));
        ach_status_t result = ach_get(&_cmd_chan, _input_data, max_expected_size,
                                 &fs, &wait_time, ACH_O_WAIT);

        if(ACH_TIMEOUT == result)
        {
            continue;
        }

        if( ACH_OK != result )
        {
            std::cout << "Ach error: (" << (int)result << ")" << ach_result_to_string(result) << std::endl;
            // TODO: Broadcast the fact that we had an ach error?
            continue;
        }

        if(hubo_cmd_header_check(_input_data) != HUBO_DATA_OKAY)
        {
            std::cout << "Malformed command header!" << std::endl;
            // TODO: Broadcast the fact that the header was malformed?
            continue;
        }

        if( hubo_cmd_data_get_size(_input_data) != fs )
        {
            std::cout << "Data size error! Expected size:" << hubo_cmd_data_get_size(_input_data)
                         << ", Frame size:" << fs <<", Max size:" << max_expected_size << std::endl;
            // TODO: Broadcast the fact that we had a sizing error?
            continue;
        }

        _check_hubocan_state();

        _collate_input();

        _send_output();
    }
}

void Aggregator::_check_hubocan_state()
{
    // TODO: Grab the current state and update data structures accordingly
}

void Aggregator::_collate_input()
{
    size_t joint_count = hubo_cmd_data_get_total_num_joints(_input_data);
    for(size_t i=0; i<joint_count; ++i)
    {
        if(hubo_cmd_data_check_if_joint_is_set(_input_data, i) == 1)
        {
            if(_resolve_ownership(i))
            {
                _accept_command(i);
            }
        }
    }
}

void Aggregator::_send_output()
{
    ach_put(&_agg_chan, _output_data, hubo_cmd_data_get_size(_output_data));
}

void Aggregator::_quit_aggregator()
{
    close_channels();

    std::cout << "Closing down aggregator" << std::endl;
    _rt.close();
    exit(0);
}

bool Aggregator::_resolve_ownership(size_t joint_index)
{
    if(joint_index >= _pids.size())
    {
        std::cerr << "Attempting to collate a joint index which is out of bounds. THIS SHOULD BE IMPOSSIBLE. REPORT BUG IMMEDIATELY." << std::endl;
        return false;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)_input_data;

    if(_pids[joint_index] == 0) // Joint is unclaimed, so we're good to go
    {
        _pids[joint_index] = header->pid;
        return true;
    }

    hubo_cmd_data_get_joint_cmd(&_container, _input_data, joint_index);

    if(_pids[joint_index] == header->pid) // Incoming PID matches the current owner
    {
        if(HUBO_CMD_RELEASE == _container.mode)
        {
            _pids[joint_index] = 0;
            return true;
        }

        return true;
    }
    else if(HUBO_CMD_CLAIM == _container.mode)
    {
        std::cout << "PID# " << header->pid << " has demanded ownership of joint '"
                  << _desc.getJointName(joint_index) << "' (" << joint_index << ")." << std::endl;
        _pids[joint_index] = header->pid;
        return true;
    }

    std::cerr << "PID# " << header->pid << " is trying to command joint '"
                 << _desc.getJointName(joint_index) << "' (" << joint_index << ") which is already "
                    << "owned by PID# " << _pids[joint_index] << "! Please resolve this conflict!" << std::endl;

    // TODO: Report this as an error in a channel that goes all the way up to the user

    return false;
}

void Aggregator::_accept_command(size_t joint_index)
{
    hubo_cmd_data_get_joint_cmd(&_container, _input_data, joint_index);

    if(HUBO_CMD_CLAIM == _container.mode || HUBO_CMD_RELEASE == _container.mode)
    {
        _container.mode = HUBO_CMD_IGNORE;
    }

    hubo_cmd_data_set_joint_cmd(_output_data, &_container, joint_index);
}


const JointCmdArray& Aggregator::update()
{
    if(!_memory_set)
    {
        std::cout << "Trying to update commands before a valid HuboDescription has been loaded! Don't do that!!" << std::endl;
        return _aggregated_cmds;
    }

    size_t fs;
    ach_status_t result = ach_get(&_agg_chan, _final_data, hubo_cmd_data_get_size(_final_data), &fs, NULL, ACH_O_LAST);
    if( ACH_OK != result && ACH_STALE_FRAMES != result && ACH_MISSED_FRAME != result )
    { // TODO: Is ACH_STALE_FRAMES really okay? It might imply that the upstream pipeline has been frozen
        std::cout << "Unexpected Ach result: " << ach_result_to_string(result) << " (" << (int)result << ")" << std::endl;
        return _aggregated_cmds;
    }

    _copy_final_data_to_array();

    return _aggregated_cmds;
}

void Aggregator::_copy_final_data_to_array()
{
    if(_aggregated_cmds.size() != hubo_cmd_data_get_total_num_joints(_final_data))
    {
        std::cout << "Mismatch between final data size (" << hubo_cmd_data_get_total_num_joints(_final_data)
                  << ") and the command array size (" << _aggregated_cmds.size()
                  << ")!\n"
                  << " -- You must have a defunct aggregator for a different version of Hubo running!"
                  << std::endl;
        return;
    }

    for(size_t i=0; i < _aggregated_cmds.size(); ++i)
    {
        hubo_cmd_data_get_joint_cmd(&_container, _final_data, i);
        _aggregated_cmds[i] = _container;
    }
}


Aggregator::Aggregator(const Aggregator&) { }

Aggregator& Aggregator::operator =(const Aggregator&) { return *this; }

} // namespace HuboCmd
