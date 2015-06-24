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

#include "HuboCmd/hubo_cmd_stream.hpp"

#define return_enum_string( X ) case X : return #X ;

const char* hubo_cmd_mode_to_string(hubo_cmd_mode_t mode)
{
    switch(mode)
    {
        return_enum_string(HUBO_CMD_IGNORE);
        return_enum_string(HUBO_CMD_RIGID);
        return_enum_string(HUBO_CMD_COMPLIANT);
        return_enum_string(HUBO_CMD_HYBRID);
        return_enum_string(HUBO_CMD_CLAIM);
        return_enum_string(HUBO_CMD_RELEASE);

        default: return "HUBO_CMD_UNKNOWN";
    }

    return "HUBO_CMD_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_cmd_mode_t& mode)
{
    stream << hubo_cmd_mode_to_string(mode);
    return stream;
}

const char* hubo_data_error_to_string(hubo_data_error_t error)
{
    switch(error)
    {
        return_enum_string(HUBO_DATA_OKAY);
        return_enum_string(HUBO_DATA_NULL);
        return_enum_string(HUBO_DATA_OUT_OF_BOUNDS);
        return_enum_string(HUBO_DATA_READ_ONLY);
        return_enum_string(HUBO_DATA_UNAVAILABLE_INDEX);
        return_enum_string(HUBO_DATA_MALFORMED_HEADER);
        return_enum_string(HUBO_DATA_IMPOSSIBLE);

        default: return "HUBO_DATA_UNKNOWN";
    }

    return "HUBO_DATA_IMPOSSIBLE";
}

std::ostream& operator<<(std::ostream& stream, const hubo_data_error_t& error)
{
    stream << hubo_data_error_to_string(error);
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_joint_cmd_t& cmd)
{
    stream.precision(3);
    stream << std::fixed;
    int width = 7;

    stream << "mode:";
    stream.width(20);
    stream << cmd.mode;

    stream << "  pos:";
    stream.width(width);
    stream << cmd.position;

    stream << "  torque:";
    stream.width(width);
    stream << cmd.base_torque;

    stream << "  kP:";
    stream.width(width);
    stream << cmd.kP_gain;

    stream << "  kD:";
    stream.width(width);
    stream << cmd.kD_gain;

    return stream;
}
