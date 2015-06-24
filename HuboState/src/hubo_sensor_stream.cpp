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

#include "HuboState/hubo_sensor_stream.hpp"

std::ostream& operator<<(std::ostream& stream, const hubo_joint_state_t& state)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "pos:";
    stream.width(width);
    stream << state.position;
    stream << "  ref:";
    stream.width(width);
    stream << state.reference;

    stream << "  duty:";
    stream.width(width);
    stream << state.duty;

    stream << "  cur:";
    stream.width(width);
    stream << state.current;

    stream << "  temp:";
    stream.width(width);
    stream << state.temperature;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_imu_state_t& imu)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "Angles x:";
    stream.width(width);
    stream << imu.angular_position[0];
    stream << "  y:";
    stream.width(width);
    stream << imu.angular_position[1];
    stream << "  z:";
    stream.width(width);
    stream << imu.angular_position[2];

    stream << " | Velocities x:";
    stream.width(width);
    stream << imu.angular_velocity[0];
    stream << "  y:";
    stream.width(width);
    stream << imu.angular_velocity[1];
    stream << "  z:";
    stream.width(width);
    stream << imu.angular_velocity[2];

    return stream;
}

std::ostream& operator<<(std::ostream& stream, const hubo_ft_state_t& ft)
{
    stream.precision(3);
    int width = 7;
    stream << std::fixed;

    stream << "Forces x:";
    stream.width(width);
    stream << ft.force[0];
    stream << "  y:";
    stream.width(width);
    stream << ft.force[1];
    stream << "  z:";
    stream.width(width);
    stream << ft.force[2];

    stream << " | Torques x:";
    stream.width(width);
    stream << ft.torque[0];
    stream << "  y:";
    stream.width(width);
    stream << ft.torque[1];
    stream << "  z:";
    stream.width(width);
    stream << ft.torque[2];

    return stream;
}
