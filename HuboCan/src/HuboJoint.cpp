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

#include "HuboCan/HuboJoint.hpp"
#include <math.h>
#include <string.h>

namespace HuboCan {

HuboJoint::HuboJoint()
{
    memset(&info, 0, sizeof(info));
    updated = true;
    dropped_count = 0;

    expected_replies = 0;
    received_replies = 0;
}

double HuboJoint::encoder2radian(int encoder)
{
    return 2*M_PI*(double)(encoder*info.drive_factor)
            /(info.driven_factor*info.harmonic_factor*info.enc_resolution);
}

int HuboJoint::radian2encoder(double radian)
{
    return radian*(info.driven_factor*info.harmonic_factor*info.enc_resolution)
            /(2*M_PI*info.drive_factor);
}

std::string HuboJoint::header()
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(5);
    str << "Index";
    str.width(16);
    str << "Joint Name";
    str.width(16);
    str << "Board Index";
    str.width(8);
    str << "Drive";
    str.width(8);
    str << "Driven";
    str.width(10);
    str << "Harmonic";
    str.width(10);
    str << "Encoder";
    str.width(10);
    str << "JMC Name";

    return str.str();
}

std::string HuboJoint::table() const
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);
    str.precision(0);

    str.width(5);   // Index
    str << info.software_index;
    str.width(16);  // Joint Name
    str << info.name;
    str.width(16);  // Board Index
    str << info.hardware_index;
    str.width(8);   // Drive
    str << info.drive_factor;
    str.width(8);   // Driven
    str << info.driven_factor;
    str.width(10);  // Harmonic
    str << info.harmonic_factor;
    str.width(10);  // Encoder
    str << info.enc_resolution;
    str.width(10);  // JMC Name
    str << info.jmc_name;

    return str.str();
}

} // HuboCan
