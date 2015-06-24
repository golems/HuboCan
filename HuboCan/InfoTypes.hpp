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

#ifndef HUBOCAN_INFOTYPES_HPP
#define HUBOCAN_INFOTYPES_HPP

#include <vector>
#include <string>
#include <iostream>

#include <stddef.h>

namespace HuboCan {

enum error_flag {

    OKAY                = 0,
    UNDEFINED_ERROR     = 1 <<  0,
    ARRAY_MISMATCH      = 1 <<  1,
    ACH_ERROR           = 1 <<  2,
    READ_ONLY           = 1 <<  3,
    INCOMPATIBLE_JOINT  = 1 <<  4,
    INDEX_OUT_OF_BOUNDS = 1 <<  5,
    SYNCH_ERROR         = 1 <<  6,
    TIMEOUT             = 1 <<  7,
    UNINITIALIZED       = 1 <<  8,
    INTERRUPTED         = 1 <<  9,
    MALFORMED_HEADER    = 1 << 10

    // make one for INVALID_COMMAND?
};
const size_t max_error_flag_bit_location = 10;


//typedef int error_result_t;
class error_result_t
{
public:
    
    error_result_t();
    error_result_t(error_flag flag);
    
    unsigned int result;
    
    bool operator==(const error_flag& flag);
    bool operator!=(const error_flag& flag);
    
    bool operator==(const error_result_t& error);
    bool operator!=(const error_result_t& error);
    
    error_result_t& operator=(const error_flag& flag);
    
    error_result_t& operator|=(const error_flag& flag);
    error_result_t& operator|=(const error_result_t& error);
    
    error_result_t& operator&=(const error_flag& flag);
    error_result_t& operator&=(const error_result_t& error);
};

std::string error_result_to_string(error_result_t error);

} // namespace HuboState

inline std::ostream& operator<<(std::ostream& stream, HuboCan::error_result_t error)
{
    stream << HuboCan::error_result_to_string(error);
    return stream;
}

typedef std::vector<size_t> IndexArray;
typedef std::vector<double> ValueArray;
typedef std::vector<std::string> StringArray;

const size_t InvalidIndex = size_t(-1);

#endif // HUBOCAN_INFOTYPES_HPP
