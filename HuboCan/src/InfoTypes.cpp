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

#include <sstream>

#include "HuboCan/InfoTypes.hpp"

namespace HuboCan {

error_result_t::error_result_t()
{
    result = OKAY;
}

error_result_t::error_result_t(error_flag flag)
{
    result = flag;
}

std::string error_result_to_string(error_result_t error)
{
    std::stringstream stream;
    if(error.result==0)
    {
        return "OKAY";
    }
    
    bool first = true;
    for(size_t i=0; i <= max_error_flag_bit_location; ++i)
    {
        if( ((error.result>>i) & 0x01) == 1 )
        {
            if(!first)
            {
                stream << " | ";
            }
            else
            {
                first = false;
            }
            
            switch(i)
            {
                case 0: stream << "UNDEFINED_ERROR";        break;
                case 1: stream << "ARRAY_MISMATCH";         break;
                case 2: stream << "ACH_ERROR";              break;
                case 3: stream << "READ_ONLY";              break;
                case 4: stream << "INCOMPATIBLE_JOINT";     break;
                case 5: stream << "INDEX_OUT_OF_BOUNDS";    break;
                case 6: stream << "SYNCH_ERROR";            break;
                case 7: stream << "TIMEOUT";                break;
                case 8: stream << "UNINITIALIZED";          break;
                case 9: stream << "INTERRUPTED";            break;
                case 10:stream << "MALFORMED_HEADER";       break;

                default:stream << "UNKNOWN";                break;
            }
        }
    }
    
    return stream.str();
}

bool error_result_t::operator ==(const error_result_t& error)
{
    if(result == error.result)
    {
        return true;
    }
    
    return false;
}

bool error_result_t::operator !=(const error_result_t& error)
{
    return !(*this == error);
}

bool error_result_t::operator ==(const error_flag& flag)
{
    if(flag == 0)
        return (result == 0);
    
    if( (result & flag) == 0 )
    {
        return false;
    }
    
    return true;
}

bool error_result_t::operator !=(const error_flag& flag)
{
    return !(*this == flag);
}

error_result_t& error_result_t::operator =(const error_flag& flag)
{
    result = flag;
    return *this;
}

error_result_t& error_result_t::operator |=(const error_flag& flag)
{
    result |= flag;
    return *this;
}

error_result_t& error_result_t::operator |=(const error_result_t& error)
{
    result |= error.result;
    return *this;
}

error_result_t& error_result_t::operator &=(const error_flag& flag)
{
    result &= flag;
    return *this;
}

error_result_t& error_result_t::operator &=(const error_result_t& error)
{
    result &= error.result;
    return *this;
}

} // namespace HuboCan
