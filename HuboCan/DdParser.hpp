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

#ifndef HUBOCAN_DESCPARSER_HPP
#define HUBOCAN_DESCPARSER_HPP

#include "InfoTypes.hpp"
#include <ostream>
#include <sstream>

namespace HuboCan {

typedef enum {
    
    DD_ERROR = 0,
    DD_OKAY,
    DD_BEGIN_DEVICE,
    DD_END_DEVICE,
    DD_END_FILE,
    
} dd_result_t;

class DdLine
{
public:

    inline DdLine()
    {
        data = "";
        file = "null";
        line_num = 0;
    }

    std::string data;
    std::string file;
    size_t line_num;
};

typedef std::vector<DdLine> DdLineArray;

class DdParser
{
public:
    
    DdParser();

    bool load_file(const std::string& filename);

    void report_error();
    std::stringstream& error() { return *_error_input_stream; }

    dd_result_t next_line(StringArray& components);
    bool current_line(StringArray& components);

    StringArray get_string_components(const std::string& line);

    size_t clear_front_back_whitespace(std::string& line);
    size_t clear_front_whitespace(std::string& line);
    void clear_back_whitespace(std::string& line);
    
    inline dd_result_t status() { return _status; }
    inline void reset()
    {
        _next_index = 0;
        _current_device_type = "";
    }
    
    std::ostream* error_output_stream;

protected:
    
    std::stringstream* _error_input_stream;

    void _initialize();

    DdLineArray _contents;
    size_t _next_index;
    std::string _current_device_type;
    dd_result_t _status;

    StringArray _filename_list;
    bool _push_back_file(const std::string& filename, bool inclusion, const std::string& parent_filename="");
    bool _inclusion_check(StringArray& line, std::string parent_filename);

};

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::DdLine& desc)
{
    oStrStream << "Filename:" << desc.file <<", Line:" << desc.line_num << "\n"
                  << " < " << desc.data << " > " << std::endl;
    return oStrStream;
}

} // namespace HuboCan


inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::DdLine& desc)
{
    oStrStream << "Filename:" << desc.file <<", Line:" << desc.line_num << "\n"
                  << " < " << desc.data << " > " << std::endl;
    return oStrStream;
}

#endif // HUBOCAN_DESCPARSER_HPP
