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

#include <iostream>

#include "HuboCan/DdParser.hpp"

int main(int, char* [])
{
    HuboCan::DdParser parser;
    parser.load_file("../HuboCan/misc/TestDevice.dd");
    
    StringArray components;
    
    while(parser.status() != HuboCan::DD_END_FILE && parser.status() != HuboCan::DD_ERROR)
    {
        if(parser.status() == HuboCan::DD_BEGIN_DEVICE)
            std::cout << "-- NEW DEVICE --\n(" << components[1] << ")" << std::endl;

        while(parser.next_line(components) == HuboCan::DD_OKAY)
        {
            if(components.size()==0)
                std::cout << "NO COMPONENTS" << std::endl;
            
            for(size_t i=0; i < components.size(); ++i)
            {
                std::cout << components[i] << " | ";
            }
            std::cout << std::endl;
        }
    }
    
    if(parser.status() == HuboCan::DD_ERROR)
    {
        std::cerr << "OH NO WE HAVE AN ERROR!!" << std::endl;
    }

    parser.current_line(components);
    std::cout << "Last components: ";
    for(size_t i=0; i < components.size(); ++i)
    {
        std::cout << components[i] << " | ";
    }
    std::cout << std::endl;
    
    return 0;
}
