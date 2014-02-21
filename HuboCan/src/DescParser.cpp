
#include "DescParser.hpp"
#include <fstream>
#include <ostream>
#include <iostream>

using namespace HuboCan;

bool DescParser::load_file(const std::string &filename)
{
    return _push_back_file(filename);
}

bool DescParser::_push_back_file(const std::string &filename)
{
    for(size_t i=0; i<_filename_list.size(); ++i)
    {
        if( filename == _filename_list[i] )
        {
            std::cerr << "Infinite loop found after including '" << filename << "'! Aborting! Culprit line:\n"
                      << _contents[_current_index] << std::endl;
            return false;
        }
    }

    _filename_list.push_back(filename);

    DescArray tempArray;
    std::ifstream filestream;
    filestream.open(filename.c_str());

    if(!filestream.is_open())
    {
        std::cerr << "Could not open '" << filename << "'!" << std::endl;
        return false;
    }

    size_t count = 0;
    while(filestream.good())
    {
        ++count;
        DescLine new_line;
        new_line.file = filename;
        new_line.line_num = count;

        std::getline(filestream, new_line.data);

        _current_index = _contents.size();
        _contents.push_back(new_line);


    }

    return true;
}

StringArray DescParser::get_components(const std::string &line)
{
    StringArray result;

    size_t progress = 0;
    std::string remaining = line;
    while(remaining.size() > 0)
    {
        clear_front_back_whitespace(remaining);

        if(remaining.size() > 0)
        {
            size_t count=0;
            while(!isspace(remaining.at(count)))
            {
                ++count;
                if(count >= remaining.size())
                    break;
            }
            progress += count;
            std::string component = remaining.substr(0, count);
            remaining = remaining.substr(count, std::string::npos);
            result.push_back(component);
        }
    }

    return result;
}

void DescParser::clear_front_back_whitespace(std::string &line)
{
    clear_front_whitespace(line);
    clear_back_whitespace(line);
}

void DescParser::clear_front_whitespace(std::string &line)
{
    if(line.size() == 0)
        return;

    size_t count=0;
    while(isspace(line.at(count)))
    {
        ++count;
        if(count >= line.size() )
            return;
    }

    line = line.substr(count, std::string::npos);
}

void DescParser::clear_back_whitespace(std::string &line)
{
    if(line.size() == 0)
        return;

    size_t count = line.size()-1;
    while(isspace(line.at(count)))
    {
        --count;
        if(count == std::string::npos)
            return;
    }

    line = line.substr(0, count+1);
}














