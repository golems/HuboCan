
#include "DdParser.hpp"
#include <fstream>
#include <ostream>
#include <iostream>

using namespace HuboCan;

DdParser::DdParser()
{
    _initialize();
}

void DdParser::_initialize()
{
    _next_index = 0;
    _current_device_type = "";
    _status = DD_END_OF_FILE;
    error_stream = &std::cerr;
}

bool DdParser::load_file(const std::string &filename)
{
    return _push_back_file(filename, false);
}

bool DdParser::_push_back_file(const std::string &filename, bool inclusion)
{
    if(_status == DD_ERROR)
        return false;
    
    if(inclusion)
    {
        for(size_t i=0; i<_filename_list.size(); ++i)
        {
            if( filename == _filename_list[i] )
            {
                error << "Infinite loop found after including '" << filename << "'! Aborting!";
                report_error();
                return false;
            }
        }
    }

    _filename_list.push_back(filename);

    std::ifstream filestream;
    filestream.open(filename.c_str());

    if(!filestream.is_open())
    {
        error << "Could not open '" << filename << "'!" << std::endl;
        report_error();
        return false;
    }
    
    if(inclusion)
    {
        _contents.erase(_contents.end());
    }

    size_t count = 0;
    while(filestream.good())
    {
        ++count;
        DescLine new_line;
        new_line.file = filename;
        new_line.line_num = count;

        std::getline(filestream, new_line.data);

        _contents.push_back(new_line);
        _next_index = _contents.size();

        StringArray inclusion_check = get_components(new_line.data);
        if(inclusion_check.size() > 0)
        {
            if(inclusion_check[0] == "include")
            {
                if(inclusion_check.size() > 1)
                {
                    if(!_push_back_file(inclusion_check[1], true))
                        return false;
                }
                else
                {
                    error << "The keyword 'include' must be followed by a file name!";
                    report_error();
                }
            }
        }
    }

    _next_index = 0;
    _status = DD_OKAY;
    return true;
}

StringArray DdParser::get_components(const std::string &line)
{
    StringArray result;

    size_t progress = 0;
    std::string remaining = line;
    while(remaining.size() > 0)
    {
        progress += clear_front_back_whitespace(remaining);

        if(remaining.size() > 0)
        {
            size_t count=0;
            
            if(remaining.at(0) == '\"')
            {
                ++count;
                while(remaining.at(count) != '\"')
                {
                    ++count;
                    
                    if(count >= remaining.size())
                    {
                        error << "Unclosed quotes starting at character #" << progress-1;
                        report_error();
                        result.clear();
                        return result;
                    }
                }
                
                if(count+1 < remaining.size())
                {
                    if(!isspace(remaining.at(count+1)))
                    {
                        error << "Found illegal quotes at character #" << progress+count+1;
                        report_error();
                        result.clear();
                        return result;
                    }
                }
                
                progress += count+1;
                std::string component = remaining.substr(1, count-1);
                remaining = remaining.substr(count+1, std::string::npos);
                result.push_back(component);
            }
            else
            {
                while(!isspace(remaining.at(count)))
                {
                    if(remaining.at(count) == '\"')
                    {
                        error << "Found illegal quotes at character #" << progress+count+1;
                        report_error();
                        result.clear();
                        return result;
                    }
                    
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
    }

    return result;
}

size_t DdParser::clear_front_back_whitespace(std::string &line)
{
    clear_back_whitespace(line);
    return clear_front_whitespace(line);
}

size_t DdParser::clear_front_whitespace(std::string &line)
{
    if(line.size() == 0)
        return 0;

    size_t count=0;
    while(isspace(line.at(count)))
    {
        ++count;
        if(count >= line.size() )
            return 0;
    }

    line = line.substr(count, std::string::npos);
    
    return count;
}

void DdParser::clear_back_whitespace(std::string &line)
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

void DdParser::report_error()
{
    if(_next_index-1 < _contents.size())
    {
        (*error_stream) << "Error! " << _contents[_next_index-1];
    }
    else
        (*error_stream) << "next_index: " << _next_index-1 << ", size: " << _contents.size() << std::endl;
    
    (*error_stream) << error.str() << std::endl;
    
    _status = DD_ERROR;
}

dd_result_t DdParser::next_line(StringArray &components)
{
    components.clear();
    
    if(_status == DD_ERROR)
        return DD_ERROR;
    
    while(   components.size() == 0
          && _next_index < _contents.size())
    {
        components = get_components(_contents[_next_index].data);
        ++_next_index;
    }
    
    if(_status == DD_ERROR)
        return DD_ERROR;
    
    if(_next_index >= _contents.size())
    {
        _status = DD_END_OF_FILE;
        return _status;
    }
    
    if(components[0] == "begin")
    {
        if(_current_device_type != "")
        {
            error << "The keyword 'begin' is being used inside of a device's description!";
            report_error();
            return _status;
        }
        
        if(components.size() < 2)
        {
            error << "The keyword 'begin' must be followed by a device type!";
            report_error();
            _current_device_type = "";
            return _status;
        }
        else
        {
            _current_device_type = components[1];
        }
    }
    
    if(components[0] == "end")
    {
        if(components.size() < 2)
        {
            error << "The keyword 'end' must be followed by a device type!";
            report_error();
            return _status;
        }
        else
        {
            if(components[1] == _current_device_type)
            {
                _current_device_type = "";
            }
            else
            {
                error << "The device type accompanying this 'end' does not match the last 'begin'!";
                report_error();
                return _status;
            }
        }
        
        _status = DD_END_OF_DEVICE;
        return _status;
    }
    
    _status = DD_OKAY;
    return _status;
}











