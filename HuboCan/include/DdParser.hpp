#ifndef DESCPARSER_HPP
#define DESCPARSER_HPP

#include "InfoTypes.hpp"
#include <ostream>
#include <sstream>

namespace HuboCan {

typedef enum {
    
    DD_ERROR = 0,
    DD_OKAY,
    DD_END_OF_DEVICE,
    DD_END_OF_FILE,
    
} dd_result_t;

class DescLine
{
public:

    inline DescLine()
    {
        data = "";
        file = "null";
        line_num = 0;
    }

    std::string data;
    std::string file;
    size_t line_num;
};

typedef std::vector<DescLine> DescArray;

class DdParser
{
public:
    
    DdParser();

    bool load_file(const std::string& filename);

    void report_error();
    std::stringstream error;

    dd_result_t next_line(StringArray& components);

    StringArray get_components(const std::string& line);

    size_t clear_front_back_whitespace(std::string& line);
    size_t clear_front_whitespace(std::string& line);
    void clear_back_whitespace(std::string& line);
    
    inline dd_result_t status() { return _status; }
    
    std::ostream* error_stream;

protected:
    
    void _initialize();

    DescArray _contents;
    size_t _next_index;
    std::string _current_device_type;
    dd_result_t _status;

    StringArray _filename_list;
    bool _push_back_file(const std::string& filename, bool inclusion);

};

} // namespace HuboCan


inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::DescLine& desc)
{
    oStrStream << "Filename:" << desc.file <<", Line:" << desc.line_num << "\n"
                  << " < " << desc.data << " > " << std::endl;
    return oStrStream;
}

#endif // DESCPARSER_HPP
