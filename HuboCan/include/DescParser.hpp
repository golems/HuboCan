#ifndef DESCPARSER_HPP
#define DESCPARSER_HPP

#include "InfoTypes.hpp"
#include <ostream>

namespace HuboCan {

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

class DescParser
{
public:

    bool load_file(const std::string& filename);

    void report_error(const std::string& error_message);

    bool next_line(StringArray& components);

    StringArray get_components(const std::string& line);

    void clear_front_back_whitespace(std::string& line);
    void clear_front_whitespace(std::string& line);
    void clear_back_whitespace(std::string& line);

protected:

    DescArray _contents;
    size_t _current_index;

    StringArray _filename_list;
    bool _push_back_file(const std::string& filename);

};

} // namespace HuboCan


inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::DescLine& desc)
{
    oStrStream << "Filename:" << desc.file <<", Line:" << desc.line_num << "\n"
                  << " -- '" << desc.data << "'" << std::endl;
    return oStrStream;
}

#endif // DESCPARSER_HPP
