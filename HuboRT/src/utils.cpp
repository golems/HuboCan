
#include "../utils.hpp"

size_t HuboRT::split_components(const std::string &text, StringArray &array, char delimiter)
{
    array.resize(0);
    size_t pos=0, last_pos=0, count=0;
    while(std::string::npos != (pos = text.find(delimiter, last_pos)))
    {
        ++count;
        array.push_back(text.substr(last_pos, pos-last_pos));
        last_pos = pos+1;
    }
    
    return count;
}
