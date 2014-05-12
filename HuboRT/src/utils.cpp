
#include "../utils.hpp"
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>

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

StringArray HuboRT::grab_files_in_dir(const std::string &directory)
{
    StringArray result;

    DIR* dptr = opendir(directory.c_str());
    if(dptr == NULL)
    {
        return result;
    }

    struct dirent* entry;
    while( NULL != (entry = readdir(dptr)) )
    {
        if(DT_REG == entry->d_type)
        {
            result.push_back(entry->d_name);
        }
    }

    return result;
}

StringArray HuboRT::grab_dirs_in_dir(const std::string &directory)
{
    StringArray result;

    DIR* dptr = opendir(directory.c_str());
    if(dptr == NULL)
    {
        return result;
    }

    struct dirent* entry;
    while( NULL != (entry = readdir(dptr)) )
    {
        if(DT_DIR == entry->d_type)
        {
            result.push_back(entry->d_name);
        }
    }

    return result;
}
