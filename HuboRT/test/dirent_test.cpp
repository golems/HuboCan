
#include "HuboRT/Manager.hpp"
#include "HuboRT/manager_msg.hpp"

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "HuboRT/Daemonizer_C.h"
} // extern "C"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <dirent.h>
#include <fstream>

using namespace HuboRT;

int main(int, char* [])
{
    StringArray result;
    
    DIR* dptr = opendir(".");
    if(dptr == NULL)
    {
        return -1;
    }
    
    struct dirent* entry;
    while( NULL != (entry = readdir(dptr)) )
    {
        if(DT_REG == entry->d_type)
        {
            result.push_back(entry->d_name);
        }
    }
    
    for(size_t i=0; i < result.size(); ++i)
    {
        std::cout << result[i] << std::endl;
    }
    
    return 0;
}
