#include "HuboCan/DdParser.hpp"
#include <iostream>

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
