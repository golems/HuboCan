
#include <vector>
#include <string>
#include <iostream>

int main(int argc, char* argv[])
{
    const std::string name = "proc:hubocan:/usr/bin/hubocan:-r -s:";
    std::vector<std::string> array;
    
    array.resize(0);
    size_t pos = 0, last_pos=0, count=0;
    while(std::string::npos != (pos = name.find(":", last_pos)))
    {
        ++count;
        array.push_back(name.substr(last_pos, pos-last_pos));
        last_pos = pos+1;
    }
    
    std::cout << "Count: " << count << std::endl;
    for(size_t i=0; i<array.size(); ++i)
    {
        std::cout << array[i] << std::endl;
    }
    
    return 0;
}
