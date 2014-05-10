
#include "HuboCan/DdParser.hpp"
#include <iostream>

void print_array(const StringArray& array)
{
    std::cout << "Array count: " << array.size() << std::endl;
    for(size_t i=0; i < array.size(); ++i)
    {
        std::cout << array[i] << std::endl;
    }
}

int main(int, char* [])
{
    HuboCan::DdParser parser;

    std::string test_string = "this 12 \"a    test\" \t \t string \t yay";
    StringArray test_array = parser.get_string_components(test_string);
    print_array(test_array);

    std::cout << std::endl;

    test_string = "name    \"This is my name!\"";
    test_array = parser.get_string_components(test_string);
    print_array(test_array);


    return 0;
}
