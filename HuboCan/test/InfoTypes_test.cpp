
#include "../InfoTypes.hpp"

using namespace HuboCan;

error_result_t make_an_error()
{
    return ARRAY_MISMATCH;
}

int main(int , char* [])
{
    
    error_result_t okay;
    std::cout << okay << std::endl;
    
    error_result_t result(READ_ONLY);
    result.result = READ_ONLY;
    std::cout << result << " (" << result.result << ")" << std::endl;
    std::cout << READ_ONLY << std::endl;
    
    
    std::cout << make_an_error() << std::endl;
    
    result |= make_an_error();
    std::cout << result << std::endl;
    
    return 0;
}
