#ifndef INFOTYPES_HPP
#define INFOTYPES_HPP

#include <vector>
#include <string>
#include <stddef.h>
#include <iostream>

namespace HuboCan {
enum error_flag {

    OKAY                = 0,
    UNDEFINED_ERROR     = 1 << 0,
    ARRAY_MISMATCH      = 1 << 1,
    ACH_ERROR           = 1 << 2,
    READ_ONLY           = 1 << 3,
    INCOMPATIBLE_JOINT  = 1 << 4,
    INDEX_OUT_OF_BOUNDS = 1 << 5,
    SYNCH_ERROR         = 1 << 6,
    TIMEOUT             = 1 << 7,
    UNINITIALIZED       = 1 << 8,
    INTERRUPTED         = 1 << 9

};
const size_t max_error_flag_bit_location = 9;


//typedef int error_result_t;
class error_result_t
{
public:
    
    error_result_t();
    error_result_t(error_flag flag);
    
    int result;
    
    bool operator==(const error_flag& flag);
    bool operator!=(const error_flag& flag);
    
    bool operator==(const error_result_t& error);
    bool operator!=(const error_result_t& error);
    
    error_result_t& operator=(const error_flag& flag);
    
    error_result_t& operator|=(const error_flag& flag);
    error_result_t& operator|=(const error_result_t& error);
    
    error_result_t& operator&=(const error_flag& flag);
    error_result_t& operator&=(const error_result_t& error);
};

std::string error_result_to_string(error_result_t error);

} // namespace HuboState

inline std::ostream& operator<<(std::ostream& stream, HuboCan::error_result_t error)
{
    stream << HuboCan::error_result_to_string(error);
    return stream;
}

typedef std::vector<size_t> IndexArray;
typedef std::vector<double> ValueArray;
typedef std::vector<std::string> StringArray;

const size_t InvalidIndex = size_t(-1);

#endif // INFOTYPES_HPP
