#ifndef INFOTYPES_HPP
#define INFOTYPES_HPP

#include <vector>
#include <string>
#include <stddef.h>

namespace HuboCan {
enum error_flag {

    OKAY                = 0,
    UNDEFINED_ERROR     = 1 >> 0,
    ARRAY_MISMATCH      = 1 >> 1,
    ACH_ERROR           = 1 >> 2,
    READ_ONLY           = 1 >> 3,
    INCOMPATIBLE_JOINT  = 1 >> 4,
    INDEX_OUT_OF_BOUNDS = 1 >> 5,
    SYNCH_ERROR         = 1 >> 6

};

typedef int error_result_t;
} // namespace HuboState

typedef std::vector<size_t> IndexArray;
typedef std::vector<double> ValueArray;
typedef std::vector<std::string> StringArray;

const size_t InvalidIndex = size_t(-1);

#endif // INFOTYPES_HPP
