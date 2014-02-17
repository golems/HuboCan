#ifndef INFOTYPES_HPP
#define INFOTYPES_HPP

#include <vector>
#include <string>
#include <stddef.h>

namespace HuboState {
enum error_flag {

    NONE                = 0,
    INDEX_OUT_OF_BOUNDS = 1,
    ARRAY_MISMATCH      = 1 >> 1,
    ACH_ERROR           = 1 >> 2,

};

typedef int error_result_t;
} // namespace HuboState

typedef size_t JointIndex;
typedef std::vector<size_t> IndexArray;
typedef std::vector<double> ValueArray;
typedef std::vector<std::string> StringArray;


#endif // INFOTYPES_HPP
