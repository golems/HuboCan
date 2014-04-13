#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include "HuboCan/InfoTypes.hpp"

namespace HuboRT {

size_t split_components(const std::string& text, StringArray& array, char delimiter=':');

} // namespace HuboRT

#endif // UTILS_HPP
