#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include "HuboCan/InfoTypes.hpp"

namespace HuboRT {

size_t split_components(const std::string& text, StringArray& array, char delimiter=':');

StringArray grab_files_in_dir(const std::string& directory);
StringArray grab_dirs_in_dir(const std::string& directory);

} // namespace HuboRT

#endif // UTILS_HPP
