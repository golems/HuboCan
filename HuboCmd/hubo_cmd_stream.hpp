#ifndef HUBO_CMD_STREAM_HPP
#define HUBO_CMD_STREAM_HPP

extern "C" {
#include "hubo_cmd_c.h"
}

#include <iostream>
#include <string>

const char* hubo_cmd_mode_to_string(hubo_cmd_mode_t mode);
std::ostream& operator<<(std::ostream& stream, const hubo_cmd_mode_t& mode);

const char* hubo_data_error_to_string(hubo_data_error_t error);
std::ostream& operator<<(std::ostream& stream, const hubo_data_error_t& error);

std::ostream& operator<<(std::ostream& stream, const hubo_joint_cmd_t& cmd);

#endif // HUBO_CMD_STREAM_HPP
