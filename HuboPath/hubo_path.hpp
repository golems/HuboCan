#ifndef HUBO_PATH_HPP
#define HUBO_PATH_HPP

extern "C" {
#include "hubo_path_c.h"
}

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <iostream>
#include <vector>

#include "HuboCan/InfoTypes.hpp"
#include "HuboCan/AchIncludes.h"

namespace HuboPath {

typedef std::vector<Eigen::VectorXd> Path;
typedef std::vector<hubo_path_element_t> Trajectory;

HuboCan::error_result_t send_trajectory(ach_channel_t& output_channel,
                                        ach_channel_t& feedback_channel,
                                        const Trajectory& trajectory,
                                        int max_wait_time = 1);

HuboCan::error_result_t receive_trajectory(ach_channel_t& input_channel,
                                            ach_channel_t& feedback_channel,
                                            Trajectory& new_trajectory,
                                            int max_wait_time = 1);


} // namespace HuboPath

const char* hubo_path_interp_to_string(hubo_path_interp_t type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_interp_t& type);

const char* hubo_path_instruction_to_string(hubo_path_instruction_t type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_instruction_t& type);

const char* hubo_path_rx_state_to_string(hubo_path_rx_state_t state);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_state_t& state);

const char* hubo_path_rx_to_string(hubo_path_rx_t rx);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_t& rx);



#endif // HUBO_PATH_HPP
