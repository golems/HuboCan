#ifndef HUBO_PATH_HPP
#define HUBO_PATH_HPP

#include "Trajectory.hpp"

namespace HuboPath {

HuboCan::error_result_t send_trajectory(ach_channel_t& output_channel,
                                        ach_channel_t& feedback_channel,
                                        const Trajectory& trajectory,
                                        int max_wait_time = 5);

HuboCan::error_result_t receive_trajectory(ach_channel_t& input_channel,
                                           ach_channel_t& feedback_channel,
                                           Trajectory& new_trajectory,
                                           int max_wait_time = 5);

} // namespace HuboPath

const char* hubo_path_interp_to_string(const hubo_path_interp_t& type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_interp_t& type);

const char* hubo_path_instruction_to_string(const hubo_path_instruction_t& type);
std::ostream& operator<<(std::ostream& stream, const hubo_path_instruction_t& type);

//const char* hubo_path_element_to_string(const hubo_path_element_t& elem);
//std::ostream& operator<<(std::ostream& stream, const hubo_path_element_t& elem);

//const char* hubo_path_params_to_string(const hubo_path_params_t& params);
std::ostream& operator<<(std::ostream& stream, const hubo_path_params_t& params);

std::ostream& operator<<(std::ostream& stream, const HuboPath::Trajectory& traj);

const char* hubo_path_rx_state_to_string(const hubo_path_rx_state_t& state);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_state_t& state);

const char* hubo_path_rx_to_string(const hubo_path_rx_t& rx);
std::ostream& operator<<(std::ostream& stream, const hubo_path_rx_t& rx);


#endif // HUBO_PATH_HPP
