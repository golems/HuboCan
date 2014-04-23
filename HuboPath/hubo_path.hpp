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

class Trajectory
{
public:
    
    hubo_path_params_t params;
    std::vector<hubo_path_element_t> elements;

    inline Trajectory()
    {
        clear();
    }
    
    inline size_t size() const
    {
        return elements.size();
    }
    
    inline hubo_path_element_t& operator[](size_t num)
    {
        return elements[num];
    }
    
    inline const hubo_path_element_t& operator[](size_t num) const
    {
        return elements[num];
    }
    
    inline void push_back(const hubo_path_element_t& new_elem)
    {
        elements.push_back(new_elem);
    }
    
    inline void clear()
    {
        elements.clear();
        memset(&params, 0, sizeof(params));
    }

    inline void claim_joint(size_t joint_index)
    {
        params.bitmap = params.bitmap | ( 0x01 << joint_index );
    }

    inline void claim_joints(const IndexArray& joint_indices)
    {
        for(size_t i=0; i<joint_indices.size(); ++i)
        {
            claim_joint(joint_indices[i]);
        }
    }
};


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
