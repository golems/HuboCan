#ifndef HUBOPATH_TRAJECTORY_HPP
#define HUBOPATH_TRAJECTORY_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

extern "C" {
#include "hubo_path_c.h"
}

#include <string>
#include <iostream>
#include <vector>

#include "HuboCan/InfoTypes.hpp"
#include "HuboCan/AchIncludes.h"
#include "HuboCan/HuboDescription.hpp"

namespace HuboPath {

typedef std::vector<Eigen::VectorXd> Path;

class Trajectory
{
public:

    HuboCan::HuboDescription desc;

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

} // namespace HuboPath

#endif // HUBOPATH_TRAJECTORY_HPP
