/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

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

    bool interpolate();
    bool interpolate(hubo_path_interp_t type);
    
    bool check_limits() const;
    
    void get_active_indices(std::vector<size_t>& mapping);
    bool get_active_joint_limits(std::vector<hubo_joint_limits_t>& limits);

protected:

    bool _optimal_interpolation(const Eigen::VectorXd& velocities,
                                const Eigen::VectorXd& accelerations,
                                double frequency);
    bool _spline_interpolation(const Eigen::VectorXd& velocities,
                               const Eigen::VectorXd& accelerations,
                               double frequency);
    bool _densify();

};

} // namespace HuboPath

std::ostream& operator<<(std::ostream& stream, const HuboPath::Trajectory& traj);

#endif // HUBOPATH_TRAJECTORY_HPP
