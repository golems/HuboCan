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

#include "HuboPath/interpolation/Trajectory.h"
#include "HuboPath/hubo_path.hpp"

int main(int, char* [])
{
    std::list<Eigen::VectorXd> waypoints;

    Eigen::VectorXd point(4);
    point << 0, 0, 0, 0;
    waypoints.push_back(point);

    point << 3, -4, 6, 10;
    waypoints.push_back(point);

    point << -6, 2, 2, -2;
    waypoints.push_back(point);

    point << 0, 0, 0, 0;
    waypoints.push_back(point);

    std::cout << "constructing path" << std::endl;

    optimal_interpolation::Path path(waypoints, 0.01);

    std::cout << "constructed path" << std::endl;

    Eigen::VectorXd vel(4);
    vel << 1, 1, 1, 3;
    Eigen::VectorXd accel(4);
    accel << 0.8, 0.8, 0.8, 0.8;

    std::cout << "constructing trajectory" << std::endl;

    optimal_interpolation::Trajectory traj(path, vel, accel);

    std::cout << "constructed trajectory" << std::endl;

    std::cout << "Duration: " << traj.getDuration() << std::endl;

    HuboPath::Trajectory htraj;
    for(int i=0; i<point.size(); ++i)
        htraj.claim_joint(i);

    size_t traj_count = traj.getDuration()*200;

    Eigen::VectorXd next_waypoint;
    hubo_path_element_t elem;
    memset(&elem, 0, sizeof(elem));
    double dt = 1.0/200.0;
    size_t checker=0;
    for(size_t i=0; i<traj_count; ++i)
    {
        next_waypoint = traj.getPosition(i*dt);

        for(int j=0; j<point.size(); ++j)
        {
            elem.references[j] = next_waypoint[j];
        }
        htraj.push_back(elem);

        if( traj.getPathSegmentIndex(i*dt) > checker )
        {
            std::cout << "switch detected at " << i*dt << " ("
                      << traj.getPathSegmentIndex(i*dt)<< ")" << std::endl;
            ++checker;
        }
    }

//    std::cout << htraj << std::endl;

    return 0;
}
