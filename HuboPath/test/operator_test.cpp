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

#include "HuboPath/Operator.hpp"

int main(int argc, char* argv[])
{
    HuboPath::Operator op;

    int test_number = 1;
    for(int c=1; c < argc; ++c)
    {
        test_number = atoi(argv[c]);
    }

    StringArray mapping;
    mapping.push_back("RSP"); // 0
    mapping.push_back("RSR"); // 1
    mapping.push_back("RSY"); // 2
    mapping.push_back("REP"); // 3
    mapping.push_back("RWY"); // 4
    mapping.push_back("RWP"); // 5

    op.setJointIndices(mapping);

    Eigen::VectorXd vec;
    vec.resize(mapping.size());
    vec.setZero();

    if(test_number == 0)
    {
        vec.setZero();
        op.addWaypoint(vec);
    }
    else if(test_number == 1)
    {
        for(size_t i=0; i<30; ++i)
        {
            vec[2] = i*M_PI/180.0;
            op.addWaypoint(vec);
        }
    }
    else if(test_number == 2)
    {
        vec[1] = -45*M_PI/180.0;
        vec[6] = -30*M_PI/180.0;

        vec[3] = -20*M_PI/180.0;
        vec[2] = 30*M_PI/180.0;

        op.addWaypoint(vec);
    }
    else if(test_number == 3)
    {
//        mapping.clear();
//        mapping.push_back("RWY");
//        op.setJointIndices(mapping);
//        vec.resize(1);

//        vec[0] = 45*M_PI/180.0;
//        op.addWaypoint(vec);

//        vec[0] = 0;
//        op.addWaypoint(vec);
    }
    else if(test_number == 4)
    {
        vec << 0, 0, 0, 0, 0, 0;
        op.addWaypoint(vec);
        vec << -0.0503237, -0.0178896, 0.0301651, -0.0681551, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.100647, -0.0357792, 0.0603302, -0.13631, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.150971, -0.0536689, 0.0904953, -0.204465, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.201295, -0.0715585, 0.12066, -0.27262, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.251618, -0.0894481, 0.150826, -0.340776, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.301942, -0.107338, 0.180991, -0.408931, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.352266, -0.125227, 0.211156, -0.477086, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.402589, -0.143117, 0.241321, -0.545241, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.452913, -0.161007, 0.271486, -0.613396, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.503237, -0.178896, 0.301651, -0.681551, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.55356, -0.196786, 0.331816, -0.749706, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.603884, -0.214675, 0.361981, -0.817861, 0, 0; 
        op.addWaypoint(vec);
    }
    else if(test_number == 5)
    {
        vec << 0, 0, 0, 0, 0, 0; 
        vec << -0.0355226, -0.0461691, -0.0185826, -0.0743613, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.0710452, -0.0923382, -0.0371652, -0.148723, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.106568, -0.138507, -0.0557479, -0.223084, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.14209, -0.184676, -0.0743305, -0.297445, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.177613, -0.230845, -0.0929131, -0.371806, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.213136, -0.277015, -0.111496, -0.446168, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.248658, -0.323184, -0.130078, -0.520529, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.284181, -0.369353, -0.148661, -0.59489, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.319703, -0.415522, -0.167244, -0.669252, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.355226, -0.461691, -0.185826, -0.743613, 0, 0;
        op.addWaypoint(vec);
        vec << -0.390748, -0.50786, -0.204409, -0.817974, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.426271, -0.554029, -0.222991, -0.892336, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.461794, -0.600198, -0.241574, -0.966697, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.497316, -0.646367, -0.260157, -1.04106, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.532839, -0.692536, -0.278739, -1.11542, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.568361, -0.738705, -0.297322, -1.18978, 0, 0; 
        op.addWaypoint(vec);
        vec << -0.603884, -0.784875, -0.315905, -1.26414, 0, 0; 
        op.addWaypoint(vec);
    }
    else if(test_number == 6)
    {
        vec << -0.603884, -0.784875, -0.315905, -1.26414, 0, 0 ;
        op.addWaypoint(vec);
        vec << -0.53947, -0.735028, -0.315905, -1.26414, -0.0379551, 0.0418414 ;
        op.addWaypoint(vec);
        vec << -0.475055, -0.685181, -0.315905, -1.26414, -0.0759102, 0.0836827 ;
        op.addWaypoint(vec);
        vec << -0.410641, -0.635335, -0.315905, -1.26414, -0.113865, 0.125524 ;
        op.addWaypoint(vec);
        vec << -0.346227, -0.585488, -0.315905, -1.26414, -0.15182, 0.167365 ;
        op.addWaypoint(vec);
        vec << -0.281812, -0.535642, -0.315905, -1.26414, -0.189775, 0.209207 ;
        op.addWaypoint(vec);
        vec << -0.217398, -0.485795, -0.315905, -1.26414, -0.227731, 0.251048 ;
        op.addWaypoint(vec);
        vec << -0.152984, -0.435948, -0.315905, -1.26414, -0.265686, 0.29289 ;
        op.addWaypoint(vec);
        vec << -0.0885696, -0.386102, -0.315905, -1.26414, -0.303641, 0.334731 ;
        op.addWaypoint(vec);
        vec << -0.0241554, -0.336255, -0.315905, -1.26414, -0.341596, 0.376572 ;
        op.addWaypoint(vec);
        vec << 0.0402589, -0.286409, -0.315905, -1.26414, -0.379551, 0.418414 ;
        op.addWaypoint(vec);
        vec << 0.104673, -0.236562, -0.315905, -1.26414, -0.417506, 0.460255 ;
        op.addWaypoint(vec);
        vec << 0.169087, -0.186715, -0.315905, -1.26414, -0.455461, 0.502096 ;
        op.addWaypoint(vec);
        vec << 0.233502, -0.136869, -0.315905, -1.26414, -0.493416, 0.543938 ;
        op.addWaypoint(vec);
        vec << 0.297916, -0.0870221, -0.315905, -1.26414, -0.531371, 0.585779 ;
        op.addWaypoint(vec);
        vec << 0.36233, -0.0371755, -0.315905, -1.26414, -0.569326, 0.62762;
        op.addWaypoint(vec);
    }
    else
    {
        std::cout << "We do not go up to a test #" << test_number << "!" << std::endl;
        return 1;
    }

//    op.setInterpolationMode(HUBO_PATH_OPTIMAL);
    op.interpolate();
    const HuboPath::Trajectory& traj = op.getCurrentTrajectory();
    
    
    std::cout << op.getCurrentTrajectory() << "\n" << std::endl;
    
    traj.check_limits();
    
    
    
//    for(size_t i=0; i<traj.size(); ++i)
//    {
//        bool hit_first = false;
//        for(size_t j=0; j<HUBO_PATH_JOINT_MAX_SIZE; ++j)
//        {
//            if( ((traj.params.bitmap>>j) & 0x01) == 1 )
//            {
//                if(hit_first)
//                    std::cout << ", ";
//                else
//                    hit_first = true;
                
//                std::cout << traj.elements[i].references[j];
//            }
//        }
//        std::cout << ";" << std::endl;
//    }
    
//    op.sendNewTrajectory();

    return 0;
}
