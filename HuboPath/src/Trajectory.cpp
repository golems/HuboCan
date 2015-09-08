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

#include "HuboPath/Trajectory.hpp"
#include "HuboPath/interpolation/Trajectory.h"
#include "HuboPath/interpolation/Spline.hpp"

bool HuboPath::Trajectory::interpolate(hubo_path_interp_t type)
{
    params.interp = type;
    return interpolate();
}

bool HuboPath::Trajectory::interpolate()
{
    if( elements.size() < 2 )
        return true;

    if( HUBO_PATH_RAW == params.interp )
    {
        return true;
    }
    
    std::vector<size_t> joint_mapping;
    get_active_indices(joint_mapping);
    
    std::vector<hubo_joint_limits_t> limits;
    if(!get_active_joint_limits(limits))
    {
        std::cout << "Cannot interpolate without knowing joint limits!" << std::endl;
        return false;
    }
    
    Eigen::VectorXd velocities(joint_mapping.size());
    Eigen::VectorXd accelerations(joint_mapping.size());
    Eigen::VectorXd max_velocities(joint_mapping.size());
    Eigen::VectorXd max_accelerations(joint_mapping.size());
    for(size_t j=0; j<joint_mapping.size(); ++j)
    {
        velocities[j] = limits[j].nominal_speed;
        max_velocities[j] = limits[j].max_speed;
        accelerations[j] = limits[j].nominal_accel;
        max_accelerations[j] = limits[j].max_accel;
    }
    
    double frequency = desc.okay()? desc.params.frequency : params.frequency;
    if( 0 == frequency )
    {
        std::cout << "Warning: Your Trajectory contained a frequency parameter of 0\n"
                     " -- We will default this to 200" << std::endl;
        frequency = 200;
    }
    
    if( HUBO_PATH_OPTIMAL == params.interp )
    {
        return _optimal_interpolation(velocities, accelerations, frequency);
    }
    else if( HUBO_PATH_SPLINE == params.interp )
    {
        return _spline_interpolation(velocities, accelerations, frequency);
    }
    else if( HUBO_PATH_DENSIFY == params.interp )
    {
        return _densify();
    }
    else if( HUBO_PATH_SATURATE == params.interp )
    {
      return _saturate(max_velocities, max_accelerations, frequency);
    }

    return false;
}

static void print_limit_violation(const std::string& type, 
                                  const std::string& name,
                                  size_t joint_index,
                                  double limit, double value,
                                  size_t traj_index)
{
    std::cout << "Joint " << name << "(" << joint_index << ") violated its "
              << type << " (" << limit << ") with a value of " << value
              << " at index " << traj_index << "!\n";
}

const double eps = 1e-6;
bool HuboPath::Trajectory::check_limits(bool enforceAccelerationLimits, bool printWarnings) const
{
    if(!desc.okay() && (params.use_custom_limits != 1))
    {
        std::cout << "Could not properly check trajectory limit violations!\n"
                  << " -- Need either a valid description to be loaded or to have custom limits set!"
                  << std::endl;
        return false;
    }
    
    size_t joint_count = 0;
    if(params.use_custom_limits == 1)
    {
        joint_count = HUBO_PATH_JOINT_MAX_SIZE;
    }
    else
    {
        joint_count = desc.joints.size();
    }
    
    double frequency = desc.okay()? desc.params.frequency : params.frequency;
    // TODO: Should I handle things differently if this is not a raw trajectory?
    if(frequency == 0)
    {
        std::cout << "No frequency is given for this trajectory. Defaulting to 200!" << std::endl;
        frequency = 200;
    }
    
    bool limits_okay = true;
    for(size_t i=0; i<elements.size(); ++i)
    {
        const hubo_path_element_t& elem = elements[i];
        const hubo_path_element_t& last_elem = ( i==0 ) ?
                    elements[0] : elements[i-1];
        
        const hubo_path_element_t& next_elem = ( i == elements.size()-1 ) ?
                    elements.back() : elements[i+1];
        
        for(size_t j=0; j<joint_count; ++j)
        {
            const hubo_joint_limits_t& limits = (params.use_custom_limits==1) ?
                                        params.limits[j] : desc.joints[j]->info.limits;
            std::string name = desc.okay() ?
                                   desc.joints[j]->info.name : "";
            
            if( ((params.bitmap >> j) & 0x01) != 0x01 )
            {
                continue;
            }

            if( !(elem.references[j] == elem.references[j]) )
            {
                print_limit_violation("NaN detection", name, j,
                                      0, elem.references[j], i);
                limits_okay = false;
            }

            if( limits.max_position+eps < elem.references[j] )
            {
                print_limit_violation("max position", name, j,
                                      limits.max_position, elem.references[j], i);
                limits_okay = false;
            }
            else if( limits.min_position-eps > elem.references[j] )
            {
                print_limit_violation("min position", name, j,
                                      limits.min_position, elem.references[j], i);
                limits_okay = false;
            }

            if( i == 0 )
            {
                // No sense in checking speed if this is the first element
                continue;
            }
            
            double speed = fabs(elem.references[j] - last_elem.references[j])
                           * frequency;
            if( speed > limits.max_speed + eps )
            {
                print_limit_violation("max speed", name, j,
                                      limits.max_speed, speed, i);
                limits_okay = false;
            }
            
            if( i == 0 || i == elements.size()-1 )
            {
                // No sense in checking acceleration if this is the first or last element
                continue;
            }

            double accel = fabs(next_elem.references[j]
                                - 2*elem.references[j]
                                + last_elem.references[j])
                           * frequency * frequency;

            if( accel > limits.max_accel + eps )
            {
                if(printWarnings)
                    print_limit_violation("max acceleration", name, j,
                                          limits.max_accel, accel, i);

                if(enforceAccelerationLimits)
                    limits_okay = false;
            }
        }
    }
    
    return limits_okay;
}

void HuboPath::Trajectory::get_active_indices(std::vector<size_t>& mapping) const
{
    mapping.clear();
    for(size_t i=0; i<HUBO_PATH_JOINT_MAX_SIZE; ++i)
    {
        if( ((params.bitmap >> i) & 0x01) == 1)
        {
            mapping.push_back(i);
        }
    }
}

std::vector<size_t> HuboPath::Trajectory::get_active_indices() const
{
  std::vector<size_t> mapping;
  get_active_indices(mapping);
  return mapping;
}

bool HuboPath::Trajectory::get_active_joint_limits(std::vector<hubo_joint_limits_t> &limits)
{
    if(!desc.okay() && (params.use_custom_limits != 1))
    {
        std::cout << "Could not retrieve joint limits!\n"
                  << " -- Need either a valid description to be loaded or to have custom limits set!"
                  << std::endl;
        return false;
    }
    
    std::vector<size_t> mapping;
    get_active_indices(mapping);
    
    hubo_joint_limits_t joint_limits;
    for(size_t i=0; i<mapping.size(); ++i)
    {
        size_t index = mapping[i];
        joint_limits = (params.use_custom_limits==1) ?
                           params.limits[index] : desc.joints[index]->info.limits;
        limits.push_back(joint_limits);
    }
    
    return true;
}

bool HuboPath::Trajectory::_optimal_interpolation(const Eigen::VectorXd& velocities,
                                                  const Eigen::VectorXd& accelerations,
                                                  double frequency)
{

    IndexArray joint_mapping;
    get_active_indices(joint_mapping);

    std::list<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_point(joint_mapping.size());
    for(size_t i=0; i<elements.size(); ++i)
    {
        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            next_point[j] = elements[i].references[joint_mapping[j]];
        }
        waypoints.push_back(next_point);
    }


    double tolerance = params.tolerance;
    if(tolerance == 0)
    {
        tolerance = 0.01;
    }

    optimal_interpolation::Path optimal_path(waypoints, tolerance);

    optimal_interpolation::Trajectory optimal_traj(optimal_path, velocities, accelerations);

    if(!optimal_traj.isValid())
    {
        std::cout << "The trajectory interpolator for HUBO_PATH_OPTIMIZE has returned invalid!"
                  << std::endl;
        return false;
    }


    double dt = 1.0/frequency;
    size_t traj_count = optimal_traj.getDuration()*frequency;
    std::vector<hubo_path_element_t> savedElements = elements;
    elements.clear();
    elements.resize(traj_count);
    for(size_t i=0; i<traj_count; ++i)
    {
        next_point = optimal_traj.getPosition(i*dt);
        elements[i] = savedElements[optimal_traj.getPathSegmentIndex(i*dt)];

        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            elements[i].references[joint_mapping[j]] = next_point[j];
        }
    }

    params.interp = HUBO_PATH_RAW;
    
    std::cout << "Successfully performed an optimal interpolation" << std::endl;

    return true;
}

bool HuboPath::Trajectory::_spline_interpolation(const Eigen::VectorXd& velocities,
                                                 const Eigen::VectorXd& accelerations,
                                                 double frequency)
{
    IndexArray joint_mapping;
    get_active_indices(joint_mapping);
    
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_point(joint_mapping.size());
    for(size_t i=0; i<elements.size(); ++i)
    {
        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            next_point[j] = elements[i].references[joint_mapping[j]];
        }
        waypoints.push_back(next_point);
    }
    
    spline_interpolation::Spline cubic_spline(waypoints, velocities, accelerations, frequency);
    if(!cubic_spline.valid())
    {
        std::cout << "ERROR: Could not produce a valid cubic spline interpolation!" << std::endl;
        return false;
    }
    
    std::vector<Eigen::VectorXd> interpolation = cubic_spline.getTrajectory();
    elements.clear();
    elements.resize(interpolation.size());
    
    for(size_t i=0; i<interpolation.size(); ++i)
    {
        Eigen::VectorXd& point = interpolation[i];
        
        for(size_t j=0; j<joint_mapping.size(); ++j)
        {
            elements[i].references[joint_mapping[j]] = point[j];
        }
    }
    params.interp = HUBO_PATH_RAW;
    
    std::cout << "Successfully performed a spline interpolation" << std::endl;
    
    return true;
}

bool HuboPath::Trajectory::_densify()
{
    // FIXME TODO
    return false;
}

static std::vector<hubo_path_element_t> saturate(
    const std::vector<hubo_path_element_t>& elements,
    const Eigen::VectorXd& max_velocities,
    const Eigen::VectorXd& max_accelerations,
    double frequency,
    const std::vector<size_t>& mapping,
    bool forward_only = false)
{
  std::vector<hubo_path_element_t> newElements;
  if(elements.size() < 2)
  {
    newElements.push_back(elements.front());
    return newElements;
  }

  newElements.reserve(10*elements.size());

  std::vector<Eigen::VectorXd> original_vels(elements.size(), Eigen::VectorXd(mapping.size()));
  std::vector<Eigen::VectorXd> original_accels(elements.size(), Eigen::VectorXd(mapping.size()));

  for(size_t i=0; i < elements.size(); ++i)
  {
    const hubo_path_element_t& elem = elements[i];
    const hubo_path_element_t& last_elem =
        ( i == 0 )? elements[0] : elements[i-1];

    const hubo_path_element_t& next_elem =
        ( i == elements.size()-1 )? elements.back() : elements[i+1];

    for(size_t j=0; j < mapping.size(); ++j)
    {
      const size_t index = mapping[j];

      original_vels[i][j] =
          std::abs(elem.references[index] - last_elem.references[index]) * frequency;

      original_accels[i][j] =
          std::abs(next_elem.references[index] - 2*elem.references[index] +last_elem.references[index])
          * pow(frequency,2);
    }
  }

  hubo_path_element_t lastElement = elements.front();
  hubo_path_element_t target = elements.front();
  hubo_path_element_t nextElement = elements.front();

  size_t upcomingIndex = 1;

  while(upcomingIndex <= elements.size())
  {
    double s = 1.0;

    for(size_t i=0; i < mapping.size(); ++i)
    {
      size_t index = mapping[i];
      double q_last = lastElement.references[index];
      double q = target.references[index];
      double v_max = max_velocities[i];

      double speed = std::abs(q-q_last)*frequency;
      if(speed <= v_max)
        continue;

      double scale = v_max / speed;
      if(scale < s)
        s = scale;
    }

    if(s < 1.0)
    {
      for(size_t i=0; i < mapping.size(); ++i)
      {
        size_t index = mapping[i];
        double& q = target.references[index];
        double q_last = lastElement.references[index];

        q = s*(q-q_last) + q_last;
      }
    }

    newElements.push_back(target);

    if(upcomingIndex < elements.size()+1)
    {
      bool skipNext = true;

      if( s < 1.0 )
      {
        for(size_t i=0; i < mapping.size(); ++i)
        {
          size_t index = mapping[i];
          double q = target.references[index];
          double q_next = nextElement.references[index];
          double v_max = max_velocities[i];

          double speed = std::abs(q_next - q) * frequency;
          if(speed > v_max)
          {
            skipNext = false;
            break;
          }
        }
      }

      if(skipNext)
      {
        nextElement =
            upcomingIndex < elements.size()? elements[upcomingIndex] : elements.back();
        ++upcomingIndex;
      }
    }

    double a = 1.0;
    for(size_t i=0; i < mapping.size(); ++i)
    {
      size_t index = mapping[i];
      double q_last = lastElement.references[index];
      double q = target.references[index];
      double q_next = nextElement.references[index];
      double a_max = max_accelerations[i];

      double accel = std::abs(q_next - 2*q + q_last) * pow(frequency, 2);
      if(accel <= a_max)
        continue;

      double scale = a_max / accel;
      if(scale < a)
        a = scale;
    }

    bool overshot = false;
    if(a < 1.0)
    {
      hubo_path_element_t potentialNext = nextElement;
      for(size_t i=0; i < mapping.size(); ++i)
      {
        size_t index = mapping[i];
        double q_last = lastElement.references[index];
        double q = target.references[index];
        double& q_next = potentialNext.references[index];
        double q_next_saved = nextElement.references[index];

        q_next = a * (q_next - 2*q + q_last) + 2*q - q_last;

        double min = std::min(q_next_saved, q);
        double max = std::max(q_next_saved, q);

        if( forward_only &&
           !(min <= q_next && q_next <= max) )
        {
          overshot = true;
          break;
        }
      }

      if(!overshot)
        nextElement = potentialNext;
    }

    lastElement = target;
    target = nextElement;
  }

  return newElements;
}

static std::vector<hubo_path_element_t> forward_saturate(
    const std::vector<hubo_path_element_t>& elements,
    const Eigen::VectorXd& max_velocities,
    const Eigen::VectorXd& max_accelerations,
    double frequency,
    const std::vector<size_t>& mapping)
{
  return saturate(elements, max_velocities, max_accelerations, frequency,
                  mapping, true);
}

bool HuboPath::Trajectory::_saturate(const Eigen::VectorXd& max_velocities,
                                     const Eigen::VectorXd& max_accelerations,
                                     double frequency)
{
//  std::cout << "original: " << elements.size() << std::endl;
//  std::vector<hubo_path_element_t> forward =
//      forward_saturate(elements, max_velocities, max_accelerations,
//                         frequency, get_active_indices());

//  std::cout << "forward: " << forward.size() << std::endl;

//  std::reverse(forward.begin(), forward.end());

//  elements = forward_saturate(forward, max_velocities, max_accelerations,
//                                frequency, get_active_indices());

//  std::cout << "reversed: " << elements.size() << std::endl;

//  std::reverse(elements.begin(), elements.end());

//  std::cout << "final: " << elements.size() << std::endl;

  elements = saturate(elements, max_velocities, max_accelerations,
                      frequency, get_active_indices());

  return true;
}






























