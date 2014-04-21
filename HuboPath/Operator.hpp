#ifndef HUBOPATH_OPERATOR_HPP
#define HUBOPATH_OPERATOR_HPP

#include "HuboState/State.hpp"

extern "C" {
#include "hubo_path_c.h"
}

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace HuboPath {

typedef std::vector<Eigen::VectorXd> Path;

class Operator : public HuboState::State 
{
public:
    
    /*!
     * \fn setJointIndices(const IndexArray& joint_names)
     * \brief Sets the mapping of incoming joint indices
     * \param joint_names
     * \return 
     * 
     * Different models, software, or versions of Hubo will have different joint
     * mappings. It is important to know how the indices of incoming waypoints are
     * meant to be interpreted. This function allows the mapping to be set to
     * whatever your convention is. It must be called before waypoints can be added.
     * 
     * The string version allows you to pass in a std::vector of joint names.
     */
    HuboCan::error_result_t setJointIndices(const StringArray& joint_names);
    
    /*!
     * \fn setJointIndices(const IndexArray& joint_names)
     * \brief Sets the mapping of incoming joint indices
     * \param joint_names
     * \return 
     * 
     * Different models, software, or versions of Hubo will have different joint
     * mappings. It is important to know how the indices of incoming waypoints are
     * meant to be interpreted. This function allows the mapping to be set to
     * whatever your convention is. It must be called before waypoints can be added.
     * 
     * The IndexArray version allows you to pass in a std::vector of size_t, each of
     * which refers to the index of the joint as given by the HuboDescription.
     * 
     * setJointIndices(const StringArray &joint_names) is the recommended version of
     * this function.
     */
    HuboCan::error_result_t setJointIndices(const IndexArray&  joint_indices);
    
    /*!
     * \fn setInputFrequency(double frequency);
     * \brief Informs the interpolator of your input frequency.
     * \param frequency
     * 
     * This only needs to be used for an interpolation mode of HUBO_PATH_DENSIFY.
     */
    void setInputFrequency(double frequency);
    
    /*!
     * \fn addWaypoint()
     * \brief Add a waypoint to the current list of points.
     * \param waypoint
     * \return 
     */
    HuboCan::error_result_t addWaypoint(const Eigen::VectorXd& waypoint);
    
    /*!
     * \fn pop_back()
     * \brief Removes the last waypoint that was added to the list
     * \return 
     */
    HuboCan::error_result_t removeLast();
    
    /*!
     * \fn clearWaypoints()
     * \brief Clears all currently existing waypoints
     * \return 
     */
    HuboCan::error_result_t clearWaypoints();
    
    /*!
     * \fn getWaypoints()
     * \brief Provides const access to the currently registered waypoints
     * \return 
     * 
     * This function is to allow the user to inspect the waypoints which
     * have currently been provided without granting the ability to alter
     * them.
     * 
     * Non-const access is not provided for the registered waypoint
     * array to ensure that incoming waypoints match the provided joint
     * index map, and that waypoints are not being entered without the
     * joint index map being set.
     */
    inline const Path& getWaypoints() { return _input_path; }
    
protected:
    
    Path _input_path;
    
};

} // HuboPath

#endif // HUBOPATH_OPERATOR_HPP
