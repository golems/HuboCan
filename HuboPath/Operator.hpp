#ifndef HUBOPATH_OPERATOR_HPP
#define HUBOPATH_OPERATOR_HPP

#include "HuboState/State.hpp"

#include "hubo_path.hpp"

#include <list>

namespace HuboPath {


class Operator : public HuboState::State 
{
public:
    
    Operator(double timeout=1);
    Operator(const HuboCan::HuboDescription& description);

    virtual bool open_channels();

    virtual bool receive_description(double timeout_sec);
    virtual void load_description(const HuboCan::HuboDescription& description);
    
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
     * \fn addWaypoints()
     * \brief Add a vector of waypoints onto the currently added ones
     * \param waypoints
     * \return 
     */
    HuboCan::error_result_t addWaypoints(const std::vector<Eigen::VectorXd>& waypoints);
    
    /*!
     * \fn addWaypoints()
     * \brief Add a list of waypoints onto the currently added ones
     * \param waypoints
     * \return 
     */
    HuboCan::error_result_t addWaypoints(const std::list<Eigen::VectorXd>& waypoints);
    
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
    void clearWaypoints();
    
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
    inline const Path& getWaypoints() const { return _input_path; }

    /*!
     * \fn send_new_trajectory()
     * \brief
     * \param instruction
     * \return
     */
    HuboCan::error_result_t sendNewTrajectory(
                            hubo_path_instruction_t instruction = HUBO_PATH_RUN,
                            int timeout_sec = 5);
    
    HuboCan::error_result_t sendNewTrajectory(
                            const Trajectory& premade_trajectory,
                            hubo_path_instruction_t instruction = HUBO_PATH_RUN,
                            int timeout_sec = 5);
    
    HuboCan::error_result_t sendInstruction(hubo_path_instruction_t instruction);

    hubo_path_command_t command;
    
    void setInterpolationMode(hubo_path_interp_t mode);
    bool interpolate();
    hubo_path_params_t params;

    const hubo_player_state_t& getPlayerState();
    
    const Trajectory& getCurrentTrajectory();

    // TODO: Make functions for inputting control schemes

protected:

    bool _constructed;
    
    bool _channels_opened;

    bool _check_mapping_set(std::string calling_function);
    bool _mapping_set;
    IndexArray _index_map;
    void _initialize_operator();
    Path _input_path;

    void _construct_trajectory();
    Trajectory _trajectory;
    hubo_player_state_t _state;

    void _update_state();

    ach_channel_t _instruction_chan;
    ach_channel_t _output_chan;
    ach_channel_t _feedback_chan;
    ach_channel_t _state_chan;
    
    void _outgoing_instruction_state_machine(hubo_path_instruction_t& s);
};

} // HuboPath

#endif // HUBOPATH_OPERATOR_HPP
