#ifndef HUBODESCRIPTION_HPP
#define HUBODESCRIPTION_HPP

#include "InfoTypes.hpp"
#include "HuboJmc.hpp"
#include "HuboJoint.hpp"
#include "HuboSensor.hpp"
#include "DdParser.hpp"

namespace HuboCan {

class HuboDescription
{
public:

    HuboDescription();
    virtual ~HuboDescription();

    /*!
     * \fn parseFile(const std::string& filename)
     * \brief Reads in a device description from the .dd file given by filename
     * \param filename
     * \return
     *
     * Returns true if and only if the parsing and processing had no errors.
     */
    virtual bool parseFile(const std::string& filename);

    /*!
     * \fn receiveInfo(double timeout=2)
     * \brief Attempts to receive the published information about what kind of Hubo is running
     * \param timeout
     * \return
     */
    error_result_t receiveInfo(double timeout_sec=1);

    /*!
     * \fn broadcastInfo()
     * \brief Sends out the HuboDescription info to the appropriate Ach Channels
     * \return
     */
    error_result_t broadcastInfo();

    /*!
     * \fn jointCount()
     * \brief How many joints the currently running Hubo has
     * \return
     */
    inline size_t getJointCount() { return joints.size(); }

    inline size_t getJmcCount() { return jmcs.size(); }

    /*!
     * \fn getJointIndex()
     * \brief Returns the joint index value corresponding to the given joint name
     * \param joint_name
     * \return
     *
     * Returns (size_t)(-1) if the given joint name does not exist.
     */
    size_t getJointIndex(const std::string& joint_name);

    /*!
     * \fn getJointIndices()
     * \brief Returns a std::vector of joint indices corresponding to the given joint names
     * \param joint_names
     * \return
     */
    IndexArray getJointIndices(StringArray joint_names);

    /*!
     * \fn getJointName()
     * \brief Returns the joint name of the requested index
     * \param joint_index
     * \return
     */
    std::string getJointName(size_t joint_index);

    /*!
     * \fn getJointNames(IndexArray joints)
     * \brief Returns a list of the joint names corresponding to the requested joints
     * \param joints
     * \return
     */
    StringArray getJointNames(IndexArray joints);

    /*!
     * \fn getJointNames()
     * \brief Returns an ordered list of all the joint names of the robot
     * \return
     */
    StringArray getJointNames();

    /*!
     * \fn getJointInfo()
     * \brief Returns a struct describing the requested joint
     * \param joint_index
     * \return
     */
    hubo_joint_info_t getJointInfo(size_t joint_index);

    /*!
     * \fn getJointTable()
     * \brief Produces a table with the basic specs of all loaded joints
     * \return
     */
    std::string getJointTable();

    /*!
     * \fn getJmcTable()
     * \brief Produces a table with the basic specs of all loaded JMCs
     * \return
     */
    std::string getJmcTable();


    size_t getJmcIndex(const std::string& jmc_name);

    HuboJointPtrArray joints;
    HuboJmcPtrArray jmcs;
    HuboSensorPtrArray sensors;

    HuboDescription& operator=(const HuboDescription& desc);
    HuboDescription(const HuboDescription& desc);

    inline bool okay() const { return _okay; }

protected:

    bool _okay;

    HuboJointPtrMap _tempJointMap;

    virtual bool _parseDevice(const std::string& device_type);

    virtual bool _parseJoint(bool strict=true);
    virtual bool _parseJMC(bool strict=true);
    virtual bool _parseIMU(bool strict=true);
    virtual bool _parseForceTorque(bool strict=true);

    virtual bool _postParseProcessing();

    DdParser _parser;

    hubo_info_data* _data;

    virtual void _copy_description(const HuboDescription& desc);

};


} // namespace HuboCan



#endif // HUBODESCRIPTION_HPP
