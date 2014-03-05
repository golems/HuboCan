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
    ~HuboDescription();

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
    int receiveInfo(double timeout=2);

    /*!
     * \fn broadcastInfo()
     * \brief Sends out the HuboDescription info to the appropriate Ach Channels
     * \return
     */
    int broadcastInfo();

    /*!
     * \fn jointCount()
     * \brief How many joints the currently running Hubo has
     * \return
     */
    inline size_t jointCount() { return joints.size(); }

    inline size_t jmcCount() { return jmcs.size(); }

    /*!
     * \fn getJointIndex()
     * \brief Returns the joint index value corresponding to the given joint name
     * \param joint_name
     * \return
     *
     * Returns (size_t)(-1) if the given joint name does not exist.
     */
    JointIndex getJointIndex(const std::string& joint_name);

    /*!
     * \fn getJointIndices()
     * \brief Returns a std::vector of joint indices corresponding to the given joint names
     * \param joint_names
     * \return
     */
    IndexArray getJointIndices(StringArray joint_names);

    /*!
     * \fn getJointInfo()
     * \brief Returns a struct describing the requested joint
     * \param joint_index
     * \return
     */
    hubo_joint_info_t getJointInfo(JointIndex joint_index);

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

protected:

    HuboJointPtrMap _tempJointMap;

    virtual bool _parseDevice(const std::string& device_type);

    virtual bool _parseJoint(bool strict=true);
    virtual bool _parseJMC(bool strict=true);
    virtual bool _parseIMU(bool strict=true);
    virtual bool _parseForceTorque(bool strict=true);

    virtual bool _postParseProcessing();

    DdParser _parser;

    hubo_info_data* _data;

};


} // namespace HuboCan



#endif // HUBODESCRIPTION_HPP
