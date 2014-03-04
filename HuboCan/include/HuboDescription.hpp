#ifndef HUBODESCRIPTION_HPP
#define HUBODESCRIPTION_HPP

#include "InfoTypes.hpp"
#include "HuboJmc.hpp"
#include "HuboJoint.hpp"
#include "DdParser.hpp"

namespace HuboCan {

class HuboDescription
{
public:

    HuboDescription();
    ~HuboDescription();

    virtual bool parseFile(const std::string& filename);

    /*!
     * \fn receiveInfo(double timeout=2)
     * \brief Attempts to receive the published information about what kind of Hubo is running
     * \param timeout
     * \return
     */
    int receiveInfo(double timeout=2);

    /*!
     * \fn jointCount()
     * \brief How many joints the currently running Hubo has
     * \return
     */
    inline size_t jointCount() { return _joints.size(); }

    inline size_t jmcCount() { return _jmcs.size(); }

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


    size_t getJmcIndex(const std::string& jmc_name);


protected:

    HuboJointPtrMap _tempJointMap;

    HuboJointPtrArray _joints;
    HuboJmcPtrArray _jmcs;

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
