#ifndef HUBODESCRIPTION_HPP
#define HUBODESCRIPTION_HPP

#include "InfoTypes.hpp"
#include "HuboJmc.hpp"
#include "HuboJoint.hpp"

namespace HuboCan {

typedef std::vector<HuboJoint*> JointPtrArray;
typedef std::vector<HuboJmc*> JmcPtrArray;

class HuboDescription
{
public:

    HuboDescription(bool receive=false, double timeout=2);
    HuboDescription(const std::string& filename);
    ~HuboDescription();

    JointPtrArray joints;
    JmcPtrArray jmcs;

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


protected:

    hubo_info_data* _data;

};


} // namespace HuboCan



#endif // HUBODESCRIPTION_HPP
