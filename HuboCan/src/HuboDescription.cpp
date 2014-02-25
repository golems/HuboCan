
#include "HuboDescription.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace HuboCan;

HuboDescription::HuboDescription(bool receive, double timeout)
{
    _data = NULL;
    if(receive)
        receiveInfo(timeout);
}

HuboDescription::~HuboDescription()
{
    free(_data);

    for(size_t i=0; i<joints.size(); ++i)
    {
        delete joints[i];
    }

    for(size_t i=0; i<jmcs.size(); ++i)
    {
        delete jmcs[i];
    }
}

int HuboDescription::receiveInfo(double timeout)
{
    free(_data);
    _data = hubo_info_receive_data(timeout);

    if(_data == NULL)
        return -1;

    size_t joint_count = hubo_info_get_joint_count(_data);

    for(size_t i=0; i < joint_count; ++i)
    {
        HuboJoint* newJoint = new HuboJoint;
        newJoint->info = *hubo_info_get_joint_info(_data, i);
        joints.push_back(newJoint);
    }

    for(size_t i=0; i < joint_count; ++i)
    {
        HuboJmc* newJmc = new HuboJmc;
        newJmc->info = *hubo_info_get_jmc_info(_data, i);
        jmcs.push_back(newJmc);
    }

    free(_data);
    _data = NULL;

    return 0;
}

bool HuboDescription::parseFile(const std::string &filename)
{
    if(!_parser.load_file(filename))
        return false;

    StringArray components;
    while(_parser.status() != HuboCan::DD_END_FILE && _parser.status() != HuboCan::DD_ERROR)
    {
        _parser.next_line(components);
        if(_parser.status() == HuboCan::DD_BEGIN_DEVICE)
        {
            if(!_parseDevice(components[1]))
                return false;
        }
    }

    return true;
}

bool HuboDescription::_parseDevice(const std::string &device_type)
{
    if("Joint" == device_type)
    {
        if(!_parseJoint())
            return false;
    }
    else if("JMC" == device_type)
    {
        if(!_parseJMC())
            return false;
    }
    else if("IMU" == device_type)
    {
        if(!_parseIMU())
            return false;
    }
    else if("ForceTorque" == device_type)
    {
        if(!_parseForceTorque())
            return false;
    }

    return true;
}

bool HuboDescription::_parseJoint()
{
    hubo_joint_info_t new_joint_info;
    memset(&new_joint_info, 0, sizeof(hubo_joint_info_t));

    StringArray components;
    while(_parser.next_line(components) == HuboCan::DD_OKAY)
    {
        if("type" == components[0])
        {

        }


    }

    return true;
}

bool HuboDescription::_parseJMC()
{

    return true;
}

bool HuboDescription::_parseIMU()
{

    return true;
}

bool HuboDescription::_parseForceTorque()
{

    return true;
}

JointIndex HuboDescription::getJointIndex(const std::string& joint_name)
{
    for(JointIndex i=0; i < jointCount(); ++i)
    {
        std::string current_name(joints[i]->info.name);
        if(joint_name == current_name)
        {
            return i;
        }
    }

    return InvalidIndex;
}

IndexArray HuboDescription::getJointIndices(StringArray joint_names)
{
    IndexArray result;
    for(size_t i=0; i < joint_names.size(); ++i)
    {
        result.push_back(getJointIndex(joint_names[i]));
    }
    return result;
}

hubo_joint_info_t HuboDescription::getJointInfo(JointIndex joint_index)
{
    if( joint_index >= jointCount() )
    {
        fprintf(stderr, "Requesting joint info for an index which is out-of-bounds (%zu)!\n"
                " -- Maximum index is %zu\n", joint_index, jointCount()-1);
        hubo_joint_info_t info;
        memset(&info, 0, sizeof(hubo_joint_info_t));
        return info;
    }

    return joints[joint_index]->info;
}
