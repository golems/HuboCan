
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

    return -1;
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