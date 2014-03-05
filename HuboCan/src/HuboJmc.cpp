
#include "HuboCan/HuboJmc.hpp"
#include <sstream>

using namespace HuboCan;

bool HuboJmc::addJoint(HuboJoint *joint, std::string &error_report)
{
    joints.clear();

    HuboJointPtrMap::iterator check = _tempJointMap.find(joint->info.hardware_index);
    if(check != _tempJointMap.end())
    {
        std::stringstream report;
        report << "The JMC named '" << info.name << "' already has a joint in index #"
                  << joint->info.hardware_index << " named '" << check->second->info.name << "'";
        report << "\n -- Therefore we cannot add the joint named '" << joint->info.name << "' to that index.";
        error_report = report.str();
        return false;
    }

    _tempJointMap[joint->info.hardware_index] = joint;
    return true;
}

bool HuboJmc::sortJoints(std::string& error_report)
{
    HuboJointPtrMap::iterator it = _tempJointMap.begin();
    for(size_t i=0; i < _tempJointMap.size(); ++i)
    {
        if(it->first != i)
        {
            std::stringstream report;
            report << "Missing joint index #" << i
                   << " for the JMC named '" << info.name << "'!";

            error_report = report.str();

            return false;
        }

        joints.push_back(it->second);

        ++it;
    }

    return true;
}
