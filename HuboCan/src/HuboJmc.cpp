
#include "HuboJmc.hpp"
#include <sstream>

using namespace HuboCan;

void HuboJmc::addJoint(HuboJoint *joint)
{
    joints.clear();
    _tempJointMap[joint->info.hardware_index] = joint;
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
                   << " for the JMC named " << info.name << "!";

            error_report = report.str();

            return false;
        }

        joints.push_back(it->second);

        ++it;
    }

    return true;
}
