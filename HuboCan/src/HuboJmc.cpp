
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

std::string HuboJmc::header()
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(8);
    str << "JMC Name";
    str.width(12);
    str << "JMC Type";
    str.width(7);
    str << "Index";
    str.width(13);
    str << "CAN Channel";

    return str.str();
}

std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

std::string HuboJmc::table() const
{
    std::stringstream str;
    str.setf(std::ios::fixed);
    str.setf(std::ios::right);

    str.width(8);   // JMC Name
    str << info.name;
    str.width(12);  // JMC Type
    str << info.type;
    str.width(7);   // Index
    str << info.hardware_index;
    str.width(13);  // CAN Channel
    str << info.can_channel;

    return str.str();
}


