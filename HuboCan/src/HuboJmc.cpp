
#include "HuboCan/HuboJmc.hpp"
#include "../HuboCanId.hpp"
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

void HuboJmc::update()
{
    if(NULL == _pump)
        return;
    
    _request_encoder_readings();
    _send_reference_commands();
}

void HuboJmc::_request_encoder_readings()
{
    _frame.can_id   = CMD_BYTE;
    _frame.data[0]  = info.hardware_index;
    _frame.data[1]  = GET_ENCODER;
    _frame.can_dlc = 3;
    
    if( _is_type("H2P_5CH") )
    {
        _frame.data[2] = 1; // An extra request frame to get the last two fingers on the hand
        _pump->addFrame(_frame, info.can_channel, 1);
        // Minor gripe: The frames that get returned by this extra encoder request have the
        // same ID as frames which are returned by the standard encoder request, which means
        // the order in which they are received matters for interpreting them correctly. This
        // is very bad CAN protocol and could easily result in botched encoder readings for
        // the fingers.
        
        // UNLESS they correctly report their DLCs, in which case they can be distinguished.
        // This needs to be investigated.
        // Note: Based on the existing hubo-ach code, it seems the DLC might be set correctly.
        
        // The CanPump runs through the frames opposite of the order in which they're added, so
        // we're adding the extra frame first and expecting it to arrive second.
    }
    
    _frame.data[2] = 0;
    _pump->addFrame(_frame, info.can_channel, 1);
}

void HuboJmc::_send_reference_commands()
{
    // TODO
}

void HuboJmc::decode(const can_frame_t &frame, size_t channel)
{
    if( channel != info.can_channel )
        return;
    
    // TODO: Decide if I should use an abstract factory to generate different HuboJmc classes
    // or if I should have the single HuboJmc class handle all different kinds of JMCs by using
    // a simple strcmp
    //
    // The cost is either repetitive strcmp operations or repetitive virtual table lookups.
    // I would guess that virtual table lookups would be faster (fewer operations) and would
    // result in prettier code.
    
    if( frame.can_id - ENCODER_REPLY == info.hardware_index )
    {
        
    }
}

