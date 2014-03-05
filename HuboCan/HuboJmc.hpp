#ifndef HUBOJMC_HPP
#define HUBOJMC_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include "CanDevice.hpp"
#include "HuboJoint.hpp"
#include <string>

const char hubo2plus_1ch_code[] = "H2P_1CH";
const char hubo2plus_2ch_code[] = "H2P_2CH";
const char hubo2plus_3ch_code[] = "H2P_3CH";
const char hubo2plus_5ch_code[] = "H2P_5CH";

const char drchubo_2ch_code[] = "DRC_2CH";
const char drchubo_hybrid_code[] = "DRC_HYBRID";


namespace HuboCan {

class HuboJmc : public CanDevice
{
public:

    hubo_jmc_info_t info;
    HuboJointPtrArray joints;

    bool addJoint(HuboJoint* joint, std::string& error_report);
    bool sortJoints(std::string& error_report);

    inline static std::string header()
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

    inline std::string table() const
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

protected:

    HuboJointPtrMap _tempJointMap;

};

typedef std::vector<HuboJmc*> HuboJmcPtrArray;

class Hubo2PlusBasicJmc : public HuboJmc
{
public:

protected:

};

class Hubo2Plus2chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2Plus3chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class Hubo2Plus5chJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

class DrcHubo2chJmc : public Hubo2Plus2chJmc
{
public:

protected:

};

class DrcHuboHybridJmc : public Hubo2PlusBasicJmc
{
public:

protected:

};

} // namespace HuboCan

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJmc& jmc)
{
    oStrStream << jmc.table();
    return oStrStream;
}

#endif // HUBOJMC_HPP
