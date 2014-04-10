#ifndef HUBOJOINT_HPP
#define HUBOJOINT_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include <vector>
#include <map>
#include <string>
#include <sstream>

namespace HuboCan {

class HuboJoint
{
public:

    hubo_joint_info_t info;

    double encoder2radian(int encoder);
    int radian2encoder(double radian);

    inline static std::string header()
    {
        std::stringstream str;
        str.setf(std::ios::fixed);
        str.setf(std::ios::right);

        str.width(5);
        str << "Index";
        str.width(16);
        str << "Joint Name";
        str.width(16);
        str << "Board Index";
        str.width(8);
        str << "Drive";
        str.width(8);
        str << "Driven";
        str.width(10);
        str << "Harmonic";
        str.width(10);
        str << "Encoder";
        str.width(10);
        str << "JMC Name";

        return str.str();
    }

    inline std::string table() const
    {
        std::stringstream str;
        str.setf(std::ios::fixed);
        str.setf(std::ios::right);
        str.precision(0);

        str.width(5);   // Index
        str << info.software_index;
        str.width(16);  // Joint Name
        str << info.name;
        str.width(16);  // Board Index
        str << info.hardware_index;
        str.width(8);   // Drive
        str << info.drive_factor;
        str.width(8);   // Driven
        str << info.driven_factor;
        str.width(10);  // Harmonic
        str << info.harmonic_factor;
        str.width(10);  // Encoder
        str << info.enc_resolution;
        str.width(10);  // JMC Name
        str << info.jmc_name;

        return str.str();
    }

    bool updated;

protected:


};

typedef std::vector<HuboJoint*> HuboJointPtrArray;
typedef std::map<size_t,HuboJoint*> HuboJointPtrMap;

} // namespace HuboCan

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJoint& joint)
{
    oStrStream << joint.table();
    return oStrStream;
}

#endif // HUBOJOINT_HPP
