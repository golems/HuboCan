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

    HuboJoint();

    hubo_joint_info_t info;

    double encoder2radian(int encoder);
    int radian2encoder(double radian);

    static std::string header();

    std::string table() const;

    bool updated;
    int dropped_count; // Counter for how many times this joint's encoder data has failed to update
    int expected_replies;
    int received_replies;

protected:


};

typedef std::vector<HuboJoint*> HuboJointPtrArray;
typedef std::map<size_t,HuboJoint*> HuboJointPtrMap;

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJoint& joint)
{
    oStrStream << joint.table();
    return oStrStream;
}

} // namespace HuboCan

inline std::ostream& operator<<(std::ostream& oStrStream, const HuboCan::HuboJoint& joint)
{
    oStrStream << joint.table();
    return oStrStream;
}

#endif // HUBOJOINT_HPP
