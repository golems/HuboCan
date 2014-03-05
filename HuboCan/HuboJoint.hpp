#ifndef HUBOJOINT_HPP
#define HUBOJOINT_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include <vector>
#include <map>

namespace HuboCan {

class HuboJoint
{
public:

    hubo_joint_info_t info;

protected:


};

typedef std::vector<HuboJoint*> HuboJointPtrArray;
typedef std::map<size_t,HuboJoint*> HuboJointPtrMap;

} // namespace HuboCan

#endif // HUBOJOINT_HPP
