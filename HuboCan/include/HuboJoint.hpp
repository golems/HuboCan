#ifndef HUBOJOINT_HPP
#define HUBOJOINT_HPP

extern "C" {
#include "hubo_info_c.h"
}

#include "CanDevice.hpp"

namespace HuboCan {

class HuboJoint : public CanDevice
{
public:

    hubo_joint_info_t info;

protected:


};

typedef std::vector<HuboJoint*> HuboJointPtrArray;

} // namespace HuboCan

#endif // HUBOJOINT_HPP
