#ifndef INFOGENERATOR_HPP
#define INFOGENERATOR_HPP

#include "InfoTypes.hpp"

extern "C" {
#include "hubo_info_c.h"
} // extern "C"


namespace HuboCan {

class InfoGenerator
{
public:

    InfoGenerator(size_t joint_count = 0);
    ~InfoGenerator();

    int sendInfo();

    void resize(size_t joint_count);


protected:


};

} // namespace HuboState

#endif // INFOGENERATOR_HPP
