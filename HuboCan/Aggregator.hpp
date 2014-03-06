#ifndef AGGREGATOR_HPP
#define AGGREGATOR_HPP


#include <vector>
#include "HuboCan/HuboDescription.hpp"

extern "C" {
#include "HuboCan/AchIncludes.h"
#include "HuboCmd/hubo_cmd_c.h"
}

namespace HuboCan {

typedef std::vector<uint16_t> PidArray;

class Aggregator
{
public:



protected:

    PidArray _pids;

    hubo_cmd_data* _data;

    HuboCan::HuboDescription _desc;

};

} // namespace HuboCan


#endif // AGGREGATOR_HPP
