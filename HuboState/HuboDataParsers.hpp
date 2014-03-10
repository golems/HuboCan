#ifndef HUBODATAPARSERS_HPP
#define HUBODATAPARSERS_HPP

extern "C" {
#include "hubo_sensor_c.h"
}

namespace HuboState {

template<class DataClass>
const DataClass* get_sensor_data_component(const hubo_sensor_data* data, size_t index)
{
    if(hubo_sensor_header_check(data) == HUBO_DATA_OKAY)
    {
        const hubo_sensor_header_t* header = (hubo_sensor_header_t*)data;
        if(index < header->array_count)
        {
            return (DataClass*)data
                    + sizeof(hubo_sensor_header_t)
                    + sizeof(DataClass)*index;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        return NULL;
    }

    return NULL;
}

template<class DataClass>
size_t get_sensor_data_size(size_t component_count)
{
    return sizeof(hubo_sensor_header_t) + sizeof(DataClass)*component_count;
}

} // namespace HuboState


#endif // HUBODATAPARSERS_HPP
