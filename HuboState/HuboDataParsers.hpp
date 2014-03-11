#ifndef HUBODATAPARSERS_HPP
#define HUBODATAPARSERS_HPP

extern "C" {
#include "hubo_sensor_c.h"
}

#include <vector>

namespace HuboState {

typedef std::vector<hubo_joint_state_t> JointStateArray;
typedef std::vector<hubo_imu_state_t> ImuStateArray;
typedef std::vector<hubo_ft_state_t> ForceTorqueStateArray;

size_t get_sensor_component_count(const hubo_sensor_data* data)
{
    const hubo_sensor_header_t* header = (hubo_sensor_header_t*)data;
    return header->array_size;
}

template<class DataClass>
size_t get_sensor_data_size(size_t array_size)
{
    return sizeof(hubo_sensor_header_t) + sizeof(DataClass)*array_size;
}

template<class DataClass>
hubo_sensor_data* initialize_sensor_data(size_t array_size)
{
    hubo_sensor_data* new_data = (hubo_sensor_data*)malloc(get_sensor_data_size<DataClass>(array_size));

    hubo_sensor_header_t header;
    strcpy(header.code, HUBO_SENSOR_HEADER_CODE);
    header.array_size = array_size;
    header.time = 0;

    memcpy(new_data, &header, sizeof(hubo_sensor_header_t));

    return new_data;
}

hubo_data_error_t set_data_timestamp(hubo_sensor_data* data, double time)
{
    if(hubo_sensor_header_check(data) == 1)
    {
        hubo_sensor_header_t* header = (hubo_sensor_header_t*)data;
        header->time = time;

        return HUBO_DATA_OKAY;
    }
    else
    {
        return HUBO_DATA_MALFORMED_HEADER;
    }

    return HUBO_DATA_IMPOSSIBLE;
}

template<class DataClass>
const DataClass* get_sensor_data_component(const hubo_sensor_data* data, size_t index)
{
    if(hubo_sensor_header_check(data) == HUBO_DATA_OKAY)
    {
        const hubo_sensor_header_t* header = (hubo_sensor_header_t*)data;
        if(index < header->array_size)
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
hubo_data_error_t set_sensor_data_component(hubo_sensor_data* data,
                               const DataClass& component,
                               size_t index)
{
    if(hubo_sensor_header_check(data) == HUBO_DATA_OKAY)
    {
        const hubo_sensor_header_t* header = (hubo_sensor_header_t*)data;
        if(index < header->array_size)
        {
            DataClass* data_location = get_sensor_data_component<DataClass>(data,index);
            if(data_location != NULL)
            {
                memcpy(data_location, &component, sizeof(DataClass));
                return HUBO_DATA_OKAY;
            }
            else
            {
                return HUBO_DATA_IMPOSSIBLE;
            }
        }
        else
        {
            return HUBO_DATA_OUT_OF_BOUNDS;
        }
    }
    else
    {
        return HUBO_DATA_MALFORMED_HEADER;
    }

    return HUBO_DATA_IMPOSSIBLE;
}



template<class DataClass>
std::vector<DataClass> get_vector_from_data(hubo_sensor_data* data)
{
    std::vector<DataClass> data_vector;
    size_t components = get_sensor_component_count(data);
    for(size_t i=0; i<components; ++i)
    {
        data_vector.push_back(*get_sensor_data_component<DataClass>(data, i));
    }

    return data_vector;
}

} // namespace HuboState


#endif // HUBODATAPARSERS_HPP
