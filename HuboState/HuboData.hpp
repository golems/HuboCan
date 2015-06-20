#ifndef HUBODATA_HPP
#define HUBODATA_HPP

extern "C" {
#include "hubo_sensor_c.h"
}

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <stdlib.h>
#include "hubo_sensor_stream.hpp"
#include "HuboCan/InfoTypes.hpp"

namespace HuboState {

typedef std::vector<hubo_joint_state_t> JointStateArray;
typedef std::vector<hubo_imu_state_t> ImuStateArray;
typedef std::vector<hubo_ft_state_t> ForceTorqueStateArray;

inline size_t get_data_component_count(const hubo_data* data)
{
    if(NULL == data)
        return 0;
    
    if(hubo_data_header_check(data) != HUBO_DATA_OKAY)
        return 0;
    
    const hubo_data_header_t* header = (hubo_data_header_t*)data;
    return header->array_size;
}

template<class DataClass>
size_t predict_data_size(size_t array_size)
{
    return sizeof(hubo_data_header_t) + sizeof(DataClass)*array_size;
}

template<class DataClass>
size_t get_data_size(const hubo_data* data)
{
    if(NULL == data)
        return 0;
    
    if(hubo_data_header_check(data) != HUBO_DATA_OKAY)
        return 0;
    
    return predict_data_size<DataClass>(get_data_component_count(data));
}

template<class DataClass>
hubo_data* initialize_data(size_t array_size)
{
    hubo_data* new_data = (hubo_data*)malloc(predict_data_size<DataClass>(array_size));
    memset(new_data, 0, predict_data_size<DataClass>(array_size));

    hubo_data_header_t header;
    strcpy(header.code, HUBO_DATA_HEADER_CODE);
    header.array_size = array_size;
    header.time = 0;

    memcpy(new_data, &header, sizeof(hubo_data_header_t));

    return new_data;
}

inline hubo_data_error_t set_data_timestamp(hubo_data* data, double time)
{
    if(NULL == data)
        return HUBO_DATA_NULL;
    
    if(hubo_data_header_check(data) == HUBO_DATA_OKAY)
    {
        hubo_data_header_t* header = (hubo_data_header_t*)data;
        header->time = time;

        return HUBO_DATA_OKAY;
    }
    else
    {
        return HUBO_DATA_MALFORMED_HEADER;
    }

    return HUBO_DATA_IMPOSSIBLE;
}

inline double get_data_timestamp(hubo_data* data)
{
    if(NULL == data)
        return HUBO_DATA_NULL;

    if(hubo_data_header_check(data) == HUBO_DATA_OKAY)
    {
        hubo_data_header_t* header = (hubo_data_header_t*)data;
        return header->time;
    }
    else
    {
        std::cout << "Requesting timestamp of hubo_data with malformed header!" << std::endl;
        return -1;
    }

    return HUBO_DATA_IMPOSSIBLE;
}

template<class DataClass>
DataClass* get_data_component(hubo_data* data, size_t index)
{
    if(NULL == data)
        return NULL;
    
    if(hubo_data_header_check(data) == HUBO_DATA_OKAY)
    {
        const hubo_data_header_t* header = (hubo_data_header_t*)data;
        if(index < header->array_size)
        {
            return (DataClass*)(data
                    + sizeof(hubo_data_header_t)
                    + sizeof(DataClass)*index);
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
hubo_data_error_t set_data_component(hubo_data* data,
                               const DataClass& component,
                               size_t index)
{
    if(hubo_data_header_check(data) == HUBO_DATA_OKAY)
    {
        const hubo_data_header_t* header = (hubo_data_header_t*)data;
        if(index < header->array_size)
        {
            DataClass* data_location = get_data_component<DataClass>(data,index);
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
std::vector<DataClass> get_vector_from_data(hubo_data* data)
{
    std::vector<DataClass> data_vector;
    size_t components = get_data_component_count(data);
    data_vector.reserve(components);
    for(size_t i=0; i<components; ++i)
    {
        data_vector.push_back(*get_data_component<DataClass>(data, i));
    }

    return data_vector;
}

typedef std::map<std::string, size_t> StringMap;

template<class DataClass>
class HuboData
{
public:

    HuboData()
    {
        _construction();
    }

    HuboData(const HuboData& copy)
    {
        _construction();
        
        if(copy.is_initialized())
        {
            _deep_copy_data(copy);
        }
    }

    HuboData& operator=(const HuboData& copy)
    {
        if(copy.is_initialized())
        {
            _deep_copy_data(copy);
        }
    }

    DataClass& operator[](size_t index)
    {
        if(!_check_initialized("[] array member access"))
        {
            return _dummy_member;
        }

        DataClass* result = get_data_component<DataClass>(_raw_data, index);
        if(NULL == result)
        {
            if(index == (size_t)(-1))
            {
                std::cout << "You have requested an InvalidIndex for channel '"
                          << _channel_name << "'.\n"
                          << "(Data Count: " << get_data_component_count(_raw_data)
                          << ")" << std::endl;
            }
            else
            {
                std::cout << "You have requested an out of bounds data member for channel '"
                         << _channel_name << "'.\n Requested: " << index
                         << ", Data Count: " << get_data_component_count(_raw_data) << std::endl;
            }
        }

        return *result;
    }

    const DataClass& const_entry(size_t index) const
    {
        if(!_check_initialized("const_entry member access"))
        {
            return _dummy_member;
        }

        DataClass* result = get_data_component<DataClass>(_raw_data, index);
        if(NULL == result)
        {
            if(index == (size_t)(-1))
            {
                std::cout << "You have requested an InvalidIndex for channel '"
                          << _channel_name << "'.\n"
                          << "(Data Count: " << get_data_component_count(_raw_data)
                          << ")" << std::endl;
            }
            else
            {
                std::cout << "You have requested an out of bounds data member for channel '"
                         << _channel_name << "'.\n Requested: " << index
                         << ", Data Count: " << get_data_component_count(_raw_data) << std::endl;
            }
            return _dummy_member;
        }

        return *result;
    }

    DataClass& operator[](const std::string& name)
    {
        StringMap::iterator it = _mapping.find(name);
        if(it == _mapping.end())
        {
            std::cout << "You have requested a data member which does not exist for channel '"
                         << "', Requested: '" << name << "'" << std::endl;
            return _dummy_member;
        }

        return (*this)[it->second];
    }

    bool initialize(const std::vector<std::string>& names, const std::string& channel_name)
    {
        free(_raw_data);
        _raw_data = initialize_data<DataClass>(names.size());

        _mapping.clear();
        _names.clear();
        for(size_t i=0; i<names.size(); ++i)
        {
            StringMap::iterator it = _mapping.find(names[i]);
            if(it == _mapping.end())
            {
                _mapping[names[i]] = i;
            }
            else
            {
                std::cout << "You have a repeated data member name ('" << names[i] << "')\n"
                            << " -- Already exists as entry #" << it->second << "\n"
                            << " -- Attempted to assign it to entry #" << i << std::endl;
                _mapping.clear();
                return false;
            }
        }
        _names = names;

        _channel_name = channel_name;
        ach_status_t r = ach_open(&_channel, channel_name.c_str(), NULL);
        if( ACH_OK == r )
        {
            _initialized = true;
            return true;
        }

        _initialized = false;

        std::cout << "Failed to open channel '" << _channel_name << "': "
                     << ach_result_to_string(r) << std::endl;

        return false;
    }

    HuboCan::error_result_t receive_data(double timeout_seconds = 0)
    {
        if(!_check_initialized("receive_data"))
            return HuboCan::ACH_ERROR;

        size_t fs;
        struct timespec wait_time;
        clock_gettime( ACH_DEFAULT_CLOCK, &wait_time );
        int nano_wait = wait_time.tv_nsec + (int)(timeout_seconds*1E9);
        wait_time.tv_sec += (int)(nano_wait/1E9);
        wait_time.tv_nsec = (int)(nano_wait%((int)1E9));
        ach_status_t r = ach_get(&_channel, _raw_data,
                                 get_data_size<DataClass>(_raw_data),
                                 &fs, &wait_time, ACH_O_LAST | ACH_O_WAIT);

        if( ACH_TIMEOUT == r )
        {
            if(verbose)
            {
                std::cout << "Ach channel '" << _channel_name << "' timed out!" << std::endl;
            }
            return HuboCan::TIMEOUT;
        }

        if(fs != get_data_size<DataClass>(_raw_data))
        {
            std::cout << "Framesize mismatch: " << fs << " received, "
                      << get_data_size<DataClass>(_raw_data) << " expected!" << std::endl;
        }

        if( ACH_OK == r || ACH_STALE_FRAMES == r || ACH_MISSED_FRAME == r )
        {
            if(verbose)
            {
                std::cout << "Ach result for channel '" << _channel_name << "': "
                             << ach_result_to_string(r) << std::endl;
            }
            return HuboCan::OKAY;
        }
        else
        {
            std::cout << "Unexpected ach_get result for channel '" << _channel_name <<"': "
                      << ach_result_to_string(r) << std::endl;
            return HuboCan::ACH_ERROR;
        }
        
        return HuboCan::UNDEFINED_ERROR;
    }

    bool send_data(double timestamp)
    {
        if(!_check_initialized("send_data"))
            return false;
        
        set_data_timestamp(_raw_data, timestamp);
        
        ach_status_t r = ach_put(&_channel, _raw_data, get_data_size<DataClass>(_raw_data));
        
        if(ACH_OK == r)
            return true;
        
        std::cout << "Unexpected ach_put result for channel '" << _channel_name << "': "
                     << ach_result_to_string(r) << std::endl;
        return false;
    }

    std::vector<DataClass> get_data(bool refresh = false)
    {
        if(!_check_initialized("get_data"))
        {
            std::vector<DataClass> empty;
            return empty;
        }
        
        if(refresh)
            receive_data(0);
        
        return get_vector_from_data<DataClass>(_raw_data);
    }

    const std::string& get_entry_name(size_t i) const
    {
        if( i < size() )
            return _names[i];
        else
            return _dummy_string;
    }

    double get_time() const
    {
        return get_data_timestamp(_raw_data);
    }

    bool set_data(const std::vector<DataClass>& copy)
    {
        if(!_check_initialized("set_data"))
            return false;

        if(copy.size() != get_data_component_count(_raw_data))
        {
            std::cout << "set_data mismatch for channel '" << _channel_name
                      << "'! Input size: " << copy.size() << ", Actual data size: "
                      << get_data_component_count(_raw_data) << std::endl;
            return false;
        }

        for(size_t i=0; i<copy.size(); ++i)
        {
            (*this)[i] = copy[i];
        }

        return true;
    }

    ~HuboData()
    {
        free(_raw_data);
    }

    std::string get_channel_name() const { return _channel_name; }
    bool is_initialized() const { return _initialized; }
    
    size_t size() const
    {
        return get_data_component_count(_raw_data);
    }
    
    bool verbose;
    hubo_data* _raw_data;

protected:
    
    void _construction()
    {
        _raw_data = NULL;
        memset(&_channel, 0, sizeof(ach_channel_t));
        _initialized = false;
        verbose = false;
        memset(&_dummy_member, 0, sizeof(DataClass));
        _dummy_string = "dummy";
    }

    void _deep_copy_data(const HuboData& copy)
    {
        initialize(get_data_size<DataClass>(copy._raw_data),
                   copy._channel_name);
        memcpy(_raw_data, copy._raw_data, get_data_size<DataClass>(copy._raw_data));
    }

    bool _check_initialized(std::string operation = "an operation") const
    {
        if(_initialized)
            return true;
        else
        {
            std::cout << "Illegal: Attempting to perform '" << operation << "' on a HuboData object "
                         << "before it has been initialized!" << std::endl;
            return false;
        }
    }

    bool _initialized;
    StringMap _mapping;
    std::vector<std::string> _names;
    std::string _channel_name;
    ach_channel_t _channel;
    DataClass _dummy_member;
    std::string _dummy_string;
    
};


} // namespace HuboState

template<class DataClass>
std::ostream& operator<<(std::ostream& stream, const HuboState::HuboData<DataClass>& data)
{
    if(!data.is_initialized())
    {
        stream << "Uninitialized HuboData" << std::endl;
        return stream;
    }

    stream << "Channel: '" << data.get_channel_name() << "', timestamp: " << data.get_time()
              << ", entry count: " << data.size() << "\n";

    size_t s = 0;
    for(size_t i=0; i<data.size(); ++i)
    {
        const std::string& entry_name = data.get_entry_name(i);
        if(entry_name.size() > s)
            s = entry_name.size();
    }

    for(size_t i=0; i<data.size(); ++i)
    {
        stream.width(s);
        stream << data.get_entry_name(i) << " | " << data.const_entry(i) << "\n";
    }

    return stream;
}

#endif // HUBODATA_HPP
