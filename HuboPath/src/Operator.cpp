
#include "../Operator.hpp"

using namespace HuboPath;

Operator::Operator(double timeout)
{
    _initialize_operator();
    receive_description(timeout);
}

Operator::Operator(const HuboCan::HuboDescription &description)
{
    _initialize_operator();
    load_description(description);
}

void Operator::_initialize_operator()
{
    _mapping_set = false;
}

HuboCan::error_result_t Operator::setJointIndices(const StringArray &joint_names)
{
    if(!_initialized)
    {
        std::cout << "This operator does not have a valid HuboDescription loaded yet!\n"
                  << " -- We cannot set joint indices until a description is loaded"
                  << std::endl;
        return HuboCan::UNINITIALIZED;
    }
    
    _index_map = _desc.getJointIndices(joint_names);
    
    IndexArray invalids;
    for(size_t i=0; i < _index_map.size(); ++i)
    {
        if( _index_map[i] == InvalidIndex )
        {
            invalids.push_back(i);
        }
    }
    
    if(invalids.size() > 0)
    {
        std::cout << "The following joint names were invalid: ";
        for(size_t i=0; i<invalids.size(); ++i)
        {
            std::cout << joint_names[invalids[i]] << " (" << i << ")";
            if(i+1 < invalids.size())
                std::cout << ", ";
        }
        std::cout << std::endl;
        _index_map.clear();
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _mapping_set = true;
    
    return HuboCan::OKAY;
}

HuboCan::error_result_t Operator::setJointIndices(const IndexArray &joint_indices)
{
    if(!_initialized)
    {
        std::cout << "This operator does not have a valid HuboDescription loaded yet!\n"
                  << " -- We cannot set joint indices until a description is loaded"
                  << std::endl;
        return HuboCan::UNINITIALIZED;
    }
    
    IndexArray invalids;
    for(size_t i=0; i<joint_indices.size(); ++i)
    {
        if(joint_indices[i] > _desc.joints.size())
        {
            invalids.push_back(i);
        }
    }
    
    if(invalids.size() > 0)
    {
        std::cout << "The following joint indices exceeded the maximum ("
                  << _desc.joints.size() << "): ";
        for(size_t i=0; i<invalids.size(); ++i)
        {
            std::cout << joint_indices[invalids[i]] << " (" << i << ")";
            if(i+1 < invalids.size())
                std::cout << ", ";
        }
        std::cout << std::endl;
        _index_map.clear();
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _index_map = joint_indices;
    _mapping_set = true;
    
    return HuboCan::OKAY;    
}

bool Operator::_check_mapping_set(std::string calling_function)
{
    if(!_mapping_set)
    {
        std::cout << "Error: Cannot use Operator function (" << calling_function
                  << ") until the joint index mapping has been set.\n"
                  << " -- Use setJointIndices() before using "
                  << calling_function << std::endl;
        return false;
    }
    
    return true;
}

HuboCan::error_result_t Operator::addWaypoint(const Eigen::VectorXd &waypoint)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }
    
    if((size_t)waypoint.size() != _index_map.size())
    {
        std::cout << "Invalid number of components in this waypoint ("
                  << waypoint.size() << "). Must be equal to " << _index_map.size()
                  << std::endl;
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    }
    
    _input_path.push_back(waypoint);
    
    return HuboCan::OKAY;
}

HuboCan::error_result_t Operator::addWaypoints(const std::list<Eigen::VectorXd> &waypoints)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }
    
    HuboCan::error_result_t result = HuboCan::OKAY;
    std::list<Eigen::VectorXd>::const_iterator it;
    size_t count = 0;
    for(it = waypoints.begin(); it != waypoints.end(); ++it)
    {
        result |= addWaypoint(*(it));
        
        if(result != HuboCan::OKAY)
        {
            std::cout << "Found item #" << count << " in the std::list to contain an error: "
                      << result << ". We will stop adding waypoints!" << std::endl;
            return result;
        }
        ++count;
    }
    
    return result;
}

HuboCan::error_result_t Operator::addWaypoints(const std::vector<Eigen::VectorXd> &waypoints)
{
    if(!_check_mapping_set("addWaypoint"))
    {
        return HuboCan::UNINITIALIZED;
    }
    
    HuboCan::error_result_t result = HuboCan::OKAY;
    
    for(size_t i=0; i<waypoints.size(); ++i)
    {
        result |= addWaypoint(waypoints[i]);
        
        if(result != HuboCan::OKAY)
        {
            std::cout << "Found item #" << i << " in the std::vector to contain an error: "
                      << result << ". We will stop adding waypoints!" << std::endl;
            return result;
        }
    }
    
    return result;
}

HuboCan::error_result_t Operator::removeLast()
{
    if(_index_map.size() == 0)
        return HuboCan::INDEX_OUT_OF_BOUNDS;
    
    _index_map.pop_back();
    
    return HuboCan::OKAY;
}

void Operator::clearWaypoints()
{
    _index_map.clear();
}



