
#include "../manager_msg.hpp"

std::string manager_err_to_string(manager_err_t error)
{
    switch(error)
    {
        case NO_ERROR:              return "NO_ERROR";
        case EMPTY_LIST:            return "EMPTY_LIST";
        case NONEXISTENT_ENTRY:     return "NONEXISTENT_ENTRY";
        case UNREGISTERED_ENTRY:    return "UNREGISTERED_ENTRY";
        case MALFORMED_REQUEST:     return "MALFORMED_REQUEST";
        case ACH_ERROR:             return "ACH_ERROR";
        case NONEXISTENT_DIR:       return "NONEXISTENT_DIR";
        case MGR_RACE_CONDITION:    return "MGR_RACE_CONDITION";
            
        case MGR_TIMEOUT:           return "MGR_TIMEOUT";
            
        default:                    return "UNKNOWN_MGR_ERROR";
    }
    
    return "IMPOSSIBLE_ERROR";
}
