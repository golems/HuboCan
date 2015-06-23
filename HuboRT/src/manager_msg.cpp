
#include "../manager_msg.hpp"

#define return_enum_string( X ) case X : return #X ;

std::string manager_cmd_to_string(manager_cmd_t cmd)
{
    switch(cmd)
    {
        return_enum_string(UNKNOWN_REQUEST);
        return_enum_string(LIST_PROCS);
        return_enum_string(LIST_LOCKED_PROCS);
        return_enum_string(LIST_CHANS);

        return_enum_string(RUN_PROC);
        return_enum_string(RUN_ALL_PROCS);

        return_enum_string(STOP_PROC);
        return_enum_string(STOP_ALL_PROCS);

        return_enum_string(KILL_PROC);
        return_enum_string(KILL_ALL_PROCS);

        return_enum_string(CREATE_ACH_CHAN);
        return_enum_string(CREATE_ALL_ACH_CHANS);

        return_enum_string(CLOSE_ACH_CHAN);
        return_enum_string(CLOSE_ALL_ACH_CHANS);

        return_enum_string(REGISTER_NEW_PROC);
        return_enum_string(UNREGISTER_OLD_PROC);

        return_enum_string(REGISTER_NEW_CHAN);
        return_enum_string(UNREGISTER_OLD_CHAN);

        return_enum_string(RESET_ROSTERS);

        return_enum_string(START_UP);
        return_enum_string(SHUT_DOWN);

        return_enum_string(LIST_CONFIGS);
        return_enum_string(SAVE_CONFIG);
        return_enum_string(LOAD_CONFIG);
        return_enum_string(DELETE_CONFIG);

        default: return "UNKNOWNK_MGR_CMD";
    }
}

std::string manager_err_to_string(manager_err_t error)
{
    switch(error)
    {
        return_enum_string(NO_ERROR);
        return_enum_string(EMPTY_LIST);
        return_enum_string(NONEXISTENT_ENTRY);
        return_enum_string(UNREGISTERED_ENTRY);
        return_enum_string(MALFORMED_REQUEST);
        return_enum_string(ACH_ERROR);
        return_enum_string(NONEXISTENT_DIR);
        return_enum_string(MGR_RACE_CONDITION);
            
        return_enum_string(MGR_TIMEOUT);
            
        default:                    return "UNKNOWN_MGR_ERROR";
    }
    
    return "IMPOSSIBLE_ERROR";
}

std::string achd_network_to_string(achd_network_t type)
{
    switch(type)
    {
        case ACHD_NOTHING:          return ACHD_INTERNAL_STRING;
        case ACHD_PULL_FROM_ROBOT:  return ACHD_PULL_STRING;
        case ACHD_PUSH_TO_ROBOT:    return ACHD_PUSH_STRING;
        default:                    return "UNKNOWN_ACHD_TYPE";
    }
    
    return "IMPOSSIBLE";
}
