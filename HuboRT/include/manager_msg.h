#ifndef MANAGERMSGS_H
#define MANAGERMSGS_H

#include <stdint.h>
#include "HuboRtParams.h"

const char hubo_rt_mgr_cmd_chan[] = "hubo_rt_mgr_cmd";
const char hubo_rt_mgr_reply_chan[] = "hubo_rt_mgr_reply";

/*! Commands available for the process management daemon */
typedef enum manager_cmd {
    
    LIST_PROCS=0,           /*!< Dump a list of all registered processes */
    LIST_LOCKED_PROCS,      /*!< Dump a list of all processes which currently have lockfiles */
    LIST_CHANS,             /*!< Dump a list of all registered ach channels */
//    LIST_OPEN_CHANS,        /*!< Dump a list of all currently open ach channels */
    
    RUN_PROC,               /*!< Spawn an instance of the specified process */
    RUN_ALL_PROCS,          /*!< Spawn all processes that have been registered with the manager */
    
    STOP_PROC,              /*!< Close the instance of the specified locked process */
    STOP_ALL_PROCS,         /*!< Close all locked processes */
    
    KILL_PROC,              /*!< Forcibly (ungracefully) close the instance of the specified locked process */
    KILL_ALL_PROCS,         /*!< Forcibly (ungracefully) close all instances of all locked processes */
    
    CREATE_ACH_CHAN,        /*!< Create the specified ach channel */
    CREATE_ALL_ACH_CHANS,   /*!< Create all registered ach channels */
    
    CLOSE_ACH_CHAN,         /*!< Close the specified ach channel */
    CLOSE_ALL_ACH_CHANS,    /*!< Close all registered ach channels */
    
    REGISTER_NEW_PROC,      /*!< Add a new process to the existing roster of processes */
    UNREGISTER_OLD_PROC,    /*!< Remove a process from the roster */
//    RESET_PROC_ROSTER,      /*!< Reset the process roster to its default lineup */
    
    REGISTER_NEW_CHAN,      /*!< Add a new ach channel to the existing roster of channels */
    UNREGISTER_OLD_CHAN,    /*!< Remove an ach channel from the roster */
//    RESET_CHAN_ROSTER,      /*!< Reset the channel roster to its default lineup */
    
    RESET_ROSTERS,          /*!< Reset the process and channel rosters to their default configuration */
    
    START_UP,               /*!< Initiates everything necessary for complete system operation */
    SHUT_DOWN,              /*!< Shuts down everything (except the manager) to bring the system to a "known state" */
    
    LIST_CONFIGS,           /*!< Lists all existing management configurations */
    SAVE_CONFIG,            /*!< Saved the current management configuration to a file */
    LOAD_CONFIG             /*!< Loads the specified management configuration */

} manager_cmd_t;


typedef struct manager_msg {
    
    manager_cmd_t request_type;
    char name[2*MAX_FILENAME_SIZE];
    
} manager_msg_t;

/*! Possible replies from the management daemon */
typedef enum manager_err {
    
    NO_ERROR = 0,           /*!< Your command was successful */
    EMPTY_LIST,             /*!< The list you requested is empty */
    NONEXISTENT_ENTRY,      /*!< The process or channel you specified is not running, not open, or could not be found */
    UNREGISTERED_ENTRY,     /*!< The process or channel you specified is not on the roster */
    MALFORMED_REQUEST,      /*!< A request was sent which does not obey proper formatting. You are encouraged to use the C++ API which is provided. */
    ACH_ERROR,              /*!< Some kind of ach error has occurred */
    NONEXISTENT_DIR,        /*!< Somehow the necessary directory does not exist -- report a bug! */
    
    MGR_TIMEOUT             /*!< The manager failed to reply in the specified amount of time. This likely means that the manager is not running. */
    
} manager_err_t;

typedef struct manager_reply {
    
    manager_err_t err;
    char reply[MAX_FILENAME_SIZE];
    uint8_t replyID;
    uint8_t numReplies;
    manager_cmd_t original_req;
    
} manager_reply_t;

#endif // MANAGERMSGS_H
