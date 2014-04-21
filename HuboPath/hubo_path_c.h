#ifndef HUBO_PATH_C_H
#define HUBO_PATH_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "HuboCan/AchIncludes.h"

#define HUBO_PATH_CHANNEL "hubo_path"
#define HUBO_PATH_FEEDBACK_CHANNEL "hubo_path_feedback"

//                             123456789012345
#define HUBO_PATH_HEADER_CODE "PATHHEADERv0.01"
#define HUBO_PATH_HEADER_CODE_SIZE 16 // including null-terminator \0

// Maximum number of joints supported in HuboPath messages.
// Note: Theoretically, this may need to be increased if Hubo
// (or another robot using this codebase) has more than 50
// joints, but this seems unlikely.
#define HUBO_PATH_JOINT_MAX_SIZE 50
// TODO: ^^^ Consider using variable-sized data structures for this based
// on the HuboDescription. For right now, this is more involved than I think
// it's worth. Also, this fixed-size format makes it easier to dump into files.

// Maximum number of waypoints transmitted in each message over Ach.
// This value could be adjusted if 100 turns out to not be optimal.
#define HUBO_PATH_CHUNK_SIZE 100

// A stream of const-sized messages are being used for paths, unlike the
// single-shoot variable-sized messages for state and command data. This
// is because those other message types have a predictable max-size for 
// a given HuboDescription of the robot. Paths, on the other hand, can be
// arbitrarily long, so we cannot make any assumptions about a max size.

typedef enum hubo_path_interp {
    
    HUBO_PATH_RAW,      /*! Use the waypoints exactly as they are given                     */
    HUBO_PATH_DENSIFY,  /*! Add points based on input frequency                             */
    HUBO_PATH_SPLINE,   /*! Go from waypoint to waypoint based on a series of splines,
                            coming to a stop at each waypoint                               */
    HUBO_PATH_OPTIMIZE  /*! Minimize the time spent travelling through waypoints without
                            violating the joints' nominal speed and acceleration settings   */
    
} hubo_path_interp_t;
// Note: No matter which option is chosen, paths will be rejected if they violate maximum
// speed or acceleration settings.

typedef enum hubo_path_instruction {
    
    HUBO_PATH_QUIT = 0, /*! Quit the current trajectory. The next command will prompt the
                            trajectory runner to load a new trajectory                              */
    HUBO_PATH_RUN,      /*! Continue running the current trajectory. If the trajectory runner
                            is not currently in a trajectory, it will attempt to load a new one
                            and then immediately start running it                                   */
    HUBO_PATH_PAUSE,    /*! Pause the current trajectory. If the trajectory runner is not
                            currently in a trajectory, it will attempt to load a new one and then
                            pause at the first timestep                                             */
    HUBO_PATH_REVERSE,  /*! Run backwards through the current trajectory. If the trajectory runner
                            is not currently in a trajectory, it will behave the same as pause      */
    HUBO_PATH_LOAD,     /*! Quit the current trajectory and attempt to load a new one. Then run.    */
    
    
} hubo_path_instruction_t;

typedef struct hubo_path_timestep {
    
    double references[HUBO_PATH_JOINT_MAX_SIZE];
    
    // TODO: Add controller-related parameters in here
    
    
}__attribute__((packed)) hubo_path_timestep_t;

typedef struct hubo_path_header {
    
    char code[HUBO_PATH_HEADER_CODE_SIZE];
    double frequency;       /*! Used for HUBO_PATH_DENSIFY */
    
    // TODO: Add any useful meta-data in here, e.g. the model
    // of hubo that the trajectory is meant for
    
}__attribute__((packed)) hubo_path_header_t;

typedef struct hubo_path_chunk {

    hubo_path_header_t header;
    
    hubo_path_timestep_t steps[HUBO_PATH_CHUNK_SIZE];
    
    uint32_t chunk_size;    /*! Number of relevant steps in this chunk      */
    uint32_t chunk_id;      /*! ID of this chunk                            */
    uint32_t total_chunks;  /*! Total number of chunks to be streamed in    */
    
}__attribute__((packed)) hubo_path_chunk_t;

typedef enum {
    
    PATH_REC_IGNORING = 0,  /*! There is no point in sending any path chunks right now  */
    PATH_REC_READ_READY,    /*! Ready to start reading a series of path chunks          */
    PATH_REC_LISTENING,     /*! Currently reading a series of path chunks               */
    PATH_REC_TIMEOUT,       /*! Timed out while waiting for the next chunk  -- quitting!*/
    PATH_REC_DISCONTINUITY, /*! Received a discontinuous chunk              -- quitting!*/
    PATH_REC_ACH_ERROR,     /*! Ran into an ach error                       -- quitting!*/
    PATH_REC_FINISHED,      /*! Finished receiving the incoming path chunks             */
    PATH_REC_CANCELED       /*! The user has canceled the chunk sending     -- quitting!*/
    
} path_reception_state_t;

typedef struct path_reception {
    
    path_reception_state_t state;
    
    uint32_t chunk_id;
    uint32_t expected_size;
    
}__attribute__((packed)) path_reception_t;

#endif // HUBO_PATH_C_H
