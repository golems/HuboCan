#ifndef HUBO_AUX_CMD_C_H
#define HUBO_AUX_CMD_C_H

#include "hubo_cmd_c.h"

#define HUBO_AUX_CMD_CHANNEL "hubo_aux_cmd"

/*                                123456789012345                       */
#define HUBO_AUX_CMD_HEADER_CODE "AUXHEADER_V0.01"
#define HUBO_AUX_CMD_HEADER_CODE_SIZE 16 // including null-terminator \0

typedef enum {

    // TODO: Put in all the different kinds of board commands
    HOME_JOINT,
    HOME_ALL_JOINTS,

    INIT_ALL_SENSORS,
    INIT_ALL_IMUS,
    INIT_ALL_FTS,
    INIT_SENSOR

} hubo_aux_cmd_id_t;

typedef enum {

    DISABLE,
    ENABLE,
    UPDATE,
    IGNORE,

    CLOCKWISE,
    COUNTERCLOCKWISE,

    // Homing type parameters
    SWITCH_AND_INDEX,
    SWITCH,
    JAM_LIMIT

} hubo_aux_cmd_param_t;

typedef struct hubo_aux_cmd {

    char code[HUBO_AUX_CMD_HEADER_CODE_SIZE];
    hubo_aux_cmd_id_t cmd_id;
    uint32_t device_id;
    uint32_t component_id;
    hubo_aux_cmd_param_t params[8];
    double values[8];

}__attribute__((packed)) hubo_aux_cmd_t;

hubo_data_error_t hubo_aux_cmd_header_check(const hubo_aux_cmd_t* cmd);

#endif // HUBO_AUX_CMD_C_H
