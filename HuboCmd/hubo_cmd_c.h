#ifndef HUBO_CMD_C_H
#define HUBO_CMD_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define HUBO_CMD_CHANNEL "hubo_cmd"

/*                            123456789012345                        */
#define HUBO_CMD_HEADER_CODE "CMDHEADER_V0.01"
#define HUBO_CMD_HEADER_CODE_SIZE 16 /* including null-terminator \0 */

typedef uint8_t hubo_cmd_data;

typedef struct hubo_cmd_header {

    char code[HUBO_CMD_HEADER_CODE_SIZE];
    uint16_t pid;
    uint8_t is_compressed;
    uint8_t total_num_joints;
    uint8_t bitmap;

}__attribute__((packed)) hubo_cmd_header_t;

typedef enum hubo_cmd_mode {

    HUBO_CMD_IGNORE = 0,
    HUBO_CMD_RIGID,
    HUBO_CMD_COMPLIANT,
    HUBO_CMD_HYBRID,

    HUBO_CMD_HOME,
    HUBO_CMD_CLAIM,
    HUBO_CMD_RELEASE

} hubo_cmd_mode_t;

typedef struct hubo_joint_cmd {

    hubo_cmd_mode_t mode;
    float position; // No sense in having refs of greater precision than the CAN frame can use
    double base_torque; // Everything else will be involved in further calculations, so more precision might be good
    double kP_gain;
    double kD_gain;

}__attribute__((packed)) hubo_joint_cmd_t;

inline size_t hubo_cmd_data_max_message_size(size_t num_joints)
{
    return sizeof(hubo_cmd_header_t)+num_joints*sizeof(hubo_joint_cmd_t);
}

inline size_t hubo_cmd_data_location(size_t joint_index)
{
    return hubo_cmd_data_max_message_size(joint_index);
}

inline int hubo_cmd_data_is_compressed(const hubo_cmd_data* data)
{
    if( NULL == data )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return header->is_compressed;
}

inline uint8_t hubo_cmd_data_get_total_num_joints(const hubo_cmd_data* data)
{
    if( NULL == data )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return header->total_num_joints;
}

int hubo_cmd_header_check(const hubo_cmd_data* cmd_message);

hubo_cmd_data* hubo_cmd_init_data(size_t num_total_joints);

size_t hubo_cmd_data_compressor(hubo_cmd_data* compressed, const hubo_cmd_data* uncompressed);

void hubo_cmd_data_set_joint_cmd(hubo_cmd_data* data, const hubo_joint_cmd_t* cmd, size_t joint_num);

int hubo_cmd_data_get_joint_cmd(hubo_joint_cmd_t* output, const hubo_cmd_data* data, size_t joint_num);

size_t hubo_cmd_data_get_size(const hubo_cmd_data* data);

int hubo_cmd_data_check_if_joint_is_set(const hubo_cmd_data* data, size_t joint_index);

#endif // HUBO_CMD_C_H
