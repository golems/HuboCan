#ifndef HUBO_CMD_C_H
#define HUBO_CMD_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

const char hubo_cmd_chan[] = "hubo_cmd";

typedef uint8_t hubo_cmd_data;

typedef struct hubo_cmd_cx {

    hubo_cmd_data* data;
    size_t data_size;

} hubo_cmd_cx_t;


/*                            123456789012345                        */
#define hubo_cmd_header_code "CMDHEADER_V0.01"
#define hubo_cmd_header_code_size 16 /* including null-terminator \0 */

typedef struct hubo_cmd_header {

    char code[hubo_cmd_header_code_size];
    uint8_t pid;
    uint8_t is_compressed;
    uint8_t total_num_joints;
    uint8_t bitmap;

} hubo_cmd_header_t;

typedef enum hubo_cmd_mode {

    HUBO_CMD_OFF = 0,
    HUBO_CMD_POSITION,
    HUBO_CMD_TORQUE,
    HUBO_CMD_HYBRID

} hubo_cmd_mode_t;

typedef struct hubo_joint_cmd {

    hubo_cmd_mode_t mode;
    float position;
    float torque;

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
    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return header->is_compressed;
}

inline uint8_t hubo_cmd_data_get_total_num_joints(const hubo_cmd_data* data)
{
    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return header->total_num_joints;
}

int hubo_cmd_header_check(const hubo_cmd_data* cmd_message);

void hubo_cmd_init_cx(hubo_cmd_cx_t* context, size_t num_total_joints);

hubo_cmd_data* hubo_cmd_init_data(size_t num_total_joints);

size_t hubo_cmd_cx_compressor(hubo_cmd_cx_t* compressed, const hubo_cmd_cx_t* uncompressed);

size_t hubo_cmd_data_compressor(hubo_cmd_data* compressed, const hubo_cmd_data* uncompressed);

void hubo_cmd_cx_set_joint_cmd(hubo_cmd_cx_t* context, const hubo_joint_cmd_t* cmd, size_t joint_num);

void hubo_cmd_cx_get_joint_cmd(hubo_joint_cmd_t* output, const hubo_cmd_cx_t* context, size_t joint_num);

size_t hubo_cmd_data_get_size(const hubo_cmd_data* data);

int hubo_cmd_data_check_if_joint_is_set(const hubo_cmd_data* data, size_t joint_index);

#endif // HUBO_CMD_C_H
