#ifndef HUBO_INFO_C_H
#define HUBO_INFO_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define HUBO_INFO_META_CHANNEL "hubo_info_meta"
#define HUBO_INFO_DATA_CHANNEL "hubo_info_data"

/*                           123456789012345 */
#define HUBO_INFO_META_CODE "META_INFO_V0.01"
#define HUBO_INFO_META_CODE_SIZE 16

#define HUBO_COMPONENT_NAME_MAX_LENGTH 32
#define HUBO_COMPONENT_TYPE_MAX_LENGTH 32

typedef uint8_t hubo_info_data;

typedef struct hubo_meta_info {

    char code[HUBO_INFO_META_CODE_SIZE];
    uint8_t joint_count;
    uint8_t jmc_count;
    uint8_t sensor_count;
    size_t data_size;

}__attribute__((packed)) hubo_meta_info_t;

typedef struct hubo_joint_info {

    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];
    uint16_t software_index;

    float drive_factor;
    float driven_factor;
    float harmonic_factor;
    float enc_resolution;

    float default_kp;
    float default_kd;
    float default_max_pwm;

    uint16_t hardware_index;
    char jmc_name[HUBO_COMPONENT_NAME_MAX_LENGTH];

    float min_position;
    float max_position;

    float nominal_speed;
    float max_speed;

    float nominal_accel;
    float max_accel;

}__attribute__((packed)) hubo_joint_info_t;

typedef struct hubo_jmc_info {

    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];
    char type[HUBO_COMPONENT_TYPE_MAX_LENGTH];

    uint16_t hardware_index;
    uint16_t can_channel;

}__attribute__((packed)) hubo_jmc_info_t;

typedef struct hubo_sensor_info {

    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];
    char type[HUBO_COMPONENT_TYPE_MAX_LENGTH];

    uint16_t hardware_index;
    uint8_t can_channel;

}__attribute__((packed)) hubo_sensor_info_t;

size_t hubo_info_get_joint_count(const hubo_info_data* data);

size_t hubo_info_get_jmc_count(const hubo_info_data* data);

size_t hubo_info_get_sensor_count(const hubo_info_data* data);

hubo_info_data* hubo_info_init_data(size_t joint_count, size_t jmc_count, size_t sensor_count);

hubo_info_data* hubo_info_receive_data(double timeout); ///< Will return NULL pointer if there is an error

int hubo_info_send_data(const hubo_info_data* data);

hubo_joint_info_t* hubo_info_get_joint_info(hubo_info_data* data, size_t joint_index);

hubo_jmc_info_t* hubo_info_get_jmc_info(const hubo_info_data *data, size_t jmc_index);

hubo_sensor_info_t* hubo_info_get_sensor_info(const hubo_info_data *data, size_t sensor_index);

size_t hubo_info_get_joint_location(const hubo_info_data* data, size_t joint_index);

size_t hubo_info_get_jmc_location(const hubo_info_data* data, size_t jmc_index);

size_t hubo_info_get_sensor_location(const hubo_info_data* data, size_t sensor_index);

size_t hubo_info_predict_data_size(size_t joint_count, size_t jmc_count, size_t sensor_count);

inline size_t hubo_info_get_data_size(const hubo_info_data* data);

#endif // HUBO_INFO_C_H
