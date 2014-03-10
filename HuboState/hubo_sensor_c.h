#ifndef HUBO_SENSOR_C_H
#define HUBO_SENSOR_C_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "HuboCan/hubo_info_c.h"
#include "HuboCmd/hubo_cmd_c.h"
#include "HuboCan/AchIncludes.h"

#define HUBO_JOINT_SENSOR_CHANNEL   "hubo_joint_sensors"
#define HUBO_IMU_SENSOR_CHANNEL     "hubo_imu_sensors"
#define HUBO_FT_SENSOR_CHANNEL      "hubo_ft_sensors"

#define HUBO_SENSOR_HEADER_CODE "SENSORHDR_V0.01"
#define HUBO_SENSOR_HEADER_CODE_SIZE 16 /* including null-terminator \0 */

typedef uint8_t hubo_sensor_data;

typedef struct hubo_sensor_header {

    char code[HUBO_SENSOR_HEADER_CODE_SIZE];
    char type[HUBO_COMPONENT_TYPE_MAX_LENGTH];
    uint8_t array_count;
    double timer;

}__attribute__((packed)) hubo_sensor_header_t;


typedef struct hubo_joint_status {

    uint8_t driver_on;
    uint8_t control_on;
    uint8_t control_mode;
    uint8_t limit_switch;

    uint8_t home_flag;

    uint8_t jam_error;
    uint8_t pwm_saturated;
    uint8_t big_error;
    uint8_t encoder_error;
    uint8_t driver_fault;
    uint8_t motor_fail_0;
    uint8_t motor_fail_1;

    uint8_t min_position_error;
    uint8_t max_position_error;
    uint8_t velocity_error;
    uint8_t acceleration_error;
    uint8_t temperature_error;

}__attribute__((packed)) hubo_joint_status_t;

typedef struct hubo_joint_state {

    double position;
    double current;
    double duty;
    double temperature;

    double reference;
    hubo_cmd_mode_t mode;

    hubo_joint_status_t status;

}__attribute__((packed)) hubo_joint_state_t;

typedef struct hubo_imu_state {

    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];

    double angular_position[3];
    double angular_velocity[3];

}__attribute__((packed)) hubo_imu_state_t;

typedef struct hubo_ft_state {

    char name[HUBO_COMPONENT_NAME_MAX_LENGTH];

    double force[3];
    double torque[3];

}__attribute__((packed)) hubo_ft_state_t;


hubo_data_error_t hubo_sensor_header_check(const hubo_sensor_data* sensor_message);

#endif // HUBO_SENSOR_C_H
