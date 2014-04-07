#ifndef HUBOCANID_H
#define HUBOCANID_H

namespace HuboCan {

typedef enum {
    
    CMD_BYTE = 0x01,
    
} can_base_id_t;

typedef enum {
    
    GET_BOARD_INFO      = 0x01,
    GET_STATUS          = 0x02,
    GET_ENCODER         = 0x03,
    GET_CURRENT         = 0x04, // electrical current
    // SendPm = 0x05??
    SET_ENC_ZERO        = 0x06, // set the current encoder value to zero
    SET_POS_GAIN_0      = 0x07,
    SET_POS_GAIN_1      = 0x08,
    SET_CUR_GAIN_0      = 0x09,
    SET_CUR_GAIN_1      = 0x0A,
    // SWITCH_DRIVER = 0x0B ??
    // GoHome = 0x0C ??
    SET_OPENLOOP_PWM    = 0x0D,
    SET_MOTOR_CTRL_ON   = 0x0E,
    SET_MOTOR_CTRL_OFF  = 0x0F,
    SET_CTRL_MODE       = 0x10,
    
    GOTO_HOME           = 0x11,
    
    SET_DEADZONE        = 0x20,
    GET_PARAMETERS      = 0x24,
    
    SET_HOME_PARAM      = 0x30,
    SET_ENC_RES         = 0x38, // set encoder resolution
    SET_MAX_ACC_VEL     = 0x40,
    SET_LOW_POS_LIM     = 0x50,
    SET_UPP_POS_LIM     = 0x56,
    SET_HOME_VEL_ACC    = 0x60,
    
    GAIN_OVERRIDE       = 0x6F,
    SET_BOARD_NUM       = 0xF0,
    SET_JAM_SAT_LIM     = 0xF2, // set the "jam" saturation limit
    SET_ERR_BOUND       = 0xF3,
    INITIALIZE_BOARD    = 0xFA
    
} can_cmd_id_t;

typedef enum {
    
    PARAM_A = 1,
    PARAM_B = 2,
    PARAM_C = 3,
    PARAM_D = 4,
    PARAM_E = 5,
    PARAM_F = 6,
    
    PARAM_G = 20,
    PARAM_H = 21,
    PARAM_I = 22
    
} can_param_type_t;

typedef enum {
    
    ENCODER_REPLY       = 0x60,
    
    
} can_reply_id_t;

} // namespace HuboCan

#endif // HUBOCANID_H
