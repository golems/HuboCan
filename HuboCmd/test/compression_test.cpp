
extern "C" {
#include "hubo_cmd_c.h"
}

#include <iostream>

int main(int argc, char* argv[])
{
    size_t num_joints = 10;
    hubo_cmd_cx_t cx;
    hubo_cmd_init_cx(&cx, num_joints);

    hubo_cmd_data* test_byte = cx.data;

    size_t byte_count = 0;
    std::cout << "Header:" << std::endl;
    for(size_t i=0; i < 16; ++i)
    {
        std::cout << i << ":\t" << *(test_byte+i) << std::endl;
        ++byte_count;
    }

    std::cout << "pid:\t" << (unsigned int)(uint8_t)(*(test_byte+byte_count)) << std::endl;
    ++byte_count;

    std::cout << "bitmap:\t" << (unsigned int)(uint8_t)(*(test_byte+byte_count)) << std::endl;
    ++byte_count;

    hubo_joint_cmd_t jc;
    jc.mode = HUBO_CMD_POSITION;
    jc.position = 1;
    jc.torque = 2;

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;

    hubo_cmd_cx_set_joint_cmd(&cx, &jc, 3);

    hubo_cmd_cx_get_joint_cmd(&jc, &cx, 4);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;

    hubo_cmd_cx_get_joint_cmd(&jc, &cx, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;

    hubo_cmd_cx_t comp; hubo_cmd_init_cx(&comp, num_joints);
    std::cout << "About to compress..." << std::endl;
    hubo_cmd_cx_compressor(&comp, &cx);
    std::cout << "...compressed!" << std::endl;

    hubo_cmd_cx_get_joint_cmd(&jc, &comp, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;

    if(hubo_cmd_data_check_if_joint_is_set(comp.data, 4))
    {
        hubo_cmd_cx_get_joint_cmd(&jc, &comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;
    }

    jc.position = 10;
    jc.torque = 15;

    hubo_cmd_cx_set_joint_cmd(&cx, &jc, 4);

    hubo_cmd_cx_compressor(&comp, &cx);

    if(hubo_cmd_data_check_if_joint_is_set(comp.data, 4))
    {
        hubo_cmd_cx_get_joint_cmd(&jc, &comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.torque << std::endl;
    }

    return 0;
}
