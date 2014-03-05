
extern "C" {
#include "HuboCmd/hubo_cmd_c.h"
}

#include <iostream>

int main(int argc, char* argv[])
{
    size_t num_joints = 10;
    hubo_cmd_data* cx = hubo_cmd_init_data(num_joints);

    hubo_cmd_data* test_byte = cx;

    size_t byte_count = 0;
    std::cout << "Header:" << std::endl;
    for(size_t i=0; i < 16; ++i)
    {
        std::cout << i << ":\t" << *(test_byte+i) << std::endl;
        ++byte_count;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)test_byte;

    std::cout << "pid:\t" << (unsigned int)header->pid << std::endl;
    std::cout << "bitmap:\t" << (unsigned int)header->bitmap << std::endl;


    hubo_joint_cmd_t jc;
    jc.mode = HUBO_CMD_RIGID;
    jc.position = 1;
    jc.base_torque = 2;

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data_set_joint_cmd(cx, &jc, 3);

    hubo_cmd_data_get_joint_cmd(&jc, cx, 4);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data_get_joint_cmd(&jc, cx, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    hubo_cmd_data* comp = hubo_cmd_init_data(num_joints);
    std::cout << "About to compress..." << std::endl;
    hubo_cmd_data_compressor(comp, cx);
    std::cout << "...compressed!" << std::endl;

    hubo_cmd_data_get_joint_cmd(&jc, comp, 3);

    std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;

    if(hubo_cmd_data_check_if_joint_is_set(comp, 4))
    {
        hubo_cmd_data_get_joint_cmd(&jc, comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;
    }

    jc.position = 10;
    jc.base_torque = 15;

    hubo_cmd_data_set_joint_cmd(cx, &jc, 4);

    hubo_cmd_data_compressor(comp, cx);

    if(hubo_cmd_data_check_if_joint_is_set(comp, 4))
    {
        hubo_cmd_data_get_joint_cmd(&jc, comp, 4);
        std::cout << "mode:" << jc.mode << ", pos:" << jc.position << ", tq:" << jc.base_torque << std::endl;
    }

    return 0;
}
