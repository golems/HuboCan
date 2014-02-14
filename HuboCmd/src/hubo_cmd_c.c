
#include "hubo_cmd_c.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


int hubo_cmd_header_check(const hubo_cmd_data *cmd_message)
{
    char header_check[hubo_cmd_header_code_size];
    memcpy(header_check, cmd_message, hubo_cmd_header_code_size);
    if( strcmp(header_check, hubo_cmd_header_code) != 0)
    {
        fprintf(stderr, "Your hubo_cmd data has a malformed header!!\n");
        return -1;
    }
    else
        return 0;
}

void hubo_cmd_init_cx(hubo_cmd_cx_t *context, size_t num_total_joints)
{
    context->data = hubo_cmd_init_data(num_total_joints);
    context->data_size = hubo_cmd_data_max_message_size(num_total_joints);
}

hubo_cmd_data* hubo_cmd_init_data(size_t num_total_joints)
{
    hubo_cmd_data* unallocated_cmd;
    size_t cmd_size = hubo_cmd_data_max_message_size(num_total_joints);
    unallocated_cmd = (hubo_cmd_data*)malloc(cmd_size);

    hubo_cmd_header_t header;
    strcpy(header.code, hubo_cmd_header_code);
    header.pid = getpid();
    header.is_compressed = 0;
    header.bitmap = 0;
    header.total_num_joints = num_total_joints;

    memcpy(unallocated_cmd, &header, sizeof(hubo_cmd_header_t));
    memset(unallocated_cmd+sizeof(hubo_cmd_header_t), 0, cmd_size-sizeof(hubo_cmd_header_t));

    return unallocated_cmd;
}

size_t hubo_cmd_cx_compressor(hubo_cmd_cx_t *compressed, const hubo_cmd_cx_t *uncompressed)
{

    compressed->data_size = hubo_cmd_data_compressor(compressed->data,
                                                     uncompressed->data);
    return compressed->data_size;
}


size_t hubo_cmd_data_compressor(hubo_cmd_data *compressed,
                                const hubo_cmd_data *uncompressed)
{
    if(hubo_cmd_header_check(uncompressed) != 0)
        return 0;

    if(hubo_cmd_header_check(compressed) != 0)
        return 0;

    if( hubo_cmd_data_get_total_num_joints(compressed)
            < hubo_cmd_data_get_total_num_joints(uncompressed) )
    {
        fprintf(stderr, "The size available for the compressed hubo_cmd_cx_t must be greater than\n"
                        "\tor equal to the size of the uncompressed!\n"
                        " -- (compressed=%d, uncompressed=%d)\n",
                        (int)hubo_cmd_data_get_total_num_joints(compressed),
                        (int)hubo_cmd_data_get_total_num_joints(uncompressed));
        return 0;
    }

    if(hubo_cmd_data_is_compressed(uncompressed) == 1)
    {
        fprintf(stdout, "Warning: You are requesting to compress hubo_cmd data which is already compressed!\n"
                        " -- Are you sure you know what you're doing??\n");
        memcpy(compressed, uncompressed,
               hubo_cmd_data_max_message_size(hubo_cmd_data_get_total_num_joints(uncompressed)) );
        return hubo_cmd_data_get_size(compressed);
    }

    hubo_cmd_header_t header;
    memcpy(&header, uncompressed, sizeof(hubo_cmd_header_t));
    header.is_compressed = 1;

    memcpy(compressed, &header, sizeof(hubo_cmd_header_t));

    size_t comp_memory_count = sizeof(hubo_cmd_header_t);
    size_t uncomp_memory_count = sizeof(hubo_cmd_header_t);

    size_t num_total_joints = header.total_num_joints;
    size_t joint_count=0;
    size_t i=0;

    for(i=0; i<num_total_joints; ++i)
    {
        if( (header.bitmap >> i) & 0x01 )
        {
            memcpy(compressed+comp_memory_count, uncompressed+uncomp_memory_count,
                   sizeof(hubo_joint_cmd_t));

            ++joint_count;
            comp_memory_count += sizeof(hubo_joint_cmd_t);
        }

        uncomp_memory_count += sizeof(hubo_joint_cmd_t);
    }

    return joint_count;
}

void hubo_cmd_cx_set_joint_cmd(hubo_cmd_cx_t *context,
                               const hubo_joint_cmd_t *cmd,
                               size_t joint_num)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(context->data);
    if(joint_num >= total_num_joints)
    {
        fprintf(stderr, "Attempting to set an out-of-bounds joint value in a hubo_cmd_cx_t!\n"
                " -- Index:%zu, Maximum:%zu\n", joint_num, total_num_joints-1);
        return;
    }

    if(hubo_cmd_data_is_compressed(context->data) == 1)
    {
        fprintf(stderr, "Attempting to set a joint value in a compressed hubo_cmd_cx_t. This is NOT valid!!\n");
        return;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)context->data;
    header->bitmap |= 0x01 << joint_num;

    memcpy(context->data+hubo_cmd_data_location(joint_num), cmd, sizeof(hubo_joint_cmd_t));
}

void hubo_cmd_cx_get_joint_cmd(hubo_joint_cmd_t* output, const hubo_cmd_cx_t *context, size_t joint_num)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(context->data);
    if(joint_num >= total_num_joints)
    {
        fprintf(stderr, "Attempting to get an out-of-bounds joint value in a hubo_cmd_cx_t!\n"
                " -- Index:%zu, Maximum:%zu\n", joint_num, total_num_joints-1);
        return;
    }

    if(hubo_cmd_data_is_compressed(context->data) == 1) // The data is compressed
    {
        hubo_cmd_header_t* header = (hubo_cmd_header_t*)context->data;
        if( ( (header->bitmap >> joint_num) & 0x01 ) == 0 )
        {
            fprintf(stderr, "Attempting to get joint information from a compressed data set which does not\n"
                    "\tcontain the requested joint index (%zu)\n", joint_num);
            return;
        }

        size_t i=0;
        size_t location=0;
        for(i=0; i < joint_num; ++i)
        {
            if( (header->bitmap >> i ) & 0x01 )
            {
                ++location;
            }
        }

        memcpy(output, context->data+hubo_cmd_data_location(location), sizeof(hubo_joint_cmd_t));
    }
    else if(hubo_cmd_data_is_compressed(context->data) == 0) // The data is NOT compressed
    {
        memcpy(output, context->data+hubo_cmd_data_location(joint_num), sizeof(hubo_joint_cmd_t));
    }
    else
    {
        fprintf(stderr, "Your hubo_cmd_cx_t is malformed!! The compressed flag believes it is '%d'\n", hubo_cmd_data_is_compressed(context->data));
    }
}


size_t hubo_cmd_data_get_size(const hubo_cmd_data *data)
{
    if( hubo_cmd_header_check(data) !=0 )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;

    if(header->is_compressed == 0)
        return hubo_cmd_data_max_message_size(header->total_num_joints);

    size_t i=0;
    size_t joints_used = 0;
    size_t total_num_joints = header->total_num_joints;
    for(i=0; i < total_num_joints; ++i)
    {
        if( (header->bitmap >> i) & 0x01 )
        {
            ++joints_used;
        }
    }

    return hubo_cmd_data_max_message_size(joints_used);
}


int hubo_cmd_data_check_if_joint_is_set(const hubo_cmd_data *data, size_t joint_index)
{
    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;

    if( joint_index >= header->total_num_joints )
        return -1;
    else
        return (header->bitmap >> joint_index) & 0x01;

}






