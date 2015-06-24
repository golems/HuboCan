/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "HuboCmd/hubo_cmd_c.h"

size_t hubo_cmd_data_predict_max_message_size(size_t num_joints)
{
    return sizeof(hubo_cmd_header_t)+num_joints*sizeof(hubo_joint_cmd_t);
}

size_t hubo_cmd_data_location(size_t joint_index)
{
    return hubo_cmd_data_predict_max_message_size(joint_index);
}

int hubo_cmd_data_is_compressed(const hubo_cmd_data* data)
{
    if( NULL == data )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return header->is_compressed;
}

size_t hubo_cmd_data_get_total_num_joints(const hubo_cmd_data* data)
{
    if( NULL == data )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    return (size_t)header->total_num_joints;
}


hubo_data_error_t hubo_cmd_header_check(const hubo_cmd_data *cmd_message)
{
    if(cmd_message==NULL)
    {
        fprintf(stderr, "Attempting to check the header on a null hubo_cmd_data!\n");
        return HUBO_DATA_NULL;
    }

    const hubo_cmd_header_t* header_check = (hubo_cmd_header_t*)cmd_message;
    if( strcmp(header_check->code, HUBO_CMD_HEADER_CODE) != 0)
    {
        fprintf(stderr, "Your hubo_cmd_data has a malformed header!!\n");
        return HUBO_DATA_MALFORMED_HEADER;
    }
    else
        return HUBO_DATA_OKAY;
}

hubo_cmd_data* hubo_cmd_init_data(size_t num_total_joints)
{
    hubo_cmd_data* unallocated_cmd;
    size_t cmd_size = hubo_cmd_data_predict_max_message_size(num_total_joints);
    unallocated_cmd = (hubo_cmd_data*)malloc(cmd_size);

    hubo_cmd_header_t header;
    strcpy(header.code, HUBO_CMD_HEADER_CODE);
    header.pid = getpid();
    header.is_compressed = 0;
    header.bitmap = 0;
    header.total_num_joints = num_total_joints;

    memcpy(unallocated_cmd, &header, sizeof(hubo_cmd_header_t));
    memset(unallocated_cmd+sizeof(hubo_cmd_header_t), 0, cmd_size-sizeof(hubo_cmd_header_t));

    return unallocated_cmd;
}

inline size_t hubo_cmd_data_get_min_data_size(const hubo_cmd_data *data)
{
    if(hubo_cmd_header_check(data) != HUBO_DATA_OKAY)
        return 0;

    size_t joint_count = 0;
    if( hubo_cmd_data_is_compressed(data) == 1 )
    {
        const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
        size_t num_total_joints = header->total_num_joints;
        size_t i=0;
        for(i=0; i<num_total_joints; ++i)
        {
            if( (header->bitmap >> i) & 0x01 )
            {
                ++joint_count;
            }
        }
    }
    else
    {
        joint_count = hubo_cmd_data_get_total_num_joints(data);
    }

    return hubo_cmd_data_predict_max_message_size(joint_count);
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
                        " -- (compressed=%zu, uncompressed=%zu)\n",
                        hubo_cmd_data_get_total_num_joints(compressed),
                        hubo_cmd_data_get_total_num_joints(uncompressed));
        return 0;
    }

    if(hubo_cmd_data_is_compressed(uncompressed) == 1)
    {
        fprintf(stdout, "Warning: You are requesting to compress hubo_cmd data which is already compressed!\n"
                        " -- Are you sure you know what you're doing??\n");
        memcpy(compressed, uncompressed, hubo_cmd_data_get_min_data_size(uncompressed) );
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

hubo_data_error_t hubo_cmd_data_set_joint_cmd(hubo_cmd_data *data, const hubo_joint_cmd_t *cmd, size_t joint_index)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(data);
    if(joint_index >= total_num_joints)
    {
        if(joint_index == (size_t)(-1))
        {
            fprintf(stderr, "You have requested an InvalidIndex from your hubo_cmd_data!\n"
                    " -- Joint count:%zu\n", total_num_joints);
            return HUBO_DATA_OUT_OF_BOUNDS;
        }

        fprintf(stderr, "Attempting to set an out-of-bounds joint value in a hubo_cmd_data!\n"
                " -- Index:%zu, Joint count:%zu\n", joint_index, total_num_joints);
        return HUBO_DATA_OUT_OF_BOUNDS;
    }

    if(hubo_cmd_data_is_compressed(data) == 1)
    {
        fprintf(stderr, "Attempting to set a joint value in a hubo_cmd_data which is compressed. Compressed data is read-only!!\n");
        return HUBO_DATA_READ_ONLY;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    header->bitmap |= (0x01 << joint_index);

    memcpy(data+hubo_cmd_data_location(joint_index), cmd, sizeof(hubo_joint_cmd_t));

    return HUBO_DATA_OKAY;
}

hubo_data_error_t hubo_cmd_data_register_joint(hubo_cmd_data *data, size_t joint_index)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(data);
    if(joint_index >= total_num_joints)
    {
        if(joint_index == (size_t)(-1))
        {
            fprintf(stderr, "Attempting to register an InvalidIndex in your hubo_cmd_data!\n"
                    " -- Joint count:%zu\n", total_num_joints);
            return HUBO_DATA_OUT_OF_BOUNDS;
        }

        fprintf(stderr, "Attempting to register an out-of-bounds joint value in your hubo_cmd_data!\n"
                " -- Index:%zu, Joint count:%zu\n", joint_index, total_num_joints);
        return HUBO_DATA_OUT_OF_BOUNDS;
    }

    if(hubo_cmd_data_is_compressed(data) == 1)
    {
        fprintf(stderr, "Attempting to register a joint value in a hubo_cmd_data which is compressed. Compressed data is read-only!!\n");
        return HUBO_DATA_READ_ONLY;
    }

    hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    header->bitmap |= (0x01 << joint_index);

    return HUBO_DATA_OKAY;
}

hubo_data_error_t hubo_cmd_data_unregister_released_joints(hubo_cmd_data *data)
{
    hubo_data_error_t check = hubo_cmd_header_check(data);
    if( check != HUBO_DATA_OKAY )
    {
        return check;
    }

    size_t num_total_joints = hubo_cmd_data_get_total_num_joints(data);
    size_t i=0;
    hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
    for(i=0; i<num_total_joints; ++i)
    {
        hubo_joint_cmd_t* cmd = hubo_cmd_data_access_joint_cmd(data, i);
        if(HUBO_CMD_RELEASE == cmd->mode)
        {
            cmd->mode = HUBO_CMD_IGNORE;
            uint64_t inverse_bitmap = ~(header->bitmap);
            inverse_bitmap |= (0x01 << i);
            header->bitmap = ~(inverse_bitmap);
        }
    }

    return HUBO_DATA_OKAY;
}

hubo_joint_cmd_t* hubo_cmd_data_access_joint_cmd(hubo_cmd_data *data, size_t joint_index)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(data);
    if(joint_index >= total_num_joints)
    {
        if(joint_index == (size_t)(-1))
        {
            fprintf(stderr, "Attempting to access an InvalidIndex from your hubo_cmd_data!\n"
                    " -- Joint count:%zu\n", total_num_joints);
            return NULL;
        }

        fprintf(stderr, "Attempting to access an out-of-bounds joint value in a hubo_cmd_data!\n"
                " -- Index:%zu, Maximum:%zu\n", joint_index, total_num_joints-1);
        return NULL;
    }

    if(hubo_cmd_data_is_compressed(data) == 1)
    {
        fprintf(stderr, "Attempting to access a joint value in a hubo_cmd_data which is compressed. Compressed data is read-only!!\n");
        return NULL;
    }

    return (hubo_joint_cmd_t*)(data+hubo_cmd_data_location(joint_index));
}

hubo_data_error_t hubo_cmd_data_get_joint_cmd(hubo_joint_cmd_t *output, const hubo_cmd_data *data, size_t joint_index)
{
    size_t total_num_joints = hubo_cmd_data_get_total_num_joints(data);
    if(joint_index >= total_num_joints)
    {
        if(joint_index == (size_t)(-1))
        {
            fprintf(stderr, "Attempting to get an InvalidIndex from your hubo_cmd_data!\n"
                    " -- Joint count:%zu\n", total_num_joints);
            return HUBO_DATA_OUT_OF_BOUNDS;
        }

        fprintf(stderr, "Attempting to get an out-of-bounds joint value from your hubo_cmd_data!\n"
                " -- Index:%zu, Maximum:%zu\n", joint_index, total_num_joints-1);
        return HUBO_DATA_OUT_OF_BOUNDS;
    }

    if(hubo_cmd_data_is_compressed(data) == 1) // The data is compressed
    {
        hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;
        if( ( (header->bitmap >> joint_index) & 0x01 ) == 0 )
        {
            fprintf(stderr, "Attempting to get joint information from a compressed data set which does not\n"
                    "\tcontain the requested joint index (%zu)\n", joint_index);
            return HUBO_DATA_UNAVAILABLE_INDEX;
        }

        size_t i=0;
        size_t location=0;
        for(i=0; i < joint_index; ++i)
        {
            if( (header->bitmap >> i ) & 0x01 )
            {
                ++location;
            }
        }

        memcpy(output, data+hubo_cmd_data_location(location), sizeof(hubo_joint_cmd_t));
    }
    else if(hubo_cmd_data_is_compressed(data) == 0) // The data is NOT compressed
    {
        memcpy(output, data+hubo_cmd_data_location(joint_index), sizeof(hubo_joint_cmd_t));
    }
    else
    {
        fprintf(stderr, "Your hubo_cmd_data is malformed!! The compressed flag believes it is '%d'\n",
                hubo_cmd_data_is_compressed(data));
        return HUBO_DATA_MALFORMED_HEADER;
    }

    return HUBO_DATA_OKAY;
}

size_t hubo_cmd_data_get_size(const hubo_cmd_data *data)
{
    if( hubo_cmd_header_check(data) !=0 )
        return 0;

    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;

    if(header->is_compressed == 0)
        return hubo_cmd_data_predict_max_message_size(header->total_num_joints);

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

    return hubo_cmd_data_predict_max_message_size(joints_used);
}


int hubo_cmd_data_check_if_joint_is_set(const hubo_cmd_data *data, size_t joint_index)
{
    const hubo_cmd_header_t* header = (hubo_cmd_header_t*)data;

    if( joint_index >= header->total_num_joints )
        return -1;
    else
        return (header->bitmap >> joint_index) & 0x01;

}






