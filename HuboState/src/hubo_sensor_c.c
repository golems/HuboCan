
#include "../hubo_sensor_c.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

hubo_data_error_t hubo_data_header_check(const hubo_data* data)
{
    if(NULL == data)
        return HUBO_DATA_NULL;
    
    const hubo_data_header_t* header_check = (hubo_data_header_t*)data;
    if( strcmp(header_check->code, HUBO_DATA_HEADER_CODE) != 0 )
    {
        fprintf(stderr, "Your hubo_data has a malformed header!!\n");
        return HUBO_DATA_MALFORMED_HEADER;
    }
    else
        return HUBO_DATA_OKAY;
}
