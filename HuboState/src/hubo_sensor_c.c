
#include "../hubo_sensor_c.h"

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

hubo_data_error_t hubo_sensor_header_check(const hubo_sensor_data* sensor_message)
{
    const hubo_sensor_header_t* header_check = (hubo_sensor_header_t*)sensor_message;
    if( strcmp(header_check->code, HUBO_SENSOR_HEADER_CODE) != 0 )
    {
        fprintf(stderr, "Your hubo_sensor_data has a malformed header!!\n");
        return HUBO_DATA_MALFORMED_HEADER;
    }
    else
        return HUBO_DATA_OKAY;
}
