
#include "../hubo_aux_cmd_c.h"

#include <stdio.h>

hubo_data_error_t hubo_aux_cmd_header_check(const hubo_aux_cmd_t *cmd)
{
    if( strcmp(cmd->code, HUBO_CMD_HEADER_CODE) != 0 )
    {
        fprintf(stderr, "Your hubo_aux_cmd_t has a malformed header!!\n");
        return HUBO_DATA_MALFORMED_HEADER;
    }
    else
        return HUBO_DATA_OKAY;
}
