
#include "HuboCan/AchIncludes.h"
#include <stdio.h>

void report_ach_errors(ach_status_t status, const char* functionName,
                       const char* achOperation, const char* channelName)
{
    if(ACH_OK != status)
    {
        fprintf(stderr, "[%s] Ach error: %s during [%s] operation with channel '%s'\n",
                functionName, ach_result_to_string(status), achOperation, channelName);
        fflush(stderr);
    }
}
