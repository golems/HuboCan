
#include "../hubo_path_c.h"

void clear_hubo_path_chunk(hubo_path_chunk_t *chunk)
{
    memset(chunk, 0, sizeof(*chunk));
    strcpy(chunk->header.code, HUBO_PATH_HEADER_CODE);
}

int check_hubo_path_chunk(const hubo_path_chunk_t *chunk)
{
    return strncmp(chunk->header.code, HUBO_PATH_HEADER_CODE, HUBO_PATH_HEADER_CODE_SIZE);
}
