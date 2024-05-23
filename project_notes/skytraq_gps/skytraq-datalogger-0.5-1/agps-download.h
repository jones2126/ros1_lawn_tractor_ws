#ifndef agps_download_h
#define agps_download_h

#include <stdio.h>
#include <stdlib.h>

typedef struct MemoryStruct {
    unsigned char *memory;
    size_t size;
    unsigned checksumA; /* sum of all bytes modulo 256 */
    unsigned checksumB; /* sum of the first 0x10000 bytes modulo 256 */
} agps_data;

int skytraq_download_agps_data( agps_data* chunk );

#endif
