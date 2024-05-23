#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>

#define EPHERMIS_URL "ftp://skytraq:skytraq@60.250.205.31/ephemeris/Eph.dat"

typedef struct MemoryStruct {
    char *memory;
    size_t size;
    unsigned checksumA; /* sum of all bytes modulo 256 */
    unsigned checksumB; /* sum of the first 0x10000 bytes modulo 256 */
} agps_data;

static void *myrealloc(void *ptr, size_t size);

static void *myrealloc(void *ptr, size_t size) {
    /* There might be a realloc() out there that doesn't like reallocing
       NULL pointers, so we take care of it here */
    if (ptr)
        return realloc(ptr, size);
    else
        return malloc(size);
}

static size_t WriteMemoryCallback(void *ptr, size_t size, size_t nmemb, void *data) {
    size_t realsize = size * nmemb;
    agps_data* mem = (agps_data*)data;

    mem->memory = myrealloc(mem->memory, mem->size + realsize + 1);
    if (mem->memory) {
        memcpy(&(mem->memory[mem->size]), ptr, realsize);
        mem->size += realsize;
        mem->memory[mem->size] = 0;
    }
    return realsize;
}

void skytraq_agps_calculate_checksums( agps_data* chunk ) {
    unsigned long sumA = 0;
    unsigned long sumB = 0;
    unsigned counter = 0;
    unsigned i;

    for ( i = 0; i < chunk->size; i++ ) {
        sumA += chunk->memory[i];
        if ( counter < 0x10000 ) {
            sumB += chunk->memory[i];
            counter++;
        }
    }

    chunk->checksumA = sumA % 256;
    chunk->checksumB = sumB % 256;
}

int skytraq_download_agps_data( agps_data* chunk ) {
    CURL *curl_handle;

    chunk->memory=NULL;
    chunk->size = 0;

    curl_global_init(CURL_GLOBAL_ALL);
    curl_handle = curl_easy_init();
    curl_easy_setopt(curl_handle, CURLOPT_URL, EPHERMIS_URL );
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, chunk);
    curl_easy_perform(curl_handle);
    curl_easy_cleanup(curl_handle);
    curl_global_cleanup();

    if ( chunk->size ) {
        skytraq_agps_calculate_checksums(chunk);
    }

    return chunk->size > 0;
}
