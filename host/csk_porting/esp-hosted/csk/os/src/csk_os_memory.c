#include <zephyr.h>

void *csk_os_malloc(size_t size)
{
    return k_malloc(size);
}

void csk_os_free(void *ptr)
{
    k_free(ptr);
}

void *csk_os_calloc(size_t nmemb, size_t size)
{
    return k_calloc(nmemb, size);
}
