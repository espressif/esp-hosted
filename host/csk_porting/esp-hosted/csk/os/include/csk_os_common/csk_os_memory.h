#pragma once

void *csk_os_malloc(size_t size);
void csk_os_free(void *ptr);
void *csk_os_calloc(size_t nmemb, size_t size);
