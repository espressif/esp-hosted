// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __PLATFORM_WRAPPER_H
#define __PLATFORM_WRAPPER_H

/* Driver Handle */
struct esp_hosted_driver_handle_t;

/* TODO: replicate in linux */
/*
 * control_path_platform_init function initializes the control
 * path data structures
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_init(void);

/*
 * control_path_platform_deinit function cleans up the control
 * path library data structure
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_deinit(void);
/*
 * esp_hosted_malloc function allocates size bytes.
 * Input parameter
 *      size    :   Number of Bytes
 * Returns
 *      pointer to allocated memory
 */
void* esp_hosted_malloc(size_t size);

/*
 * esp_hosted_calloc function allocates memory for an array
 * of nmemb elements of size bytes each.
 * Input parameter
 *      size    :   Number of Bytes
 *      nmemb   :   Number of blocks of size bytes
 * Returns
 *     pointer to allocated memory
 */
void* esp_hosted_calloc(size_t blk_no, size_t size);

/*
 * esp_hosted_free function frees the memory space pointed to by ptr.
 * Input parameter
 *      ptr     :   Address of pointer to allocated memory
 */

void esp_hosted_free(void* ptr);

/*
 * esp_hosted_driver_open function opens driver interface.
 *
 * Input parameter
 *      transport                   :   Pointer to transport driver
 * Returns
 *      esp_hosted_driver_handle    :   Driver Handle
 */
struct esp_hosted_driver_handle_t* esp_hosted_driver_open (const char* transport); 

/*
 * esp_hosted_driver_write function writes in_count bytes
 * from buffer to driver interface
 *
 * Input parameter
 *      esp_hosted_driver_handle    :   Driver Handler
 *      buf                         :   Data Buffer (Data written from buf to 
 *                                      driver interface)
 *      in_count                    :   Number of Bytes to be written
 * Output parameter
 *      out_count                   :   Number of Bytes written
 *
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int esp_hosted_driver_write (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
     uint8_t* buf, int in_count, int* out_count);

/*
 * esp_hosted_driver_read function reads read_len bytes
 * from driver interface to data buffer
 *
 * Input parameter
 *      esp_hosted_driver_handle    :   Driver Handle
 *      read_len                    :   Number of Bytes to be read
 *      wait                        :   Maximum timeout for read
 * Output parameter
 *      buf_len                     :   Length of received data
 * Returns
 *      buf                         :   Data Buffer (Data written on
 *                                      data buffer from driver interface)
 */

uint8_t* esp_hosted_driver_read (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
    int read_len, uint8_t wait, uint32_t* buf_len);

/*
 * esp_hosted_driver_close function closes driver interface.
 *
 * Input parameter
 *      esp_hosted_driver_handle    :   Driver Handle
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */

int esp_hosted_driver_close (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle);
#endif
