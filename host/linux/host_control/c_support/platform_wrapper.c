/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include "transport_pserial.h"
#include "platform_wrapper.h"

#define SUCCESS                 0
#define FAILURE                 -1
#define DUMMY_READ_BUF_LEN      64
#define EAGAIN                  11

extern const char* transport;
struct esp_hosted_driver_handle_t {
    int file_desc;
};

extern int errno;

int control_path_platform_init(void)
{
	int ret = 0, count = 0;
	uint8_t *buf = NULL;
	struct esp_hosted_driver_handle_t esp_hosted_driver_handle = {0};

	esp_hosted_driver_handle.file_desc = open(transport,O_NONBLOCK|O_RDWR);
	if (esp_hosted_driver_handle.file_desc == -1) {
		printf("Failed to open driver interface \n");
		return FAILURE;
	}

	buf = (uint8_t *)esp_hosted_calloc(1, DUMMY_READ_BUF_LEN);
	if (!buf) {
		printf("%s, Failed to allocate memory \n", __func__);
		goto close1;
	}

	do {
		count = read(esp_hosted_driver_handle.file_desc,
				(buf), (DUMMY_READ_BUF_LEN));
		if (count < 0) {
			if (-errno != -EAGAIN) {
				perror("Failed to read ringbuffer:\n");
				goto close;
			}
			break;
		}
	} while (count>0);

	esp_hosted_free(buf);
	buf = NULL;
	ret = close(esp_hosted_driver_handle.file_desc);
	if (ret < 0) {
		perror("close:");
		return FAILURE;
	}
	return SUCCESS;

close:
	esp_hosted_free(buf);
	buf = NULL;
close1:
	ret = close(esp_hosted_driver_handle.file_desc);
	if (ret < 0) {
		perror("close: Failed to close interface for control path platform init:");
	}
	return FAILURE;

}

/* TODO :: Add functionality */
int control_path_platform_deinit(void)
{
    return SUCCESS;
}

void* esp_hosted_malloc(size_t size)
{
    return malloc(size);
}

void* esp_hosted_calloc(size_t blk_no, size_t size)
{
    return calloc(blk_no, size);
}

void esp_hosted_free(void *ptr)
{
    free(ptr);
}

struct esp_hosted_driver_handle_t* esp_hosted_driver_open(const char *transport)
{
    struct esp_hosted_driver_handle_t *esp_hosted_driver_handle = NULL;
    if (!transport) {
        return NULL;
    }
    esp_hosted_driver_handle = (struct esp_hosted_driver_handle_t *)
        esp_hosted_calloc(1, sizeof(struct esp_hosted_driver_handle_t));
    if (!esp_hosted_driver_handle) {
        printf("%s, Failed to allocate memory \n",__func__);
        return NULL;
    }
    esp_hosted_driver_handle->file_desc = open(transport, O_RDWR);
    if (esp_hosted_driver_handle->file_desc == -1) {
        free(esp_hosted_driver_handle);
        perror("open: ");
        return NULL;
    }
    return esp_hosted_driver_handle;
}

int esp_hosted_driver_write (struct esp_hosted_driver_handle_t *esp_hosted_driver_handle,
    uint8_t *buf, int in_count, int *out_count)
{
    if (!esp_hosted_driver_handle || esp_hosted_driver_handle->file_desc < 0 ||
        !buf || !in_count || !out_count) {
        return FAILURE;
    }
    *out_count = write(esp_hosted_driver_handle->file_desc, buf, in_count);
    if (*out_count <= 0) {
        perror("write: ");
        return FAILURE;
    }
    return SUCCESS;
}

uint8_t* esp_hosted_driver_read (struct esp_hosted_driver_handle_t *esp_hosted_driver_handle,
    int read_len, uint8_t wait, uint32_t *buf_len)
{
    int ret = 0, count = 0, check = false, total_read_len = 0;
    struct timeval timeout;
    uint8_t *buf = NULL;
    if (!esp_hosted_driver_handle || esp_hosted_driver_handle->file_desc < 0
        || !read_len || !buf_len || !wait) {
        return NULL;
    }
    buf = (uint8_t *)esp_hosted_calloc(1, read_len);
    if (!buf) {
        printf("%s, Failed to allocate memory \n", __func__);
        return NULL;
    }
    do {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(esp_hosted_driver_handle->file_desc, &set);
        timeout.tv_sec = wait;
        timeout.tv_usec = 0;
        ret = select(esp_hosted_driver_handle->file_desc+1, &set, NULL, NULL, &timeout);
        if (ret < 0) {
            perror("select: ");
        } else if (ret == 0) {
            printf("%s, Unable to read data before %d seconds timeout \n", __func__, wait);
        } else {
/*
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 */
            if (FD_ISSET(esp_hosted_driver_handle->file_desc, &set)) {
                check = false;
                total_read_len = 0;
                do {
                    count = read(esp_hosted_driver_handle->file_desc,
                            (buf+total_read_len), (read_len-total_read_len));
                    if (count <= 0) {
                        perror("read: Failed to read fixed length serial data");
                        goto err;
                    }
                    total_read_len += count;
                } while (total_read_len < read_len);

                if (total_read_len != read_len) {
                    printf("%s, Expected fixed data %d vs received %d\n",__func__, read_len, total_read_len);
                    goto err;
                }
                ret = parse_tlv(buf, buf_len);
                if ((ret != SUCCESS) || !*buf_len) {
                   goto err;
                }
                esp_hosted_free(buf);
                buf = NULL;
/*
 * Read variable length of received data. Variable length is obtained after
 * parsing of previously read data.
 */
                buf = (uint8_t *)esp_hosted_calloc(1, *buf_len);
                if (!buf) {
                    printf("%s, Failed to allocate memory \n",__func__);
                    goto free_err;
                }
                total_read_len = 0;
                do {
                    count = read(esp_hosted_driver_handle->file_desc,
                            (buf+total_read_len), (*buf_len-total_read_len));
                    if (count <= 0) {
                        perror("read: Failed to read variable length serial data");
                        break;
                    }
                    total_read_len += count;
                } while (total_read_len < *buf_len);

                if (total_read_len != *buf_len) {
                    printf("%s, Expected variable data %d vs received %d\n",__func__, *buf_len, total_read_len);
                    goto err;
                }
                return buf;
            } else {
                check = true;
            }
        }
    } while (check);
err:
    if (buf) {
        esp_hosted_free(buf);
        buf = NULL;
    }

free_err:
    return NULL;
}

int esp_hosted_driver_close(struct esp_hosted_driver_handle_t **esp_hosted_driver_handle)
{
    if (!esp_hosted_driver_handle || !(*esp_hosted_driver_handle) || (*esp_hosted_driver_handle)->file_desc < 0) {
        return FAILURE;
    }
    if(close((*esp_hosted_driver_handle)->file_desc) < 0) {
        perror("close:");
        return FAILURE;
    }
    if (*esp_hosted_driver_handle) {
        esp_hosted_free(*esp_hosted_driver_handle);
        *esp_hosted_driver_handle = NULL;
    }
    return SUCCESS;
}
