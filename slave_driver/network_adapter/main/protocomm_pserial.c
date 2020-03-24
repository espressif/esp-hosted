// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
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

#include <stdio.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <protocomm.h>
#include <protocomm_priv.h>
#include "protocomm_pserial.h"

static const char TAG[] = "protocomm_pserial";

#define EPNAME_MAX                  16
#define REQ_Q_MAX                   4

#define PROTO_PSER_TLV_T_EPNAME     1
#define PROTO_PSER_TLV_T_DATA       2

struct pserial_config {
    pserial_xmit    xmit;
    pserial_recv    recv;
    xQueueHandle    req_queue;
};

static esp_err_t parse_tlv(uint8_t **buf, size_t *total_len, int *type, size_t *len, uint8_t **ptr)
{
    uint8_t *b = *buf;
    uint16_t *out_len;
    
    if (*total_len == 0) {
        return ESP_FAIL;
    }

    *type = b[0];
    out_len = (uint16_t *)(b + 1);
    *len = *out_len;
    *ptr = (uint8_t *) (b + 3);
	//printf("*len %d \n", *len);
    *total_len -= (*len + 1 + 2);
    *buf = b + 1 + 2 + (*len);
    return ESP_OK;
}

static esp_err_t protocomm_pserial_common_handler(protocomm_t *pc, uint8_t *in, size_t inlen)
{
    uint8_t *buf = in;
    size_t total_len, len;
    int type, ret;
    uint8_t *ptr;

    char epname[EPNAME_MAX];
    uint8_t *data = NULL;
    size_t data_len = 0;

    uint8_t *out = NULL;
    size_t outlen = 0;
    struct pserial_config *pserial_cfg;

    total_len = inlen;
    memset(epname, 0, EPNAME_MAX);

    while (parse_tlv(&buf, &total_len, &type, &len, &ptr) == 0) {
        ESP_LOGI(TAG, "Parsed type %d len %d", type, len);
        switch(type) {
            case PROTO_PSER_TLV_T_EPNAME:
                if (len >= EPNAME_MAX - 1) {
                    ESP_LOGE(TAG, "EP Name bigger than supported");
                    return ESP_FAIL;
                }
                memcpy(epname, ptr, len);
                ESP_LOGI(TAG, "Found ep %s", epname);
                break;
            case PROTO_PSER_TLV_T_DATA:
                data = ptr;
                data_len = len;
                break;
            default:
                ESP_LOGE(TAG, "Invalid type found in the packet");
        }
    }

    if (data == NULL || data_len == 0 || strlen(epname) == 0) {
        ESP_LOGE(TAG, "TLV components not complete for parsing");
        return ESP_FAIL;
    }
    ret = protocomm_req_handle(pc, epname, 0, data, data_len, &out, (ssize_t *) &outlen);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error in handling protocomm request %d", ret);
        return ESP_FAIL;
    }

    pserial_cfg = pc->priv;
    ret = (pserial_cfg->xmit)(out, (ssize_t) outlen);

    free(out);
    return ret;
}

esp_err_t protocomm_pserial_data_ready(protocomm_t *pc, int len)
{
	printf("data ready \n");
    struct pserial_config *pserial_cfg;
    pserial_cfg = (struct pserial_config *) pc->priv;
    if (!pserial_cfg) {
        ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
        return ESP_FAIL;
    }

    if (xQueueSend(pserial_cfg->req_queue, &len, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to indicate data ready");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t protocomm_pserial_add_ep(const char *ep_name, protocomm_req_handler_t req_handler, void *priv_data)
{
    ESP_LOGD(TAG, "Adding endpoint for SDIO_SERIAL");
    return ESP_OK;
}

static esp_err_t protocomm_pserial_remove_ep(const char *ep_name)
{
    ESP_LOGD(TAG, "Removing endpoint for SDIO_SERIAL");
    return ESP_OK;
}

static void pserial_task(void *params)
{
    protocomm_t *pc = (protocomm_t *) params;
    struct pserial_config *pserial_cfg;
    int len;
    uint8_t *buf;
    int ret;

    pserial_cfg = (struct pserial_config *) pc->priv;
    if (!pserial_cfg) {
        ESP_LOGE(TAG, "Unexpected. No pserial_cfg found");
        return;
    }

    while (xQueueReceive(pserial_cfg->req_queue, &len, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "Read request for %d", len);
        buf = (uint8_t *) malloc(len);
        if (buf == NULL) {
            ESP_LOGE(TAG,"Failed to allocate memory");
            return;
        }
        pserial_cfg->recv(buf, len);
        protocomm_pserial_common_handler(pc, buf, len);
        free(buf);
    }

    ESP_LOGI(TAG, "Unexpected termination of pserial task");
}

esp_err_t protocomm_pserial_start(protocomm_t *pc, pserial_xmit xmit, pserial_recv recv)
{
	printf("pserial start \n");
    struct pserial_config *pserial_cfg;

    if (pc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pc->add_endpoint = protocomm_pserial_add_ep;
    pc->remove_endpoint = protocomm_pserial_remove_ep;

    pserial_cfg = (struct pserial_config *) malloc(sizeof(struct pserial_config));
    if (pserial_cfg == NULL) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    pserial_cfg->xmit = xmit;
    pserial_cfg->recv = recv;
    pserial_cfg->req_queue = xQueueCreate(REQ_Q_MAX, sizeof(ssize_t));

    pc->priv = pserial_cfg;
    
    xTaskCreate(pserial_task, "pserial_task", 4096, (void *) pc, 5, NULL);

    return ESP_OK;
}

esp_err_t protocomm_pserial_stop(protocomm_t *pc)
{
    struct pserial_config *pserial_cfg;
    if (pc->priv) {
        pserial_cfg = (struct pserial_config *) pc->priv;
        vQueueDelete(pserial_cfg->req_queue);
        free(pserial_cfg);
        pc->priv = NULL;
    }

    return ESP_OK;
}
