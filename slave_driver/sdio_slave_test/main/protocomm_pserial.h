#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*pserial_xmit)(uint8_t *buf, ssize_t len);
typedef ssize_t (*pserial_recv)(uint8_t *buf, ssize_t len);

esp_err_t protocomm_pserial_start(protocomm_t *pc, pserial_xmit xmit, pserial_recv recv);
esp_err_t protocomm_pserial_data_ready(protocomm_t *pc, int len);

#ifdef __cplusplus
}
#endif

