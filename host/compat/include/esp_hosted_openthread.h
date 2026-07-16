/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU OpenThread type + API names. */

#ifndef __ESP_HOSTED_OPENTHREAD_H__
#define __ESP_HOSTED_OPENTHREAD_H__

#include "eh_host_openthread.h"
#include "eh_host_port_openthread.h"

#define esp_hosted_openthread_radio_transport_t   eh_host_port_openthread_radio_transport_t
#define esp_hosted_openthread_uart_config_t       eh_host_port_openthread_uart_config_t
#define esp_hosted_openthread_radio_config_t      eh_host_port_openthread_radio_config_t

#define esp_hosted_openthread_query_t             eh_host_openthread_query_t

#define HOSTED_OPENTHREAD_TRANSPORT_UART          EH_HOST_PORT_OPENTHREAD_TRANSPORT_UART
#define HOSTED_OPENTHREAD_TRANSPORT_MAX           EH_HOST_PORT_OPENTHREAD_TRANSPORT_MAX
#define HOSTED_OPENTHREAD_QUERY_CONFIGURED        EH_HOST_OPENTHREAD_QUERY_CONFIGURED
#define HOSTED_OPENTHREAD_QUERY_INITED            EH_HOST_OPENTHREAD_QUERY_INITED
#define HOSTED_OPENTHREAD_QUERY_ENABLED           EH_HOST_OPENTHREAD_QUERY_ENABLED
#define HOSTED_OPENTHREAD_QUERY_READY             EH_HOST_OPENTHREAD_QUERY_READY

#define esp_hosted_openthread_get_radio_config(c)  eh_host_port_openthread_get_radio_config(c)
#define esp_hosted_openthread_rcp_init()           eh_host_openthread_rcp_init()
#define esp_hosted_openthread_rcp_deinit()         eh_host_openthread_rcp_deinit()
#define esp_hosted_openthread_rcp_start()          eh_host_openthread_rcp_start()
#define esp_hosted_openthread_rcp_stop()           eh_host_openthread_rcp_stop()
#define esp_hosted_openthread_rcp_query(q)         eh_host_openthread_rcp_query(q)

#endif /* __ESP_HOSTED_OPENTHREAD_H__ */
