# API Reference

## Header files

- [iperf.h](#file-iperfh)

## File iperf.h





## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**iperf\_cfg\_t**](#struct-iperf_cfg_t) <br>_Iperf Configuration._ |
| typedef void(\* | [**iperf\_hook\_func\_t**](#typedef-iperf_hook_func_t)  <br> |
| enum  | [**iperf\_output\_format\_t**](#enum-iperf_output_format_t)  <br>_Iperf output report format._ |
| enum  | [**iperf\_status\_t**](#enum-iperf_status_t)  <br>_Iperf status._ |
| enum  | [**iperf\_traffic\_type\_t**](#enum-iperf_traffic_type_t)  <br>_Iperf traffic type._ |

## Functions

| Type | Name |
| ---: | :--- |
|  void | [**iperf\_register\_hook\_func**](#function-iperf_register_hook_func) (iperf\_hook\_func\_t func) <br>_Registers iperf traffic start/stop hook function._ |
|  esp\_err\_t | [**iperf\_start**](#function-iperf_start) ([**iperf\_cfg\_t**](#struct-iperf_cfg_t) \*cfg) <br>_Iperf traffic start with given config._ |
|  esp\_err\_t | [**iperf\_stop**](#function-iperf_stop) (void) <br>_Iperf traffic stop._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**IPERF\_DEFAULT\_INTERVAL**](#define-iperf_default_interval)  3<br> |
| define  | [**IPERF\_DEFAULT\_IPV4\_UDP\_TX\_LEN**](#define-iperf_default_ipv4_udp_tx_len)  CONFIG\_IPERF\_DEF\_IPV4\_UDP\_TX\_BUFFER\_LEN<br> |
| define  | [**IPERF\_DEFAULT\_IPV6\_UDP\_TX\_LEN**](#define-iperf_default_ipv6_udp_tx_len)  CONFIG\_IPERF\_DEF\_IPV6\_UDP\_TX\_BUFFER\_LEN<br> |
| define  | [**IPERF\_DEFAULT\_NO\_BW\_LIMIT**](#define-iperf_default_no_bw_limit)  -1<br> |
| define  | [**IPERF\_DEFAULT\_PORT**](#define-iperf_default_port)  5001<br> |
| define  | [**IPERF\_DEFAULT\_TCP\_RX\_LEN**](#define-iperf_default_tcp_rx_len)  CONFIG\_IPERF\_DEF\_TCP\_RX\_BUFFER\_LEN<br> |
| define  | [**IPERF\_DEFAULT\_TCP\_TX\_LEN**](#define-iperf_default_tcp_tx_len)  CONFIG\_IPERF\_DEF\_TCP\_TX\_BUFFER\_LEN<br> |
| define  | [**IPERF\_DEFAULT\_TIME**](#define-iperf_default_time)  30<br> |
| define  | [**IPERF\_DEFAULT\_UDP\_RX\_LEN**](#define-iperf_default_udp_rx_len)  CONFIG\_IPERF\_DEF\_UDP\_RX\_BUFFER\_LEN<br> |
| define  | [**IPERF\_FLAG\_CLIENT**](#define-iperf_flag_client)  (1)<br> |
| define  | [**IPERF\_FLAG\_CLR**](#define-iperf_flag_clr) (cfg, flag) ((cfg) &= (~(flag)))<br> |
| define  | [**IPERF\_FLAG\_SERVER**](#define-iperf_flag_server)  (1 &lt;&lt; 1)<br> |
| define  | [**IPERF\_FLAG\_SET**](#define-iperf_flag_set) (cfg, flag) ((cfg) |= (flag))<br> |
| define  | [**IPERF\_FLAG\_TCP**](#define-iperf_flag_tcp)  (1 &lt;&lt; 2)<br> |
| define  | [**IPERF\_FLAG\_UDP**](#define-iperf_flag_udp)  (1 &lt;&lt; 3)<br> |
| define  | [**IPERF\_IPV4\_ENABLED**](#define-iperf_ipv4_enabled)  LWIP\_IPV4<br> |
| define  | [**IPERF\_IPV6\_ENABLED**](#define-iperf_ipv6_enabled)  LWIP\_IPV6<br> |
| define  | [**IPERF\_IP\_TYPE\_IPV4**](#define-iperf_ip_type_ipv4)  0<br> |
| define  | [**IPERF\_IP\_TYPE\_IPV6**](#define-iperf_ip_type_ipv6)  1<br> |
| define  | [**IPERF\_MAX\_DELAY**](#define-iperf_max_delay)  64<br> |
| define  | [**IPERF\_REPORT\_TASK\_NAME**](#define-iperf_report_task_name)  "iperf\_report"<br> |
| define  | [**IPERF\_REPORT\_TASK\_PRIORITY**](#define-iperf_report_task_priority)  CONFIG\_IPERF\_REPORT\_TASK\_PRIORITY<br> |
| define  | [**IPERF\_REPORT\_TASK\_STACK**](#define-iperf_report_task_stack)  4096<br> |
| define  | [**IPERF\_SOCKET\_ACCEPT\_TIMEOUT**](#define-iperf_socket_accept_timeout)  5<br> |
| define  | [**IPERF\_SOCKET\_RX\_TIMEOUT**](#define-iperf_socket_rx_timeout)  CONFIG\_IPERF\_SOCKET\_RX\_TIMEOUT<br> |
| define  | [**IPERF\_SOCKET\_TCP\_TX\_TIMEOUT**](#define-iperf_socket_tcp_tx_timeout)  CONFIG\_IPERF\_SOCKET\_TCP\_TX\_TIMEOUT<br> |
| define  | [**IPERF\_TRAFFIC\_TASK\_NAME**](#define-iperf_traffic_task_name)  "iperf\_traffic"<br> |
| define  | [**IPERF\_TRAFFIC\_TASK\_PRIORITY**](#define-iperf_traffic_task_priority)  CONFIG\_IPERF\_TRAFFIC\_TASK\_PRIORITY<br> |
| define  | [**IPERF\_TRAFFIC\_TASK\_STACK**](#define-iperf_traffic_task_stack)  4096<br> |
| define  | [**IPERF\_TRANS\_TYPE\_TCP**](#define-iperf_trans_type_tcp)  0<br> |
| define  | [**IPERF\_TRANS\_TYPE\_UDP**](#define-iperf_trans_type_udp)  1<br> |
| define  | [**app\_register\_iperf\_hook\_func**](#define-app_register_iperf_hook_func)  iperf\_register\_hook\_func<br> |

## Structures and Types Documentation

### struct `iperf_cfg_t`

_Iperf Configuration._

Variables:

-  union iperf\_cfg\_t::@0 @1  

-  union iperf\_cfg\_t::@2 @3  

-  int32\_t bw_lim  <br>bandwidth limit in Mbits/s

-  uint32\_t destination_ip4  <br>destination ipv4

-  char \* destination_ip6  <br>destination ipv6

-  uint16\_t dport  <br>destination port

-  uint32\_t flag  <br>iperf flag

-  iperf\_output\_format\_t format  <br>output format, K(bits/s), M(bits/s)

-  uint32\_t interval  <br>report interval in secs

-  uint16\_t len_send_buf  <br>send buffer length in bytes

-  uint32\_t source_ip4  <br>source ipv4

-  char \* source_ip6  <br>source ipv6

-  uint16\_t sport  <br>source port

-  uint32\_t time  <br>total send time in secs

-  uint8\_t type  <br>address type, ipv4 or ipv6

### typedef `iperf_hook_func_t`

```c
typedef void(* iperf_hook_func_t) (iperf_traffic_type_t type, iperf_status_t status);
```

### enum `iperf_output_format_t`

_Iperf output report format._
```c
enum iperf_output_format_t {
    MBITS_PER_SEC,
    KBITS_PER_SEC
};
```

### enum `iperf_status_t`

_Iperf status._
```c
enum iperf_status_t {
    IPERF_STARTED,
    IPERF_STOPPED
};
```

### enum `iperf_traffic_type_t`

_Iperf traffic type._
```c
enum iperf_traffic_type_t {
    IPERF_TCP_SERVER,
    IPERF_TCP_CLIENT,
    IPERF_UDP_SERVER,
    IPERF_UDP_CLIENT
};
```


## Functions Documentation

### function `iperf_register_hook_func`

_Registers iperf traffic start/stop hook function._
```c
void iperf_register_hook_func (
    iperf_hook_func_t func
) 
```

### function `iperf_start`

_Iperf traffic start with given config._
```c
esp_err_t iperf_start (
    iperf_cfg_t *cfg
) 
```


**Parameters:**


* `cfg` pointer to traffic config structure


**Returns:**

ESP\_OK on success
### function `iperf_stop`

_Iperf traffic stop._
```c
esp_err_t iperf_stop (
    void
) 
```


**Returns:**

ESP\_OK on success

## Macros Documentation

### define `IPERF_DEFAULT_INTERVAL`

```c
#define IPERF_DEFAULT_INTERVAL 3
```

### define `IPERF_DEFAULT_IPV4_UDP_TX_LEN`

```c
#define IPERF_DEFAULT_IPV4_UDP_TX_LEN CONFIG_IPERF_DEF_IPV4_UDP_TX_BUFFER_LEN
```

### define `IPERF_DEFAULT_IPV6_UDP_TX_LEN`

```c
#define IPERF_DEFAULT_IPV6_UDP_TX_LEN CONFIG_IPERF_DEF_IPV6_UDP_TX_BUFFER_LEN
```

### define `IPERF_DEFAULT_NO_BW_LIMIT`

```c
#define IPERF_DEFAULT_NO_BW_LIMIT -1
```

### define `IPERF_DEFAULT_PORT`

```c
#define IPERF_DEFAULT_PORT 5001
```

### define `IPERF_DEFAULT_TCP_RX_LEN`

```c
#define IPERF_DEFAULT_TCP_RX_LEN CONFIG_IPERF_DEF_TCP_RX_BUFFER_LEN
```

### define `IPERF_DEFAULT_TCP_TX_LEN`

```c
#define IPERF_DEFAULT_TCP_TX_LEN CONFIG_IPERF_DEF_TCP_TX_BUFFER_LEN
```

### define `IPERF_DEFAULT_TIME`

```c
#define IPERF_DEFAULT_TIME 30
```

### define `IPERF_DEFAULT_UDP_RX_LEN`

```c
#define IPERF_DEFAULT_UDP_RX_LEN CONFIG_IPERF_DEF_UDP_RX_BUFFER_LEN
```

### define `IPERF_FLAG_CLIENT`

```c
#define IPERF_FLAG_CLIENT (1)
```

### define `IPERF_FLAG_CLR`

```c
#define IPERF_FLAG_CLR (
    cfg,
    flag
) ((cfg) &= (~(flag)))
```

### define `IPERF_FLAG_SERVER`

```c
#define IPERF_FLAG_SERVER (1 << 1)
```

### define `IPERF_FLAG_SET`

```c
#define IPERF_FLAG_SET (
    cfg,
    flag
) ((cfg) |= (flag))
```

### define `IPERF_FLAG_TCP`

```c
#define IPERF_FLAG_TCP (1 << 2)
```

### define `IPERF_FLAG_UDP`

```c
#define IPERF_FLAG_UDP (1 << 3)
```

### define `IPERF_IPV4_ENABLED`

```c
#define IPERF_IPV4_ENABLED LWIP_IPV4
```

### define `IPERF_IPV6_ENABLED`

```c
#define IPERF_IPV6_ENABLED LWIP_IPV6
```

### define `IPERF_IP_TYPE_IPV4`

```c
#define IPERF_IP_TYPE_IPV4 0
```

### define `IPERF_IP_TYPE_IPV6`

```c
#define IPERF_IP_TYPE_IPV6 1
```

### define `IPERF_MAX_DELAY`

```c
#define IPERF_MAX_DELAY 64
```

### define `IPERF_REPORT_TASK_NAME`

```c
#define IPERF_REPORT_TASK_NAME "iperf_report"
```

### define `IPERF_REPORT_TASK_PRIORITY`

```c
#define IPERF_REPORT_TASK_PRIORITY CONFIG_IPERF_REPORT_TASK_PRIORITY
```

### define `IPERF_REPORT_TASK_STACK`

```c
#define IPERF_REPORT_TASK_STACK 4096
```

### define `IPERF_SOCKET_ACCEPT_TIMEOUT`

```c
#define IPERF_SOCKET_ACCEPT_TIMEOUT 5
```

### define `IPERF_SOCKET_RX_TIMEOUT`

```c
#define IPERF_SOCKET_RX_TIMEOUT CONFIG_IPERF_SOCKET_RX_TIMEOUT
```

### define `IPERF_SOCKET_TCP_TX_TIMEOUT`

```c
#define IPERF_SOCKET_TCP_TX_TIMEOUT CONFIG_IPERF_SOCKET_TCP_TX_TIMEOUT
```

### define `IPERF_TRAFFIC_TASK_NAME`

```c
#define IPERF_TRAFFIC_TASK_NAME "iperf_traffic"
```

### define `IPERF_TRAFFIC_TASK_PRIORITY`

```c
#define IPERF_TRAFFIC_TASK_PRIORITY CONFIG_IPERF_TRAFFIC_TASK_PRIORITY
```

### define `IPERF_TRAFFIC_TASK_STACK`

```c
#define IPERF_TRAFFIC_TASK_STACK 4096
```

### define `IPERF_TRANS_TYPE_TCP`

```c
#define IPERF_TRANS_TYPE_TCP 0
```

### define `IPERF_TRANS_TYPE_UDP`

```c
#define IPERF_TRANS_TYPE_UDP 1
```

### define `app_register_iperf_hook_func`

```c
#define app_register_iperf_hook_func iperf_register_hook_func
```


