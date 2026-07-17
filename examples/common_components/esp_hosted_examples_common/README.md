<!-- SPDX-License-Identifier: Apache-2.0 -->

# esp_hosted_examples_common

Small modular WiFi helper for ESP-Hosted Linux + MCU example apps.
Maps onto the canonical WiFi state surface (init → mode → start →
{connect | softap | scan} → stop → deinit) with one function per
operation; no fat configs.

## API

```c
/* one-time platform init (idempotent) */
esp_err_t eh_example_init(void);
esp_err_t eh_example_deinit(void);

/* STA */
esp_err_t eh_example_sta_connect(const char *ssid, const char *password);
esp_err_t eh_example_sta_disconnect(void);
esp_err_t eh_example_connect(void);      /* menuconfig SSID/PWD shorthand */
esp_err_t eh_example_disconnect(void);

/* scan — channel=0 → all bands; works on 2.4 GHz and 5 GHz chips */
esp_err_t eh_example_scan(uint8_t channel,
                          wifi_ap_record_t *out, uint16_t *count);

/* SoftAP — channel 1-13 = 2.4 GHz, 36+ = 5 GHz (chip-dependent).
 * Auth = OPEN if password empty/null, else WPA2-PSK. */
esp_err_t eh_example_softap_start(const char *ssid,
                                  const char *password, uint8_t channel);
esp_err_t eh_example_softap_stop(void);
```

## Files

| File | Owns |
|---|---|
| `init.c` | init/deinit + shared netif handles |
| `sta.c` | STA connect/disconnect + menuconfig shim |
| `scan.c` | active scan |
| `ap.c` | SoftAP start/stop |
| `include/esp_hosted_examples_common.h` | public surface |
| `example_private.h` | shared internal state declarations |

## Menuconfig

*Example Connection Configuration*:

| Knob | Default |
|---|---|
| `EH_EXAMPLE_WIFI_SSID` | `myssid` |
| `EH_EXAMPLE_WIFI_PASSWORD` | `mypassword` |
| `EH_EXAMPLE_WIFI_MAXIMUM_RETRY` | `6` |

SoftAP and scan APIs take explicit args; no menuconfig knobs for them.
