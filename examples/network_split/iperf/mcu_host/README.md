| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# ESP-Hosted Network Split iPerf Example

This example demonstrates network split with iPerf testing. Network split lets the host and the ESP32 (slave) share the same IP address but handle different port ranges:

- **Host ports**: 49152-61439
- **Slave ports**: 61440-65535

## Features

- iPerf server on slave (port 5001)
- TCP echo server (port 61500)
- HTTP server (port 61600)
- Port-based packet routing between host and slave
- Shared IP address

## Building

```sh
eh.py set-target <target>
eh.py menuconfig   # set WiFi credentials + transport
eh.py build flash monitor
```

## Console Commands

| Command | Action |
|---------|--------|
| `help` | show available commands |
| `wifi connect <ssid> <password>` | connect to WiFi |
| `iperf -s` | start iPerf server |
| `demo-servers start` | start echo + HTTP demo servers |
| `demo-servers stop` | stop demo servers |
| `demo-servers status` | status of demo servers |
| `network-split-info` | show port-range configuration |

## Testing from an external client

```bash
# slave services
iperf3 -c <ip> -p 5001       # iPerf
telnet <ip> 61500            # echo server
curl http://<ip>:61600       # HTTP server

# host services (if host is running them)
ssh user@<ip>                # port 22 → host
curl http://<ip>:80          # port 80 → host
```

## Port Forwarding Rules (host-bound by default)

- TCP source ports → host: 22, 8554
- TCP destination ports → host: 22, 80, 443, 8080, 8554
- UDP destination ports → host: 53, 123

Anything else is routed by the host/slave port ranges above.

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Network split not working | `CONFIG_ESP_HOSTED_CP_NETWORK_SPLIT_ENABLED=y`, port ranges, lwIP hooks |
| iPerf connection failed | server running (`iperf -s`), client firewall, IP and port |
| Demo servers unreachable | `demo-servers start`, ports 61500/61600 not blocked |
| Poor throughput | WiFi signal, task priorities, lwIP buffer sizes |

## Notes

- The iperf component supports iperf v2.x (see [iperf-cmd](https://components.espressif.com/components/espressif/iperf-cmd)).
- For 80 MHz SPI flash on ESP-WROVER-KIT, R140-R145 must be removed; default is 40 MHz to remain safe.
