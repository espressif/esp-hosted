# Hosted solution with ESP32
This project adds a capability to use ESP32 as a communication processor for Wi-Fi and bluetooth/BLE connectivity with an external host. The project provides ESP32 side firmware, example Linux driver and protocol description. This can directly be used with Linux based hosts or can easily be ported to other MCUs with available open protocol description.

## Wi-Fi connectivity solution
This project uses a protobuf based command-set for control path and uses a separate connection (currently supported on SDIO) for data path. The ESP32 provides a simple interface to the host to provide ethernet interface that can transmit and receive 802.3 frames. This allows the TCP/IP and higher level protocol stack to run on the host.

## Bluetooth/BLE connectivity solution
This functionality is provided through standard HCI interface created either over SDIO or UART. Linux based host can use standard hci tools/commands to control this interface.

## Pointers to start with ESP-Hosted Solutions

[For hardware setup and initial configuration](docs/Setup.md)

[Understanding Protocol followed behind ESP-Hosted](docs/Design.md)

[Starting with ESP-Hosted](docs/Getting_started.md)

[Stuck somewhere? Here is basic troubleshoot guide](docs/Troubleshoot.md)
