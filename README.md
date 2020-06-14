# ESP-Hosted solution
ESP-Hosted project provides a way to use ESP32 as a communication processor providing Wi-Fi and Bluetooth/BLE connectivity to the host MCU. It provides a standard network interface implementation for the host to receive and transmit 802.3 frames from the host. The host can use its own TCP/IP and TLS stack above the network interface that the application can use. For BT connectivity, it provides a standard HCI interface to the host so that the host can run a Bluetooth host stack on the same. Please note that this project doesn't provide standard 802.11 interface to the host and for the control path, it provides custom command implementation on host and ESP32 side based on Protobufs.
The below diagram shows hardware and software block diagram for a typical system built with ESP-Hosted.

![alt text](docs/esp-hosted-block-diagram.png "ESP-Hosted Block Diagram")

# Getting Started 

[Hardware Setup and Compilation](docs/Setup.md)

[Getting Started](docs/Getting_started.md)

[Protocol Design](docs/Design.md)

[Troubleshooting Guide](docs/Troubleshoot.md)
