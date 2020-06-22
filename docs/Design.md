# Architecture
Following diagram depicts s/w and h/w components that are part of ESP hosted solution. This document briefly explains the software components that are part of this system.

![ESP-Hosted Block Diagram](esp-hosted-block-diagram.png).

## The Host Software
The host software consists of the host driver and a control path implementation.

### The Host Driver
The host driver consists of following:
1. 802.3 Network Interface
- The driver registers a network interface with linux kernel and implements a data path to allow exchange of 802.3 frames with ESP module.
- It provides 2 network interfaces to host: ethsta0 and ethap0.
- Through ethsta0 interface, the host can interact with external Access point and other wi-fi stations.
- Through ethap0 interface, the host can interact with wi-fi clients connected to soft AP created on ESP module.

2. BT HCI Interface
- BT/BLE is supported over SDIO or over UART. This section is applicable only when BT/BLE is supported over SDIO.
- The host driver registers a hci interface with linux kernel.
- The host can access BT/BLE functionality of ESP module over this interface.

3. Serial driver
- This registers a serial interface with linux kernel.
- This is used as a control interface to configure wi-fi functionality of ESP module

4. SDIO driver
- This implements a data path over SDIO interface
- This data path is used by above components in order to interact with ESP module
- The communication protocol is explained in further sections of this document

### Control Path
- This consists of custom commands which are based on protobufs.
- These commands are used to control and configure wi-fi functionality on ESP module
- It makes use of serial interface created by host driver

## ESP Firmware
This includes ESP hosted application and existing peripheral drivers from esp-idf repo. ESP hosted application performs following activities:
- It implements SDIO driver which essentially makes ESP module a SDIO slave device.
- It opens up communication path between a host (e.g. Raspberry-Pi) and various functional blocks on ESP module (such as Wi-Fi, BT/BLE etc)
- It implements control interface commands which are used to control and configure Wi-Fi.
- It sets up UART transport layer for support BT/BLE over UART
- It utilizes following drivers from esp-idf repository
	- Wi-Fi driver
	- HCI controller driver
	- SDIO Slave driver

# Protocol Definition
This section explains the communication protocol between a host and ESP module. As mentioned earlier, the basic transport layer is implemented over SDIO. BT/BLE is also supported over UART which follows standard UART protocol.

## SDIO transport layer
ESP32 module advertises 2 SDIO functions. ESP hosted solution is implemented on function 1. Though function 2 is advertised, it is not in use.

Following are few important SDIO registers provided by ESP Module:
* 0x3FF5508C: Interrupt vector used by host to interrupt slave
```
bit 0: Open data path on slave device
bit 1: Close data path on slave device
bit 2: Reset SDIO queues on slave device
```
* 0x3FF55058: Interrupt status register used by slave to interrupt host
* 0x3FF55060: Accumulated value of data length sent by slave
* 0x3FF55044: Accumulated number of buffers for receiving packets at slave
* 0x3FF5506C: Device capabilities. Indicates features supported by ESP32 device.
```
bit 0: WLAN support
bit 1: BT supported over UART
bit 2: BT supported over SDIO
bit 3: BT mode - BLE only mode
bit 4: BT mode - BR/EDR only mode
```

### Initialization of slave device
1. Soft reset SDIO slave
	1. Host resets SDIO part of slave by setting bit 2 of register at 0x3FF5508C
	2. This generates an interrupt for slave device, on which firmware on slave resets its SDIO related data structures.
2. Host reads accumulated length and buffer count at slave.
	1. Host reads and processes 0x3FF55060 and 0x3FF55044 registers and stores the values in it's data structure
	2. These counters are required while performing read and write operation on SDIO interface
3. Open data path on slave
	1. Host sets 0th bit of 0x3FF5508C interrupt register
	2. This indicates slave that host is ready for data transmission

### Data transfer from Host to slave
1. Get Buffer count
	1. Host reads the current buffer count from slave [0x3FF55044]
	2. Based on that value, host calculates the number of available buffers at slave
	3. The host transfers the packet only when slave has required number of free buffers.
	4. Size of a buffer at slave is 2048 bytes
2. The host transfers data in multiples of 512 bytes and max data length per write operation is limited to buffer size [2048 bytes]
3. Host then updates it's own counter that keeps track of number of buffers it has transmitted.

### Data transfer from slave to host
1. Whenever slave has data to transfer, it updates the length in 0x3FF55060 registers and generates an interrupt for host.
2. On interruption, host reads interrupt status register [0x3FF55058]. Bit 23 of this register tells host that slave desires to send data.
3. Host then gets the length set by slave by reading register mentioned in step 1. Based on previous received byte count and this length, host understands the actual length of data packet.
4. Host performs read operation to get data from slave
5. Once it receives the data, it updates it's counter that stores byte count received from slave.

`Note: Slave stays in blocked state during steps 1 to 4 [ i.e till host reads the data packet]`

### Deinit slave device
Host sets bit 1 of 0x3FF5508C interrupt register. This tells slave device to stop the data path.

### Payload format for data transfer
* Host and slave makes use of 8 byte payload header which preceeds every data packet. This payload header provides additional information about the data packet. Based on this header, host/slave consumes transmitted data packet.

* Payload format is as below

| Field | Length | Description |
|:-------:|:---------:|:--------:|
| packet type | 2 bits | Not in use |
| interface type | 3 bits | possible values: STA(0), AP(1), Serial interface(2). Rest all values are reserved |
| interface number | 3 bits | Not in use |
| reserved | 1 byte | Not in use |
| packet length | 2 bytes | Actual length of data packet |
| offset to packet | 2 bytes | Offset of actual data packet |
| reserved | 2 bytes  | Not in use |
