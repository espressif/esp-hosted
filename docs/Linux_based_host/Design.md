# Architecture
Following diagram depicts software and hardware components that are part of ESP-Hosted solution. This document briefly explains the software components that are part of this system.

![ESP-Hosted linux based design](linux_hosted_design.png).

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
	- BT/BLE is supported over SDIO, SPI and UART. This section is applicable only when BT/BLE is supported over SDIO and SPI.
	- The host driver registers a hci interface with linux kernel.
	- The host can access BT/BLE functionality of ESP module over this interface.
3. Virtual Serial Interface
	- This registers a serial interface with linux kernel.
	- This is used as a control interface to configure wi-fi functionality of ESP module
4. Transport Driver
	- SDIO Driver
		- This implements data path over SDIO interface
	- SPI Driver
		- This implements data path over SPI interface
	- This data path is used by above components in order to interact with ESP module
	- The communication protocol is explained in further sections of this document

### Control Path
- This consists of custom commands which are based on protobufs.
- These commands are used to control and configure wi-fi functionality on ESP module
- It makes use of serial interface created by host driver

## ESP Firmware
This includes ESP-Hosted application and existing peripheral drivers from ESP-IDF repo. ESP-Hosted application performs following activities:
- It implements SDIO and SPI transport drivers. Only one driver of these driver can be active at a time and that can be selected during project compilation.
- Selected transport driver establishes communication path between a host (e.g. Raspberry-Pi) and various functional blocks on ESP module (such as Wi-Fi, BT/BLE etc)
- ESP firmware also implements control interface commands which are used to control and configure Wi-Fi.
- It sets up UART transport layer to support BT/BLE over UART
- Following components of ESP-IDF repository are used in this project:
	- Wi-Fi driver
	- HCI controller driver
	- SDIO peripheral driver
	- SPI peripheral driver

# Protocol Definition
This section explains the communication protocol between a host and ESP module. As mentioned earlier, the basic transport layer is implemented over SDIO and SPI. BT/BLE is also supported over UART which follows standard UART protocol. Below section explains SDIO and SPI transport protocol.

### SDIO transport layer
**This section is only applicable for ESP32. ESP32-S2 does not support SDIO interface**
ESP peripheral advertises 2 SDIO functions. ESP-Hosted solution is implemented on function 1. Though function 2 is advertised, it is not in use.

Following are few important SDIO registers provided by ESP Module:
* 0x3FF5508C: Interrupt vector used by host to interrupt ESP peripheral
```
bit 0: Open data path on ESP peripheral device
bit 1: Close data path on ESP peripheral device
bit 2: Reset SDIO queues on ESP peripheral device
```
* 0x3FF55058: Interrupt status register used by ESP peripheral to interrupt host
* 0x3FF55060: Accumulated value of data length sent by ESP peripheral
* 0x3FF55044: Accumulated number of buffers for receiving packets at ESP peripheral
* 0x3FF5506C: Device capabilities. Indicates features supported by ESP32 peripheral
```
bit 0: WLAN support
bit 1: BT supported over UART
bit 2: BT supported over SDIO
bit 3: BT mode - BLE only mode
bit 4: BT mode - BR/EDR only mode
```

#### Initialization of ESP peripheral device
1. Soft reset SDIO interface of ESP peripheral
	1. Host resets SDIO part of ESP peripheral by setting bit 2 of register at 0x3FF5508C
	2. This generates an interrupt for ESP peripheral, on which firmware on ESP peripheral resets its SDIO related data structures.
2. Host reads accumulated length and buffer count at ESP peripheral.
	1. Host reads and processes 0x3FF55060 and 0x3FF55044 registers and stores the values in it's data structure
	2. These counters are required while performing read and write operation on SDIO interface
3. Open data path on ESP peripheral
	1. Host sets 0th bit of 0x3FF5508C interrupt register
	2. This indicates ESP peripheral that host is ready for data transmission

#### Data transfer from Host to ESP peripheral
1. Get Buffer count
	1. Host reads the current buffer count from ESP peripheral [0x3FF55044]
	2. Based on that value, host calculates the number of available buffers at ESP peripheral
	3. The host transfers the packet only when ESP peripheral has required number of free buffers.
	4. Size of a buffer at ESP peripheral is 2048 bytes
2. The host transfers data in multiples of 512 bytes and max data length per write operation is limited to buffer size [2048 bytes]
3. Host then updates it's own counter that keeps track of number of buffers it has transmitted.

#### Data transfer from ESP peripheral to host
1. Whenever ESP peripheral has data to transfer, it updates the length in 0x3FF55060 registers and generates an interrupt for host.
2. On interruption, host reads interrupt status register [0x3FF55058]. Bit 23 of this register tells host that ESP peripheral desires to send data.
3. Host then gets the length set by ESP peripheral by reading register mentioned in step 1. Based on previous received byte count and this length, host understands the actual length of data packet.
4. Host performs read operation to get data from ESP peripheral
5. Once it receives the data, it updates it's counter that stores byte count received from ESP peripheral.

`Note: ESP peripheral stays in blocked state during steps 1 to 4 [ i.e till host reads the data packet]`

#### Deinit peripheral device
Host sets bit 1 of 0x3FF5508C interrupt register. This tells ESP peripheral to stop the data path.


### SPI transport layer
#### Basic concepts
##### Additional pin setup
Apart from basic 4 pin SPI configuration, following are additional pins used in ESP-Hosted solution.

###### Handshake pin
This is a output pin for ESP peripheral. ESP peripheral makes use of this pin to convey its readiness for execution of SPI transaction. The host is not supposed to initiate SPI transaction if ESP peripheral has not indicated its readiness. This pin stays high till the end of SPI transaction.

###### Data ready pin
This is a output pin for ESP peripheral. This pin is used to indicate host that the ESP peripheral wants to send a data packet to it. This pin stays high till the host reads this data packet.

###### Reset/EN pin
This is a input pin for ESP peripheral. This pin resets ESP peripheral and is mandatory in SPI based ESP-Hosted solution.


#### Initialization of peripheral device
* Connection of 'EN'/Reset pin to host is mandatory in case of SPI communication. Once driver is loaded on host, it resets ESP peripheral through this pin.
* Firmware on ESP peripheral then initializes itself and preapres itself for communication over SPI interface. Once it is ready for communication, it generates INIT event for host.
* Host driver, on receiving this event, opens up data path for higher layers.

#### Data transfer between Host and ESP peripheral
* This solution makes use of SPI full duplex commmunication mode. i.e. read and write operations are performed at the same time in same SPI transaction.
* As a protocol, host is not supposed to start a transaction before ESP SPI peripheral device is ready for receiving data. Therefore, through Handshake pin, ESP peripheral indicates host when it is ready for SPI transaction.
* To allow seamless data traffic between host and ESP peripheral, ESP peripheral needs to be ready for data reception from host all the time. For that, after completion of every SPI transaction, ESP peripheral immediately queues next SPI transaction.
* The data transfer protocol works as below:
	* Each SPI transaction has a TX buffer and a RX buffer.
		* TX buffer contains data which ESP peripheral wants to send to host.
		* RX buffer is an empty buffer space, which on the completion of SPI transaction will hold data received from host.
	* ESP peripheral initiates SPI transaction by setting TX and RX buffers of size 1600 bytes. This is a maximum buffer size. i.e at a time host can send and receive at max 1600 bytes of data.
	* There are two cases with respect to TX buffer here:
		* In case if ESP peripheral has no data to transfer to host, a dummy TX buffer of size 1600 bytes is allocated and is set in SPI transaction. Packet length field in payload header of such buffer is set to 0.
		* If ESP peripheral has a valid data buffer to be sent to host, then TX buffer will point to that buffer.
	* SPI transaction length is set to 1600 bytes [irrespective of size of TX buffer]
	* Once this SPI transaction is submitted to SPI driver on ESP peripheral, Handshake pin is pulled high to indicate host that ESP peripheral is ready for transaction.
	* In case if TX buffer has valid data, Data ready pin is also pulled high by ESP peripheral.
	* Host receives an interrupt through Handshake pin. On this interrupt, host needs to decide whether or not to perform SPI transaction.
		* If Data ready pin is high, host performs SPI transaction
		* Or if host has data to transfer, then host performs SPI transaction
		* If both the above conditions are false, then host does not perform SPI transaction. This transaction is then performed later when host has data to be sent or interrupt is received on Data ready pin.
	* During this SPI transaction, TX and RX buffers are exchanged on SPI data lines.
	* Based on payload header in received buffer, both ESP peripheral and host processes the buffer.
	* On completion of transaction, ESP peripheral pulls Handshake pin low. If completed transaction had a valid TX buffer, then it also pulls Data ready pin low.


### Payload format for data transfer
* Host and ESP peripheral makes use of 8 byte payload header which preceeds every data packet.
* This payload header provides additional information about the data packet. Based on this header, host/ESP peripheral consumes transmitted data packet.
* This is applicable for both SDIO and SPI transport layers.
* Payload format is as below

| Field | Length | Description |
|:-------:|:---------:|:--------|
| interface type | 4 bits | possible values: STA(0), SoftAP(1), Serial interface(2), HCI (3), Priv interface(4). Rest all values are reserved |
| interface number | 4 bits | Not in use |
| reserved | 1 byte | Not in use |
| packet length | 2 bytes | Actual length of data packet |
| offset to packet | 2 bytes | Offset of actual data packet |
| reserved | 1 byte  | Not in use |
| packet type | 1 byte | reserved when interface type is 0,1 and 2. Applicable only for interface type 3 and 4 |
