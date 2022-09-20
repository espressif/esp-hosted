# SPI Communication Protocol

# 1. Protocol Definition
This section explains the SPI communication protocol between a host and ESP peripheral.

---

## 1.1 Basic concepts
### 1.1.1 Additional pin setup
Apart from basic 4 pin SPI configuration, following are additional pins used in ESP-Hosted solution.

#### Handshake pin
This is a output pin for ESP peripheral. ESP peripheral makes use of this pin to convey its readiness for execution of SPI transaction. The host is not supposed to initiate SPI transaction if ESP peripheral has not indicated its readiness. This pin stays high till the end of SPI transaction.

#### Data ready pin
This is a output pin for ESP peripheral. This pin is used to indicate host that the ESP peripheral wants to send a data packet to it. This pin stays high till the host reads this data packet.

#### Reset/EN pin
This is a input pin for ESP peripheral. This pin resets ESP peripheral and is mandatory in SPI based ESP-Hosted solution.

---

## 1.2 Initialization of peripheral device
* Connection of 'EN'/Reset pin to host is mandatory in case of SPI communication. Once driver is loaded on host, it resets ESP peripheral through this pin.
* Firmware on ESP peripheral then initializes itself and prepares itself for communication over SPI interface. Once it is ready for communication, it generates INIT event for host.
* Host driver, on receiving this event, opens up data path for higher layers.

---

## 1.3 Data transfer between Host and ESP peripheral
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

