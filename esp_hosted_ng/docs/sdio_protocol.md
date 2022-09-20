# SDIO Communication Protocol

## 1. Protocol Definition
This section explains the SDIO communication protocol between a host and ESP peripheral.

### 1.1 SDIO transport layer
**This section is only applicable for ESP32. ESP32-S2/ESP32-C3 does not support SDIO interface**

ESP peripheral advertises 2 SDIO functions. ESP-Hosted solution is implemented on function 1. Though function 2 is advertised, it is not in use.

Following are few important SDIO registers provided by ESP peripheral:
* 0x3FF5508C: Interrupt vector used by host to interrupt ESP peripheral
```
bit 0: Open data path on ESP peripheral device
bit 1: Close data path on ESP peripheral device
bit 2: Reset SDIO queues on ESP peripheral device
```
* 0x3FF55058: Interrupt status register used by ESP peripheral to interrupt host
* 0x3FF55060: Accumulated value of data length sent by ESP peripheral
* 0x3FF55044: Accumulated number of buffers for receiving packets at ESP peripheral
* 0x3FF5506C: Device capabilities. Indicates features supported by ESP peripheral
```
bit 0: WLAN support
bit 1: Reserved
bit 2: Reserved
bit 3: Reserved
bit 4: Reserved
```

#### 1.1.1 Initialization of ESP peripheral device

1. Soft reset SDIO interface of ESP peripheral
	* Host resets SDIO part of ESP peripheral by setting bit 2 of register at 0x3FF5508C
	* This generates an interrupt for ESP peripheral, on which firmware on ESP peripheral resets its SDIO related data structures.
2. Host reads accumulated length and buffer count at ESP peripheral.
	* Host reads and processes 0x3FF55060 and 0x3FF55044 registers and stores the values in it's data structure
	* These counters are required while performing read and write operation on SDIO interface
3. Open data path on ESP peripheral
	* Host sets 0th bit of 0x3FF5508C interrupt register
	* This indicates ESP peripheral that host is ready for data transmission

#### 1.1.2 Data transfer from Host to ESP peripheral

1. Get Buffer count
	* Host reads the current buffer count from ESP peripheral [0x3FF55044]
	* Based on that value, host calculates the number of available buffers at ESP peripheral
	* The host transfers the packet only when ESP peripheral has required number of free buffers.
	* Size of a buffer at ESP peripheral is 2048 bytes
2. The host transfers data in multiples of 512 bytes and max data length per write operation is limited to buffer size [2048 bytes]
3. Host then updates it's own counter that keeps track of number of buffers it has transmitted.

#### 1.1.3 Data transfer from ESP peripheral to host
1. Whenever ESP peripheral has data to transfer, it updates the length in 0x3FF55060 registers and generates an interrupt for host.
2. On interruption, host reads interrupt status register [0x3FF55058]. Bit 23 of this register tells host that ESP peripheral desires to send data.
3. Host then gets the length set by ESP peripheral by reading register mentioned in step 1. Based on previous received byte count and this length, host understands the actual length of data packet.
4. Host performs read operation to get data from ESP peripheral
5. Once it receives the data, it updates it's counter that stores byte count received from ESP peripheral.

`Note: ESP peripheral stays in blocked state during steps 1 to 4 [ i.e till host reads the data packet]`

#### 1.1.4 Deinit peripheral device
Host sets bit 1 of 0x3FF5508C interrupt register. This tells ESP peripheral to stop the data path.

