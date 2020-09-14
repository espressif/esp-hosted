# Architecture
Following diagram depicts software and hardware components that are part of ESP-Hosted MCU based solution. This document briefly explains the software components that are part of this system.

![ESP-Hosted MCU based design](./MCU_based_design.png).

## The Host Software
The host software mainly consists of following building blocks.
	1. SPI host driver
	2. Virtual serial interface driver
	3. Control/Command interface
	4. Network interface layer [netif]
	5. Network stack stub
	6. Demo application

Each of these are explained in following sub sections.

#### SPI Host Driver
- ESP-Hosted solution provides Thin SPI master interface layer which transmits/receives data from SPI hardware driver and makes it available to serial or network interface
- Asynchrounous in nature, higher layers have flexibility to transmit and/or receive data as needed
- Currently, Maximum 2048 bytes of data can be trasmitted in single transmit or receive transaction

#### Virtual serial interface driver
- ESP-Hosted solution provides a generic virtual serial interface implementation.
- Control interface component of ESP-Hosted solution is built on top of this interface.
- Similarly, HCI interface can be built on top of virtual serial interface. This HCI interface can provide BT/BLE functionality to the host.  
`Note: Implementation of HCI interface and BT/BLE stack is not in scope of this project`

#### Control/Command Interface
- As mentioned above, this interface is implemented over virtual serial interface.
- This interface is used for sending control commands to control and configure Wi-Fi functionality of attached ESP device.
- This is an optional interface and in case virtual serial interface is not used, the control path or BT functionality can be used on physical UART interface connected to ESP32.

#### Network interface layer [netif]
- This is an abstraction layer between SPI host driver and a network stack.
- This gives flexibility of using any network stack with ESP-Hosted solution.
- This interface layer defines set of APIs and data structure that network stack must implement in order to make it work with SPI host driver.

#### Network stack stub
- This is a simple network stack stub which demonstrates how a network stack can implement network interface layer and work with SPI host driver.
- This does not represent actual network stack. This should be used as a reference for developing network interface layer [netif] provided by ESP-Hosted solution

#### Demo application
- This application demonstrates capabilities of ESP-Hosted solution.
- It makes use of control interface to control and configure Wi-Fi interface of attached ESP device.
- It makes use of network interface to send and receive data over Wi-Fi interface of attached ESP device.
- As demonstration of working data path, 
	- This application sends ARP request to other network devices connected over same network
	- This application also processes and responds to received ARP request packets from other network devices


## ESP Firmware
- ESP firmware consistes of ESP-Hosted application and existing peripheral drivers from ESP-IDF repository.
- ESP-Hosted application performs following activities:
	- It implements SPI transport drivers.
	- SPI trnasport driver establishes communication path between host and various functional blocks on ESP module (such as Wi-Fi, BT/BLE etc)
	- ESP firmware also implements control interface commands which are used to control and configure Wi-Fi.
	- Following components of ESP-IDF repository are used in this project:
		- Wi-Fi driver
		- HCI controller driver
		- SPI Slave driver


# Protocol Definition
This section explains communication protocol between a host and ESP module. It also explains serial interface and network interface APIs provided by ESP-Hosted Host software.

## Communication Protocol over SPI Interface
### Initialization of slave device
* Connection of 'EN' pin to host is mandatory in case of SPI communication. Once driver is loaded on host, it resets SPI slave through this pin.
* SPI slave then initializes itself and preapres itself for communication. Once it is ready for communication, it generates INIT event packet for host.
* Host driver, on receiving this event, opens up data path for higher layers.

### Data transfer between Host and slave
* This solution makes use of SPI full duplex commmunication mode. i.e. read and write operations are performed at the same time in same SPI transaction.
* As a protocol, host is not supposed to start a transaction before ESP SPI slave device is ready for receiving data. Therefore, a seaparate GPIO pin is used for a handshake signal through which ESP slave indicates host when it is ready for data reception.
* When ESP SPI slave is ready for SPI transaction,
	* It initiates SPI transaction by setting tx and rx buffers of size 2048 bytes. This is a maximum buffer size. i.e at a time host can send and receive at max 2048 bytes of data.
	* It is mandatory for ESP SPI slave to set both tx and rx buffers in each and every SPI transaction. Thus, in absence of valid tx buffer, a dummy tx buffer of size 2048 bytes is set in SPI transaction. Packet length field in payload header of dummy packet it set to 0.
	* Once a transaction is queued, ESP SPI slave uses handshake pin to indicate host that it is ready for the SPI transaction.
	* This generates an interrupt for host driver. On this interrupt, SPI host driver executes SPI transaction.
	* During this SPI transaction, tx and rx buffers are exchanged on SPI data lines.
	* Based on payload header in received buffer, both ESP SPI slave and host processes the buffer.
	* The host driver, before executing next SPI transaction, waits for an interrupt on handshake line.

### Payload format for data transfer
* Host and slave makes use of 8 byte payload header which preceeds every data packet.
* This payload header provides additional information about the data packet. Based on this header, host/slave consumes transmitted data packet.
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

## Serial interface layer
This section explains serial interface APIs and data structures provided by ESP Host software.

### Data Structures

##### struct serial_handle_s
```
typedef struct serial_handle_s {
    QueueHandle_t queue;
    uint8_t if_type;
    uint8_t if_num;
    struct serial_operations *ops;
    uint8_t state;
    void (*serial_rx_callback) (void);
} serial_handle_t;
```
- This structure holds instance of a virtual serial interface. It contains following fields:
	- __queue__: Buffer queue for storing received data packets
	- __if_type__: Interface type
	- __if_num__: Interface number
	- __ops__: Hook functions supported by virtual serial interface
	- __state__: State of a handle
	- __serial_rx_callback__: Rx callback registered by upper layer.

##### struct serial_operations
```
struct serial_operations {
    int        (*open)  (serial_handle_t *serial_hdl);
    uint8_t *  (*read)  (const serial_handle_t * serial_hdl, uint16_t * rlen);
    int        (*write) (const serial_handle_t * serial_hdl, uint8_t * wbuffer, const uint16_t wlen);
    int        (*close) (serial_handle_t * serial_hdl);
};
```
- This structure defines hook functions that are defined by virtual serial driver
	- __int (*open)  (serial_handle_t *serial_hdl)__
		- Prepares and opens specified serial interface for communication.
		- Function parameters:
			> serial_hdl: Non null handle for virtual serial interface
		- Return value
			> On success, 0  
			> On failure, -1
	- __uint8_t *(*read)  (const serial_handle_t * serial_hdl, uint16_t * rlen)__
		- This function performs non blocking read operation over specified serial interface handle.
		- Function parameters:
			> serial_hdl: Non null handle for virtual serial interface  
			> rlen: output parameter, indicates number of bytes read
		- Return value:
			> On success, pointer to received buffer  
			> On failure [and when no data is available to read], NULL
	- __int (*write) (const serial_handle_t * serial_hdl, uint8_t * wbuffer, const uint16_t wlen)__
		- This function performs write operation on specified serial interface handle.
		- Function parameters:
			> serial_hdl: Non null handle for virtual serial interface  
			> wbuffer: data buffer to be written over serial interface  
			> wlen: length of data to be written in bytes
		- Return value
			> On success, 0  
			> On failure, -1
	- __int (*close) (serial_handle_t * serial_hdl)__
		- Closes specified serial interface handle.
		- Function parameters:
			> serial_hdl: Non null handle for virtual serial interface
		- Return value
			> On success, 0  
			> On failure, -1

### APIs

##### serial_handle_t * serial_init(void(*rx_data_ind)(void))
- This API can be used by higher layer to setup a serial interface. A serial interface handle is returned in response. This handle can be used to perform further operations on specified serial interface.

- Function parameters:
	> void(*rx_data_ind)(void): Call back function in higher layer. This is used to indicate availibility of Rx data to higher layer. Higher layer can perform read operation to get this data.

- Return value:
	> On success, pointer to a valid serial interface handle  
	> On failure, NULL

## Network interface layer
This section explains network interface APIs and data structures provided by ESP Host software.

### Data structures

##### struct netdev
- This is a opaque structure which network stack should define.
- This denotes instance of a network device in a network stack.
- Along with network stack specific data, it should also hold private structure of underlying driver.

##### struct pbuf
```
struct pbuf{
	/* Data buffer */
	uint8_t *payload;
	/* Length of data buffer */
	uint16_t len;
};
```
- This is a basic structure that holds a network packet which is being transmitted/received.
- It contains following fields.
	> payload: This points to network payload buffer which is being transmitted/received  
	> len: This represents length of payload field in bytes

##### struct netdev_ops
```
struct netdev_ops {
	/* Open device */
	int (* netdev_open) (netdev_handle_t netdev);
	/* Close device */
	int (* netdev_close) (netdev_handle_t netdev);
	/* Transmit packet */
	int (* netdev_xmit) (netdev_handle_t netdev, struct pbuf *net_buf);
};
```
- This structure represents management hooks which are implemented by underlying driver [which is SPI host driver in this case].
	- __int (* netdev_open) (struct netdev *)__
		- This function prepares underlying driver for data transmission/reception on specified network device instance.
		- Function parameters:
			> netdev: Non null network device instance
		- Return value:
			> On success: 0  
			> On failure: -1
	- __int (* netdev_close) (struct netdev *)__
		- Network stack calls this function while closing a network device instance.
		- Function parameters:
			> netdev: Non null network device instance
		- Return value:
			> On success: 0  
			> On failure: -1
	- __int (* netdev_xmit) (struct netdev *, struct pbuf *)__
		- Network stack calls this function to transfer a data packet from network stack to underlying SPI host driver.
		- Function parameters:
			> netdev: Non null network device instance  
			> pbuf: Non null buffer pointer. This will be freed by SPI host driver.
		- Return value:
			> On success: 0  
			> On failure: -1

### APIs
Below are the APIs that are to be implemented by network stack.

##### struct netdev * netdev_alloc(uint32_t sizeof_priv, char *name
- This function allocates and initializes a network device instance for a given network interface name. This function also allocates memory for storing SPI host driver's priv instance.
- Function parameters:
	> sizeof_priv: This denotes size [in bytes] of a priv instance of SPI host driver.  
	> name: Name of network interface for which a network device instance to be allocated
- Return value:
	> On success, pointer to the struct netdev  
	> On failure, NULL

##### void netdev_free(struct netdev *ndev)
- This function frees network device instance. This also frees memory allocated for storing priv instance of SPI host driver.
- Function parameters:
	> ndev: Non null network device instance to be freed
- Return value:
	> None

##### void * netdev_get_priv(struct netdev *ndev)
- This function returns pointer to the SPI host driver's priv instance from the specified network device instance
- Function parameter:
	> ndev: Non null network device instance
- Return value:
	> On success, pointer to the priv instance of SPI host driver  
	> On failure, NULL

##### int netdev_register(struct netdev *ndev, struct netdev_ops *ops)
- This function registers a network device with network stack.
- Function parameters:
	> ndev: Non null network device instance to be registered  
	> ops: management hooks implemented by SPI host driver
- Return value:
	> On success, 0  
	> On failure, -1

##### int netdev_unregister(struct netdev *ndev)
- This function unregisters a network device
- Function parameters:
	> ndev: Non null network device instance to be unregistered
- Return value:
	> On success, 0  
	> On failure, -1


##### int netdev_rx(struct netdev *ndev, struct pbuf *net_buf)
- SPI host driver calls this function to pass on received network packet to network stack
- Function parameters:
	> ndev: Non null network device instance for which a packet has been received  
	> net_buf: Non null buffer that contains received network packet. The SPI host driver will allocate this buffer. Network stack is supposed to free this buffer.
- Return value:
	> On success, 0  
	> On failure, -1
