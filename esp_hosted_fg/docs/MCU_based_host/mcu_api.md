# APIs for MCU based Host

# 1. Serial interface layer
This section explains serial interface APIs and data structures provided by MCU based ESP Host software.

## 1.1 Data Structures

### _struct_ `serial_handle_s`
This structure holds instance of a virtual serial interface. It contains following fields:

*Public Members*
- `QueueHandle_t queue`:
Buffer queue for storing received data packets
- `uint8_t if_type`:
ESP Hosted Interface type
- `uint8_t if_num`:
ESP Hosted Interface number
- `struct serial_operations *ops`:
Hook functions supported by virtual serial interface
- `uint8_t state`:
State of a handle
- `void (*serial_rx_callback) (void)`:
Rx callback registered by upper layer.

---

### _struct_ `serial_operations`
This structure defines hook functions that are defined by virtual serial driver

*Public Members*
- `int (*open)  (serial_handle_t *serial_hdl)`:
Prepares and opens specified serial interface for communication.
  - Parameters:
    - `serial_hdl`:
Non null handle for virtual serial interface
  - Returns:
    - 0: success
    - -1: failure

- `uint8_t *(*read)  (const serial_handle_t * serial_hdl, uint16_t * rlen)`:
This function performs non blocking read operation over specified serial interface handle.
  - Parameters:
    - `serial_hdl`:
Non null handle for virtual serial interface
    - `rlen`:
Output parameter, indicates number of bytes read
  - Returns:
    - On success, pointer to received buffer
    - On failure [and when no data is available to read], NULL

- `int (*write) (const serial_handle_t * serial_hdl, uint8_t * wbuffer, const uint16_t wlen)`:
This function performs write operation on specified serial interface handle.
  - Parameters:
    - `serial_hdl`:
Non null handle for virtual serial interface
    - `wbuffer`:
Data buffer to be written over serial interface
    - `wlen`:
Length of data to be written in bytes
  - Returns
    - 0: success
    - -1: failure

- `int (*close) (serial_handle_t * serial_hdl)`:
Closes specified serial interface handle.
  - Parameters:
    - `serial_hdl`:
Non null handle for virtual serial interface
  - Returns
    - 0: success
    - -1: failure

---


## 1.2 Functions

### `serial_handle_t * serial_init(void(*rx_data_ind)(void))`
This API can be used by higher layer to setup a serial interface. A serial interface handle is returned in response. This handle can be used to perform further operations on specified serial interface.

#### Parameters
- `void(*rx_data_ind)(void)`:
Call back function in higher layer. This is used to indicate availibility of Rx data to higher layer. Higher layer can perform read operation to get this data.

#### Return
- NULL: failure
- Non NULL: pointer to a valid serial interface handle

---

# 2. Network interface layer
This section explains network interface APIs and data structures provided by ESP Host software.

## 2.1 Data structures

### _struct_ `netdev`
- This is a opaque structure which network stack should define.
- This denotes instance of a network device in a network stack.
- Along with network stack specific data, it should also hold private structure of underlying driver.

---

### _struct_ `pbuf`
This is a basic structure that holds a network packet which is being transmitted or received.

*Public Members*
- `uint8_t *payload`:
This points to network payload buffer which is being transmitted or received
- `uint16_t len`:
This represents length of payload field in bytes

---

### _struct_ `netdev_ops`
This structure represents management hooks which are implemented by underlying driver [which is SPI host driver in this case].

*Public Members*
- `int (* netdev_open) (struct netdev *)`
This function prepares underlying driver for data transmission or reception on specified network device instance.
- Parameters:
    - `netdev`:
Non null network device instance
- Returns:
    - 0: success
    - -1: failure

- `int (* netdev_close) (struct netdev *)`
Network stack calls this function while closing a network device instance.
  - Parameters:
    - `netdev`:
Non null network device instance
  - Returns:
    - 0: success
    - -1: failure

- `int (* netdev_xmit) (struct netdev *, struct pbuf *)`
Network stack calls this function to transfer a data packet from network stack to underlying SPI host driver.
  - Parameters:
    - `netdev`:
Non null network device instance
    - `pbuf`:
Non null buffer pointer. This will be freed by SPI host driver.
  - Returns:
    - 0: success
    - -1: failure

---

## 2.2 Functions
Below are the APIs that are to be implemented by network stack.

### `struct netdev * netdev_alloc(uint32_t sizeof_priv, char *name)`
This function allocates and initializes a network device instance for a given network interface name. This function also allocates memory for storing priv instance of SPI host driver.

#### Parameters
- `sizeof_priv`:
This denotes size [in bytes] of a priv instance of SPI host driver.
- `name`:
Name of network interface for which a network device instance to be allocated

#### Return
- NULL: failure
- Non NULL: Pointer to the struct netdev

---

### `void netdev_free(struct netdev *ndev)`
This function frees network device instance. This also frees memory allocated for storing priv instance of SPI host driver.

#### Parameters
- `ndev`:
Non null network device instance to be freed

---

### `void * netdev_get_priv(struct netdev *ndev)`
This function returns pointer to the priv instance of SPI host driver from the specified network device instance

#### Parameters
- `ndev`:
Non null network device instance

#### Return
- NULL: failure
- Non NULL : pointer to the priv instance of SPI host driver

---

### `int netdev_register(struct netdev *ndev, struct netdev_ops *ops)`
This function registers a network device with network stack.

#### Parameters
- `ndev`:
Non null network device instance to be registered
- `ops`:
Management hooks implemented by SPI host driver

#### Return
- 0: success
- -1: failure

---

### `int netdev_unregister(struct netdev *ndev)`
This function unregisters a network device

#### Parameters
- `ndev`:
Non null network device instance to be unregistered

#### Return
- 0: success
- -1: failure

---


### `int netdev_rx(struct netdev *ndev, struct pbuf *net_buf)`
SPI host driver calls this function to pass on received network packet to network stack

#### Parameters
- `ndev`:
Non null network device instance for which a packet has been received
- `net_buf`:
Non null buffer that contains received network packet. The SPI host driver will allocate this buffer. Network stack is supposed to free this buffer.

#### Return
- 0: success
- -1: failure

---
