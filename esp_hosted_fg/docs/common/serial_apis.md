# Serial APIs
- Serial APIs are split into `virtual serial interface APIs` and `serial driver APIs`
- Demo app of control path makes use of **Virtual Serial Interface**, [esp_hosted_fg/host/virtual_serial_if/include/serial_if.h](../../host/virtual_serial_if/include/serial_if.h)
- Virtual serial interface makes use of serial driver
- Serial driver is platform specific port which actually implements serial driver specific to that platform \
  For example, Linux uses `/dev/esps0` as serial driver file
- Serial driver for MPU *.i.e.* Linux based host is at **Serial Driver for Linux**, [esp_hosted_fg/host/linux/port/include/platform_wrapper.h](../../host/linux/port/include/platform_wrapper.h)

# 1. Virtual Serial Interface APIs

### 1.1 `int transport_pserial_open(void)`
Open the virtual serial interface

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---

### 1.2 `uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length)`
- The data is expected to be written over virtual serial interface, need to be encoded first in form of Type, Length and Value tuple
- In TLV i.e. Type Length Value format, to transfer data between host and ESP

```
| type | length | value |
```

- `type`
  - 0x01 : for endpoint name
  - 0x02 : for data
- `length`
  - respective value field's data length in 16 bits
- `value`
  - actual data to be transferred

#### Parameters
- `data`
  - Data to be formatted into TLV format
- `data_length`
  - size of `data`

#### Output Parameters
- `buf`
  - TLV formatted buffer

#### Returns
- `uint16_t`
  - size of `buf` which is TLV formatted

---

### 1.3 `int transport_pserial_send(uint8_t* data, uint16_t data_length)`
Send buffer with length as argument on transport as serial interface type

#### Parameters
- `data`
  - Data to be written
- `data_length`
  - Number of bytes to be written

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---

### 1.4 `uint8_t * transport_pserial_read(uint32_t *out_nbyte)`
Read using serial driver and return protobuf encoded buffer

#### Parameters
- `uint32_t *out_nbyte`
  - Number of bytes read on virtual serial interface

#### Returns
- `uint8_t *`
  - Pointer to data read

---

### 1.5 `uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len)`
- Parse the protobuf encoded data in format of tag, length and value
- This will help application to decode protobuf payload and payload length

#### Parameters
- `uint8_t* data`
  - Data buffer read in `serial_drv_read()` operation

#### Output Parameters
- `uint32_t* pro_len`
  - There are two TLVs transferred for serial data
  - After parsing first TLV, parse function understands the length in next TLV, this value is protobuf encoded msg length

#### Returns
- 0 : SUCCESS
- -1 : FAILURE if data is not in TLV format

---

### 1.6 `int transport_pserial_close(void)`
- Close the virtual serial interface

#### Returns
- 0 : SUCCESS
- -1 : FAILURE

---

# 2. Serial Driver APIs

### 2.1 `struct serial_drv_handle_t* serial_drv_open (const char* transport)`
Opens character driver interface on `/dev/esps0`

#### Parameters
- `transport`
  - character driver file name. for example, `/dev/esps0`

#### Returns
- `struct serial_drv_handle_t*`
Serial driver structure pointer for open serial driver instance
- NULL
  - If open failed

---

### 2.2 `int control_path_platform_init(void)`
Initialises valid serial driver instance, flushes any stale data in serial driver

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---

### 2.3 `int serial_drv_write (struct serial_drv_handle_t* serial_drv_handle, uint8_t* buf, int in_count, int* out_count)`
Write to the serial driver. On write, it invokes write method provided by ESP-Hosted's transport like SPI / SDIO

#### Parameters
- `struct serial_drv_handle_t* serial_drv_handle`
  - Serial driver instance
- `uint8_t* buf`
  - Buffer to write from
- `int in_count`
  - Number of bytes to write from `buf`

#### Output Parameters
- `int* out_count`
  - Number of bytes written to serial driver

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---

### 2.4  `uint8_t * serial_drv_read(struct serial_drv_handle_t *serial_drv_handle, int *out_nbyte)`
- Read serial message on serial driver and do two step parsing of TLV
- TLV parsed buffer is returned
- `parse_tlv()` is used to parse TLV (Type, Length, Value)
- Output buffer is still protobuf encoded, caller should do protobuf decoding

#### Parameters
- `struct serial_drv_handle_t* serial_drv_handle`
  - Serial driver instance

#### Output Parameters
- `out_nbyte`
  - Size of returned buffer in bytes

#### Returns
- Output buffer read free from TLV format

---

### 2.5 `int control_path_platform_deinit(void)`
Cleans up the resoiurces used by serial driver. It should be called before before closing serial driver

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---

### 2.6 `int serial_drv_close (struct serial_drv_handle_t** serial_drv_handle)`
Close the opened valid serial driver instance

#### Parameters
- `struct serial_drv_handle_t** serial_drv_handle`
serial driver pointer for open serial driver instance

#### Returns
- 0 : SUCCESS
- !=0 : FAILURE

---
