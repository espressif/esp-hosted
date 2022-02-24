# System Architecture: Linux Based Host
Below diagram depicts building blocks of Linux based ESP-Hosted solution.

![ESP-Hosted linux based design](./linux_hosted_design.png)


## 1. ESP Host Software

This implements ESP-Hosted solution part that runs on Linux host. It mainly consists of following.
* ESP Host Driver
* Control/command interface
* Python convenience scripts
  
---

### 1.1 ESP Host Driver

ESP Host driver implements following.  

* **SDIO Host driver**  
This implements data path over SDIO interface. Communication protocol is explained in further section.

* **SPI Host driver**  
This implements data path over SPI interface. Communication protocol is explained in further section.

* **Virtual Serial interface driver**  
This implements virtual serial interface over SDIO/SPI interface. This virtual serial interface is used as a control interface to configure Wi-Fi of ESP peripheral

* **802.3 network interface**  
This registers two network interfaces with Linux kernel: ethsta0 and ethap0. This allows exchange of 802.3 frames between Linux kernel and ESP firmware.

* **HCI interface**  
This registers HCI interface with Linux kernel. This interface is implemented over SDIO/SPI.

  
---

### 1.2 Control/Command Interface

* This implements custom control commands that are based on protobuf.
* These commands are used to control and configure Wi-Fi on ESP peripheral.
* Control interface makes use of virtual serial interface provided by ESP Host driver.
* There are 2 flavors of control interface implementation:
	* C based implementation
	* Python based implementation - It uses C based implementation using `ctypes` package.
* API's are described in subsequent section
* Control path design and implemetation details explained in [Control Path](../common/contrl_path.md) documentation
