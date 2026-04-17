# System Architecture: MCU Based Host
Following diagram depicts building blocks of MCU based ESP-Hosted solution.

![ESP-Hosted MCU based design](./MCU_based_design.png).


## 1. ESP Host Software
The host software mainly consists of following building blocks.

### 1.1 SPI/SDIO Host Driver

* ESP-Hosted solution provides thin SPI/SDIO host interface layer which transmits/receives data from SPI/SDIO hardware driver and makes it available to serial or network interface
* Asynchrounous in nature, higher layers have flexibility to transmit and/or receive data as needed
* Currently, Maximum bytes of data trasmitted in single transmit or receive transaction is as follows:
	* SPI - 1600 bytes
	* SDIO - 4096 bytes
  
---

### 1.2 Virtual serial interface driver  

* ESP-Hosted solution provides a generic virtual serial interface implementation.
* Control interface component of ESP-Hosted solution is built on top of this interface.
* Similarly, HCI interface can be built on top of virtual serial interface. This HCI interface can provide BT/BLE functionality to the host.  
`Note: Implementation of HCI interface and BT/BLE stack is not in scope of this project`
  
---

### 1.3 Control/Command Interface  

* As mentioned above, this interface is implemented over virtual serial interface.
* This interface is used for sending control commands to control and configure Wi-Fi functionality of attached ESP peripheral.
* This is an optional interface and in case virtual serial interface is not used, the control path or BT functionality can be used on physical UART interface connected to ESP peripheral.
* Control path design and implemetation details explained in [Control Path](../common/contrl_path.md) documentation
  
---

### 1.4 Network interface layer [netif]  

* This is an abstraction layer between SDIO/SPI host driver and a network stack.
* This gives flexibility of using any network stack with ESP-Hosted solution.
* This interface layer defines set of APIs and data structure that network stack must implement in order to make it work with SDIO/SPI host driver.
  
---

### 1.5 Network stack stub  

* This is a simple network stack stub which demonstrates how a network stack can implement network interface layer and work with SDIO/SPI host driver.
* This does not represent actual network stack. This should be used as a reference for developing network interface layer [netif] provided by ESP-Hosted solution
  
---

### 1.6 Demo application  

* This application demonstrates capabilities of ESP-Hosted solution.
* It makes use of control interface to control and configure Wi-Fi interface of attached ESP peripheral.
* It makes use of network interface to send and receive data over Wi-Fi interface of attached ESP peripheral.
* As demonstration of working data path, 
	* This application sends ARP request to other network devices connected over same network
	* This application also processes and responds to received ARP request packets from other network devices

