# ESP-Hosted GPIO Control Example

This example demonstrates how to use the ESP-Hosted solution to control the GPIOs of the slave (co-processor) from the host MCU.

## How to Use

### Feature Switch

Feature can be controlled at Host and Slave using `CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP` (host)
and `CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP` (coprocessor).
It is by default disabled at slave.

**Slave Configuration:**

    *   The firmware flashed on the slave board must have the GPIO RPC feature enabled. In the slave's `eh.py menuconfig`, set the following:

        ```
        (Top) →
            Example Configuration --->
                [*] Enable GPIO Expander support (host can control slave GPIOs)
        ```

        This corresponds to the `CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP=y` option.

### Build and Flash

1. **Slave**
    * Slave build & flashed with above config

2.  **Host:**
    *   Set your target board (e.g., `eh.py set-target esp32p4`).
    *   Build and flash the project: `eh.py -p <host_port> build flash monitor`.

The host application will:

1.  Initialize the ESP-Hosted transport.
2.  Configure a specific GPIO pin on the slave as an output.
3.  Toggle the GPIO level (HIGH and LOW) every second.
4.  After a few toggles, it will reconfigure the same GPIO as an input.
5.  Read and print the GPIO level.

You can connect an LED to the specified GPIO on the slave board to visually verify the output functionality.
