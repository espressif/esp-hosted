# Setup for building ESP-Hosted-FG on Windows 11

Follow these steps to setup ESP-IDF and to compile ESP-Hosted-FG using the
ESP-IDF configured Windows Powershell Command Line tool.

1. Install and setup ESP-IDF on Windows as documented in the [Standard Setup of Toolchain for
Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).

2. Use the ESP-IDF [Powershell Command
Prompt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#using-the-command-prompt) to execute `setup-idf.ps1` in the `esp_hosted_fg/esp/esp_driver`
directory. It will setup ESP-IDF in the `esp-idf` directory to be used by
`network_adapter`. Run `setup-idf.ps1 -h` for supported options. To
remove the ESP-IDF installed by `setup-idf.ps1`, you can run the
`remove-idf.ps1` script.

3. Setup ESP-IDF environment by running `export.ps1` in `esp-idf`
directory

4. In the `network_adapter` directory of this project, input command
`idf.py set-target <chip_name>` to set target.

5. Use `idf.py build` to recompile `network_adapter` and generate new
firmware.

6. Use `idf.py flash` to flash the firmware.

7. Use `idf.py monitor` to monitor the serial out. You can combine
these two steps (flash and monitor) by running `idf.py flash monitor`.
