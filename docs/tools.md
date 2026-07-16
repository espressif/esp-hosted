# Tools: eh.py

[Home](../README.md) · [Getting Started: Linux](getting-started-linux.md) · [Getting Started: MCU](getting-started-mcu.md) · [Troubleshooting](troubleshooting.md)

`eh.py` is the unified command-line entry point for ESP-Hosted — it configures, builds, flashes, and runs both the co-processor firmware (`cp/`) and the host applications from one tool.

## Install and set up the environment

Run the installer once per checkout, then source the environment once per shell so `eh.py` is on your `PATH`:

```sh
cd /path/to/esp_hosted
./install.sh     # or ./install.fish   (fish shell)
. ./export.sh    # or . ./export.fish  (fish shell)
```

- `install.sh` — installs the toolchain and dependencies ESP-Hosted needs (one-time).
- `export.sh` — adds `eh.py` (at `tools/eh.py`) to your `PATH` for the current shell.

> [!NOTE]
> `eh.py set-idf-path <dir>` can be used to re-use a pre-existing ESP-IDF setup.

## Common commands

| Command | What it does |
| :--- | :--- |
| `eh.py set-target <target>` | Select the target (e.g. `esp32c6`, `esp32p4`, `linux`) |
| `eh.py menuconfig` | Open the configuration UI (transport, features, pins) |
| `eh.py build` | Build the current project |
| `eh.py -p <port> flash monitor` | Flash over serial and open the serial monitor |
| `eh.py run` | Run a Linux host application |
| `eh.py test <substrate>` | Run the automated test suites |
| `eh.py patch-idf` | Apply the required SDIO patch to the resolved ESP-IDF |

For parallel emulator runs, `eh.py test emu --jobs N --prewarm` builds firmware before launching emulators. Tests on an unpatched ESP-IDF tell you to run `eh.py patch-idf`; use `eh.py test emu --auto-apply-idf-patches` (or `eh.py test hw --auto-apply-idf-patches`) only to apply it in place automatically.

`eh.py` wraps the underlying ESP-IDF build tooling, so if you already use `idf.py` the commands will feel familiar.

> [!NOTE]
> On Linux, the kernel-module transport driver is built and loaded with `build.sh` / `load.sh` / `unload.sh` inside the example's `linux_802_3_host/kmod/` folder — `eh.py` is used for the co-processor firmware and the user-space apps.
