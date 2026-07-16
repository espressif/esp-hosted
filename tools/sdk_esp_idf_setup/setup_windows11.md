# Setup ESP-Hosted + ESP-IDF on Windows 11

This page is **setup only**. After finishing it, you should be able to:

```powershell
cd <some example>
eh.py menuconfig
eh.py build
```

No extra setup step should be needed inside the example.

## Prerequisites

1. Install **Git** and ensure it is available in `PATH`.
2. Install **ESP-IDF tools for Windows** by following Espressif's standard guide:
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html

## One-Time ESP-IDF Checkout For ESP-Hosted

Open the **ESP-IDF PowerShell** prompt, then run:

```powershell
cd <esp_hosted_repo>\tools\sdk_esp_idf_setup
.\setup-idf.ps1
```

Options:

- `.\setup-idf.ps1 -h` shows supported options
- `.\remove-idf.ps1` removes the local ESP-IDF checkout created by this flow

## Per-Shell Environment Setup

Each new PowerShell session needs both:

- the ESP-IDF environment, so `idf.py` works
- the ESP-Hosted repo environment, so `eh.py` works

From the ESP-Hosted repo root:

```powershell
$env:EH_PATH = (Get-Location).Path
$env:IDF_PATH = Join-Path $env:EH_PATH "tools\sdk_esp_idf_setup\esp-idf"
& "$env:IDF_PATH\export.ps1"
$env:Path += ";$env:EH_PATH\tools"
```

After that, verify:

```powershell
idf.py --version
eh.py --help
```

## What This Does

- `IDF_PATH` points at the ESP-IDF checkout managed by `setup-idf.ps1`
- `export.ps1` sets up the ESP-IDF toolchain for the current shell
- `EH_PATH` points at the ESP-Hosted repo root
- adding `$env:EH_PATH\tools` to `PATH` makes the repo-local `eh.py` command available

## Using Examples

Once the environment above is active, usage should be the same in every example:

```powershell
cd <example directory>
eh.py set-target <target>
eh.py menuconfig
eh.py build
```

For flashing or monitoring:

```powershell
eh.py flash
eh.py monitor
eh.py flash monitor
```

## Notes

- `eh.py` is **not** installed by ESP-IDF. It is provided by this repo at `tools/eh.py`.
- If `eh.py` is not found, the usual cause is that `$env:EH_PATH\tools` was not added to `PATH` in the current PowerShell session.
- If `idf.py` is not found, the usual cause is that `export.ps1` was not run in the current PowerShell session.

## Additional Resources

- ESP-IDF Programming Guide:
  https://docs.espressif.com/projects/esp-idf/en/latest/
- ESP-Hosted example documentation:
  [docs/getting-started-mcu.md](../../docs/getting-started-mcu.md)
