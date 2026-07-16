# ext_coex · linux_802_3_host/py_app

External-coexistence (PTA) control via the `eh_host` Python ctypes
binding.

## Build

```sh
cd /path/to/esp_hosted
./install.sh      # install.fish for the fish shell
. ./export.sh     # . ./export.fish for the fish shell

cd examples/ext_coex/linux_802_3_host/py_app
eh.py set-target linux
eh.py build
```

`eh.py build` produces `build/eh_libeh_host/libeh_host.so` from the
host stack via CMake — same shape as any other hosted project.

## Run

```sh
../kmod/load.sh --bus spi
eh.py run
../kmod/unload.sh
```

`eh.py run` rebuilds if needed, sets `EH_HOST_LIB` + `PYTHONPATH` for
this build, and execs `main.py`.

## Other commands

```sh
eh.py clean
eh.py fullclean
eh.py menuconfig
```

## Feature

`eh_host.ext_coex` from `host/linux/eh_host_linux_python_ctypes/eh_host/ext_coex.py`.
