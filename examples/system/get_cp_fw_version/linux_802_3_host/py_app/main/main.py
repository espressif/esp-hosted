#!/usr/bin/env python3

import sys
import time

import eh_host
from eh_host import system
from eh_host.log import ESP_LOGI, ESP_LOGE


TAG = "get_cp_fw_version"


def main() -> int:
    eh_host.init_linux_default()
    rc = 0
    try:
        while True:
            try:
                fw = system.get_cp_fw_version()
                ESP_LOGI(TAG, "CP firmware: %u.%u.%u",
                         fw.major1, fw.minor1, fw.patch1)
            except eh_host.EhHostError as err:
                ESP_LOGE(TAG, "get_cp_fw_version failed: %s", err)
                rc = 1
                break
            time.sleep(5)
    except KeyboardInterrupt:
        ESP_LOGI(TAG, "interrupted")
    finally:
        eh_host.deinit()
    return rc


if __name__ == "__main__":
    sys.exit(main())
