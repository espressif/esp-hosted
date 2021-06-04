# Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

(WIFI_MODE_NONE, WIFI_MODE_STATION,
        WIFI_MODE_SOFTAP, WIFI_MODE_SOFTAP_STATION,
        WIFI_MODE_MAX) = (0, 1, 2, 3, 4)
(WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM,
        WIFI_PS_INVALID) = (1, 2, 3)

failure = "failure"
success = "success"
not_connected = "not_connected"

# station mode
STATION_MODE_MAC_ADDRESS="1a:11:11:11:11:11"
STATION_MODE_SSID="MyWifi"
STATION_MODE_PWD="MyWifiPass@123"
STATION_MODE_BSSID=""
STATION_MODE_IS_WPA3_SUPPORTED=False
STATION_MODE_LISTEN_INTERVAL=5

# softap mode
SOFTAP_MODE_MAC_ADDRESS="1a:22:22:22:22:22"
SOFTAP_MODE_SSID="ESPWifi"
SOFTAP_MODE_PWD="ESPWifi@123"
SOFTAP_MODE_CHANNEL=1
SOFTAP_MODE_ENCRYPTION_MODE=3
SOFTAP_MODE_MAX_ALLOWED_CLIENTS=4
SOFTAP_MODE_SSID_HIDDEN=False
SOFTAP_MODE_BANDWIDTH=2
