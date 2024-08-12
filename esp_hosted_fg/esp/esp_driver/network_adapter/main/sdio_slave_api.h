// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef __SDIO_SLAVE_API_H
#define __SDIO_SLAVE_API_H

#ifdef CONFIG_IDF_TARGET_ESP32
#elif defined CONFIG_IDF_TARGET_ESP32S2
    #error "SDIO is not supported for ESP32S2. Please use SPI"
#endif

#define BUFFER_NUM      	10

#endif
