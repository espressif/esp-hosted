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

/** Includes **/
#include "common.h"
#include "trace.h"
#include "stdlib.h"
#include <errno.h>

/** Constants/Macros **/

/** Exported variables **/


/** Function declaration **/

/** Exported Functions **/
/**
  * @brief  debug buffer print
  * @param  buff - input buffer to print in hex
  *         rx_len - buff len
  *         human_str - helping string to describe about buffer
  * @retval None
  */
#if DEBUG_HEX_STREAM_PRINT
char print_buff[MAX_SPI_BUFFER_SIZE*3];
#endif

uint16_t hton_short (uint16_t x)
{
#if BYTE_ORDER == BIG_ENDIAN
  return x;
#elif BYTE_ORDER == LITTLE_ENDIAN
  uint16_t val = 0;

  val = (x &0x00FF)<<8;
  val |= (x &0xFF00)>>8;

  return val;
#else
# error "not able to identify endianness"
#endif
}

uint32_t hton_long (uint32_t x)
{
#if BYTE_ORDER == BIG_ENDIAN
  return x;
#elif BYTE_ORDER == LITTLE_ENDIAN
    uint32_t val = (x&0xFF000000) >> 24;

    val |= (x&0x00FF0000) >> 8;
    val |= (x&0x0000FF00) << 8;
    val |= (x&0x000000FF) << 24;

    return val;
#else
# error "not able to identify endianness"
#endif
}

/**
  * @brief  dump hex for buffer
  * @param  buff - buff to print
  *         rx_len - size of buf
  *         human_str - help string to prepend
  * @retval None
  */
void print_hex_dump(uint8_t *buff, uint16_t rx_len, char *human_str)
{
#if DEBUG_HEX_STREAM_PRINT
    char * print_ptr = print_buff;
    for (int i = 0; i< rx_len;i++) {
        sprintf(print_ptr, "%02x ",buff[i]);
        print_ptr+=3;
    }
    print_buff[rx_len*3] = '\0';

    if(human_str)
        printf("%s (len: %5u) -> %s\n\r",human_str, rx_len, print_buff);
    else
        printf("(len %5u), Data %s\n\r", rx_len, print_buff);
#else
    UNUSED_VAR(buff);
    UNUSED_VAR(rx_len);
    UNUSED_VAR(human_str);
#endif /* DEBUG_HEX_STREAM_PRINT */
}

/**
  * @brief  Calculate minimum
  * @param  x - number
  *         y - number
  * @retval minimum
  */
int min(int x, int y) {
    return (x < y) ? x : y;
}

/**
  * @brief Delay without context switch
  * @param  None
  * @retval None
  */
void hard_delay(int x)
{
    volatile int idx = 0;
    for (idx=0; idx<100*x; idx++) {
    }
}

/**
  * @brief  get numbers from string
  * @param  val - return integer value,
  *         arg - input string
  * @retval STM_OK on success, else STM_FAIL
  */
int get_num_from_string(int *val, char *arg)
{
  int base = 10;
  char *endptr = NULL, *str = NULL;

  if (!arg || (arg[0]=='\0')) {
    printf("No number Identified \n");
    return STM_FAIL;
  }

  if (!val) {
    printf("No memory allocated \n");
    return STM_FAIL;
  }

  errno = 0;
  str = arg;
  *val = strtol(str, &endptr, base);

  if (endptr == str) {
    printf("No digits found \n");
    *val = 0;
    return STM_FAIL;
  }

  if ((errno == ERANGE) && ((*val == INT32_MAX) || (*val == INT32_MIN))) {
    perror("strtol");
    *val = 0;
    return STM_FAIL;
  }

  return STM_OK;
}

/** Local functions **/
