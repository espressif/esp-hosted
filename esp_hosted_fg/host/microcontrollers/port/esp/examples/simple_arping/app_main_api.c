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

/** Include **/
#include "trace.h"
#include "app_main_api.h"

/** Exported Constants/Macros **/

/** Exported variables **/

/** Function declarations **/

/** Function definitions **/

/** Exported Functions **/

/**
  * @brief  Print details of arp received
  * @param  stream - input stream
  *         len - len of buffer
  *         custom_str - prepend string with, could be NULL
  * @retval None
  */
void print_stream(uint8_t *stream, int len, char *custom_str)
{
#if DEBUG_STREAM_ENABLED
	uint16_t idx;
	if(custom_str)
		printf("%s -> ",custom_str);
	else
		printf("stream -> ");

	for (idx=0;idx<200;idx++) {
		if (idx%16==0)
			printf("\n%04x: ",idx);
		if (idx%8==0)
			printf(" ");
		printf("%02x",stream[idx]);
	}
	printf("\n");
#endif
}

/**
  * @brief  set val from stream at offset of len
  * @param  stream - input stream
  *         val - buffer to be set
  *         offset - offset from stream
  *         len - len of buffer
  * @retval None
  */
void stream_set(uint8_t * stream, const void *val, uint8_t offset, uint16_t len)
{
	uint8_t * src = (uint8_t *)val;
	uint16_t idx;

	for (idx=0;idx<len;idx++) {
		stream[offset+idx] = src[idx];
	}
}


/**
  * @brief  get stream ptr from stream at offset of len
  * @param  stream - input stream
  *         offset - offset from stream
  *         len - len of buffer
  * @retval value at offset from stream of len
  */
uint8_t * stream_get(uint8_t * stream, uint8_t offset, uint16_t len)
{
	(void)len;
	assert(stream);

	return (stream+offset);
}


/** Local functions **/
