/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include <unistd.h>
#include "test_api.h"

/***** Please Read *****/
/* Before use stress.c : User must enter user configuration parameter in "test_config.h" file */

int main(int argc, char *argv[])
{
    /* Below APIs could be used by demo application */
    char* num = argv[1];
    int ret = 0, stress_test_count = 0;
    ret = control_path_platform_init();
    if (ret != SUCCESS) {
        printf("EXIT!!!!\n");
        exit(0);
    }

    stress_test_count = atoi(num);
    printf("stoi %d\n", stress_test_count);
    for (int i=2; i<argc; i++) {
        if (0 == strncasecmp(STA_CONNECT, argv[i], sizeof(STA_CONNECT))) {
            for (int j=0; j<stress_test_count; j++) {
                test_station_mode_connect();
            }
        }
        if (0 == strncasecmp(STA_DISCONNECT, argv[i], sizeof(STA_DISCONNECT))) {
            for (int j=0; j<stress_test_count; j++) {
                test_station_mode_disconnect();
            }
        }
        if (0 == strncasecmp(AP_START, argv[i], sizeof(AP_START))) {
            for (int j=0; j<stress_test_count; j++) {
                test_softap_mode_start();
            }
        }
        if (0 == strncasecmp(AP_STOP, argv[i], sizeof(AP_STOP))) {
            for (int j=0; j<stress_test_count; j++) {
                test_softap_mode_stop();
            }
        }
        if (0 == strncasecmp(SCAN, argv[i], sizeof(SCAN))) {
            for (int j=0; j<stress_test_count; j++) {
                test_get_available_wifi();
            }
        }
        if (0 == strncasecmp(STA_LIST, argv[i], sizeof(STA_LIST))) {
            for (int j=0; j<stress_test_count; j++) {
                test_softap_mode_connected_clients_info();
            }
        }
    }

    return 0;
}
