/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

/* Register the eh_host_* API-exerciser console commands. Each command marshals
 * argv -> one eh_host_* call and prints a single uniform result line
 *   EH rc=<int> cmd=<name> [k=v ...]
 * so a test scenario is data (console commands + expected lines), not firmware. */
void eh_api_cmd_register_all(void);
