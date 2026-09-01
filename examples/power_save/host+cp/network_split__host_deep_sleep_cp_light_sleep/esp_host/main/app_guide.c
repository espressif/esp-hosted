/* SPDX-License-Identifier: Apache-2.0 */
/* `guide` and `demo`: printed walkthroughs. Neither touches the radio. */

#include <stdio.h>
#include <string.h>

#include "esp_console.h"

#include "app_wifi.h"

#define B  "\033[1m"      /* mandatory */
#define O  "\033[36m"     /* optional  */
#define R  "\033[0m"
#define D  "\033[2m"      /* mimicked output */

static void box_top(const char *label)
{
    printf("\n  ┌─[ %s ]", label);
    for (size_t i = strlen(label) + 6; i < 66; i++) {
        printf("─");
    }
    printf("┐\n");
}

static void box_bottom(void)
{
    printf("  └");
    for (int i = 0; i < 66; i++) {
        printf("─");
    }
    printf("┘\n\n");
}

static void row(const char *s) { printf("  │ %-64s │\n", s); }
static void raw(const char *s) { printf("  │ %s\n", s); }

static int cmd_guide(int argc, char **argv)
{
    box_top("guide");
    row("");
    raw(B "  1. connect" R "        sta " B "<ssid>" R " " O "[password]" R);
    raw(D "                     stored in NVS, so it survives a wake" R);
    row("");
    raw(B "  2. check" R "          ip");
    row("");
    raw(B "  3. sleep" R "          host_power_save");
    raw(D "                     host powers down; the coprocessor keeps the" R);
    raw(D "                     link and wakes it on incoming traffic" R);
    row("");
    raw(B "  4. wake" R "           send anything to the host's address");
    raw(D "                     on wake the host adopts the coprocessor's" R);
    raw(D "                     link — no reconnect, no teardown" R);
    row("");
    raw("  " B "bold" R " = required   " O "cyan" R " = optional");
    raw("  demo --2g / demo --5g   the same sequence, printed only");
    row("");
    box_bottom();
    return 0;
}

static void demo_band(const char *band, const char *ssid, const char *ch)
{
    char line[96];

    box_top(band);
    row("");
    raw(D "  (printed sequence — nothing is sent to the radio)" R);
    row("");
    snprintf(line, sizeof(line), B "host> sta %s secret123" R, ssid);
    raw(line);
    snprintf(line, sizeof(line), D "  app_wifi: connecting to \"%s\" (listen_interval 6)" R, ssid);
    raw(line);
    snprintf(line, sizeof(line), D "  app_wifi: IP_EVENT_STA_GOT_IP 192.168.4.2  (ch %s)" R, ch);
    raw(line);
    row("");
    raw(B "host> ip" R);
    raw(D "  ip 192.168.4.2  mask 255.255.255.0  gw 192.168.4.1" R);
    row("");
    raw(B "host> host_power_save" R);
    raw(D "  eh_ps_cli: entering power-save (type=deep)" R);
    raw(D "  cp:  Light sleep ENABLED" R);
    row("");
    raw(D "  ... traffic arrives for the host ..." R);
    raw(D "  cp:  Wakeup needed, reason sta tx msg" R);
    raw(D "  host: Host woke up from power save" R);
    snprintf(line, sizeof(line), D "  app_wifi: adopting the coprocessor's link: \"%s\"" R, ssid);
    raw(line);
    row("");
    raw(D "  no set_config, no connect, no network DOWN" R);
    row("");
    box_bottom();
}

static int cmd_demo(int argc, char **argv)
{
    bool two = false, five = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--2g")) { two = true; }
        else if (!strcmp(argv[i], "--5g")) { five = true; }
    }
    if (!two && !five) {
        printf("usage: demo --2g | --5g\n");
        return 1;
    }
    if (two)  { demo_band("demo 2.4 GHz", "myssid-2g", "6"); }
    if (five) { demo_band("demo 5 GHz",   "myssid-5g", "36"); }
    return 0;
}

void app_guide_register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "guide", .help = "how to use this example, step by step", .func = cmd_guide },
        { .command = "demo",  .help = "printed walkthrough: demo --2g | --5g", .func = cmd_demo },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}
