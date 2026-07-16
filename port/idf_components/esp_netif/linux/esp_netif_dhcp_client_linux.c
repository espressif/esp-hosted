/* SPDX-License-Identifier: Apache-2.0 */
/* Linux DHCP client multiplexer: dhcpcd/dhclient/udhcpc. Needs CAP_NET_ADMIN + CAP_NET_RAW. */
/* Fork-and-forget; auto-probe on AUTO; per-iface pidfile; constructor-registered hooks. */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <net/if.h>
#include <pthread.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_linux.h"
#include "sdkconfig.h"

static const char *TAG = "dhcpc_linux";

#ifndef CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR
#define CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR  "/run"
#endif

typedef enum {
    BACKEND_NONE     = 0,
    BACKEND_DHCPCD,
    BACKEND_DHCLIENT,
    BACKEND_UDHCPC,
} backend_id_t;

typedef struct {
    backend_id_t id;
    const char  *exe;
    const char  *name;
} backend_desc_t;

static const backend_desc_t k_backends[] = {
    { BACKEND_DHCPCD,   "/usr/sbin/dhcpcd",      "dhcpcd"   },
    { BACKEND_DHCLIENT, "/usr/sbin/dhclient",    "dhclient" },
    { BACKEND_UDHCPC,   "/usr/sbin/udhcpc",      "udhcpc"   },
    { BACKEND_NONE,     NULL,                    "none"     },
};

static backend_id_t resolve_backend(void)
{
#if defined(CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_DHCPCD)
    return BACKEND_DHCPCD;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_DHCLIENT)
    return BACKEND_DHCLIENT;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_UDHCPC)
    return BACKEND_UDHCPC;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_NONE) \
   || defined(CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_EXTERNAL)
    return BACKEND_NONE;
#else  /* AUTO */
    for (size_t i = 0; k_backends[i].id != BACKEND_NONE; i++) {
        /* dhclient in /sbin on Debian — check both */
        if (access(k_backends[i].exe, X_OK) == 0)
            return k_backends[i].id;
        if (k_backends[i].id == BACKEND_DHCLIENT &&
            access("/sbin/dhclient", X_OK) == 0)
            return BACKEND_DHCLIENT;
    }
    return BACKEND_NONE;
#endif
}

static const backend_desc_t *backend_get(backend_id_t id)
{
    for (size_t i = 0; k_backends[i].id != BACKEND_NONE; i++)
        if (k_backends[i].id == id) return &k_backends[i];
    return NULL;
}

typedef struct dhcpc_inst {
    char         ifname[IFNAMSIZ];
    pid_t        pid;
    backend_id_t backend;
    char         pidfile[PATH_MAX];
    struct dhcpc_inst *next;
} dhcpc_inst_t;

static dhcpc_inst_t   *s_list;
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;

static dhcpc_inst_t *find_or_alloc(const char *ifname)
{
    for (dhcpc_inst_t *i = s_list; i; i = i->next)
        if (strcmp(i->ifname, ifname) == 0) return i;

    dhcpc_inst_t *i = calloc(1, sizeof(*i));
    if (!i) return NULL;
    strncpy(i->ifname, ifname, sizeof(i->ifname) - 1);
    snprintf(i->pidfile, sizeof(i->pidfile),
             "%s/eh_host_dhcpc_%s.pid",
             CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR, ifname);
    i->next = s_list;
    s_list  = i;
    return i;
}

static pid_t read_pidfile(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    long p = -1;
    if (fscanf(f, "%ld", &p) != 1) p = -1;
    fclose(f);
    return (p > 0) ? (pid_t)p : -1;
}

static bool pid_alive(pid_t p)
{
    return p > 0 && kill(p, 0) == 0;
}

static void kill_stale(const char *pidfile)
{
    pid_t p = read_pidfile(pidfile);
    if (p > 0 && pid_alive(p)) {
        ESP_LOGW(TAG, "stale DHCP-client pid %d from %s; SIGTERM", (int)p, pidfile);
        kill(p, SIGTERM);
        for (int i = 0; i < 20 && pid_alive(p); i++) usleep(50 * 1000);
        if (pid_alive(p)) {
            ESP_LOGW(TAG, "SIGTERM ignored; SIGKILL pid %d", (int)p);
            kill(p, SIGKILL);
        }
    }
    unlink(pidfile);
}

/* dhcpcd: --nobackground for in-flight control; own pidfile at /run/dhcpcd-<iface>.pid */
static int dhcpcd_argv(dhcpc_inst_t *inst, char *argv[8])
{
    int i = 0;
    argv[i++] = "/usr/sbin/dhcpcd";
    argv[i++] = "--nobackground";
    argv[i++] = "--quiet";
    argv[i++] = (char *)inst->ifname;
    argv[i++] = NULL;
    return i;
}

/* dhclient: -d foreground, -pf <pidfile>. /usr/sbin (most distros) or /sbin (Debian). */
static int dhclient_argv(dhcpc_inst_t *inst, char *argv[8])
{
    int i = 0;
    argv[i++] = (access("/usr/sbin/dhclient", X_OK) == 0)
                  ? "/usr/sbin/dhclient" : "/sbin/dhclient";
    argv[i++] = "-d";
    argv[i++] = "-pf"; argv[i++] = inst->pidfile;
    argv[i++] = (char *)inst->ifname;
    argv[i++] = NULL;
    return i;
}

/* udhcpc: -f foreground, -i <iface>, -p <pidfile> */
static int udhcpc_argv(dhcpc_inst_t *inst, char *argv[10])
{
    int i = 0;
    argv[i++] = "/usr/sbin/udhcpc";
    argv[i++] = "-f";
    argv[i++] = "-i"; argv[i++] = (char *)inst->ifname;
    argv[i++] = "-p"; argv[i++] = inst->pidfile;
    argv[i++] = NULL;
    return i;
}

static esp_err_t spawn_backend(dhcpc_inst_t *inst)
{
    const backend_desc_t *be = backend_get(inst->backend);
    if (!be) return ESP_ERR_NOT_FOUND;

    char *argv[12] = { NULL };
    switch (inst->backend) {
    case BACKEND_DHCPCD:   dhcpcd_argv  (inst, argv); break;
    case BACKEND_DHCLIENT: dhclient_argv(inst, argv); break;
    case BACKEND_UDHCPC:   udhcpc_argv  (inst, argv); break;
    default: return ESP_ERR_NOT_SUPPORTED;
    }

    pid_t pid = fork();
    if (pid < 0) {
        ESP_LOGE(TAG, "fork: %s", strerror(errno));
        return ESP_FAIL;
    }
    if (pid == 0) {
        setpgid(0, 0);
        int dn = open("/dev/null", O_RDWR | O_CLOEXEC);
        if (dn >= 0) {
            dup2(dn, 0); dup2(dn, 1); dup2(dn, 2);
            if (dn > 2) close(dn);
        }
        execv(argv[0], argv);
        _exit(127);
    }
    inst->pid = pid;
    ESP_LOGI(TAG, "spawned %s on %s (pid %d)", be->name, inst->ifname, (int)pid);
    return ESP_OK;
}

/* External entry points called directly from esp_netif_linux.c's
 * esp_netif_dhcpc_start() / _stop().  Matches IDF's esp_netif
 * convention of no constructor-time hook registration — the linker
 * pulls this TU in because of these symbol references, no sentinel
 * or `--undefined=` trick needed. */

esp_err_t eh_dhcpc_start_linux(const char *ifname)
{
    if (!ifname || !*ifname) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    dhcpc_inst_t *inst = find_or_alloc(ifname);
    if (!inst) { pthread_mutex_unlock(&s_lock); return ESP_ERR_NO_MEM; }

    if (inst->pid > 0 && pid_alive(inst->pid)) {
        pthread_mutex_unlock(&s_lock);
        return ESP_OK;
    }

    inst->backend = resolve_backend();
    if (inst->backend == BACKEND_NONE) {
        ESP_LOGE(TAG, "no DHCP client backend available "
                       "(checked dhcpcd, dhclient, udhcpc); install one "
                       "or set CONFIG_ESP_NETIF_LINUX_DHCP_CLIENT_BACKEND_*");
        pthread_mutex_unlock(&s_lock);
        return ESP_ERR_NOT_FOUND;
    }

    kill_stale(inst->pidfile);
    const backend_desc_t *bd = backend_get(inst->backend);
    ESP_LOGI(TAG, "starting DHCP client on %s: backend=%s (%s)", ifname,
             bd ? bd->name : "?", bd ? bd->exe : "?");
    esp_err_t rc = spawn_backend(inst);
    if (rc != ESP_OK)
        ESP_LOGE(TAG, "DHCP client spawn failed on %s: %s", ifname, esp_err_to_name(rc));
    pthread_mutex_unlock(&s_lock);
    return rc;
}

esp_err_t eh_dhcpc_stop_linux(const char *ifname)
{
    if (!ifname || !*ifname) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    dhcpc_inst_t *inst = NULL;
    for (dhcpc_inst_t *i = s_list; i; i = i->next)
        if (strcmp(i->ifname, ifname) == 0) { inst = i; break; }
    if (!inst) { pthread_mutex_unlock(&s_lock); return ESP_OK; }

    if (inst->pid > 0) {
        kill(inst->pid, SIGTERM);
        for (int i = 0; i < 20 && pid_alive(inst->pid); i++) usleep(50 * 1000);
        if (pid_alive(inst->pid)) kill(inst->pid, SIGKILL);
        waitpid(inst->pid, NULL, WNOHANG);
        inst->pid = 0;
    }
    unlink(inst->pidfile);
    ESP_LOGI(TAG, "DHCP client stopped on %s", ifname);
    pthread_mutex_unlock(&s_lock);
    return ESP_OK;
}
