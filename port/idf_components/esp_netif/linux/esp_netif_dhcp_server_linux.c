/* SPDX-License-Identifier: Apache-2.0 */
/* Linux DHCP server multiplexer: dnsmasq/udhcpd/isc-dhcpd. Needs CAP_NET_BIND_SERVICE + /run. */
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

static const char *TAG = "dhcps_linux";

#ifndef CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR
#define CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR  "/run"
#endif
#ifndef CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_LEASEFILE
#define CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_LEASEFILE  "/run/eh_host_dhcps.leases"
#endif

typedef enum {
    BACKEND_NONE     = 0,
    BACKEND_DNSMASQ,
    BACKEND_UDHCPD,
    BACKEND_ISC_DHCPD,
} backend_id_t;

typedef struct {
    backend_id_t id;
    const char  *exe;
    const char  *name;
} backend_desc_t;

static const backend_desc_t k_backends[] = {
    { BACKEND_DNSMASQ,   "/usr/sbin/dnsmasq", "dnsmasq"   },
    { BACKEND_UDHCPD,    "/usr/sbin/udhcpd",  "udhcpd"    },
    { BACKEND_ISC_DHCPD, "/usr/sbin/dhcpd",   "isc-dhcpd" },
    { BACKEND_NONE,      NULL,                "none"      },
};

static backend_id_t resolve_backend(void)
{
#if defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_DNSMASQ)
    return BACKEND_DNSMASQ;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_UDHCPD)
    return BACKEND_UDHCPD;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_ISC_DHCPD)
    return BACKEND_ISC_DHCPD;
#elif defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_NONE) \
   || defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_EXTERNAL)
    return BACKEND_NONE;
#else  /* AUTO */
    for (size_t i = 0; k_backends[i].id != BACKEND_NONE; i++) {
        if (access(k_backends[i].exe, X_OK) == 0)
            return k_backends[i].id;
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

typedef struct dhcps_inst {
    char         ifname[IFNAMSIZ];
    pid_t        pid;
    backend_id_t backend;
    char         pidfile[PATH_MAX];
    char         conffile[PATH_MAX];   /* udhcpd / isc-dhcpd generated cfg */
    struct dhcps_inst *next;
} dhcps_inst_t;

static dhcps_inst_t   *s_list;
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;

static dhcps_inst_t *find_or_alloc(const char *ifname)
{
    for (dhcps_inst_t *i = s_list; i; i = i->next)
        if (strcmp(i->ifname, ifname) == 0) return i;

    dhcps_inst_t *i = calloc(1, sizeof(*i));
    if (!i) return NULL;
    strncpy(i->ifname, ifname, sizeof(i->ifname) - 1);
    snprintf(i->pidfile, sizeof(i->pidfile),
             "%s/eh_host_dhcps_%s.pid",
             CONFIG_ESP_NETIF_LINUX_DHCP_PIDFILE_DIR, ifname);
    snprintf(i->conffile, sizeof(i->conffile),
             "%s/eh_host_dhcps_%s.conf",
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
        ESP_LOGW(TAG, "stale DHCP-server pid %d from %s; SIGTERM", (int)p, pidfile);
        kill(p, SIGTERM);
        for (int i = 0; i < 20 && pid_alive(p); i++) usleep(50 * 1000);
        if (pid_alive(p)) {
            ESP_LOGW(TAG, "SIGTERM ignored; SIGKILL pid %d", (int)p);
            kill(p, SIGKILL);
        }
    }
    unlink(pidfile);
}

/* dnsmasq: pure CLI, --bind-interfaces scopes to ifname, DNS gated by Kconfig */
static int dnsmasq_argv(dhcps_inst_t *inst,
                        const char *args_buf, size_t buf_sz,
                        char *argv[12])
{
    (void)args_buf; (void)buf_sz;
    int i = 0;
    argv[i++] = "/usr/sbin/dnsmasq";
    argv[i++] = "--keep-in-foreground";
    argv[i++] = "--bind-interfaces";
#if !defined(CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_DNS) || CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_DNS == 0
    argv[i++] = "--port=0";                       /* DNS off — DHCP only */
#else
    argv[i++] = "--no-resolv";
    argv[i++] = "--no-poll";
#endif
    /* soft-AP default subnet 192.168.4.1/24, clients .10-.200, 12h lease */
    static char if_arg[64], range_arg[96], pid_arg[160], lease_arg[160];
    /* %.<N>s precisions satisfy -Wformat-truncation on PATH_MAX-sized strings */
    snprintf(if_arg,    sizeof(if_arg),    "--interface=%.32s", inst->ifname);
    snprintf(range_arg, sizeof(range_arg), "--dhcp-range=192.168.4.10,192.168.4.200,12h");
    snprintf(pid_arg,   sizeof(pid_arg),   "--pid-file=%.140s", inst->pidfile);
    snprintf(lease_arg, sizeof(lease_arg), "--dhcp-leasefile=%.140s",
             CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_LEASEFILE);
    argv[i++] = if_arg;
    argv[i++] = range_arg;
    argv[i++] = pid_arg;
    argv[i++] = lease_arg;
    argv[i++] = NULL;
    return i;
}

/* udhcpd: needs config file (no CLI range) */
static int udhcpd_argv(dhcps_inst_t *inst,
                       char *cfg_buf, size_t cfg_sz,
                       char *argv[6])
{
    snprintf(cfg_buf, cfg_sz,
        "start 192.168.4.10\n"
        "end 192.168.4.200\n"
        "interface %.32s\n"
        "pidfile %.150s\n"
        "lease_file %.150s\n"
        "opt subnet 255.255.255.0\n"
        "opt router 192.168.4.1\n"
        "opt lease 43200\n",
        inst->ifname, inst->pidfile,
        CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_LEASEFILE);

    FILE *f = fopen(inst->conffile, "w");
    if (!f) return -1;
    fputs(cfg_buf, f);
    fclose(f);

    int i = 0;
    argv[i++] = "/usr/sbin/udhcpd";
    argv[i++] = "-f";
    argv[i++] = inst->conffile;
    argv[i++] = NULL;
    return i;
}

/* isc-dhcpd: config-file-driven */
static int isc_dhcpd_argv(dhcps_inst_t *inst,
                          char *cfg_buf, size_t cfg_sz,
                          char *argv[8])
{
    snprintf(cfg_buf, cfg_sz,
        "subnet 192.168.4.0 netmask 255.255.255.0 {\n"
        "  range 192.168.4.10 192.168.4.200;\n"
        "  option routers 192.168.4.1;\n"
        "  default-lease-time 43200;\n"
        "}\n");

    FILE *f = fopen(inst->conffile, "w");
    if (!f) return -1;
    fputs(cfg_buf, f);
    fclose(f);

    int i = 0;
    argv[i++] = "/usr/sbin/dhcpd";
    argv[i++] = "-f";
    argv[i++] = "-pf";  argv[i++] = inst->pidfile;
    argv[i++] = "-cf";  argv[i++] = inst->conffile;
    argv[i++] = inst->ifname;
    argv[i++] = NULL;
    return i;
}

static esp_err_t spawn_backend(dhcps_inst_t *inst)
{
    const backend_desc_t *be = backend_get(inst->backend);
    if (!be) return ESP_ERR_NOT_FOUND;

    char    cfg_buf[512];
    char   *argv[12] = { NULL };

    switch (inst->backend) {
    case BACKEND_DNSMASQ:   if (dnsmasq_argv  (inst, cfg_buf, sizeof(cfg_buf), argv) < 0) return ESP_FAIL; break;
    case BACKEND_UDHCPD:    if (udhcpd_argv   (inst, cfg_buf, sizeof(cfg_buf), argv) < 0) return ESP_FAIL; break;
    case BACKEND_ISC_DHCPD: if (isc_dhcpd_argv(inst, cfg_buf, sizeof(cfg_buf), argv) < 0) return ESP_FAIL; break;
    default: return ESP_ERR_NOT_SUPPORTED;
    }

    pid_t pid = fork();
    if (pid < 0) {
        ESP_LOGE(TAG, "fork: %s", strerror(errno));
        return ESP_FAIL;
    }
    if (pid == 0) {
        /* new process group + closed stdio: shield from app SIGINT and noise */
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
 * esp_netif_dhcps_start() / _stop().  See client-side TU for the
 * rationale (no ctor, no hook table, no sentinel — direct call). */

esp_err_t eh_dhcps_start_linux(const char *ifname)
{
    if (!ifname || !*ifname) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    dhcps_inst_t *inst = find_or_alloc(ifname);
    if (!inst) { pthread_mutex_unlock(&s_lock); return ESP_ERR_NO_MEM; }

    if (inst->pid > 0 && pid_alive(inst->pid)) {
        pthread_mutex_unlock(&s_lock);
        return ESP_OK;
    }

    inst->backend = resolve_backend();
    if (inst->backend == BACKEND_NONE) {
        ESP_LOGE(TAG, "no DHCP server backend available "
                       "(checked dnsmasq, udhcpd, isc-dhcpd); install one "
                       "or set CONFIG_ESP_NETIF_LINUX_DHCP_SERVER_BACKEND_*");
        pthread_mutex_unlock(&s_lock);
        return ESP_ERR_NOT_FOUND;
    }

    kill_stale(inst->pidfile);
    esp_err_t rc = spawn_backend(inst);
    pthread_mutex_unlock(&s_lock);
    return rc;
}

esp_err_t eh_dhcps_stop_linux(const char *ifname)
{
    if (!ifname || !*ifname) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    dhcps_inst_t *inst = NULL;
    for (dhcps_inst_t *i = s_list; i; i = i->next)
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
    if (inst->conffile[0]) unlink(inst->conffile);
    ESP_LOGI(TAG, "DHCP server stopped on %s", ifname);
    pthread_mutex_unlock(&s_lock);
    return ESP_OK;
}
