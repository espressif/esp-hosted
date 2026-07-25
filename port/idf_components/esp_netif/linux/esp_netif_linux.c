/* SPDX-License-Identifier: Apache-2.0 */
/* esp_netif_linux.c — Linux kmod-state observer (read-only; kmod owns the netdev). */

#ifndef ESP_PLATFORM   /* entire file is Linux-host-only */

#define _GNU_SOURCE             /* pipe2(), O_CLOEXEC                   */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>              /* O_CLOEXEC for pipe2()                */
#include <pthread.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_arp.h>          /* ARPHRD_ETHER for SIOCSIFHWADDR       */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/rtnetlink.h>
#include <linux/netlink.h>
#include <linux/if_addr.h>      /* IFA_ADDRESS, IFA_RTA, RTA_OK helpers */
#include <net/route.h>          /* struct rtentry, SIOCADDRT, RTF_*    */

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_linux.h"
#include "esp_netif_types.h"
#include "esp_netif_net_stack.h"

/* ── Constants ───────────────────────────────────────────────────────── */

#define TAG             "esp_netif"
#define DEFAULT_NETDEV  "wlan0"
#define NL_BUF_SZ       8192   /* large enough for a burst of RTM msgs   */

/* IDF defines NETIF_NAMESIZE as 6 (lwip's NETIF_NAMESIZE).
 * Guard it here so the Linux build does not need to pull lwip headers. */
#ifndef NETIF_NAMESIZE
#define NETIF_NAMESIZE  6
#endif

/* IDF esp_netif error codes — defined in esp_netif_types.h upstream.
 * Guard each one so we don't clash if the port header already has them. */
#ifndef ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED
#define ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED  0x5009
#endif

/* ── Internal netif descriptor ───────────────────────────────────────── */

struct esp_netif_obj {
    /* IDF-visible identity */
    char              if_key[32];
    char              if_desc[32];
    int               route_prio;
    esp_netif_flags_t flags;

    /* Linux binding */
    char              linux_ifname[IFNAMSIZ];
    unsigned int      ifindex;          /* cached once on watcher start   */

    /* Cached state */
    esp_netif_ip_info_t  last_ip;       /* last posted GOT_IP info        */
    esp_netif_dhcp_status_t dhcpc_status;
    esp_netif_dhcp_status_t dhcps_status;
    esp_netif_dns_info_t dns[ESP_NETIF_DNS_MAX];

    /* Watcher thread */
    pthread_t         watcher_tid;
    int               wakeup_pipe[2];   /* [0]=read end, [1]=write end    */
    int               nl_fd;
    bool              watcher_alive;    /* protected by s_lock            */
    volatile bool     running;          /* set false to request stop      */

    /* Interface state */
    bool              up;               /* esp_netif_up() called          */
    bool              ipv6_enabled;     /* set by create_ip6_linklocal    */

    struct esp_netif_obj *next;
};

/* ── Module globals ──────────────────────────────────────────────────── */

static struct esp_netif_obj *s_list   = NULL;
static pthread_mutex_t       s_lock   = PTHREAD_MUTEX_INITIALIZER;
static bool                  s_inited = false;

/* Forward declarations (not in public esp_netif.h; lwip-internal upstream). */
esp_err_t esp_netif_up(esp_netif_t *netif);
esp_err_t esp_netif_down(esp_netif_t *netif);

/* ── Kernel IP reader ────────────────────────────────────────────────── */

static esp_err_t kernel_get_ip_info(const char *ifname,
                                    esp_netif_ip_info_t *out)
{
    memset(out, 0, sizeof(*out));

    /* ── Address + netmask ── */
    struct ifaddrs *list = NULL;
    if (getifaddrs(&list) < 0) {
        ESP_LOGE(TAG, "getifaddrs: %s", strerror(errno));
        return ESP_FAIL;
    }
    for (struct ifaddrs *ifa = list; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        if (strcmp(ifa->ifa_name, ifname) != 0) continue;

        out->ip.addr =
            ((struct sockaddr_in *)ifa->ifa_addr)->sin_addr.s_addr;
        if (ifa->ifa_netmask)
            out->netmask.addr =
                ((struct sockaddr_in *)ifa->ifa_netmask)->sin_addr.s_addr;
        break;
    }
    freeifaddrs(list);

    /* ── Default gateway ── */
    FILE *f = fopen("/proc/net/route", "r");
    if (!f) return ESP_OK;   /* not fatal; gateway stays 0 */

    char     line[256];
    char     iface[IFNAMSIZ];
    uint32_t dest, gw, mask;
    unsigned flags;

    /* skip header */
    if (!fgets(line, sizeof(line), f)) { fclose(f); return ESP_OK; }

    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "%15s %x %x %x %*u %*u %*u %x",
                   iface, &dest, &gw, &flags, &mask) != 5) continue;

        /* RTF_UP (0x1) | RTF_GATEWAY (0x2), destination = 0.0.0.0 */
        if (dest == 0 && (flags & 0x0003u) == 0x0003u &&
            strcmp(iface, ifname) == 0) {
            out->gw.addr = gw;
            break;
        }
    }
    fclose(f);
    return ESP_OK;
}

/* ── Event posting ───────────────────────────────────────────────────── */

static void post_got_ip(esp_netif_t *n)
{
    esp_netif_ip_info_t ip;
    if (kernel_get_ip_info(n->linux_ifname, &ip) != ESP_OK) return;
    if (ip.ip.addr == 0) return;   /* not yet assigned — wait for RTM_NEWADDR */

    /* Atomic compare-and-store under s_lock — symmetric with
     * post_lost_ip() and esp_netif_{get,set}_old_ip_info().  Gating
     * the emit on `changed` is the dedup point: action_connected and
     * the watcher probe can race on first connect (static-IP path);
     * whichever updates last_ip first emits, the other no-ops. */
    pthread_mutex_lock(&s_lock);
    bool was_first  = (n->last_ip.ip.addr == 0);
    bool changed    = (memcmp(&ip, &n->last_ip, sizeof(ip)) != 0);
    if (changed) n->last_ip = ip;
    pthread_mutex_unlock(&s_lock);

    if (!changed) return;

    char buf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &ip.ip.addr, buf, sizeof(buf));
    ESP_LOGI(TAG, "[%s] GOT_IP %s", n->linux_ifname, buf);

    /* ip_changed: IDF semantic is "differs from a previously-known IP".
     * First emit (was_first → previous IP was zero, i.e. unknown) sets
     * false; subsequent renewals with different lease set true. */
    ip_event_got_ip_t evt = {
        .esp_netif  = n,
        .ip_info    = ip,
        .ip_changed = !was_first,
    };
    esp_event_post(IP_EVENT, IP_EVENT_STA_GOT_IP, &evt, sizeof(evt),
                   portMAX_DELAY);
}

/*
 * post_lost_ip: zero the cache (under lock to prevent double-post) then
 * post LOST_IP with a zeroed ip_event_got_ip_t (matches IDF semantics).
 *
 * Returns false if another caller already claimed the post (cache was 0).
 */
static bool post_lost_ip(esp_netif_t *n)
{
    /* Atomic test-and-clear of last_ip under the global lock.
     * Both watcher thread (RTM_DELADDR path) and esp_netif_down() call
     * this; the first one wins, the second returns false. */
    pthread_mutex_lock(&s_lock);
    if (n->last_ip.ip.addr == 0) {
        pthread_mutex_unlock(&s_lock);
        return false;
    }
    memset(&n->last_ip, 0, sizeof(n->last_ip));
    pthread_mutex_unlock(&s_lock);

    ESP_LOGI(TAG, "[%s] LOST_IP", n->linux_ifname);

    ip_event_got_ip_t evt = {
        .esp_netif  = n,
        .ip_info    = { {0}, {0}, {0} },
        .ip_changed = true,
    };
    esp_event_post(IP_EVENT, IP_EVENT_STA_LOST_IP, &evt, sizeof(evt),
                   portMAX_DELAY);
    return true;
}

/*
 * post_got_ip6: extract the inet6 address from an RTM_NEWADDR netlink
 * message and emit IP_EVENT_GOT_IP6. SLAAC, kernel-generated link-local,
 * and manually-assigned v6 addresses all flow through the same handler.
 */
static void post_got_ip6(esp_netif_t *n, struct nlmsghdr *nh)
{
    struct ifaddrmsg *ifa = (struct ifaddrmsg *)NLMSG_DATA(nh);
    int rta_len = (int)(nh->nlmsg_len - NLMSG_LENGTH(sizeof(*ifa)));

    for (struct rtattr *rta = IFA_RTA(ifa);
         RTA_OK(rta, rta_len);
         rta = RTA_NEXT(rta, rta_len)) {
        if (rta->rta_type != IFA_ADDRESS) continue;
        if (RTA_PAYLOAD(rta) != 16) continue;

        ip_event_got_ip6_t evt = { .esp_netif = n, .ip_index = 0 };
        memcpy(evt.ip6_info.ip.addr, RTA_DATA(rta), 16);
        evt.ip6_info.ip.zone = 0;

        char buf[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, evt.ip6_info.ip.addr, buf, sizeof(buf));
        ESP_LOGI(TAG, "[%s] GOT_IP6 %s", n->linux_ifname, buf);

        esp_event_post(IP_EVENT, IP_EVENT_GOT_IP6, &evt, sizeof(evt),
                       portMAX_DELAY);
        return;
    }
}

/* ── Netlink watcher thread ──────────────────────────────────────────── */

static void *watcher_main(void *arg)
{
    esp_netif_t *n = (esp_netif_t *)arg;

    /* ── Open netlink socket ── */
    int fd = socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (fd < 0) {
        ESP_LOGE(TAG, "[%s] netlink socket: %s",
                 n->linux_ifname, strerror(errno));
        goto done;
    }

    struct sockaddr_nl sa = {
        .nl_family = AF_NETLINK,
        .nl_groups = RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR | RTMGRP_LINK,
    };

    /* ── Bind BEFORE probing current state so no RTM_NEWADDR is missed ── */
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        ESP_LOGE(TAG, "[%s] netlink bind: %s",
                 n->linux_ifname, strerror(errno));
        close(fd);
        goto done;
    }

    n->nl_fd   = fd;

    /* Cache ifindex once; kmod netdev index is stable for netdev lifetime. */
    n->ifindex = if_nametoindex(n->linux_ifname);
    if (n->ifindex == 0)
        ESP_LOGW(TAG, "[%s] if_nametoindex: %s — will accept all ifindex",
                 n->linux_ifname, strerror(errno));

    /* ── Probe: interface may already have an IP (e.g. reconnect path) ── */
    post_got_ip(n);

    /* ── Event loop ── */
    uint8_t buf[NL_BUF_SZ] __attribute__((aligned(NLMSG_ALIGNTO)));

    while (n->running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        FD_SET(n->wakeup_pipe[0], &rfds);
        int maxfd = (fd > n->wakeup_pipe[0]) ? fd : n->wakeup_pipe[0];

        int rc = select(maxfd + 1, &rfds, NULL, NULL, NULL);
        if (rc < 0) {
            if (errno == EINTR) continue;
            ESP_LOGE(TAG, "[%s] select: %s", n->linux_ifname, strerror(errno));
            break;
        }

        /* Wakeup pipe written by watcher_stop() → clean exit. */
        if (FD_ISSET(n->wakeup_pipe[0], &rfds)) break;
        if (!FD_ISSET(fd, &rfds)) continue;

        ssize_t len = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (len < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            ESP_LOGE(TAG, "[%s] recv: %s", n->linux_ifname, strerror(errno));
            break;
        }

        for (struct nlmsghdr *nh = (struct nlmsghdr *)buf;
             NLMSG_OK(nh, (uint32_t)len);
             nh = NLMSG_NEXT(nh, len))
        {
            if (nh->nlmsg_type == NLMSG_DONE)  break;
            if (nh->nlmsg_type == NLMSG_ERROR) break;

            /* Link-state transitions: emit LOST_IP on carrier-down so apps
             * notice cable pulls / radio drops even when the kernel doesn't
             * synthesise an RTM_DELADDR (it usually keeps the address while
             * the interface is briefly NO-CARRIER). */
            if (nh->nlmsg_type == RTM_NEWLINK ||
                nh->nlmsg_type == RTM_DELLINK) {
                struct ifinfomsg *ifi = (struct ifinfomsg *)NLMSG_DATA(nh);
                if (n->ifindex != 0 &&
                    (unsigned)ifi->ifi_index != n->ifindex) continue;
                bool running = (ifi->ifi_flags & IFF_RUNNING) != 0 &&
                               nh->nlmsg_type != RTM_DELLINK;
                ESP_LOGD(TAG, "[%s] netlink LINK event type=%u running=%d flags=0x%x",
                         n->linux_ifname, (unsigned)nh->nlmsg_type, running ? 1 : 0,
                         (unsigned)ifi->ifi_flags);
                if (!running) post_lost_ip(n);
                continue;
            }

            if (nh->nlmsg_type != RTM_NEWADDR &&
                nh->nlmsg_type != RTM_DELADDR) continue;

            struct ifaddrmsg *ifa = (struct ifaddrmsg *)NLMSG_DATA(nh);
            if (ifa->ifa_family != AF_INET &&
                ifa->ifa_family != AF_INET6) continue;

            /* Filter by ifindex if we resolved one; otherwise accept all
             * (edge case: kmod registers netdev after watcher start). */
            if (n->ifindex != 0 && ifa->ifa_index != n->ifindex) continue;

            if (ifa->ifa_family == AF_INET) {
                ESP_LOGD(TAG, "[%s] netlink ADDR event type=%u family=AF_INET",
                         n->linux_ifname, (unsigned)nh->nlmsg_type);
                if (nh->nlmsg_type == RTM_NEWADDR) post_got_ip(n);
                else                                post_lost_ip(n);
            } else if (n->ipv6_enabled) {
                /* Per-netif gate: app armed v6 via esp_netif_create_ip6_
                 * linklocal().  No IDF LOST_IP6 event — emit on add only. */
                ESP_LOGD(TAG, "[%s] netlink ADDR event type=%u family=AF_INET6",
                         n->linux_ifname, (unsigned)nh->nlmsg_type);
                if (nh->nlmsg_type == RTM_NEWADDR) post_got_ip6(n, nh);
            }
        }
    }

    close(fd);
    n->nl_fd = -1;

done:
    /* Signal esp_netif_destroy() / watcher_stop() that we are done.
     * watcher_alive is cleared under s_lock so the caller's pthread_join
     * cannot return before this assignment completes. */
    pthread_mutex_lock(&s_lock);
    n->watcher_alive = false;
    pthread_mutex_unlock(&s_lock);
    return NULL;
}

/* ── Watcher start / stop ────────────────────────────────────────────── */

static esp_err_t watcher_start(esp_netif_t *n)
{
    /* Caller must hold s_lock. */
    if (n->watcher_alive) return ESP_OK;

    if (pipe2(n->wakeup_pipe, O_CLOEXEC) < 0) {
        ESP_LOGE(TAG, "pipe: %s", strerror(errno));
        return ESP_FAIL;
    }

    n->running       = true;
    n->watcher_alive = true;
    n->nl_fd         = -1;

    if (pthread_create(&n->watcher_tid, NULL, watcher_main, n) != 0) {
        ESP_LOGE(TAG, "pthread_create: %s", strerror(errno));
        close(n->wakeup_pipe[0]);
        close(n->wakeup_pipe[1]);
        n->wakeup_pipe[0] = n->wakeup_pipe[1] = -1;
        n->running       = false;
        n->watcher_alive = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*
 * watcher_stop: signal the watcher thread and join it.
 *
 * The s_lock "claim" pattern (clear watcher_alive inside the lock, then
 * operate outside) ensures only one caller does the pipe-write + join,
 * even if esp_netif_down() and esp_netif_destroy() race.
 */
static void watcher_stop(esp_netif_t *n)
{
    pthread_mutex_lock(&s_lock);
    if (!n->watcher_alive) {
        pthread_mutex_unlock(&s_lock);
        return;
    }
    /* Claim the stop atomically: clear watcher_alive HERE so a racing
     * caller (e.g. esp_netif_destroy() vs esp_netif_down()) hits the
     * !watcher_alive early-return and avoids a double pthread_join.
     * watcher_main will redundantly clear the same flag on exit — same
     * value, no race. */
    n->watcher_alive = false;
    n->running = false;
    int pipe_wr = n->wakeup_pipe[1];
    pthread_mutex_unlock(&s_lock);

    /* Unblock select() in the watcher thread. */
    if (pipe_wr >= 0) {
        char b = 0;
        ssize_t w = write(pipe_wr, &b, 1);
        (void)w;
    }

    /* Join: watcher_main sets watcher_alive=false then returns. */
    pthread_join(n->watcher_tid, NULL);

    /* Close pipe fds — watcher_main already closed nl_fd. */
    if (n->wakeup_pipe[0] >= 0) { close(n->wakeup_pipe[0]); n->wakeup_pipe[0] = -1; }
    if (n->wakeup_pipe[1] >= 0) { close(n->wakeup_pipe[1]); n->wakeup_pipe[1] = -1; }
}

/* ── Module init / deinit ────────────────────────────────────────────── */

esp_err_t esp_netif_init(void)
{
    pthread_mutex_lock(&s_lock);
    if (s_inited) { pthread_mutex_unlock(&s_lock); return ESP_OK; }
    s_inited = true;
    pthread_mutex_unlock(&s_lock);
    ESP_LOGI(TAG, "esp_netif (Linux/kmod observer) ready");
    return ESP_OK;
}

esp_err_t esp_netif_deinit(void)
{
    pthread_mutex_lock(&s_lock);
    esp_netif_t *n = s_list;
    s_list   = NULL;
    s_inited = false;
    pthread_mutex_unlock(&s_lock);

    while (n) {
        esp_netif_t *nx = n->next;
        watcher_stop(n);   /* safe: called outside s_lock */
        free(n);
        n = nx;
    }
    return ESP_OK;
}

/* ── Default netdev name (per-role) ──────────────────────────────────── */

/*
 * Default Linux netdev name resolution, keyed off the IDF if_key.
 *
 * Today the kmod registers two ethernet-class netdevs that mirror the
 * coprocessor's Wi-Fi STA and AP MACs (ethsta0 + ethap0).  IDF examples
 * call esp_netif_create_default_wifi_sta() / _ap() which set if_key to
 * "WIFI_STA_DEF" / "WIFI_AP_DEF"; map those to the matching netdev so
 * apps just work without manually calling esp_netif_linux_set_netdev_name().
 *
 * Env overrides (rarely needed; useful when the kmod is renamed via udev
 * rules or when running with a single combined netdev):
 *   ESPNETIF_NETDEV_STA   — overrides WIFI_STA_DEF default
 *   ESPNETIF_NETDEV_AP    — overrides WIFI_AP_DEF  default
 *   ESPNETIF_NETDEV       — legacy single-knob fallback (any other if_key)
 */
static const char *default_netdev_for_key(const char *if_key)
{
    if (if_key && strcmp(if_key, "WIFI_STA_DEF") == 0) {
        const char *e = getenv("ESPNETIF_NETDEV_STA");
        if (e && *e) return e;
        return "ethsta0";
    }
    if (if_key && strcmp(if_key, "WIFI_AP_DEF") == 0) {
        const char *e = getenv("ESPNETIF_NETDEV_AP");
        if (e && *e) return e;
        return "ethap0";
    }
    const char *e = getenv("ESPNETIF_NETDEV");
    return (e && *e) ? e : DEFAULT_NETDEV;
}

/* SIOCSIFFLAGS — administratively bring a netdev UP/DOWN.  DHCP-client
 * binders (dhcpcd, dhclient, udhcpc) and dnsmasq all require the iface
 * to be UP; kmod may register the netdev in DOWN state.  Best-effort:
 * warn on failure but don't fail esp_netif_up — apps may have CAP_NET_
 * ADMIN dropped and rely on an external udev / systemd-networkd rule. */
static void linux_netdev_set_updown(const char *ifname, bool up)
{
    int s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (s < 0) return;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
        ESP_LOGW(TAG, "SIOCGIFFLAGS [%s]: %s", ifname, strerror(errno));
        close(s);
        return;
    }
    short want = up ? (ifr.ifr_flags |  (IFF_UP | IFF_RUNNING))
                     : (ifr.ifr_flags & ~(IFF_UP | IFF_RUNNING));
    if (ifr.ifr_flags == want) { close(s); return; }
    ifr.ifr_flags = want;
    if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0)
        ESP_LOGW(TAG, "SIOCSIFFLAGS %s [%s]: %s",
                 up ? "UP" : "DOWN", ifname, strerror(errno));
    close(s);
}

/* ── esp_netif_new / destroy ─────────────────────────────────────────── */

esp_netif_t *esp_netif_new(const esp_netif_config_t *cfg)
{
    if (!cfg || !cfg->base) return NULL;

    esp_netif_t *n = calloc(1, sizeof(*n));
    if (!n) return NULL;

    if (cfg->base->if_key)
        strncpy(n->if_key,  cfg->base->if_key,  sizeof(n->if_key)  - 1);
    if (cfg->base->if_desc)
        strncpy(n->if_desc, cfg->base->if_desc, sizeof(n->if_desc) - 1);

    n->route_prio   = cfg->base->route_prio;
    n->flags        = cfg->base->flags;
    n->dhcpc_status = ESP_NETIF_DHCP_INIT;
    n->dhcps_status = ESP_NETIF_DHCP_INIT;
    n->nl_fd        = -1;
    n->wakeup_pipe[0] = n->wakeup_pipe[1] = -1;

    strncpy(n->linux_ifname,
            default_netdev_for_key(n->if_key), IFNAMSIZ - 1);

    pthread_mutex_lock(&s_lock);
    /* Reject duplicate if_key — IDF guarantees one instance per key. */
    for (esp_netif_t *c = s_list; c; c = c->next) {
        if (strcmp(c->if_key, n->if_key) == 0) {
            ESP_LOGE(TAG, "esp_netif_new: duplicate if_key '%s'", n->if_key);
            pthread_mutex_unlock(&s_lock);
            free(n);
            return NULL;
        }
    }
    n->next = s_list;
    s_list  = n;
    pthread_mutex_unlock(&s_lock);

    ESP_LOGI(TAG, "esp_netif_new: '%s' → '%s'", n->if_key, n->linux_ifname);
    return n;
}

void esp_netif_destroy(esp_netif_t *n)
{
    if (!n) return;

    watcher_stop(n);

    pthread_mutex_lock(&s_lock);
    if (s_list == n) {
        s_list = n->next;
    } else {
        for (esp_netif_t *c = s_list; c; c = c->next) {
            if (c->next == n) { c->next = n->next; break; }
        }
    }
    pthread_mutex_unlock(&s_lock);

    free(n);
}

/* ── Default constructors (Wi-Fi defaults are in esp_wifi/wifi_default.c) ─ */

esp_netif_t *esp_netif_create_default_eth(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg = { .base = &base };
    return esp_netif_new(&cfg);
}

/* ── Linux-specific helpers ──────────────────────────────────────────── */

/*
 * Pre-up-only: must be called before esp_netif_up() (and therefore
 * before any STA/AP action handler that triggers up).  The watcher
 * thread reads linux_ifname/ifindex without taking s_lock; renaming
 * a netdev that already has a live watcher races those reads.
 */
esp_err_t esp_netif_linux_set_netdev_name(esp_netif_t *n, const char *name)
{
    if (!n || !name || !*name) return ESP_ERR_INVALID_ARG;
    if (n->up) return ESP_ERR_INVALID_STATE;   /* watcher already running */
    strncpy(n->linux_ifname, name, IFNAMSIZ - 1);
    n->linux_ifname[IFNAMSIZ - 1] = '\0';
    n->ifindex = 0;   /* will be re-resolved on next watcher start */
    return ESP_OK;
}

const char *esp_netif_linux_get_netdev_name(esp_netif_t *n)
{
    return n ? n->linux_ifname : NULL;
}

/* esp_netif_linux_register_wifi_handlers() is gone; handler registration
 * is now owned by esp_wifi/wifi_default.c
 * (esp_wifi_set_default_wifi_sta_handlers / _ap_handlers).  Apps
 * calling esp_netif_create_default_wifi_sta() get handlers wired
 * automatically as part of that call. */

/* ── Lifecycle ───────────────────────────────────────────────────────── */

esp_err_t esp_netif_up(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    if (n->up) { pthread_mutex_unlock(&s_lock); return ESP_OK; }
    n->up = true;
    esp_err_t r = watcher_start(n);   /* sets watcher_alive under s_lock */
    pthread_mutex_unlock(&s_lock);

    /* Bring the netdev administratively UP so DHCP client/server can
     * bind to it.  kmod may register in DOWN state. */
    if (r == ESP_OK)
        linux_netdev_set_updown(n->linux_ifname, true);

    /* AP netifs (ESP_NETIF_DHCP_SERVER) kick the DHCP server at netif
     * up — soft-AP has its static IP already set, so the daemon can
     * bind immediately.  STA netifs (ESP_NETIF_DHCP_CLIENT) DO NOT
     * kick the client here; the client is started from
     * esp_netif_action_connected, matching IDF's lifecycle where
     * DHCP only fires once the link associates (avoids dhcpcd
     * spinning while the radio waits for carrier).
     *
     * Skip if app pre-empted via esp_netif_dhcps_stop() (status==STOPPED)
     * — symmetric with STA dhcpc gating. */
    if (r == ESP_OK && (n->flags & ESP_NETIF_DHCP_SERVER) &&
        n->dhcps_status != ESP_NETIF_DHCP_STOPPED) {
        esp_netif_dhcps_start(n);   /* public wrapper → updates status */
    }

    return r;
}

esp_err_t esp_netif_down(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&s_lock);
    if (!n->up) { pthread_mutex_unlock(&s_lock); return ESP_OK; }
    n->up = false;
    pthread_mutex_unlock(&s_lock);

    /* Post LOST_IP before stopping watcher (watcher may also post it;
     * post_lost_ip's test-and-clear ensures only one fires). */
    post_lost_ip(n);

    /* AP netif: stop the DHCP server.  STA netif's DHCP client is
     * stopped from esp_netif_action_disconnected (lifecycle-symmetric
     * with the client start in action_connected).  Use the public
     * wrapper so dhcps_status tracks reality. */
    if ((n->flags & ESP_NETIF_DHCP_SERVER) &&
        n->dhcps_status == ESP_NETIF_DHCP_STARTED) {
        esp_err_t rc = esp_netif_dhcps_stop(n);
        if (rc != ESP_OK)
            ESP_LOGW(TAG, "[%s] dhcps_stop on netif down: 0x%x",
                     n->linux_ifname, (unsigned)rc);
    }

    /* Bring the netdev administratively DOWN — mirror of esp_netif_up. */
    linux_netdev_set_updown(n->linux_ifname, false);

    watcher_stop(n);
    return ESP_OK;
}

bool esp_netif_is_netif_up(esp_netif_t *n)
{
    return n && n->up;
}

/* ── IP info ─────────────────────────────────────────────────────────── */

esp_err_t esp_netif_get_ip_info(esp_netif_t *n, esp_netif_ip_info_t *out)
{
    if (!n || !out) return ESP_ERR_INVALID_ARG;
    return kernel_get_ip_info(n->linux_ifname, out);
}

/*
 * set_ip_info: static IP assignment via ioctl.
 *
 * IDF contract: call esp_netif_dhcpc_stop() first, then set_ip_info().
 * We mirror that: refuse only if DHCP is *currently running* (STARTED),
 * not merely because the DHCP_CLIENT capability flag is set on the netif.
 * An app that follows the IDF idiom (stop → set) works correctly.
 *
 * Requires CAP_NET_ADMIN for SIOCSIFADDR / SIOCSIFNETMASK.
 */
esp_err_t esp_netif_set_ip_info(esp_netif_t *n, const esp_netif_ip_info_t *ip)
{
    if (!n || !ip) return ESP_ERR_INVALID_ARG;

    /* (B) Gate on runtime DHCP status, not the static capability flag. */
    if (n->dhcpc_status == ESP_NETIF_DHCP_STARTED) {
        ESP_LOGW(TAG, "set_ip_info: DHCP client is running — "
                      "call esp_netif_dhcpc_stop() first");
        return ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED;
    }

    int s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (s < 0) return ESP_FAIL;

    struct ifreq      ifr;
    struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;

    /* (A) Inline the two ioctls so each failure is caught independently
     *     and propagated — no macro that silently discards the return. */

    /* --- SIOCSIFADDR --- */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, n->linux_ifname, IFNAMSIZ - 1);
    sin->sin_family      = AF_INET;
    sin->sin_addr.s_addr = ip->ip.addr;
    if (ioctl(s, SIOCSIFADDR, &ifr) < 0) {
        ESP_LOGE(TAG, "SIOCSIFADDR [%s]: %s", n->linux_ifname, strerror(errno));
        close(s);
        return ESP_FAIL;
    }

    /* --- SIOCSIFNETMASK (best-effort; non-fatal if netmask is zero) --- */
    if (ip->netmask.addr != 0) {
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, n->linux_ifname, IFNAMSIZ - 1);
        sin->sin_family      = AF_INET;
        sin->sin_addr.s_addr = ip->netmask.addr;
        if (ioctl(s, SIOCSIFNETMASK, &ifr) < 0)
            ESP_LOGW(TAG, "SIOCSIFNETMASK [%s]: %s",
                     n->linux_ifname, strerror(errno));
            /* Not fatal — address is set; warn and continue. */
    }

    /* --- Default route via gw (best-effort; SIOCDELRT then SIOCADDRT
     *     so a pre-existing route from DHCP doesn't trip EEXIST). --- */
    if (ip->gw.addr != 0) {
        struct rtentry      rt;
        struct sockaddr_in *sin_dst, *sin_mask, *sin_gw;
        memset(&rt, 0, sizeof(rt));

        sin_dst  = (struct sockaddr_in *)&rt.rt_dst;
        sin_mask = (struct sockaddr_in *)&rt.rt_genmask;
        sin_gw   = (struct sockaddr_in *)&rt.rt_gateway;
        sin_dst->sin_family  = AF_INET;
        sin_mask->sin_family = AF_INET;
        sin_gw->sin_family   = AF_INET;
        sin_gw->sin_addr.s_addr = ip->gw.addr;
        rt.rt_flags = RTF_UP | RTF_GATEWAY;
        rt.rt_dev   = (char *)n->linux_ifname;

        (void)ioctl(s, SIOCDELRT, &rt);
        if (ioctl(s, SIOCADDRT, &rt) < 0)
            ESP_LOGW(TAG, "SIOCADDRT [%s]: %s",
                     n->linux_ifname, strerror(errno));
    }

    close(s);
    return ESP_OK;
}

esp_err_t esp_netif_get_old_ip_info(esp_netif_t *n, esp_netif_ip_info_t *out)
{
    if (!n || !out) return ESP_ERR_INVALID_ARG;
    pthread_mutex_lock(&s_lock);
    *out = n->last_ip;
    pthread_mutex_unlock(&s_lock);
    return ESP_OK;
}

esp_err_t esp_netif_set_old_ip_info(esp_netif_t *n,
                                     const esp_netif_ip_info_t *ip)
{
    if (!n || !ip) return ESP_ERR_INVALID_ARG;
    pthread_mutex_lock(&s_lock);
    n->last_ip = *ip;
    pthread_mutex_unlock(&s_lock);
    return ESP_OK;
}

/* ── IPv6 ────────────────────────────────────────────────────────────── */

esp_err_t esp_netif_create_ip6_linklocal(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;
    /* Linux SLAAC is always on at the kernel level; this call just
     * arms IPv6 event emission for the netif (IP_EVENT_GOT_IP6 fires
     * from the netlink watcher on RTM_NEWADDR/AF_INET6). esp_netif has
     * no IPv6-flag in esp_netif_flags_t — explicit opt-in via this
     * API is the IDF idiom, so we use it as the per-netif gate. */
    n->ipv6_enabled = true;
    return ESP_OK;
}

esp_err_t esp_netif_get_ip6_linklocal(esp_netif_t *n, esp_ip6_addr_t *out)
{
    if (!n || !out) return ESP_ERR_INVALID_ARG;

    struct ifaddrs *list = NULL;
    if (getifaddrs(&list) < 0) return ESP_FAIL;

    esp_err_t r = ESP_ERR_NOT_FOUND;
    for (struct ifaddrs *ifa = list; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET6) continue;
        if (strcmp(ifa->ifa_name, n->linux_ifname) != 0) continue;

        struct sockaddr_in6 *s6 = (struct sockaddr_in6 *)ifa->ifa_addr;
        if (!IN6_IS_ADDR_LINKLOCAL(&s6->sin6_addr)) continue;

        memcpy(out->addr, &s6->sin6_addr, sizeof(out->addr));
        r = ESP_OK;
        break;
    }
    freeifaddrs(list);
    return r;
}

/*
 * esp_netif_ip6_get_addr_type: classify by IPv6 prefix.
 * Bytes are stored in network order (memcpy from sin6_addr).
 */
esp_ip6_addr_type_t esp_netif_ip6_get_addr_type(esp_ip6_addr_t *ip6)
{
    if (!ip6) return ESP_IP6_ADDR_IS_UNKNOWN;
    const uint8_t *b = (const uint8_t *)ip6->addr;
    /* IPv4-mapped: ::ffff:0:0/96 */
    if (b[0]==0 && b[1]==0 && b[2]==0 && b[3]==0 &&
        b[4]==0 && b[5]==0 && b[6]==0 && b[7]==0 &&
        b[8]==0 && b[9]==0 && b[10]==0xff && b[11]==0xff)
        return ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6;
    if (b[0]==0xfe && (b[1] & 0xc0)==0x80) return ESP_IP6_ADDR_IS_LINK_LOCAL;
    if (b[0]==0xfe && (b[1] & 0xc0)==0xc0) return ESP_IP6_ADDR_IS_SITE_LOCAL;
    if ((b[0] & 0xfe)==0xfc)               return ESP_IP6_ADDR_IS_UNIQUE_LOCAL;
    return ESP_IP6_ADDR_IS_GLOBAL;
}

/*
 * esp_netif_get_all_ip6: enumerate every IPv6 address bound to the
 * netdev. IDF caller passes a fixed-size array (MAX_IP6_ADDRS_PER_NETIF,
 * default 5); we cap at that bound to mirror the contract.
 */
int esp_netif_get_all_ip6(esp_netif_t *n, esp_ip6_addr_t if_ip6[])
{
    if (!n || !if_ip6) return 0;
    enum { MAX_IP6 = 5 };

    struct ifaddrs *list = NULL;
    if (getifaddrs(&list) < 0) return 0;

    int count = 0;
    for (struct ifaddrs *ifa = list; ifa && count < MAX_IP6; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET6) continue;
        if (strcmp(ifa->ifa_name, n->linux_ifname) != 0) continue;

        struct sockaddr_in6 *s6 = (struct sockaddr_in6 *)ifa->ifa_addr;
        memcpy(if_ip6[count].addr, &s6->sin6_addr, sizeof(if_ip6[0].addr));
        count++;
    }
    freeifaddrs(list);
    return count;
}

/* ── DNS (in-process cache; kernel resolves via /etc/resolv.conf) ─── */

esp_err_t esp_netif_set_dns_info(esp_netif_t *n, esp_netif_dns_type_t t,
                                  esp_netif_dns_info_t *dns)
{
    if (!n || !dns || (unsigned)t >= ESP_NETIF_DNS_MAX)
        return ESP_ERR_INVALID_ARG;
    n->dns[t] = *dns;
    return ESP_OK;
}

esp_err_t esp_netif_get_dns_info(esp_netif_t *n, esp_netif_dns_type_t t,
                                  esp_netif_dns_info_t *dns)
{
    if (!n || !dns || (unsigned)t >= ESP_NETIF_DNS_MAX)
        return ESP_ERR_INVALID_ARG;
    *dns = n->dns[t];
    return ESP_OK;
}

/* ── DHCP bookkeeping + direct backend dispatch ──────────────────────── */

esp_err_t esp_netif_dhcpc_start(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;
    if (n->dhcpc_status == ESP_NETIF_DHCP_STARTED)
        return ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED;
    esp_err_t rc = eh_dhcpc_start_linux(n->linux_ifname);
    if (rc == ESP_OK) n->dhcpc_status = ESP_NETIF_DHCP_STARTED;
    return rc;
}

esp_err_t esp_netif_dhcpc_stop(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;
    if (n->dhcpc_status == ESP_NETIF_DHCP_STOPPED)
        return ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED;
    esp_err_t rc = eh_dhcpc_stop_linux(n->linux_ifname);
    if (rc == ESP_OK) n->dhcpc_status = ESP_NETIF_DHCP_STOPPED;
    return rc;
}

esp_err_t esp_netif_dhcpc_get_status(esp_netif_t *n,
                                      esp_netif_dhcp_status_t *st)
{
    if (!n || !st) return ESP_ERR_INVALID_ARG;
    *st = n->dhcpc_status;
    return ESP_OK;
}

/* DHCP server — direct call into esp_netif_dhcp_server_linux.c's
 * dnsmasq / udhcpd / isc-dhcpd dispatcher.  Real esp_err_t propagates
 * (no silent ESP_OK when nothing is wired). */
esp_err_t esp_netif_dhcps_start(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;
    if (n->dhcps_status == ESP_NETIF_DHCP_STARTED)
        return ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED;
    esp_err_t rc = eh_dhcps_start_linux(n->linux_ifname);
    if (rc == ESP_OK) n->dhcps_status = ESP_NETIF_DHCP_STARTED;
    return rc;
}
esp_err_t esp_netif_dhcps_stop(esp_netif_t *n)
{
    if (!n) return ESP_ERR_INVALID_ARG;
    if (n->dhcps_status == ESP_NETIF_DHCP_STOPPED)
        return ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED;
    esp_err_t rc = eh_dhcps_stop_linux(n->linux_ifname);
    if (rc == ESP_OK) n->dhcps_status = ESP_NETIF_DHCP_STOPPED;
    return rc;
}
esp_err_t esp_netif_dhcps_get_status(esp_netif_t *n,
                                      esp_netif_dhcp_status_t *st)
{
    if (!n || !st) return ESP_ERR_INVALID_ARG;
    *st = n->dhcps_status;
    return ESP_OK;
}

/* ── MAC ─────────────────────────────────────────────────────────────── */

esp_err_t esp_netif_get_mac(esp_netif_t *n, uint8_t mac[6])
{
    if (!n || !mac) return ESP_ERR_INVALID_ARG;

    int s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (s < 0) return ESP_FAIL;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, n->linux_ifname, IFNAMSIZ - 1);

    int r = ioctl(s, SIOCGIFHWADDR, &ifr);
    close(s);
    if (r < 0) {
        ESP_LOGE(TAG, "SIOCGIFHWADDR [%s]: %s",
                 n->linux_ifname, strerror(errno));
        return ESP_FAIL;
    }
    memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
    return ESP_OK;
}

esp_err_t esp_netif_set_mac(esp_netif_t *n, uint8_t mac[6])
{
    /* The kmod's esp_set_mac_address ndo (driver/src/main.c:190) accepts
     * SIOCSIFHWADDR from user space and writes priv->mac_address +
     * eth_hw_addr_set(ndev, …).  Without this push, the kmod's stored
     * MAC stays all-zero, SIOCSIFFLAGS UP fails, and DHCP never starts.
     * Mirror of esp_netif_get_mac above (SIOCGIFHWADDR), but writing. */
    if (!n || !mac) return ESP_ERR_INVALID_ARG;

    int s = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (s < 0) return ESP_FAIL;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, n->linux_ifname, IFNAMSIZ - 1);
    ifr.ifr_hwaddr.sa_family = ARPHRD_ETHER;
    memcpy(ifr.ifr_hwaddr.sa_data, mac, 6);

    int r = ioctl(s, SIOCSIFHWADDR, &ifr);
    close(s);
    if (r < 0) {
        ESP_LOGE(TAG, "SIOCSIFHWADDR [%s]: %s",
                 n->linux_ifname, strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ── Metadata / iteration ────────────────────────────────────────────── */

const char *esp_netif_get_ifkey(esp_netif_t *n)
{
    return n ? n->if_key : NULL;
}

const char *esp_netif_get_desc(esp_netif_t *n)
{
    return n ? n->if_desc : NULL;
}

int esp_netif_get_route_prio(esp_netif_t *n)
{
    return n ? n->route_prio : -1;
}

esp_netif_flags_t esp_netif_get_flags(esp_netif_t *n)
{
    return n ? n->flags : 0;
}

esp_netif_t *esp_netif_get_handle_from_ifkey(const char *key)
{
    if (!key) return NULL;
    pthread_mutex_lock(&s_lock);
    esp_netif_t *n = s_list;
    for (; n; n = n->next)
        if (strcmp(n->if_key, key) == 0) break;
    pthread_mutex_unlock(&s_lock);
    return n;
}

/*
 * esp_netif_next / esp_netif_next_unsafe
 *
 * IDF semantics: NULL → first element; non-NULL → next element.
 * "unsafe" variant skips the lock — caller guarantees serialisation
 * (e.g. inside esp_netif_tcpip_exec).
 */
esp_netif_t *esp_netif_next_unsafe(esp_netif_t *n)
{
    return n ? n->next : s_list;
}

esp_netif_t *esp_netif_next(esp_netif_t *n)
{
    pthread_mutex_lock(&s_lock);
    esp_netif_t *r = esp_netif_next_unsafe(n);
    pthread_mutex_unlock(&s_lock);
    return r;
}

size_t esp_netif_get_nr_of_ifs(void)
{
    size_t c = 0;
    pthread_mutex_lock(&s_lock);
    for (esp_netif_t *n = s_list; n; n = n->next) c++;
    pthread_mutex_unlock(&s_lock);
    return c;
}

/*
 * esp_netif_find_if — iterate list calling fn(netif, ctx); return first match.
 * Used by connect.c::get_example_netif_from_desc().
 *
 * Lock-held callback: fn() runs under s_lock. Callers MUST NOT re-enter
 * any esp_netif_* API from fn or they will self-deadlock. Typical use
 * (string-compare predicate) is safe.
 */
esp_netif_t *esp_netif_find_if(bool (*fn)(esp_netif_t *, void *), void *ctx)
{
    if (!fn) return NULL;
    pthread_mutex_lock(&s_lock);
    esp_netif_t *n = s_list;
    for (; n; n = n->next)
        if (fn(n, ctx)) break;
    pthread_mutex_unlock(&s_lock);
    return n;
}

/* ── Default / get-default netif ─────────────────────────────────────── */

static esp_netif_t *s_default_netif = NULL;

esp_err_t esp_netif_set_default_netif(esp_netif_t *n)
{
    pthread_mutex_lock(&s_lock);
    s_default_netif = n;
    pthread_mutex_unlock(&s_lock);
    return ESP_OK;
}

esp_netif_t *esp_netif_get_default_netif(void)
{
    pthread_mutex_lock(&s_lock);
    esp_netif_t *n = s_default_netif;
    pthread_mutex_unlock(&s_lock);
    return n;
}

/* ── TCPIP exec — direct call on Linux (no TCPIP task) ──────────────── */

esp_err_t esp_netif_tcpip_exec(esp_netif_callback_fn fn, void *ctx)
{
    if (!fn) return ESP_ERR_INVALID_ARG;
    return fn(ctx);
}

/* ── Stack impl accessor ─────────────────────────────────────────────── */

void *esp_netif_get_netif_impl(esp_netif_t *n)
{
    /* Linux has no separate stack-netif object; the esp_netif is its own impl.
     * Report it only once started (esp_netif_up on STA_START) so callers can gate
     * on "netif added" the way the lwip port gates on netif->input. */
    return (n && n->up) ? n : NULL;
}

/* ── Stubs: impl-name, NAPT, hostname (not meaningful on kmod host) ─── */

int esp_netif_get_netif_impl_index(esp_netif_t *n)
{
    return n ? (int)n->ifindex : -1;
}

esp_err_t esp_netif_get_netif_impl_name(esp_netif_t *n, char *name)
{
    if (!n || !name) return ESP_ERR_INVALID_ARG;
    /* IDF caller buffer is NETIF_NAMESIZE (6) bytes. Linux netdev names
     * can be up to IFNAMSIZ (16) — fail loudly instead of truncating so
     * apps using long names (e.g. our default "ethsta0"/"ethap0") notice
     * and switch to esp_netif_linux_get_netdev_name(). */
    if (strlen(n->linux_ifname) >= NETIF_NAMESIZE)
        return ESP_ERR_INVALID_SIZE;
    strcpy(name, n->linux_ifname);
    return ESP_OK;
}

esp_err_t esp_netif_napt_enable(esp_netif_t *n)
{
    (void)n; return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t esp_netif_napt_disable(esp_netif_t *n)
{
    (void)n; return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_netif_set_hostname(esp_netif_t *n, const char *hostname)
{
    (void)n; (void)hostname; return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t esp_netif_get_hostname(esp_netif_t *n, const char **hostname)
{
    (void)n; (void)hostname; return ESP_ERR_NOT_SUPPORTED;
}

/* ── Driver / TX / RX — no-ops (kmod owns these) ─────────────────────── */

esp_err_t esp_netif_attach(esp_netif_t *n, esp_netif_iodriver_handle h)
{
    (void)n; (void)h; return ESP_OK;
}
esp_err_t esp_netif_transmit(esp_netif_t *n, void *d, size_t l)
{
    (void)n; (void)d; (void)l; return ESP_OK;
}
esp_err_t esp_netif_receive(esp_netif_t *n, void *b, size_t l, void *eb)
{
    (void)n; (void)b; (void)l; (void)eb; return ESP_OK;
}
void esp_netif_free_rx_buffer(void *h, void *b)
{
    (void)h; (void)b;
}
void esp_netif_netstack_buf_ref(void *buf) { (void)buf; }
void esp_netif_netstack_buf_free(void *buf) { (void)buf; }

/* ── Action callbacks (esp_netif_action_*) ───────────────────────────── */

void esp_netif_action_start(void *n, esp_event_base_t b,
                            int32_t id, void *d)
{
    (void)b; (void)id; (void)d;
    esp_netif_up((esp_netif_t *)n);
}

void esp_netif_action_stop(void *n, esp_event_base_t b,
                           int32_t id, void *d)
{
    (void)b; (void)id; (void)d;
    esp_netif_down((esp_netif_t *)n);
}

void esp_netif_action_connected(void *n, esp_event_base_t b,
                                int32_t id, void *d)
{
    /* Bring netif up, then mirror upstream esp_netif_handlers.c — branch
     * on dhcpc_status rather than just the static DHCP_CLIENT capability
     * flag.  IDF static-IP idiom (dhcpc_stop → set_ip_info) lands in the
     * STOPPED branch and we synthesise GOT_IP from kernel state. */
    esp_netif_action_start(n, b, id, d);

    esp_netif_t *netif = (esp_netif_t *)n;
    if (!netif || !(netif->flags & ESP_NETIF_DHCP_CLIENT)) {
        ESP_LOGI(TAG, "connected: no DHCP_CLIENT flag on netif — not starting DHCP");
        return;
    }

    if (netif->dhcpc_status == ESP_NETIF_DHCP_STARTED) {
        ESP_LOGI(TAG, "connected: DHCP client already started (idempotent)");
        return;  /* already running — idempotent reconnect path */
    }

    if (netif->dhcpc_status != ESP_NETIF_DHCP_STOPPED) {
        ESP_LOGI(TAG, "connected: starting DHCP client (dhcpc_status=%d)",
                 (int)netif->dhcpc_status);
        esp_netif_dhcpc_start(netif);   /* INIT → STARTED via public wrapper */
        return;
    }
    ESP_LOGI(TAG, "connected: dhcpc STOPPED (static IP) — synthesising GOT_IP");

    /* STOPPED: app set a static IP — synthesise GOT_IP via the shared
     * post_got_ip() path, which dedups against the watcher's probe so
     * apps see exactly one IP_EVENT_STA_GOT_IP on first connect.
     *
     * If the kernel hasn't yet reflected the address (rare on Linux —
     * SIOCSIFADDR is synchronous), this call returns silently and the
     * watcher's later RTM_NEWADDR will fire GOT_IP. Apps that need a
     * hard guarantee should call esp_netif_set_ip_info() and verify
     * via esp_netif_get_ip_info() before STA_CONNECTED. */
    post_got_ip(netif);
}

void esp_netif_action_disconnected(void *n, esp_event_base_t b,
                                   int32_t id, void *d)
{
    /* Symmetric with action_connected — only stop if currently STARTED.
     * If app pre-emptively called dhcpc_stop() for static IP, we skip
     * the redundant hook (already-stopped daemon is harmless, but
     * status tracking should reflect what actually ran). */
    esp_netif_t *netif = (esp_netif_t *)n;
    if (netif && (netif->flags & ESP_NETIF_DHCP_CLIENT) &&
        netif->dhcpc_status == ESP_NETIF_DHCP_STARTED) {
        esp_netif_dhcpc_stop(netif);
    }
    esp_netif_action_stop(n, b, id, d);
}

void esp_netif_action_got_ip(void *n, esp_event_base_t b,
                             int32_t id, void *d)
{
    /* On IDF this updates lwip state; on Linux the kernel already has it. */
    (void)n; (void)b; (void)id; (void)d;
}

void esp_netif_action_join_ip6_multicast_group(void *n, esp_event_base_t b,
                                               int32_t id, void *d)
{
    (void)n; (void)b; (void)id; (void)d;
}

void esp_netif_action_leave_ip6_multicast_group(void *n, esp_event_base_t b,
                                                int32_t id, void *d)
{
    (void)n; (void)b; (void)id; (void)d;
}

void esp_netif_action_add_ip6_address(void *n, esp_event_base_t b,
                                      int32_t id, void *d)
{
    (void)n; (void)b; (void)id; (void)d;
}

void esp_netif_action_remove_ip6_address(void *n, esp_event_base_t b,
                                         int32_t id, void *d)
{
    (void)n; (void)b; (void)id; (void)d;
}

/* ── event_id getter (used by upstream IDF internals) ────────────────── */

int32_t esp_netif_get_event_id(esp_netif_t *n,
                                esp_netif_ip_event_type_t event_type)
{
    if (!n) return -1;
    switch (event_type) {
    case ESP_NETIF_IP_EVENT_GOT_IP:   return IP_EVENT_STA_GOT_IP;
    case ESP_NETIF_IP_EVENT_LOST_IP:  return IP_EVENT_STA_LOST_IP;
    default: return -1;
    }
}

#endif /* !ESP_PLATFORM */
