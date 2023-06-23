#include "lwip/prot/dhcp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/prot/iana.h"

#if ESP_LWIP
#include <string.h>

#if LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS
#define DHCP_OPTION_VCI             60
#define DHCP_OPTION_VSI_MAX         16

static u8_t vendor_class_len = 0;
static char *vendor_class_buf = NULL;
static u32_t dhcp_option_vsi[DHCP_OPTION_VSI_MAX] = {0};

void dhcp_free_vendor_class_identifier(void)
{
  mem_free(vendor_class_buf);
}

int dhcp_get_vendor_specific_information(uint8_t len, char * str)
{
  u8_t copy_len = 0;

  if (len == 0 || str == NULL) {
    return ERR_ARG;
  }

  copy_len = LWIP_MIN(len, sizeof(dhcp_option_vsi));

  memcpy(str, dhcp_option_vsi, copy_len);

  return ERR_OK;
}

int dhcp_set_vendor_class_identifier(uint8_t len, const char * str)
{
  if (len == 0 || str == NULL) {
    return ERR_ARG;
  }

  if (vendor_class_buf && vendor_class_len != len) {
    mem_free(vendor_class_buf);
    vendor_class_buf = NULL;
  }

  if (!vendor_class_buf) {
    vendor_class_buf = (char *)mem_malloc(len + 1);
    if (vendor_class_buf == NULL) {
      return ERR_MEM;
    }

    vendor_class_len = len;
  }

  memcpy(vendor_class_buf, str, len);
  return ERR_OK;
}
#endif /* LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS */

void dhcp_parse_extra_opts(struct dhcp *dhcp, uint8_t state, uint8_t option, uint8_t len, struct pbuf* p, uint16_t offset)
{
  LWIP_UNUSED_ARG(dhcp);
  LWIP_UNUSED_ARG(state);
  LWIP_UNUSED_ARG(option);
  LWIP_UNUSED_ARG(len);
  LWIP_UNUSED_ARG(p);
  LWIP_UNUSED_ARG(offset);
#if LWIP_DHCP_ENABLE_MTU_UPDATE
  if ((option == DHCP_OPTION_MTU) &&
     (state == DHCP_STATE_REBOOTING || state == DHCP_STATE_REBINDING ||
      state == DHCP_STATE_RENEWING  || state == DHCP_STATE_REQUESTING)) {
    u32_t mtu = 0;
    struct netif *netif;
    LWIP_ERROR("dhcp_parse_extra_opts(): MTU option's len != 2", len == 2, return;);
    LWIP_ERROR("dhcp_parse_extra_opts(): extracting MTU option failed",
               pbuf_copy_partial(p, &mtu, 2, offset) == 2, return;);
    mtu = lwip_htons((u16_t)mtu);
    NETIF_FOREACH(netif) {
      /* find the netif related to this dhcp */
      if (dhcp == netif_dhcp_data(netif)) {
        if (mtu < netif->mtu) {
          netif->mtu = mtu;
          LWIP_DEBUGF(DHCP_DEBUG | LWIP_DBG_TRACE, ("dhcp_parse_extra_opts(): Negotiated netif MTU is %d\n", netif->mtu));
        }
        return;
      }
    }
  } /* DHCP_OPTION_MTU */
#endif /* LWIP_DHCP_ENABLE_MTU_UPDATE */
#if LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS
    if ((option == DHCP_OPTION_VSI) &&
             (state == DHCP_STATE_REBOOTING || state == DHCP_STATE_REBINDING ||
              state == DHCP_STATE_RENEWING  || state == DHCP_STATE_REQUESTING || state == DHCP_STATE_SELECTING)) {
      u8_t n;
      u32_t value;
      u16_t copy_len;
      for (n = 0; n < DHCP_OPTION_VSI_MAX && len > 0; n++) {
        copy_len = LWIP_MIN(len, 4);
        LWIP_ERROR("dhcp_parse_extra_opts(): extracting VSI option failed",
           pbuf_copy_partial(p, &value, copy_len, offset) == copy_len, return;);
        dhcp_option_vsi[n] = lwip_htonl(value);
        len -= copy_len;
      }
    } /* DHCP_OPTION_VSI */
#endif /* LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS */
}

void dhcp_append_extra_opts(struct netif *netif, uint8_t state, struct dhcp_msg *msg_out, uint16_t *options_out_len)
{
  LWIP_UNUSED_ARG(netif);
  LWIP_UNUSED_ARG(state);
  LWIP_UNUSED_ARG(msg_out);
  LWIP_UNUSED_ARG(options_out_len);
#if LWIP_DHCP_ENABLE_CLIENT_ID
  if (state == DHCP_STATE_RENEWING || state == DHCP_STATE_REBINDING ||
      state == DHCP_STATE_REBOOTING || state == DHCP_STATE_OFF ||
      state == DHCP_STATE_REQUESTING || state == DHCP_STATE_BACKING_OFF || state == DHCP_STATE_SELECTING) {
    size_t i;
    u8_t *options = msg_out->options + *options_out_len;
    LWIP_ERROR("dhcp_append(client_id): options_out_len + 3 + netif->hwaddr_len <= DHCP_OPTIONS_LEN",
               *options_out_len + 3U + netif->hwaddr_len <= DHCP_OPTIONS_LEN, return;);
    *options_out_len = *options_out_len + netif->hwaddr_len + 3;
    *options++ = DHCP_OPTION_CLIENT_ID;
    *options++ = netif->hwaddr_len + 1; /* option size */
    *options++ = LWIP_IANA_HWTYPE_ETHERNET;
    for (i = 0; i < netif->hwaddr_len; i++) {
      *options++ = netif->hwaddr[i];
    }
  }
#endif /* LWIP_DHCP_ENABLE_CLIENT_ID */
#if LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS
  if (state == DHCP_STATE_RENEWING || state == DHCP_STATE_REBINDING ||
      state == DHCP_STATE_REBOOTING || state == DHCP_STATE_OFF ||
      state == DHCP_STATE_REQUESTING || state == DHCP_STATE_BACKING_OFF || state == DHCP_STATE_SELECTING) {
    size_t i;
    const char *p = NULL;
    u8_t len = 0;

    if (vendor_class_buf && vendor_class_len) {
      p = vendor_class_buf;
      len = vendor_class_len;
    } else {
#if LWIP_NETIF_HOSTNAME
      if (netif->hostname != NULL && strlen(netif->hostname) < 0xff) {
        p = netif->hostname;
        len = (u8_t)namelen;
      }
#endif /* LWIP_NETIF_HOSTNAME */
    }
    LWIP_ERROR("dhcp_append(vci): options_out_len + 3 + vci_size <= DHCP_OPTIONS_LEN",
               *options_out_len + 3U + len <= DHCP_OPTIONS_LEN, return;);
    if (p) {
      u8_t *options = msg_out->options + *options_out_len;
      *options_out_len = *options_out_len + len + 3;
      *options++ = DHCP_OPTION_VCI;
      *options++ = len;
      for (i = 0; i < len; i ++) {
        *options++ = p[i];
      }
    }
    return;
  }
#endif /* LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS */

}

#endif
