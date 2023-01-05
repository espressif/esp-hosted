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

#include "esp.h"
#include "esp_api.h"
#include "esp_cfg80211.h"
#include "esp_cmd.h"
#include "esp_kernel_port.h"

/**
  * @brief WiFi PHY rate encodings
  *
  */
typedef enum {
	WIFI_PHY_RATE_1M_L      = 0x00, /**< 1 Mbps with long preamble */
	WIFI_PHY_RATE_2M_L      = 0x01, /**< 2 Mbps with long preamble */
	WIFI_PHY_RATE_5M_L      = 0x02, /**< 5.5 Mbps with long preamble */
	WIFI_PHY_RATE_11M_L     = 0x03, /**< 11 Mbps with long preamble */
	WIFI_PHY_RATE_2M_S      = 0x05, /**< 2 Mbps with short preamble */
	WIFI_PHY_RATE_5M_S      = 0x06, /**< 5.5 Mbps with short preamble */
	WIFI_PHY_RATE_11M_S     = 0x07, /**< 11 Mbps with short preamble */
	WIFI_PHY_RATE_48M       = 0x08, /**< 48 Mbps */
	WIFI_PHY_RATE_24M       = 0x09, /**< 24 Mbps */
	WIFI_PHY_RATE_12M       = 0x0A, /**< 12 Mbps */
	WIFI_PHY_RATE_6M        = 0x0B, /**< 6 Mbps */
	WIFI_PHY_RATE_54M       = 0x0C, /**< 54 Mbps */
	WIFI_PHY_RATE_36M       = 0x0D, /**< 36 Mbps */
	WIFI_PHY_RATE_18M       = 0x0E, /**< 18 Mbps */
	WIFI_PHY_RATE_9M        = 0x0F, /**< 9 Mbps */
	WIFI_PHY_RATE_MCS0_LGI  = 0x10, /**< MCS0 with long GI, 6.5 Mbps for 20MHz, 13.5 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS1_LGI  = 0x11, /**< MCS1 with long GI, 13 Mbps for 20MHz, 27 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS2_LGI  = 0x12, /**< MCS2 with long GI, 19.5 Mbps for 20MHz, 40.5 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS3_LGI  = 0x13, /**< MCS3 with long GI, 26 Mbps for 20MHz, 54 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS4_LGI  = 0x14, /**< MCS4 with long GI, 39 Mbps for 20MHz, 81 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS5_LGI  = 0x15, /**< MCS5 with long GI, 52 Mbps for 20MHz, 108 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS6_LGI  = 0x16, /**< MCS6 with long GI, 58.5 Mbps for 20MHz, 121.5 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS7_LGI  = 0x17, /**< MCS7 with long GI, 65 Mbps for 20MHz, 135 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS0_SGI  = 0x18, /**< MCS0 with short GI, 7.2 Mbps for 20MHz, 15 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS1_SGI  = 0x19, /**< MCS1 with short GI, 14.4 Mbps for 20MHz, 30 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS2_SGI  = 0x1A, /**< MCS2 with short GI, 21.7 Mbps for 20MHz, 45 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS3_SGI  = 0x1B, /**< MCS3 with short GI, 28.9 Mbps for 20MHz, 60 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS4_SGI  = 0x1C, /**< MCS4 with short GI, 43.3 Mbps for 20MHz, 90 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS5_SGI  = 0x1D, /**< MCS5 with short GI, 57.8 Mbps for 20MHz, 120 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS6_SGI  = 0x1E, /**< MCS6 with short GI, 65 Mbps for 20MHz, 135 Mbps for 40MHz */
	WIFI_PHY_RATE_MCS7_SGI  = 0x1F, /**< MCS7 with short GI, 72.2 Mbps for 20MHz, 150 Mbps for 40MHz */
	WIFI_PHY_RATE_LORA_250K = 0x29, /**< 250 Kbps */
	WIFI_PHY_RATE_LORA_500K = 0x2A, /**< 500 Kbps */
	WIFI_PHY_RATE_MAX,
} wifi_phy_rate_t;

/* Supported rates to be advertised to the cfg80211 */
static struct ieee80211_rate esp_rates[] = {
	{.bitrate = 10, .hw_value = WIFI_PHY_RATE_1M_L, },
	{.bitrate = 20, .hw_value = WIFI_PHY_RATE_2M_L, },
	{.bitrate = 55, .hw_value = WIFI_PHY_RATE_5M_L, .hw_value_short = WIFI_PHY_RATE_5M_S},
	{.bitrate = 110, .hw_value = WIFI_PHY_RATE_11M_L, .hw_value_short = WIFI_PHY_RATE_11M_S},
	{.bitrate = 60, .hw_value = WIFI_PHY_RATE_6M, },
	{.bitrate = 90, .hw_value = WIFI_PHY_RATE_9M, },
	{.bitrate = 120, .hw_value = WIFI_PHY_RATE_12M, },
	{.bitrate = 180, .hw_value = WIFI_PHY_RATE_18M, },
	{.bitrate = 240, .hw_value = WIFI_PHY_RATE_24M, },
	{.bitrate = 360, .hw_value = WIFI_PHY_RATE_36M, },
	{.bitrate = 480, .hw_value = WIFI_PHY_RATE_48M, },
	{.bitrate = 540, .hw_value = WIFI_PHY_RATE_54M, },
};


/* Channel definitions to be advertised to cfg80211 */
static struct ieee80211_channel esp_channels_2ghz[] = {
	{.center_freq = 2412, .hw_value = 1, },
	{.center_freq = 2417, .hw_value = 2, },
	{.center_freq = 2422, .hw_value = 3, },
	{.center_freq = 2427, .hw_value = 4, },
	{.center_freq = 2432, .hw_value = 5, },
	{.center_freq = 2437, .hw_value = 6, },
	{.center_freq = 2442, .hw_value = 7, },
	{.center_freq = 2447, .hw_value = 8, },
	{.center_freq = 2452, .hw_value = 9, },
	{.center_freq = 2457, .hw_value = 10, },
	{.center_freq = 2462, .hw_value = 11, },
	{.center_freq = 2467, .hw_value = 12, },
	{.center_freq = 2472, .hw_value = 13, },
	{.center_freq = 2484, .hw_value = 14, },
};

static struct ieee80211_supported_band esp_wifi_bands = {
	.channels = esp_channels_2ghz,
	.n_channels = ARRAY_SIZE(esp_channels_2ghz),
	.bitrates = esp_rates,
	.n_bitrates = ARRAY_SIZE(esp_rates),
};

/* Supported crypto cipher suits to be advertised to cfg80211 */
static const u32 esp_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_SMS4,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

static const struct wiphy_wowlan_support esp_wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT,
	.n_patterns = 0,
	.pattern_max_len = 0,
	.pattern_min_len = 0,
	.max_pkt_offset = 0,
};

static int esp_inetaddr_event(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct in_ifaddr *ifa = data;
	struct net_device *netdev = ifa->ifa_dev ? ifa->ifa_dev->dev : NULL;
	struct esp_wifi_device *priv = netdev_priv(netdev);

	/*printk(KERN_INFO "------- IP event -------: %d\n", priv->if_type);*/

	if (!strstr(netdev->name, "espsta")) {
		return 0;
	}

	switch (event) {

	case NETDEV_UP:
		if (priv && (priv->if_type == ESP_STA_IF)) {
			cmd_set_ip_address(priv, ifa->ifa_local);
			printk(KERN_INFO "%s: NETDEV_UP interface %s ip changed to  %pi4 \n",
					__func__, netdev->name, &ifa->ifa_local);
		}
		break;

	case NETDEV_DOWN:
		printk(KERN_INFO "Interface Down: %d\n", priv->if_type);
		if (priv && (priv->if_type == ESP_STA_IF))
			cmd_set_ip_address(priv, 0);
		break;
	}

	return 0;
}

struct wireless_dev *esp_cfg80211_add_iface(struct wiphy *wiphy,
		const char *name,
		unsigned char name_assign_type,
		enum nl80211_iftype type,
		struct vif_params *params)
{
	struct esp_device *esp_dev = NULL;
/*	struct wireless_dev *wdev = NULL;*/
	struct net_device *ndev;
	struct esp_wifi_device *esp_wdev;
	uint8_t esp_nw_if_num = 0;

	if (!wiphy || !name) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return NULL;
	}

	esp_dev = wiphy_priv(wiphy);

	if (!esp_dev || !esp_dev->adapter) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return NULL;
	}

	if (NL80211_IFTYPE_STATION == type) {
		esp_nw_if_num = ESP_STA_NW_IF;
	} else if (NL80211_IFTYPE_AP == type) {
		esp_nw_if_num = ESP_AP_NW_IF;
	} else {
		printk(KERN_INFO "%s:%u network type[%u] is not supported\n",
				__func__, __LINE__, type);
		return NULL;
	}

	ndev = ALLOC_NETDEV(sizeof(struct esp_wifi_device), name, name_assign_type,
			ether_setup);

	if (!ndev)
		return ERR_PTR(-ENOMEM);

	set_bit(ESP_DRIVER_ACTIVE, &esp_dev->adapter->state_flags);
	esp_wdev = netdev_priv(ndev);

	ndev->ieee80211_ptr = &esp_wdev->wdev;
	esp_wdev->wdev.wiphy = wiphy;
	esp_wdev->esp_dev = esp_dev;
	esp_wdev->ndev = ndev;
	esp_wdev->adapter = esp_dev->adapter;
	esp_wdev->adapter->priv[esp_nw_if_num] = esp_wdev;
	/*printk(KERN_INFO "Updated priv[%u] to %px\n",
	 * esp_nw_if_num, esp_wdev->adapter->priv[esp_nw_if_num]);*/
	dev_net_set(ndev, wiphy_net(wiphy));
	SET_NETDEV_DEV(ndev, wiphy_dev(esp_wdev->wdev.wiphy));
	esp_wdev->wdev.netdev = ndev;
	esp_wdev->wdev.iftype = type;

	init_waitqueue_head(&esp_wdev->wait_for_scan_completion);
	esp_wdev->stop_data = 1;
	esp_wdev->port_open = 0;

	if (cmd_init_interface(esp_wdev))
		goto free_and_return;

	if (cmd_get_mac(esp_wdev))
		goto free_and_return;

/*	memcpy(ndev->dev_addr, esp_wdev->mac_address, ETH_ALEN);*/
	ether_addr_copy(ndev->dev_addr, esp_wdev->mac_address);

	esp_init_priv(ndev);

	if (register_netdevice(ndev))
		goto free_and_return;


	set_bit(ESP_NETWORK_UP, &esp_wdev->priv_flags);
	clear_bit(ESP_CLEANUP_IN_PROGRESS, &esp_dev->adapter->state_flags);

	esp_wdev->nb.notifier_call = esp_inetaddr_event;
	register_inetaddr_notifier(&esp_wdev->nb);

	return &esp_wdev->wdev;

free_and_return:
	clear_bit(ESP_DRIVER_ACTIVE, &esp_wdev->adapter->state_flags);
	dev_net_set(ndev, NULL);
	free_netdev(ndev);
	esp_wdev->ndev = NULL;
	esp_wdev->wdev.netdev = NULL;
	ndev = NULL;
	return NULL;
}

#if 0
static int esp_cfg80211_del_iface(struct wiphy *wiphy,
							  struct wireless_dev *wdev)
{
	return 0;
}

static int esp_cfg80211_change_iface(struct wiphy *wiphy,
							  struct net_device *ndev,
							  enum nl80211_iftype type,
							  struct vif_params *params)
{
	return 0;
}
#endif

static int esp_cfg80211_scan(struct wiphy *wiphy,
		struct cfg80211_scan_request *request)
{

	struct net_device *ndev = NULL;
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !request || !request->wdev || !request->wdev->netdev) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	ndev = request->wdev->netdev;
	priv = netdev_priv(ndev);

	printk(KERN_INFO "%s\n", __func__);
	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return -EINVAL;
	}

	return cmd_scan_request(priv, request);
}

#if 0
static int esp_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
							  struct cfg80211_connect_params *sme)
{
	struct esp_wifi_device *priv = netdev_priv(dev);

	printk(KERN_INFO "%s\n", __func__);
	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return -EINVAL;
	}

	return cmd_connect_request(priv, sme);
}
#endif

static ESP_MGMT_TX_PROTOTYPE()
{
	return 0;
}

static int esp_cfg80211_set_default_key(struct wiphy *wiphy,
		struct net_device *dev, u8 key_index, bool unicast, bool multicast)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev) {
		printk(KERN_ERR "%s:%u invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	priv = netdev_priv(dev);
	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return -EINVAL;
	}

	return cmd_set_default_key(priv, key_index);
}

static int esp_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
			u8 key_index, bool pairwise, const u8 *mac_addr)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev) {
		printk(KERN_ERR "%s:%u invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	priv = netdev_priv(dev);
	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return -EINVAL;
	}
	printk(KERN_INFO "%s\n", __func__);

	return cmd_del_key(priv, key_index, pairwise, mac_addr);
}

static int esp_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
		 u8 key_index, bool pairwise, const u8 *mac_addr,
		 struct key_params *params)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev || !params) {
		printk(KERN_ERR "%s:%u invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	priv = netdev_priv(dev);
	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return -EINVAL;
	}
	printk(KERN_INFO "%s\n", __func__);

	if (params->key_len == 0)
		return esp_cfg80211_del_key(wiphy, dev, key_index, pairwise, mac_addr);

	return cmd_add_key(priv, key_index, pairwise, mac_addr, params);
}

static int esp_cfg80211_disconnect(struct wiphy *wiphy,
		struct net_device *dev, u16 reason_code)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	priv = netdev_priv(dev);

	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return 0;
	}
	printk(KERN_INFO "%s\n", __func__);

	return cmd_disconnect_request(priv, reason_code);
}

static int esp_cfg80211_authenticate(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_auth_request *req)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev || !req) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	printk(KERN_INFO "%s\n", __func__);

	priv = netdev_priv(dev);

	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return 0;
	}

	return cmd_auth_request(priv, req);
}


static int esp_cfg80211_associate(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_assoc_request *req)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev || !req) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	priv = netdev_priv(dev);

	printk(KERN_INFO "%s\n", __func__);

	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return 0;
	}

	return cmd_assoc_request(priv, req);
}

static int esp_cfg80211_deauth(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_deauth_request *req)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev || !req) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	printk(KERN_INFO "%s\n", __func__);
	priv = netdev_priv(dev);

	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return 0;
	}

	return cmd_disconnect_request(priv, req->reason_code);
}

static int esp_cfg80211_disassoc(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_disassoc_request *req)
{
	struct esp_wifi_device *priv = NULL;

	if (!wiphy || !dev || !req) {
		printk(KERN_INFO "%s:%u invalid input\n", __func__, __LINE__);
		return -EINVAL;
	}

	printk(KERN_INFO "%s\n", __func__);
	priv = netdev_priv(dev);

	if (!priv) {
		printk(KERN_ERR "%s: empty priv\n", __func__);
		return 0;
	}

	return cmd_disconnect_request(priv, req->reason_code);
}

static int esp_cfg80211_suspend(struct wiphy *wiphy,
			struct cfg80211_wowlan *wowlan)
{
	/*printk(KERN_INFO "%s\n", __func__);*/
	return 0;
}

static int esp_cfg80211_resume(struct wiphy *wiphy)
{
	/*printk(KERN_INFO "%s\n", __func__);*/
	return 0;
}

static void esp_cfg80211_set_wakeup(struct wiphy *wiphy,
			bool enabled)
{
	/*printk(KERN_INFO "%s\n", __func__);*/
}

static struct cfg80211_ops esp_cfg80211_ops = {
#if 0
	.add_virtual_intf = esp_cfg80211_add_iface,
	.del_virtual_intf = esp_cfg80211_del_iface,
	.change_virtual_intf = esp_cfg80211_change_iface,
#endif
	.scan = esp_cfg80211_scan,
	/*.connect = esp_cfg80211_connect,*/
	.disconnect = esp_cfg80211_disconnect,
	.add_key = esp_cfg80211_add_key,
	.del_key = esp_cfg80211_del_key,
	.set_default_key = esp_cfg80211_set_default_key,
	.mgmt_tx = esp_cfg80211_mgmt_tx,
	.auth = esp_cfg80211_authenticate,
	.deauth = esp_cfg80211_deauth,
	.disassoc = esp_cfg80211_disassoc,
	.assoc = esp_cfg80211_associate,
	.suspend = esp_cfg80211_suspend,
	.resume = esp_cfg80211_resume,
	.set_wakeup = esp_cfg80211_set_wakeup,
};

int esp_cfg80211_register(struct esp_adapter *adapter)
{
	struct wiphy *wiphy;
	struct esp_device *esp_dev;
	int ret = 0;

	wiphy = wiphy_new(&esp_cfg80211_ops, sizeof(struct esp_device));

	if (!wiphy) {
		printk(KERN_ERR "Failed to create wiphy\n");
		return -EFAULT;
	}

	adapter->wiphy = wiphy;

	esp_dev = wiphy_priv(wiphy);
	esp_dev->wiphy = wiphy;
	esp_dev->adapter = adapter;

	esp_dev->dev = adapter->dev;

	set_wiphy_dev(wiphy, esp_dev->dev);

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wiphy->bands[NL80211_BAND_2GHZ] = &esp_wifi_bands;

	/* Initialize cipher suits */
	wiphy->cipher_suites = esp_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(esp_cipher_suites);

	/* TODO: check and finalize the numbers */
	wiphy->max_scan_ssids = 10;
	/*	wiphy->max_match_sets = 10;*/
	wiphy->max_scan_ie_len = 1000;
	wiphy->max_sched_scan_ssids = 10;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->wowlan = &esp_wowlan_support;

	/* Advertise SAE support */
	wiphy->features |= NL80211_FEATURE_SAE;

	ret = wiphy_register(wiphy);

	return ret;
}

int esp_mark_disconnect(struct esp_wifi_device *priv, uint16_t reason,
		uint8_t locally_disconnect)
{
	if (priv && priv->ndev)
		if (priv->ndev->reg_state == NETREG_REGISTERED)
			CFG80211_DISCONNECTED(priv->ndev, reason, NULL, 0, locally_disconnect,
					GFP_KERNEL);
	return 0;
}

int esp_mark_scan_done_and_disconnect(struct esp_wifi_device *priv,
		uint8_t locally_disconnect)
{

	if (!priv)
		return -EINVAL;

	ESP_MARK_SCAN_DONE(priv, true);

	ESP_CANCEL_SCHED_SCAN();

	esp_mark_disconnect(priv, 0, locally_disconnect);

	return 0;
}
