/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
#include "esp_bt_api.h"
#include "esp_api.h"

#define INVALID_HDEV_BUS (0xff)

void esp_hci_update_tx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len)
{
	if (hdev) {
		if (pkt_type == HCI_COMMAND_PKT) {
			hdev->stat.cmd_tx++;
		} else if (pkt_type == HCI_ACLDATA_PKT) {
			hdev->stat.acl_tx++;
		} else if (pkt_type == HCI_SCODATA_PKT) {
			hdev->stat.sco_tx++;
		}

		hdev->stat.byte_tx += len;
	}
}

void esp_hci_update_rx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len)
{
	if (hdev) {
		if (pkt_type == HCI_EVENT_PKT) {
			hdev->stat.evt_rx++;
		} else if (pkt_type == HCI_ACLDATA_PKT) {
			hdev->stat.acl_rx++;
		} else if (pkt_type == HCI_SCODATA_PKT) {
			hdev->stat.sco_rx++;
		}

		hdev->stat.byte_rx += len;
	}
}

static int esp_bt_open(struct hci_dev *hdev)
{
	return 0;
}

static int esp_bt_close(struct hci_dev *hdev)
{
	return 0;
}

static int esp_bt_flush(struct hci_dev *hdev)
{
	return 0;
}

static int esp_bt_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct esp_payload_header *hdr;
	size_t total_len, len = skb->len;
	int ret = 0;
	struct esp_adapter *adapter = hci_get_drvdata(hdev);
	struct sk_buff *new_skb;

	if (!adapter) {
		printk(KERN_ERR "%s: invalid args", __func__);
		return -EINVAL;
	}

	total_len = len + sizeof(struct esp_payload_header);

	if (skb_headroom(skb) < sizeof(struct esp_payload_header)) {
		/* insufficent headroom to add payload header */
		new_skb = skb_realloc_headroom(skb, sizeof(struct esp_payload_header));

		if(!new_skb) {
			printk(KERN_ERR "%s: Failed to allocate SKB", __func__);
			dev_kfree_skb(skb);
			hdev->stat.err_tx++;
			return -ENOMEM;
		}

		dev_kfree_skb(skb);

		skb = new_skb;
	}

	skb_push(skb, sizeof(struct esp_payload_header));

	hdr = (struct esp_payload_header *) skb->data;

	memset (hdr, 0, sizeof(struct esp_payload_header));

	hdr->if_type = ESP_HCI_IF;
	hdr->if_num = 0;
	hdr->len = cpu_to_le16(len);
	hdr->offset = cpu_to_le16(sizeof(struct esp_payload_header));
	hdr->hci_pkt_type = hci_skb_pkt_type(skb);

	ret = esp_send_packet(adapter, skb->data, skb->len);

	if (ret) {
		hdev->stat.err_tx++;
	} else {
		esp_hci_update_tx_counter(hdev, hdr->hci_pkt_type, skb->len);
	}

	dev_kfree_skb(skb);

	return 0;
}

static int esp_bt_setup(struct hci_dev *hdev)
{
	return 0;
}

static int esp_bt_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	return 0;
}

int esp_deinit_bt(struct esp_adapter *adapter)
{
	struct hci_dev *hdev = NULL;

	if (!adapter || !adapter->hcidev)
		return 0;

	hdev = adapter->hcidev;

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);

	adapter->hcidev = NULL;

	return 0;
}

int esp_init_bt(struct esp_adapter *adapter)
{
	int ret = 0;
	struct hci_dev *hdev = NULL;

	if (!adapter) {
		return -EINVAL;
	}

	if (adapter->hcidev) {
		return -EEXIST;
	}

	hdev = hci_alloc_dev();

	if (!hdev) {
		BT_ERR("Can not allocate HCI device");
		return -ENOMEM;
	}

	adapter->hcidev = hdev;
	hci_set_drvdata(hdev, adapter);

	hdev->bus = INVALID_HDEV_BUS;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19))
	if (adapter->if_type == ESP_IF_TYPE_SDIO)
		hdev->bus   = HCI_SDIO;
  #if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	else if (adapter->if_type == ESP_IF_TYPE_SPI)
		hdev->bus   = HCI_SPI;
  #endif
#endif

	if (hdev->bus == INVALID_HDEV_BUS) {

		if (adapter->if_type == ESP_IF_TYPE_SDIO) {
			printk(KERN_ERR "%s: Kernel version does not support HCI over SDIO BUS\n",__func__);
		} else if (adapter->if_type == ESP_IF_TYPE_SPI) {
			printk(KERN_ERR "%s: Kernel version does not support HCI over SPI BUS\n",__func__);
		} else {
			printk(KERN_ERR "%s: HCI over expected BUS[%u] is not supported\n",__func__, adapter->if_type);
		}
		hci_free_dev(hdev);
		adapter->hcidev = NULL;
		return -EINVAL;
	}

	hdev->open  = esp_bt_open;
	hdev->close = esp_bt_close;
	hdev->flush = esp_bt_flush;
	hdev->send  = esp_bt_send_frame;
	hdev->setup = esp_bt_setup;
	hdev->set_bdaddr = esp_bt_set_bdaddr;

	hdev->dev_type = HCI_PRIMARY;

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		BT_ERR("Can not register HCI device");
		hci_free_dev(hdev);
		return -ENOMEM;
	}

	return 0;
}
