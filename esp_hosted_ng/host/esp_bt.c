// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include "esp_api.h"
#include "esp_kernel_port.h"
#include "esp_bt_api.h"

#define INVALID_HDEV_BUS (0xff)

void esp_hci_update_tx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len)
{
	if (!hdev)
		return;
	if (pkt_type == HCI_COMMAND_PKT) {
		hdev->stat.cmd_tx++;
	} else if (pkt_type == HCI_ACLDATA_PKT) {
		hdev->stat.acl_tx++;
	} else if (pkt_type == HCI_SCODATA_PKT) {
		hdev->stat.sco_tx++;
	}

	hdev->stat.byte_tx += len;
}

void esp_hci_update_rx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len)
{
	if (!hdev)
		return;

	if (pkt_type == HCI_EVENT_PKT) {
		hdev->stat.evt_rx++;
	} else if (pkt_type == HCI_ACLDATA_PKT) {
		hdev->stat.acl_rx++;
	} else if (pkt_type == HCI_SCODATA_PKT) {
		hdev->stat.sco_rx++;
	}

	hdev->stat.byte_rx += len;
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

static ESP_BT_SEND_FRAME_PROTOTYPE()
{
	struct esp_payload_header *hdr;
	size_t total_len, len = skb->len;
	int ret = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
	struct hci_dev *hdev = (struct hci_dev *)(skb->dev);
#endif
	struct esp_adapter *adapter = hci_get_drvdata(hdev);
	struct sk_buff *new_skb;
	u8 pad_len = 0, realloc_skb = 0;
	u8 *pos = NULL;
	u8 pkt_type;

	if (!adapter) {
		esp_err("Invalid args");
		return -EINVAL;
	}
	esp_hex_dump_verbose("bt_tx: ", skb->data, len);

	/* Create space for payload header */
	pad_len = sizeof(struct esp_payload_header);
	total_len = len + sizeof(struct esp_payload_header);

	/* Align buffer len */
	pad_len += SKB_DATA_ADDR_ALIGNMENT - (total_len % SKB_DATA_ADDR_ALIGNMENT);

	pkt_type = hci_skb_pkt_type(skb);

	if (skb_headroom(skb) < pad_len) {
		/* Headroom is not sufficient */
		realloc_skb = 1;
	}

	if (realloc_skb || !IS_ALIGNED((unsigned long) skb->data, SKB_DATA_ADDR_ALIGNMENT)) {
		/* Realloc SKB */
		if (skb_linearize(skb)) {
			hdev->stat.err_tx++;
			return -EINVAL;
		}

		new_skb = esp_alloc_skb(skb->len + pad_len);

		if (!new_skb) {
			esp_err("Failed to allocate SKB");
			hdev->stat.err_tx++;
			return -ENOMEM;
		}

		pos = new_skb->data;

		pos += pad_len;

		/* Populate new SKB */
		skb_copy_from_linear_data(skb, pos, skb->len);
		skb_put(new_skb, skb->len);

		/* Replace old SKB */
		dev_kfree_skb_any(skb);
		skb = new_skb;
	} else {
		/* Realloc is not needed, Make space for interface header */
		skb_push(skb, pad_len);
	}

	hdr = (struct esp_payload_header *) skb->data;

	memset(hdr, 0, sizeof(struct esp_payload_header));

	hdr->if_type = ESP_HCI_IF;
	hdr->if_num = 0;
	hdr->len = cpu_to_le16(len);
	hdr->offset = cpu_to_le16(pad_len);
	pos = skb->data;

	/* set HCI packet type */
	*(pos + pad_len - 1) = pkt_type;

	if (adapter->capabilities & ESP_CHECKSUM_ENABLED)
		hdr->checksum = cpu_to_le16(compute_checksum(skb->data, (len + pad_len)));

	ret = esp_send_packet(adapter, skb);

	if (ret) {
		hdev->stat.err_tx++;
		return ret;
	} else {
		esp_hci_update_tx_counter(hdev, hdr->hci_pkt_type, skb->len);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
static int esp_bt_setup(struct hci_dev *hdev)
{
	return 0;
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0))
static int esp_bt_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	return 0;
}
#endif

int esp_deinit_bt(struct esp_adapter *adapter)
{
	struct hci_dev *hdev = NULL;

	if (!adapter || !adapter->hcidev)
		return 0;

	hdev = adapter->hcidev;

	hci_set_drvdata(hdev, NULL);
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

	if (adapter->if_type == ESP_IF_TYPE_SDIO)
		hdev->bus   = HCI_SDIO;
    #if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	else if (adapter->if_type == ESP_IF_TYPE_SPI)
		hdev->bus   = HCI_SPI;
    #endif

	if (hdev->bus == INVALID_HDEV_BUS) {

		if (adapter->if_type == ESP_IF_TYPE_SDIO) {
			esp_err("Kernel version does not support HCI over SDIO BUS\n");
		} else if (adapter->if_type == ESP_IF_TYPE_SPI) {
			esp_err("Kernel version does not support HCI over SPI BUS\n");
		} else {
			esp_err("HCI over expected BUS[%u] is not supported\n", adapter->if_type);
		}
		hci_free_dev(hdev);
		adapter->hcidev = NULL;
		return -EINVAL;
	}

	hdev->open  = esp_bt_open;
	hdev->close = esp_bt_close;
	hdev->flush = esp_bt_flush;
	hdev->send  = esp_bt_send_frame;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
	hdev->setup = esp_bt_setup;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0))
	hdev->set_bdaddr = esp_bt_set_bdaddr;
#endif

#ifdef HCI_PRIMARY
	hdev->dev_type = HCI_PRIMARY;
#endif

	if (adapter->dev)
		SET_HCIDEV_DEV(hdev, adapter->dev);

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		BT_ERR("Can not register HCI device");
		hci_free_dev(hdev);
		return -ENOMEM;
	}

	return 0;
}
