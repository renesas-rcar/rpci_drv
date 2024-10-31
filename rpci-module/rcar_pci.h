/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/**
 * rcar_pci.h - R-Car PCI API
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */

#ifndef __RCAR_PCI_H
#define __RCAR_PCI_H

#include <linux/list.h>

#define RCAR_PCI_COMMAND                0x04
#define COMMAND_RAISE_LEGACY_IRQ        BIT(0)
#define COMMAND_RAISE_MSI_IRQ           BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ                    BIT(3)
#define COMMAND_WRITE                   BIT(4)
#define COMMAND_EP_TX_DONE		BIT(5)

#define RCAR_PCI_STATUS                 0x08
#define STATUS_READ_SUCCESS             BIT(0)
#define STATUS_READ_FAILED              BIT(1)
#define STATUS_WRITE_SUCCESS            BIT(2)
#define STATUS_WRITE_FAILED             BIT(3)
#define STATUS_IRQ_RAISED               BIT(6)
#define STATUS_SRC_ADDR_INVALID         BIT(7)
#define STATUS_DST_ADDR_INVALID         BIT(8)

#define LOWER_SRC_ADDR                  0x0c
#define UPPER_SRC_ADDR                  0x10
#define LOWER_DST_ADDR                  0x14
#define UPPER_DST_ADDR                  0x18

#define XFER_SIZE                       0x1c
#define RC_RX_SIZE			0x20

#define IRQ_TYPE                        0x24
#define IRQ_TYPE_LEGACY                 0
#define IRQ_TYPE_MSI                    1
#define IRQ_TYPE_MSIX                   2

#define IRQ_NUM                         0x28
#define IRQ_XFER			0x01
#define IRQ_RC_RX			0x02

struct rcar_pci_queue_data {
	struct device *dev;
	dma_addr_t phys_addr;
	void *buff;
	size_t size;

	struct list_head list;
};

struct queue {
	struct list_head head;
};


/* ioctl */
#define RCAR_PCI_HAS_RX_DATA		_IOW('V', 0, size_t)
#define RCAR_PCI_RECEIVE		_IOW('V', 1, struct rcar_pci_xfer_buff)
#define RCAR_PCI_XMIT			_IOW('V', 2, struct rcar_pci_xfer_buff)

struct rcar_pci_xfer_buff {
	void *buffer;
	size_t size;
};

#endif /* __RCAR_PCI_H */
