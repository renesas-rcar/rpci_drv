// SPDX-License-Identifier: GPL-2.0
/*
 * R-Car PCI Interface Endpoint Functions driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/poll.h>

#include "rcar_pci.h"

#define DRV_MODULE_NAME		"rcar-pci-ep"

#define to_rcar_pci_epf_priv(file) container_of((file), struct rcar_pci_epf_private, miscdev)

static DEFINE_IDA(rcar_pci_epf_ida);

static struct workqueue_struct *rcar_pci_epf_workqueue;

struct rcar_pci_epf_private {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	const struct pci_epc_features *epc_features;

	struct miscdevice       miscdev;

	struct completion	tx_completion;
	struct mutex		tx_lock;
	struct queue		*tx_queue;

	struct mutex		rx_lock;
	struct queue		*rx_queue;
	wait_queue_head_t       rx_ioctl_wait;
};

static struct pci_epf_header rcar_pci_epf_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

struct rcar_pci_epf_reg {
	u32 magic;
	u32 command;
	u32 status;
	u64 src_addr;
	u64 dst_addr;
	u32 size;
	u32 tx_size;
	u32 irq_type;
	u32 irq_number;
	u32 flags;
} __packed;

static size_t bar_size[] = {512, 512, 1024, 16384, 131072, 1048576};

static int rcar_pci_epf_write(struct rcar_pci_epf_private *priv)
{
	struct pci_epf *epf = priv->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno reg_bar = priv->reg_bar;
	struct rcar_pci_epf_reg *reg = priv->reg[reg_bar];
	struct rcar_pci_queue_data *data;
	phys_addr_t phys_addr;
	void __iomem *dst_addr;
	int ret;

	reg->tx_size = 0;

	if (!list_empty(&priv->tx_queue->head)) {
		data = list_first_entry(&priv->tx_queue->head,
					struct rcar_pci_queue_data, list);

		reg->tx_size = data->size;
	} else {
		mutex_unlock(&priv->tx_lock);

		return 0;
	}

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->tx_size);
	if (!dst_addr) {
		dev_err(dev, "Failed to allocate address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		ret = -ENOMEM;
		goto error;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr, reg->dst_addr, reg->tx_size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		goto err_addr;
	}

	/* TODO: Support DMA according to EP driver's support status */
	memcpy_toio(dst_addr, data->buff, reg->tx_size);

	dma_free_coherent(data->dev, data->size, data->buff, data->phys_addr);
	list_del(&data->list);

	mutex_unlock(&priv->tx_lock);

	usleep_range(1000, 2000);

	pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, dst_addr, reg->size);

error:
	return ret;
}

static int rcar_pci_epf_read(struct rcar_pci_epf_private *priv)
{
	struct pci_epf *epf = priv->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct device *dma_dev = epc->dev.parent;
	enum pci_barno reg_bar = priv->reg_bar;
	struct rcar_pci_epf_reg *reg = priv->reg[reg_bar];
	phys_addr_t phys_addr;
	struct rcar_pci_queue_data *data;
	void __iomem *src_addr;
	int ret;

	src_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->size);
	if (!src_addr) {
		dev_err(dev, "Failed to allocate address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		ret = -ENOMEM;
		goto error;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr, reg->src_addr, reg->size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		goto err_addr;
	}

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_map_addr;
	}

	data->buff = dma_alloc_coherent(dma_dev, reg->size, &data->phys_addr, GFP_KERNEL);
	if (!data->buff) {
		kfree(data);
		ret = -ENOMEM;
		goto err_map_addr;
	}

	data->dev = dma_dev;
	data->size = reg->size;

	memcpy_fromio(data->buff, src_addr, reg->size);

	mutex_lock(&priv->rx_lock);

	list_add_tail(&data->list, &priv->rx_queue->head);

	wake_up_interruptible(&priv->rx_ioctl_wait);

	mutex_unlock(&priv->rx_lock);

err_map_addr:
	pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, src_addr, reg->size);

error:
	return ret;
}

static void rcar_pci_epf_raise_irq(struct rcar_pci_epf_private *priv, u8 irq_type, u16 irq)
{
	struct pci_epf *epf = priv->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno reg_bar = priv->reg_bar;
	struct rcar_pci_epf_reg *reg = priv->reg[reg_bar];

	reg->status |= STATUS_IRQ_RAISED;

	switch (irq_type) {
	case IRQ_TYPE_LEGACY:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		break;
	case IRQ_TYPE_MSI:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI, irq);
		break;
	case IRQ_TYPE_MSIX:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX, irq);
		break;
	default:
		dev_err(dev, "Failed to raise IRQ, unknown type\n");
		break;
	}
}

static void rcar_pci_epf_cmd_handler(struct work_struct *work)
{
	struct rcar_pci_epf_private *priv = container_of(work, struct rcar_pci_epf_private,
							 cmd_handler.work);
	struct pci_epf *epf = priv->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno reg_bar = priv->reg_bar;
	struct rcar_pci_epf_reg *reg = priv->reg[reg_bar];
	int ret, count;
	u32 command;

	command = reg->command;
	if (!command)
		goto reset_handler;

	reg->command = 0;
	reg->status = 0;

	if (reg->irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_LEGACY_IRQ) {
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		goto reset_handler;
	}

	if (command & COMMAND_WRITE) {
		ret = rcar_pci_epf_write(priv);
		if (ret)
			reg->status |= STATUS_WRITE_FAILED;
		else
			reg->status |= STATUS_WRITE_SUCCESS;

		rcar_pci_epf_raise_irq(priv, reg->irq_type, reg->irq_number);

		goto reset_handler;
	}

	if (command & COMMAND_READ) {
		ret = rcar_pci_epf_read(priv);
		if (ret)
			reg->status |= STATUS_READ_FAILED;
		else
			reg->status |= STATUS_READ_SUCCESS;

		rcar_pci_epf_raise_irq(priv, reg->irq_type, reg->irq_number);

		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSI_IRQ) {
		count = pci_epc_get_msi(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;

		reg->status = STATUS_IRQ_RAISED;

		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI, reg->irq_number);

		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSIX_IRQ) {
		count = pci_epc_get_msix(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;

		reg->status = STATUS_IRQ_RAISED;

		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX, reg->irq_number);

		goto reset_handler;
	}

	if (command & COMMAND_EP_TX_DONE) {
		complete(&priv->tx_completion);

		goto reset_handler;
	}

reset_handler:
	queue_delayed_work(rcar_pci_epf_workqueue, &priv->cmd_handler, msecs_to_jiffies(1));
}

static int rcar_pci_epf_set_bar(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno reg_bar = priv->reg_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	int bar, add, ret;

	epc_features = priv->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];

		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf_bar);
		if (ret) {
			pci_epf_free_space(epf, priv->reg[bar], bar);
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == reg_bar)
				return ret;
		}
	}

	return 0;
}

static int rcar_pci_epf_core_init(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool msix_capable = false;
	bool msi_capable = true;
	int ret;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
	}

	ret = pci_epc_write_header(epc, epf->func_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}

	ret = rcar_pci_epf_set_bar(epf);
	if (ret)
		return ret;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->msix_interrupts,
					    priv->reg_bar, priv->msix_table_offset);

		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	return 0;
}

static int rcar_pci_epf_notifier(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = rcar_pci_epf_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;
	case LINK_UP:
		queue_delayed_work(rcar_pci_epf_workqueue, &priv->cmd_handler,
				   msecs_to_jiffies(1));
		break;
	default:
		dev_err(&epf->dev, "Invalid PCI-EPF notifier event\n");

		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int rcar_pci_epf_alloc_space(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno reg_bar = priv->reg_bar;
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	size_t msix_table_size = 0;
	size_t reg_bar_size;
	size_t pba_size = 0;
	size_t reg_size;
	bool msix_capable;
	int bar, add;
	void *base;

	epc_features = priv->epc_features;

	reg_bar_size = ALIGN(sizeof(struct rcar_pci_epf_reg), 128);

	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		priv->msix_table_offset = reg_bar_size;

		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}

	reg_size = reg_bar_size + msix_table_size + pba_size;

	if (epc_features->bar_fixed_size[reg_bar]) {
		if (reg_size > bar_size[reg_bar])
			return -ENOMEM;
		reg_size = bar_size[reg_bar];
	}

	base = pci_epf_alloc_space(epf, reg_size, reg_bar, epc_features->align);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}

	priv->reg[reg_bar] = base;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (bar == reg_bar)
			continue;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		base = pci_epf_alloc_space(epf, bar_size[bar], bar, epc_features->align);
		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n", bar);

		priv->reg[bar] = base;
	}

	return 0;
}

static void rcar_pci_epf_configure_bar(struct pci_epf *epf,
				       const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			bar_size[i] = epc_features->bar_fixed_size[i];
	}
}

static int rcar_pci_epf_tx(struct rcar_pci_epf_private *priv, void *buff, size_t size)
{
	struct device *dma_dev = priv->epf->epc->dev.parent;
	enum pci_barno reg_bar = priv->reg_bar;
	struct rcar_pci_epf_reg *reg = priv->reg[reg_bar];
	struct rcar_pci_queue_data *data;
	dma_addr_t phys_addr;
	u32 val;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	data->size = size;

	data->buff = dma_alloc_coherent(dma_dev, size, &phys_addr, GFP_KERNEL);
	if (!data->buff) {
		kfree(data);
		return -ENOMEM;
	}

	data->dev = dma_dev;
	data->phys_addr = phys_addr;

	ret = copy_from_user(data->buff, buff, size);
	if (ret)
		return ret;

	reg->tx_size = size;

	mutex_lock(&priv->tx_lock);

	list_add_tail(&data->list, &priv->tx_queue->head);

	mutex_unlock(&priv->tx_lock);

	rcar_pci_epf_raise_irq(priv, IRQ_TYPE_MSI, IRQ_RC_RX);

	val = wait_for_completion_timeout(&priv->tx_completion, msecs_to_jiffies(100));
	if (!val)
		return -ETIMEDOUT;

	return 0;
}

static int rcar_pci_epf_has_rx_data(struct rcar_pci_epf_private *priv, unsigned long arg)
{
	struct rcar_pci_queue_data *data;
	size_t size = 0;
	int ret;

	mutex_lock(&priv->rx_lock);

	if (!list_empty(&priv->rx_queue->head)) {
		data = list_first_entry(&priv->rx_queue->head, struct rcar_pci_queue_data, list);
		size = data->size;
	}

	ret = copy_to_user((char __user *)arg, &size, sizeof(size));

	mutex_unlock(&priv->rx_lock);

	return ret;
}

static int rcar_pci_epf_receive(struct rcar_pci_epf_private *priv, unsigned long arg)
{
	struct rcar_pci_queue_data *data;
	struct rcar_pci_xfer_buff buff;
	int ret;

	mutex_lock(&priv->rx_lock);

	if (!list_empty(&priv->rx_queue->head)) {
		data = list_first_entry(&priv->rx_queue->head, struct rcar_pci_queue_data, list);

		ret = copy_from_user(&buff, (char __user *)arg, sizeof(buff));
		if (ret)
			goto error;

		ret = copy_to_user(buff.buffer, data->buff, buff.size);
		if (ret)
			goto error;

		dma_free_coherent(data->dev, data->size, data->buff, data->phys_addr);

		list_del(&data->list);
	}
error:
	mutex_unlock(&priv->rx_lock);

	return ret;
}

static long rcar_pci_epf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rcar_pci_epf_private *priv = to_rcar_pci_epf_priv(file->private_data);
	struct rcar_pci_xfer_buff buff;
	int ret;

	switch (cmd) {
	case RCAR_PCI_HAS_RX_DATA:
		return rcar_pci_epf_has_rx_data(priv, arg);
	case RCAR_PCI_RECEIVE:
		return rcar_pci_epf_receive(priv, arg);
	case RCAR_PCI_XMIT:
		ret = copy_from_user(&buff, (char __user *)arg, sizeof(buff));
		if (ret)
			return ret;

		if (!buff.size)
			return -EINVAL;

		ret = rcar_pci_epf_tx(priv, buff.buffer, buff.size);
		if (ret)
			return ret;

		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static __poll_t rcar_pci_epf_ioctl_poll(struct file *file, struct poll_table_struct *wait)
{
	struct rcar_pci_epf_private *priv = to_rcar_pci_epf_priv(file->private_data);
	__poll_t events = 0;

	poll_wait(file, &priv->rx_ioctl_wait, wait);

	mutex_lock(&priv->rx_lock);

	if (!list_empty(&priv->rx_queue->head))
		events = POLLIN | POLLRDNORM;

	mutex_unlock(&priv->rx_lock);

	return events;
}

static const struct file_operations rcar_pci_epf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rcar_pci_epf_ioctl,
	.poll = rcar_pci_epf_ioctl_poll,
};

static int rcar_pci_epf_bind(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	linkup_notifier = epc_features->linkup_notifier;
	core_init_notifier = epc_features->core_init_notifier;

	reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (reg_bar < 0)
		return -EINVAL;

	rcar_pci_epf_configure_bar(epf, epc_features);
	priv->reg_bar = reg_bar;
	priv->epc_features = epc_features;

	ret = rcar_pci_epf_alloc_space(epf);
	if (ret)
		return ret;

	if (!core_init_notifier) {
		ret = rcar_pci_epf_core_init(epf);
		if (ret)
			return ret;
	}

	/* TODO: DMA support */

	if (linkup_notifier) {
		epf->nb.notifier_call = rcar_pci_epf_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(rcar_pci_epf_workqueue, &priv->cmd_handler.work);
	}

	return 0;
}

static void rcar_pci_epf_unbind(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	cancel_delayed_work(&priv->cmd_handler);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (priv->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf_bar);
			pci_epf_free_space(epf, priv->reg[bar], bar);
		}
	}

}

static int rcar_pci_epf_probe(struct pci_epf *epf)
{
	struct rcar_pci_epf_private *priv;
	struct device *dev = &epf->dev;
	struct miscdevice *misc_device;
	char name[24];
	int id, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	epf->header = &rcar_pci_epf_header;
	priv->epf = epf;

	INIT_DELAYED_WORK(&priv->cmd_handler, rcar_pci_epf_cmd_handler);

	epf_set_drvdata(epf, priv);

	id = ida_simple_get(&rcar_pci_epf_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		dev_err(dev, "Unable to get id\n");
		return id;
	}

	snprintf(name, sizeof(name), DRV_MODULE_NAME "%d", id);
	misc_device = &priv->miscdev;
	misc_device->minor = MISC_DYNAMIC_MINOR;
	misc_device->name = kstrdup(name, GFP_KERNEL);
	if (!misc_device->name) {
		ret = -ENOMEM;
		goto err_ida_remove;
	}

	misc_device->fops = &rcar_pci_epf_fops,

	ret = misc_register(misc_device);
	if (ret) {
		dev_err(dev, "Failed to register device\n");
		kfree(misc_device->name);
		goto err_free_name;
	}

	init_completion(&priv->tx_completion);
	mutex_init(&priv->tx_lock);

	priv->tx_queue = kmalloc(sizeof(struct queue *), GFP_KERNEL);
	if (!priv->tx_queue) {
		ret = -ENOMEM;
		goto err_free_name;
	}

	INIT_LIST_HEAD(&priv->tx_queue->head);

	priv->rx_queue = kmalloc(sizeof(struct queue *), GFP_KERNEL);
	if (!priv->rx_queue) {
		ret = -ENOMEM;
		goto err_free_tx_queue;
	}

	INIT_LIST_HEAD(&priv->rx_queue->head);

	init_waitqueue_head(&priv->rx_ioctl_wait);

	return 0;

err_free_tx_queue:
	kfree(priv->tx_queue);

err_free_name:
	kfree(misc_device->name);

err_ida_remove:
	ida_simple_remove(&rcar_pci_epf_ida, id);

	return ret;
}

static struct pci_epf_ops ops = {
	.bind	= rcar_pci_epf_bind,
	.unbind	= rcar_pci_epf_unbind,
};

static const struct pci_epf_device_id rcar_pci_epf_ids[] = {
	{
		.name = "rcar_pci_epf",
	},
	{},
};

static struct pci_epf_driver rcar_pci_epf_drv = {
	.driver.name	= "rcar_pci_epf",
	.probe		= rcar_pci_epf_probe,
	.id_table	= rcar_pci_epf_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init rcar_pci_epf_init(void)
{
	int ret;

	rcar_pci_epf_workqueue = alloc_workqueue("rcar_pci_epf", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!rcar_pci_epf_workqueue) {
		pr_err("Failed to allocate the rcar_pci_epf work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&rcar_pci_epf_drv);
	if (ret) {
		destroy_workqueue(rcar_pci_epf_workqueue);
		pr_err("Failed to register rcar_pci_epf_drv --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(rcar_pci_epf_init);

static void __exit rcar_pci_epf_exit(void)
{
	if (rcar_pci_epf_workqueue)
		destroy_workqueue(rcar_pci_epf_workqueue);
	pci_epf_unregister_driver(&rcar_pci_epf_drv);
}
module_exit(rcar_pci_epf_exit);

MODULE_DESCRIPTION("R-Car PCI API ENDPOINT FUNCTIONS DRIVER");
MODULE_AUTHOR("Phong Hoang <phong.hoang.wz@renesas.com>");
MODULE_LICENSE("GPL v2");
