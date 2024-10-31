// SPDX-License-Identifier: GPL-2.0
/*
 * R-Car PCI Interface Root complex driver
 *
 * Copyright (C) 2024 Renesas Electronics Corporation
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/poll.h>

#include "rcar_pci.h"

#define PCI_DEVICE_ID_RENESAS_R8A779F0		0x0031
#define PCI_DEVICE_ID_RENESAS_R8A779G0		0x0030

#define DRV_MODULE_NAME "rcar-pci-rc"

#define to_rcar_pci_rc_priv(file) container_of((file), struct rcar_pci_rc_private, miscdev)

static DEFINE_IDA(rcar_pci_rc_ida);

struct rcar_pci_rc_private {
	struct pci_dev		*pdev;
	void __iomem		*base;
	void __iomem		*bar[PCI_STD_NUM_BARS];
	struct completion	irq_raised;
	int			irq_rx;
	int			num_irqs;
	struct mutex		mutex;
	struct miscdevice	miscdev;
	const char		*name;

	struct workqueue_struct *rx_workqueue;
	struct delayed_work	rx_handler;

	struct mutex		rx_lock;
	struct queue		*rx_queue;
	wait_queue_head_t	rx_ioctl_wait;
};

static inline u32 rcar_pci_rc_readl(struct rcar_pci_rc_private *priv, u32 offset)
{
	return readl(priv->base + offset);
}

static inline void rcar_pci_rc_writel(struct rcar_pci_rc_private *priv, u32 offset, u32 val)
{
	writel(val, priv->base + offset);
}

static irqreturn_t rcar_pci_rc_irqhandler(int irq, void *dev_id)
{
	struct rcar_pci_rc_private *priv = dev_id;
	u32 reg;

	if (irq == priv->irq_rx) {
		queue_work(priv->rx_workqueue, &priv->rx_handler.work);
	} else {
		reg = rcar_pci_rc_readl(priv, RCAR_PCI_STATUS);
		if (reg & STATUS_IRQ_RAISED) {
			complete(&priv->irq_raised);
			reg &= ~STATUS_IRQ_RAISED;
		}

		rcar_pci_rc_writel(priv, RCAR_PCI_STATUS, reg);
	}

	return IRQ_HANDLED;
}

static void rcar_pci_rc_release_irq(struct rcar_pci_rc_private *priv)
{
	struct pci_dev *pdev = priv->pdev;
	struct device *dev = &pdev->dev;
	int i;

	for (i = 0; i < priv->num_irqs; i++)
		devm_free_irq(dev, pci_irq_vector(pdev, i), priv);

	priv->num_irqs = 0;
}

static int rcar_pci_rc_trigger_ep_send(struct rcar_pci_rc_private *priv, size_t size)
{
	struct pci_dev *pdev = priv->pdev;
	struct device *dev = &pdev->dev;
	struct rcar_pci_queue_data *data;
	dma_addr_t phys_addr;
	void *virt_addr;
	u32 reg;
	int ret;

	virt_addr = dma_alloc_coherent(dev, size, &phys_addr, GFP_KERNEL);
	if (!virt_addr) {
		dev_err(dev, "Failed to allocate address\n");
		return -ENOMEM;
	}

	rcar_pci_rc_writel(priv, LOWER_DST_ADDR, lower_32_bits(phys_addr));
	rcar_pci_rc_writel(priv, UPPER_DST_ADDR, upper_32_bits(phys_addr));

	rcar_pci_rc_writel(priv, XFER_SIZE, size);
	rcar_pci_rc_writel(priv, IRQ_TYPE, IRQ_TYPE_MSI);
	rcar_pci_rc_writel(priv, IRQ_NUM, IRQ_XFER);
	rcar_pci_rc_writel(priv, RCAR_PCI_COMMAND, COMMAND_WRITE);

	wait_for_completion(&priv->irq_raised);

	rcar_pci_rc_writel(priv, RCAR_PCI_COMMAND, COMMAND_EP_TX_DONE);

	reg = rcar_pci_rc_readl(priv, RCAR_PCI_STATUS);
	if (!(reg & STATUS_WRITE_SUCCESS)) {
		ret = -ENODATA;
		goto err_dma_free;
	}

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_dma_free;
	}

	data->dev = dev;
	data->phys_addr = phys_addr;
	data->buff = virt_addr;
	data->size = size;

	mutex_lock(&priv->rx_lock);

	list_add_tail(&data->list, &priv->rx_queue->head);

	mutex_unlock(&priv->rx_lock);

	return 0;

err_dma_free:
	dma_free_coherent(dev, size, virt_addr, phys_addr);

	return ret;
}

static int rcar_pci_rc_xmit(struct rcar_pci_rc_private *priv, unsigned long arg)
{
	struct pci_dev *pdev = priv->pdev;
	struct device *dev = &pdev->dev;
	struct rcar_pci_xfer_buff buff;
	dma_addr_t phys_addr;
	void *virt_addr;
	u32 reg;
	int ret;

	ret = copy_from_user(&buff, (char __user *)arg, sizeof(buff));
	if (ret)
		return ret;

	if (!buff.size)
		return -EINVAL;

	virt_addr = dma_alloc_coherent(dev, buff.size, &phys_addr, GFP_KERNEL);
	if (!virt_addr)
		return -ENOMEM;

	ret = copy_from_user(virt_addr, buff.buffer, buff.size);
	if (ret)
		goto error;

	rcar_pci_rc_writel(priv, LOWER_SRC_ADDR, lower_32_bits(phys_addr));
	rcar_pci_rc_writel(priv, UPPER_SRC_ADDR, upper_32_bits(phys_addr));

	rcar_pci_rc_writel(priv, XFER_SIZE, buff.size);
	rcar_pci_rc_writel(priv, IRQ_TYPE, IRQ_TYPE_MSI);
	rcar_pci_rc_writel(priv, IRQ_NUM, IRQ_XFER);

	rcar_pci_rc_writel(priv, RCAR_PCI_COMMAND, COMMAND_READ);

	wait_for_completion(&priv->irq_raised);

	reg = rcar_pci_rc_readl(priv, RCAR_PCI_STATUS);
	if (!(reg & STATUS_READ_SUCCESS))
		return -EIO;

error:
	dma_free_coherent(dev, buff.size, virt_addr, phys_addr);

	return ret;
}

static void rcar_pci_rc_rx_handler(struct work_struct *work)
{
	struct rcar_pci_rc_private *priv = container_of(work, struct rcar_pci_rc_private,
							rx_handler.work);
	u32 size;
	int ret;

	size = rcar_pci_rc_readl(priv, RC_RX_SIZE);
	rcar_pci_rc_writel(priv, RC_RX_SIZE, 0);

	ret = rcar_pci_rc_trigger_ep_send(priv, size);
	if (ret)
		return;

	wake_up_interruptible(&priv->rx_ioctl_wait);
}

static int rcar_pci_rc_has_rx_data(struct rcar_pci_rc_private *priv, unsigned long arg)
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

static int rcar_pci_rc_receive(struct rcar_pci_rc_private *priv, unsigned long arg)
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

static long rcar_pci_rc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rcar_pci_rc_private *priv = to_rcar_pci_rc_priv(file->private_data);

	switch (cmd) {
	case RCAR_PCI_HAS_RX_DATA:
		return rcar_pci_rc_has_rx_data(priv, arg);
	case RCAR_PCI_RECEIVE:
		return rcar_pci_rc_receive(priv, arg);
	case RCAR_PCI_XMIT:
		return rcar_pci_rc_xmit(priv, arg);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static __poll_t rcar_pci_rc_ioctl_poll(struct file *file, struct poll_table_struct *wait)
{
	struct rcar_pci_rc_private *priv = to_rcar_pci_rc_priv(file->private_data);
	__poll_t events = 0;

	poll_wait(file, &priv->rx_ioctl_wait, wait);

	mutex_lock(&priv->rx_lock);

	if (!list_empty(&priv->rx_queue->head))
		events = POLLIN | POLLRDNORM;

	mutex_unlock(&priv->rx_lock);

	return events;
}


static const struct file_operations rcar_pci_rc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rcar_pci_rc_ioctl,
	.poll = rcar_pci_rc_ioctl_poll,
};

static int rcar_pci_rc_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct rcar_pci_rc_private *priv;
	struct device *dev = &pdev->dev;
	struct miscdevice *misc_device;
	int ret, i, id, irq;
	void __iomem *base;
	char name[24];

	if (pci_is_bridge(pdev))
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	init_completion(&priv->irq_raised);
	mutex_init(&priv->mutex);

	if ((dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40)) != 0) &&
	    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		dev_err(dev, "Cannot set DMA mask\n");
		return -EINVAL;
	}

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	ret = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (ret)
		goto err_disable_pdev;

	pci_set_master(pdev);

	irq = pci_alloc_irq_vectors(pdev, 1, 32, PCI_IRQ_MSI);
	if (irq < 0) {
		dev_err(dev, "failed to get MSI interrupts\n");
		ret = -EINVAL;
		goto err_disable_irq;
	}

	priv->num_irqs = irq;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		if (pci_resource_flags(pdev, i) & IORESOURCE_MEM) {
			base = pci_ioremap_bar(pdev, i);
			if (!base)
				dev_err(dev, "Failed to read BAR%d\n", i);
		}
		priv->bar[i] = base;
	}

	/* Fixed to use BAR0 */
	priv->base = priv->bar[0];
	if (!priv->base) {
		ret = -ENOMEM;
		dev_err(dev, "Cannot perform PCI without BAR0");

		goto err_iounmap;
	}

	pci_set_drvdata(pdev, priv);

	id = ida_simple_get(&rcar_pci_rc_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		dev_err(dev, "unable to get id\n");
		goto err_iounmap;
	}

	snprintf(name, sizeof(name), DRV_MODULE_NAME "%d", id);
	priv->name = kstrdup(name, GFP_KERNEL);
	if (!priv->name) {
		ret = -ENOMEM;
		goto err_ida_remove;
	}

	for (i = 0; i < irq; i++) {
		ret = devm_request_irq(dev, pci_irq_vector(pdev, i),
				       rcar_pci_rc_irqhandler, IRQF_SHARED,
				       priv->name, priv);
		if (ret) {
			ret = -EINVAL;
			goto err_kfree_priv_name;
		}
	}

	priv->irq_rx = pci_irq_vector(pdev, 0) + IRQ_RC_RX - 1;

	misc_device = &priv->miscdev;
	misc_device->minor = MISC_DYNAMIC_MINOR;
	misc_device->name = kstrdup(name, GFP_KERNEL);
	if (!misc_device->name) {
		ret = -ENOMEM;
		goto err_release_irq;
	}

	misc_device->fops = &rcar_pci_rc_fops;

	ret = misc_register(misc_device);
	if (ret)
		goto err_kfree_name;

	priv->rx_workqueue = alloc_workqueue("rcar-pci-rc-rx-wq", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!priv->rx_workqueue) {
		dev_err(dev, "Failed to allocate the RX workqueue\n");
		ret = -ENOMEM;
		goto err_kfree_name;
	}

	INIT_DELAYED_WORK(&priv->rx_handler, rcar_pci_rc_rx_handler);

	priv->rx_queue = kmalloc(sizeof(struct queue *), GFP_KERNEL);
	if (!priv->rx_queue) {
		dev_err(dev, "Failed to allocate the RX data queue\n");
		ret = -ENOMEM;
		goto err_kfree_name;
	}

	INIT_LIST_HEAD(&priv->rx_queue->head);

	init_waitqueue_head(&priv->rx_ioctl_wait);

	mutex_init(&priv->rx_lock);

	return 0;

err_kfree_name:
	kfree(misc_device->name);

err_release_irq:
	rcar_pci_rc_release_irq(priv);

err_kfree_priv_name:
	kfree(priv->name);

err_ida_remove:
	ida_simple_remove(&rcar_pci_rc_ida, id);

err_iounmap:
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		if (priv->bar[i])
			pci_iounmap(pdev, priv->bar[i]);
	}

err_disable_irq:
	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);

err_disable_pdev:
	pci_disable_device(pdev);

	return ret;
}

static void rcar_pci_rc_remove(struct pci_dev *pdev)
{
	struct rcar_pci_rc_private *priv = pci_get_drvdata(pdev);
	struct miscdevice *misc_device = &priv->miscdev;
	int id, bar;

	if (sscanf(misc_device->name, DRV_MODULE_NAME ".%d", &id) != 1)
		return;
	if (id < 0)
		return;

	misc_deregister(misc_device);
	kfree(misc_device->name);

	if (priv->rx_workqueue)
		destroy_workqueue(priv->rx_workqueue);

	kfree(priv->rx_queue);
	kfree(priv->name);
	rcar_pci_rc_release_irq(priv);

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (priv->bar[bar])
			iounmap(priv->bar[bar]);
	}

	ida_simple_remove(&rcar_pci_rc_ida, id);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	return;
}

static const struct pci_device_id rcar_pci_rc_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_RENESAS, PCI_DEVICE_ID_RENESAS_R8A779F0) },
	{ PCI_DEVICE(PCI_VENDOR_ID_RENESAS, PCI_DEVICE_ID_RENESAS_R8A779G0) },
	{ PCI_DEVICE(PCI_VENDOR_ID_SYNOPSYS, PCI_DEVICE_ID_SYNOPSYS_EDDA) },
	{ }
};
MODULE_DEVICE_TABLE(pci, rcar_pci_rc_tbl);

static struct pci_driver rcar_pci_rc_driver = {
	.name		= DRV_MODULE_NAME,
	.id_table	= rcar_pci_rc_tbl,
	.probe		= rcar_pci_rc_probe,
	.remove		= rcar_pci_rc_remove,
};
module_pci_driver(rcar_pci_rc_driver);

MODULE_DESCRIPTION("R-CAR PCI INTERFACE HOST DRIVER");
MODULE_AUTHOR("Phong Hoang <phong.hoang.wz@renesas.com>");
MODULE_LICENSE("GPL v2");
