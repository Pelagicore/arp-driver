/******************************************************************************
 *
 *   Copyright (C) 2017-2018 Luxoft Sweden AB. All rights reserved.
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/property.h>


struct arp_info {
	struct pci_dev *dev;
	unsigned int regs_base;
	unsigned int regs_size;
	void *regs_mem;
	int num_irqs;
	struct platform_device *camera_dev;
	spinlock_t lock;
};

struct arp_info *ai = NULL;


#ifdef ARP_ENABLE_CAN
static struct resource can_resources[] = {
	{
		.start = 0x2200,
		.end = 0x23ff,
		.flags = IORESOURCE_MEM,
		.name = "regs",
	},
	{
		.start = 16,
		.end = 16,
		.flags = IORESOURCE_IRQ,
		.name = "irq",
	},
};

static struct property_entry can_properties[] = {
	{},
};
#endif

#ifdef ARP_ENABLE_CAMERA
static struct resource camera_resources[] = {
	[0] = {
		.start = 0,
		.end   = 0x3fff,
		.flags = IORESOURCE_MEM,
		.name = "regs",
	},
	[1] = {
		.start = 15,
		.end   = 15,
		.flags = IORESOURCE_IRQ,
		.name = "dma_irq",
	},
	[2] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
		.name = "cam0_irq",
	},
	[3] = {
		.start = 1,
		.end   = 1,
		.flags = IORESOURCE_IRQ,
		.name = "cam1_irq",
	},
	[4] = {
		.start = 2,
		.end   = 2,
		.flags = IORESOURCE_IRQ,
		.name = "cam2_irq",
	},
	[5] = {
		.start = 3,
		.end   = 3,
		.flags = IORESOURCE_IRQ,
		.name = "cam3_irq",
	},
};

static struct property_entry camera_properties[] = {
	{ },
};
#endif

static const struct mfd_cell arp_cells_bar0[] = {
#ifdef ARP_ENABLE_CAN
	{
		.name = "d_can",
		.of_compatible = "bosch,d_can",
		.num_resources = ARRAY_SIZE(can_resources),
		.resources = can_resources,
		.properties = can_properties,
	},
#endif
#ifdef ARP_ENABLE_CAMERA
	{
		.name = "arp-camera",
		.num_resources = ARRAY_SIZE(camera_resources),
		.resources = camera_resources,
		.properties = camera_properties,
	},
#endif
};

static int arp_pci_probe(struct pci_dev *dev,
			 const struct pci_device_id *id)
{
	struct arp_info *info;
	int i;
	int err;

	info = devm_kzalloc(&dev->dev, sizeof(struct arp_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (pci_enable_device(dev))
		return -ENODEV;

	pci_set_master(dev);

	if (pci_request_regions(dev, "arp"))
		goto out_disable;

	info->regs_base = pci_resource_start(dev, 0);
	if (!info->regs_base)
		goto out_release;

	info->regs_size = pci_resource_len(dev, 0);
	info->regs_mem = pci_ioremap_bar(dev, 0);
	if (!info->regs_mem) {
		goto out_release;
	}

	pr_info("Registers at: %x\n", info->regs_base);

	info->dev = dev;

	ai = info;

	pci_set_drvdata(dev, info);

	ai->num_irqs = pci_alloc_irq_vectors(dev, 1, 16, PCI_IRQ_MSIX);
	if (ai->num_irqs < 0) {
		pr_err("Failed IRQ alloc\n");
		goto out_release;
	}

	err = mfd_add_devices(&dev->dev, -1, arp_cells_bar0,
			      ARRAY_SIZE(arp_cells_bar0), &dev->resource[0],
			      pci_irq_vector(dev, 0), NULL);
	if (err) {
		dev_err(&dev->dev,
			"Failed to add MFD devices: %d\n", err);
		goto out_irq_vectors;
	}

	/* All vectors except DMA are assigned to the camera IRQ handler */
	for (i = 0; i < ai->num_irqs;i++) {
		pr_info("IRQ Vector %d: %d\n", i, pci_irq_vector(dev, i));
	}

	return 0;

//out_device:
	mfd_remove_devices(&dev->dev);
out_irq_vectors:
	pci_free_irq_vectors(dev);
out_release:
	pci_release_regions(dev);
out_disable:
	pci_disable_device(dev);
	return -ENODEV;
}

static void arp_pci_remove(struct pci_dev *dev)
{
	mfd_remove_devices(&dev->dev);
	pci_free_irq_vectors(dev);
	pci_release_regions(dev);
	pci_disable_device(dev);
	iounmap(ai->regs_mem);
}

static struct pci_device_id arp_pci_ids[] = {
	{
		.vendor =	0x8086,
		.device =	0xfffd,
		.subvendor =	PCI_ANY_ID,
		.subdevice =	PCI_ANY_ID,
	},
	{ 0, }
};

static struct pci_driver arp_pci_driver = {
	.name = "arp",
	.id_table = arp_pci_ids,
	.probe = arp_pci_probe,
	.remove = arp_pci_remove,
};

module_pci_driver(arp_pci_driver);

MODULE_DEVICE_TABLE(pci, arp_pci_ids);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Linus Nielsen");
