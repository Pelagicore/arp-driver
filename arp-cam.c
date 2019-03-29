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
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#define NUM_CAMERAS 4

#define DMA_FRAME_SIZE (WIDTH*HEIGHT*3)

#define WIDTH 1280
#define HEIGHT 1080

/* ioctl commands */
#define SET_ANGLE      0
#define SET_HDMI_ANGLE 1
#define SET_MS_SINCE_UI_START 2
#define SET_DASHBOARD_TELLTALES 3

#define NUM_DESCRIPTORS 16

/* DMA descriptor definitions */
struct std_dma_desc {
	u32 rdaddr;
	u32 wraddr;
	u32 length;
	u32 next;
	u32 bytes_transferred;
	u32 status;
	u32 reserved;
	u32 ctrl;
};

#define DESC_CTRL_GO         (1 << 31)
#define DESC_CTRL_HW         (1 << 30)
#define DESC_CTRL_EARLY_DONE (1 << 24)
#define DESC_CTRL_ET_IRQ     (1 << 15)
#define DESC_CTRL_TC_IRQ     (1 << 14)
#define DESC_CTRL_END_ON_EOP (1 << 11)
#define DESC_CTRL_PARK_W     (1 << 11)
#define DESC_CTRL_PARK_R     (1 << 10)
#define DESC_CTRL_GEN_EOP    (1 << 9)
#define DESC_CTRL_GEN_SOP    (1 << 8)


/* BAR0 Register map */
#define PREFETCH_BASE       0x2000
#define PREFETCH_CTRL       0x2080
#define PREFETCH_NEXT_LO    0x2084
#define PREFETCH_NEXT_HI    0x2088
#define PREFETCH_POLL_FREQ  0x208c
#define PREFETCH_STATUS     0x2090
#define DISPATCH_STATUS     0x20a0
#define DISPATCH_CTRL       0x20a4
#define MS_SINCE_UI_START   0x2400
#define DASHBOARD_TELLTALES 0x2404
#define CAMERA_IRQ_ACK      0x2900
#define HDMI_ANGLE          0x2c00

/* BAR1 Register map */

/* Register bit definitions */
#define PREFETCH_CTRL_PARK    (1 << 4)
#define PREFETCH_CTRL_INT_EN  (1 << 3)
#define PREFETCH_CTRL_RESET   (1 << 2)
#define PREFETCH_CTRL_POLL_EN (1 << 1)
#define PREFETCH_CTRL_RUN     (1 << 0)

#define DISPATCH_STATUS_BUSY          (1 << 0)
#define DISPATCH_STATUS_DESC_EMPTY    (1 << 1)
#define DISPATCH_STATUS_DESC_FULL     (1 << 2)
#define DISPATCH_STATUS_RESP_EMPTY    (1 << 3)
#define DISPATCH_STATUS_RESP_FULL     (1 << 4)
#define DISPATCH_STATUS_STOPPED       (1 << 5)
#define DISPATCH_STATUS_RESETTING     (1 << 6)
#define DISPATCH_STATUS_STOPPED_ERROR (1 << 7)
#define DISPATCH_STATUS_STOPPED_EARLY (1 << 8)
#define DISPATCH_STATUS_IRQ           (1 << 9)

#define DISPATCH_CTRL_STOP_DISPATCHER  (1 << 0)
#define DISPATCH_CTRL_RESET_DISPATCHER (1 << 1)
#define DISPATCH_CTRL_STOP_ON_ERROR    (1 << 2)
#define DISPATCH_CTRL_STOP_ON_EARLY    (1 << 3)
#define DISPATCH_CTRL_IRQ_ENABLE       (1 << 4)
#define DISPATCH_CTRL_STOP_DESCRIPTORS (1 << 5)

struct camera_info {
	struct device *dev;
	dev_t devno;
	struct cdev cdev;
	struct class *dev_class;
	u32 regs;
	void *regs_mem;
	dma_addr_t dma_addr[NUM_CAMERAS];
	unsigned char *cpu_addr[NUM_CAMERAS];
	dma_addr_t desc_dma_addr;
	struct std_dma_desc *desc_cpu_addr;
	int next_desc;
	spinlock_t lock;
	int angle;
	int dma_irq;
	int cam_irq[NUM_CAMERAS];

	/* FIXME: Band-aid solution because the interrupts can't be masked */
	bool setup_complete;
};

static struct camera_info *ci = NULL;

const uint32_t offsets[] = {0x00400000,
			    0x01000000,
			    0x01c00000,
			    0x02800000};


static int dma_reset(void)
{
	int i;
	u32 val;

	writel(DISPATCH_CTRL_RESET_DISPATCHER, ci->regs_mem + DISPATCH_CTRL);
	for(i = 0;i < 1000;i++) {
		val = readl(ci->regs_mem + DISPATCH_STATUS);
		if((val & DISPATCH_STATUS_RESETTING) == 0) {
			return val;
		}
		msleep(1);
        }
	return -1;
}

static int dma_prefetch_reset(void)
{
	int i;
	u32 val;

	writel(PREFETCH_CTRL_RESET, ci->regs_mem + PREFETCH_CTRL);
	for(i = 0;i < 1000;i++) {
		val = readl(ci->regs_mem + PREFETCH_CTRL);
		if((val & PREFETCH_CTRL_RESET) == 0) {
			/* Clear the global IRQ */
			writel(1, ci->regs_mem + PREFETCH_STATUS);
			return val;
		}
		msleep(1);
	}
        return -1;
}

static int start_dma_transfer(u32 from, u32 to, u32 num_bytes)
{
	int next, last;

	pr_debug("DMA 0x%08x -> 0x%08x (0x%08x bytes)\n", from, to, num_bytes);

	next = ci->next_desc;
	last = (next+1) % NUM_DESCRIPTORS;

	if(ci->desc_cpu_addr[last].ctrl & DESC_CTRL_HW) {
		pr_warn("No free descriptors available.\n");
		return -1;
	}

	pr_debug("next = %d, last = %d\n", next, last);

	ci->desc_cpu_addr[next].rdaddr = from;
	ci->desc_cpu_addr[next].wraddr = to;
	ci->desc_cpu_addr[next].length = num_bytes;
	ci->desc_cpu_addr[next].ctrl =
		DESC_CTRL_HW | DESC_CTRL_GO | DESC_CTRL_TC_IRQ;
	ci->next_desc = last;
	return 0;
}


static int cam_open(struct inode *inode, struct file *f)
{
	return 0;
}


static int cam_close(struct inode *inode, struct file *f)
{
	return 0;
}

static ssize_t cam_read(struct file * f, char *buf,
			size_t count, loff_t * ppos)
{
	dma_addr_t dma_addr;
	u8 *cpu_addr;
	int res;
	unsigned long j_start, j_end;
	unsigned long ms;

	j_start = jiffies;

	dma_addr = ci->dma_addr[ci->angle];
	cpu_addr = ci->cpu_addr[ci->angle];

	dma_sync_single_for_cpu(ci->dev, dma_addr,
				DMA_FRAME_SIZE, DMA_FROM_DEVICE);

	res = copy_to_user(buf, cpu_addr, count);
	if(res) {
		pr_err("Failed to copy to user space");
		return 0;
	}

	j_end = jiffies;
	ms = ((j_end - j_start) * 1000) / HZ;

	pr_debug("read() done in %lu ms\n", ms);
	return count;
}


static ssize_t cam_write (struct file * f, __user const char *buf,
			  size_t count, loff_t * ppos)
{
	return 0;
}

static long cam_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	switch(cmd) {
	case SET_ANGLE:
		if(arg < NUM_CAMERAS) {
			ci->angle = arg;
		} else {
			retval = -EINVAL;
		}
		break;
	case SET_HDMI_ANGLE:
		if(arg < NUM_CAMERAS) {
			writel(arg, ci->regs_mem + HDMI_ANGLE);
		} else {
			retval = -EINVAL;
		}
		break;
	case SET_MS_SINCE_UI_START:
		writel(arg, ci->regs_mem + MS_SINCE_UI_START);
		break;
	case SET_DASHBOARD_TELLTALES:
		writel(arg, ci->regs_mem + DASHBOARD_TELLTALES);
		break;
	}
	return retval;
}

static struct file_operations cam_fops =
{
	.owner = THIS_MODULE,
	.read = cam_read,
	.write = cam_write,
	.open = cam_open,
	.release = cam_close,
	.unlocked_ioctl = cam_ioctl,
};

static void prepare_descriptors(void) {
	int i;
	dma_addr_t next;

	next = ci->desc_dma_addr + sizeof(struct std_dma_desc);

	for(i = 0;i < NUM_DESCRIPTORS-1;i++) {
		ci->desc_cpu_addr[i].ctrl = 0;
		ci->desc_cpu_addr[i].next = next;
		next += sizeof(struct std_dma_desc);
	}

	/* Last descriptor points to the first */
	ci->desc_cpu_addr[i].ctrl = 0;
	ci->desc_cpu_addr[i].next = ci->desc_dma_addr;
}

static int dma_setup(void)
{
	int i;
	dma_addr_t dma_addr;
	u8 *cpu_addr;

	pr_info("Preparing DMA buffers\n");

	ci->next_desc = 0;

	for(i = 0;i < NUM_CAMERAS;i++) {
		cpu_addr = dmam_alloc_coherent(
			ci->dev, DMA_FRAME_SIZE, &dma_addr, GFP_KERNEL);
		if(!cpu_addr) {
			pr_err("DMA alloc failed\n");
			return -1;
		}

		pr_info("Buffer %d at 0x%llx\n", i, dma_addr);

		ci->dma_addr[i] = dma_addr;
		ci->cpu_addr[i] = cpu_addr;
	}

	cpu_addr = dmam_alloc_coherent(
		ci->dev, NUM_DESCRIPTORS * sizeof(struct std_dma_desc),
		&dma_addr, GFP_KERNEL);
	if(!cpu_addr) {
		pr_err("DMA descriptor alloc failed\n");
		return -1;
	}

	pr_info("Descriptors at 0x%llx\n", dma_addr);

	ci->desc_dma_addr = dma_addr;
	ci->desc_cpu_addr = (struct std_dma_desc *)cpu_addr;

	prepare_descriptors();

	writel(dma_addr & 0xffffffff, ci->regs_mem + PREFETCH_NEXT_LO);
	writel(dma_addr >> 32, ci->regs_mem + PREFETCH_NEXT_HI);
	writel(125*500, ci->regs_mem + PREFETCH_POLL_FREQ); /* 0.5ms */
	writel(PREFETCH_CTRL_RUN |
	       PREFETCH_CTRL_POLL_EN |
	       PREFETCH_CTRL_INT_EN, ci->regs_mem + PREFETCH_CTRL);
	return 0;
}

void handle_camera_irq(int camera) {
	unsigned long flags;

	/* FIXME: Band-aid solution because the interrupts can't be masked */
	if(!ci->setup_complete)
		return;

	spin_lock_irqsave(&ci->lock, flags);
	writel(0xF, ci->regs_mem + CAMERA_IRQ_ACK);//BAND-AID for missed IRQs
	start_dma_transfer(offsets[camera], ci->dma_addr[camera],
			   DMA_FRAME_SIZE);
	spin_unlock_irqrestore(&ci->lock, flags);
}

irqreturn_t camera_irq_func(int irq, void *dev_id)
{
	int i;
	for(i = 0;i < NUM_CAMERAS;i++) {
		if(irq == ci->cam_irq[i]) {
			handle_camera_irq(i);
		}
	}
	return IRQ_HANDLED;
}

irqreturn_t dma_irq_func(int irq, void *dev_id)
{
	unsigned long flags;

	spin_lock_irqsave(&ci->lock, flags);

	/* Clear the global IRQ */
	writel(1, ci->regs_mem + PREFETCH_STATUS);

	spin_unlock_irqrestore(&ci->lock, flags);
	return IRQ_HANDLED;
}




static int arp_camera_probe (struct platform_device *pdev) {
	struct resource *regs_res;
	int i;
	int err;

	pr_info("ARP camera platform device probed\n");

	ci = devm_kzalloc(&pdev->dev, sizeof(struct camera_info), GFP_KERNEL);
	if (!ci)
		return -ENOMEM;

	spin_lock_init(&ci->lock);

	ci->dev = &pdev->dev;

	regs_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	if((!regs_res)){
		pr_err(" Register resource not available");
		goto out_resource;
	}
	ci->regs = regs_res->start;

	ci->dma_irq = platform_get_irq(pdev, 0);
	pr_info("DMA IRQ: %d\n", ci->dma_irq);
	if((ci->dma_irq < 0)){
		pr_err(" IRQ 0 resource not available");
		goto out_resource;
	}

	for(i = 0;i < NUM_CAMERAS;i++) {
		ci->cam_irq[i] = platform_get_irq(pdev, 1+i);
		pr_info("Cam %d IRQ: %d\n", i, ci->cam_irq[i]);
		if((ci->cam_irq[i] < 0)){
			pr_err(" IRQ %d resource not available", i+1);
			goto out_resource;
		}
	}

	if((alloc_chrdev_region(&ci->devno, 0, 1, "framegrabber")) < 0){
		pr_err("Cannot allocate major number\n");
		goto out_region;
	}

	pr_info("Major = %d Minor = %d\n", MAJOR(ci->devno), MINOR(ci->devno));

	cdev_init(&ci->cdev, &cam_fops);
	ci->cdev.owner = THIS_MODULE;

	if((cdev_add(&ci->cdev, ci->devno, 1)) < 0){
		pr_err("Cannot add the device to the system\n");
		goto out_cdev;
	}

	if((ci->dev_class = class_create(THIS_MODULE, "cam_class")) == NULL){
		pr_err("Cannot create the struct class\n");
		goto out_class;
	}

	if((device_create(ci->dev_class, NULL, ci->devno,
			  NULL,"framegrabber")) == NULL) {
		pr_err("Cannot create the device\n");
		goto out_device;
	}

	ci->regs_mem = devm_ioremap(&pdev->dev, ci->regs, SZ_16K);
	pr_info("Regs: 0x%08x -> %p\n", ci->regs, ci->regs_mem);

	err = devm_request_irq(&pdev->dev, ci->dma_irq,
			       dma_irq_func, IRQF_SHARED,
			       "ARP Camera DMA", pdev);
	if(err < 0){
		pr_err("ARP Camera DMA can't get assigned IRQ\n");
		goto out_dma_irq_vector;
	}

	for(i = 0;i < NUM_CAMERAS;i++) {
		err = devm_request_irq(&pdev->dev,
				       ci->cam_irq[i],
				       camera_irq_func,
				       IRQF_SHARED, "ARP Camera",
				       pdev);
		if (err < 0) {
			pr_err("ARP Camera DMA can't get assigned IRQ\n");
			goto out_cam_irq_vectors;
		}
	}

	dma_prefetch_reset();
	dma_reset();
	err = dma_setup();
	if(err < 0) {
		pr_err("ARP Camera DMA setup failed\n");
		goto out_cam_irq_vectors;
	}

	/* FIXME: Band-aid solution because the interrupts can't be masked */
	ci->setup_complete = true;
	writel(0xf, ci->regs_mem + CAMERA_IRQ_ACK);
	return 0;

out_cam_irq_vectors:
out_dma_irq_vector:
	device_destroy(ci->dev_class, ci->devno);
out_device:
	class_destroy(ci->dev_class);
out_class:
	cdev_del(&ci->cdev);
out_cdev:
	unregister_chrdev_region(ci->devno, 1);
out_region:
out_resource:
	return -ENODEV;
}

static int arp_camera_remove(struct platform_device *pdev)
{
	dma_prefetch_reset();
	dma_reset();

	device_destroy(ci->dev_class, ci->devno);
	class_destroy(ci->dev_class);
	cdev_del(&ci->cdev);
	unregister_chrdev_region(ci->devno, 1);
	pr_info("ARP camera platform device removed\n");
	return 0;
}

static struct platform_driver arp_camera_driver = {
	.probe = arp_camera_probe,
	.remove = arp_camera_remove,
	.driver = {
		.name = "arp-camera",
		.owner = THIS_MODULE,
	},
};

static int __init arp_camera_mod_init(void)
{
	pr_info("ARP camera platform module loaded\n");
	platform_driver_register(&arp_camera_driver);
	return 0;
}

static void __exit arp_camera_mod_remove (void)
{
	pr_info("ARP camera platform module unloaded\n");
	platform_driver_unregister(&arp_camera_driver);
}

module_init(arp_camera_mod_init);
module_exit(arp_camera_mod_remove);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Linus Nielsen");
MODULE_DESCRIPTION("Platform driver for the ARP camera input");
