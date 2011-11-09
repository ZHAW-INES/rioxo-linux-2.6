/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 * Description:
 *
 * ISP1763 BASE driver - common initialization and OTG support
 *
 * Originally being written for a simple bus powered device, this
 * driver does not support or implement HRP or SRP.
 *
 * (c) 2010 F.A. Voegel, Carangul.Tech
 * (c) 2010 I+ME ACTIA GmbH
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include "../core/hcd.h" /* NOTE: new kernel: #include <linux/usb/hcd.h> */
#include <linux/sched.h>
#include <linux/kthread.h>

#ifdef CONFIG_PPC_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif

#include <linux/usb/isp1763.h>
#include <linux/platform_device.h>

/* FIXME: Cleanup if it is not breaking anything, comment out now just in case
* #include <asm/byteorder.h>
* #include <asm/dma.h>
* #include <asm/system.h>
*/

#include "isp1763.h"
#include "isp1763-hcd.h"

static struct isp1763_dev *ispdev;


#define DRIVER_NAME "isp1763"
static const char driver_name[] = DRIVER_NAME;
static const char driver_desc[] = "ISP 1763A device controller driver";
//static struct task_struct *otgtask;
static char *role[] = { "HOST", "PERIPHERAL" };

static unsigned int otg_role;
module_param(otg_role, int, S_IRUGO | S_IWUSR);

/* sysfs file to show otg role */
static ssize_t show_otg_role(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", otg_role);
}

static ssize_t store_otg_role(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val) != 0)
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	otg_role = val;
	return count;
}

static DEVICE_ATTR(otg_role, S_IRUGO | S_IWUSR, show_otg_role, store_otg_role);

static struct attribute *dev_attrs[] = {
	&dev_attr_otg_role.attr,
	NULL,
};

const struct attribute_group dev_attr_group = {
	.attrs = dev_attrs,
};

/**
 * otg_set_role - set/switch role between host and device
 * @dev: isp1763_dev structure
 * @role: role to switch to
*/
static void otg_set_role(struct isp1763_dev *dev, int role)
{
	unsigned long flags;
	local_irq_save(flags);
	if (role == ROLE_HOST) {
		printk(KERN_ERR "%s() switching to HOST role\n", __func__);
		isp1763_writel(0xFFFF << 16, &dev->regs->otg_ctrl);
		isp1763_writel(MASK_OTGCTRL_VBUS_DRV |
			       MASK_OTGCTRL_VBUS_CHRG |
			       MASK_OTGCTRL_DM_PULLDOWN |
			       MASK_OTGCTRL_DP_PULLDOWN,
			       &dev->regs->otg_ctrl);
	}

	else {
		printk(KERN_ERR "%s() switching to PERIPHERAL role\n",
			__func__);
		isp1763_writel(0xFFFF << 16, &dev->regs->otg_ctrl);
		isp1763_writel(MASK_OTGCTRL_SW_SEL_HC_DC |
			       MASK_OTGCTRL_DP_PULLUP,
			       &dev->regs->otg_ctrl);
	}
	dev->current_role = role;
	local_irq_restore(flags);
}

static int isp1763_init_device(struct isp1763_dev *dev);

/**
* otg_role_switcher - role switch handling task
* @data: pointer to struct isp1763_dev
*/
int otg_role_switcher(void *data)
{
	struct isp1763_dev *dev = data;
	int id_pin;

	printk(KERN_ERR "%s starting up...\n", __func__);

	/* Set this once, so we can override it by sysfs afterwards */
	otg_role = 0;

	while (!kthread_should_stop()) {

		schedule_timeout_interruptible(1);

		if (kthread_should_stop())
			break;

#if 0
		id_pin = get_id_pin(dev);
		if (id_pin == dev->current_role)
			continue;
#endif
		if (otg_role != dev->current_role)
			id_pin = otg_role;
		else
			continue;

		printk(KERN_ERR "%s: init role change %s => %s\n", __func__,
		       role[dev->current_role], role[id_pin]);

		if (dev->ctrl[dev->current_role]) {
			if (dev->ctrl[dev->current_role]->suspend)
				printk(KERN_ERR "=> SUSPEND %s\n",
				       role[dev->current_role]);
				dev->ctrl[dev->current_role]->
					suspend(dev->ctrl[dev->current_role]);
		}

		isp1763_init_device(dev);
		otg_set_role(dev, id_pin);

		if (dev->ctrl[dev->current_role]) {
			if (dev->ctrl[dev->current_role]->resume) {
				printk(KERN_ERR "=> RESUME %s\n",
				       role[dev->current_role]);
				dev->ctrl[dev->current_role]->
					resume(dev->ctrl[dev->current_role]);
				dev->ctrl[dev->current_role]->
					do_irq(dev->ctrl[dev->current_role],
					       MASK_DCINT_VBUS);
			}
		}
	}
	return 0;
}

#if 0
/**
 * otg_do_interrupt - Handle OTG interrupt
 * @dev: pointer to isp1763_dev struct
*/
static void otg_do_interrupt(struct isp1763_dev *dev, u32 latch)
{
	if ((latch & MASK_OTGSTATUS_ID)
	    && !static_role(dev->devflags)) {
//		wake_up_process(otgtask);
	} else if (latch & MASK_OTGSTATUS_ID) {
		printk(KERN_EMERG "Ignoring OTG event\n");
	}
	if (latch & MASK_OTGINTEN_TMR_TIMEOUT) {
		if (dev->otgtimer_cb)
			dev->otgtimer_cb(dev->otgtimer_data);
	}
}
#endif

/**
 * isp1763 - main IRQ handler
 * @data: pointer to struct isp1763_dev
 */
static irqreturn_t isp1763_irq(int irq, void *data)
{
	struct isp1763_dev *dev = data;
	u32 flags_hc;
	u32 flags_dc;
	u16 flags_otg;

	isp1763_writew(UNLOCK_CODE, &dev->regs->unlock);

	disable_glint(dev->regs);

#if 0
	flags_otg = isp1763_readw(&dev->regs->otg_int_latch_set);
	isp1763_writew(0xFFFF, &dev->regs->otg_int_latch_clear);
#endif

	pr_debug("%.8lx %.4x %.8lx\n", flags_dc, flags_hc, flags_otg);

#if 0
	if (flags_otg)
		otg_do_interrupt(dev, flags_otg);
#endif

	if (dev->current_role == ROLE_HOST) {
		flags_hc = isp1763_readw(&dev->regs->hc_interrupt);
		isp1763_writew(flags_hc, &dev->regs->hc_interrupt);

		if (flags_hc && dev->ctrl[ROLE_HOST]
				/*&& dev->ctrl[ROLE_HOST]->do_irq*/)
			dev->ctrl[ROLE_HOST]->
				do_irq(dev->ctrl[ROLE_HOST], flags_hc);
	} else {
		flags_dc = isp1763_readl(&dev->regs->dc_interrupt);
		isp1763_writel(flags_dc, &dev->regs->dc_interrupt);

		pr_info("DC IRQ %08x\n", flags_dc);

		if (flags_dc && dev->ctrl[ROLE_DEVICE]
				/*&& dev->ctrl[ROLE_DEVICE]->do_irq*/)
			dev->ctrl[ROLE_DEVICE]->
				do_irq(dev->ctrl[ROLE_DEVICE], flags_dc);
	}

	enable_glint(dev->regs);

	return IRQ_HANDLED;
}

static char *bustypes[] = {
	"NAND", "Generic", "NOR", "SRAM"
};

/**
 * isp1763_init_device - reset & initialize controller hardware
 * @dev: pointer to struct isp1763_dev
*/
static int isp1763_init_device(struct isp1763_dev *dev)
{
	int i;
	int ret = 0;
//	u32 base;
	u16 hwmode =
		MASK_HWMODECTRL_COMN_INT | MASK_HWMODECTRL_GLOBAL_INT_ENABLE;

	mdelay(10);
	/* Dummy reads to "stabilize host controller access" */
	isp1763_readw(&dev->regs->chip_id);
	isp1763_readw(&dev->regs->chip_id);
	isp1763_readw(&dev->regs->chip_id);
	mdelay(20);

	if (dev->devflags & ISP1763_FLAG_BUS_WIDTH_8) {
		hwmode |= MASK_HWMODECTRL_DATA_BUS_WIDTH;
		printk(KERN_NOTICE "hwmode 8bit\n");
	}
	if (dev->devflags & ISP1763_FLAG_DACK_POL_HIGH) {
		hwmode |= MASK_HWMODECTRL_DACK_POL;
		printk(KERN_NOTICE "hwmode dack polarity high\n");
	}
	if (dev->devflags & ISP1763_FLAG_DREQ_POL_HIGH) {
		hwmode |= MASK_HWMODECTRL_DREQ_POL;
		printk(KERN_NOTICE "hwmode dreq polarity hight\n");
	}
	if (dev->devflags & ISP1763_FLAG_INTR_POL_HIGH) {
		hwmode |= MASK_HWMODECTRL_INTR_POLARITY;
		printk(KERN_NOTICE "hwmode int polarity high\n");
	}
	if (dev->devflags & ISP1763_FLAG_INTR_EDGE_TRIG) {
		hwmode |= MASK_HWMODECTRL_INTR_EDGE;
		printk(KERN_NOTICE "hwmode int edge triggered\n");
	}
	if (static_role(dev->devflags)) {
		hwmode |= MASK_HWMODECTRL_ID_PU_DISABLE;
		printk(KERN_NOTICE "hwmode pullup disable\n");
	}

#if 0
	hwmode &= ~MASK_HWMODECTRL_INTR_POLARITY;
	hwmode &= ~MASK_HWMODECTRL_INTR_EDGE;
#endif

	if (isp1763_readl(&dev->regs->chip_id) != ISP_CHIP_ID) {
		printk(KERN_ERR
		       "Error: ISP1763 chip ID wrong! (%.8x, expected %.8lx)\n",
		       isp1763_readl(&dev->regs->chip_id), ISP_CHIP_ID);
		ret = -EIO;
		goto out;
	}

	printk(KERN_NOTICE "%s: %s mode, %i bit bus width\n", DRIVER_NAME,
	       bustypes[(isp1763_readw(&dev->regs->swreset) >> 6) & 0x3],
	       isp1763_readw(&dev->regs->hwmodectrl) &
				MASK_HWMODECTRL_DATA_BUS_WIDTH ? 8 : 16);

	isp1763_writew(UNLOCK_CODE, &dev->regs->unlock);

	isp1763_writew(isp1763_readw(&dev->regs->swreset) | SW_RESET_RESET_ATX
						| SW_RESET_RESET_ALL,
						&dev->regs->swreset);
	isp1763_writew(isp1763_readw(&dev->regs->mode) | MASK_MODE_SFRESET,
		       &dev->regs->mode);
	udelay(50);
	isp1763_writew(isp1763_readw(&dev->regs->mode) &
		       ~MASK_MODE_SFRESET, &dev->regs->mode);
	mdelay(1);

	isp1763_writew(UNLOCK_CODE, &dev->regs->unlock);

	if (isp1763_readl(&dev->regs->chip_id) != ISP_CHIP_ID) {
		printk(KERN_ERR
		       "Error: ISP1763 chip ID wrong after reset! "
		       "(%.8x, expected %.8lx)\n",
		       isp1763_readl(&dev->regs->chip_id), ISP_CHIP_ID);
		ret = -EIO;
		goto out;
	}

	isp1763_writew(hwmode, &dev->regs->hwmodectrl);
	isp1763_writew(MASK_HCINT_OTG, &dev->regs->hc_interrupt_enable);

	/*
	  You'd think there was one single way to configure the interrupt for
	  this chip, but no. You have to independently configure both
	  peripheral and host controller IRQs despite the settings obviously
	  having to be the same...
	*/
	isp1763_writew(0xFC |
		       ((ispdev->devflags & ISP1763_FLAG_INTR_EDGE_TRIG) >> 1)
		       | (ispdev->devflags & ISP1763_FLAG_INTR_POL_HIGH),
		       &dev->regs->icr);

	/* data line test */
	for (i = 0; i < 0x10; i++) {
		u16 readback = 0;
		isp1763_writew(i, &dev->regs->scratch);
		readback = isp1763_readw(&dev->regs->scratch);
		if (readback != i) {
			printk(KERN_ERR
			       "ERROR: Scratch register write test failed: scratch=%.4x != i=%.4x!\n",
			       readback, i);
			ret = -EIO;
			/*
			* Don't break here, let us see the other errors too,
			* helps with diagnosis
			*/
		}
	}

	isp1763_writew(1, &dev->regs->int_pulse_width);

out:
	return ret;
}


/** otg_setup - initialize ISP1763 OTG component
  * @dev: isp1763_dev struture
*/
static int otg_setup(struct isp1763_dev *dev)
{
	int role = ROLE_DEVICE;

#if 0
	if (static_role_host(dev->devflags))
		role = ROLE_HOST;
	else
		role = get_id_pin(dev);
#endif
	role = otg_role;

	printk(KERN_NOTICE "ISP1783 using %s mode\n",
				static_role(dev->devflags) ?
					(static_role_host(dev->devflags) ?
						"host" : "device") : "otg");

	otg_set_role(dev, role);

	enable_glint(dev->regs);

	isp1763_writel(0xFFFF0000, &dev->regs->otg_int_enable_fall);
	isp1763_writel(0xFFFF0000, &dev->regs->otg_int_enable_rise);
	isp1763_writew(0xFFFF, &dev->regs->otg_int_latch_clear);

#if 0
	isp1763_writel(MASK_OTGINTEN_ID | MASK_OTGINTEN_SESS_VALID |
		       MASK_OTGINTEN_VBUS_VALID |
		       MASK_OTGINTEN_TMR_TIMEOUT,
		       &dev->regs->otg_int_enable_fall);

	isp1763_writel(MASK_OTGINTEN_ID | MASK_OTGINTEN_SESS_VALID |
		       MASK_OTGINTEN_VBUS_VALID |
		       MASK_OTGINTEN_TMR_TIMEOUT,
		       &dev->regs->otg_int_enable_rise);
#endif

	ispdev->current_role = role;

	return role;
}

/**
* isp1763_otg_timer_start - start OTG timer in ISP1763
* @cb: callback function
* @timeout: timeout in multiples of 10 microseconds
* @data: callback context data
*/
int isp1763_otg_timer_start(otgtimer_callback_t cb,
				unsigned long timeout,
				void *data)
{
	if (isp1763_readw(&ispdev->regs->otg_timer_hw_set) & 0x8000)
		return -EBUSY;

	ispdev->otgtimer_cb = cb;
	ispdev->otgtimer_data = data;

	isp1763_writew(0xFFFF, &ispdev->regs->otg_timer_lw_clear);
	isp1763_writew(0xFFFF, &ispdev->regs->otg_timer_hw_clear);

	isp1763_writew(timeout & 0xFFFF, &ispdev->regs->otg_timer_lw_set);
	isp1763_writew(((timeout >> 16) & 0x00FF) | 0x8000,
			       &ispdev->regs->otg_timer_hw_set);
	return 0;
}
EXPORT_SYMBOL_GPL(isp1763_otg_timer_start);

/**
* isp1763_otg_timer_cancel - cancel running OTG timer
*
* No harm done if timer isn't actually running
*/
void isp1763_otg_timer_cancel(void)
{
	isp1763_writew(isp1763_readl(&ispdev->regs->otg_timer_hw_set) & 0x7FFF,
		       &ispdev->regs->otg_timer_hw_set);
	ispdev->otgtimer_cb = NULL;
	ispdev->otgtimer_data = NULL;
}
EXPORT_SYMBOL_GPL(isp1763_otg_timer_cancel);

/**
* isp1763_register_ctrl - register a host or device controller driver
* @ctrl: controller structure to register
* @role: role for which to register driver
*
* Register a driver for a host or peripheral controller driver. After
* registering, the driver's probe function will be called, and if OTG is
* enabled and the current role matches the driver, it's resume function
* will be called as well.
*/
int isp1763_register_ctrl(struct isp1763_controller *ctrl, int role)
{
	if (!ispdev) {
		pr_err("no ispdev\n");
		return -ENODEV;
	}

	if (ispdev->ctrl[role]) {
		printk(KERN_ERR
		       "Trying to register driver for role %s which already has driver registered\n",
		       role == ROLE_HOST ? "host" : "device");
		return -EBUSY;
	}
	ispdev->ctrl[role] = ctrl;

	ctrl->device = ispdev->dev;
	ctrl->regs = ispdev->regs;

	if (!ctrl->probe(ctrl) < 0) {
		pr_err("probe failed\n");
		return -ENODEV;
	}

	/* driver for the current role? resume right now. */
	if (role == ispdev->current_role) {
		printk(KERN_ERR "Registered %s as active driver\n",
		       role == ROLE_HOST ? "host" : "device");
		ctrl->resume(ctrl);
	} else
		printk(KERN_ERR "Registered %s as suspended driver\n",
		       role == ROLE_HOST ? "host" : "device");

	return 0;
}
EXPORT_SYMBOL_GPL(isp1763_register_ctrl);

/**
 * isp1763_unregister_ctrl - unregister a previously registered controller
 * @role: role the controller filled
 */
void isp1763_unregister_ctrl(int _role)
{
	if (!ispdev->ctrl[_role]) {
		printk(KERN_ERR "Failed to unregister driver for '%s'"
				"controller\n", role[_role]);
		return;
	}

	ispdev->ctrl[_role]->suspend(ispdev->ctrl[_role]);

	if (ispdev->ctrl[_role]->remove)
		ispdev->ctrl[_role]->remove(ispdev->ctrl[_role]);

	ispdev->ctrl[_role] = NULL;
}
EXPORT_SYMBOL_GPL(isp1763_unregister_ctrl);

/*******************************************/
/* module init / exit / probe related code */
/*******************************************/


/**
* isp1763_common_init - common module init
* @dev: struct device pointer passed from probe function
* @devflags: hardware flags as determined by calling probe function
* @irq: IRQ number to use
* @res: ISP1763 register memory resource pointer
* @res_len: length of resources
*
* Called by all probe functions to initialize the hardware and setup data
* structures.
*/
static int isp1763_common_init(struct device *dev,
				unsigned long devflags,
				int irq,
				struct resource *res,
				resource_size_t res_len)
{
	int ret = 0;

	ispdev = devm_kzalloc(dev, sizeof(struct isp1763_dev), GFP_KERNEL);
	if (!ispdev)
		return -ENOMEM;

	ispdev->irq = irq;
	ispdev->devflags = devflags;
	ispdev->regs = devm_ioremap_nocache(dev, res->start, res_len);
	if (!ispdev->regs)
		return -ENOMEM;

	ret = devm_request_irq(dev, ispdev->irq, isp1763_irq, IRQF_SHARED, "ISP1763 IRQ",
							(void *) ispdev);
	if (ret)
		return ret;

	isp1763_init_device(ispdev);
	otg_setup(ispdev);

#if 0
	/* Only start role switch thread if we're doing OTG */
	if (!static_role(ispdev->devflags)) {
		otgtask = kthread_run(otg_role_switcher, ispdev,
							"isp1763otg");
		if (IS_ERR(otgtask)) {
			ret = PTR_ERR(otgtask);
			otgtask = NULL;
			goto out_free;
		}
	}
#endif

	ispdev->dev = dev;

	ret = sysfs_create_group(&dev->kobj, &dev_attr_group);
	if (ret) {
		pr_err("failed to create sysfs files\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PPC_OF
static int __devinit isp1763_of_probe(struct of_device *dev,
					const struct of_device_id *match)
{
	struct device_node *np = dev->node;
	const u32 *prop;
	int ret = 0;
	unsigned long devflags = 0;
	struct resource *res;
	struct resource memory;
	int irq;
	resource_size_t res_len;
	const char *port1_role = NULL;

	prop = of_get_property(np, "bus-width", NULL);
	if (prop && be32_to_cpu(*prop) == 8)
		devflags |= ISP1763_FLAG_BUS_WIDTH_8;

	port1_role = of_get_property(np, "port1-role", NULL);
	if (port1_role) {
		printk(KERN_ERR "PORT1 OTG setting: %s\n", port1_role);
		if (strcmp(port1_role, "otg") == 0)
			devflags |= ISP1763_FLAG_PORT1_ROLE_OTG;
		else if (strcmp(port1_role, "host") == 0)
			devflags |= ISP1763_FLAG_PORT1_ROLE_HOST;
		else
			devflags |= ISP1763_FLAG_PORT1_ROLE_GADGET;
	} else
		devflags |= ISP1763_FLAG_PORT1_ROLE_GADGET;

	if (of_get_property(np, "dack-polarity-high", NULL) != NULL)
		devflags |= ISP1763_FLAG_DACK_POL_HIGH;

	if (of_get_property(np, "dreq-polarity-high", NULL) != NULL)
		devflags |= ISP1763_FLAG_DREQ_POL_HIGH;

	if (of_get_property(np, "intr-polarity-high", NULL) != NULL)
		devflags |= ISP1763_FLAG_INTR_POL_HIGH;

	if (of_get_property(np, "intr-edge-trig", NULL) != NULL)
		devflags |= ISP1763_FLAG_INTR_EDGE_TRIG;

	irq = irq_of_parse_and_map(np, 0);

	if (irq == NO_IRQ) {
		pr_warning("of_isp1763: no interrupt!\n");
		ret = -EIO;
		goto error_out;
	}

	ret = of_address_to_resource(np, 0, &memory);
	if (ret) {
		pr_warning("of_isp1763: Memory resource not available\n");
		goto error_out;
	}

	res_len = resource_size(&memory);

	res = request_mem_region(memory.start, res_len, dev_name(&dev->dev));
	if (!res) {
		pr_warning
		    ("of_isp1763: Cannot reserve the memory resource\n");
		return -EBUSY;
	}

	return isp1763_common_init(&dev->dev, devflags, irq, res, res_len);

error_out:
	return ret;
}

static int __devexit isp1763_of_remove(struct of_device *ofdev)
{

	/* HCD specific removal */
	struct usb_hcd *hcd = dev_get_drvdata(&ofdev->dev);

	dev_set_drvdata(&ofdev->dev, NULL);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	/* TODO: Any UDC specific removal? */

	/* Common operation for all */
	free_irq(ispdev->irq, ispdev);
	kfree(ispdev);
	return 0;
}


static struct of_device_id __devinitdata isp1763_match[] = {
	{
	 .compatible = "st,usb-isp1763",
	 .compatible = "nxp,usb-isp1763",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, isp1763_match);

static struct of_platform_driver isp1763_driver = {
	.name = driver_name,
	.match_table = isp1763_match,
	.probe = isp1763_of_probe,
	.remove = __devexit_p(isp1763_of_remove),
};

#endif /* CONFIG_PPC_OF */


/******************************************************************/
/* PCI PROBE                                                      */
/******************************************************************/

#ifdef CONFIG_PCI
static int __devinit isp1763_pci_probe(struct pci_dev *dev,
				       const struct pci_device_id *id)
{
	u8 latency, limit;
	__u32 reg_data;
	int retry_count;
	unsigned int devflags = 0;
	int ret_status = 0;

	resource_size_t pci_mem_phy0;
	resource_size_t memlength;

	u8 __iomem *chip_addr;
	u8 __iomem *iobase;
	resource_size_t nxp_pci_io_base;
	resource_size_t iolength;

	if (usb_disabled())
		return -ENODEV;

	if (pci_enable_device(dev) < 0)
		return -ENODEV;

	if (!dev->irq)
		return -ENODEV;

	/* Grab the PLX PCI mem maped port start address we need  */
	nxp_pci_io_base = pci_resource_start(dev, 0);
	iolength = pci_resource_len(dev, 0);

	if (!request_mem_region
	    (nxp_pci_io_base, iolength, "ISP1763 IO MEM")) {
		printk(KERN_ERR "request region #1\n");
		return -EBUSY;
	}

	iobase = ioremap(nxp_pci_io_base, iolength);
	if (!iobase) {
		printk(KERN_ERR "ioremap #1\n");
		ret_status = -ENOMEM;
		goto cleanup1;
	}
	/* Grab the PLX PCI shared memory of the ISP 1763 we need  */
	pci_mem_phy0 = pci_resource_start(dev, 3);
	memlength = pci_resource_len(dev, 3);
	if (memlength < 0xffff) {
		printk(KERN_ERR
		       "memory length for this resource is wrong\n");
		ret_status = -ENOMEM;
		goto cleanup2;
	}

	if (!request_mem_region(pci_mem_phy0, memlength, "ISP-PCI")) {
		printk(KERN_ERR "host controller already in use\n");
		ret_status = -EBUSY;
		goto cleanup2;
	}

	/* map available memory */
	chip_addr = ioremap_nocache(pci_mem_phy0, memlength);
	if (!chip_addr) {
		printk(KERN_ERR "Error ioremap failed\n");
		ret_status = -ENOMEM;
		goto cleanup3;
	}

	/* bad pci latencies can contribute to overruns */
	pci_read_config_byte(dev, PCI_LATENCY_TIMER, &latency);
	if (latency) {
		pci_read_config_byte(dev, PCI_MAX_LAT, &limit);
		if (limit && limit < latency)
			pci_write_config_byte(dev, PCI_LATENCY_TIMER,
					      limit);
	}

	/* Try to check whether we can access Scratch Register of
	 * Host Controller or not. The initial PCI access is retried until
	 * local init for the PCI bridge is completed
	 */
	retry_count = 20;
	reg_data = 0;
	while ((reg_data != 0xFACE) && retry_count) {
		/*by default host is in 16bit mode, so
		 * io operations at this stage must be 16 bit
		 * */
		writel(0xface, chip_addr + HC_SCRATCH_REG);
		udelay(100);
		reg_data = readl(chip_addr + HC_SCRATCH_REG) & 0x0000ffff;
		retry_count--;
	}

	iounmap(chip_addr);

	/* Host Controller presence is detected by writing to scratch register
	 * and reading back and checking the contents are same or not
	 */
	if (reg_data != 0xFACE) {
		dev_err(&dev->dev, "scratch register mismatch %x\n",
			reg_data);
		ret_status = -ENOMEM;
		goto cleanup3;
	}

	pci_set_master(dev);

	/* configure PLX PCI chip to pass interrupts */
#define PLX_INT_CSR_REG 0x68
	reg_data = readl(iobase + PLX_INT_CSR_REG);
	reg_data |= 0x900;
	writel(reg_data, iobase + PLX_INT_CSR_REG);

	dev->dev.dma_mask = NULL;

	/* FIXME: Is there a better way than assuming this? */
	devflags |= ISP1763_FLAG_PORT1_ROLE_HOST;

	if (isp1763_common_init(&dev->dev, devflags, dev->irq,
				(void *) pci_mem_phy0, memlength) < 0)
		goto cleanup3;

	/* done with PLX IO access */
	iounmap(iobase);
	release_mem_region(nxp_pci_io_base, iolength);

	return 0;

cleanup3:
	release_mem_region(pci_mem_phy0, memlength);
cleanup2:
	iounmap(iobase);
cleanup1:
	release_mem_region(nxp_pci_io_base, iolength);
	return ret_status;
}

static void isp1763_pci_remove(struct pci_dev *dev)
{
	struct usb_hcd *hcd;

	hcd = pci_get_drvdata(dev);
	pci_disable_device(dev);
}

static void isp1763_pci_shutdown(struct pci_dev *dev)
{
	printk(KERN_ERR "ips1763_pci_shutdown\n");
}

static const struct pci_device_id isp1763_plx[] = {
	{
	 .class = PCI_CLASS_BRIDGE_OTHER << 8,
	 .class_mask = ~0,
	 .vendor = PCI_VENDOR_ID_PLX,
	 .device = 0x5406,
	 .subvendor = PCI_VENDOR_ID_PLX,
	 .subdevice = 0x9054,
	 },
	{}
};

MODULE_DEVICE_TABLE(pci, isp1763_plx);

static struct pci_driver isp1763_pci_driver = {
	.name = "isp1763",
	.id_table = isp1763_plx,
	.probe = isp1763_pci_probe,
	.remove = isp1763_pci_remove,
	.shutdown = isp1763_pci_shutdown,
};
#endif /* CONFIG_PCI */


/******************************************************************/
/* PLATFORM DEVICE PROBE                                          */
/******************************************************************/

static int __devexit isp1763_plat_remove(struct platform_device *pdev)
{
#if 0
	struct resource *mem_res;
	resource_size_t mem_size;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mem_size = resource_size(mem_res);
	release_mem_region(mem_res->start, mem_size);
#endif

	/* devm takes care of releasing resources */

	return 0;
}

static int __devinit isp1763_plat_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *mem_res;
	struct resource *irq_res;
	resource_size_t mem_size;
	struct isp1763_platform_data *priv = pdev->dev.platform_data;
	unsigned int devflags = 0;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		pr_warning("isp1763: Memory resource not available\n");
		return -ENODEV;
	}
	mem_size = resource_size(mem_res);
	if (!devm_request_mem_region(&pdev->dev, mem_res->start, mem_size, "isp1763")) {
		pr_warning
		    ("isp1763: Cannot reserve the memory resource\n");
		return -EBUSY;
	}

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		pr_warning("isp1763: IRQ resource not available\n");
		return -ENODEV;
	}

	if (priv) {
		if (priv->bus_width_8)
			devflags |= ISP1763_FLAG_BUS_WIDTH_8;
#if 0
		if (priv->port1_otg == 1)
			devflags |= ISP1763_FLAG_PORT1_ROLE_OTG;
		else
			devflags |= ISP1763_FLAG_PORT1_ROLE_HOST;
#endif

		if (priv->dack_polarity_high)
			devflags |= ISP1763_FLAG_DACK_POL_HIGH;
		if (priv->dreq_polarity_high)
			devflags |= ISP1763_FLAG_DREQ_POL_HIGH;
		if (priv->intr_polarity_high)
			devflags |= ISP1763_FLAG_INTR_POL_HIGH;
		if (priv->intr_edge_trigger)
			devflags |= ISP1763_FLAG_INTR_EDGE_TRIG;
	}

	return isp1763_common_init(&pdev->dev, devflags, irq_res->start,
						mem_res, mem_size);
}

static struct platform_driver isp1763_plat_driver = {
	.remove = __devexit_p(isp1763_plat_remove),
	.driver = {
		.name = "isp1763",
	},
};

static int __init isp1763_init(void)
{
	int ret = 0;
	int any_ret = -ENODEV;

	ret = platform_driver_probe(&isp1763_plat_driver, isp1763_plat_probe);
	if (!ret)
		any_ret = 0;

#ifdef CONFIG_PPC_OF
	ret = of_register_platform_driver(&isp1763_driver);
	if (!ret)
		any_ret = 0;
#endif

#ifdef CONFIG_PCI
	ret = pci_register_driver(&isp1763_pci_driver);
	if (!ret)
		any_ret = 0;
#endif
	ret = isp1763_hc_init();
	if (!ret)
		any_ret = 0;

	return any_ret;
}

static void __exit isp1763_exit(void)
{
	isp1763_hc_exit();

#ifdef CONFIG_PPC_OF
	of_unregister_platform_driver(&isp1763_driver);
#endif

#ifdef CONFIG_PCI
	pci_unregister_driver(&isp1763_pci_driver);
#endif

	platform_driver_unregister(&isp1763_plat_driver);
}

module_init(isp1763_init);
module_exit(isp1763_exit);

MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("F.A. Voegel, Carangul.Tech");
MODULE_LICENSE("GPL");
