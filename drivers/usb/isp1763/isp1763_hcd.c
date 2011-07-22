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
 * isp1763 Host Controller Driver
 *
 * Based on isp1760 driver by Sebastian Siewior <bigeasy@linutronix.de> and
 * pehci hcd from ST-Ericsson wired support <wired.support@stericsson.com>
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <asm/unaligned.h>
#include <asm/cacheflush.h>

#include "../core/hcd.h"
#include "../core/hub.h"
#include "isp1763.h"
#include "isp1763_hcd.h"


static struct kmem_cache *qtd_cachep;
static struct kmem_cache *qh_cachep;

static inline struct isp1763_hcd *hcd_to_priv(struct usb_hcd *hcd)
{
	return (struct isp1763_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *priv_to_hcd(struct isp1763_hcd *priv)
{
	return container_of((void *) priv, struct usb_hcd, hcd_priv);
}

/**
 * isp1763_read_mem - read HC descriptor+payload memory
 * @hcd: the usb_hcd handle (needed to know the device base address)
 * @srcaddr: the source (start) address where the data reside on the hcd device
 * @dstpt: the destination where we want to store the data
 * @len: the length of the data
 */
static void isp1763_read_mem(struct usb_hcd *hcd,
			     u16 srcaddr, u16 *dstptr, u32 len)
{
	if (!hcd || !dstptr || len <= 0) {
		printk(KERN_ERR "ERROR: hcd: %p dstptr: %p len: %d\n",
		       hcd, dstptr, len);
		return;
	}

	/* Write the starting device address to the hcd memory register */
	isp1763_writew(srcaddr, hcd->regs + HC_MEMORY_REG);

	/* As long there are at least 16-bit to read ... */
	while (len >= 2) {
		*dstptr = isp1763_readw(hcd->regs + HC_DATA_REG);
		len -= 2;	/* Adjust len */
		dstptr++;	/* Set dest data pointer to its next address */
	}

	/* If there are no more bytes to read, return */
	if (len <= 0)
		return;

	*((u8 *) dstptr) = (u8) (isp1763_readw(hcd->regs + HC_DATA_REG) & 0xFF);
}

/**
 * isp1763_write_mem - write HC descriptor+payload memory
 * @hcd: the usb_hcd handle (needed to know the device base address)
 * @dataptr: the source of the data we want to write
 * @dstaddr: the desitnation (start) address where data is to be written
 * @len: the length of the data
 */
static void isp1763_write_mem(struct usb_hcd *hcd,
			      u16 *dataptr, u16 dstaddr, u32 len)
{
	if (!hcd || !dataptr || len <= 0) {
		printk(KERN_ERR "ERROR: hcd: %p dataptr: %p len: %d\n",
		       hcd, dataptr, len);
		return;
	}

	isp1763_writew(dstaddr, hcd->regs + HC_MEMORY_REG);
	while (len >= 2) {
		isp1763_writew(*dataptr, hcd->regs + HC_DATA_REG);
		dataptr++;
		len -= 2;
	}

	if (len <= 0)
		return;

	isp1763_writew(*((u8 *) dataptr), hcd->regs + HC_DATA_REG);
}

/**
 * init_memory - initialize HC memory management
 * @priv: HC private data
 *
 * memory management of the 20kb payload on the chip from 0x1000 to 0x5fff
 */
static void init_memory(struct isp1763_hcd *priv)
{
	int i;
	u32 payload;

	payload = 0x1000;
	for (i = 0; i < BLOCK_1_NUM; i++) {
		priv->memory_pool[i].start = payload;
		priv->memory_pool[i].size = BLOCK_1_SIZE;
		priv->memory_pool[i].free = 1;
		payload += priv->memory_pool[i].size;
	}


	for (i = BLOCK_1_NUM; i < BLOCK_1_NUM + BLOCK_2_NUM; i++) {
		priv->memory_pool[i].start = payload;
		priv->memory_pool[i].size = BLOCK_2_SIZE;
		priv->memory_pool[i].free = 1;
		payload += priv->memory_pool[i].size;
	}


	for (i = BLOCK_1_NUM + BLOCK_2_NUM; i < BLOCKS; i++) {
		priv->memory_pool[i].start = payload;
		priv->memory_pool[i].size = BLOCK_3_SIZE;
		priv->memory_pool[i].free = 1;
		payload += priv->memory_pool[i].size;
	}

	BUG_ON(payload - priv->memory_pool[i - 1].size > PAYLOAD_SIZE);
}

/**
 * alloc_mem - allocate HC-internal memory for payload data
 * @priv: HC private data
 * @size: size of memory chunk to allocate
 *
 * memory management of the 20kb payload on the chip from 0x1000 to 0x5fff
 */
static u32 alloc_mem(struct isp1763_hcd *priv, u32 size)
{
	int i;

	if (!size)
		return ISP1763_NULL_POINTER;

	for (i = 0; i < BLOCKS; i++) {
		if (priv->memory_pool[i].size >= size &&
		    priv->memory_pool[i].free) {

			priv->memory_pool[i].free = 0;
			return priv->memory_pool[i].start;
		}
	}

	printk(KERN_ERR
	       "ISP1763 MEM: can not allocate %d bytes of memory\n",
	       size);
	printk(KERN_ERR "Current memory map:\n");
	for (i = 0; i < BLOCKS; i++) {
		printk(KERN_ERR "Pool %2d size %4d status: %d\n",
		       i,
		       priv->memory_pool[i].size,
		       priv->memory_pool[i].free);
	}
	/* XXX maybe -ENOMEM could be possible */
	BUG();
	return 0;
}

/**
 * free_mem - free previously allocated HC-internal memory
 * @priv: HC private data
 * @mem: offset of memory block to free
 */
static void free_mem(struct isp1763_hcd *priv, u32 mem)
{
	int i;

	if (mem == ISP1763_NULL_POINTER)
		return;

	for (i = 0; i < BLOCKS; i++) {
		if (priv->memory_pool[i].start == mem) {

			BUG_ON(priv->memory_pool[i].free);

			priv->memory_pool[i].free = 1;
			return;
		}
	}

	printk(KERN_ERR "Trying to free not-here-allocated memory :%08x\n",
	       mem);
	BUG();
}

/**
 * isp1763_init_regs - initialize registers
 * @hcd: the usb_hcd handle
 */
static void isp1763_init_regs(struct usb_hcd *hcd)
{
	isp1763_writew(0, hcd->regs + HC_BUFFER_STATUS_REG);

	isp1763_writew(NO_TRANSFER_ACTIVE,
		       hcd->regs + HC_ATL_PTD_SKIPMAP_REG);
	isp1763_writew(NO_TRANSFER_ACTIVE,
		       hcd->regs + HC_INT_PTD_SKIPMAP_REG);
	isp1763_writew(NO_TRANSFER_ACTIVE,
		       hcd->regs + HC_ISO_PTD_SKIPMAP_REG);

	isp1763_writew((u16) (~NO_TRANSFER_ACTIVE),
		       hcd->regs + HC_ATL_PTD_DONEMAP_REG);
	isp1763_writew((u16) (~NO_TRANSFER_ACTIVE),
		       hcd->regs + HC_INT_PTD_DONEMAP_REG);
	isp1763_writew((u16) (~NO_TRANSFER_ACTIVE),
		       hcd->regs + HC_ISO_PTD_DONEMAP_REG);
}

/**
 * handshake - wait for bitmask to be set in register
 * @priv: hc private data
 * @ptr: pointer to memory-mapped register to read
 * @mask: mask to apply to register (bitwise AND)
 * @done: expected result of masked register contents
 * @usec: timeout in microseconds
 *
 * WARNING!
 * This handshake() function should only be used for 32-bit registers
 * All the accesses so far seems okay for isp1763, since they only access the
 * standard EHCI regs, which are all 32-bit.
 */
static int handshake(struct isp1763_hcd *priv, void __iomem * ptr,
		     u32 mask, u32 done, int usec)
{
	u32 result;

	do {
		result = isp1763_readl(ptr);
		if (result == ~0)
			return -ENODEV;

		result &= mask;
		if (result == done)
			return 0;
		udelay(1);
		usec--;
	} while (usec > 0);


	pr_err("hadnshake timed out\n");
	return -ETIMEDOUT;
}

/**
 * ehci_reset - reset EHCI registers
 * @priv: HC private data
 *
 * reset a non-running (STS_HALT == 1) controller
*/
static int ehci_reset(struct isp1763_hcd *priv)
{
	int retval;
	struct usb_hcd *hcd = priv_to_hcd(priv);
	u32 command = isp1763_readl(hcd->regs + HC_USBCMD);

	command |= CMD_RESET;
	isp1763_writel(command, hcd->regs + HC_USBCMD);
	hcd->state = HC_STATE_HALT;
	priv->next_statechange = jiffies;
	retval = handshake(priv, hcd->regs + HC_USBCMD,
			   CMD_RESET, 0, 250 * 1000);
	return retval;
}

static void qh_destroy(struct isp1763_qh *qh)
{
	BUG_ON(!list_empty(&qh->qtd_list));
	kmem_cache_free(qh_cachep, qh);
}

static struct isp1763_qh *isp1763_qh_alloc(struct isp1763_hcd *priv,
							gfp_t flags)
{
	struct isp1763_qh *qh;

	qh = kmem_cache_zalloc(qh_cachep, flags);
	if (!qh)
		return qh;

	INIT_LIST_HEAD(&qh->qtd_list);
	qh->priv = priv;
	return qh;
}

/**
 * priv_init - one-time init, only for memory state
 * @hcd: HC structure
 */
static int priv_init(struct usb_hcd *hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u32 hcc_params = HCC_HARDCODE;	/* Use hardcoded value */

	spin_lock_init(&priv->lock);

	/*
	 * hw default: 1K periodic list heads, one per frame.
	 * periodic_size can shrink by USBCMD update if hcc_params allows.
	 */
	priv->periodic_size = DEFAULT_I_TDPS;

	/* full frame cache */
	if (HCC_ISOC_CACHE(hcc_params))
		priv->i_thresh = 8;
	else			/* N microframes cached */
		priv->i_thresh = 2 + HCC_ISOC_THRES(hcc_params);

	return 0;
}

/**
 * isp1763_hc_setup - setup host controller
 * @hcd: host controller structure
 *
 * Fill registers with initial values and reset EHCI controller, enable IRQs
 */
static int isp1763_hc_setup(struct usb_hcd *hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	int result;

	isp1763_init_regs(hcd);

	isp1763_writew(isp1763_readw(hcd->regs + HC_RESET_REG) |
						SW_RESET_RESET_HC,
		       hcd->regs + HC_RESET_REG);
	result = ehci_reset(priv);
	if (result)
		return result;

	isp1763_info(priv, "bus width: %d\n",
		     (priv->devflags & ISP1763_FLAG_BUS_WIDTH_8) ? 8 : 16);

	isp1763_dbg(priv, "hwmode: 0x%04X\n",
		    isp1763_readw(hcd->regs + HC_HW_MODE_CTRL));

	isp1763_writew(INTERRUPT_ENABLE_MASK,
		       hcd->regs + HC_INTERRUPT_REG);
	isp1763_writew(INTERRUPT_ENABLE_MASK,
		       hcd->regs + HC_INTERRUPT_ENABLE);

	/* This chip does not have HCSPARAMS register, use a hardcoded value. */
	priv->hcs_params = HCS_HARDCODE;

	return priv_init(hcd);
}

static void isp1763_init_maps(struct usb_hcd *hcd)
{
	/* Set last maps, for iso its only 1, else 16 tds bitmap */
	isp1763_writew(0x8000, hcd->regs + HC_ATL_PTD_LASTPTD_REG);
	isp1763_writew(0x8000, hcd->regs + HC_INT_PTD_LASTPTD_REG);
	isp1763_writew(0x0001, hcd->regs + HC_ISO_PTD_LASTPTD_REG);
}

static void isp1763_enable_interrupts(struct usb_hcd *hcd)
{
	isp1763_writew(0x0000, hcd->regs + HC_ATL_IRQ_MASK_AND_REG);
	isp1763_writew(0x0000, hcd->regs + HC_ATL_IRQ_MASK_OR_REG);
	isp1763_writew(0x0000, hcd->regs + HC_INT_IRQ_MASK_AND_REG);
	isp1763_writew(0x0000, hcd->regs + HC_INT_IRQ_MASK_OR_REG);
	isp1763_writew(0x0000, hcd->regs + HC_ISO_IRQ_MASK_AND_REG);
	isp1763_writew(0xffff, hcd->regs + HC_ISO_IRQ_MASK_OR_REG);
}

static int isp1763_run(struct usb_hcd *hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	int retval;
	u16 temp;
	u32 command;
	u32 chipid;

	hcd->uses_new_polling = 1;
	hcd->poll_rh = 0;
	hcd->state = HC_STATE_RUNNING;

	isp1763_enable_interrupts(hcd);
	temp = isp1763_readw(hcd->regs + HC_HW_MODE_CTRL);
	isp1763_writew(temp | HW_GLOBAL_INTR_EN,
		       hcd->regs + HC_HW_MODE_CTRL);

	command = isp1763_readl(hcd->regs + HC_USBCMD);
	command &= ~(CMD_LRESET | CMD_RESET);
	command |= CMD_RUN;
	isp1763_writel(command, hcd->regs + HC_USBCMD);

	retval = handshake(priv, hcd->regs + HC_USBCMD, CMD_RUN, CMD_RUN,
			   250 * 1000);
	if (retval)
		return retval;

	down_write(&ehci_cf_port_reset_rwsem);
	isp1763_writel(FLAG_CF, hcd->regs + HC_CONFIGFLAG);

	retval = handshake(priv, hcd->regs + HC_CONFIGFLAG, FLAG_CF, FLAG_CF,
								250 * 1000);
	up_write(&ehci_cf_port_reset_rwsem);
	if (retval)
		return retval;

	chipid = isp1763_readl(hcd->regs + HC_CHIP_ID_REG);
	isp1763_info(priv, "USB ISP %04x HW rev. %d started\n",
		     ((chipid & 0x00ffff00) >> 8), (chipid & 0x000000ff));

	/* PTD Register Init Part 2, Step 28 */
	/* enable INTs */
	isp1763_init_maps(hcd);

	return 0;
}

static inline u32 base_to_chip(u32 base)
{
	u32 chipaddr = ((base - 0x400) >> 3);
	return chipaddr;
}

static void transform_into_atl(struct isp1763_hcd *priv,
			       struct isp1763_qh *qh,
			       struct isp1763_qtd *qtd, struct urb *urb,
			       u32 payload, struct ptd *ptd)
{
	u32 dw0;
	u32 dw1;
	u32 dw2;
	u32 dw3;
	u32 maxpacket;
	u32 multi;
	u32 pid_code;
	u32 rl = RL_COUNTER;
	u32 nak = NAK_COUNTER;

	/* according to 3.6.2, max packet len can not be > 0x400 */
	maxpacket = usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe));
	multi = 1 + ((maxpacket >> 11) & 0x3);
	maxpacket &= 0x7ff;

	dw0 = PTD_VALID;
	dw0 |= PTD_LENGTH(qtd->length);
	dw0 |= PTD_MAXPACKET(maxpacket);
	dw0 |= PTD_ENDPOINT(usb_pipeendpoint(urb->pipe));
	dw1 = usb_pipeendpoint(urb->pipe) >> 1;

	dw1 |= PTD_DEVICE_ADDR(usb_pipedevice(urb->pipe));

	pid_code = qtd->packet_type;
	dw1 |= PTD_PID_TOKEN(pid_code);

	if (usb_pipebulk(urb->pipe))
		dw1 |= PTD_TRANS_BULK;
	else if (usb_pipeint(urb->pipe))
		dw1 |= PTD_TRANS_INT;

	if (urb->dev->speed != USB_SPEED_HIGH) {
		/* split transaction */

		dw1 |= PTD_TRANS_SPLIT;
		if (urb->dev->speed == USB_SPEED_LOW)
			dw1 |= PTD_SE_USB_LOSPEED;

		dw1 |= PTD_PORT_NUM(urb->dev->ttport);
		dw1 |= PTD_HUB_NUM(urb->dev->tt->hub->devnum);

		/* SE bit for Split INT transfers */
		if (usb_pipeint(urb->pipe) &&
		    (urb->dev->speed == USB_SPEED_LOW))
			dw1 |= 2 << 16;

		dw3 = 0;
		rl = 0;
		nak = 0;
	} else {
		dw0 |= PTD_MULTI(multi);
		if (usb_pipecontrol(urb->pipe) || usb_pipebulk(urb->pipe))
			dw3 = qh->ping;
		else
			dw3 = 0;
	}

	dw2 = 0;
	dw2 |= PTD_DATA_START_ADDR(base_to_chip(payload));
	dw2 |= PTD_RL_CNT(rl);
	dw3 |= PTD_NAC_CNT(nak);

	if (usb_pipecontrol(urb->pipe))
		dw3 |= PTD_DATA_TOGGLE(qtd->toggle);
	else
		dw3 |= qh->toggle;


	dw3 |= PTD_ACTIVE;
	dw3 |= PTD_CERR(ERR_COUNTER);

	memset((void *)ptd + (4 * sizeof(u32)), 0, (4 * sizeof(u32)));

	ptd->dw0 = cpu_to_le32(dw0);
	ptd->dw1 = cpu_to_le32(dw1);
	ptd->dw2 = cpu_to_le32(dw2);
	ptd->dw3 = cpu_to_le32(dw3);
}

static void transform_add_int(struct isp1763_hcd *priv,
			      struct isp1763_qh *qh,
			      struct isp1763_qtd *qtd, struct urb *urb,
			      u32 payload, struct ptd *ptd)
{
	u32 maxpacket;
	u32 multi;
	u32 numberofusofs;
	u32 i;
	u32 usofmask, usof;
	u32 period;

	maxpacket =
	    usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe));
	multi = 1 + ((maxpacket >> 11) & 0x3);
	maxpacket &= 0x7ff;
	/* length of the data per uframe */
	maxpacket = multi * maxpacket;

	numberofusofs = urb->transfer_buffer_length / maxpacket;
	if (urb->transfer_buffer_length % maxpacket)
		numberofusofs += 1;

	usofmask = 1;
	usof = 0;
	for (i = 0; i < numberofusofs; i++) {
		usof |= usofmask;
		usofmask <<= 1;
	}

	/* FIXME: Vendor code checks for:
	 * if (urb->dev->speed != USB_SPEED_HIGH && usb_pipeint(urb->pipe))
	 */
	if (urb->dev->speed != USB_SPEED_HIGH) {
		/* split */
		ptd->dw5 = cpu_to_le32(0x1c);

		if (qh->period >= 32)
			period = qh->period / 2;
		else
			period = qh->period;

	} else {

		if (qh->period >= 8)
			period = qh->period / 8;
		else
			period = qh->period;

		if (period >= 32)
			period = 16;

		if (qh->period >= 8) {
			/* millisecond period */
			period = (period << 3);
		} else {
			/* usof based transfers */
			/* minimum 4 usofs */
			usof = 0x11;
		}
	}

	ptd->dw2 |= cpu_to_le32(period);
	ptd->dw4 = cpu_to_le32(usof);
}

static void transform_into_int(struct isp1763_hcd *priv,
			       struct isp1763_qh *qh,
			       struct isp1763_qtd *qtd, struct urb *urb,
			       u32 payload, struct ptd *ptd)
{
	transform_into_atl(priv, qh, qtd, urb, payload, ptd);
	transform_add_int(priv, qh, qtd, urb, payload, ptd);
}

static int qtd_fill(struct isp1763_qtd *qtd, void *databuffer, size_t len,
		    u32 token)
{
	int count;

	qtd->data_buffer = databuffer;
	qtd->packet_type = GET_QTD_TOKEN_TYPE(token);
	qtd->toggle = GET_DATA_TOGGLE(token);

	if (len > HC_ATL_PL_SIZE)
		count = HC_ATL_PL_SIZE;
	else
		count = len;

	qtd->length = count;
	return count;
}

static int check_error(struct ptd *ptd)
{
	int error = 0;
	u32 dw3;

	dw3 = le32_to_cpu(ptd->dw3);
	if (dw3 & DW3_HALT_BIT) {
		error = -EPIPE;

		if (dw3 & DW3_ERROR_BIT)
			pr_err("error bit is set in DW3\n");
	}

	if (dw3 & DW3_QTD_ACTIVE) {
		printk(KERN_ERR "transfer active bit is set DW3\n");
		printk(KERN_ERR "nak counter: %d, rl: %d\n",
		       (dw3 >> 19) & 0xf,
		       (le32_to_cpu(ptd->dw2) >> 25) & 0xf);
	}

	return error;
}

static void check_int_err_status(u32 dw4)
{
	u32 i;

	dw4 >>= 8;

	for (i = 0; i < 8; i++) {
		switch (dw4 & 0x7) {
		case INT_UNDERRUN:
			printk(KERN_ERR "ERROR: under run , %d\n", i);
			break;

		case INT_EXACT:
			printk(KERN_ERR "ERROR: transaction error, %d\n",
			       i);
			break;

		case INT_BABBLE:
			printk(KERN_ERR "ERROR: babble error, %d\n", i);
			break;
		}
		dw4 >>= 3;
	}
}

static void enqueue_one_qtd(struct isp1763_qtd *qtd,
			    struct isp1763_hcd *priv, u32 payload)
{
	u32 token;
	struct usb_hcd *hcd = priv_to_hcd(priv);

	token = qtd->packet_type;

	if (qtd->length && (qtd->length <= HC_ATL_PL_SIZE)) {
		switch (token) {
		case IN_PID:
			break;
		case OUT_PID:
		case SETUP_PID:
			isp1763_write_mem(hcd, (u16 *) qtd->data_buffer,
					  (u16) payload, qtd->length);
		}
	}
}

static void enqueue_one_atl_qtd(u32 atl_regs, u32 payload,
				struct isp1763_hcd *priv,
				struct isp1763_qh *qh, struct urb *urb,
				u32 slot, struct isp1763_qtd *qtd)
{
	struct ptd ptd;
	struct usb_hcd *hcd = priv_to_hcd(priv);

	transform_into_atl(priv, qh, qtd, urb, payload, &ptd);
	isp1763_write_mem(hcd, (u16 *) &ptd, atl_regs, sizeof(ptd));
	enqueue_one_qtd(qtd, priv, payload);

	priv->atl_ints[slot].urb = urb;
	priv->atl_ints[slot].qh = qh;
	priv->atl_ints[slot].qtd = qtd;
	priv->atl_ints[slot].data_buffer = qtd->data_buffer;
	priv->atl_ints[slot].payload = payload;
	qtd->status |= URB_ENQUEUED | URB_TYPE_ATL;
	qtd->status |= slot << 16;
}

static void enqueue_one_int_qtd(u32 int_regs, u32 payload,
				struct isp1763_hcd *priv,
				struct isp1763_qh *qh, struct urb *urb,
				u32 slot, struct isp1763_qtd *qtd)
{
	struct ptd ptd;
	struct usb_hcd *hcd = priv_to_hcd(priv);


	transform_into_int(priv, qh, qtd, urb, payload, &ptd);
	isp1763_write_mem(hcd, (u16 *) &ptd, int_regs, sizeof(ptd));
	enqueue_one_qtd(qtd, priv, payload);

	priv->int_ints[slot].urb = urb;
	priv->int_ints[slot].qh = qh;
	priv->int_ints[slot].qtd = qtd;
	priv->int_ints[slot].data_buffer = qtd->data_buffer;
	priv->int_ints[slot].payload = payload;
	qtd->status |= URB_ENQUEUED | URB_TYPE_INT;
	qtd->status |= slot << 16;
}

static void enqueue_an_ATL_packet(struct usb_hcd *hcd,
				  struct isp1763_qh *qh,
				  struct isp1763_qtd *qtd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u16 skip_map, or_map;
	u16 queue_entry;
	u32 slot;
	u32 atl_regs, payload;
	u16 buffstatus;

	/*
	 * When this function is called from the interrupt handler to enqueue
	 * a follow-up packet, the SKIP register gets written and read back
	 * almost immediately. With ISP1763, this register requires a delay of
	 * 100ns between a write and subsequent read (see section 15.1.2).
	 */
	ndelay(100);

	skip_map = isp1763_readw(hcd->regs + HC_ATL_PTD_SKIPMAP_REG);
	BUG_ON(!skip_map);

	/* 32-bit to 16-bit adjustment */
	slot = __ffs((u32) skip_map);
	BUG_ON(slot > 16);	/* skip_map is only 16 bit */

	queue_entry = 1 << slot;

	atl_regs = ATL_REGS_OFFSET + slot * sizeof(struct ptd);

	payload = alloc_mem(priv, qtd->length);

	enqueue_one_atl_qtd(atl_regs, payload, priv, qh, qtd->urb, slot,
			    qtd);

	or_map = isp1763_readw(hcd->regs + HC_ATL_IRQ_MASK_OR_REG);
	or_map |= queue_entry;
	isp1763_writew(or_map, hcd->regs + HC_ATL_IRQ_MASK_OR_REG);

	skip_map &= ~queue_entry;
	isp1763_writew(skip_map, hcd->regs + HC_ATL_PTD_SKIPMAP_REG);

	buffstatus = isp1763_readw(hcd->regs + HC_BUFFER_STATUS_REG);
	buffstatus |= ATL_BUFFER;
	isp1763_writew(buffstatus, hcd->regs + HC_BUFFER_STATUS_REG);
}

static void enqueue_an_INT_packet(struct usb_hcd *hcd,
				  struct isp1763_qh *qh,
				  struct isp1763_qtd *qtd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u16 skip_map, or_map;
	u16 queue_entry;
	u32 slot;
	u32 int_regs, payload;
	u16 buffstatus;

	/*
	 * When this function is called from the interrupt handler to enqueue
	 * a follow-up packet, the SKIP register gets written and read back
	 * almost immediately. With ISP1763, this register requires a delay of
	 * 100ns between a write and subsequent read (see section 15.1.2).
	 */
	ndelay(100);

	skip_map = isp1763_readw(hcd->regs + HC_INT_PTD_SKIPMAP_REG);
	BUG_ON(!skip_map);

	/* 32-bit to 16-bit adjustment */
	slot = __ffs((u32) skip_map);
	BUG_ON(slot > 16);	/* skip_map is only 16 bit */

	queue_entry = 1 << slot;

	int_regs = INT_REGS_OFFSET + slot * sizeof(struct ptd);

	payload = alloc_mem(priv, qtd->length);

	enqueue_one_int_qtd(int_regs, payload, priv, qh, qtd->urb, slot,
			    qtd);

	or_map = isp1763_readw(hcd->regs + HC_INT_IRQ_MASK_OR_REG);
	or_map |= queue_entry;
	isp1763_writew(or_map, hcd->regs + HC_INT_IRQ_MASK_OR_REG);

	skip_map &= ~queue_entry;
	isp1763_writew(skip_map, hcd->regs + HC_INT_PTD_SKIPMAP_REG);

	buffstatus = isp1763_readw(hcd->regs + HC_BUFFER_STATUS_REG);
	buffstatus |= INT_BUFFER;
	isp1763_writew(buffstatus, hcd->regs + HC_BUFFER_STATUS_REG);
}

static void isp1763_urb_done(struct isp1763_hcd *priv, struct urb *urb,
			     int status)
__releases(priv->lock)
__acquires(priv->lock)
{
	if (!urb->unlinked) {
		if (status == -EINPROGRESS)
			status = 0;
	}

	if (usb_pipein(urb->pipe) && usb_pipetype(urb->pipe) != PIPE_CONTROL) {
		void *ptr;
		for (ptr = urb->transfer_buffer;
		     ptr < urb->transfer_buffer + urb->transfer_buffer_length;
		     ptr += PAGE_SIZE)
			flush_dcache_page(virt_to_page(ptr));
	}

	/* complete() can reenter this HCD */
	usb_hcd_unlink_urb_from_ep(priv_to_hcd(priv), urb);
	spin_unlock(&priv->lock);
	usb_hcd_giveback_urb(priv_to_hcd(priv), urb, status);
	spin_lock(&priv->lock);
}

static void isp1763_qtd_free(struct isp1763_qtd *qtd)
{
	kmem_cache_free(qtd_cachep, qtd);
}

static struct isp1763_qtd *clean_this_qtd(struct isp1763_qtd *qtd)
{
	struct isp1763_qtd *tmp_qtd;

	tmp_qtd = qtd->hw_next;
	list_del(&qtd->qtd_list);
	isp1763_qtd_free(qtd);
	return tmp_qtd;
}

/*
 * Remove this QTD from the QH list and free its memory. If this QTD
 * isn't the last one than remove also his successor(s).
 * Returns the QTD which is part of an new URB and should be enqueued.
 */
static struct isp1763_qtd *clean_up_qtdlist(struct isp1763_qtd *qtd)
{
	struct isp1763_qtd *tmp_qtd;
	int last_one;

	do {
		tmp_qtd = qtd->hw_next;
		last_one = qtd->status & URB_COMPLETE_NOTIFY;
		list_del(&qtd->qtd_list);
		isp1763_qtd_free(qtd);
		qtd = tmp_qtd;
	} while (!last_one && qtd);

	return qtd;
}

/* NOTE:
 * It seems that setting up two memory banks in the isp1760's code
 * is merely for convenience's sake; right now trying with:
 *
 * 1. Setup up the memory register for the PTD
 * 2. Copy PTD from device to the driver's PTD copy
 * 3. Process the PTD data
 * (later on)
 * 4. Setup the memory register for the payload
 * 5. Copy payload from device to the driver's payload space
 *
 * There may be a gotcha here if the isp1760 is taking advantage of the ISP_BANK
 * ability to automatically increment the address per bank on consecutive access
 * to the same bank; It does not seem so because atl_regs & payload are
 * reassigned at every while(done_map) iteration.
 */
static void do_atl_int(struct usb_hcd *usb_hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(usb_hcd);
	u16 done_map, skip_map;
	struct ptd ptd;
	struct urb *urb = NULL;
	u32 atl_regs_base;
	u32 atl_regs;
	u32 queue_entry;
	u32 payload;
	u32 length;
	u16 or_map;
	u32 status = -EINVAL;
	int error;
	struct isp1763_qtd *qtd;
	struct isp1763_qh *qh;
	u32 rl;
	u32 nakcount;
	u16 buffstatus;

	done_map = isp1763_readw(usb_hcd->regs + HC_ATL_PTD_DONEMAP_REG);

	/* NOTE: Move inside while (done_map) at the beginning and remove the
	 * skip_map read at the end of the loop?
	 */
	skip_map = isp1763_readw(usb_hcd->regs + HC_ATL_PTD_SKIPMAP_REG);

	or_map = isp1763_readw(usb_hcd->regs + HC_ATL_IRQ_MASK_OR_REG);
	or_map &= ~done_map;
	isp1763_writew(or_map, usb_hcd->regs + HC_ATL_IRQ_MASK_OR_REG);

	atl_regs_base = ATL_REGS_OFFSET;

	while (done_map) {
		u32 dw1;
		u32 dw2;
		u32 dw3;

		status = 0;

		/* 32-bit to 16-bit done_map adjustment */
		queue_entry = __ffs((u32) done_map);
		BUG_ON(queue_entry > 16);

		done_map &= ~(1 << queue_entry);
		skip_map |= 1 << queue_entry;

		atl_regs =
		    atl_regs_base + (queue_entry * sizeof(struct ptd));

		urb = priv->atl_ints[queue_entry].urb;
		qtd = priv->atl_ints[queue_entry].qtd;
		qh = priv->atl_ints[queue_entry].qh;
		payload = priv->atl_ints[queue_entry].payload;

		if (!qh) {
			printk(KERN_ERR "qh is 0\n");
			continue;
		}

		memset(&ptd, 0, sizeof(ptd));
		isp1763_read_mem(usb_hcd, (u16) atl_regs, (u16 *) &ptd,
				 sizeof(u32)*4/*sizeof(ptd)*/);

		dw1 = le32_to_cpu(ptd.dw1);
		dw2 = le32_to_cpu(ptd.dw2);
		dw3 = le32_to_cpu(ptd.dw3);
		rl = (dw2 >> 25) & 0x0f;
		nakcount = (dw3 >> 19) & 0xf;

		/* Transfer Error, *but* active and no HALT -> reload */
		if (unlikely((dw3 & DW3_ERROR_BIT) && (dw3 & DW3_QTD_ACTIVE) &&
		    !(dw3 & DW3_HALT_BIT))) {

			/* according to priv code, we have to
			 * reload this one if trasfered bytes != requested bytes
			 * else act like everything went smooth..
			 * XXX This just doesn't feel right and hasn't
			 * triggered so far.
			 */

			length = PTD_XFERRED_LENGTH(dw3);
			printk(KERN_ERR
			       "Should reload now.... transfered %d "
			       "of %zu\n", length, qtd->length);
			BUG();
		}

		if (unlikely(!nakcount && (dw3 & DW3_QTD_ACTIVE))) {
			printk(KERN_NOTICE
			       "Reloading ptd %p/%p... qh %p read: "
			       "%d of %zu done: %08x cur: %08x\n", qtd,
			       urb, qh, PTD_XFERRED_LENGTH(dw3),
			       qtd->length, done_map, (1 << queue_entry));

			/* RL counter = ERR counter */
			dw3 &= ~(0xf << 19);
			dw3 |= rl << 19;
			dw3 &= ~(3 << (55 - 32));
			dw3 |= ERR_COUNTER << (55 - 32);

			/*
			 * It is not needed to write skip map back because it
			 * is unchanged. Just make sure that this entry is
			 * unskipped once it gets written to the HW.
			 */
			skip_map &= ~(1 << queue_entry);
			or_map = isp1763_readw(usb_hcd->regs +
					       HC_ATL_IRQ_MASK_OR_REG);
			or_map |= 1 << queue_entry;
			isp1763_writew(or_map, usb_hcd->regs +
				       HC_ATL_IRQ_MASK_OR_REG);

			/* FIXME: Why isp1760 need 2 write_mem back-to-back? */
			ptd.dw3 = cpu_to_le32(dw3);
			isp1763_write_mem(usb_hcd, (u16 *) &ptd, atl_regs,
					  sizeof(ptd));

			ptd.dw0 |= cpu_to_le32(PTD_VALID);
			isp1763_write_mem(usb_hcd, (u16 *) &ptd, atl_regs,
					  sizeof(u32)/*sizeof(ptd)*/);

			buffstatus = isp1763_readw(usb_hcd->regs +
						   HC_BUFFER_STATUS_REG);
			buffstatus |= ATL_BUFFER;
			isp1763_writew(buffstatus, usb_hcd->regs +
				       HC_BUFFER_STATUS_REG);
			continue;
		}

		error = check_error(&ptd);
		if (unlikely(error)) {
			status = error;
			priv->atl_ints[queue_entry].qh->toggle = 0;
			priv->atl_ints[queue_entry].qh->ping = 0;
			urb->status = -EPIPE;
		} else {
			if (usb_pipetype(urb->pipe) == PIPE_BULK) {
				priv->atl_ints[queue_entry].qh->toggle =
				    dw3 & (1 << 25);
				priv->atl_ints[queue_entry].qh->ping =
				    dw3 & (1 << 26);
			}
		}

		length = PTD_XFERRED_LENGTH(dw3);
		if (length) {
			switch (DW1_GET_PID(dw1)) {
			case IN_PID:
				isp1763_read_mem(usb_hcd, (u16) payload,
						 priv->
						 atl_ints[queue_entry].
						 data_buffer, length);
			case OUT_PID:
				urb->actual_length += length;
			case SETUP_PID:
				break;
			}
		}

		priv->atl_ints[queue_entry].data_buffer = NULL;
		priv->atl_ints[queue_entry].urb = NULL;
		priv->atl_ints[queue_entry].qtd = NULL;
		priv->atl_ints[queue_entry].qh = NULL;

		free_mem(priv, payload);

		isp1763_writew(skip_map, usb_hcd->regs +
			       HC_ATL_PTD_SKIPMAP_REG);

		if (unlikely(urb->status == -EPIPE)) {
			/* HALT was received */

			qtd = clean_up_qtdlist(qtd);
			isp1763_urb_done(priv, urb, urb->status);
		} else if (usb_pipebulk(urb->pipe)
			   && (length < qtd->length)) {
			/* short BULK received */

			if (urb->transfer_flags & URB_SHORT_NOT_OK) {
				urb->status = -EREMOTEIO;
				isp1763_dbg(priv,
					    "short bulk, %d instead %zu "
					    "with URB_SHORT_NOT_OK flag.\n",
					    length, qtd->length);
			}

			if (urb->status == -EINPROGRESS)
				urb->status = 0;
			qtd = clean_up_qtdlist(qtd);
			isp1763_urb_done(priv, urb, urb->status);

		} else if (qtd->status & URB_COMPLETE_NOTIFY) {
			/* that was the last qtd of that URB */

			if (urb->status == -EINPROGRESS)
				urb->status = 0;

			qtd = clean_this_qtd(qtd);
			isp1763_urb_done(priv, urb, urb->status);

		} else {
			/* next QTD of this URB */

			qtd = clean_this_qtd(qtd);
			BUG_ON(!qtd);
		}

		if (qtd)
			enqueue_an_ATL_packet(usb_hcd, qh, qtd);

		/* NOTE: Move to the beginning of while (done_map) loop? */
		skip_map = isp1763_readw(usb_hcd->regs +
					 HC_ATL_PTD_SKIPMAP_REG);
	}
}


/* NOTE:
 * It seems that setting up two memory banks in the isp1760's code
 * is merely for convenience's sake; right now trying with:
 *
 * 1. Setup up the memory register for the PTD
 * 2. Copy PTD from device to the driver's PTD copy
 * 3. Process the PTD data
 * (later on)
 * 4. Setup the memory register for the payload
 * 5. Copy payload from device to the driver's payload space
 *
 * There may be a gotcha here if the isp1760 is taking advantage of the ISP_BANK
 * ability to automatically increment the address per bank on consecutive access
 * to the same bank; It does not seem so because atl_regs & payload are
 * reassigned at every while(done_map) iteration.
 */
static void do_intl_int(struct usb_hcd *usb_hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(usb_hcd);
	u16 done_map, skip_map;
	struct ptd ptd;
	struct urb *urb = NULL;
	u32 int_regs;
	u32 int_regs_base;
	u32 payload;
	u32 length;
	u16 or_map;
	int error;
	u32 queue_entry;
	struct isp1763_qtd *qtd;
	struct isp1763_qh *qh;

	done_map = isp1763_readw(usb_hcd->regs + HC_INT_PTD_DONEMAP_REG);

	/* NOTE: Move inside while (done_map) at the beginning and remove the
	 * skip_map read at the end of the loop?
	 */
	skip_map = isp1763_readw(usb_hcd->regs + HC_INT_PTD_SKIPMAP_REG);

	or_map = isp1763_readw(usb_hcd->regs + HC_INT_IRQ_MASK_OR_REG);
	or_map &= ~done_map;
	isp1763_writew(or_map, usb_hcd->regs + HC_INT_IRQ_MASK_OR_REG);

	int_regs_base = INT_REGS_OFFSET;

	while (done_map) {
		u32 dw1;
		u32 dw3;

		/* 32-bit to 16-bit done_map adjustment */
		queue_entry = __ffs((u32) done_map);
		BUG_ON(queue_entry > 16);

		done_map &= ~(1 << queue_entry);
		skip_map |= 1 << queue_entry;

		int_regs =
		    int_regs_base + (queue_entry * sizeof(struct ptd));
		urb = priv->int_ints[queue_entry].urb;
		qtd = priv->int_ints[queue_entry].qtd;
		qh = priv->int_ints[queue_entry].qh;
		payload = priv->int_ints[queue_entry].payload;

		if (!qh) {
			printk(KERN_ERR "(INT) qh is 0\n");
			continue;
		}
		isp1763_read_mem(usb_hcd, (u16) int_regs, (u16 *) &ptd,
				 sizeof(ptd));

		dw1 = le32_to_cpu(ptd.dw1);
		dw3 = le32_to_cpu(ptd.dw3);
		check_int_err_status(le32_to_cpu(ptd.dw4));

		error = check_error(&ptd);
		if (error) {
#if 0
			printk(KERN_ERR "Error in %s().\n", __func__);
			printk(KERN_ERR "IN dw0: %08x dw1: %08x dw2: %08x "
			       "dw3: %08x dw4: %08x dw5: %08x dw6: "
			       "%08x dw7: %08x\n",
			       ptd.dw0, ptd.dw1, ptd.dw2, ptd.dw3,
			       ptd.dw4, ptd.dw5, ptd.dw6, ptd.dw7);
#endif
			urb->status = -EPIPE;
			priv->int_ints[queue_entry].qh->toggle = 0;
			priv->int_ints[queue_entry].qh->ping = 0;

		} else {
			priv->int_ints[queue_entry].qh->toggle =
			    dw3 & (1 << 25);
			priv->int_ints[queue_entry].qh->ping =
			    dw3 & (1 << 26);
		}

		if (urb->dev->speed != USB_SPEED_HIGH)
			length = PTD_XFERRED_LENGTH_LO(dw3);
		else
			length = PTD_XFERRED_LENGTH(dw3);

		if (length) {
			switch (DW1_GET_PID(dw1)) {
			case IN_PID:

				isp1763_read_mem(usb_hcd, (u16) payload,
						 priv->
						 int_ints[queue_entry].
						 data_buffer, length);
			case OUT_PID:

				urb->actual_length += length;

			case SETUP_PID:
				break;
			}
		}

		priv->int_ints[queue_entry].data_buffer = NULL;
		priv->int_ints[queue_entry].urb = NULL;
		priv->int_ints[queue_entry].qtd = NULL;
		priv->int_ints[queue_entry].qh = NULL;

		isp1763_writew(skip_map, usb_hcd->regs +
			       HC_INT_PTD_SKIPMAP_REG);
		free_mem(priv, payload);

		if (urb->status == -EPIPE) {
			/* HALT received */

			qtd = clean_up_qtdlist(qtd);
			isp1763_urb_done(priv, urb, urb->status);

		} else if (qtd->status & URB_COMPLETE_NOTIFY) {

			if (urb->status == -EINPROGRESS)
				urb->status = 0;

			qtd = clean_this_qtd(qtd);
			isp1763_urb_done(priv, urb, urb->status);

		} else {
			/* next QTD of this URB */

			qtd = clean_this_qtd(qtd);
			BUG_ON(!qtd);
		}

		if (qtd)
			enqueue_an_INT_packet(usb_hcd, qh, qtd);

		/* NOTE: Move to the beginning of while (done_map) loop? */
		skip_map = isp1763_readw(usb_hcd->regs +
					 HC_INT_PTD_SKIPMAP_REG);
	}
}

#define max_packet(wMaxPacketSize) ((wMaxPacketSize) & 0x07ff)

/*
 * GIT COMMIT: 1b9a38bfa6e664ff02511314f5586d711c83cc91 adapted
 * "USB: EHCI: fix handling of unusual interrupt intervals"
 */
static struct isp1763_qh *qh_make(struct isp1763_hcd *priv,
				  struct urb *urb, gfp_t flags)
{
	struct isp1763_qh *qh;
	int is_input, type;

	qh = isp1763_qh_alloc(priv, flags);
	if (!qh)
		return qh;

	/*
	 * init endpoint/device data for this QH
	 */
	is_input = usb_pipein(urb->pipe);
	type = usb_pipetype(urb->pipe);

	if (type == PIPE_INTERRUPT) {

		if (urb->dev->speed == USB_SPEED_HIGH) {

			qh->period = urb->interval >> 3;
			if (qh->period == 0 && urb->interval != 1) {
				/* NOTE interval 2 or 4 uframes could work.
				 * But interval 1 scheduling is simpler, and
				 * includes high bandwidth.
				 */
				urb->interval = 1;
			} else if (qh->period > priv->periodic_size) {
				qh->period = priv->periodic_size;
				urb->interval = qh->period << 3;
			}
		} else {
			qh->period = urb->interval;
			if (qh->period > priv->periodic_size) {
				qh->period = priv->periodic_size;
				urb->interval = qh->period;
			}
		}
	}

	/* support for tt scheduling, and access to toggles */
	qh->dev = urb->dev;

	if (!usb_pipecontrol(urb->pipe))
		usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
			      !is_input, 1);
	return qh;
}

/*
 * For control/bulk/interrupt, return QH with these TDs appended.
 * Allocates and initializes the QH if necessary.
 * Returns null if it can't allocate a QH it needs to.
 * If the QH has TDs (urbs) already, that's great.
 */
static struct isp1763_qh *qh_append_tds(struct isp1763_hcd *priv,
					struct urb *urb,
					struct list_head *qtd_list,
					int epnum, void **ptr)
{
	struct isp1763_qh *qh;
	struct isp1763_qtd *qtd;
	struct isp1763_qtd *prev_qtd;

	qh = (struct isp1763_qh *) *ptr;
	if (!qh) {
		/* can't sleep here, we have priv->lock... */
		qh = qh_make(priv, urb, GFP_ATOMIC);
		if (!qh)
			return qh;
		*ptr = qh;
	}

	qtd = list_entry(qtd_list->next, struct isp1763_qtd, qtd_list);
	if (!list_empty(&qh->qtd_list))
		prev_qtd = list_entry(qh->qtd_list.prev,
				      struct isp1763_qtd, qtd_list);
	else
		prev_qtd = NULL;

	list_splice(qtd_list, qh->qtd_list.prev);
	if (prev_qtd) {
		BUG_ON(prev_qtd->hw_next);
		prev_qtd->hw_next = qtd;
	}

	urb->hcpriv = qh;
	return qh;
}

static void qtd_list_free(struct isp1763_hcd *priv, struct urb *urb,
			  struct list_head *qtd_list)
{
	struct list_head *entry, *temp;

	list_for_each_safe(entry, temp, qtd_list) {
		struct isp1763_qtd *qtd;

		qtd = list_entry(entry, struct isp1763_qtd, qtd_list);
		list_del(&qtd->qtd_list);
		isp1763_qtd_free(qtd);
	}
}

static int isp1763_prepare_enqueue(struct isp1763_hcd *priv,
				   struct urb *urb,
				   struct list_head *qtd_list,
				   gfp_t mem_flags, packet_enqueue *p)
{
	struct isp1763_qtd *qtd;
	int epnum;
	unsigned long flags;
	struct isp1763_qh *qh = NULL;
	int rc;
	int qh_busy;

	qtd = list_entry(qtd_list->next, struct isp1763_qtd, qtd_list);
	epnum = urb->ep->desc.bEndpointAddress;

	spin_lock_irqsave(&priv->lock, flags);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &priv_to_hcd(priv)->flags)) {
		rc = -ESHUTDOWN;
		goto done;
	}
	rc = usb_hcd_link_urb_to_ep(priv_to_hcd(priv), urb);
	if (rc)
		goto done;

	qh = urb->ep->hcpriv;
	if (qh)
		qh_busy = !list_empty(&qh->qtd_list);
	else
		qh_busy = 0;

	qh = qh_append_tds(priv, urb, qtd_list, epnum, &urb->ep->hcpriv);
	if (!qh) {
		usb_hcd_unlink_urb_from_ep(priv_to_hcd(priv), urb);
		rc = -ENOMEM;
		goto done;
	}

	if (!qh_busy)
		p(priv_to_hcd(priv), qh, qtd);

done:
	spin_unlock_irqrestore(&priv->lock, flags);
	if (!qh)
		qtd_list_free(priv, urb, qtd_list);
	return rc;
}

static struct isp1763_qtd *isp1763_qtd_alloc(struct isp1763_hcd *priv,
					     gfp_t flags)
{
	struct isp1763_qtd *qtd;

	qtd = kmem_cache_zalloc(qtd_cachep, flags);
	if (qtd)
		INIT_LIST_HEAD(&qtd->qtd_list);

	return qtd;
}

/*
 * create a list of filled qtds for this URB; won't link into qh.
 */
static struct list_head *qh_urb_transaction(struct isp1763_hcd *priv,
					    struct urb *urb,
					    struct list_head *head,
					    gfp_t flags)
{
	struct isp1763_qtd *qtd, *qtd_prev;
	void *buf;
	int len, maxpacket;
	int is_input;
	u32 token;

	/*
	 * URBs map to sequences of QTDs:  one logical transaction
	 */
	qtd = isp1763_qtd_alloc(priv, flags);
	if (!qtd)
		return NULL;

	list_add_tail(&qtd->qtd_list, head);
	qtd->urb = urb;
	urb->status = -EINPROGRESS;

	token = 0;
	/* for split transactions, SplitXState initialized to zero */

	len = urb->transfer_buffer_length;
	is_input = usb_pipein(urb->pipe);
	if (usb_pipecontrol(urb->pipe)) {
		/* SETUP pid */
		qtd_fill(qtd, urb->setup_packet,
			 sizeof(struct usb_ctrlrequest),
			 token | SETUP_PID);

		/* ... and always at least one more pid */
		token ^= DATA_TOGGLE;
		qtd_prev = qtd;
		qtd = isp1763_qtd_alloc(priv, flags);
		if (!qtd)
			goto cleanup;
		qtd->urb = urb;
		qtd_prev->hw_next = qtd;
		list_add_tail(&qtd->qtd_list, head);

		/* for zero length DATA stages, STATUS is always IN */
		if (len == 0)
			token |= IN_PID;
	}

	/*
	 * data transfer stage:  buffer setup
	 */
	buf = urb->transfer_buffer;

	if (is_input)
		token |= IN_PID;
	else
		token |= OUT_PID;

	maxpacket =
	    max_packet(usb_maxpacket(urb->dev, urb->pipe, !is_input));

	/*
	 * buffer gets wrapped in one or more qtds;
	 * last one may be "short" (including zero len)
	 * and may serve as a control status ack
	 */
	for (;;) {
		int this_qtd_len;

		if (!buf && len) {
			/* XXX This looks like usb storage / SCSI bug */
			printk(KERN_ERR
			       "buf is null, dma is %08lx len is %d\n",
			       (long unsigned) urb->transfer_dma, len);
			WARN_ON(1);
		}

		this_qtd_len = qtd_fill(qtd, buf, len, token);
		len -= this_qtd_len;
		buf += this_qtd_len;

		/* qh makes control packets use qtd toggle; maybe switch it */
		if ((maxpacket & (this_qtd_len + (maxpacket - 1))) == 0)
			token ^= DATA_TOGGLE;

		if (len <= 0)
			break;

		qtd_prev = qtd;
		qtd = isp1763_qtd_alloc(priv, flags);
		if (!qtd)
			goto cleanup;
		qtd->urb = urb;
		qtd_prev->hw_next = qtd;
		list_add_tail(&qtd->qtd_list, head);
	}

	/*
	 * control requests may need a terminating data "status" ack;
	 * bulk ones may need a terminating short packet (zero length).
	 */
	if (urb->transfer_buffer_length != 0) {
		int one_more = 0;

		if (usb_pipecontrol(urb->pipe)) {
			one_more = 1;
			/* "in" <--> "out"  */
			token ^= IN_PID;
			/* force DATA1 */
			token |= DATA_TOGGLE;
		} else if (usb_pipebulk(urb->pipe)
			   && (urb->transfer_flags & URB_ZERO_PACKET)
			   && !(urb->transfer_buffer_length % maxpacket)) {
			one_more = 1;
		}
		if (one_more) {
			qtd_prev = qtd;
			qtd = isp1763_qtd_alloc(priv, flags);
			if (!qtd)
				goto cleanup;
			qtd->urb = urb;
			qtd_prev->hw_next = qtd;
			list_add_tail(&qtd->qtd_list, head);

			/* never any data in such packets */
			qtd_fill(qtd, NULL, 0, token);
		}
	}

	qtd->status = URB_COMPLETE_NOTIFY;
	return head;

cleanup:
	qtd_list_free(priv, urb, head);
	return NULL;
}

static int isp1763_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			       gfp_t mem_flags)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	struct list_head qtd_list;
	packet_enqueue *pe;

	INIT_LIST_HEAD(&qtd_list);

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_BULK:

		if (!qh_urb_transaction(priv, urb, &qtd_list, mem_flags))
			return -ENOMEM;
		pe = enqueue_an_ATL_packet;
		break;

	case PIPE_INTERRUPT:
		if (!qh_urb_transaction(priv, urb, &qtd_list, mem_flags))
			return -ENOMEM;
		pe = enqueue_an_INT_packet;
		break;

	case PIPE_ISOCHRONOUS:
		printk(KERN_ERR "PIPE_ISOCHRONOUS ain't supported\n");
	default:
		return -EPIPE;
	}

	return isp1763_prepare_enqueue(priv, urb, &qtd_list, mem_flags,
				       pe);
}

static int isp1763_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
			       int status)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	struct inter_packet_info *ints;
	u32 i;
	u32 reg_base, or_reg, skip_reg;
	unsigned long flags;
	struct ptd ptd;
	packet_enqueue *pe;

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_ISOCHRONOUS:
		return -EPIPE;
		break;

	case PIPE_INTERRUPT:
		ints = priv->int_ints;
		reg_base = INT_REGS_OFFSET;
		or_reg = HC_INT_IRQ_MASK_OR_REG;
		skip_reg = HC_INT_PTD_SKIPMAP_REG;
		pe = enqueue_an_INT_packet;
		break;

	default:
		ints = priv->atl_ints;
		reg_base = ATL_REGS_OFFSET;
		or_reg = HC_ATL_IRQ_MASK_OR_REG;
		skip_reg = HC_ATL_PTD_SKIPMAP_REG;
		pe = enqueue_an_ATL_packet;
		break;
	}

	memset(&ptd, 0, sizeof(ptd));
	spin_lock_irqsave(&priv->lock, flags);

	/* NOTE: isp1763 only have 16 PTDs */
	for (i = 0; i < NUM_OF_PTD; i++) {
		if (ints->urb == urb) {
			u16 skip_map;
			u16 or_map;
			struct isp1763_qtd *qtd;
			struct isp1763_qh *qh = ints->qh;

			skip_map = isp1763_readw(hcd->regs + skip_reg);
			skip_map |= 1 << (u16) i;
			isp1763_writew(skip_map, hcd->regs + skip_reg);

			or_map = isp1763_readw(hcd->regs + or_reg);
			or_map &= ~(1 << (u16) i);
			isp1763_writew(or_map, hcd->regs + or_reg);

			isp1763_write_mem(hcd, (u16 *) &ptd,
					  reg_base + i * sizeof(ptd),
					  sizeof(ptd));

			qtd = ints->qtd;
			qtd = clean_up_qtdlist(qtd);

			free_mem(priv, ints->payload);

			ints->urb = NULL;
			ints->qh = NULL;
			ints->qtd = NULL;
			ints->data_buffer = NULL;
			ints->payload = 0;

			isp1763_urb_done(priv, urb, status);
			if (qtd)
				pe(hcd, qh, qtd);
			break;

		} else if (ints->qtd) {
			struct isp1763_qtd *qtd, *prev_qtd = ints->qtd;

			for (qtd = ints->qtd->hw_next; qtd;
			     qtd = qtd->hw_next) {
				if (qtd->urb == urb) {
					prev_qtd->hw_next =
					    clean_up_qtdlist(qtd);
					isp1763_urb_done(priv, urb,
							 status);
					break;
				}
				prev_qtd = qtd;
			}
			/* we found the urb before the end of the list */
			if (qtd)
				break;
		}
		ints++;
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

/**
 * isp1763_irq - host controller interrupt routine
 * @ctrl: controller structure
 *
 * Called by main IRQ handler in isp1763_base.c and not registered via
 * usb_hcd structure.
 */
static int isp1763_hcd_irq(struct isp1763_controller *ctrl, u32 flags)
{
	struct usb_hcd *usb_hcd = ctrl->priv;
	struct isp1763_hcd *priv = hcd_to_priv(usb_hcd);
	u16 imask;
	irqreturn_t irqret = IRQ_NONE;

	spin_lock(&priv->lock);

	if (!(usb_hcd->state & HC_STATE_RUNNING))
		goto leave;

	imask = flags;
	if (unlikely(!imask))
		goto leave;

	if (imask & HC_ATL_INT)
		do_atl_int(usb_hcd);

	if (imask & HC_INTL_INT)
		do_intl_int(usb_hcd);

	irqret = IRQ_HANDLED;
leave:
	spin_unlock(&priv->lock);
	return irqret;
}

static int isp1763_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u32 temp, status = 0;
	u32 mask;
	int retval = 1;
	unsigned long flags;

	/* if !USB_SUSPEND, root hub timers won't get shut down ... */
	if (!HC_IS_RUNNING(hcd->state))
		return 0;

	/* init status to no-changes */
	buf[0] = 0;
	mask = PORT_CSC;

	spin_lock_irqsave(&priv->lock, flags);
	temp = isp1763_readl(hcd->regs + HC_PORTSC1);

	if (temp & PORT_OWNER) {
		if (temp & PORT_CSC) {
			temp &= ~PORT_CSC;
			isp1763_writel(temp, hcd->regs + HC_PORTSC1);
			goto done;
		}
	}

	/*
	 * Return status information even for ports with OWNER set.
	 * Otherwise khubd wouldn't see the disconnect event when a
	 * high-speed device is switched over to the companion
	 * controller by the user.
	 */

	if ((temp & mask) != 0
	    || ((temp & PORT_RESUME) != 0
		&& time_after_eq(jiffies, priv->reset_done))) {
		buf[0] |= 1 << (0 + 1);
		status = STS_PCD;
	}
	/* FIXME autosuspend idle root hubs */
done:
	spin_unlock_irqrestore(&priv->lock, flags);
	return status ? retval : 0;
}

/* ref: isp1763: pehci.c:pehci_hub_descriptor() */
static void isp1763_hub_descriptor(struct isp1763_hcd *priv,
				   struct usb_hub_descriptor *desc)
{
	int ports = HCS_N_PORTS(priv->hcs_params);
	u16 temp;

	desc->bDescriptorType = 0x29;
	/* priv 1.0, 2.3.9 says 20ms max */
	desc->bPwrOn2PwrGood = 10;
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = ports;
	temp = 1 + (ports / 8);
	desc->bDescLength = 7 + 2 * temp;

	/* two bitmaps:  ports removable, and usb 1.0 legacy PortPwrCtrlMask */
	memset(&desc->bitmap[0], 0, temp);
	memset(&desc->bitmap[temp], 0xff, temp);

	/* per-port overcurrent reporting; no power switching */
	temp = 0x0008;

	if (HCS_PPC(priv->hcs_params))
		temp |= 0x0001;	/* per-port power control */
	else
		temp |= 0x0002;	/* no power switching */

	desc->wHubCharacteristics = cpu_to_le16(temp);
}

#define	PORT_WAKE_BITS	(PORT_WKOC_E|PORT_WKDISC_E|PORT_WKCONN_E)

static int check_reset_complete(struct isp1763_hcd *priv, int index,
				u32 __iomem *status_reg, int port_status)
{
	if (!(port_status & PORT_CONNECT))
		return port_status;

	/* if reset finished and it's still not enabled -- handoff */
	if (!(port_status & PORT_PE)) {

		printk(KERN_ERR "port %d full speed --> companion\n",
		       index + 1);

		port_status |= PORT_OWNER;
		port_status &= ~PORT_RWC_BITS;
		isp1763_writel(port_status, status_reg);

	} else
		printk(KERN_ERR "port %d high speed\n", index + 1);

	return port_status;
}

static int isp1763_hub_control(struct usb_hcd *hcd, u16 typeReq,
			       u16 wValue, u16 wIndex, char *buf,
			       u16 wLength)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	int ports = HCS_N_PORTS(priv->hcs_params);
	u32 __iomem *status_reg = hcd->regs + HC_PORTSC1;
	u32 temp, status;
	unsigned long flags;
	int retval = 0;
	unsigned selector;

	spin_lock_irqsave(&priv->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		switch (wValue) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			goto error;
		}
		break;

	case ClearPortFeature:
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		temp = isp1763_readl(status_reg);

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			isp1763_writel(temp & ~PORT_PE, status_reg);
			break;
		case USB_PORT_FEAT_C_ENABLE:
			/* XXX error? */
			break;
		case USB_PORT_FEAT_SUSPEND:
			if (temp & PORT_RESET)
				goto error;

			if (temp & PORT_SUSPEND) {
				if ((temp & PORT_PE) == 0)
					goto error;
				/* resume signaling for 20 msec */
				temp &= ~(PORT_RWC_BITS);
				isp1763_writel(temp | PORT_RESUME, status_reg);
				priv->reset_done = jiffies +
				    msecs_to_jiffies(20);
			}
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			/* we auto-clear this feature */
			break;
		case USB_PORT_FEAT_POWER:
			if (HCS_PPC(priv->hcs_params))
				isp1763_writel(temp & ~PORT_POWER, status_reg);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			isp1763_writel(temp | PORT_CSC, status_reg);
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			/* XXX error ? */
			break;
		case USB_PORT_FEAT_C_RESET:
			/* GetPortStatus clears reset */
			break;
		default:
			goto error;
		}
		isp1763_readl(hcd->regs + HC_USBCMD);
		break;

	case GetHubDescriptor:
		isp1763_hub_descriptor(priv, (struct usb_hub_descriptor *)buf);
		break;

	case GetHubStatus:
		/* no hub-wide feature/status flags */
		memset(buf, 0, 4);
		break;

	case GetPortStatus:
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		status = 0;
		temp = isp1763_readl(status_reg);

		/* wPortChange bits */
		if (temp & PORT_CSC) {
			status |= 1 << USB_PORT_FEAT_C_CONNECTION;
		}

		/* whoever resumes must GetPortStatus to complete it!! */
		if (temp & PORT_RESUME) {
			printk(KERN_ERR
			       "Port resume should be skipped.\n");

			/* Remote Wakeup received? */
			if (!priv->reset_done) {
				/* resume signaling for 20 msec */
				priv->reset_done = jiffies
				    + msecs_to_jiffies(20);
				/* check the port again */
				mod_timer(&priv_to_hcd(priv)->rh_timer,
					  priv->reset_done);
			}

			/* resume completed? */
			else if (time_after_eq(jiffies, priv->reset_done)) {
				status |= 1 << USB_PORT_FEAT_C_SUSPEND;
				priv->reset_done = 0;

				/* stop resume signaling */
				temp = isp1763_readl(status_reg);
				isp1763_writel(temp &
					       ~(PORT_RWC_BITS |
						 PORT_RESUME), status_reg);
				retval =
				    handshake(priv, status_reg,
					      PORT_RESUME, 0, 2000); /* 2msec */
				if (retval != 0) {
					isp1763_err(priv,
						    "port %d resume error %d\n",
						    wIndex + 1, retval);
					goto error;
				}
				temp &= ~(PORT_SUSPEND|PORT_RESUME|(3 << 10));
			}
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if ((temp & PORT_RESET)
		    && time_after_eq(jiffies, priv->reset_done)) {
			status |= 1 << USB_PORT_FEAT_C_RESET;
			priv->reset_done = 0;

			/* force reset to complete */
			isp1763_writel(temp & ~PORT_RESET, status_reg);
			/* REVISIT:  some hardware needs 550+ usec to clear
			 * this bit; seems too long to spin routinely...
			 */
			retval = handshake(priv, status_reg,
					   PORT_RESET, 0, 750);
			if (retval != 0) {
				isp1763_err(priv,
					    "port %d reset error %d\n",
					    wIndex + 1, retval);
				goto error;
			}

			/* see what we found out */
			temp =
			    check_reset_complete(priv, wIndex, status_reg,
						 isp1763_readl(status_reg));
		}
		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */

		if (temp & PORT_OWNER)
			printk(KERN_ERR "Warning: PORT_OWNER is set\n");

		if (temp & PORT_CONNECT) {
			status |= 1 << USB_PORT_FEAT_CONNECTION;
			/* status may be from integrated TT */
			status |= ehci_port_speed(priv, temp);
		}
		if (temp & PORT_PE)
			status |= 1 << USB_PORT_FEAT_ENABLE;
		if (temp & (PORT_SUSPEND | PORT_RESUME))
			status |= 1 << USB_PORT_FEAT_SUSPEND;
		if (temp & PORT_RESET)
			status |= 1 << USB_PORT_FEAT_RESET;
		if (temp & PORT_POWER)
			status |= 1 << USB_PORT_FEAT_POWER;

		put_unaligned(cpu_to_le32(status), (__le32 *) buf);
		break;

	case SetHubFeature:
		switch (wValue) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			goto error;
		}
		break;

	case SetPortFeature:
		selector = wIndex >> 8;
		wIndex &= 0xff;
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		temp = isp1763_readl(status_reg);
		if (temp & PORT_OWNER)
			break;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			isp1763_writel(temp | PORT_PE, status_reg);
			break;

		case USB_PORT_FEAT_SUSPEND:
			if ((temp & PORT_PE) == 0
			    || (temp & PORT_RESET) != 0)
				goto error;

			isp1763_writel(temp | PORT_SUSPEND, status_reg);
			break;
		case USB_PORT_FEAT_POWER:
			if (HCS_PPC(priv->hcs_params))
				isp1763_writel(temp | PORT_POWER,
					       status_reg);
			break;
		case USB_PORT_FEAT_RESET:
			if (temp & PORT_RESUME)
				goto error;

			if ((temp & (PORT_PE | PORT_CONNECT)) ==
			    PORT_CONNECT && PORT_USB11(temp)) {
				temp |= PORT_OWNER;
			} else {
				if ((temp & PORT_CONNECT) == 0) {
					printk(KERN_ERR
					       "ERROR: Port not connected\n");
					goto error;
				}
				if ((temp & PORT_POWER) == 0) {
					printk(KERN_ERR
					       "ERROR: Port power not enabled!\n");
					goto error;
				}

				temp |= PORT_RESET;
				temp &= ~PORT_PE;

				isp1763_writel(temp, status_reg);
				/*
				 * caller must wait, then call GetPortStatus
				 * usb 2.0 spec says 50 ms resets on root
				 */
				priv->reset_done = jiffies +
				    msecs_to_jiffies(50);
				mdelay(50);

				temp &= ~PORT_RESET;
				isp1763_writel(temp, status_reg);

				while (isp1763_readl(status_reg) & PORT_RESET)
					continue; /* do nothing */

				temp |= PORT_PE;
				isp1763_writel(temp, status_reg);
			}
			break;
		default:
			goto error;
		}
		isp1763_readl(hcd->regs + HC_USBCMD);
		break;

	default:
error:
		/* "stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	return retval;
}

static void isp1763_endpoint_disable(struct usb_hcd *usb_hcd,
				     struct usb_host_endpoint *ep)
{
	struct isp1763_hcd *priv = hcd_to_priv(usb_hcd);
	struct isp1763_qh *qh;
	struct isp1763_qtd *qtd;
	unsigned long flags;
	struct urb *urb;

	spin_lock_irqsave(&priv->lock, flags);
	qh = ep->hcpriv;
	if (!qh)
		goto out;

	ep->hcpriv = NULL;
	do {
		/* more than entry might get removed */
		if (list_empty(&qh->qtd_list))
			break;

		qtd = list_first_entry(&qh->qtd_list, struct isp1763_qtd,
				       qtd_list);

		if (qtd->status & URB_ENQUEUED) {

			spin_unlock_irqrestore(&priv->lock, flags);
			isp1763_urb_dequeue(usb_hcd, qtd->urb,
					    -ECONNRESET);
			spin_lock_irqsave(&priv->lock, flags);
		} else {
			urb = qtd->urb;
			clean_up_qtdlist(qtd);
			isp1763_urb_done(priv, urb, -ECONNRESET);
		}
	} while (1);

	qh_destroy(qh);
	/* remove requests and leak them.
	 * ATL are pretty fast done, INT could take a while...
	 * The latter shoule be removed
	 */
out:
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int isp1763_get_frame(struct usb_hcd *hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u32 fr;

	fr = isp1763_readl(hcd->regs + HC_FRINDEX);
	return (fr >> 3) % priv->periodic_size;
}

static void isp1763_stop(struct usb_hcd *hcd)
{
	struct isp1763_hcd *priv = hcd_to_priv(hcd);
	u32 command;

	isp1763_hub_control(hcd, ClearPortFeature, USB_PORT_FEAT_POWER, 1,
			    NULL, 0);
	mdelay(20);

	spin_lock_irq(&priv->lock);
	/* FIXME: is this really needed? */
	ehci_reset(priv);

	/* Stop host controller */
	command = isp1763_readl(hcd->regs + HC_USBCMD);
	command &= ~CMD_RUN;
	isp1763_writel(command, hcd->regs + HC_USBCMD);

	spin_unlock_irq(&priv->lock);

	isp1763_writel(0, hcd->regs + HC_CONFIGFLAG);
}

static void isp1763_shutdown(struct usb_hcd *hcd)
{
	u16 temp;
	u32 command;

	isp1763_stop(hcd);
	temp = isp1763_readw(hcd->regs + HC_HW_MODE_CTRL);
	isp1763_writew(temp &=
		       ~HW_GLOBAL_INTR_EN, hcd->regs + HC_HW_MODE_CTRL);

	command = isp1763_readl(hcd->regs + HC_USBCMD);
	command &= ~CMD_RUN;
	isp1763_writel(command, hcd->regs + HC_USBCMD);
}

static const struct hc_driver isp1763_hc_driver = {
	.description = "isp1763-hcd",
	.product_desc = "ST-Ericsson ISP1763 USB Host Controller",
	.hcd_priv_size = sizeof(struct isp1763_hcd),
	.flags = HCD_MEMORY | HCD_USB2,
	.reset = isp1763_hc_setup,
	.start = isp1763_run,
	.stop = isp1763_stop,
	.shutdown = isp1763_shutdown,
	.urb_enqueue = isp1763_urb_enqueue,
	.urb_dequeue = isp1763_urb_dequeue,
	.endpoint_disable = isp1763_endpoint_disable,
	.get_frame_number = isp1763_get_frame,
	.hub_status_data = isp1763_hub_status_data,
	.hub_control = isp1763_hub_control,
};

int __init init_kmem_once(void)
{
	qtd_cachep = kmem_cache_create("isp1763_qtd",
				       sizeof(struct isp1763_qtd), 0,
				       SLAB_TEMPORARY | SLAB_MEM_SPREAD,
				       NULL);

	if (!qtd_cachep)
		return -ENOMEM;

	qh_cachep =
	    kmem_cache_create("isp1763_qh", sizeof(struct isp1763_qh), 0,
			      SLAB_TEMPORARY | SLAB_MEM_SPREAD, NULL);

	if (!qh_cachep) {
		kmem_cache_destroy(qtd_cachep);
		return -ENOMEM;
	}

	return 0;
}

void deinit_kmem_cache(void)
{
	kmem_cache_destroy(qtd_cachep);
	kmem_cache_destroy(qh_cachep);
}

/**
 * isp1763_hcd_resume - resume host
 * @ctrl: controller
 *
 * Register and start up host controller, called by OTG code either during
 * init or after a role change
 */
static int isp1763_hcd_resume(struct isp1763_controller *ctrl)
{
	struct usb_hcd *hcd = ctrl->priv;


	usb_add_hcd(hcd, 0, 0);

	ctrl->active = 1;
	return 1;
}

/**
 * isp1763_hcd_suspend - suspend host
 * @ctrl: controller
 *
 * Stop and deregister host controller, called by OTG code after a role
 * change.
 */
static int isp1763_hcd_suspend(struct isp1763_controller *ctrl)
{
	struct usb_hcd *hcd = ctrl->priv;

	if (hcd->state == HC_STATE_RUNNING)
		usb_remove_hcd(hcd);

	ctrl->active = 0;
	return 0;
}

/**
 * isp1763_hcd_probe - initialize USB host controller structures
 * @ctrl: controller
 */
static int isp1763_hcd_probe(struct isp1763_controller *ctrl)
{
	struct usb_hcd *hcd;
	struct isp1763_hcd *priv;

	if (usb_disabled())
		return -ENODEV;

	if (!ctrl->device)
		return -ENODEV;

	hcd = usb_create_hcd(&isp1763_hc_driver, ctrl->device,
					dev_name(ctrl->device));
	if (!hcd)
		return -ENOMEM;

	priv = hcd_to_priv(hcd);
	priv->devflags = 0;
	init_memory(priv);

	ctrl->active = 0;
	ctrl->priv = hcd;

	hcd->regs = ctrl->regs;
	if (hcd->regs)
		return 0;

	usb_put_hcd(hcd);
	return -EIO;
}

static struct isp1763_controller isp1763_hc_controller = {
	.active = 0,
	.do_irq = isp1763_hcd_irq,
	.suspend = isp1763_hcd_suspend,
	.resume = isp1763_hcd_resume,
	.probe = isp1763_hcd_probe,
};

static int __init isp1763_hc_init(void)
{
	isp1763_register_ctrl(&isp1763_hc_controller, ROLE_HOST);
	init_kmem_once();

	printk(KERN_ERR "isp1763 host controller driver loaded\n");
	return 0;
}

static void __exit isp1763_hc_exit(void)
{
	isp1763_unregister_ctrl(ROLE_HOST);
	if (isp1763_hc_controller.priv)
		usb_put_hcd(isp1763_hc_controller.priv);
	return;
}

module_init(isp1763_hc_init);
module_exit(isp1763_hc_exit);


MODULE_DESCRIPTION("ISP1763 USB host controller driver");
MODULE_AUTHOR("Richard Retanubun <richardretanubun@ruggedcom.com>");
MODULE_LICENSE("GPL v2");
