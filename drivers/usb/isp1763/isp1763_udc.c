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
 * ISP1763A UDC driver
 *
 * TODO:
 * - check locking for completeness
 * - use double buffering
 *
 * (c) 2010 F. Voegel, Carangul.Tech
 * (c) 2010 I+ME ACTIA Informatik und Mikroelektronik GmbH
 *
 */
#define DEBUG

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
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
#include <linux/gpio.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/cdc.h>

/* FIXME: Cleanup if this is not needed, comment out for now, just in case
* #include <asm/byteorder.h>
* #include <asm/dma.h>
* #include <asm/system.h>
*/

#include "isp1763.h"
#include "isp1763_udc.h"

#define DRIVER_NAME "isp1763_udc"
static const char driver_name[] = DRIVER_NAME;
static const char driver_desc[] = "ISP 1763A udc";

static const char ep0name[] = "ep0";

static struct isp1763_udc udc_dev;
static struct isp_ep isp_eps[USB_MAX_ENDPOINTS];

#define USE_DOUBLE_BUFFERING

/* Use this #define to activate dump debug functions and add it where you need
 * to dump in the code */
#define ISP1763_UDC_DUMP_DEBUG

static u32 irq_bits = MASK_DCINTENABLE_RELEVANT;

/* Ladies and gentlemen, I give you... THE WHEEL */

/**
* queue_add - add item to queue
* @head: queue head
* @elem: element for which to create entry
*/
static void queue_add(struct queue **head, void *elem)
{
	struct queue *p = NULL;
	struct queue *new = kmalloc(sizeof(struct queue), GFP_ATOMIC);

	new->next = NULL;
	new->elem = elem;

	queue_printk(KERN_ERR "Adding queue entry %p with elem %p\n", new,
		     new->elem);

	if (*head) {
		for (p = *head; p->next != NULL;)
			p = p->next;
		p->next = new;
	} else
		*head = new;
}

/**
* queue_get - get item from top of queue
* @head: queue head
*/
static void *queue_get(struct queue **head)
{
	if (head && *head)
		return (*head)->elem;
	return NULL;
}

/**
* queue_topkill - remove first entry from queue
* @head: queue head
*/
static void queue_topkill(struct queue **head)
{
	struct queue *old = *head;

	queue_printk(KERN_ERR "Killing queue entry %p with elem %p\n",
		     *head, (*head)->elem);

	if (*head == NULL)
		return;

	if ((*head)->next)
		*head = (*head)->next;
	else
		*head = NULL;

	kfree(old);
}

/**
* queue_empty - is queue empty?
* @head: queue head
*/
static int queue_empty(struct queue **head)
{
	return *head == NULL;
}

/**
* queue_count - return number of entries in queue
* @head: queue head
*/
static int queue_count(struct queue **head)
{
	int cnt = 0;
	struct queue *p = *head;

	for (; p != NULL; p = p->next) {
		struct usb_request *r = p->elem;
		if (!r)
			continue;
		if (r->buf == NULL)
			queue_printk(KERN_ERR
				     "queued request %i (%p) has NULL buffer and %i size (actual=%i)\n",
				     cnt, r, r->length, r->actual);
		cnt++;
	}

	return cnt;
}

static void queue_flush(struct queue **head)
{
	while (!queue_empty(head))
		queue_topkill(head);
}

/* Use this #define to activate dump debug functions and add it where you need
 * to dump in the code */
#ifdef ISP1763_UDC_DUMP_DEBUG
/**
* dump_ctrlrequest - debug - dump control request
* @req: pointer to request structure
*/
static void dump_ctrlrequest(struct usb_ctrlrequest *req)
{
	char reqname[64] = "???";
	char typename[64] = "???";
	switch (req->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		sprintf(typename, "STD");
		switch (req->bRequest) {
		case USB_REQ_GET_STATUS:
			sprintf(reqname, "USB_REQ_GET_STATUS");
			break;
		case USB_REQ_CLEAR_FEATURE:
			sprintf(reqname, "USB_REQ_CLEAR_FEATURE");
			break;
		case USB_REQ_SET_FEATURE:
			sprintf(reqname, "USB_REQ_SET_FEATURE");
			break;
		case USB_REQ_SET_ADDRESS:
			sprintf(reqname, "USB_REQ_SET_ADDRESS(%i)",
				le16_to_cpu(req->wValue));
			break;
		case USB_REQ_GET_DESCRIPTOR:
			sprintf(reqname, "USB_REQ_GET_DESCRIPTOR");
			break;
		case USB_REQ_SET_DESCRIPTOR:
			sprintf(reqname, "USB_REQ_SET_DESCRIPTOR");
			break;
		case USB_REQ_GET_CONFIGURATION:
			sprintf(reqname, "USB_REQ_GET_CONFIGURATION");
			break;
		case USB_REQ_SET_CONFIGURATION:
			sprintf(reqname, "USB_REQ_SET_CONFIGURATION");
			break;
		case USB_REQ_GET_INTERFACE:
			sprintf(reqname, "USB_REQ_GET_INTERFACE");
			break;
		case USB_REQ_SET_INTERFACE:
			sprintf(reqname, "USB_REQ_SET_INTERFACE");
			break;
		case USB_REQ_SYNCH_FRAME:
			sprintf(reqname, "USB_REQ_SYNCH_FRAME");
			break;
		}
		break;
	case USB_TYPE_CLASS:
		sprintf(typename, "CLASS");
		switch (req->bRequest) {
		case USB_CDC_SEND_ENCAPSULATED_COMMAND:
			sprintf(reqname,
				"USB_CDC_SEND_ENCAPSULATED_COMMAND");
			break;
		case USB_CDC_GET_ENCAPSULATED_RESPONSE:
			sprintf(reqname,
				"USB_CDC_GET_ENCAPSULATED_RESPONSE");
			break;
		case USB_CDC_REQ_SET_LINE_CODING:
			sprintf(reqname, "USB_CDC_REQ_SET_LINE_CODING");
			break;
		case USB_CDC_REQ_GET_LINE_CODING:
			sprintf(reqname, "USB_CDC_REQ_GET_LINE_CODING");
			break;
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			sprintf(reqname,
				"USB_CDC_REQ_SET_CONTROL_LINE_STATE");
			break;
		case USB_CDC_REQ_SEND_BREAK:
			sprintf(reqname, "USB_CDC_REQ_SEND_BREAK");
			break;
		case USB_CDC_SET_ETHERNET_MULTICAST_FILTERS:
			sprintf(reqname,
				"USB_CDC_SET_ETHERNET_MULTICAST_FILTERS");
			break;
		case USB_CDC_SET_ETHERNET_PM_PATTERN_FILTER:
			sprintf(reqname,
				"USB_CDC_SET_ETHERNET_PM_PATTERN_FILTER");
			break;
		case USB_CDC_GET_ETHERNET_PM_PATTERN_FILTER:
			sprintf(reqname,
				"USB_CDC_GET_ETHERNET_PM_PATTERN_FILTER");
			break;
		case USB_CDC_SET_ETHERNET_PACKET_FILTER:
			sprintf(reqname,
				"USB_CDC_SET_ETHERNET_PACKET_FILTER");
			break;
		case USB_CDC_GET_ETHERNET_STATISTIC:
			sprintf(reqname, "USB_CDC_GET_ETHERNET_STATISTIC");
			break;
		}

		break;
	case USB_TYPE_VENDOR:
		sprintf(typename, "VENDOR");
		sprintf(reqname, "%.2x", req->bRequest);
		break;
	}

	printk(KERN_INFO "bRequestType: 0x%.2x (%s)\n",
		    req->bRequestType, typename);
	printk(KERN_INFO "bRequest:     0x%.2x (%s)\n", req->bRequest,
		    reqname);
	printk(KERN_INFO "wValue:       0x%.4x\n", req->wValue);
	printk(KERN_INFO "wIndex:       0x%.4x\n", req->wIndex);
	printk(KERN_INFO "wLength:      0x%.4x\n", req->wLength);
}

/**
* dump_usb_request - debug - dump USB request
* @ep: pointer to EP struct
* @req: pointer to request
*/
static void dump_usb_request(struct usb_ep *ep, struct usb_request *req)
{
	int i;

#define MAXDUMPLEN 64
	info_printk("\n%s on %s: ", __func__, ep->name);
	for (i = 0;
	     i < (req->length > MAXDUMPLEN ? MAXDUMPLEN : req->length);
	     i++) {
		info_printk("%.2x ", ((char *) req->buf)[i]);
	}
	info_printk("\n");
}
#endif /* ISP1763_UDC_DUMP_DEBUG */

static inline struct isp1763_ep *usb_ep_to_isp1763_ep(struct usb_ep *usb_ep)
{
	return container_of(usb_ep, struct isp1763_ep, ep);
}

/**
* get_ep_index - get endpoint index
* @ep: endpoint pointer
*/
static int get_ep_index(struct usb_ep *ep)
{
	int i;
	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		if (&isp_eps[i].ep == ep)
			return i;
	}
	return -1;
}

/**
* ep_set_index - set PIO endpoint index
* @dev: UDC device pointer
* @index: new EP index
*/
static void ep_set_index(struct isp1763_udc *dev, unsigned char index)
{
	isp1763_writew(index, &dev->regs->ep_index);
	isp1763_writew(~index, &dev->regs->dma_ep);
	ndelay(200);
}

/**
* ep_stall - stall endpoint
* @dev: UDC device pointer
* @idx: index of ep to stall
* @do_stall: 1=stall, 0=unstall
*/
static void ep_stall(struct isp1763_udc *dev, int idx, int do_stall)
{
	warn_printk(KERN_ERR "%sSTALLING EP INDEX %i\n",
		    do_stall ? "" : "UN", idx);
	ep_set_index(dev, idx);

	if (do_stall)
		set_stall_flag(dev);
	else
		clr_stall_flag(dev);
}

#define REAL_EP_NUM(x) (((x & 0x0F) << 1) | x >> 7)

/**
* ep_feature - handle EP feature request
* @dev: UDC device pointer
* @req: pointer to request
*/
static void
ep_feature(struct isp1763_udc *dev, struct usb_ctrlrequest *req)
{
	if (le16_to_cpu(req->wValue) == USB_ENDPOINT_HALT) {
		ep_stall(dev, REAL_EP_NUM(le16_to_cpu(req->wIndex)),
			 req->bRequest == USB_REQ_SET_FEATURE ? 1 : 0);
	} else {
		set_status_flag(dev);
	}
}

/**
* write_ep - write data to endpoint
* @dev: UDC device pointer
* @buf: buffer to send
* @buflen: buffer length
*/
static int
write_ep(struct isp1763_udc *dev, unsigned short *buf, int buflen)
{
	int i = 0;

	if (!buf && buflen > 0) {
		warn_printk(KERN_ERR
			    "%s: NULL buffer submitted with len %i!\n",
			    __func__, buflen);
		return 0;
	}

	isp1763_writew(buflen, &dev->regs->buflen);
	disable_glint__(dev);

	for (i = 0; i < (buflen / 2); i++)
		isp1763_writew(buf[i], &dev->regs->data_port);

	if (buflen % 2) {
		char *bufc = (char *) buf;
		isp1763_writew((unsigned short)
				(bufc[buflen - 1] | (bufc[buflen - 1] << 8)),
				&dev->regs->data_port);
	}

	enable_glint(dev);

	return i * 2 - (buflen % 2);
}

/**
* read_ep0 - read request from endpoint 0
* @dev: UDC device pointer
* @req: pointer to request to fill
*/
int read_ep0(struct isp_ep *ep, struct usb_request *req)
{
	struct isp1763_udc *dev = ep->udc;
	unsigned short *buf;
	int i;
	int reqreadlen;
	int buflen = 0;

	if (!req)
		return -EINVAL;

	reqreadlen = req->length - req->actual;

	disable_glint(dev);

	buf = (unsigned short *) ((char *) req->buf + req->actual);
	buflen = reqreadlen > 64 ? 64 : reqreadlen;

	for (i = 0; i < (buflen / 2); i++)
		buf[i] = isp1763_readw(&dev->regs->data_port);

	if (buflen % 2) {
		char *bufc = (char *) buf;
		bufc[buflen - 1] =
		    (char) (isp1763_readw(&dev->regs->data_port) & 0xff);
	}

	req->actual += buflen;

	/*
	 * Complete if buf filled to capacity or buflen < wMaxPacketSize
	 * (short packet)
	*/
	if (req->actual == req->length || buflen < ep->maxpacketsize) {
		req->status = 0;
		set_status_flag(dev);
		enable_glint(dev);
		return 1;
	}

	enable_glint(dev);
	return 0;
}

/**
* read_ep - read data from endpoint
* @ep: endpoint
* @req: request
*/
int read_ep(struct isp_ep *ep, struct usb_request *req)
{
	struct isp1763_udc *dev = ep->udc;
	int buflen = -1;
	unsigned short *buf;
	int i;
	int reqreadlen = req->length - req->actual;

	/*
	 * At first I believed this to be a source of errors, but actually
	 * a RX interrupt without any data is caused by a icr setting where
	 * an int is generated on the first NAK after previously having an ACK.
	 * This is it. Nothing to it, so just leave and do naught.
	 */
	if (isp1763_readw(&dev->regs->dcbufstatus) == 0)
		return -1;

	if (req->buf == NULL) {
		error_printk(KERN_ERR
			     "%s(): ERROR! req->buf == NULL! (len=%i) %p\n",
			     __func__, req->length, req);
		req->actual = 0;
		req->status = -EINVAL;
		if (!req->no_interrupt)
			if (req->complete) {
				pr_debug("Completing request %p\n", req);
				req->complete(&ep->ep, req);
			}
		return -1;
	}

	buflen = isp1763_readw(&dev->regs->buflen);

	/*
	 * Seems inevitable, but what would happen to the rest of the buffer?
	 * Does this even happen?
	*/
	if (reqreadlen < buflen)
		buflen = reqreadlen;

	buf = (unsigned short *) ((char *) req->buf + req->actual);

	disable_glint(dev);

	for (i = 0; i < (buflen / 2); i++)
		buf[i] = isp1763_readw(&dev->regs->data_port);

	if (buflen % 2) {
		char *bufc = (char *) buf;
		bufc[buflen - 1] =
		    (char) (isp1763_readw(&dev->regs->data_port) & 0xFF);
	}

	req->actual += buflen;

	enable_glint(dev);

	/*
	 * Complete if buf filled to capacity or buflen < wMaxPacketSize
	 * (short packet)
	*/
	if (req->actual == req->length || buflen < ep->maxpacketsize) {
		req->status = 0;
		isp1763_writew(0, &dev->regs->dcbufstatus);
		set_clbuf_flag(dev);
		if (!req->no_interrupt)
			if (req->complete) {
				pr_debug("Completing OUT request %p (%i bytes)\n",
					     req, req->actual);
				req->complete(&ep->ep, req);
			}
		return 1;
	}

	return 0;
}

/* EP0 FUNCTIONS */

/**
* configure_ep0 - configure control endpoint
* @dev: UDC device pointer
*/
static int configure_ep0(struct isp1763_udc *dev)
{
	int i;

	for (i = 0; i < 2; i++) {
		ep_set_index(dev, EP_INDEX(0, DIR_RX));
		isp1763_writew(64, &dev->regs->ep_maxpktsize);
		isp1763_writew(EP_TYPE_UNUSED | (i << 3),
			       &dev->regs->ep_type);

		ep_set_index(dev, EP_INDEX(0, DIR_TX));
		isp1763_writew(64, &dev->regs->ep_maxpktsize);
		isp1763_writew(EP_TYPE_UNUSED | (i << 3),
			       &dev->regs->ep_type);

		ep_set_index(dev, IDX_EP0_SETUP);
		isp1763_writew(64, &dev->regs->ep_maxpktsize);
		isp1763_writew(EP_TYPE_UNUSED | (i << 3),
			       &dev->regs->ep_type);

		udelay(5);
	}

	return 0;
}

/**
* read_ep0_setup - read control endpoint setup request
* @dev: UDC device pointer
* @buf: buffer
* @buflen: length of buffer
*/
static int read_ep0_setup(struct isp1763_udc *dev, unsigned short *buf, int buflen)
{
	int readlen = 0;
	int i;

	ep_set_index(dev, IDX_EP0_SETUP);

	readlen = isp1763_readw(&dev->regs->buflen);

	if (buflen < readlen)
		return -ENOMEM;

	disable_glint(dev);

	for (i = 0; i < ((readlen / 2) + (readlen % 2)); i++)
		buf[i] = isp1763_readw(&dev->regs->data_port);

	if (buflen % 2) {
		char *bufc = (char *) buf;
		bufc[buflen - 1] =
		    (char) (isp1763_readw(&dev->regs->data_port) & 0xff);
	}

	enable_glint(dev);

	return readlen;
}

/**
* handle_ep0_setup - handle control endpoint setup IRQ
*/
static void handle_ep0_setup(struct isp1763_udc *dev)
{
	char buffer[8];
	struct usb_ctrlrequest *setup_request;
	int readlen = 0;
	int ret = 0;

	memset(buffer, 0, sizeof(buffer));
	readlen = read_ep0_setup(dev, (unsigned short *) buffer,
						sizeof(buffer));

	if (readlen < sizeof(struct usb_ctrlrequest)) {
		pr_warning("short setup packet (%d < %zu)!\n", readlen, sizeof(struct usb_ctrlrequest));
		return;
	}
	setup_request = (struct usb_ctrlrequest *) buffer;

	if ((setup_request->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		goto non_standard;

	/* STANDARD REQUEST */
	dev->ep0->state = DIR_TX;

	switch (setup_request->bRequest) {
	case USB_REQ_SET_ADDRESS:
		isp1763_writew(le16_to_cpu(setup_request->wValue) |
					0x80, &dev->regs->address);
		set_status_flag(dev);
		break;
	case USB_REQ_SET_FEATURE:
	case USB_REQ_CLEAR_FEATURE:
		ep_feature(dev, setup_request);
		break;
	default:
		/*
		 * TODO: Check if setup() left any data for us to process,
		 * if not, ZiLP EP
		*/

		if (!dev->driver)
			return;

		ret = dev->driver->setup(dev->gadget, setup_request);
		if (ret < 0) {
			error_printk(KERN_ERR
					"###### ERROR : ->setup() returned with %i\n",
					ret);
			ep_stall(dev, 0, 1);
		}
		if (setup_request->bRequest & USB_DIR_IN) {
			if (queue_empty(&dev->ep0->queue) &&
					(setup_request->bRequest & 0x80)) {
				ep_set_index(dev, EP_INDEX(0, DIR_TX));
				set_dsen_flag(dev);
				write_ep(dev, NULL, 0);
				set_vendp_flag(dev);
			}
		} else if (setup_request->bRequest ==
				USB_REQ_SET_CONFIGURATION) {
			ep_set_index(dev, EP_INDEX(0, DIR_TX));
			write_ep(dev, NULL, 0);
			set_status_flag(dev);
		}
	}
	return;

	/* CLASS / VENDOR REQUEST */
non_standard:
	debug_printk(KERN_ERR
			"Processing EP0 class/vendor request\n");

	if (setup_request->bRequestType & USB_DIR_IN) {
		info_printk(KERN_ERR "CLASS IN REQUEST\n");
		dev->ep0->state = DIR_TX;
	} else {
		int timeout = 0;
		ep_set_index(&udc_dev, EP_INDEX(0, DIR_RX));
		clr_dsen_flag(&udc_dev);
		set_dsen_flag(&udc_dev);
#if 1
		while (!isp1763_readw(&dev->regs->buflen)) {
			udelay(10);
			if (++timeout > 1000) {
				printk(KERN_EMERG "TIMEOUT!\n");
				break;
			ep_set_index(dev, EP_INDEX(0, DIR_RX));

			}
		}
#endif
		pr_debug("CLASS OUT REQUEST\n");
		if (0) {
			int i;

			printk(KERN_ERR "DATA IN BUFFER FOR EP %i RIGHT NOW: ",
			       isp1763_readw(&dev->regs->ep_index));

			for (i = 0; i < 16; i++)
				printk("%.4x ",
				       isp1763_readw(&dev->regs->data_port));
			printk(KERN_ERR "\n");
		}

		pr_debug("dcBufferStatus: %.4x buflen: %i\n",
				isp1763_readw(&dev->regs->dcbufstatus),
				isp1763_readw(&dev->regs->buflen));
		dev->ep0->state = DIR_RX;
	}

	/* Probably a class request */
	ret = dev->driver->setup(dev->gadget, setup_request);
	if (ret < 0) {
		printk(KERN_ERR "dev->driver->setup() returned %i!"
			"stalling ep0...\n", ret);
		ep_stall(dev, 0, 1);
		return;
	}

	if (setup_request->bRequestType & USB_DIR_IN) {
		/* ??? */
	} else {
		ep_set_index(dev, EP_INDEX(0, DIR_RX));
		set_dsen_flag(dev);
		/* printk(KERN_ERR "%s:%i %s()\n",
		 * __FILE__,__LINE__,__func__);
		 */
		set_status_flag(dev);
	}
}

/* DEVICE ENABLE / DISABLE */

/**
* isp1763_udc_disable - disable controller activity
* @dev: UDC device pointer
*/
static void isp1763_udc_disable(struct isp1763_udc *dev)
{
	isp1763_writew(0, &dev->regs->address);
}

/**
* isp1763_udc_enable - enable controller activity
* @dev: UDC device pointer
*/
static void isp1763_udc_enable(struct isp1763_udc *dev)
{
	isp1763_writew(MASK_MODE_CLKAON | MASK_MODE_GLINTENA |
		       MASK_MODE_WKUPCS, &dev->regs->mode);
	isp1763_writel(0xFFFFFFFF, &dev->regs->dc_interrupt);
	isp1763_writel(irq_bits,
		       &dev->regs->dc_int_enable);
	isp1763_writew(0x80, &dev->regs->address);
}

/* ENDPOINT OPS */

static int ep_fifo_space_available = TOTAL_FIFO_SIZE - (3 * 64);

/**
* isp1763_udc_ep_enable - enable EP (EP API)
* @ep: USB endpoint to enable
* @desc: configuration descriptor
*/
static int isp1763_udc_ep_enable(struct usb_ep *ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct isp1763_udc *dev = &udc_dev;
	struct isp_ep *ispep = ep->driver_data;
	int ep_index = 0;
	int fifo_size = 0;
	unsigned long flags = 0;

	info_printk(KERN_ERR "%s:%i: %s(%s)\n", __FILE__, __LINE__,
		    __func__, ep->name);

	fifo_size = le16_to_cpu(desc->wMaxPacketSize) & 0x7FF;
	if (ep_fifo_space_available < fifo_size) {
		error_printk(KERN_ERR
			     "%s: no FIFO space available for %s, requested %i bytes, have %i\n",
			     __func__, ep->name, fifo_size,
			     ep_fifo_space_available);
		return -ENOMEM;
	}

	local_irq_save(flags);

	ep_index = get_ep_index(ep);
	if (ep_index < 0) {
		error_printk(KERN_ERR
			     "%s:%i: %s INVALID ENDPOINT INDEX %i\n",
			     __FILE__, __LINE__, __func__, ep_index);
		local_irq_restore(flags);
		return -EINVAL;
	}

	ispep = &isp_eps[ep_index];
	dev = ispep->udc;

	ep_set_index(dev, ispep->index);

	info_printk(KERN_ERR "%s:%i: %s(%s=type %i;maxsize=%i)\n",
		    __FILE__, __LINE__, __func__, ep->name,
		    desc->bmAttributes & 0x03,
		    le16_to_cpu(desc->wMaxPacketSize));

	isp1763_writew(desc->bmAttributes & 0x03, &dev->regs->ep_type);
	isp1763_writew(fifo_size, &dev->regs->ep_maxpktsize);

	isp1763_writew((desc->
			bmAttributes & 0x03) | EP_TYPE_ENABLE |
		       EP_TYPE_DBLBUF, &dev->regs->ep_type);

	/* clear buffers, twice for double buffering */
	set_clbuf_flag(dev);
	clr_clbuf_flag(dev);
	set_clbuf_flag(dev);
	clr_clbuf_flag(dev);

	ispep->maxpacketsize = fifo_size;
	ispep->ep_type = desc->bmAttributes & 0x03;

	ep_fifo_space_available -= fifo_size * 2;

	memcpy(&ispep->desc, desc, sizeof(struct usb_endpoint_descriptor));

	irq_bits |= 1 << (ep_index + 12);
	isp1763_writew(irq_bits, &dev->regs->dc_int_enable);
	/* printk(KERN_ERR "Enabling bit %.8lx in irq_bits\n",
	 *      1 << (ep_index + 10));
	 */
	local_irq_restore(flags);
	return 0;
}

/**
* isp1763_udc_ep_disable - disable EP (EP API)
* @ep: USB endpoint to disable
*/
static int isp1763_udc_ep_disable(struct usb_ep *ep)
{
	struct isp1763_udc *dev = &udc_dev;
	struct isp_ep *ispep;
	int ep_index = 0;
	unsigned long flags;

	info_printk(KERN_ERR "%s:%i: %s(%s)\n", __FILE__, __LINE__,
		    __func__, ep->name);

	local_irq_save(flags);

	ep_index = get_ep_index(ep);
	if (ep_index < 0) {
		error_printk(KERN_ERR
			     "%s:%i: %s INVALID ENDPOINT INDEX %i\n",
			     __FILE__, __LINE__, __func__, ep_index);
		local_irq_restore(flags);
		return -EINVAL;
	}


	irq_bits &= ~(1 << (ep_index + 12));
	isp1763_writew(irq_bits, &dev->regs->dc_int_enable);
	/* printk(KERN_ERR "Disabling bit %.8lx in irq_bits\n",
	 * 1 << (ep_index + 10));
	 */

	ispep = &isp_eps[ep_index];
	dev = ispep->udc;

	ep_fifo_space_available += ispep->maxpacketsize;

	ispep->maxpacketsize = 0;

	ep_set_index(dev, ispep->index);
	isp1763_writew(0, &dev->regs->ep_type);
	local_irq_restore(flags);

	return 0;
}

/**
* isp1763_udc_ep_alloc_request - allocate usb_request structure (EP API)
* @ep: USB endpoint
* @gfp_flags: unused
*/
static struct usb_request *isp1763_udc_ep_alloc_request(struct usb_ep *ep,
							gfp_t gfp_flags)
{
	return kzalloc(sizeof(struct usb_request), gfp_flags);
}

/**
* isp1763_udc_ep_free_request - free USB request (EP API)
* @ep: USB endpoint
* @req: request to free
*/
static void isp1763_udc_ep_free_request(struct usb_ep *ep,
					struct usb_request *req)
{
	kfree(req);
}

/**
* isp1763_udc_ep_queue - queue request on endpoint queue (EP API)
* @ep: USB endpoint
* @req: request to queue
* @gfp_flags: unused
*/
static int
isp1763_udc_ep_queue(struct usb_ep *ep, struct usb_request *req,
		     gfp_t gfp_flags)
{
	unsigned long flags;
	struct isp1763_ep *ispep = usb_ep_to_isp1763_ep(ep);
	struct isp1763_udc *dev = &udc_dev;

	local_irq_save(flags);

	disable_glint(dev);

	pr_debug("%s(%s) %i bytes %p\n", __func__, ep->name, req->length, req);

	if (req->buf == NULL && req->length > 0) {
		error_printk(KERN_ERR "%s: INVALID REQUEST\n",
			     __func__);
		req->status = -EINVAL;
		local_irq_restore(flags);
		enable_glint(dev);
		return req->status;
	}

	req->status = -EINPROGRESS;
	req->actual = 0;

	if (ep == udc_dev.gadget->ep0) {
		queue_add(&udc_dev.ep0->queue, req);

		if (udc_dev.ep0->state == DIR_TX) {
			pr_debug("QUEUE EP0 TX PACKET\n");

			ep_set_index(&udc_dev, EP_INDEX(0, DIR_TX));
			set_dsen_flag(&udc_dev);

			req->actual =
				write_ep(&udc_dev,
					(unsigned short *) req->buf,
					req->length <= 64 ? req->length : 64);
			if (req->actual < 64)
				set_vendp_flag(&udc_dev);
		} else {
			int i;
			int ret = 0;

			mdelay(1);
			memset((char *) req->buf, 0, req->length);
			#if 0
			ep_set_index(&udc_dev, EP_INDEX(0, DIR_RX));
			set_dsen_flag(&udc_dev);
			#endif

more:
			for (i = 0; i < 150; i++) {
				ep_set_index(&udc_dev, EP_INDEX(0, DIR_RX));
				if (isp1763_readw(&dev->regs->buflen))
					break;
			}
			if (!isp1763_readw(&dev->regs->buflen)) {
				printk(KERN_ERR
				       "%s ERROR: buflen == 0, bufstatus=%.4x\n",
				       __func__,
				       isp1763_readw(&dev->regs->dcbufstatus));
				local_irq_restore(flags);
				enable_glint(&udc_dev);
				return -EAGAIN;
			}

			ret = read_ep0(udc_dev.ep0, req);
			if (ret == 1) {
				if ((!req->no_interrupt) && (req->complete)) {
					debug_printk(KERN_ERR "Completing OUT "
						"request %p (%i bytes) "
						"p=%p ep=%p\n",
						req, req->actual,
						req->complete, ep);
						req->complete(ep, req);
				}
			} else if (ret == 0)
				goto more;

		}
	}
	/* normal data transfer */
	else {
		struct isp_ep *ispep;
		int ep_index = get_ep_index(ep);

		if (ep_index < 0) {
			error_printk(KERN_ERR
				     "ERROR: INVALID EP POINTER %p\n", ep);
			local_irq_restore(flags);
			enable_glint(dev);
			return -EINVAL;
		}

		ispep = &isp_eps[ep_index];
		queue_add(&ispep->queue, req);

		/* IN transfer */
		if (GET_EP_DIR(ep_index) == DIR_TX)
			/* && ispep->ep_type != EP_TYPE_INTERRUPT */ {
			if (isp1763_readw(&dev->regs->buflen) == 0
				    && queue_count(&ispep->queue) == 1) {
				ep_set_index(&udc_dev, ispep->index);
				req->actual +=
				    write_ep(&udc_dev,
					     (unsigned short *) req->buf,
					     req->length <=
					     ispep->maxpacketsize ? req->length
					     : ispep->maxpacketsize);
				if (req->actual == req->length)
					set_vendp_flag(&udc_dev);
			}
		}
	}

	local_irq_restore(flags);
	enable_glint(dev);
	return 0;
}

/**
* isp1763_udc_ep_dequeue - dequeue endpoint request
* @ep: endpoint
* @req: request
*
* NOTE: currently unimplemented
*/
static int
isp1763_udc_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	pr_err("%s:%i: %s(%s)\n", __FILE__, __LINE__, __func__, ep->name);
	return 0;
}

static int isp1763_udc_ep_set_halt(struct usb_ep *ep, int value)
{
	pr_err("%s:%i: %s(%s)\n", __FILE__, __LINE__, __func__, ep->name);
	return 0;
}

static int isp1763_udc_ep_set_wedge(struct usb_ep *ep)
{
	pr_err("%s:%i: %s(%s)\n", __FILE__, __LINE__, __func__, ep->name);
	return 0;
}

static int isp1763_udc_ep_fifo_status(struct usb_ep *ep)
{
	pr_err("%s:%i: %s(%s)\n", __FILE__, __LINE__, __func__, ep->name);
	return 0;
}

static void isp1763_udc_ep_fifo_flush(struct usb_ep *ep)
{
	pr_err("%s:%i: %s(%s)\n", __FILE__, __LINE__, __func__, ep->name);
}

static struct usb_ep_ops isp1763_ep_ops = {
	.enable = isp1763_udc_ep_enable,
	.disable = isp1763_udc_ep_disable,
	.alloc_request = isp1763_udc_ep_alloc_request,
	.free_request = isp1763_udc_ep_free_request,
	.queue = isp1763_udc_ep_queue,
	.dequeue = isp1763_udc_ep_dequeue,
	.set_halt = isp1763_udc_ep_set_halt,
	.set_wedge = isp1763_udc_ep_set_wedge,
	.fifo_status = isp1763_udc_ep_fifo_status,
	.fifo_flush = isp1763_udc_ep_fifo_flush,
};

/* GADGET OPS */

/**
* isp_get_frame - return frame number
* @gadget: gadget pointer
*/
static int isp_get_frame(struct usb_gadget *gadget)
{
	/* FIXE: replaced with get_gadget_data
	 * struct isp1763_dev *dev = gadget->dev.driver_data;
	 */
	struct isp1763_dev *dev = get_gadget_data(gadget);
	unsigned long flags;
	int ret = 0;

	local_irq_save(flags);
	ret = isp1763_readw(&dev->regs->frameno);
	local_irq_restore(flags);

	return ret;
}

/**
* isp_pullup - enable or disable pullup
* @gadget: gadget pointer
* @is_on: on/off
*
* Enable or disable pullup on D+
* I'm not actually sure it's a good idea to implement this, nor do
* I know a gadget driver that uses it...
*/
static int isp_pullup(struct usb_gadget *gadget, int is_on)
{
	/* FIXME: replaced with get_gadget_data
	 * struct isp1763_dev *dev = gadget->dev.driver_data;
	 */
	struct isp1763_dev *dev = get_gadget_data(gadget);
	unsigned long flags;

	local_irq_save(flags);
	if (is_on)
		isp1763_writel(MASK_OTGCTRL_DP_PULLUP,
			       &dev->regs->otg_ctrl);
	else
		isp1763_writel(MASK_OTGCTRL_DP_PULLUP << 16,
			       &dev->regs->otg_ctrl);
	local_irq_restore(flags);

	return 0;
}

static int isp_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	return -ENOTSUPP;
}

static int isp_vbus_session(struct usb_gadget *gadget, int is_active)
{
	return -ENOTSUPP;
}

static int isp_wakeup(struct usb_gadget *gadget)
{
	return -ENOTSUPP;
}

static struct usb_gadget_ops isp1763_gadget_ops = {
	.get_frame = isp_get_frame,
	.wakeup = isp_wakeup,
	.vbus_session = isp_vbus_session,
	.vbus_draw = isp_vbus_draw,
	.pullup = isp_pullup,

};

/* GADGET DEVICE FUNCTIONS */

/**
* usb_gadget_register_driver - register gadget driver
* @driver: gadget driver to register
*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	if (!driver ||
	    !driver->bind ||
	    !driver->unbind ||
	    !driver->setup /* || driver->speed != USB_SPEED_HIGH */ ) {
		return -EINVAL;
	}

	pr_info("%s: registering gadget function '%s'\n", __func__, driver->function);

	udc_dev.driver = driver;

	udc_dev.driver->bind(udc_dev.gadget);

	if (udc_dev.ctrl.active) {
		pr_info("active\n");
		configure_ep0(&udc_dev);
		isp1763_udc_enable(&udc_dev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(usb_gadget_register_driver);

/**
* usb_gadget_unregister_driver - unregister gadget driver
* @driver: gadget driver to unregister
*/
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	if (udc_dev.ctrl.active)
		isp1763_udc_disable(&udc_dev);

	/* FIXME: Need to do some more here? */
	udc_dev.driver->unbind(udc_dev.gadget);

	udc_dev.driver = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(usb_gadget_unregister_driver);

static struct usb_ep ep0 = {
	.name = "ep0",
	.ops = &isp1763_ep_ops,
	.maxpacket = 64,
};

static struct usb_gadget isp1763_gadget = {
	.ops = &isp1763_gadget_ops,
	.ep0 = &ep0,
	.name = "isp1763_udc",
	.speed = USB_SPEED_HIGH,
	.is_otg = 1,
	.b_hnp_enable = 0,
	.a_hnp_support = 0,
	.a_alt_hnp_support = 0,
};

#if 0
static struct isp1763_udc controller = {
	.gadget = {
		.ops	= &isp1763_ops,
		.ep0	= &controller.ep[0].ep,
		.name	= "isp1763_udc",
		.speed	= USB_SPEED_HIGH,
		.is_otg	= 1,
		.b_hnp_enable	= 0,
		.a_hnp_support	= 0,
		.a_alt_hnp_support = 0,
	},
	.ep[0] = {
		.ep = {
			.name		= ep0name,
			.ops		= &isp1763_ep_ops,
			.maxpacket	= 64,
		},
		.udc	= &controller,
	},
};
#endif

/**
* setup_ep_struct - setup endpoint structures
* @dev: UDC device
*/
static int setup_ep_struct(struct isp1763_udc *dev)
{
	int i = 0;

	dev->ep0 = kzalloc(sizeof(struct isp_ep), GFP_ATOMIC);

	INIT_LIST_HEAD(&dev->gadget->ep_list);
	dev->ep0->queue = NULL;
	dev->ep0->udc = &udc_dev;
	dev->ep0->gadget = dev->gadget;
	dev->ep0->state = 0;
	dev->ep0->ep_type = EP_TYPE_UNUSED;
	snprintf(dev->ep0->name, 8, "ep0");
	dev->ep0->index = 0;

	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		isp_eps[i].queue = NULL;
		isp_eps[i].ep.driver_data = NULL;
		isp_eps[i].ep.ops = &isp1763_ep_ops;
		isp_eps[i].ep.ep_list = dev->gadget->ep_list;
		isp_eps[i].ep.maxpacket = 4096;
		isp_eps[i].udc = dev;
		isp_eps[i].gadget = dev->gadget;
		isp_eps[i].state = 0;
		isp_eps[i].ep_type = EP_TYPE_UNUSED;
		snprintf(isp_eps[i].name, 8, "ep%i%s",
			 GET_EP_NUM_CANONICAL(i) + 1,
			 GET_EP_DIR(i) == DIR_TX ? "in" : "out");
		isp_eps[i].ep.name = kstrdup(isp_eps[i].name, GFP_KERNEL);
		isp_eps[i].index = i + 2;
		list_add_tail(&isp_eps[i].ep.ep_list,
			      &dev->gadget->ep_list);

	}
}

static void destroy_ep_struct(struct isp1763_udc *dev)
{
	int i = 0;

	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		queue_flush(&isp_eps[i].queue);
		kfree(isp_eps[i].ep.name);
	}

	queue_flush(&dev->ep0->queue);
	kfree(dev->ep0);
}

/* IF FUNCTIONS */
/**
* handle_ep_irq - handle endpoint interrupts
* @dev: UDC device
* @bit: interrupt bit
*/
static void handle_ep_irq(struct isp1763_udc *dev, int bit)
{
	struct usb_request *req;
	int ep_index = bit + 2;
	struct isp_ep *ispep = NULL;

	ispep = &dev->eps[bit];

	pr_debug("Interrupt for EP %s (Bit %i, epindex %i)\n",
			ispep->name, bit, ep_index);

	if (queue_empty(&ispep->queue)) {
		queue_printk(KERN_ERR "%s/%s queue empty!\n",
				ispep->name,
				ispep->ep.name);
		goto done;
	}

	ep_set_index(dev, ep_index);
	req = queue_get(&ispep->queue);

	if (GET_EP_DIR(ep_index) == DIR_RX) {
		if (read_ep(ispep, req) > 0)
			queue_topkill(&ispep->queue);
	} else {
		int already_sent_data = 0;
		/*
			* check if there's more data to write for
			* previous packet, otherwise send completion
		*/
		if (req->actual == req->length) {
			if (req->actual > 0
				&& req->actual % 512 == 0) {
				/*
				* insert ZLP to terminate transaction where
				* length is a multiple of 512
				*/
				write_ep(&udc_dev, NULL, 0);
				set_vendp_flag(&udc_dev);
			}
			req->status = 0;

			if (req->complete) {
				debug_printk
					(KERN_ERR
					"Completing IN request %p (%i bytes)\n",
					req, req->actual);
				req->complete(&ispep->ep, req);
			}
			queue_topkill(&ispep->queue);
		} else {
			int written;

			already_sent_data = 1;
			written = write_ep(&udc_dev,
					(unsigned short	*) ((char *)
						req->buf + req->actual),
					(req->length - req->actual) <=
						ispep->maxpacketsize
						? (req->length - req->actual) :
						ispep->maxpacketsize);
			if (written < ispep->maxpacketsize)
				set_vendp_flag(&udc_dev);

			req->actual += written;
		}

		/*
		 * if no data was submitted from previous paket, get next from
		 * list and submit that
		 */
		if (!already_sent_data) {
			if (!queue_empty
				(&ispep->queue)) {

				req = queue_get(&ispep->queue);
				req->actual = write_ep(&udc_dev,
					(unsigned short *)req->buf,
					req->length <= ispep->maxpacketsize
					? req->length :	ispep->maxpacketsize);
				if (req->actual < ispep->maxpacketsize)
					set_vendp_flag(&udc_dev);
			}
		}
	}
done:
	return;
}

/**
* isp1763_udc_do_irq - UDC interrupt handler
* @ctrl: controller pointer
*/
static int isp1763_udc_do_irq(struct isp1763_controller *ctrl, u32 flags)
{
	struct isp1763_udc *dev = ctrl->priv;
	u32 interrupts = 0;
	int bit = 0;
	int i;

	isp1763_writew(UNLOCK_CODE, &dev->regs->unlock);
	disable_glint(dev);

	isp1763_writel(irq_bits,
		       &dev->regs->dc_int_enable);
	interrupts = flags & irq_bits;

	isp1763_otg_timer_cancel();
	isp1763_otg_timer_start((otgtimer_callback_t) isp1763_udc_do_irq,
				100, ctrl);
	for (i = 1; i < USB_MAX_ENDPOINTS; i += 2) {
		struct isp_ep *ispep = &udc_dev.eps[i];
		if (queue_count(&ispep->queue) > 0) {
			ep_set_index(dev, ispep->index);
			if (isp1763_readw(&dev->regs->dcbufstatus)
				== 0) {
				interrupts |= 1 << (i + 12);
			}
		}
	}

	if (!interrupts)
		return 0;

#if 0
	printk(KERN_ERR "dev->regs->dc_int_enable: %.8lx\n",
		isp1763_readl(&dev->regs->dc_int_enable));
	printk(KERN_ERR "%s() flags = %.8lx irqbits = %.8lx\n",
		__func__, flags, irq_bits);
#endif
	/* VBUS event */
	if (interrupts & MASK_DCINT_VBUS) {
		u16 mode = isp1763_readw(&dev->regs->mode);

		/* Double negation (!!) used to make bitmask into boolean */
		pr_debug("VBUS interrupt (Vbus = %i)\n",
			     !!(mode & MASK_MODE_VBUSSTAT));
		if (mode & MASK_MODE_VBUSSTAT) {
			isp1763_writel(MASK_OTGCTRL_DP_PULLUP |
				       MASK_OTGCTRL_SW_SEL_HC_DC,
				       &dev->regs->otg_ctrl);
		} else
			isp1763_writel(MASK_OTGCTRL_SW_SEL_HC_DC,
				       &dev->regs->otg_ctrl);
	}

	/* Control endpoint setup event */
	if (interrupts & MASK_DCINT_EP0SETUP) {
		pr_debug("EP0 SETUP event\n");
		handle_ep0_setup(dev);
	}

	/* EP0 TX */
	if (interrupts & MASK_DCINT_EP0TX) {
		struct usb_request *req;
		int written = 0;

		pr_debug("EP0 TX\n");

		if (dev->ep0->state == DIR_RX)
			goto done;

		if (queue_empty(&dev->ep0->queue)) {
			pr_debug("EP0TX queue empty!\n");
			goto done;
		}

		req = queue_get(&dev->ep0->queue);

		queue_printk(KERN_ERR "EP0 request len=%i, written=%i\n",
			     req->length, req->actual);
		ep_set_index(dev, EP_INDEX(0, DIR_TX));

		if (req->actual < req->length) {
			queue_printk(KERN_ERR
				     "EP0 request need to write more data\n");
			written += write_ep(dev,
				     (unsigned short *) ((char *) req->buf +
				     req->actual),
				     (req->length - req->actual) <=
				     64 ? (req->length - req->actual) : 64);
			if (written < 64)
				set_vendp_flag(dev);

			req->actual += written;
		}
		/* complete? */
		if (req->actual == req->length) {
			pr_debug("EP0 TX request complete\n");
			ep_set_index(dev, EP_INDEX(0, DIR_RX));
			/* printk(KERN_ERR "%s:%i %s()\n",
			 * __FILE__,__LINE__,__func__);
			 */
			set_status_flag(dev);
			req->status = 0;
			if (!req->no_interrupt)
				if (req->complete)
					req->complete(dev->gadget->ep0,
						      req);
			queue_topkill(&dev->ep0->queue);
		}

		pr_debug("EP0 TX event\n");
	}

	/* EP0 RX */
	if (interrupts & MASK_DCINT_EP0RX) {
		struct usb_request *req;

		ep_set_index(dev, EP_INDEX(0, DIR_RX));
		if (isp1763_readw(&dev->regs->buflen) == 0) {
			set_status_flag(dev);
			goto done;
		}

		if (queue_empty(&dev->ep0->queue)) {
			pr_debug("EP0RX queue empty!\n");
			goto done;
		}

		req = queue_get(&dev->ep0->queue);

		queue_printk(KERN_ERR "EP0 request len=%i, written=%i\n",
			     req->length, req->actual);

		read_ep(dev->ep0, req);
		set_status_flag(dev);
		queue_topkill(&dev->ep0->queue);

		debug_printk(KERN_ERR "EP0 RX event\n");
	}

	/* BUS reset */
	if (interrupts & MASK_DCINT_BRESET) {
		int i;
#if 1
		pr_debug("BUS reset detected\n");
		dev->gadget->speed = USB_SPEED_FULL;
		for (i = 0; i < 14; i++)
			isp1763_udc_ep_disable(&dev->eps[i].ep);
#endif
	}

	/* Highspeed status change */
	if (interrupts & MASK_DCINT_HS_STAT) {
		pr_debug("Speed change detected\n");
		dev->gadget->speed = USB_SPEED_HIGH;
	}

	/* EP interrupts */
	if (interrupts & MASK_DCINT_EP_EVENT) {
		for (bit = 0; bit < 14; bit++) {
			if ((interrupts & (1 << (12 + bit))) == 0)
				continue;
			handle_ep_irq(dev, bit);
		}
	}

done:
	enable_glint(dev);
	return 0;
}

/**
* isp1763_udc_suspend - suspend gadget device
* @ctrl: controller pointer
*
* Suspend device controller, usually in order to then activate the
* host controller
*/
static int isp1763_udc_suspend(struct isp1763_controller *ctrl)
{
	struct isp1763_udc *dev = ctrl->priv;

	pr_debug("udc suspend called\n");

	ctrl->active = 0;
	isp1763_udc_disable(dev);

	return 0;
}

/**
* isp1763_udc_resume - resume gadget device
* @ctrl: controller pointer
*
* Resume/Init device controller
*/
static int isp1763_udc_resume(struct isp1763_controller *ctrl)
{
	struct isp1763_udc *dev = ctrl->priv;

	pr_debug("udc resume called\n");

	ctrl->active = 1;

	configure_ep0(dev);
	isp1763_udc_enable(dev);

	return 0;
}

static void isp1763_dc_release(struct device *dev)
{
	printk(KERN_EMERG "%s:%i %s()\n", __FILE__, __LINE__,
	       __func__);
}

/**
* isp1763_udc_probe - device controller 'probe' function
* @ctrl: controller pointer
*
* Initialize driver, but don't start controller yet
*/
static int isp1763_udc_probe(struct isp1763_controller *ctrl)
{
	struct isp1763_udc *dev = ctrl->priv;
	int i;

	dev->regs = ctrl->regs;
	if (!dev->regs) {
		error_printk(KERN_ERR "%s: error retrieving memory mapping "
				"from base driver!\n", __FILE__);
		return -ENODEV;
	}
	ep0.driver_data = dev;

	dev->gadget = &isp1763_gadget;
	/* FIXME: replaced with set_gadget_data
	 * dev->gadget->dev.driver_data = (void *) &udc_dev;
	 */
#if 0
	set_gadget_data(&controller->gadget, &controller);
#endif
	set_gadget_data(dev->gadget, &udc_dev);
	setup_ep_struct(dev);

	dev_set_name(&udc_dev.gadget->dev, "gadget");
	udc_dev.gadget->dev.release = isp1763_dc_release;
	device_register(&udc_dev.gadget->dev);

	udc_dev.eps = isp_eps;

	for (i = 0; i < USB_MAX_ENDPOINTS; i++) {
		pr_debug("EP %i (%s/%s)\n", i,
			    udc_dev.eps[i].name, udc_dev.eps[i].ep.name);
	}
	return 1;
}

/*-------------------------------------------------------------------------*/

static int __init isp1763_udc_init(void)
{
	int ret;

	udc_dev.ctrl.active = 0;
	udc_dev.ctrl.probe = isp1763_udc_probe;
	udc_dev.ctrl.do_irq = isp1763_udc_do_irq;
	udc_dev.ctrl.suspend = isp1763_udc_suspend;
	udc_dev.ctrl.resume = isp1763_udc_resume;
	udc_dev.ctrl.priv = &udc_dev;
	ret = isp1763_register_ctrl(&udc_dev.ctrl, ROLE_DEVICE);
	if (ret)
		pr_err("Failed to register device controller (%d)\n", ret);

	return ret;
}

static void __exit isp1763_udc_exit(void)
{
	isp1763_unregister_ctrl(ROLE_DEVICE);
	destroy_ep_struct(&udc_dev);
}

module_init(isp1763_udc_init);
module_exit(isp1763_udc_exit);

MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("F. Voegel, Carangul.Tech");
MODULE_LICENSE("GPL");
