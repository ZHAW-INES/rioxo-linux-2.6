/*
 * isp1763_udc -- driver for ST Ericsson ISP1763 USB peripheral controller
 *
 * Copyright (C) 2011 Tobias Klauser <tklauser@distanz.ch>
 * Copyright (C) 2010 F. Voegel, Carangul.Tech
 * Copyright (C) 2010 I+ME ACTIA Informatik und Mikroelektronik GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

//#define DEBUG

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "isp1763_udc.h"

#define DRIVER_NAME	"isp1763_udc"
#define DRIVER_DESC	"ISP1763 USB Peripheral Controller Driver"
#define DRIVER_AUTHOR	"Tobias Klauser"
#define DRIVER_VERSION	"1.0"

static const char driver_name[] = DRIVER_NAME;
static const char ep0name[] = "ep0";

static struct isp1763_udc *controller = NULL;

static void isp1763_udc_complete_req(struct isp1763_ep *ep,
				     struct isp1763_request *req, int status)
{
	/* remove request from ep queue */
	dev_dbg(ep->udc->dev, "completing request on EP%d%s\n", ep->num, isp1763_ep_is_tx(ep) ? "TX" : "RX");
	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;

	spin_unlock(&ep->udc->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->udc->lock);
}

static void isp1763_udc_enable_glint(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_MODE);
	isp1763_writew(udc, tmp | ISP1763_MODE_GLINTENA, ISP1763_REG_MODE);
}

static void isp1763_udc_disable_glint(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_MODE);
	isp1763_writew(udc, tmp & ~ISP1763_MODE_GLINTENA, ISP1763_REG_MODE);
}

static void isp1763_udc_set_clbuf(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp | ISP1763_CTRL_FUNC_CLBUF, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_clear_clbuf(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp & ~ISP1763_CTRL_FUNC_CLBUF, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_set_vendp(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp | ISP1763_CTRL_FUNC_VENDP, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_set_dsen(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp | ISP1763_CTRL_FUNC_DSEN, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_clear_dsen(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp & ~ISP1763_CTRL_FUNC_DSEN, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_set_status(struct isp1763_udc *udc)
{
	u16 tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	isp1763_writew(udc, tmp | ISP1763_CTRL_FUNC_STATUS, ISP1763_REG_CTRL_FUNC);
}

static void isp1763_udc_set_ep_index(struct isp1763_udc *udc, u8 index)
{
	isp1763_writew(udc, index, ISP1763_REG_EP_INDEX);
	isp1763_writew(udc, ~index, ISP1763_REG_EP_INDEX);
	udelay(1);
}

static void isp1763_udc_connect(struct isp1763_udc *udc)
{
	u32 tmp;

	/* Enable device */
	isp1763_writew(udc, ISP1763_ADDR_DEVEN, ISP1763_REG_ADDR);

	tmp = ISP1763_OTG_CTRL_DP_PULLUP | ISP1763_OTG_CTRL_SW_SEL_HC_DC;
	isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_SET);

	tmp = ISP1763_OTG_CTRL_DP_PULLDOWN | ISP1763_OTG_CTRL_DM_PULLDOWN
		| ISP1763_OTG_CTRL_VBUS_DRV;
	isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_CLEAR);

	/* Connect to the bus */
	tmp = ISP1763_MODE_WKUPCS | ISP1763_MODE_CLKAON | ISP1763_MODE_GLINTENA;
	isp1763_writew(udc, tmp, ISP1763_REG_MODE);

	/* 1us pulse width */
	isp1763_writew(udc, 0x1E, ISP1763_REG_INT_PULSE_WIDTH);

	/* Set interrupt configuration register: interrupt on all ACK... */
	tmp = ISP1763_INT_CONF_DDBGMODOUT_ALL_ACK
		| ISP1763_INT_CONF_DDBGMODIN_ALL_ACK
		| ISP1763_INT_CONF_CDBGMOD_ALL_ACK;
	isp1763_writew(udc, tmp, ISP1763_REG_INT_CONF);

	/* Enable default interrupts */
	isp1763_writel(udc, ISP1763_DC_INT_EN_DEFAULT, ISP1763_REG_DC_INT_EN);

	tmp = isp1763_readw(udc, ISP1763_REG_HW_MODE_CTRL);
	tmp |= ISP1763_HW_MODE_CTRL_COMN_INT
		| ISP1763_HW_MODE_CTRL_GLOBAL_INTR_EN
		| ISP1763_HW_MODE_CTRL_ID_PULLUP;	/* disable sampling of ID line */
	isp1763_writew(udc, tmp, ISP1763_REG_HW_MODE_CTRL);
}

static void __ep_disable(struct isp1763_ep *ep)
{
	struct isp1763_udc *udc = ep->udc;
	u32 tmp;

	/* disable interrupt for this endpoint (XXX: shall we do this here?) */
	tmp = isp1763_readl(udc, ISP1763_REG_DC_INT_EN);
	isp1763_writel(udc, tmp & ~(1 << (ep->num + ep->dir + 10)), ISP1763_REG_DC_INT_EN);

	udc->ep_fifo_space += ep->ep.maxpacket;
	ep->ep.maxpacket = 0;

	isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));
	isp1763_writew(udc, 0, ISP1763_REG_EP_TYPE);
}

static void isp1763_udc_stall_ep(struct isp1763_ep *ep, int do_stall)
{
	struct isp1763_udc *udc = ep->udc;
	u16 tmp;

	isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));

	tmp = isp1763_readw(udc, ISP1763_REG_CTRL_FUNC);
	if (do_stall)
		tmp |= ISP1763_CTRL_FUNC_STALL;
	else
		tmp &= ~ISP1763_CTRL_FUNC_STALL;

	isp1763_writew(udc, tmp, ISP1763_REG_CTRL_FUNC);
}

static inline void isp1763_udc_zlp(struct isp1763_udc *udc)
{
	isp1763_writew(udc, 0, ISP1763_REG_BUF_LEN);
	isp1763_udc_set_vendp(udc);
}

static void isp1763_udc_write_ep(struct isp1763_ep *ep,
				 struct isp1763_request *req, bool complete)
{
	struct isp1763_udc *udc = ep->udc;
	unsigned int to_write, i;
	u16 *buf;

	if (req->req.actual == req->req.length) {
		if (complete)
			isp1763_udc_complete_req(ep, req, 0);
		return;
	}

	to_write = req->req.length - req->req.actual;
	if (to_write > ep->ep.maxpacket)
		to_write = ep->ep.maxpacket;

	isp1763_writew(udc, to_write, ISP1763_REG_BUF_LEN);

	buf = (u16 *)(((u8 *) req->req.buf) + req->req.actual);
	for (i = 0; i < (to_write / 2); i++)
		isp1763_writew(udc, buf[i], ISP1763_REG_DATA_PORT);

	if (to_write & 1) {
		u8 *bufc = (u8 *) buf;
		u16 val = (u16)(bufc[to_write - 1] | (bufc[to_write - 1] << 8));
		isp1763_writew(udc, val, ISP1763_REG_DATA_PORT);
	}

	req->req.actual += to_write;
	if (complete && req->req.actual == req->req.length)
		isp1763_udc_complete_req(ep, req, 0);

	if (to_write > 0 && to_write < ep->ep.maxpacket)
		isp1763_udc_set_vendp(udc);
}

static void isp1763_udc_read_ep(struct isp1763_ep *ep,
				 struct isp1763_request *req, bool complete)
{
	struct isp1763_udc *udc = ep->udc;
	unsigned int bufspace, buflen, i;
	u16 *buf;
	bool is_done = false;

	isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));

	/*
	 * An RX interrupt without any data is caused by a icr setting where
	 * an interrupt is generated on the first NAK after previously having an
	 * ACK. This is it. Nothing to it, so just leave and do naught.
	 */
	if (isp1763_readw(udc, ISP1763_REG_DC_BUF_STATUS) == 0) {
		isp1763_udc_set_clbuf(udc);
		return;
	}

	if (!req->req.buf) {
		dev_err(udc->dev, "%s: request without a buffer\n", ep->name);
		req->req.actual = 0;
		if (!req->req.no_interrupt)
			isp1763_udc_complete_req(ep, req, -EINVAL);
		return;;
	}

	if (req->req.actual == req->req.length)
		return;

	buf = (u16 *)(((u8 *) req->req.buf) + req->req.actual);

	bufspace = req->req.length - req->req.actual;
	buflen = isp1763_readw(udc, ISP1763_REG_BUF_LEN);

	if (buflen > ep->ep.maxpacket)
		buflen = ep->ep.maxpacket;
	if (buflen > bufspace) {
		dev_dbg(udc->dev, "%s: buffer overflow\n", ep->name);
		req->req.status = -EOVERFLOW;
		buflen = bufspace;
	}

	ndelay(500);

	for (i = 0; i < (buflen / 2); i++)
		buf[i] = isp1763_readw(udc, ISP1763_REG_DATA_PORT);

	if (buflen & 1) {
		u8 *bufc = (u8 *) buf;
		bufc[buflen - 1] = (u8) isp1763_readw(udc, ISP1763_REG_DATA_PORT) & 0xFF;
	}

	req->req.actual += buflen;
	if (buflen < ep->ep.maxpacket)
		is_done = true;
	if (buflen == bufspace)
		is_done = true;

	if (complete && is_done) {
		if (ep == &udc->ep[0]) {
			isp1763_udc_set_status(udc);
		} else {
			isp1763_writew(udc, 0, ISP1763_REG_DC_BUF_STATUS);
			isp1763_udc_set_clbuf(udc);
		}
		isp1763_udc_complete_req(ep, req, 0);
	}
}

static int isp1763_udc_handle_ep_irq(struct isp1763_ep *ep)
{
	struct isp1763_udc *udc = ep->udc;
	struct isp1763_request *req;

	dev_dbg(udc->dev, "Interrupt for EP %s\n", ep->name);

	if (list_empty(&ep->queue)) {
		/* XXX: something else needed here? */
		dev_dbg(udc->dev, "%s: queue empty\n", ep->name);
		return 0;
	}

	isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));
	req = list_entry(ep->queue.next, struct isp1763_request, queue);

	if (isp1763_ep_is_tx(ep)) {
		bool complete = false;
		unsigned int i;

		dev_dbg(udc->dev, "write %s: actual = %u length = %u", ep->name,
			req->req.actual, req->req.length);

#ifdef DEBUG
		for (i = 0; i < req->req.length; i++)
			pr_cont(" %02x", ((unsigned char *)req->req.buf)[i]);
		pr_cont("\n");
#endif

		/*
		 * Check if there is more data to write for previous packet,
		 * otherwise send completion
		 */

		/* insert ZLP after 512 bytes */
		if (req->req.actual == req->req.length) {
			if (req->req.actual > 0 && req->req.actual % 512 == 0)
				isp1763_udc_zlp(udc);

			complete = true;
		}

		isp1763_udc_write_ep(ep, req, complete);
	} else {
		dev_dbg(udc->dev, "read %s: actual = %u length = %u\n", ep->name,
			req->req.actual, req->req.length);

		isp1763_udc_read_ep(ep, req, true);
	}

	return 0;
}

union isp1763_udc_setup_pkt {
	struct usb_ctrlrequest r;
	u16 raw[4];
};

static void isp1763_udc_handle_ep0setup(struct isp1763_udc *udc)
{
	unsigned int buflen, i;
	union isp1763_udc_setup_pkt pkt;
	u16 wValue, wIndex, wLength;
	int status;

	isp1763_udc_set_ep_index(udc, ISP1763_EP_INDEX_EP0SETUP);

	buflen = isp1763_readw(udc, ISP1763_REG_BUF_LEN);
	if (buflen != sizeof(pkt)) {
		dev_err(udc->dev, "Invalid size for USB control request: %d\n", buflen);
		return;
	}

	/*
	 * A 500 ns delay starrting from the reception of the endpoint interrupt
	 * may be required for the first read from the data port.
	 */
	ndelay(500);

	for (i = 0; i < (buflen / 2); i++)
		pkt.raw[i] = isp1763_readw(udc, ISP1763_REG_DATA_PORT);

	wValue = le16_to_cpu(pkt.r.wValue);
	wIndex = le16_to_cpu(pkt.r.wIndex);
	wLength = le16_to_cpu(pkt.r.wLength);

	dev_dbg(udc->dev, "SETUP %02x.%02x v%04x i%04x l%04x\n",
			pkt.r.bRequestType, pkt.r.bRequest,
			wValue, wIndex, wLength);

	if ((pkt.r.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		goto non_standard;

	udc->ep[0].dir = ISP1763_EP_INDEX_DIR_TX;

	switch (pkt.r.bRequest) {
	case USB_REQ_SET_ADDRESS:
		dev_dbg(udc->dev, "USB_REQ_SET_ADDRESS: %04x\n", wValue);
		isp1763_writew(udc, wValue | ISP1763_ADDR_DEVEN, ISP1763_REG_ADDR);
		isp1763_udc_set_status(udc);
		break;
	case USB_REQ_SET_FEATURE:
	case USB_REQ_CLEAR_FEATURE:
		if (wValue == USB_ENDPOINT_HALT) {
			struct isp1763_ep *ep = &udc->ep[WINDEX_TO_EP_INDEX(wIndex)];
			isp1763_udc_stall_ep(ep, pkt.r.bRequest == USB_REQ_SET_FEATURE ? 1 : 0);
		} else
			isp1763_udc_set_status(udc);

		break;
	default:
		if (udc->driver) {
			dev_dbg(udc->dev, "Setting up gadget driver\n");
			status = udc->driver->setup(&udc->gadget, &pkt.r);
		} else
			status = -ENODEV;
		if (status < 0) {
			dev_dbg(udc->dev, "req %02x.%02x protocol STALL; status %d\n",
				pkt.r.bRequestType, pkt.r.bRequest, status);
			isp1763_udc_stall_ep(&udc->ep[0], 1);
		}

		/* Device to host */
		if (pkt.r.bRequest & USB_DIR_IN) {
			if (list_empty(&udc->ep[0].queue)) {
				isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_TX));
				isp1763_udc_set_dsen(udc);
				isp1763_udc_zlp(udc);
			}
		} else if (pkt.r.bRequest == USB_REQ_SET_CONFIGURATION) {
			isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_TX));
			isp1763_writew(udc, 0, ISP1763_REG_BUF_LEN);
			isp1763_udc_set_status(udc);
		}
	}

	return;

non_standard:
	if (pkt.r.bRequestType & USB_DIR_IN) {
		dev_dbg(udc->dev, "EP0 CLASS IN REQUEST\n");
		udc->ep[0].dir = ISP1763_EP_INDEX_DIR_TX;
	} else {
		unsigned int timeout = 0;
		dev_dbg(udc->dev, "EP0 CLASS OUT REQUEST\n");

		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
		isp1763_udc_clear_dsen(udc);
		isp1763_udc_set_dsen(udc);

		while (!isp1763_readw(udc, ISP1763_REG_BUF_LEN)) {
			udelay(10);
			if (++timeout > 1000) {
				dev_emerg(udc->dev, "TIMEOUT!\n");
				break;
			}
			isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
		}

		dev_dbg(udc->dev, "DcBufferStatus: %04x buflen: %d\n",
				isp1763_readw(udc, ISP1763_REG_DC_BUF_STATUS),
				isp1763_readw(udc, ISP1763_REG_BUF_LEN));
		udc->ep[0].dir = ISP1763_EP_INDEX_DIR_RX;
	}

	if (udc->driver)
		status = udc->driver->setup(&udc->gadget, &pkt.r);
	else
		status = -ENODEV;
	if (status < 0) {
		dev_dbg(udc->dev, "req %02x.%02x protocol STALL; status %d\n",
			pkt.r.bRequestType, pkt.r.bRequest, status);
		isp1763_udc_stall_ep(&udc->ep[0], 1);
	}

	if (!(pkt.r.bRequestType & USB_DIR_IN)) {
		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
		isp1763_udc_set_dsen(udc);
		isp1763_udc_set_status(udc);
	}
}

static int isp1763_udc_ep_enable(struct usb_ep *_ep,
				 const struct usb_endpoint_descriptor *desc)
{
	struct isp1763_ep *ep = container_of(_ep, struct isp1763_ep, ep);
	struct isp1763_udc *udc = ep->udc;
	unsigned long flags;
	u16 fifo_size;
	u32 tmp;

	if (!desc)
		return -EINVAL;

	/* Check whether we've got enough space in the shared FIFO */
	fifo_size = le16_to_cpu(desc->wMaxPacketSize) & 0x7FF;
	if (fifo_size > udc->ep_fifo_space)
		return -ENOMEM;

	spin_lock_irqsave(&udc->lock, flags);

	dev_dbg(udc->dev, "ep_enable(%i): %s type %x maxsize %d\n", ep->num, ep->name,
		usb_endpoint_type(desc), le16_to_cpu(desc->wMaxPacketSize));

	dev_dbg(udc->dev, "Set EP index: %04x\n", EP_INDEX(ep->num, ep->dir));
	isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));

	isp1763_writew(udc, usb_endpoint_type(desc), ISP1763_REG_EP_TYPE);
	isp1763_writew(udc, fifo_size, ISP1763_REG_EP_MAXPKTSIZE);

	tmp = usb_endpoint_type(desc) | ISP1763_EP_TYPE_ENABLE | ISP1763_EP_TYPE_DBLBUF;
	isp1763_writew(udc, tmp, ISP1763_REG_EP_TYPE);

	/* clear buffer, twice for double buffering */
	isp1763_udc_set_clbuf(udc);
	isp1763_udc_clear_clbuf(udc);
	isp1763_udc_set_clbuf(udc);
	isp1763_udc_clear_clbuf(udc);

	ep->ep.maxpacket = fifo_size;
	udc->ep_fifo_space -= fifo_size;

	/* enable interrupt for this endpoint (XXX: shall we do this here?) */
	tmp = isp1763_readl(udc, ISP1763_REG_DC_INT_EN);
#if 0
	dev_dbg(udc->dev, "DcIntEnable: %08x\n", tmp);
	isp1763_writel(udc, tmp | (1 << (ep->num + ep->dir + 10)), ISP1763_REG_DC_INT_EN);
	tmp = isp1763_readl(udc, ISP1763_REG_DC_INT_EN);
	dev_dbg(udc->dev, "DcIntEnable: %08x\n", tmp);
#endif

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int isp1763_udc_ep_disable(struct usb_ep *_ep)
{
	struct isp1763_ep *ep = container_of(_ep, struct isp1763_ep, ep);
	struct isp1763_udc *udc = ep->udc;
	unsigned long flags;

	if (ep == &udc->ep[0]) {
		dev_err(udc->dev, "Cannot disable EP0\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);
	__ep_disable(ep);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static struct usb_request *isp1763_udc_ep_alloc_request(struct usb_ep *ep,
							gfp_t gfp_flags)
{
	struct isp1763_request *req;

	req = kzalloc(sizeof(struct isp1763_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void isp1763_udc_ep_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct isp1763_request *req;

	req = container_of(_req, struct isp1763_request, req);
	BUG_ON(!list_empty(&req->queue));
	kfree(req);
}

static int isp1763_udc_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
				gfp_t gfp_flags)
{
	struct isp1763_ep *ep;
	struct isp1763_udc *udc;
	struct isp1763_request *req;
	unsigned long flags;
	int ret;

	if (!_ep || !_req || !_req->complete || !_req->buf)
		return -EINVAL;

	ep = container_of(_ep, struct isp1763_ep, ep);
	req = container_of(_req, struct isp1763_request, req);
	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -EINVAL;

	dev_dbg(udc->dev, "ep_queue(%s): %d bytes:", ep->name, _req->length);

#ifdef DEBUG
	if (ep != &udc->ep[0]) {
		unsigned int i;
		for (i = 0; i < _req->length; i++)
			pr_cont(" %02x", ((unsigned char *) (_req->buf))[i]);
	}
	pr_cont("\n");
#endif

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&udc->lock, flags);

	list_add_tail(&req->queue, &ep->queue);

	/* Control transfer */
	if (ep == &udc->ep[0]) {
		dev_dbg(udc->dev, "queueing on EP0%s\n", isp1763_ep_is_tx(ep) ? "TX" : "RX");
		if (isp1763_ep_is_tx(ep)) {
			isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_TX));
			isp1763_udc_set_dsen(udc);

			isp1763_udc_write_ep(ep, req, false);
			if (req->req.actual <= 64)
				isp1763_udc_set_vendp(udc);
		} else {
			unsigned int i;

			dev_dbg(udc->dev, "EP0RX transfer\n");

			mdelay(1);
			memset(_req->buf, 0, _req->length);

more:
			for (i = 0; i < 150; i++) {
				isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
				if (isp1763_readw(udc, ISP1763_REG_BUF_LEN))
					break;
			}

			if (isp1763_readw(udc, ISP1763_REG_BUF_LEN) == 0) {
				dev_err(udc->dev, "EP0RX: No data present\n");
				ret = -EAGAIN;
				goto out;
			}

			isp1763_udc_read_ep(ep, req, true);
			if (req->req.status != 0)
				goto more;
		}

	} else {	/* normal data transfer */
		dev_dbg(udc->dev, "Normal data transfer\n");

		/* IN transfer */
		if (isp1763_ep_is_tx(ep)) {
			if (isp1763_readw(udc, ISP1763_REG_BUF_LEN == 0) &&
			    !list_empty(&ep->queue)) {
				isp1763_udc_set_ep_index(udc, EP_INDEX(ep->num, ep->dir));
				isp1763_udc_write_ep(ep, req, false);
#if 0
				if (req->req.actual == req->req.length)
					isp1763_udc_set_vendp(udc);
#endif
			}
		}

	}

	ret = 0;
out:
	spin_unlock_irqrestore(&udc->lock, flags);
	return ret;
}

static int isp1763_udc_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct isp1763_ep *ep;
	struct isp1763_request *req;
	struct isp1763_udc *udc;
	unsigned long flags;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct isp1763_ep, ep);
	udc = ep->udc;

	if (ep == &udc->ep[0])
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* make sure the request is actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}

	if (&req->req != _req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	isp1763_udc_complete_req(ep, req, -ECONNRESET);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int isp1763_udc_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct isp1763_ep *ep;
	struct isp1763_udc *udc;

	if (!_ep)
		return -EINVAL;

	ep = container_of(_ep, struct isp1763_ep, ep);
	udc = ep->udc;

	/* TODO */
	dev_err(udc->dev, "set_halt not implemented (yet)\n");

	return 0;
}

static struct usb_ep_ops isp1763_udc_ep_ops = {
	.enable		= isp1763_udc_ep_enable,
	.disable	= isp1763_udc_ep_disable,
	.alloc_request	= isp1763_udc_ep_alloc_request,
	.free_request	= isp1763_udc_ep_free_request,
	.queue		= isp1763_udc_ep_queue,
	.dequeue	= isp1763_udc_ep_dequeue,
	.set_halt	= isp1763_udc_ep_set_halt,
};

static int isp1763_udc_get_frame(struct usb_gadget *gadget)
{
	struct isp1763_udc *udc = get_gadget_data(gadget);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&udc->lock, flags);
	ret = isp1763_readw(udc, ISP1763_REG_FRAME_NO);
	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static struct usb_gadget_ops isp1763_udc_gadget_ops = {
	.get_frame	= isp1763_udc_get_frame,
};

static void isp1763_udc_configure_ep0(struct isp1763_udc *udc)
{
	unsigned int i;

	for (i = 0; i < 2; i++) {
		/* setup EP0 out */
		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
		isp1763_writew(udc, 64, ISP1763_REG_EP_MAXPKTSIZE);
		isp1763_writew(udc, i << 3, ISP1763_REG_EP_TYPE);
		/* setup EP0 in */
		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_TX));
		isp1763_writew(udc, 64, ISP1763_REG_EP_MAXPKTSIZE);
		isp1763_writew(udc, i << 3, ISP1763_REG_EP_TYPE);
		/* setup EP0SETUP */
		isp1763_udc_set_ep_index(udc, ISP1763_EP_INDEX_EP0SETUP);
		isp1763_writew(udc, 64, ISP1763_REG_EP_MAXPKTSIZE);
		isp1763_writew(udc, i << 3, ISP1763_REG_EP_TYPE);
	}
}

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
			    int (*bind)(struct usb_gadget *))
{
	struct isp1763_udc *udc = controller;
	int ret;

	if (!udc || !driver || !bind || !driver->unbind || !driver->setup)
		return -EINVAL;

	if (udc->driver)
		return -EBUSY;

	udc->driver = driver;

	ret = bind(&udc->gadget);
	if (ret) {
		dev_err(udc->dev, "Failed to bind gadget\n");
		udc->driver = NULL;
		return ret;
	}

	isp1763_udc_configure_ep0(udc);

	dev_dbg(udc->dev, "bound to %s\n", driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_probe_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct isp1763_udc *udc = controller;

	if (!udc || !driver || driver != udc->driver || !driver->unbind)
		return 0;

	driver->unbind(&udc->gadget);
	udc->driver = NULL;

	dev_dbg(udc->dev, "unbound from %s\n", driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static irqreturn_t isp1763_udc_irq(int irq, void *data)
{
	struct isp1763_udc *udc = data;
	unsigned long flags;
	u32 irqs, irqs_en;

	spin_lock_irqsave(&udc->lock, flags);

	/* unlock controller */
	isp1763_writew(udc, ISP1763_UNLOCK_CODE, ISP1763_REG_UNLOCK);

	isp1763_udc_disable_glint(udc);

	irqs = isp1763_readl(udc, ISP1763_REG_DC_INT);
	isp1763_writel(udc, irqs, ISP1763_REG_DC_INT);

	/* Only consider interrupt for enabled endpoints */
	irqs_en = isp1763_readl(udc, ISP1763_REG_DC_INT_EN);
	irqs &= irqs_en;

	if (!irqs)
		goto out;

	if (irqs & ISP1763_DC_INT_VBUS) {
		u16 mode = isp1763_readw(udc, ISP1763_REG_MODE);

		dev_dbg(udc->dev, "VBUS\n");

		if (mode & ISP1763_MODE_VBUSSTAT) {
			u16 tmp = isp1763_readw(udc, ISP1763_REG_OTG_CTRL_SET);
			tmp |= ISP1763_OTG_CTRL_DP_PULLUP;
			isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_SET);
		} else
			isp1763_writew(udc, ISP1763_OTG_CTRL_DP_PULLUP, ISP1763_REG_OTG_CTRL_CLEAR);

	}

	/* EP0 setup */
	if (irqs & ISP1763_DC_INT_EP0SETUP) {
		dev_dbg(udc->dev, "EP0SETUP\n");

		isp1763_udc_handle_ep0setup(udc);
	}

	/* EP0 RX: Control Out */
	if (irqs & ISP1763_DC_INT_EP0RX) {
		struct isp1763_request *req;
		struct isp1763_ep *ep0 = &udc->ep[0];

		dev_dbg(udc->dev, "EP0RX\n");

		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));

		if (isp1763_readw(udc, ISP1763_REG_BUF_LEN) == 0) {
			isp1763_udc_set_status(udc);
			goto ep0rx_out;
		}

		if (list_empty(&ep0->queue)) {
			dev_dbg(udc->dev, "EP0RX: queue empty\n");
			goto ep0rx_out;
		}

		req = list_entry(ep0->queue.next, struct isp1763_request, queue);

		dev_dbg(udc->dev, "EP0TX: request len=%d, written=%d\n",
				req->req.length, req->req.actual);

		isp1763_udc_read_ep(ep0, req, false);
		isp1763_udc_set_status(udc);
		list_del_init(&req->queue);
	}
ep0rx_out:

	/* EP0 TX: Control In */
	if (irqs & ISP1763_DC_INT_EP0TX) {
		struct isp1763_ep *ep = &udc->ep[0];
		struct isp1763_request *req;

		dev_dbg(udc->dev, "EP0TX\n");

		if (ep->dir != ISP1763_EP_INDEX_DIR_TX)
			goto ep0tx_out;

		if (list_empty(&ep->queue)) {
			dev_dbg(udc->dev, "EP0TX: queue empty\n");
			goto ep0tx_out;
		}

		req = list_entry(ep->queue.next, struct isp1763_request, queue);

		dev_dbg(udc->dev, "EP0TX: request len=%d, written=%d\n",
				req->req.length, req->req.actual);

		isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_TX));
		if (req->req.actual < req->req.length) {
			dev_dbg(udc->dev, "EP0TX: request not complete\n");

			isp1763_udc_write_ep(ep, req, false);
		}
		if (req->req.actual == req->req.length) {
			dev_dbg(udc->dev, "EP0TX: request complete\n");
			isp1763_udc_set_ep_index(udc, EP_INDEX(0, ISP1763_EP_INDEX_DIR_RX));
			isp1763_udc_set_status(udc);
			isp1763_udc_complete_req(ep, req, 0);
		}
	}
ep0tx_out:

	if (irqs & ISP1763_DC_INT_BRESET) {
		dev_dbg(udc->dev, "BRESET\n");

		isp1763_writel(udc, 0, ISP1763_REG_DC_INT);
		isp1763_udc_connect(udc);
		isp1763_writel(udc, ISP1763_DC_INT_EP_ANY | ISP1763_DC_INT_EN_DEFAULT, ISP1763_REG_DC_INT_EN);
	}

	/* EP interrupts */
	if (irqs & ISP1763_DC_INT_EP_ANY) {
		unsigned int i;
		dev_dbg(udc->dev, "EP IRQ: %08x\n", irqs & ISP1763_DC_INT_EP_ANY);
		/* EP0 is special and not handled here */
		for (i = 1; i < ISP1763_UDC_MAX_ENDPOINTS; i++) {
			if (irqs & (1 << (11 + i))) {
				if (isp1763_udc_handle_ep_irq(&udc->ep[i]))
					dev_err(udc->dev, "Error while handling endpoint interrupt for EP %s\n", udc->ep[i + 1].name);
			}
		}
	}

out:
	isp1763_udc_enable_glint(udc);
	spin_unlock_irqrestore(&udc->lock, flags);
	return IRQ_HANDLED;
}

static int isp1763_udc_init_hw(struct isp1763_udc *udc)
{
	u32 chip_id;
	u16 tmp;
	int ret;

	pr_debug("-> entering %s\n", __func__);

	/* Unlock the controller */
	isp1763_writew(udc, ISP1763_UNLOCK_CODE, ISP1763_REG_UNLOCK);

	/* Dummy reads to stabilize controller access */
	mdelay(10);
	isp1763_readw(udc, ISP1763_REG_CHIP_ID);
	isp1763_readw(udc, ISP1763_REG_CHIP_ID);
	isp1763_readw(udc, ISP1763_REG_CHIP_ID);
	mdelay(20);

	/* Check the chip ID */
	chip_id = isp1763_readl(udc, ISP1763_REG_CHIP_ID);
	if (chip_id != ISP1763_CHIP_ID) {
		dev_err(udc->dev, "ISP1763 chip ID is wrong (%08x instead of %08x)\n",
			chip_id, ISP1763_CHIP_ID);
		return -EIO;
	}

#if 0
	/* Select bus width and interrupt polarity (XXX: hardcoded for now) */
	hwmode &= ~ISP1763_HW_MODE_CTRL_DATA_BUS_WIDTH;
	hwmode &= ~ISP1763_HW_MODE_CTRL_INTR_POL;
	hwmode &= ~ISP1763_HW_MODE_CTRL_INTR_LEVEL;
	isp1763_writew(udc, hwmode, ISP1763_REG_HW_MODE_CTRL);

	/* Lock interface mode */
	tmp = isp1763_readw(udc, ISP1763_REG_HW_MODE_CTRL);
	tmp |= ISP1763_HW_MODE_CTRL_INTF_LOCK;
	isp1763_writew(udc, tmp, ISP1763_REG_HW_MODE_CTRL);

	/* Unlock the controller */
	isp1763_writew(udc, ISP1763_UNLOCK_CODE, ISP1763_REG_UNLOCK);
#endif
	/* Soft reset */
#if 0
	tmp = isp1763_readw(udc, ISP1763_REG_SWRESET);
	tmp |= ISP1763_SWRESET_RESET_ALL | ISP1763_SWRESET_RESET_ATX;
	isp1763_writew(udc, tmp, ISP1763_REG_SWRESET);
#endif
	isp1763_writew(udc, ISP1763_MODE_SFRESET, ISP1763_REG_MODE);
	mdelay(1);
	isp1763_writew(udc, 0, ISP1763_REG_MODE);
	mdelay(5);

	/* Check the chip ID, again */
	chip_id = isp1763_readl(udc, ISP1763_REG_CHIP_ID);
	if (chip_id != ISP1763_CHIP_ID) {
		dev_err(udc->dev, "ISP1763 chip ID is wrong after soft reset (%08x instead of %08x)\n",
			chip_id, ISP1763_CHIP_ID);
		return -EIO;
	}

	/* Check register access */
	ret = 0;
	for (tmp = 0; tmp < 0x10; tmp++) {
		u16 val;

		isp1763_writew(udc, tmp, ISP1763_REG_SCRATCH);
		val = isp1763_readw(udc, ISP1763_REG_SCRATCH);
		if (val != tmp) {
			dev_err(udc->dev, "Scratch register write test failed (wrote %04x,read %04x)\n", tmp, val);
			ret = -EIO;
			/* don't break here */
		}
	}

	if (ret)
		return ret;

#if 0
	isp1763_writew(udc, hwmode, ISP1763_REG_HW_MODE_CTRL);

	/* Set interrupt level */
	tmp = isp1763_readw(udc, ISP1763_REG_HW_MODE_CTRL);
	tmp &= ~ISP1763_HW_MODE_CTRL_INTR_POL;
	tmp &= ~ISP1763_HW_MODE_CTRL_INTR_LEVEL;
	isp1763_writew(udc, tmp, ISP1763_REG_HW_MODE_CTRL);
#endif
	/* Enable device mode */
	isp1763_writew(udc, 0xFFFF, ISP1763_REG_OTG_CTRL_CLEAR);
	tmp = isp1763_readw(udc, ISP1763_REG_OTG_CTRL_SET);
	tmp |= ISP1763_OTG_CTRL_DM_PULLDOWN | ISP1763_OTG_CTRL_DP_PULLDOWN;
	isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_SET);
	/* disable OTG */
	tmp |= ISP1763_OTG_CTRL_OTG_DISABLE;
	isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_SET);
	tmp |= ISP1763_OTG_CTRL_SW_SEL_HC_DC;
	isp1763_writew(udc, tmp, ISP1763_REG_OTG_CTRL_SET);

	mdelay(1);

	/* Set mode register */
	tmp = ISP1763_MODE_GLINTENA | ISP1763_MODE_WKUPCS;
	isp1763_writew(udc, tmp, ISP1763_REG_MODE);

	tmp = isp1763_readw(udc, ISP1763_REG_HW_MODE_CTRL);
	tmp |= ISP1763_HW_MODE_CTRL_GLOBAL_INTR_EN | ISP1763_HW_MODE_CTRL_INTF_LOCK | ISP1763_HW_MODE_CTRL_COMN_INT;
	isp1763_writew(udc, tmp, ISP1763_REG_HW_MODE_CTRL);

	/* "Connect" the chip */
	isp1763_udc_connect(udc);

	pr_debug("<- leaving %s\n", __func__);

	return 0;
}

static void isp1763_udc_setup_device(struct isp1763_udc *udc)
{
	unsigned int i;

	spin_lock_init(&udc->lock);

	udc->gadget.ops = &isp1763_udc_gadget_ops;
	udc->gadget.ep0	= &udc->ep[0].ep;
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.speed = USB_SPEED_HIGH;
	udc->gadget.name = "isp1763_udc";

	/*
	 * Total EP FIFO size, excluding set-up token buffer, control IN and
	 * control OUT (64 bytes each)
	 */
	udc->ep_fifo_space = 4096 - (3 * 64);

	/* Initialize endpoints */
	for (i = 0; i < ISP1763_UDC_MAX_ENDPOINTS; i++) {
		struct isp1763_ep *ep = &udc->ep[i];

		/* ep0 is special */
		if (i == 0) {
			ep->ep.maxpacket = 64;
			snprintf(ep->name, 8, ep0name);
			ep->dir = 255;
		} else {
			snprintf(ep->name, 8, "ep%i%s",
				 (i + 1) >> 1,
				 ((i + 1) & 1) ? "in" : "out");
			ep->ep.maxpacket = 4096;
			ep->dir = ((i + 1) & 1) ?
				ISP1763_EP_INDEX_DIR_TX : ISP1763_EP_INDEX_DIR_RX;
		}

		ep->ep.name = ep->name;
		dev_dbg(udc->dev, "%d: ep->name = %s\n", i, ep->name);
		ep->ep.ops = &isp1763_udc_ep_ops;

		INIT_LIST_HEAD(&ep->queue);
		ep->udc = udc;
		ep->num = (i + 1) >> 1;

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	}
}

static int isp1763_udc_probe(struct platform_device *pdev)
{
	struct isp1763_udc *udc;
	struct resource *res;
	int ret = 0;

	/* only allow one instance */
	if (controller)
		return -EBUSY;

	udc = devm_kzalloc(&pdev->dev, sizeof(struct isp1763_udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	udc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get memory resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start, resource_size(res),
			"isp1763")) {
		dev_err(&pdev->dev, "can't reserve memory region\n");
		return -EBUSY;
	}

	udc->base = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!udc->base) {
		dev_err(&pdev->dev, "can't remap memory region\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get IRQ resource\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, res->start, isp1763_udc_irq, 0, driver_name, udc);
	if (ret) {
		dev_err(&pdev->dev, "can't get IRQ %i\n", res->start);
		return ret;
	}

	udc->irq = res->start;

	/* Initialize device data */
	isp1763_udc_setup_device(udc);

	/* Hardware power on initialization */
	ret = isp1763_udc_init_hw(udc);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize device\n");
		return ret;
	}

	set_gadget_data(&udc->gadget, udc);

	dev_set_name(&udc->gadget.dev, "gadget");
	ret = device_register(&udc->gadget.dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register gadget device\n");
		return ret;
	}

	platform_set_drvdata(pdev, udc);
	controller = udc;

	return 0;
}

static int __devexit isp1763_udc_remove(struct platform_device *pdev)
{
	struct isp1763_udc *udc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	kfree(udc);
	controller = NULL;

	return 0;
}

static struct platform_driver isp1763_udc_driver = {
	.driver		= {
		.name		= "isp1763",
		.owner		= THIS_MODULE,
	},
	.remove		= __devexit_p(isp1763_udc_remove),
};

static int __init isp1763_udc_init(void)
{
	return platform_driver_probe(&isp1763_udc_driver, isp1763_udc_probe);
}

static void __exit isp1763_udc_exit(void)
{
	platform_driver_unregister(&isp1763_udc_driver);
}

module_init(isp1763_udc_init);
module_exit(isp1763_udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:isp1763_udc");
