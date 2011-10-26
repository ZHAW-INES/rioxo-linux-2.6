#ifndef __ISP1763_UDC_H__
#define __ISP1763_UDC_H__

enum ep_types {
	EP_TYPE_UNUSED = 0,
	EP_TYPE_ISOCHRONOUS,
	EP_TYPE_BULK,
	EP_TYPE_INTERRUPT,
};

#define EP_TYPE_ENABLE  (1 << 3)
#define EP_TYPE_DBLBUF  (1 << 2)
#define EP_TYPE_NOEMPKT (1 << 4)
#define EP_MAXPKTSIZE_NTRANS_SHIFT 11

#define USB_MAX_ENDPOINTS 14

/**
* struct queue - queue structure
* @next: next pointer
* @elem: data pointer
*/
struct queue {
	struct queue *next;
	void *elem;
};

struct isp1763_ep {
	struct usb_ep ep;
	struct isp1763_udc *udc;

	unsigned int maxpacket;

	unsigned int is_in:1;

	const struct usb_endpoint_descriptor *desc;
};

/**
* struct isp1763_udc - UDC device
* @isp_ep: endpoints
* @ep0: endpoint 0
* @gadget: gadget interface
* @gadget_driver: pointer to gadget driver
* @isp1763_regs: pointer to memory mapped registers
* @isp1763_controller: pointer to controller structure
*/
struct isp1763_udc {
	struct isp1763_ep ep[USB_MAX_ENDPOINTS];
	struct usb_gadget_driver *driver;

	struct isp_ep *eps;
	struct isp_ep *ep0;
	struct usb_gadget *gadget;
	struct isp1763_regs *regs;

	struct isp1763_controller ctrl;

	spinlock_t lock;
};

/**
* struct isp_ep - endpoint structure
* @ep: embedded usb_ep structure
* @queue: pointer to EP queue
* @udc: pointer to parent UDC device struct
* @gadget: pointer to gadget interface
* @state: current state
* @index: EP index
* @name: EP name (eg ep0-in)
* @maxpacketsize: maximum packet size
* @ep_type: type of endpoint
* @desc: copy of EP descriptor
*/
struct isp_ep {
	struct usb_ep ep;
	struct queue *queue;
	struct isp1763_udc *udc;
	struct usb_gadget *gadget;
	u8 state;
	int index;
	char name[8];
	int maxpacketsize;
	int ep_type;
	struct usb_endpoint_descriptor desc;
};

#define GET_EP_DIR(idx) ((idx) & 1 ? DIR_TX : DIR_RX)
#define GET_EP_NUM_CANONICAL(epn) ((epn) >> 1)

#define EP_INDEX(epnum, is_out) (((epnum) << 1) | !!is_out)
#define set_clbuf_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				| MASK_CTRL_CLBUF, &(dev)->regs->ctrlfn)
#define set_dsen_flag(dev)   isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				| MASK_CTRL_DSEN, &(dev)->regs->ctrlfn)
#define set_status_flag(dev) isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				| MASK_CTRL_STATUS, &(dev)->regs->ctrlfn)
#define set_stall_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				| MASK_CTRL_STALL, &(dev)->regs->ctrlfn)
#define set_vendp_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				| MASK_CTRL_VENDP, &(dev)->regs->ctrlfn)

#define clr_clbuf_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				& ~MASK_CTRL_CLBUF, &(dev)->regs->ctrlfn)
#define clr_dsen_flag(dev)   isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				& ~MASK_CTRL_DSEN, &(dev)->regs->ctrlfn)
#define clr_status_flag(dev) isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				& ~MASK_CTRL_STATUS, &(dev)->regs->ctrlfn)
#define clr_stall_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				& ~MASK_CTRL_STALL, &(dev)->regs->ctrlfn)
#define clr_vendp_flag(dev)  isp1763_writew(isp1763_readw(&(dev)->regs->ctrlfn)\
				& ~MASK_CTRL_VENDP, &(dev)->regs->ctrlfn)

static inline void disable_glint(struct isp1763_udc *dev)
{
	isp1763_writew(isp1763_readw(&dev->regs->mode)
				& ~MASK_MODE_GLINTENA, &dev->regs->mode);
}

static inline void enable_glint(struct isp1763_udc *dev)
{
	isp1763_writew(isp1763_readw(&dev->regs->mode)
				| MASK_MODE_GLINTENA,  &dev->regs->mode);
}

#define disable_glint__(dev)	disable_glint(dev)

#endif
