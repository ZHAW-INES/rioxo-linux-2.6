#ifndef __USB_GADGET_ISP1763_UDC_H
#define __USB_GADGET_ISP1763_UDC_H

#include <linux/types.h>

/* ISP1763 UDC register offsets */
#define ISP1763_REG_ADDR		0x00
#define ISP1763_REG_EP_MAXPKTSIZE	0x04
#define ISP1763_REG_EP_TYPE		0x08
#define ISP1763_REG_MODE		0x0C
#define ISP1763_REG_INT_CONF		0x10
#define ISP1763_REG_DC_INT_EN		0x14
#define ISP1763_REG_DC_INT		0x18
#define ISP1763_REG_BUF_LEN		0x1C
#define ISP1763_REG_DC_BUF_STATUS	0x1E
#define ISP1763_REG_DATA_PORT		0x20
#define ISP1763_REG_CTRL_FUNC		0x28
#define ISP1763_REG_EP_INDEX		0x2C
#define ISP1763_REG_CHIP_ID		0x70
#define ISP1763_REG_FRAME_NO		0x74
#define ISP1763_REG_SCRATCH		0x78
#define ISP1763_REG_UNLOCK		0x7C
#define ISP1763_REG_INT_PULSE_WIDTH	0x80
#define ISP1763_REG_HW_MODE_CTRL	0xB6
#define ISP1763_REG_SWRESET		0xB8
#define ISP1763_REG_HC_INT_EN		0xD6
#define ISP1763_REG_OTG_CTRL_SET	0xE4
#define ISP1763_REG_OTG_CTRL_CLEAR	0xE6

/* ISP1763 UDC register masks */

#define ISP1763_ADDR_DEVEN		0x0080

#define ISP1763_EP_TYPE_ENABLE		0x0008
#define ISP1763_EP_TYPE_DBLBUF		0x0004

#define ISP1763_MODE_WKUPCS		0x0004
#define ISP1763_MODE_GLINTENA		0x0008
#define ISP1763_MODE_SFRESET		0x0010
#define ISP1763_MODE_CLKAON		0x0080
#define ISP1763_MODE_VBUSSTAT		0x0100
#define ISP1763_MODE_DMACLKON		0x0200

#define ISP1763_INT_CONF_INTPOL		0x0001
#define ISP1763_INT_CONF_INTLVL		0x0002
#define ISP1763_INT_CONF_DDBGMODOUT_ALL_ACK	0x0004
#define ISP1763_INT_CONF_DDBGMODIN_ALL_ACK	0x0010
#define ISP1763_INT_CONF_CDBGMOD_ALL_ACK	0x0040

#define ISP1763_DC_INT_BRESET		0x00000001
#define ISP1763_DC_INT_SUSP		0x00000008
#define ISP1763_DC_INT_RESM		0x00000010
#define ISP1763_DC_INT_HS_STA		0x00000020
#define ISP1763_DC_INT_VBUS		0x00000080
#define ISP1763_DC_INT_EP0SETUP		0x00000100
#define ISP1763_DC_INT_EP0RX		0x00000400
#define ISP1763_DC_INT_EP0TX		0x00000800
#define ISP1763_DC_INT_EP_ANY		0x03FFF000

#define ISP1763_DC_INT_EN_DEFAULT	(ISP1763_DC_INT_EP0SETUP	\
					 | ISP1763_DC_INT_EP0RX		\
					 | ISP1763_DC_INT_EP0TX		\
					 | ISP1763_DC_INT_VBUS		\
					 | ISP1763_DC_INT_HS_STA	\
					 | ISP1763_DC_INT_SUSP		\
					 | ISP1763_DC_INT_BRESET	\
					 | ISP1763_DC_INT_SUSP		\
					 | ISP1763_DC_INT_RESM)

#define ISP1763_CTRL_FUNC_STALL		0x0001
#define ISP1763_CTRL_FUNC_STATUS	0x0002
#define ISP1763_CTRL_FUNC_DSEN		0x0004
#define ISP1763_CTRL_FUNC_VENDP		0x0008
#define ISP1763_CTRL_FUNC_CLBUF		0x0010

#define ISP1763_HW_MODE_CTRL_GLOBAL_INTR_EN	0x0001
#define ISP1763_HW_MODE_CTRL_INTR_LEVEL		0x0004
#define ISP1763_HW_MODE_CTRL_INTR_POL		0x0004
#define ISP1763_HW_MODE_CTRL_INTF_LOCK		0x0008
#define ISP1763_HW_MODE_CTRL_DATA_BUS_WIDTH	0x0010
#define ISP1763_HW_MODE_CTRL_COMN_INT		0x0400
#define ISP1763_HW_MODE_CTRL_ID_PULLUP		0x1000

#define ISP1763_SWRESET_RESET_ALL	0x0001
#define ISP1763_SWRESET_RESET_ATX	0x0008

#define ISP1763_OTG_CTRL_DP_PULLUP	0x0001
#define ISP1763_OTG_CTRL_DP_PULLDOWN	0x0002
#define ISP1763_OTG_CTRL_DM_PULLDOWN	0x0004
#define ISP1763_OTG_CTRL_VBUS_DRV	0x0010
#define ISP1763_OTG_CTRL_SW_SEL_HC_DC	0x0080
#define ISP1763_OTG_CTRL_OTG_DISABLE	0x0400

#define ISP1763_EP_INDEX_DIR_TX		0x0001
#define ISP1763_EP_INDEX_DIR_RX		0x0000
#define ISP1763_EP_INDEX_EP0SETUP	0x0020

#define ISP1763_CHIP_ID			0x00176320
#define ISP1763_UNLOCK_CODE		0xAA37

#define ISP1763_UDC_MAX_ENDPOINTS	15

#define ISP1763_USB_REQ_CLASS		0x20
#define ISP1763_USB_REQ_VENDOR		0x40

struct isp1763_udc;

struct isp1763_ep {
	struct usb_ep ep;
	struct list_head queue;
	struct isp1763_udc *udc;
	char name[8];
	unsigned int stopped;
	u8 num;
	u8 dir;
};

#define WINDEX_TO_EP_INDEX(x)	(((x & 0x0F) << 1) | (x >> 7))
#define EP_INDEX(ep, dir)	(((ep) << 1) | (dir))
#define EP_IRQ_BIT(ep, dir)	(1 << (((ep) * 2) + (dir) + 10))

struct isp1763_udc {
	void __iomem *base;
	int irq;

	spinlock_t lock;

	struct isp1763_ep ep[ISP1763_UDC_MAX_ENDPOINTS];
	unsigned int ep_fifo_space;

	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct device *dev;
};

struct isp1763_request {
	struct usb_request req;
	struct list_head queue;
};

static inline bool isp1763_ep_is_tx(struct isp1763_ep *ep)
{
	return ep->dir == ISP1763_EP_INDEX_DIR_TX;
}

#define __REG(x)	((x) << 2)

static inline u32 isp1763_readl(struct isp1763_udc *udc, unsigned int reg)
{
	u32 val;

	val = readw(udc->base + __REG(reg));
	val |= (u16) readw(udc->base + __REG(reg + 2)) << 16;

	return val;
}

static inline void isp1763_writel(struct isp1763_udc *udc, u32 val, unsigned int reg)
{

	writew((u16)(val & 0xFFFF), udc->base + __REG(reg));
	writew((u16)(val >> 16), udc->base + __REG(reg + 2));
	udelay(1);
}

static inline u16 isp1763_readw(struct isp1763_udc *udc, unsigned int reg)
{
	return readw(udc->base + __REG(reg));
}

static inline void isp1763_writew(struct isp1763_udc *udc, u16 val, unsigned int reg)
{
	writew(val, udc->base + __REG(reg));
	udelay(1);
}

#endif /* __USB_GADGET_ISP1763_UDC_H */
