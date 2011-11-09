#ifndef _ISP1763_HCD_H_
#define _ISP1763_HCD_H_

int isp1763_hc_init(void);
void isp1763_hc_exit(void);

/* exports for isp1763-if */
struct usb_hcd *isp1763_register(phys_addr_t res_start,
				 resource_size_t res_len,
				 int irq,
				 unsigned long irqflags,
				 struct device *dev,
				 const char *busname, unsigned int devflags);
int init_kmem_once(void);
void deinit_kmem_cache(void);

/* EHCI capability registers;
 * Vendor claims that HCCPARAMS and HCSPARAMS does not exist for isp1763,
 * Any access to these registers need to return some hardcoded value.
 */
#define HC_CAPLENGTH		0x00
#define HC_HCSPARAMS		0x04
#define HC_HCCPARAMS		0x08

/* EHCI operational registers */
#define HC_USBCMD		0x8C
#define HC_USBSTS		0x90
#define HC_FRINDEX		0x98
#define HC_CONFIGFLAG		0x9C
#define HC_PORTSC1		0xA0
#define HC_ISO_PTD_DONEMAP_REG	0xA4
#define HC_ISO_PTD_SKIPMAP_REG	0xA6
#define HC_ISO_PTD_LASTPTD_REG	0xA8
#define HC_INT_PTD_DONEMAP_REG	0xAA
#define HC_INT_PTD_SKIPMAP_REG	0xAC
#define HC_INT_PTD_LASTPTD_REG	0xAE
#define HC_ATL_PTD_DONEMAP_REG	0xB0
#define HC_ATL_PTD_SKIPMAP_REG	0xB2
#define HC_ATL_PTD_LASTPTD_REG	0xB4

/* Configuration Register */
#define HC_HW_MODE_CTRL		0xB6
#define HW_ID_PULLUP		(1 << 12)
#define HW_DEV_DMA_EN		(1 << 11)
#define HW_COMN_INT_EN		(1 << 10)
#define HW_COMN_DMA_EN		(1 << 9)
#define HW_DACK_POL_HIGH	(1 << 6)
#define HW_DREQ_POL_HIGH	(1 << 5)
#define HW_DATA_BUS_8BIT	(1 << 4)
#define HW_INTF_LOCK		(1 << 3)
#define HW_INTR_HIGH_ACT	(1 << 2)
#define HW_INTR_EDGE_TRIG	(1 << 1)
#define HW_GLOBAL_INTR_EN	(1 << 0)

#define HC_CHIP_ID_REG		0x70
#define HC_SCRATCH_REG		0x78
#define HC_RESET_REG		0xB8
#define HC_PORT1_SC		0xA0

/* INTF_MODE[1:0] is SW_RESET[7:6] */
#define SW_INTF_MODE_MASK	0xC0
/* The various operation mode the chip supports */
#define SW_INTF_MODE_NAND	0x00
#define SW_INTF_MODE_GNRC	0x40
#define SW_INTF_MODE_NOR	0x80
#define SW_INTF_MODE_SRAM	0xC0

#define SW_RESET_RESET_ATX	(1 << 3)
#define SW_RESET_RESET_HC	(1 << 1)
#define SW_RESET_RESET_ALL	(1 << 0)

#define HC_BUFFER_STATUS_REG	0xBA
#define ATL_BUFFER		0x1
#define INT_BUFFER		0x2
#define ISO_BUFFER		0x4
#define BUFFER_MAP		0x7

#define HC_MEMORY_REG		0xC4

#define HC_DATA_REG		0xC6

#define HW_OTG_CTRL_SET		0xE4
#define HW_OTG_CTRL_CLR		0xE6
#define HW_OTG_CTRL_HC_2_DIS		(1 << 15)
#define HW_OTG_CTRL_OTG_DIS		(1 << 10)
#define HW_OTG_CTRL_SW_SEL_HC_DC	(1 << 7)

/* Interrupt Register */
#define HC_INTERRUPT_REG	0xD4

#define HC_INTERRUPT_ENABLE	0xD6
#define INTERRUPT_ENABLE_MASK	(HC_INTL_INT | HC_ATL_INT | HC_EOT_INT \
					| HC_OPR_REG_INT | HC_OTG_INT)
#define HC_OTG_INT		(1 << 10)
#define HC_ISO_INT		(1 << 9)
#define HC_ATL_INT		(1 << 8)
#define HC_INTL_INT		(1 << 7)
#define HC_CLK_READY_INT	(1 << 6)
#define HC_HCSUSP_INT		(1 << 5)
#define HC_OPR_REG_INT		(1 << 4)
#define HC_EOT_INT		(1 << 3)
#define HC_SOF_INT		(1 << 1) /* renamed from HC_SOT_INT */
#define HC_USOF_INT		(1 << 0) /* micro SOF interrupt */

#define HC_ISO_IRQ_MASK_OR_REG	0xD8
#define HC_INT_IRQ_MASK_OR_REG	0xDA
#define HC_ATL_IRQ_MASK_OR_REG	0xDC
#define HC_ISO_IRQ_MASK_AND_REG	0xDE
#define HC_INT_IRQ_MASK_AND_REG	0xE0
#define HC_ATL_IRQ_MASK_AND_REG	0xE2

/* Register sets */
#define HC_BEGIN_OF_ATL		0x0c00
#define HC_BEGIN_OF_INT		0x0800
#define HC_BEGIN_OF_ISO		0x0400
#define HC_BEGIN_OF_PAYLOAD	0x1000 /* size 20KB */

/* urb state*/
#define DELETE_URB		(0x0008)
#define NO_TRANSFER_ACTIVE	(0xffff)

#define ATL_REGS_OFFSET		(0x0c00)
#define INT_REGS_OFFSET		(0x0800)

/* Philips/Proprietary Transfer Descriptor (PTD) */
struct ptd {
	__le32 dw0;
	__le32 dw1;
	__le32 dw2;
	__le32 dw3;
	__le32 dw4;
	__le32 dw5;
	__le32 dw6;
	__le32 dw7;
};

struct inter_packet_info {
	void *data_buffer;
	u32 payload;
#define PTD_FIRE_NEXT           (1 << 0)
#define PTD_URB_FINISHED        (1 << 1)
	struct urb *urb;
	struct isp1763_qh *qh;
	struct isp1763_qtd *qtd;
};

typedef void (packet_enqueue) (struct usb_hcd *hcd,
			       struct isp1763_qh *qh,
			       struct isp1763_qtd *qtd);

#define isp1763_dbg(priv, fmt, args...) \
	dev_dbg(priv_to_hcd(priv)->self.controller, fmt, ##args)

#define isp1763_info(priv, fmt, args...) \
	dev_info(priv_to_hcd(priv)->self.controller, fmt, ##args)

#define isp1763_err(priv, fmt, args...) \
	dev_err(priv_to_hcd(priv)->self.controller, fmt, ##args)

/* chip memory management */
struct memory_chunk {
	unsigned int start;
	unsigned int size;
	unsigned int free;
};

/*
 * 20kb divided in: (per pehci.h from ST-Ericsson pehci driver)
 * -  8 blocks @ 256  bytes =  2KB (10%)
 * -  6 blocks @ 1024 bytes =  6KB (30%)
 * -  3 blocks @ 4096 bytes = 12KB (60%)
 */
#define BLOCK_1_NUM 8
#define BLOCK_2_NUM 6
#define BLOCK_3_NUM 3

#define BLOCK_1_SIZE 256
#define BLOCK_2_SIZE 1024
#define BLOCK_3_SIZE 4096
#define BLOCKS (BLOCK_1_NUM + BLOCK_2_NUM + BLOCK_3_NUM)
#define PAYLOAD_SIZE 0x5000	/* 20 KB */

/* I saw if some reloads if the pointer was negative */
#define ISP1763_NULL_POINTER	(0x400)

/* ATL */
/* DW0 */
#define PTD_VALID			1
#define PTD_LENGTH(x)			(((u32) x) << 3)
#define PTD_MAXPACKET(x)		(((u32) x) << 18)
#define PTD_MULTI(x)			(((u32) x) << 29)
#define PTD_ENDPOINT(x)			(((u32) x) << 31)
/* DW1 */
#define PTD_DEVICE_ADDR(x)		(((u32) x) << 3)
#define PTD_PID_TOKEN(x)		(((u32) x) << 10)
#define PTD_TRANS_BULK			((u32) 2 << 12)
#define PTD_TRANS_INT			((u32) 3 << 12)
#define PTD_TRANS_SPLIT			((u32) 1 << 14)
#define PTD_SE_USB_LOSPEED		((u32) 2 << 16)
#define PTD_PORT_NUM(x)			(((u32) x) << 18)
#define PTD_HUB_NUM(x)			(((u32) x) << 25)
#define PTD_PING(x)			(((u32) x) << 26)
/* DW2 */
#define PTD_RL_CNT(x)			(((u32) x) << 25)
#define PTD_DATA_START_ADDR(x)		(((u32) x) << 8)
#define BASE_ADDR			0x1000
/* DW3 */
#define PTD_CERR(x)			(((u32) x) << 23)
#define PTD_NAC_CNT(x)			(((u32) x) << 19)
#define PTD_ACTIVE			((u32) 1 << 31)
#define PTD_DATA_TOGGLE(x)		(((u32) x) << 25)

#define DW3_HALT_BIT			(1 << 30)
#define DW3_ERROR_BIT			(1 << 28)
#define DW3_QTD_ACTIVE			(1 << 31)

#define INT_UNDERRUN			(1 << 2)
#define INT_BABBLE			(1 << 1)
#define INT_EXACT			(1 << 0)

#define DW1_GET_PID(x)			(((x) >> 10) & 0x3)
#define PTD_XFERRED_LENGTH(x)		((x) & 0x7fff)
#define PTD_XFERRED_LENGTH_LO(x)	((x) & 0x7ff)

#define SETUP_PID	(2)
#define IN_PID		(1)
#define OUT_PID		(0)
#define GET_QTD_TOKEN_TYPE(x)	((x) & 0x3)

#define DATA_TOGGLE		(1 << 31)
#define GET_DATA_TOGGLE(x)	((x) >> 31)

/* FIXME!: Does this apply to isp1763? ST-Ericsson believes no */
/* Set these to HW defaults of ZERO? */
/* Errata 1 */
#define RL_COUNTER	(0) /* isp1760 (0) */
#define NAK_COUNTER	(0) /* isp1760 (0) */
#define ERR_COUNTER	(0) /* isp1760 (2) */

#define HC_ATL_PL_SIZE        (4096)	/* isp1760 sets this to (8192) */

/* Section 2.2 Host Controller Capability Registers */
#define HC_LENGTH(p)		(((p)>>00)&0x00ff)	/* bits 7:0 */
#define HC_VERSION(p)		(((p)>>16)&0xffff)	/* bits 31:16 */
#define HCS_INDICATOR(p)	((p)&(1 << 16))	/* true: has port indicators */
#define HCS_PPC(p)		((p)&(1 << 4))	/* true: port power control */
#define HCS_N_PORTS(p)		(((p)>>0)&0xf)	/* bits 3:0, ports on HC */
#define HCC_ISOC_CACHE(p)       ((p)&(1 << 7))	/* true: can cache isoc frame */
#define HCC_ISOC_THRES(p)       (((p)>>4)&0x7)	/* bits 6:4, uframes cached */

/* Section 2.3 Host Controller Operational Registers */
#define CMD_LRESET	(1<<7)	/* partial reset (no ports, etc) */
#define CMD_RESET	(1<<1)	/* reset HC not bus */
#define CMD_RUN		(1<<0)	/* start/stop HC */
#define STS_PCD		(1<<2)	/* port change detect */
#define FLAG_CF		(1<<0)	/* true: we'll support "high speed" */

#define PORT_OWNER	(1<<13)	/* true: companion hc owns this port */
#define PORT_POWER	(1<<12)	/* true: has power (see PPC) */
#define PORT_USB11(x)	(((x) & (3 << 10)) == (1 << 10)) /* USB 1.1 device */
#define PORT_RESET	(1<<8)	/* reset port */
#define PORT_SUSPEND	(1<<7)	/* suspend port */
#define PORT_RESUME	(1<<6)	/* resume it */
#define PORT_PE		(1<<2)	/* port enable */
#define PORT_CSC	(1<<1)	/* connect status change */
#define PORT_CONNECT	(1<<0)	/* device connected */
#define PORT_RWC_BITS   (PORT_CSC)

/* isp1763 does not have HCCPARAMS and HCSPARAMS, use hardcoded values here */
#define HCS_HARDCODE	(1 | (1 << 4))	/* Port Power Control */
#define HCC_HARDCODE	(1 << 1)	/* Programmable Frame List */

#define NUM_OF_PTD	16	/* isp1760 have 32 max PTDs; isp1763 have 16 */

struct isp1763_hcd {
	u32 hcs_params;
	spinlock_t lock;
	struct inter_packet_info atl_ints[NUM_OF_PTD];
	struct inter_packet_info int_ints[NUM_OF_PTD];
	struct memory_chunk memory_pool[BLOCKS];

	/* periodic schedule support */
#define	DEFAULT_I_TDPS		1024
	unsigned periodic_size;
	unsigned i_thresh;
	unsigned long reset_done;
	unsigned long next_statechange;
	unsigned int devflags;
};

struct isp1763_qtd {
	struct isp1763_qtd *hw_next;
	u8 packet_type;
	u8 toggle;

	void *data_buffer;
	/* the rest is HCD-private */
	struct list_head qtd_list;
	struct urb *urb;
	size_t length;

	/* isp special */
	u32 status;
#define URB_COMPLETE_NOTIFY	(1 << 0)
#define URB_ENQUEUED		(1 << 1)
#define URB_TYPE_ATL		(1 << 2)
#define URB_TYPE_INT		(1 << 3)
};

struct isp1763_qh {
	/* first part defined by EHCI spec */
	struct list_head qtd_list;
	struct isp1763_hcd *priv;

	/* periodic schedule info */
	unsigned short period;	/* polling interval */
	struct usb_device *dev;

	u32 toggle;
	u32 ping;
};

#define ehci_port_speed(priv, portsc) (1 << USB_PORT_FEAT_HIGHSPEED)

#endif
