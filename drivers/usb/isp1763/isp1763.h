#ifndef __ISP1763_H__
#define __ISP1763_H__

/*
* Device flags that can vary from board to board.  All of these
* indicate the most "atypical" case, so that a devflags of 0 is
* a sane default configuration.
*/
#define ISP1763_FLAG_BUS_WIDTH_8	0x00000002 /* 8-bit data bus width */
#define ISP1763_FLAG_PORT1_ROLE_HOST	0x00000004 /* Static host */
#define ISP1763_FLAG_PORT1_ROLE_GADGET	0x00000008 /* Static gadget */
#define ISP1763_FLAG_PORT1_ROLE_OTG	0x0000000C /* Port 1 supports OTG */
#define ISP1763_FLAG_DACK_POL_HIGH	0x00000010 /* DACK active high */
#define ISP1763_FLAG_DREQ_POL_HIGH	0x00000020 /* DREQ active high */
#define ISP1763_FLAG_INTR_POL_HIGH	0x00000080 /* Active high Interrupt */
#define ISP1763_FLAG_INTR_EDGE_TRIG	0x00000100 /* Edge trigger Interrupt */
#define ISP1763_FLAG_DMA_ENABLED	0x00001000
#define ISP1763_FLAG_DMA_EXTERNAL	0x00002000 /* 1:ext DMA; 0: Internal */

/* memory map of ISP1763A USB dual role/OTG controller */
struct isp1763_regs {

	/* DC registers (mostly) */
	u16 address;		/* 0x00 device address */
	u16 reserved001;
	u16 ep_maxpktsize;	/* 0x04 EP maxpacketsize */
	u16 reserved002;
	u16 ep_type;		/* 0x08 EP type */
	u16 reserved003;
	u16 mode;		/* 0x0C mode register */
	u16 reserved004;
	u16 icr;		/* 0x10 Interrupt control register */
	u16 debug;		/* 0x12 debug register - useless */
	u32 dc_int_enable;	/* 0x14 DC interrupt enable */
	u32 dc_interrupt;	/* 0x18 DC interrupt */
	u16 buflen;		/* 0x1C EP buffer length */
	u16 dcbufstatus;	/* 0x1E EP buffer status */
	u16 data_port;		/* 0x20 DC data port */
	u16 reserved006[3];
	u16 ctrlfn;		/* 0x28 DC control function */
	u16 reserved007;
	u16 ep_index;		/* 0x2C PIO EP index */
	u16 reserved008;
	u16 dma_cmd;		/* 0x30 DMA command */
	u16 reserved009;
	u32 dma_xfer_cnt;	/* 0x34 DMA transfer count */
	u16 dcdmaconf;		/* 0x38 DC DMA config */
	u16 reserved010;
	u16 dmahw;		/* 0x3C DMA hardware parameters */
	u8 reserved011[18];
	u16 dma_ireason;	/* 0x50 DMA interrupt reason */
	u16 reserved012;
	u16 dma_ienable;	/* 0x54 DMA interrupt enable */
	u16 reserved013;
	u16 dma_ep;		/* 0x58 DMA EP index */
	u8 reserved014[6];
	u16 dma_data;		/* 0x60 DMA data register */
	u16 reserved015;
	u16 dma_burst_cnt;	/* 0x64 DMA burst count */
	u16 reserved016[5];
	u32 chip_id;		/* 0x70 chip ID */
	u16 frameno;		/* 0x74 current frame number */
	u16 reserved017;
	u16 scratch;		/* 0x78 scratch register */
	u16 reserved018;
	u16 unlock;		/* 0x7C unlock register */
	u16 reserved019;
	u16 int_pulse_width;	/* 0x80 interrupt pulse width */
	u16 reserved020;
	u16 testmode;		/* 0x84 testmode */
	u16 reserved021[3];

	/* HC registers */
	u32 usbcmd;		/* 0x8C USB command register */
	u32 usbsts;		/* 0x90 USB status register */
	u32 reserved022;
	u32 frindex;		/* 0x98 frame index */
	u32 configflag;		/* 0x9C config valid flag */
	u32 portsc1;		/* 0xA0 Port 1 status register */
	u16 iso_pt_done_map;	/* 0xA4 ISO transfer done map */
	u16 iso_pt_skip_map;	/* 0xA6 ISO transfer skip map */
	u16 iso_pt_last_ptd;	/* 0xA8 ISO transfer last  */
	u16 int_pt_done_map;	/* 0xAA INT transfer done map */
	u16 int_pt_skip_map;	/* 0xAC INT transfer skip map */
	u16 int_pt_last_ptd;	/* 0xAE INT transfer last */
	u16 atl_pt_done_map;	/* 0xB0 ATL transfer done map */
	u16 atl_pt_skip_map;	/* 0xB2 ATL transfer skip map */
	u16 atl_pt_last_ptd;	/* 0xB4 ATL transfer last map */
	u16 hwmodectrl;		/* 0xB6 Hardware mode control */
	u16 swreset;		/* 0xB8 software reset */
	u16 hcbufferstatus;	/* 0xBA Host controller buffer status */
	u32 hcdmaconfig;	/* 0xBC Host controller DMA config */
	u32 atl_done_timeout;	/* 0xC0 ATL done timeout */
	u16 memory;		/* 0xC2 HC memory pointer register */
	u16 data;		/* 0xC6 HC data register */
	u32 edge_int_count;	/* 0xC8 Edge interrupt configuration */
	u16 dma_start_addr;	/* 0xCC DMA start address */
	u16 reserved023;
	u32 power_down_ctrl;	/* 0xD0 power down control */
	u16 hc_interrupt;	/* 0xD4 HC interrupt status */
	u16 hc_interrupt_enable;	/* 0xD6 HC interrupt enable */
	u16 iso_irq_mask_or;	/* 0xD8 ISO IRQ mask OR */
	u16 int_irq_mask_or;	/* 0xDA INT IRQ mask OR */
	u16 atl_irq_mask_or;	/* 0xDC ATL IRQ mask OR */
	u16 reserved024;

	/* OTG registers */
	u32 reserved025;
	u32 otg_ctrl;            /* 0xE4 OTG control set & clear */
	u16 otg_status;          /* 0xE8 OTG status set & clear */
	u16 reserved026;
	u16 otg_int_latch_set;   /* 0xEC OTG interrupt latch set */
	u16 otg_int_latch_clear; /* 0xEE OTG interrupt latch clear */
	u32 otg_int_enable_fall; /* 0xF0 OTG interrupt falling set & clr */
	u32 otg_int_enable_rise; /* 0xF4 OTG interrupt rising set & clr */
	u16 otg_timer_lw_set;	 /* 0xF8 OTG timer configuration low word set */
	u16 otg_timer_lw_clear;	 /* 0xFA OTG timer configuration low word clear */
	u16 otg_timer_hw_set;	 /* 0xFC OTG timer configuration high word set */
	u16 otg_timer_hw_clear;	 /* 0xFE OTG timer configuration high word clear */
} __attribute__ ((packed,aligned(1)));

/** struct isp1763_controller - common struct for interface and private data
*/
struct isp1763_controller {
	int active;
	struct device *device;
	struct isp1763_regs *regs;
	int (*do_irq) (struct isp1763_controller *ctrl, u32 flags);
	int (*suspend) (struct isp1763_controller *ctrl);
	int (*resume) (struct isp1763_controller *ctrl);
	int (*probe) (struct isp1763_controller *ctrl);
	int (*remove) (struct isp1763_controller *ctrl);

	void *priv;
};

typedef void (*otgtimer_callback_t) (void *data);

/**
* struct isp1763_dev - main driver structure
* @irq: interrupt number
* @current_role: currently active role
* @ctrl: array of controllers, role is index
* @regs: pointer to memory mapped chip registers
* @otgtimer_cb: registered OTG timer callback
* @otgtimer_data: registered OTG timer callback context data
* @devflags: saved hardware configuration flags
* @dev: pointer to parent device structure
*/
struct isp1763_dev {
	int irq;
	int current_role;
	struct isp1763_controller *ctrl[2];
	struct isp1763_regs *regs;
	otgtimer_callback_t otgtimer_cb;
	void *otgtimer_data;
	unsigned long devflags;
	struct device *dev;
};

#define ISP_CHIP_ID	0x00176320ul

#define MASK_OTGINTEN_DP2_SRP		(1 << 11)
#define MASK_OTGINTEN_P2_A_SESS_VALID	(1 << 10)
#define MASK_OTGINTEN_TMR_TIMEOUT	(1 <<  9)
#define MASK_OTGINTEN_B_SE0_SRP		(1 <<  8)
#define MASK_OTGINTEN_B_SESS_END	(1 <<  7)
#define MASK_OTGINTEN_BDIS_ACON		(1 <<  6)
#define MASK_OTGINTEN_RMT_CONN		(1 <<  4)
#define MASK_OTGINTEN_ID		(1 <<  3)
#define MASK_OTGINTEN_DP_SRP		(1 <<  2)
#define MASK_OTGINTEN_SESS_VALID	(1 <<  1)
#define MASK_OTGINTEN_VBUS_VALID	(1 <<  0)

#define MASK_HCINT_OTG		(1 << 10)
#define MASK_HCINT_ISO		(1 <<  9)
#define MASK_HCINT_ATL		(1 <<  8)
#define MASK_HCINT_INT		(1 <<  7)
#define MASK_HCINT_CLK_READY	(1 <<  6)
#define MASK_HCINT_HCSUSP	(1 <<  5)
#define MASK_HCINT_OPR_REG	(1 <<  4)
#define MASK_HCINT_DMAEOT_INT	(1 <<  3)
#define MASK_HCINT_SOFINT	(1 <<  1)
#define MASK_HCINT_MSOFINT	(1 <<  0)

#define MASK_OTGSTATUS_ID	0x0008

#define MASK_INTCONFIG_INTLVL	0x02
#define MASK_INTCONFIG_INTPOL	0x01

#define USE_LEVEL	1
#define USE_PULSE	0
#define TRIG_HIGH	1
#define TRIG_LOW	0

#define MASK_HWMODECTRL_ID_PU_DISABLE		(1 << 12)
#define MASK_HWMODECTRL_DEV_DMA			(1 << 11)
#define MASK_HWMODECTRL_COMN_INT		(1 << 10)
#define MASK_HWMODECTRL_COMN_DMA		(1 <<  9)
#define MASK_HWMODECTRL_DACK_POL		(1 <<  6)
#define MASK_HWMODECTRL_DREQ_POL		(1 <<  5)
#define MASK_HWMODECTRL_DATA_BUS_WIDTH		(1 <<  4)
#define MASK_HWMODECTRL_INTF_LOCK		(1 <<  3)
#define MASK_HWMODECTRL_INTR_POLARITY		(1 <<  2)
#define MASK_HWMODECTRL_INTR_EDGE		(1 <<  1)
#define MASK_HWMODECTRL_GLOBAL_INT_ENABLE	(1 <<  0)

#define MASK_OTGCTRL_HC2DIS		(1 << 15)
#define MASK_OTGCTRL_TMR_SEL		(1 << 13)
#define MASK_OTGCTRL_DISABLE		(1 << 10)
#define MASK_OTGCTRL_SE0_EN		(1 <<  9)
#define MASK_OTGCTRL_BDIS_ACON_EN	(1 <<  8)
#define MASK_OTGCTRL_SW_SEL_HC_DC	(1 <<  7)
#define MASK_OTGCTRL_VBUS_CHRG		(1 <<  6)
#define MASK_OTGCTRL_VBUS_DISCHRG	(1 <<  5)
#define MASK_OTGCTRL_VBUS_DRV		(1 <<  4)
#define MASK_OTGCTRL_DM_PULLDOWN	(1 <<  2)
#define MASK_OTGCTRL_DP_PULLDOWN	(1 <<  1)
#define MASK_OTGCTRL_DP_PULLUP		(1 <<  0)

#define MASK_MODE_DMACLOCK_ON	(1 << 9)
#define MASK_MODE_VBUSSTAT	(1 << 8)
#define MASK_MODE_CLKAON	(1 << 7)
#define MASK_MODE_SNDRSU	(1 << 6)
#define MASK_MODE_GOSUSP	(1 << 5)
#define MASK_MODE_SFRESET	(1 << 4)
#define MASK_MODE_GLINTENA	(1 << 3)
#define MASK_MODE_WKUPCS	(1 << 2)

#define MASK_DCINT_BRESET	(1 <<  0)
#define MASK_DCINT_SOF		(1 <<  1)
#define MASK_DCINT_PSOF		(1 <<  2)
#define MASK_DCINT_SUSP		(1 <<  3)
#define MASK_DCINT_RESUME	(1 <<  4)
#define MASK_DCINT_HS_STAT	(1 <<  5)
#define MASK_DCINT_DMA		(1 <<  6)
#define MASK_DCINT_VBUS		(1 <<  7)
#define MASK_DCINT_EP0SETUP	(1 <<  8)
#define MASK_DCINT_EP0RX	(1 << 10)
#define MASK_DCINT_EP0TX	(1 << 11)
#define MASK_DCINT_EP1RX	(1 << 12)
#define MASK_DCINT_EP1TX	(1 << 13)
#define MASK_DCINT_EP2RX	(1 << 14)
#define MASK_DCINT_EP2TX	(1 << 15)
#define MASK_DCINT_EP3RX	(1 << 16)
#define MASK_DCINT_EP3TX	(1 << 17)
#define MASK_DCINT_EP4RX	(1 << 18)
#define MASK_DCINT_EP4TX	(1 << 19)
#define MASK_DCINT_EP5RX	(1 << 20)
#define MASK_DCINT_EP5TX	(1 << 21)
#define MASK_DCINT_EP6RX	(1 << 22)
#define MASK_DCINT_EP6TX	(1 << 23)
#define MASK_DCINT_EP7RX	(1 << 24)
#define MASK_DCINT_EP7TX	(1 << 25)

#define MASK_DCINT_EP_EVENT		0x03FFF000
#define MASK_DCINTENABLE_RELEVANT	0x03FFFDF9
#define UNLOCK_CODE			0xAA37

#define MASK_CTRL_STALL		(1 << 0)
#define MASK_CTRL_STATUS	(1 << 1)
#define MASK_CTRL_DSEN		(1 << 2)
#define MASK_CTRL_VENDP		(1 << 3)
#define MASK_CTRL_CLBUF		(1 << 4)

#define TOTAL_FIFO_SIZE	4096

#define IDX_EP0_SETUP	(1 << 5)
#define DIR_TX	1
#define DIR_RX	0

#define SW_RESET_RESET_ATX	(1 << 3)
#define SW_RESET_RESET_HC	(1 << 1)
#define SW_RESET_RESET_ALL	(1 << 0)

#if 0
#define isp1763_readl(val)		readl(val)
#define isp1763_writel(val, regs)	writel(val, regs)
#define isp1763_readw(regs)		readw(regs)
#define isp1763_writew(val, regs)	writew(val, regs)
#else

/* XXX. Dirty hack because of fscked Avalon interface to ISP1763 */

static inline u32 isp1763_readl(void __iomem *addr)
{
	u32 val;
	u32 a = (u32) addr;
	u32 off = a & 0xFF;

	a &= ~0x0000000000FF;
	a += off << 2;

	val = readw(a);
	val |= (u16) readw(a + (2 << 2)) << 16;

	return val;
}

static inline void isp1763_writel(u32 val, void __iomem *addr)
{
	u32 a = (u32) addr;
	u32 off = a & 0xFF;

	a &= ~0x0000000000FF;
	a += off << 2;

	writew((u16)val & 0xFFFF, a);
	writew((u16)(val >> 16), a + (2 << 2));
}

static inline u16 isp1763_readw(void __iomem *addr)
{
	u16 val;
	u32 a = (u32) addr;
	u32 off = a & 0xFF;

	a &= ~0x0000000000FF;
	a += off << 2;

	val = readw(a);

	return val;
}

static inline void isp1763_writew(u16 val, void __iomem *addr)
{
	u32 a = (u32) addr;
	u32 off = a & 0xFF;

	a &= ~0x0000000000FF;
	a += off << 2;

	writew(val, a);
}

#endif

#define get_id_pin(dev) (!!(isp1763_readw(&dev->regs->otg_status) \
				& MASK_OTGSTATUS_ID))

enum id_status {
	ID_PIN_HOST = 0,
	ID_PIN_DEVICE,
};

enum role {
	ROLE_HOST = 0,
	ROLE_DEVICE,
};

#define queue_printk(args...)	/* printk(args) */
#define info_printk(args...)	/* printk(args) */
#define warn_printk(args...)	/* printk(args) */
#define error_printk(args...)	/* printk(args) */
#define debug_printk(args...)	/* printk(args) */

int isp1763_otg_timer_start(otgtimer_callback_t cb, unsigned long timeout,
			    void *data);
void isp1763_otg_timer_cancel(void);
int isp1763_register_ctrl(struct isp1763_controller *ctrl, int role);
void isp1763_unregister_ctrl(int role);

#define static_role(x) ((x & ISP1763_FLAG_PORT1_ROLE_OTG) \
			!= ISP1763_FLAG_PORT1_ROLE_OTG)

#define static_role_host(x) ((x & ISP1763_FLAG_PORT1_ROLE_OTG) \
			== ISP1763_FLAG_PORT1_ROLE_HOST)

#endif /* ! __ISP1763_H__ */
