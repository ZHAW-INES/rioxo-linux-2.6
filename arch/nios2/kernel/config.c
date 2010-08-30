/*
 * Adjusted specifically for HD over IP board to only contain the stuff for the
 * hardware actually present
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c-ocores.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/altera_jtaguart.h>
#include <linux/altera_uart.h>

#if !defined(CONFIG_INES_HDOIP_EVA)
# error "This kernel only builds for HD over IP Eva board"
#endif

/* FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU */
#ifndef na_epcs_controller
#define na_epcs_controller na_epcs_flash_controller
#endif

#ifndef na_epcs_controller_irq
#define na_epcs_controller_irq na_epcs_flash_controller_irq
#endif

/*
 *	Altera JTAG UART
 */

#if (defined(CONFIG_SERIAL_ALTERA_JTAGUART) || defined(CONFIG_SERIAL_ALTERA_JTAGUART_MODULE)) && defined(na_jtag_uart)
static struct altera_jtaguart_platform_uart nios2_jtaguart_platform[] = {
#ifdef na_jtag_uart
	{
		.mapbase = (unsigned long) na_jtag_uart,
		.irq = na_jtag_uart_irq,
	},
#endif
	{},
};

static struct platform_device nios2_jtaguart = {
	.name = "altera_jtaguart",
	.id = 0,
	.dev.platform_data = nios2_jtaguart_platform,
};
#endif

/*
 *	Altera UART
 */

#if (defined(CONFIG_SERIAL_ALTERA_UART) || defined(CONFIG_SERIAL_ALTERA_UART_MODULE)) && defined(na_uart0)
static struct altera_uart_platform_uart nios2_uart_platform[] = {
#ifdef na_uart0
	{
		.mapbase = (unsigned long) na_uart0,
		.irq = na_uart0_irq,
		.uartclk = nasys_clock_freq,
	},
#endif
	{},
};

static struct platform_device nios2_uart = {
	.name = "altera_uart",
	.id = 0,
	.dev.platform_data = nios2_uart_platform,
};
#endif

/*
 *	MTD map, SPI/EPCS flash
 */

#if (defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
#define EPCS_SPI_OFFSET 0x400
static struct resource na_epcs_controller_resource[] = {
	[0] = {
		.start = na_epcs_controller + EPCS_SPI_OFFSET,
		.end = na_epcs_controller + EPCS_SPI_OFFSET + 31,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = na_epcs_controller_irq,
		.end = na_epcs_controller_irq,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device na_epcs_controller_device = {
	.name = "altspi",
	.id = 0,		/* Bus number */
	.num_resources = ARRAY_SIZE(na_epcs_controller_resource),
	.resource = na_epcs_controller_resource,
};
#endif /* spi master and devices */

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition nios2_spi_flash_partitions[] = {
	{
		.name = "fpga configuration",
		.size = 0x400000,
		.offset = 0,
	},
	{
		.name = "kernel/rootfs",
		.size = 0xb00000,
		.offset = 0x400000,
	},
	{
		.name = "config",
		.size = 0x100000,
		.offset = 0xf00000,
	},
};

static struct flash_platform_data nios2_spi_flash_data = {
	.name = "m25p80",
	.parts = nios2_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(nios2_spi_flash_partitions),
	.type = "m25p128",	/* depend on the actual size of spi flash */
};
#endif

/*
 *	Altera SPI
 */

#if defined(CONFIG_SPI) || defined(CONFIG_SPI_MODULE)
static struct spi_board_info nios2_spi_devices[] = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80",		/* Name of spi_driver for this device */
		.max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
		.bus_num = 0,			/* bus number */
		.chip_select = 0,
		.platform_data = &nios2_spi_flash_data,
	},
#endif
};
#endif

/*
 *	Opencores I2C
 */

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(I2C_0_BASE)
static struct resource i2c_oc_0_resources[] = {
	[0] = {
		.start = I2C_0_BASE,
		.end = I2C_0_BASE + 31,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = I2C_0_IRQ,
		.end = I2C_0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ocores_i2c_platform_data i2c_oc_0_platform_data = {
	.regstep = 4,		/* four bytes between registers */
	.clock_khz = I2C_0_FREQ / 1000,	/* input clock */
};

static struct platform_device i2c_oc_0_device = {
	.name = "ocores-i2c",
	.id = 0,
	.dev = {
		.platform_data = &i2c_oc_0_platform_data,
		},
	.num_resources = ARRAY_SIZE(i2c_oc_0_resources),
	.resource = i2c_oc_0_resources,
};
#endif

/*
 *	Altera Remote update
 */

#if defined(CONFIG_ALTERA_REMOTE_UPDATE) && defined(na_altremote)
static struct resource altremote_resources[] = {
	[0] = {
		.start	= na_altremote,
		.end	= na_altremote + 0x200 - 1,
		.flags	= IORESOURCE_MEM,
	},
};
static struct platform_device altremote_device = {
	.name		= "altremote",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(altremote_resources),
	.resource	= altremote_resources,
};
#endif

/* HD over IP hardware components */

/*
 *	Ethernet, Altera TSE
 */

static struct platform_device hdoip_ether_device = {
	.name           = "hdoip_ether",
	.id             = 0,
	.num_resources  = 0,
	.resource       = NULL,
	.dev            = {
		.platform_data = NULL,
	},
};

/*
 *	Nios2 platform devices
 */

static struct platform_device *nios2_devices[] __initdata = {

#if (defined(CONFIG_SERIAL_ALTERA_JTAGUART) || defined(CONFIG_SERIAL_ALTERA_JTAGUART_MODULE)) && defined(na_jtag_uart)
	&nios2_jtaguart,
#endif

#if (defined(CONFIG_SERIAL_ALTERA_UART) || defined(CONFIG_SERIAL_ALTERA_UART_MODULE)) && defined(na_uart0)
	&nios2_uart,
#endif

#if (defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
	&na_epcs_controller_device,
#endif

#if defined(CONFIG_ALTERA_REMOTE_UPDATE) && defined(na_altremote)
	&altremote_device,
#endif

#if defined (CONFIG_INES_HDOIP_EVA)
	&hdoip_ether_device,
#endif
};

static int __init init_BSP(void)
{
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));

#if defined(CONFIG_SPI) || defined(CONFIG_SPI_MODULE)
	spi_register_board_info(nios2_spi_devices,
				ARRAY_SIZE(nios2_spi_devices));
#endif
	return 0;
}

arch_initcall(init_BSP);
