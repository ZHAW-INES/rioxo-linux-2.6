/*
 *  linux/arch/niosnommu/kernel/setup.c
 *
 *  Copyright (C) 2004       Microtronix Datacom Ltd.
 *  Copyright (C) 2001       Vic Phillips {vic@microtronix.com}
 *  Copyleft  (C) 2000       James D. Schettine {james@telos-systems.com}
 *  Copyright (C) 1999       Greg Ungerer (gerg@moreton.com.au)
 *  Copyright (C) 1998,2000  D. Jeff Dionne <jeff@lineo.ca>
 *  Copyright (C) 1998       Kenneth Albanowski <kjahds@kjahds.com>
 *  Copyright (C) 1995       Hamish Macdonald
 *
 * All rights reserved.          
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * This file handles the architecture-dependent parts of system setup
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>
#include <linux/mii.h>
#include <linux/phy.h>

#include <asm/irq.h>
#include <asm/byteorder.h>
#include <asm/asm-offsets.h>
#include <asm/pgtable.h>

#ifdef CONFIG_BLK_DEV_INITRD
#include <linux/blkdev.h>
#endif

#ifdef CONFIG_NIOS_SPI
#include <asm/spi.h>
extern ssize_t spi_write(struct file *filp, const char *buf, size_t count, loff_t *ppos);
extern ssize_t spi_read (struct file *filp, char *buf, size_t count, loff_t *ppos);
extern loff_t spi_lseek (struct file *filp, loff_t offset, int origin);
extern int spi_open     (struct inode *inode, struct file *filp);
extern int spi_release  (struct inode *inode, struct file *filp);
#endif

#ifdef CONFIG_CONSOLE
extern struct consw *conswitchp;
#endif

unsigned long rom_length;
unsigned long memory_start;
unsigned long memory_end;

EXPORT_SYMBOL(memory_start);
EXPORT_SYMBOL(memory_end);

#ifndef CONFIG_PASS_CMDLINE
static char default_command_line[] = CONFIG_CMDLINE;
#endif
static char __initdata command_line[COMMAND_LINE_SIZE] = { 0, };


/*				   r1  r2  r3  r4  r5  r6  r7  r8  r9 r10 r11*/
/*				   r12 r13 r14 r15 or2                      ra  fp  sp  gp es  ste  ea*/
static struct pt_regs fake_regs = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,\
				    0,  0,  0,  0,  0, (unsigned long)cpu_idle,  0,  0,  0, 0,   0,  0};

#define CPU "NIOS2"

// save args passed from u-boot, called from head.S
void nios2_boot_init(unsigned r4,unsigned r5,unsigned r6,unsigned r7)
{
#if defined(CONFIG_PASS_CMDLINE)
  if (r4 == 0x534f494e)   // r4 is magic NIOS, to become board info check in the future
    {
#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r5) {
		initrd_start = r5;
		initrd_end = r6;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */
	if (r7)
		strncpy(command_line, (char *)r7, COMMAND_LINE_SIZE);
    }
#endif
}

void __init setup_arch(char **cmdline_p)
{
	int bootmap_size;
	extern int _stext, _etext;
	extern int _edata, _end;
#ifdef DEBUG
	extern int _sdata, _sbss, _ebss;
#ifdef CONFIG_BLK_DEV_BLKMEM
	extern int *romarray;
#endif
#endif

	memory_start = PAGE_ALIGN((unsigned long)&_end);
	memory_end = (unsigned long) nasys_program_mem_end;

#ifndef CONFIG_PASS_CMDLINE
		memcpy(command_line, default_command_line, sizeof(default_command_line));
#endif

	printk("uClinux/Nios II\n");

#ifdef DEBUG
	printk("KERNEL -> TEXT=0x%08x-0x%08x DATA=0x%08x-0x%08x "
		"BSS=0x%08x-0x%08x\n", (int) &_stext, (int) &_etext,
		(int) &_sdata, (int) &_edata,
		(int) &_sbss, (int) &_ebss);
	printk("KERNEL -> MEM=0x%06x-0x%06x STACK=0x%06x-0x%06x\n",
		(int) memory_start, (int) memory_end,
		(int) memory_end, (int) nasys_program_mem_end);
#endif

	init_mm.start_code = (unsigned long) &_stext;
	init_mm.end_code = (unsigned long) &_etext;
	init_mm.end_data = (unsigned long) &_edata;
	init_mm.brk = (unsigned long) 0;
	init_task.thread.kregs = &fake_regs;

	/* Keep a copy of command line */
	*cmdline_p = &command_line[0];

	memcpy(boot_command_line, command_line, COMMAND_LINE_SIZE);
	boot_command_line[COMMAND_LINE_SIZE-1] = 0;

#ifdef DEBUG
	if (strlen(*cmdline_p))
		printk("Command line: '%s'\n", *cmdline_p);
	else
		printk("No Command line passed\n");
#endif

	/*
	 * give all the memory to the bootmap allocator,  tell it to put the
	 * boot mem_map at the start of memory
	 */
	bootmap_size = init_bootmem_node(
			NODE_DATA(0),
			memory_start >> PAGE_SHIFT, /* map goes here */
			PAGE_OFFSET >> PAGE_SHIFT,	/* 0 on coldfire */
			memory_end >> PAGE_SHIFT);
	/*
	 * free the usable memory,  we have to make sure we do not free
	 * the bootmem bitmap so we then reserve it after freeing it :-)
	 */
	free_bootmem(memory_start, memory_end - memory_start);
	reserve_bootmem(memory_start, bootmap_size, BOOTMEM_DEFAULT);
#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		reserve_bootmem(virt_to_phys((void *)initrd_start),
			initrd_end - initrd_start, BOOTMEM_DEFAULT);
#endif /* CONFIG_BLK_DEV_INITRD */
	/*
	 * get kmalloc into gear
	 */
	paging_init();
#ifdef CONFIG_VT
#if defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
#endif

#ifdef DEBUG
	printk("Done setup_arch\n");
#endif

}

int get_cpuinfo(char * buffer)
{
    char *cpu, *mmu, *fpu;
    u_long clockfreq;

    cpu = CPU;
    mmu = "none";
    fpu = "none";

    clockfreq = na_cpu_clock_freq;

    return(sprintf(buffer, "CPU:\t\t%s\n"
		   "MMU:\t\t%s\n"
		   "FPU:\t\t%s\n"
		   "Clocking:\t%lu.%1luMHz\n"
		   "BogoMips:\t%lu.%02lu\n"
		   "Calibration:\t%lu loops\n",
		   cpu, mmu, fpu,
		   clockfreq/1000000,(clockfreq/100000)%10,
		   (loops_per_jiffy*HZ)/500000,((loops_per_jiffy*HZ)/5000)%100,
		   (loops_per_jiffy*HZ)));

}

/*
 *	Get CPU information for use by the procfs.
 */

static int show_cpuinfo(struct seq_file *m, void *v)
{
    char *cpu, *mmu, *fpu;
    u_long clockfreq;

    cpu = CPU;
    mmu = "none";
    fpu = "none";

    clockfreq = na_cpu_clock_freq;

    seq_printf(m, "CPU:\t\t%s\n"
		   "MMU:\t\t%s\n"
		   "FPU:\t\t%s\n"
		   "Clocking:\t%lu.%1luMHz\n"
		   "BogoMips:\t%lu.%02lu\n"
		   "Calibration:\t%lu loops\n",
		   cpu, mmu, fpu,
		   clockfreq/1000000,(clockfreq/100000)%10,
		   (loops_per_jiffy*HZ)/500000,((loops_per_jiffy*HZ)/5000)%100,
		   (loops_per_jiffy*HZ));

	return 0;
}

#ifdef CONFIG_NIOS_SPI

static int bcd2char( int x )
{
        if ( (x & 0xF) > 0x90 || (x & 0x0F) > 0x09 )
                return 99;

        return (((x & 0xF0) >> 4) * 10) + (x & 0x0F);
}

#endif // CONFIG_NIOS_SPI


void arch_gettod(int *year, int *month, int *date, int *hour, int *min, int *sec)
{
#ifdef CONFIG_NIOS_SPI
        /********************************************************************/
  	/* Read the CMOS clock on the Microtronix Datacom O/S Support card. */
  	/* Use the SPI driver code, but circumvent the file system by using */
        /* its internal functions.                                          */
        /********************************************************************/
        int  hr;

	struct                               /*********************************/
        {                                    /* The SPI payload. Warning: the */
	      unsigned short register_addr;  /* sizeof() operator will return */
	      unsigned char  value;          /* a length of 4 instead of 3!   */
        } spi_data;                          /*********************************/


	if ( spi_open( NULL, NULL ) )
	{
	    printk( "Cannot open SPI driver to read system CMOS clock.\n" );
	    *year = *month = *date = *hour = *min = *sec = 0;
	    return;
	}

	spi_lseek( NULL, clockCS, 0 /* == SEEK_SET */ );

	spi_data.register_addr = clock_write_control;
	spi_data.value         = 0x40; // Write protect
	spi_write( NULL, (const char *)&spi_data, 3, NULL  );

	spi_data.register_addr = clock_read_sec;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL );
	*sec = (int)bcd2char( spi_data.value );

	spi_data.register_addr = clock_read_min;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL  );
	*min = (int)bcd2char( spi_data.value );

	spi_data.register_addr = clock_read_hour;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL  );
	hr = (int)bcd2char( spi_data.value );
	if ( hr & 0x40 )  // Check 24-hr bit
 	    hr = (hr & 0x3F) + 12;     // Convert to 24-hr

	*hour = hr;



	spi_data.register_addr = clock_read_date;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL  );
	*date = (int)bcd2char( spi_data.value );

	spi_data.register_addr = clock_read_month;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL  );
	*month = (int)bcd2char( spi_data.value );

	spi_data.register_addr = clock_read_year;
	spi_data.value         = 0;
	spi_read( NULL, (char *)&spi_data, 3, NULL  );
	*year = (int)bcd2char( spi_data.value );


	spi_release( NULL, NULL );
#else
	*year = *month = *date = *hour = *min = *sec = 0;

#endif
}

static void *cpuinfo_start (struct seq_file *m, loff_t *pos)
{
	return *pos < NR_CPUS ? ((void *) 0x12345678) : NULL;
}

static void *cpuinfo_next (struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return cpuinfo_start (m, pos);
}

static void cpuinfo_stop (struct seq_file *m, void *v)
{
}

const struct seq_operations cpuinfo_op = {
	.start	= cpuinfo_start,
	.next	= cpuinfo_next,
	.stop	= cpuinfo_stop,
	.show	= show_cpuinfo
};
