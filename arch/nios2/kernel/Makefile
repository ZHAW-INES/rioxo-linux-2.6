#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now in the main makefile...

extra-y := head.o init_task.o vmlinux.lds

obj-y   := entry.o traps.o irq.o syscalltable.o \
	    process.o signal.o setup.o sys_nios2.o \
	    usb.o config.o dma-mapping.o \
	    time.o ptrace.o nios2_ksyms.o

obj-$(CONFIG_MODULES) += module.o
obj-$(CONFIG_AVALON_DMA) += dma.o

obj-y += hidg.o
