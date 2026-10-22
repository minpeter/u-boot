# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
#
# Based on some other board Makefile
#
# (C) Copyright 2000-2003
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
obj-y	+= board.o
obj-$(CONFIG_H713_POWER_GATE_IR)	+= h713_gate_ir.o
obj-$(CONFIG_H713_VENDOR_CHAINLOAD)	+= h713_vendor_chain.o \
					   h713_vendor_chain_tramp.o
obj-$(CONFIG_SUN7I_GMAC)	+= gmac.o
obj-$(CONFIG_MACH_SUN4I)	+= dram_sun4i_auto.o
obj-$(CONFIG_MACH_SUN5I)	+= dram_sun5i_auto.o
obj-$(CONFIG_MACH_SUN7I)	+= dram_sun5i_auto.o
obj-$(CONFIG_$(PHASE_)CHIP_DIP_SCAN)	+= chip.o
