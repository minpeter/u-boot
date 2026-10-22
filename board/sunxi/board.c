// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 * (C) Copyright 2013 Luke Kenneth Casson Leighton <lkcl@lkcl.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Some board init for the Allwinner A10-evb board.
 */

#include <clock_legacy.h>
#include <dm.h>
#include <env.h>
#include <fastboot.h>
#include <hang.h>
#include <i2c.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <mmc.h>
#include <axp_pmic.h>
#include <generic-phy.h>
#include <phy-sun4i-usb.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/display.h>
#include <asm/arch/dram.h>
#include <asm/arch/mmc.h>
#include <asm/arch/prcm.h>
#include <asm/arch/pmic_bus.h>
#include <asm/arch/spl.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/types.h>
#ifndef CONFIG_ARM64
#include <asm/armv7.h>
#endif
#include <asm/gpio.h>
#include <sunxi_gpio.h>
#include <asm/io.h>
#include <asm/system.h>
#include <u-boot/crc.h>
#include <env_internal.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <nand.h>
#include <net.h>
#include <spl.h>
#include <sy8106a.h>
#include <time.h>
#include <usb.h>
#include <watchdog.h>
#include <asm/setup.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_FASTBOOT) && defined(CONFIG_MACH_SUN50I_H713)
#define H713_RTC_GP7_REG		0x0709011cUL
#define H713_REBOOT_BOOTLOADER_MAGIC	0xb007c0de

int fastboot_set_reboot_flag_board(enum fastboot_reboot_reason reason)
{
	if (reason != FASTBOOT_REBOOT_REASON_BOOTLOADER)
		return -ENOSYS;

	/*
	 * Reuse Linux's nvmem-reboot-mode handoff. RTC GP7 survives the PSCI
	 * watchdog reset, and H713 preboot consumes and clears this magic before
	 * dropping to the U-Boot prompt.
	 */
	writel(H713_REBOOT_BOOTLOADER_MAGIC, H713_RTC_GP7_REG);

	return readl(H713_RTC_GP7_REG) == H713_REBOOT_BOOTLOADER_MAGIC ?
	       0 : -EIO;
}
#endif

void i2c_init_board(void)
{
#ifdef CONFIG_I2C0_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN5I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(0), SUN4I_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(1), SUN4I_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(14), SUN6I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(15), SUN6I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I_V3S)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(6), SUN8I_V3S_GPB_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(7), SUN8I_V3S_GPB_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN8I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN8I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_GPH_TWI0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_GPH_TWI0);
	clock_twi_onoff(0, 1);
#endif
#endif

#ifdef CONFIG_I2C1_ENABLE
#if defined(CONFIG_MACH_SUN4I) || \
    defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(18), SUN4I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(19), SUN4I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN5I)
	sunxi_gpio_set_cfgpin(SUNXI_GPB(15), SUN5I_GPB_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(16), SUN5I_GPB_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN6I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(16), SUN6I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(17), SUN6I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN8I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(4), SUN8I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(5), SUN8I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#elif defined(CONFIG_MACH_SUN50I)
	sunxi_gpio_set_cfgpin(SUNXI_GPH(2), SUN50I_GPH_TWI1);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(3), SUN50I_GPH_TWI1);
	clock_twi_onoff(1, 1);
#endif
#endif

#ifdef CONFIG_R_I2C_ENABLE
#ifdef CONFIG_MACH_SUN50I
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(8), SUN50I_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(9), SUN50I_GPL_R_TWI);
#elif defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_H713)
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN50I_H616_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN50I_H616_GPL_R_TWI);
#elif CONFIG_MACH_SUN55I_A523
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN50I_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN50I_GPL_R_TWI);
#else
	clock_twi_onoff(5, 1);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(0), SUN8I_H3_GPL_R_TWI);
	sunxi_gpio_set_cfgpin(SUNXI_GPL(1), SUN8I_H3_GPL_R_TWI);
#endif
#endif
}

/*
 * Try to use the environment from the boot source first.
 * For MMC, this means a FAT partition on the boot device (SD or eMMC).
 * If the raw MMC environment is also enabled, this is tried next.
 * When booting from NAND we try UBI first, then NAND directly.
 * SPI flash falls back to FAT (on SD card).
 */
enum env_location env_get_location(enum env_operation op, int prio)
{
	if (prio > 1)
		return ENVL_UNKNOWN;

	/* NOWHERE is exclusive, no other option can be defined. */
	if (IS_ENABLED(CONFIG_ENV_IS_NOWHERE))
		return ENVL_NOWHERE;

	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
			return ENVL_MMC;
		break;
	case BOOT_DEVICE_NAND:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_NAND))
			return ENVL_NAND;
		break;
	case BOOT_DEVICE_SPI:
		if (prio == 0 && IS_ENABLED(CONFIG_ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		break;
	case BOOT_DEVICE_BOARD:
		break;
	default:
		break;
	}

	/*
	 * If we come here for the first time, we *must* return a valid
	 * environment location other than ENVL_UNKNOWN, or the setup sequence
	 * in board_f() will silently hang. This is arguably a bug in
	 * env_init(), but for now pick one environment for which we know for
	 * sure to have a driver for. For all defconfigs this is either FAT
	 * or UBI, or NOWHERE, which is already handled above.
	 */
	if (prio == 0) {
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		/*
		 * Booting over FEL lands here: the SoC came up over USB, so
		 * there is no boot medium to derive a location from, and a
		 * defconfig keeping its environment solely in MMC matches
		 * none of the cases above. The on-board MMC is still present
		 * either way, so use it rather than hanging.
		 */
		if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
			return ENVL_MMC;
	}

	return ENVL_UNKNOWN;
}

int board_init(void)
{
	__maybe_unused int id_pfr1, ret;

	gd->bd->bi_boot_params = (PHYS_SDRAM_0 + 0x100);

#if !defined(CONFIG_ARM64) && !defined(CONFIG_MACH_SUNIV)
	asm volatile("mrc p15, 0, %0, c0, c1, 1" : "=r"(id_pfr1));
	debug("id_pfr1: 0x%08x\n", id_pfr1);
	/* Generic Timer Extension available? */
	if ((id_pfr1 >> CPUID_ARM_GENTIMER_SHIFT) & 0xf) {
		uint32_t freq;

		debug("Setting CNTFRQ\n");

		/*
		 * CNTFRQ is a secure register, so we will crash if we try to
		 * write this from the non-secure world (read is OK, though).
		 * In case some bootcode has already set the correct value,
		 * we avoid the risk of writing to it.
		 */
		asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(freq));
		if (freq != CONFIG_COUNTER_FREQUENCY) {
			debug("arch timer frequency is %d Hz, should be %d, fixing ...\n",
			      freq, CONFIG_COUNTER_FREQUENCY);
#ifdef CONFIG_NON_SECURE
			printf("arch timer frequency is wrong, but cannot adjust it\n");
#else
			asm volatile("mcr p15, 0, %0, c14, c0, 0"
				     : : "r"(CONFIG_COUNTER_FREQUENCY));
#endif
		}
	}
#endif /* !CONFIG_ARM64 && !CONFIG_MACH_SUNIV */

	ret = axp_gpio_init();
	if (ret)
		return ret;


	eth_init_board();

	return 0;
}

/*
 * On older SoCs the SPL is actually at address zero, so using NULL as
 * an error value does not work.
 */
#define INVALID_SPL_HEADER ((void *)~0UL)

static struct boot_file_head * get_spl_header(uint8_t req_version)
{
	struct boot_file_head *spl = (void *)(ulong)SPL_ADDR;
	uint8_t spl_header_version = spl->spl_signature[3];

	/* Is there really the SPL header (still) there? */
	if (memcmp(spl->spl_signature, SPL_SIGNATURE, 3) != 0)
		return INVALID_SPL_HEADER;

	if (spl_header_version < req_version) {
		printf("sunxi SPL version mismatch: expected %u, got %u\n",
		       req_version, spl_header_version);
		return INVALID_SPL_HEADER;
	}

	return spl;
}

static const char *get_spl_dt_name(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	/* Check if there is a DT name stored in the SPL header. */
	if (spl != INVALID_SPL_HEADER && spl->dt_name_offset)
		return (char *)spl + spl->dt_name_offset;

	return NULL;
}

int dram_init(void)
{
	struct boot_file_head *spl = get_spl_header(SPL_DRAM_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		gd->ram_size = get_ram_size((long *)PHYS_SDRAM_0,
					    PHYS_SDRAM_0_SIZE);
	else
		gd->ram_size = (phys_addr_t)spl->dram_size << 20;

	if (gd->ram_size > CONFIG_SUNXI_DRAM_MAX_SIZE)
		gd->ram_size = CONFIG_SUNXI_DRAM_MAX_SIZE;

	return 0;
}

#if defined(CONFIG_NAND_SUNXI) && defined(CONFIG_XPL_BUILD)
static void nand_pinmux_setup(void)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(19); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);

#if defined CONFIG_MACH_SUN4I || defined CONFIG_MACH_SUN7I
	for (pin = SUNXI_GPC(20); pin <= SUNXI_GPC(22); pin++)
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_NAND);
#endif
	/* sun4i / sun7i do have a PC23, but it is not used for nand,
	 * only sun7i has a PC24 */
#ifdef CONFIG_MACH_SUN7I
	sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_NAND);
#endif
}

static void nand_clock_setup(void)
{
	void * const ccm = (void *)SUNXI_CCM_BASE;

#if defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_H713) || \
    defined(CONFIG_MACH_SUN50I_H6)
	setbits_le32(ccm + CCU_H6_NAND_GATE_RESET,
		     (1 << GATE_SHIFT) | (1 << RESET_SHIFT));
	setbits_le32(ccm + CCU_H6_MBUS_GATE, (1 << MBUS_GATE_OFFSET_NAND));
	setbits_le32(ccm + CCU_NAND1_CLK_CFG, CCM_NAND_CTRL_ENABLE |
		     CCM_NAND_CTRL_N(0) | CCM_NAND_CTRL_M(1));
#else
	setbits_le32(ccm + CCU_AHB_GATE0,
		     (CLK_GATE_OPEN << AHB_GATE_OFFSET_NAND0));
#if defined CONFIG_MACH_SUN6I || defined CONFIG_MACH_SUN8I || \
    defined CONFIG_MACH_SUN9I || defined CONFIG_MACH_SUN50I
	setbits_le32(ccm + CCU_AHB_RESET0_CFG, (1 << AHB_GATE_OFFSET_NAND0));
#endif
#endif
	setbits_le32(ccm + CCU_NAND0_CLK_CFG, CCM_NAND_CTRL_ENABLE |
		     CCM_NAND_CTRL_N(0) | CCM_NAND_CTRL_M(1));
}

void board_nand_init(void)
{
	nand_pinmux_setup();
	nand_clock_setup();
}
#endif /* CONFIG_NAND_SUNXI */

#ifdef CONFIG_MMC
static void mmc_pinmux_setup(int sdc)
{
	unsigned int pin;

	switch (sdc) {
	case 0:
		/* SDC0: PF0-PF5 */
		for (pin = SUNXI_GPF(0); pin <= SUNXI_GPF(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPF_SDC0);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
		break;

	case 1:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		if (IS_ENABLED(CONFIG_MMC1_PINS_PH)) {
			/* SDC1: PH22-PH-27 */
			for (pin = SUNXI_GPH(22); pin <= SUNXI_GPH(27); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPH_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		} else {
			/* SDC1: PG0-PG5 */
			for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
				sunxi_gpio_set_cfgpin(pin, SUN4I_GPG_SDC1);
				sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
				sunxi_gpio_set_drv(pin, 2);
			}
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC1: PG3-PG8 */
		for (pin = SUNXI_GPG(3); pin <= SUNXI_GPG(8); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN5I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN8I)
		/* SDC1: PG0-PG5 */
		for (pin = SUNXI_GPG(0); pin <= SUNXI_GPG(5); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN8I_GPG_SDC1);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#endif
		break;

	case 2:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I)
		/* SDC2: PC6-PC11 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(11); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN5I)
		/* SDC2: PC6-PC15 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I_R40)
		/* SDC2: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUNXI_GPC_SDC2);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#elif defined(CONFIG_MACH_SUN8I) || defined(CONFIG_MACH_SUN50I)
		/* SDC2: PC5-PC6, PC8-PC16 */
		for (pin = SUNXI_GPC(5); pin <= SUNXI_GPC(6); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		for (pin = SUNXI_GPC(8); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H6)
		/* SDC2: PC4-PC14 */
		for (pin = SUNXI_GPC(4); pin <= SUNXI_GPC(14); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN50I_H616) || defined(CONFIG_MACH_SUN50I_H713) || \
      defined(CONFIG_MACH_SUN50I_A133) || defined(CONFIG_MACH_SUN55I_A523)
		/* SDC2: PC0-PC1, PC5-PC6, PC8-PC11, PC13-PC16 */
		for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(16); pin++) {
			if (pin > SUNXI_GPC(1) && pin < SUNXI_GPC(5))
				continue;
			if (pin == SUNXI_GPC(7) || pin == SUNXI_GPC(12))
				continue;
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 3);
		}
#elif defined(CONFIG_MACH_SUN9I)
		/* SDC2: PC6-PC16 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(16); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN8I_R528)
                /* SDC2: PC2-PC7 */
                for (pin = SUNXI_GPC(2); pin <= SUNXI_GPC(7); pin++) {
                        sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SDC2);
                        sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
                        sunxi_gpio_set_drv(pin, 2);
                }
#else
		puts("ERROR: No pinmux setup defined for MMC2!\n");
#endif
		break;

	case 3:
#if defined(CONFIG_MACH_SUN4I) || defined(CONFIG_MACH_SUN7I) || \
    defined(CONFIG_MACH_SUN8I_R40)
		/* SDC3: PI4-PI9 */
		for (pin = SUNXI_GPI(4); pin <= SUNXI_GPI(9); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUNXI_GPI_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}
#elif defined(CONFIG_MACH_SUN6I)
		/* SDC3: PC6-PC15, PC24 */
		for (pin = SUNXI_GPC(6); pin <= SUNXI_GPC(15); pin++) {
			sunxi_gpio_set_cfgpin(pin, SUN6I_GPC_SDC3);
			sunxi_gpio_set_pull(pin, SUNXI_GPIO_PULL_UP);
			sunxi_gpio_set_drv(pin, 2);
		}

		sunxi_gpio_set_cfgpin(SUNXI_GPC(24), SUN6I_GPC_SDC3);
		sunxi_gpio_set_pull(SUNXI_GPC(24), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_drv(SUNXI_GPC(24), 2);
#endif
		break;

	default:
		printf("sunxi: invalid MMC slot %d for pinmux setup\n", sdc);
		break;
	}
}

int board_mmc_init(struct bd_info *bis)
{
	/*
	 * The BROM always accesses MMC port 0 (typically an SD card), and
	 * most boards seem to have such a slot. The others haven't reported
	 * any problem with unconditionally enabling this in the SPL.
	 */
	if (!IS_ENABLED(CONFIG_UART0_PORT_F)) {
		mmc_pinmux_setup(0);
		if (!sunxi_mmc_init(0))
			return -1;
	}

	if (CONFIG_MMC_SUNXI_SLOT_EXTRA != -1) {
		mmc_pinmux_setup(CONFIG_MMC_SUNXI_SLOT_EXTRA);
		if (!sunxi_mmc_init(CONFIG_MMC_SUNXI_SLOT_EXTRA))
			return -1;
	}

	return 0;
}

#ifdef CONFIG_H713_VENDOR_CHAINLOAD
/*
 * The chainloader runs in board_init_f, before the block layer is up, so it
 * cannot reach the eMMC through find_mmc_device().  Hand it the controller
 * directly.  Slot 2 is the eMMC on this board; slot 0 is the absent SD socket.
 */
struct mmc *h713_chain_emmc(void)
{
	mmc_pinmux_setup(2);
	return sunxi_mmc_init(2);
}
#endif

#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
int mmc_get_env_dev(void)
{
	switch (sunxi_get_boot_device()) {
	case BOOT_DEVICE_MMC1:
		return 0;
	case BOOT_DEVICE_MMC2:
		return 1;
	default:
		return CONFIG_ENV_MMC_DEVICE_INDEX;
	}
}
#endif
#endif /* CONFIG_MMC */

#ifdef CONFIG_XPL_BUILD

static void sunxi_spl_store_dram_size(phys_addr_t dram_size)
{
	struct boot_file_head *spl = get_spl_header(SPL_DT_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DRAM_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DRAM_HEADER_VERSION;

	spl->dram_size = dram_size >> 20;
}

static void status_led_init(void)
{
#if CONFIG_IS_ENABLED(SUNXI_LED_STATUS)
	unsigned int state = IS_ENABLED(CONFIG_SPL_SUNXI_LED_STATUS_ACTIVE_HIGH);
	unsigned int gpio = CONFIG_SPL_SUNXI_LED_STATUS_GPIO;

	gpio_request(gpio, "gpio_led");
	gpio_direction_output(gpio, state);
#endif
}

void sunxi_board_init(void)
{
	int power_failed = 0;

	if (CONFIG_IS_ENABLED(SUNXI_LED_STATUS))
		status_led_init();

#ifdef CONFIG_SY8106A_POWER
	power_failed = sy8106a_set_vout1(CONFIG_SY8106A_VOUT1_VOLT);
#endif

#if defined CONFIG_AXP152_POWER || defined CONFIG_AXP209_POWER || \
	defined CONFIG_AXP221_POWER || defined CONFIG_AXP305_POWER || \
	defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER || \
	defined CONFIG_AXP313_POWER || defined CONFIG_AXP717_POWER || \
	defined CONFIG_AXP803_POWER
	power_failed = axp_init();

	if (IS_ENABLED(CONFIG_AXP_DISABLE_BOOT_ON_POWERON) && !power_failed) {
		u8 boot_reason;

		pmic_bus_read(AXP_POWER_STATUS, &boot_reason);
		if (boot_reason & AXP_POWER_STATUS_ALDO_IN) {
			printf("Power on by plug-in, shutting down.\n");
			pmic_bus_write(0x32, BIT(7));
		}
	}

#ifdef CONFIG_AXP_DCDC1_VOLT
	power_failed |= axp_set_dcdc1(CONFIG_AXP_DCDC1_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC2_VOLT
	power_failed |= axp_set_dcdc2(CONFIG_AXP_DCDC2_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC3_VOLT
	power_failed |= axp_set_dcdc3(CONFIG_AXP_DCDC3_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC4_VOLT
	power_failed |= axp_set_dcdc4(CONFIG_AXP_DCDC4_VOLT);
#endif
#ifdef CONFIG_AXP_DCDC5_VOLT
	power_failed |= axp_set_dcdc5(CONFIG_AXP_DCDC5_VOLT);
#endif

#ifdef CONFIG_AXP_ALDO1_VOLT
	power_failed |= axp_set_aldo1(CONFIG_AXP_ALDO1_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO2_VOLT
	power_failed |= axp_set_aldo2(CONFIG_AXP_ALDO2_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO3_VOLT
	power_failed |= axp_set_aldo3(CONFIG_AXP_ALDO3_VOLT);
#endif
#ifdef CONFIG_AXP_ALDO4_VOLT
	power_failed |= axp_set_aldo4(CONFIG_AXP_ALDO4_VOLT);
#endif

#ifdef CONFIG_AXP_DLDO1_VOLT
	power_failed |= axp_set_dldo(1, CONFIG_AXP_DLDO1_VOLT);
	power_failed |= axp_set_dldo(2, CONFIG_AXP_DLDO2_VOLT);
#endif
#ifdef CONFIG_AXP_DLDO3_VOLT
	power_failed |= axp_set_dldo(3, CONFIG_AXP_DLDO3_VOLT);
	power_failed |= axp_set_dldo(4, CONFIG_AXP_DLDO4_VOLT);
#endif
#ifdef CONFIG_AXP_ELDO1_VOLT
	power_failed |= axp_set_eldo(1, CONFIG_AXP_ELDO1_VOLT);
	power_failed |= axp_set_eldo(2, CONFIG_AXP_ELDO2_VOLT);
	power_failed |= axp_set_eldo(3, CONFIG_AXP_ELDO3_VOLT);
#endif

#ifdef CONFIG_AXP_FLDO1_VOLT
	power_failed |= axp_set_fldo(1, CONFIG_AXP_FLDO1_VOLT);
	power_failed |= axp_set_fldo(2, CONFIG_AXP_FLDO2_VOLT);
	power_failed |= axp_set_fldo(3, CONFIG_AXP_FLDO3_VOLT);
#endif

#if defined CONFIG_AXP809_POWER || defined CONFIG_AXP818_POWER
	power_failed |= axp_set_sw(IS_ENABLED(CONFIG_AXP_SW_ON));
#endif
#endif	/* CONFIG_AXPxxx_POWER */
	printf("DRAM:");
	gd->ram_size = sunxi_dram_init();
	printf(" %d MiB\n", (int)(gd->ram_size >> 20));
	if (!gd->ram_size)
		hang();

	sunxi_spl_store_dram_size(gd->ram_size);

	/*
	 * Only clock up the CPU to full speed if we are reasonably
	 * assured it's being powered with suitable core voltage
	 */
	if (!power_failed)
		clock_set_pll1(get_board_sys_clk());
	else
		printf("Failed to set core voltage! Can't set CPU frequency\n");
}
#endif /* CONFIG_XPL_BUILD */

#ifdef CONFIG_USB_GADGET
int board_usb_init(int index, enum usb_init_type init)
{
	struct udevice *dev;

	if (init != USB_INIT_DEVICE)
		return 0;

	return uclass_get_device(UCLASS_USB_GADGET_GENERIC, index, &dev);
}

int g_dnl_board_usb_cable_connected(void)
{
	struct udevice *dev;
	struct phy phy;
	int ret;

	ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
	if (ret) {
		pr_err("%s: Cannot find USB device\n", __func__);
		return ret;
	}

	ret = generic_phy_get_by_name(dev, "usb", &phy);
	if (ret) {
		pr_err("failed to get %s USB PHY\n", dev->name);
		return ret;
	}

	ret = generic_phy_init(&phy);
	if (ret) {
		pr_debug("failed to init %s USB PHY\n", dev->name);
		return ret;
	}

	return sun4i_usb_phy_vbus_detect(&phy);
}
#endif /* CONFIG_USB_GADGET */

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	char *serial_string;
	unsigned long long serial;

	serial_string = env_get("serial#");

	if (serial_string) {
		serial = simple_strtoull(serial_string, NULL, 16);

		serialnr->high = (unsigned int) (serial >> 32);
		serialnr->low = (unsigned int) (serial & 0xffffffff);
	} else {
		serialnr->high = 0;
		serialnr->low = 0;
	}
}
#endif

/*
 * Check the SPL header for the "sunxi" variant. If found: parse values
 * that might have been passed by the loader ("fel" utility), and update
 * the environment accordingly.
 */
static void parse_spl_header(const uint32_t spl_addr)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	if (!spl->fel_script_address)
		return;

	if (spl->fel_uEnv_length != 0) {
		/*
		 * data is expected in uEnv.txt compatible format, so "env
		 * import -t" the string(s) at fel_script_address right away.
		 */
		himport_r(&env_htab, (char *)(uintptr_t)spl->fel_script_address,
			  spl->fel_uEnv_length, '\n', H_NOCLEAR, 0, 0, NULL);
		return;
	}
	/* otherwise assume .scr format (mkimage-type script) */
	env_set_hex("fel_scriptaddr", spl->fel_script_address);
}

static bool get_unique_sid(unsigned int *sid)
{
	if (sunxi_get_sid(sid) != 0)
		return false;

	if (!sid[0])
		return false;

	/*
	 * The single words 1 - 3 of the SID have quite a few bits
	 * which are the same on many models, so we take a crc32
	 * of all 3 words, to get a more unique value.
	 *
	 * Note we only do this on newer SoCs as we cannot change
	 * the algorithm on older SoCs since those have been using
	 * fixed mac-addresses based on only using word 3 for a
	 * long time and changing a fixed mac-address with an
	 * u-boot update is not good.
	 */
#if !defined(CONFIG_MACH_SUN4I) && !defined(CONFIG_MACH_SUN5I) && \
    !defined(CONFIG_MACH_SUN6I) && !defined(CONFIG_MACH_SUN7I) && \
    !defined(CONFIG_MACH_SUN8I_A23) && !defined(CONFIG_MACH_SUN8I_A33)
	sid[3] = crc32(0, (unsigned char *)&sid[1], 12);
#endif

	/* Ensure the NIC specific bytes of the mac are not all 0 */
	if ((sid[3] & 0xffffff) == 0)
		sid[3] |= 0x800000;

	return true;
}

/*
 * Note this function gets called multiple times.
 * It must not make any changes to env variables which already exist.
 */
static void setup_environment(const void *fdt)
{
	char serial_string[17] = { 0 };
	unsigned int sid[4];
	uint8_t mac_addr[6];
	char ethaddr[16];
	int i;

	if (!get_unique_sid(sid))
		return;

	for (i = 0; i < 4; i++) {
		sprintf(ethaddr, "ethernet%d", i);
		if (!fdt_get_alias(fdt, ethaddr))
			continue;

		if (i == 0)
			strcpy(ethaddr, "ethaddr");
		else
			sprintf(ethaddr, "eth%daddr", i);

		if (env_get(ethaddr))
			continue;

		/* Non OUI / registered MAC address */
		mac_addr[0] = (i << 4) | 0x02;
		mac_addr[1] = (sid[0] >>  0) & 0xff;
		mac_addr[2] = (sid[3] >> 24) & 0xff;
		mac_addr[3] = (sid[3] >> 16) & 0xff;
		mac_addr[4] = (sid[3] >>  8) & 0xff;
		mac_addr[5] = (sid[3] >>  0) & 0xff;

		eth_env_set_enetaddr(ethaddr, mac_addr);
	}

	if (!env_get("serial#")) {
		snprintf(serial_string, sizeof(serial_string),
			"%08x%08x", sid[0], sid[3]);

		env_set("serial#", serial_string);
	}
}

int misc_init_r(void)
{
	const char *spl_dt_name;
	uint boot;

	env_set("fel_booted", NULL);
	env_set("fel_scriptaddr", NULL);
	env_set("mmc_bootdev", NULL);

	boot = sunxi_get_boot_device();
	/* determine if we are running in FEL mode */
	if (boot == BOOT_DEVICE_BOARD) {
		env_set("fel_booted", "1");
		parse_spl_header(SPL_ADDR);
	/* or if we booted from MMC, and which one */
	} else if (boot == BOOT_DEVICE_MMC1) {
		env_set("mmc_bootdev", "0");
	} else if (boot == BOOT_DEVICE_MMC2) {
		env_set("mmc_bootdev", "1");
	}

	/* Set fdtfile to match the FIT configuration chosen in SPL. */
	spl_dt_name = get_spl_dt_name();
	if (spl_dt_name) {
		const char *prefix = "";
		char str[64];

		if (IS_ENABLED(CONFIG_ARM64) && !IS_ENABLED(CONFIG_OF_UPSTREAM))
			prefix = "allwinner/";

		snprintf(str, sizeof(str), "%s%s.dtb", prefix, spl_dt_name);
		env_set("fdtfile", str);
	}

	/* Existing H713 boards may load an environment saved before these defaults
	 * were added.  Populate missing values without replacing user overrides,
	 * and retire the one stale default that is actively dangerous.
	 */
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713)) {
		/* name, current default, superseded default to retire */
		static const struct {
			const char *name, *val, *old;
		} raw_parts[] = {
			{ "fastboot_raw_partition_bootloader",
			  "0x10 0x40",       "0x10 0x1ff0" },
			{ "fastboot_raw_partition_uboot",
			  "0x10 0x40",       "0x10 0x1ff0" },
			{ "fastboot_raw_partition_ubootp",
			  "0x800 0x2800",    "0x49ac00 0x2000" },
			{ "fastboot_raw_partition_splstash",
			  "0x3880 0x40",     "0x49cc00 0x40" },
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(raw_parts); i++) {
			const char *cur = env_get(raw_parts[i].name);

			/* Retire defaults that are now actively wrong: the
			 * 0x1ff0-sector first-stage guard is wide enough to
			 * flash the whole concatenated image over the vendor's
			 * boot region, and the old 0x49ac00/0x49cc00 targets
			 * sit 2.3 GiB into the device, which under the current
			 * layout is the middle of the root filesystem.  A value
			 * someone actually chose is left alone.
			 *
			 * vboot0 went with the vendor boot0 copy at LBA 256:
			 * there is no second vendor chain left to flash.
			 */
			if (!cur ||
			    (raw_parts[i].old && !strcmp(cur, raw_parts[i].old)))
				env_set(raw_parts[i].name, raw_parts[i].val);
		}
	}
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713) &&
	    !env_get("switch_vendor"))
		env_set("switch_vendor", H713_SWITCH_VENDOR_COMMAND);
	if (IS_ENABLED(CONFIG_H713_VENDOR_CHAINLOAD) &&
	    !env_get("boot_vendor"))
		env_set("boot_vendor", H713_BOOT_VENDOR_COMMAND);
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_ACM) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_FASTBOOT) &&
	    !env_get("serial_mode"))
		env_set("serial_mode", H713_SERIAL_COMMAND);
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_ACM) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_FASTBOOT) &&
	    !env_get("acm_mode"))
		env_set("acm_mode", H713_ACM_COMMAND);
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_ACM) &&
	    IS_ENABLED(CONFIG_USB_FUNCTION_FASTBOOT) &&
	    !env_get("fastboot_mode"))
		env_set("fastboot_mode", H713_FASTBOOT_COMMAND);

	setup_environment(gd->fdt_blob);

	return 0;
}

#ifdef CONFIG_H713_POWERON_LIGHT_FAN
/*
 * Drive the two power-enable lines the board needs.  This has to happen in
 * board_late_init, not board_init: at board_init time the GPIO uclass is not
 * up yet, gpio_request() fails silently, and both lines stay unconfigured --
 * visible as "func" with no output value in `gpio status`.  Neither the fan
 * nor the USB socket came up that way.
 *
 * PB5 is the shared fan-power / backlight-enable line: driving it high powers
 * the cooling fan and enables the LED backlight together, which makes the fan
 * a hard interlock -- the backlight cannot be lit without the fan running.
 * Never drive it low.
 *
 * PL3 is the USB-A socket's VBUS enable ("cam-usb-power-gpio" in the stock
 * GPIO map; the vendor bootloader DT carries it as
 * prj/usb0 = <.. 0x0b 0x03 0x01 .. 0x01>, i.e. PL3 output high).  It lives in
 * R_PIO, so the r_pio node must be present in the DT or the request fails.
 */
static void h713_poweron_lines(void)
{
	static const char * const pins[] = { "PB5", "PL3" };
	static const char * const labels[] = { "fan-bl-power", "usb-vbus-power" };
	struct gpio_desc desc;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(pins); i++) {
		/*
		 * By name, not by legacy linear number.  The legacy numbering
		 * assumes 32-pin banks from PA=0, which does not survive the
		 * DT-registered banks here: SUNXI_GPB(5) resolved to some other
		 * pin (request succeeded, fan stayed off) and SUNXI_GPL(3) did
		 * not resolve at all (-ENOENT), because R_PIO is a separate
		 * controller outside that scheme.
		 */
		ret = dm_gpio_lookup_name(pins[i], &desc);
		if (ret) {
			printf("%s (%s): lookup failed: %d\n",
			       labels[i], pins[i], ret);
			continue;
		}
		ret = dm_gpio_request(&desc, labels[i]);
		if (ret && ret != -EBUSY) {
			printf("%s (%s): request failed: %d\n",
			       labels[i], pins[i], ret);
			continue;
		}
		ret = dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		if (ret)
			printf("%s (%s): set output failed: %d\n",
			       labels[i], pins[i], ret);
	}
}
#endif

#ifdef CONFIG_H713_POWER_GATE
/*
 * Power gate: mains-on leaves the board in standby -- dark, silent, LED red --
 * until the power key is pressed.  There is no PMIC and no power-hold line on
 * this board, so "off" can only mean "SoC running, nothing else powered":
 * everything the user can see or hear hangs off PB5 and PL3, and those are
 * exactly the lines h713_poweron_lines() drives.  Holding that call back is
 * the whole gate; until then both pins sit at their reset level (low).
 *
 * The LED needs no code of its own: it follows PB5 (high = blue, low = red),
 * measured on the board.  The red standby light therefore appears by itself
 * while the gate keeps PB5 down.  PL0/PL1, which the vendor device tree calls
 * the LEDs, have no visible effect here.
 *
 * RTC general-purpose word 5 separates a cold start from a warm one.  The RTC
 * domain has no battery: after mains-off every GP word reads back 0, while a
 * value written before a reset (U-Boot "reset", Linux "reboot" via PSCI, or a
 * watchdog reset) survives unchanged.  Hence:
 *
 *   0        cold start, mains just came on          -> crust holds the board
 *                                                       off, else gate
 *   "RUN1"   we were already up                      -> boot straight through
 *   "GATE"   power-off requested (written by TF-A)   -> gate
 *   "STBY"   a standby did not finish (Linux marks    -> gate
 *            it until the display core runs again)
 *   other    unknown                                 -> gate
 *
 * GP7 is the fastboot handoff and is consumed and cleared by preboot; GP2, GP3
 * and GP6 are taken by the vendor firmware and by the ARISC driver.  GP5 is
 * free, and nothing between SPL and the kernel writes it.
 *
 * A cold start does not have to wait on an ARM core.  Since the power off
 * through crust (TF-A sunxi_scp_off.c), Linux's poweroff leaves only crust
 * awake on the ARISC with the board's rails cut: 0.3 W, where this loop draws
 * 2.4-2.9 W.  The vendor firmware treats mains-on like its power off, and so
 * does this gate: on a cold start it asks for PSCI SYSTEM_OFF instead of
 * waiting, and BL31 hands the wait to crust as it does for Linux.  The power
 * key or the remote's power code wakes crust, crust writes "RUN1" and resets
 * the SoC, and the gate boots through.  The key held at mains-on still skips
 * all of it, and h713_gate=0 still turns the gate off.
 *
 * Only a cold start goes that way, so there is at most one SYSTEM_OFF per
 * mains-on: if crust does not take it, BL31 resets with "GATE" and the loop
 * waits.  "GATE" and "STBY" wait in the loop as before - asking crust again
 * after a failed standby would overwrite crust's record of the failure that
 * the next boot reports.
 *
 * Without crust no firmware watches the remote while the loop waits, so the
 * loop reads the IR receiver itself (h713_gate_ir.c): the power key of a
 * stock remote starts the board as the key on the device does.  With crust
 * on the ARISC the remote stays crust's and the loop waits for the key alone.
 */
#define H713_RTC_GP5_REG	0x07090114UL
#define H713_GATE_RUN1		0x52554e31UL	/* "RUN1": system is up */
#define H713_GATE_REQUESTED	0x47415445UL	/* "GATE": TF-A wants standby */
#define H713_GATE_STANDBY	0x53544259UL	/* "STBY": a standby did not finish */

/* 50 ms debounce is what the stock firmware applies to this key. */
#define H713_GATE_POLL_MS	10
#define H713_GATE_DEBOUNCE_MS	50
#define H713_GATE_BYPASS_MS	3000
#define H713_GATE_NOTE_MS	60000

/*
 * crust's contract page in SRAM A2 (ARM view), the words TF-A reads too
 * (sunxi_mmap.h; crust's platform/memory.h is the source).
 */
#define H713_SCP_SHARED		0x00123800UL
#define H713_SCP_MAGIC_REG	(H713_SCP_SHARED + 0x000)
#define H713_SCP_VERSION_REG	(H713_SCP_SHARED + 0x004)
#define H713_SCP_STATUS_REG	(H713_SCP_SHARED + 0x008)	/* 31:16 heartbeat */
#define H713_SCP_BUILD2_REG	(H713_SCP_SHARED + 0x264)	/* contract v5 on */
#define H713_SCP_MAGIC		0x33313748UL	/* "H713" */
#define H713_SCP_VERSION	5
#define H713_SCP_POWER_OFF	BIT(24)		/* BUILD2: H713_POWER_OFF */
#define H713_SCP_RUNNING	BIT(0)		/* STATUS */

/* h713_gate_ir.c */
void h713_gate_ir_start(void);
int h713_gate_ir_poll(void);
void h713_gate_ir_stop(void);

/*
 * The power key is PL4 in R_PIO: active low against an external pull-up, so no
 * bias of ours is needed (measured: reads 1 idle, 0 pressed).  By name for the
 * same reason h713_poweron_lines() uses names -- the legacy linear numbering
 * does not reach R_PIO at all.
 */
static int h713_gate_key_get(struct gpio_desc *key)
{
	int ret;

	ret = dm_gpio_lookup_name("PL4", key);
	if (ret)
		return ret;

	ret = dm_gpio_request(key, "power-key");
	if (ret && ret != -EBUSY)
		return ret;

	ret = dm_gpio_set_dir_flags(key, GPIOD_IS_IN);
	if (ret)
		return ret;

	/* Read once here so a pin we cannot read fails before the loop. */
	return dm_gpio_get_value(key) < 0 ? -EIO : 0;
}

/*
 * Raw level rather than GPIOD_ACTIVE_LOW, so the one inversion this file makes
 * is written down at the place it happens.
 */
static bool h713_gate_key_down(struct gpio_desc *key)
{
	return dm_gpio_get_value(key) == 0;
}

/*
 * Service back door: a key that is already held when the mains comes on, and
 * stays held, skips the gate for this boot.  It is the only way to boot a
 * gated board with no serial console and no working flag.
 */
static bool h713_gate_key_held(struct gpio_desc *key, unsigned int ms)
{
	unsigned int waited;

	if (!h713_gate_key_down(key))
		return false;

	for (waited = 0; waited < ms; waited += H713_GATE_POLL_MS) {
		schedule();
		mdelay(H713_GATE_POLL_MS);
		if (!h713_gate_key_down(key))
			return false;
	}

	return true;
}

/*
 * Wait for one whole key stroke: idle, then low for at least the debounce
 * time, then idle again.  The release is what starts the boot.  If the press
 * alone did, a key still held from the back-door check above -- or simply held
 * a moment too long -- would start the boot the instant the gate opened, and
 * nobody would ever see the standby state.
 *
 * With @remote the same pass drains the IR receiver, and a power code ends
 * the wait at once: a remote sends no release, and its repeat frames carry
 * no code.  Returns that code, or -1 for the key.
 */
static int h713_gate_wait(struct gpio_desc *key, bool remote)
{
	unsigned int low_ms = 0, high_ms = 0;
	bool idle_seen = false, pressed = false, noted = false;
	ulong last_note = get_timer(0);
	int code = -1;

	printf("gate: waiting for power key\n");

	for (;;) {
		if (h713_gate_key_down(key)) {
			high_ms = 0;
			if (low_ms < H713_GATE_DEBOUNCE_MS)
				low_ms += H713_GATE_POLL_MS;
			if (idle_seen && low_ms >= H713_GATE_DEBOUNCE_MS)
				pressed = true;
		} else {
			low_ms = 0;
			if (high_ms < H713_GATE_DEBOUNCE_MS)
				high_ms += H713_GATE_POLL_MS;
			if (high_ms >= H713_GATE_DEBOUNCE_MS) {
				if (pressed)
					break;
				idle_seen = true;
			}
		}

		if (remote) {
			code = h713_gate_ir_poll();
			if (code >= 0)
				break;
		}

		/* One dot a minute: proof of life without flooding the log. */
		if (get_timer(last_note) >= H713_GATE_NOTE_MS) {
			last_note = get_timer(0);
			noted = true;
			printf(".");
		}

		schedule();
		mdelay(H713_GATE_POLL_MS);
	}

	if (noted)
		printf("\n");

	return code;
}

/*
 * Does the crust on the ARISC say it can hold the board off?  Magic, contract
 * version 5 and BUILD2 bit 24 - the vendor's ARISC firmware and an older crust
 * have none of that, and the gate then waits in its loop without the detour
 * of a reset.  That a crust which says so also runs and is outside a standby
 * is BL31's check before it asks; this one only saves the pointless reset.
 */
static bool h713_gate_crust_can_hold(void)
{
	u32 version, build2;

	if (readl(H713_SCP_MAGIC_REG) != H713_SCP_MAGIC) {
		printf("gate: no crust on the ARISC\n");
		return false;
	}

	version = readl(H713_SCP_VERSION_REG);
	build2 = version >= H713_SCP_VERSION ? readl(H713_SCP_BUILD2_REG) : 0;
	if (!(build2 & H713_SCP_POWER_OFF)) {
		printf("gate: crust (contract v%u, build2 %08x) cannot hold the board off\n",
		       version, build2);
		return false;
	}

	return true;
}

/*
 * Does crust run on the ARISC?  Its page in SRAM A2 outlives a reset and a
 * change of firmware, so the page alone does not say: crust runs when the
 * page says running and its main loop moves the heartbeat, microseconds
 * apart - BL31's test before it hands crust the power off.
 */
static bool h713_gate_crust_runs(void)
{
	u32 status = readl(H713_SCP_STATUS_REG);
	unsigned int i;

	if (readl(H713_SCP_MAGIC_REG) != H713_SCP_MAGIC ||
	    !(status & H713_SCP_RUNNING))
		return false;

	for (i = 0; i < 100; i++) {
		udelay(100);
		if (readl(H713_SCP_STATUS_REG) != status)
			return true;
	}
	return false;
}

/*
 * The remote's power key wakes the loop too, unless crust runs on the ARISC:
 * on an image with standby the remote is crust's, which reads it in its own
 * off state, and the loop stays as it was there.
 */
static bool h713_gate_remote_start(void)
{
	if (!IS_ENABLED(CONFIG_H713_POWER_GATE_IR))
		return false;

	if (h713_gate_crust_runs()) {
		printf("gate: crust runs on the ARISC, the remote is not read here\n");
		return false;
	}

	h713_gate_ir_start();
	printf("gate: the remote's power key starts the board too\n");
	return true;
}

static void h713_power_gate(void)
{
	struct gpio_desc key;
	const char *sel;
	bool gate = true, remote;
	u32 flag;
	int ret, code;

	/*
	 * The flag goes into every line so a console log from the field
	 * shows what the previous life of the board left behind.
	 */
	flag = readl(H713_RTC_GP5_REG);

	/* Override without a rebuild; anything but "0" leaves the gate on. */
	sel = env_get("h713_gate");
	if (sel && !strcmp(sel, "0")) {
		printf("gate: off (h713_gate=0, GP5 %08x)\n", flag);
		gate = false;
	}

	if (gate) {
		if (flag == H713_GATE_RUN1) {
			printf("gate: warm start (GP5 %08x), booting\n", flag);
			gate = false;
		} else if (flag == H713_GATE_REQUESTED) {
			printf("gate: power-off requested (GP5 %08x)\n", flag);
		} else if (flag == H713_GATE_STANDBY) {
			printf("gate: a standby did not finish (GP5 %08x)\n", flag);
		} else if (!flag) {
			printf("gate: cold start (GP5 %08x)\n", flag);
		} else {
			printf("gate: unknown word in GP5 (%08x)\n", flag);
		}
	}

	if (gate) {
		ret = h713_gate_key_get(&key);
		if (ret) {
			/*
			 * Without a readable key there is no way out of the
			 * loop, so a board that cannot see PL4 must boot.
			 */
			printf("gate: power key PL4 unavailable (%d), booting\n",
			       ret);
			gate = false;
		}
	}

	if (gate && h713_gate_key_held(&key, H713_GATE_BYPASS_MS)) {
		printf("gate: key held at power-on, bypass\n");
		gate = false;
	}

	/*
	 * A cold start hands the wait to crust (see the top of this block).
	 * PSCI SYSTEM_OFF does not come back: crust takes the cluster down,
	 * or BL31 resets with "GATE" and the next pass waits in the loop.
	 */
	if (gate && !flag && h713_gate_crust_can_hold()) {
		printf("gate: crust holds the board off, the power key or the remote wakes it\n");
		psci_system_off();
	}

	if (gate) {
		remote = h713_gate_remote_start();
		code = h713_gate_wait(&key, remote);
		if (remote)
			h713_gate_ir_stop();
		if (code < 0)
			printf("gate: power key, booting\n");
		else
			printf("gate: remote power key (NEC %04x), booting\n", code);
	}

	/*
	 * Mark the system as up on every path, before anything is powered.
	 * A reboot, a crash or a watchdog reset from here on finds RUN1 and
	 * boots through, so a device in the field never ends up waiting for a
	 * key that nobody is there to press.
	 */
	writel(H713_GATE_RUN1, H713_RTC_GP5_REG);
}
#endif

int board_late_init(void)
{
#ifdef CONFIG_H713_POWER_GATE
	/* Must run before the power lines: the gate is their absence. */
	h713_power_gate();
#endif
#ifdef CONFIG_H713_POWERON_LIGHT_FAN
	h713_poweron_lines();
#endif
#ifdef CONFIG_USB_ETHER
	usb_ether_init();
#endif

	return 0;
}

static void bluetooth_dt_fixup(void *blob)
{
	/* Some devices ship with a Bluetooth controller default address.
	 * Set a valid address through the device tree.
	 */
	uchar tmp[ETH_ALEN], bdaddr[ETH_ALEN];
	unsigned int sid[4];
	int i;

	if (!CONFIG_BLUETOOTH_DT_DEVICE_FIXUP[0])
		return;

	if (eth_env_get_enetaddr("bdaddr", tmp)) {
		/* Convert between the binary formats of the corresponding stacks */
		for (i = 0; i < ETH_ALEN; ++i)
			bdaddr[i] = tmp[ETH_ALEN - i - 1];
	} else {
		if (!get_unique_sid(sid))
			return;

		bdaddr[0] = ((sid[3] >>  0) & 0xff) ^ 1;
		bdaddr[1] = (sid[3] >>  8) & 0xff;
		bdaddr[2] = (sid[3] >> 16) & 0xff;
		bdaddr[3] = (sid[3] >> 24) & 0xff;
		bdaddr[4] = (sid[0] >>  0) & 0xff;
		bdaddr[5] = 0x02;
	}

	do_fixup_by_compat(blob, CONFIG_BLUETOOTH_DT_DEVICE_FIXUP,
			   "local-bd-address", bdaddr, ETH_ALEN, 1);
}

#define PINEPHONE_LIS3MDL_I2C_ADDR	0x1e
#define PINEPHONE_LIS3MDL_I2C_BUS	1 /* I2C1 */

static void board_dt_fixup(void *blob)
{
	struct udevice *bus, *dev;

	if (IS_ENABLED(CONFIG_PINEPHONE_DT_SELECTION) &&
	    !fdt_node_check_compatible(blob, 0, "pine64,pinephone-1.2")) {
		if (!uclass_get_device_by_seq(UCLASS_I2C,
					      PINEPHONE_LIS3MDL_I2C_BUS,
					      &bus)) {
			dm_i2c_probe(bus, PINEPHONE_LIS3MDL_I2C_ADDR, 0, &dev);
			fdt_set_status_by_compatible(blob, "st,lis3mdl-magn",
				dev ? FDT_STATUS_OKAY  : FDT_STATUS_DISABLED);
			fdt_set_status_by_compatible(blob, "voltafield,af8133j",
				dev ? FDT_STATUS_DISABLED : FDT_STATUS_OKAY);
		}
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int __maybe_unused r;

	/*
	 * Call setup_environment and fdt_fixup_ethernet again
	 * in case the boot fdt has ethernet aliases the u-boot
	 * copy does not have.
	 */
	setup_environment(blob);
	fdt_fixup_ethernet(blob);

	bluetooth_dt_fixup(blob);
	board_dt_fixup(blob);

#ifdef CONFIG_VIDEO_DT_SIMPLEFB
	r = sunxi_simplefb_setup(blob);
	if (r)
		return r;
#endif
	return 0;
}

#ifdef CONFIG_SPL_LOAD_FIT
static void set_spl_dt_name(const char *name)
{
	struct boot_file_head *spl = get_spl_header(SPL_ENV_HEADER_VERSION);

	if (spl == INVALID_SPL_HEADER)
		return;

	/* Promote the header version for U-Boot proper, if needed. */
	if (spl->spl_signature[3] < SPL_DT_HEADER_VERSION)
		spl->spl_signature[3] = SPL_DT_HEADER_VERSION;

	strcpy((char *)&spl->string_pool, name);
	spl->dt_name_offset = offsetof(struct boot_file_head, string_pool);
}

int board_fit_config_name_match(const char *name)
{
	const char *best_dt_name = get_spl_dt_name();
	int ret;

#ifdef CONFIG_DEFAULT_DEVICE_TREE
	if (best_dt_name == NULL)
		best_dt_name = CONFIG_DEFAULT_DEVICE_TREE;
#endif

	if (best_dt_name == NULL) {
		/* No DT name was provided, so accept the first config. */
		return 0;
	}
#ifdef CONFIG_PINE64_DT_SELECTION
	if (strstr(best_dt_name, "-pine64-plus")) {
		/* Differentiate the Pine A64 boards by their DRAM size. */
		if (gd->ram_size == SZ_512M)
			best_dt_name = "sun50i-a64-pine64";
	}
#endif
#ifdef CONFIG_PINEPHONE_DT_SELECTION
	if (strstr(best_dt_name, "-pinephone")) {
		/* Differentiate the PinePhone revisions by GPIO inputs. */
		prcm_apb0_enable(PRCM_APB0_GATE_PIO);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_UP);
		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_INPUT);
		udelay(100);

		/* PL6 is pulled low by the modem on v1.2. */
		if (gpio_get_value(SUNXI_GPL(6)) == 0)
			best_dt_name = "sun50i-a64-pinephone-1.2";
		else
			best_dt_name = "sun50i-a64-pinephone-1.1";

		sunxi_gpio_set_cfgpin(SUNXI_GPL(6), SUNXI_GPIO_DISABLE);
		sunxi_gpio_set_pull(SUNXI_GPL(6), SUNXI_GPIO_PULL_DISABLE);
		prcm_apb0_disable(PRCM_APB0_GATE_PIO);
	}
#endif

	ret = strcmp(name, best_dt_name);

	/*
	 * If one of the FIT configurations matches the most accurate DT name,
	 * update the SPL header to provide that DT name to U-Boot proper.
	 */
	if (ret == 0)
		set_spl_dt_name(best_dt_name);

	return ret;
}
#endif /* CONFIG_SPL_LOAD_FIT */
