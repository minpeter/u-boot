// SPDX-License-Identifier: GPL-2.0+
/*
 * The L018's lamp and fan lines.
 *
 * The L018 shares the HY310's DRAM words, panel and display firmware, and
 * builds from its base, but three lines around them behave the other way
 * round (read off a running unit under its stock firmware, 24.10.2026):
 *
 *   PB5   the lamp and fan enable. Left at its reset level (input, no pull)
 *         the HY310's is off; the L018's is on - lamp and fan run in the gate
 *         unless PB5 is driven low, and go off when it is.
 *   PB4   PWM2, the lamp's dimmer, which DIMS as the duty rises: the three
 *         stock modes are high 30 %, std 50 %, low 75 % at 25 kHz, and an
 *         undriven PB4 is full light. Held high (100 %) the lamp goes dark.
 *   PH17  PWM0, the fan's speed: 95/70/60 % with the lamp at high/std/low.
 *
 * h713_l018_lamp_fan_off() makes the gate what it is on the HY310 - dark and
 * quiet. h713_l018_lamp_fan_std() sets the stock std mode before PB5 comes up,
 * and Linux keeps both PWMs as they are found (its pwm-sun8i probe only
 * deasserts the block's reset; the kernel's pwm-backlight then owns PWM2).
 */

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <sunxi_gpio.h>
#include <stdio.h>

#define L018_PB_DATA		0x02000040UL	/* bank B: 0x02000030 + 0x10 */

#define L018_CCU_PWM_BGR	0x020017acUL
#define L018_PWM_BASE		0x02000c00UL
#define L018_PWM_PCCR(pair)	(L018_PWM_BASE + 0x020 + (pair) * 4)
#define L018_PWM_CLK_GATE	(L018_PWM_BASE + 0x040)
#define L018_PWM_ENABLE		(L018_PWM_BASE + 0x080)
#define L018_PWM_CTL(ch)	(L018_PWM_BASE + 0x100 + (ch) * 0x20)
#define L018_PWM_PERIOD(ch)	(L018_PWM_BASE + 0x104 + (ch) * 0x20)

#define L018_PWM_CYCLES		960		/* 24 MHz / 25 kHz, the stock period */
#define L018_LAMP_PWM		2		/* PB4, pinmux function 2 */
#define L018_FAN_PWM		0		/* PH17, pinmux function 3 */
#define L018_LAMP_STD		50		/* % duty, stock std */
#define L018_FAN_STD		70		/* % duty, stock std */

static void l018_pwm_bus_enable(void)
{
	setbits_le32((void *)L018_CCU_PWM_BGR, BIT(0));		/* bus gate */
	udelay(20);
	setbits_le32((void *)L018_CCU_PWM_BGR, BIT(16));	/* reset off */
	udelay(20);
}

/* One channel at 25 kHz from HOSC, active high (stock PCR 0x100). */
static void l018_pwm_set(uint ch, uint percent)
{
	u32 act = L018_PWM_CYCLES * percent / 100;

	clrbits_le32((void *)L018_PWM_PCCR(ch / 2), 0x18f);	/* HOSC, /1 */
	writel(BIT(8), L018_PWM_CTL(ch));
	writel(((L018_PWM_CYCLES - 1) << 16) | act, L018_PWM_PERIOD(ch));
	setbits_le32((void *)L018_PWM_CLK_GATE, BIT(ch));
	setbits_le32((void *)L018_PWM_ENABLE, BIT(ch));
}

void h713_l018_lamp_fan_off(void)
{
	setbits_le32((void *)L018_PB_DATA, BIT(4));		/* lamp dark */
	sunxi_gpio_set_cfgpin(SUNXI_GPB(4), SUNXI_GPIO_OUTPUT);
	clrbits_le32((void *)L018_PB_DATA, BIT(5));		/* lamp, fan off */
	sunxi_gpio_set_cfgpin(SUNXI_GPB(5), SUNXI_GPIO_OUTPUT);
	printf("L018: PB5 low, PB4 high - lamp and fan off\n");
}

void h713_l018_lamp_fan_std(void)
{
	l018_pwm_bus_enable();
	l018_pwm_set(L018_FAN_PWM, L018_FAN_STD);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(17), 3);
	l018_pwm_set(L018_LAMP_PWM, L018_LAMP_STD);
	sunxi_gpio_set_cfgpin(SUNXI_GPB(4), 2);
	setbits_le32((void *)L018_PB_DATA, BIT(5));
	sunxi_gpio_set_cfgpin(SUNXI_GPB(5), SUNXI_GPIO_OUTPUT);
	printf("L018: lamp PWM%u %u %%, fan PWM%u %u %% (stock std), PB5 high\n",
	       L018_LAMP_PWM, L018_LAMP_STD, L018_FAN_PWM, L018_FAN_STD);
}
