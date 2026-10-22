// SPDX-License-Identifier: GPL-2.0+
/*
 * H713 power gate: the remote's power key starts the board as well.
 *
 * In the gate (board.c) no firmware runs that watches the remote - the
 * release image starts nothing on the ARISC - so the gate reads the IR
 * receiver itself. R_CIR samples PL9 on its own and queues runs of one level
 * in its FIFO; the gate's loop drains the FIFO every 10 ms and decodes NEC
 * here. Both stock remotes send NEC: the HY310's (address 0x02, power 0x1d,
 * the kernel's rc-h713-fd02) and the original HY300's (address 0x00, power
 * 0x14, measured on 22.10.2026).
 *
 * The receiver is set up as Linux's sunxi-cir sets it up on this SoC: module
 * clock hosc / 3 = 8 MHz, one sample per 64 clocks = 8 us, input inverted so
 * that a burst of carrier reads as a pulse. A FIFO byte is one run: bit 7 the
 * level, bits 6:0 its length in samples minus one; a longer run goes on in
 * the next byte. A NEC frame queues at most about 20 bytes in 10 ms, so the
 * 64-byte FIFO lets the loop keep the pace it polls the key at.
 *
 * Pin, clock and receiver go back to what the gate found before U-Boot goes
 * on, so Linux finds them as it did before this file.
 */

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>

#define H713_R_CCU_IR_CLK	0x070101c0UL	/* R_IR_RX module clock */
#define   H713_IR_CLK_8MHZ	(BIT(31) | (1 << 24) | 2)	/* on, hosc, /3 */
#define H713_R_CCU_IR_BGR	0x070101ccUL
#define   H713_IR_BGR_ON	(BIT(16) | BIT(0))	/* reset released, bus on */
#define H713_PL_CFG1		0x07022004UL	/* PL8-PL15, 4 bits a pin */
#define   H713_PL9_FUNC		(0xf << 4)
#define   H713_PL9_S_CIR	(3 << 4)
#define H713_PL_PULL0		0x07022024UL	/* PL0-PL15, 2 bits a pin */
#define   H713_PL9_PULL		(3 << 18)
#define   H713_PL9_PULL_UP	(1 << 18)

#define H713_CIR_CTL		0x07040000UL
#define   H713_CIR_ON		(BIT(0) | BIT(1))	/* global, rx */
#define   H713_CIR_MODE		(3 << 4)
#define H713_CIR_RXCTL		0x07040010UL
#define   H713_CIR_INVERT	BIT(2)
#define H713_CIR_RXFIFO		0x07040020UL
#define H713_CIR_RXINT		0x0704002cUL
#define   H713_CIR_RXINT_ALL	(BIT(0) | BIT(1) | BIT(4) | (31 << 8))
#define H713_CIR_RXSTA		0x07040030UL
#define   H713_CIR_OVERFLOW	BIT(0)
#define   H713_CIR_PACKET_END	BIT(1)
#define   H713_CIR_COUNT(s)	(((s) >> 8) & 0x7f)
#define H713_CIR_CFG		0x07040034UL
/* noise below 1 sample; idle after 20 x 128 samples = 20.5 ms */
#define   H713_CIR_THRESHOLDS	((1 << 2) | (19 << 8))
#define H713_CIR_FIFO		64
#define H713_CIR_SAMPLE_US	8

/* NEC, in us, units of 562.5 us; each bound holds a remote 15 % off */
#define NEC_HEADER_PULSE(us)	((us) >= 7500 && (us) <= 10500)	/* 16 */
#define NEC_HEADER_SPACE(us)	((us) >= 3500 && (us) <= 5500)	/* 8 */
#define NEC_UNIT(us)		((us) >= 300 && (us) <= 900)	/* 1 */
#define NEC_THREE(us)		((us) >= 1300 && (us) <= 2100)	/* 3 */
#define NEC_GAP			5000	/* after the stop pulse; NEC leaves 40 ms */

enum nec_state { NEC_IDLE, NEC_SPACE, NEC_PULSE, NEC_BIT, NEC_GAP_WAIT };

static struct {
	enum nec_state state;
	unsigned int bits;
	u32 data;
	bool pulse;		/* level of the run being measured */
	u32 run;		/* its length so far */
} nec;

static u32 saved_clk, saved_bgr, saved_cfg1, saved_pull0;

static const u16 h713_gate_ir_power[] = {
	0x021d,			/* HY310 remote */
	0x0014,			/* original HY300 remote */
};

/*
 * A frame is address, ~address, command, ~command, each LSB first. Both
 * remotes send the plain form, so both complements are checked.
 */
static int nec_frame(u32 data)
{
	u8 addr = data, naddr = data >> 8, cmd = data >> 16, ncmd = data >> 24;

	if ((u8)~addr != naddr || (u8)~cmd != ncmd)
		return -1;
	return addr << 8 | cmd;
}

/* A run of one level ended: one step through header, 32 bits, stop pulse. */
static void nec_step(bool pulse, u32 us)
{
	switch (nec.state) {
	case NEC_IDLE:
		break;
	case NEC_SPACE:
		/* 2.25 ms here is a repeat: it carries no code and starts nothing */
		if (!pulse && NEC_HEADER_SPACE(us)) {
			nec.state = NEC_PULSE;
			nec.bits = 0;
			nec.data = 0;
			return;
		}
		break;
	case NEC_PULSE:
		if (pulse && NEC_UNIT(us)) {
			nec.state = nec.bits == 32 ? NEC_GAP_WAIT : NEC_BIT;
			return;
		}
		break;
	case NEC_BIT:
		if (!pulse && (NEC_UNIT(us) || NEC_THREE(us))) {
			if (NEC_THREE(us))
				nec.data |= BIT(nec.bits);
			nec.bits++;
			nec.state = NEC_PULSE;
			return;
		}
		break;
	case NEC_GAP_WAIT:	/* a pulse soon after the stop pulse: not NEC */
		break;
	}
	nec.state = pulse && NEC_HEADER_PULSE(us) ? NEC_SPACE : NEC_IDLE;
}

/* One FIFO run; returns the code of a frame it completes, else -1. */
static int nec_feed(bool pulse, u32 us)
{
	if (pulse != nec.pulse) {
		if (nec.run)
			nec_step(nec.pulse, nec.run);
		nec.pulse = pulse;
		nec.run = 0;
	}
	nec.run += us;
	if (nec.state == NEC_GAP_WAIT && !nec.pulse && nec.run >= NEC_GAP) {
		nec.state = NEC_IDLE;
		return nec_frame(nec.data);
	}
	return -1;
}

static void nec_reset(void)
{
	nec.state = NEC_IDLE;
	nec.pulse = false;
	nec.run = 0;
}

/*
 * The receiver saw the line idle for its threshold. Whether it wrote the
 * space up to there or not, a frame that has its stop pulse is whole; a
 * stop pulse still open ends here. Anything else is left to the next run.
 */
static int nec_idle(void)
{
	int code;

	if (nec.state == NEC_PULSE && nec.bits == 32 && nec.pulse)
		nec_step(true, nec.run);
	if (nec.state != NEC_GAP_WAIT)
		return -1;
	code = nec_frame(nec.data);
	nec_reset();
	return code;
}

void h713_gate_ir_start(void)
{
	saved_clk = readl(H713_R_CCU_IR_CLK);
	saved_bgr = readl(H713_R_CCU_IR_BGR);
	saved_cfg1 = readl(H713_PL_CFG1) & H713_PL9_FUNC;
	saved_pull0 = readl(H713_PL_PULL0) & H713_PL9_PULL;

	/* through reset, whatever ran the receiver before */
	clrbits_le32(H713_R_CCU_IR_BGR, H713_IR_BGR_ON);
	writel(H713_IR_CLK_8MHZ, H713_R_CCU_IR_CLK);
	setbits_le32(H713_R_CCU_IR_BGR, H713_IR_BGR_ON);
	clrsetbits_le32(H713_PL_PULL0, H713_PL9_PULL, H713_PL9_PULL_UP);
	clrsetbits_le32(H713_PL_CFG1, H713_PL9_FUNC, H713_PL9_S_CIR);

	writel(H713_CIR_MODE, H713_CIR_CTL);
	writel(H713_CIR_THRESHOLDS, H713_CIR_CFG);
	writel(H713_CIR_INVERT, H713_CIR_RXCTL);
	writel(0xff, H713_CIR_RXSTA);
	/* as Linux sets them; U-Boot takes no interrupt, the status is polled */
	writel(H713_CIR_RXINT_ALL, H713_CIR_RXINT);
	writel(H713_CIR_MODE | H713_CIR_ON, H713_CIR_CTL);
	nec_reset();
}

static int h713_gate_ir_power_key(int code)
{
	unsigned int i;

	for (i = 0; code >= 0 && i < ARRAY_SIZE(h713_gate_ir_power); i++)
		if (code == h713_gate_ir_power[i])
			return code;
	return -1;
}

/* Drain the FIFO; the code of a power key, else -1. */
int h713_gate_ir_poll(void)
{
	u32 sta = readl(H713_CIR_RXSTA);
	unsigned int n = H713_CIR_COUNT(sta);
	int code;
	u8 b;

	writel(sta | 0xff, H713_CIR_RXSTA);
	if (n > H713_CIR_FIFO)
		n = H713_CIR_FIFO;
	if (sta & H713_CIR_OVERFLOW)	/* runs were lost: start over */
		nec_reset();

	while (n--) {
		b = readb(H713_CIR_RXFIFO);
		if (sta & H713_CIR_OVERFLOW)
			continue;
		code = nec_feed(b & 0x80, ((b & 0x7f) + 1) * H713_CIR_SAMPLE_US);
		if (h713_gate_ir_power_key(code) >= 0)
			return code;
	}

	if (sta & H713_CIR_PACKET_END)
		return h713_gate_ir_power_key(nec_idle());
	return -1;
}

void h713_gate_ir_stop(void)
{
	writel(0, H713_CIR_RXINT);
	writel(0, H713_CIR_CTL);
	writel(0xff, H713_CIR_RXSTA);
	clrsetbits_le32(H713_PL_CFG1, H713_PL9_FUNC, saved_cfg1);
	clrsetbits_le32(H713_PL_PULL0, H713_PL9_PULL, saved_pull0);
	writel(saved_clk, H713_R_CCU_IR_CLK);
	writel(saved_bgr, H713_R_CCU_IR_BGR);
}
