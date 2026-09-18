// SPDX-License-Identifier: GPL-2.0+
/*
 * Manual H713 display MIPS firmware loader.
 *
 * The register sequence is recovered from the HY200 factory U-Boot. Keep the
 * command manual until the firmware supplies a readiness witness independent
 * of the CPU status bit. Only the bench-proven display clock and routing
 * prerequisites belong here; LVDS, TVCAP, HDMI, and INCAP remain out of scope.
 */

#include <command.h>
#include <blk.h>
#include <part.h>
#include <sunxi_image.h>
#include <bmp_layout.h>
#include <env.h>
#include <console.h>
#include <cpu_func.h>
#include <fs.h>
#include <linux/delay.h>
#include <time.h>
#include <linux/kernel.h>
#include <vsprintf.h>
#include <sunxi_gpio.h>
#include <linux/string.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <u-boot/sha256.h>

#define H713_MIPS_FW_ADDR		0x4b100000UL
/* First row's size; the starting value for h713_mips_fw_size. */
#define H713_MIPS_FW_SIZE		0x00132910UL
#define H713_MIPS_FW_WINDOW_SIZE	0x00500000UL
#define H713_MIPS_BSS_START		0x4b232c00UL
#define H713_MIPS_BSS_END		0x4bac7c40UL
#define H713_MIPS_WITNESS_ADDR		(H713_MIPS_FW_ADDR + \
					 H713_MIPS_FW_WINDOW_SIZE)
#define H713_MIPS_WITNESS_SEED		0x4d495053

#define H713_MIPS_CLK_REG		0x02001600UL
#define H713_MIPS_RESET_REG		0x0200160cUL
#define H713_MIPS_STATUS_REG		0x0306101cUL
#define H713_MIPS_SHARE_ADDR_REG	0x03061024UL
#define H713_MIPS_SHARE_SIZE_REG	0x03061028UL
#define H713_MIPS_BOOTADDR_REG		0x03061030UL

/*
 * The MIPS core clock: gate on, parent 0, no pre-divider, M = 2.
 *
 * bit 31 gates SCLK, bits 26:24 select the parent, bits 9:8 are the power-of-
 * two pre-divider and bits 4:0 the divider M, which divides by M+1. 0x80000002
 * is therefore parent 0 divided by three, and parent 0 of a module clock in
 * this CCU is PLL_PERI(2X). Our own SPL programs PLL6 with CCM_PLL6_DEFAULT
 * 0xa8003100 for the H713 -- N+1 = 50, both output dividers 1, so 24 MHz * 50
 * / 2 = 600 MHz on the 1X output and 1200 MHz on the 2X.
 *
 *	1200 MHz / 3 = 400 MHz
 *
 * which is exactly what the vendor kernel asks for: sunxi-mipsloader calls
 * clk_set_rate(mips_clk, 0x17d78400) -- 400000000 -- on its very first
 * mips_reset(), i.e. after we have already started the core (A7 report, 5).
 * The rate we hand over is the rate it wants, so its call is a no-op and there
 * is nothing to change here. Had the two disagreed, the driver would have
 * re-rated a running coprocessor.
 *
 * The arithmetic assumes the standard sun50iw12 module-clock encoding; the
 * exact match with the vendor's 400 MHz is what makes that reading credible,
 * and it is the only cross-check available without the CCU driver's parent
 * table.
 */
#define H713_MIPS_CLK_VALUE		0x80000002
#define H713_MIPS_CLK_DISABLED		0x00000000

/*
 * The reset ladder, with the vendor's own names for its rungs.
 *
 * 0x0200160c is the MIPS bus-gating-and-reset register, and the vendor device
 * tree names every bit in it. mipsloader@3061000 declares
 *
 *	resets      = <&ccu 1>, <&ccu 2>, <&ccu 3>;
 *	reset-names = "bus-reset", "cold-reset", "soft-reset";
 *	clocks      = <&ccu 27>, <&ccu 26>;
 *	clock-names = "bus-clk", "mips-clk";
 *
 * and entries 1/2/3 of sun50iw12_ccu_resets are bits 16, 17 and 18 of this
 * register (A7 report, 5). Bit 0 is the bus-clk gate, as in every other
 * Allwinner BGR register. So the stages are, in the order the kernel's
 * mips_reset() issues them:
 *
 *	0x00000000  everything asserted, gate off -- the parked state
 *	0x00010000  bus-reset deasserted
 *	0x00030000  cold-reset deasserted as well
 *	0x00030001  bus clock gated on
 *	0x00070001  soft-reset deasserted: the core starts fetching
 *
 * The last line is what the vendor kernel's ioctl start does and nothing
 * else: mips_reset() brings the first four up, and the "start" opcode of
 * ioctl 0x10648 writes the boot address and then deasserts bit 18. That is
 * why we write 0x03061030 immediately before it.
 */
#define H713_MIPS_RESET_ASSERTED	0x00000000
#define H713_MIPS_RESET_STAGE1		0x00010000	/* bus-reset    */
#define H713_MIPS_RESET_STAGE2		0x00030000	/* + cold-reset */
#define H713_MIPS_RESET_STAGE3		0x00030001	/* + bus clock  */
#define H713_MIPS_RESET_RELEASED	0x00070001	/* + soft-reset */
#define H713_MIPS_STATUS_RELEASED	0x00000001

/*
 * Workspace layout, from the vendor display_cfg.xml header. The ARM stages the
 * config and TSE windows, so those are the only regions it must not clear.
 * Everything else above the firmware image is uninitialized DRAM that differs
 * per boot, and the firmware reads it — leaving it alone makes runs
 * irreproducible.
 */
#define H713_MIPS_DBG_ADDR		0x4bd01000UL
#define H713_MIPS_CFG_ADDR		0x4be01000UL
#define H713_MIPS_CFG_SIZE		0x00040000UL
#define H713_MIPS_TSE_ADDR		0x4be41000UL
#define H713_MIPS_TSE_SIZE		0x00100000UL
#define H713_MIPS_FB_ADDR		0x4bf41000UL
#define H713_MIPS_FB_SIZE		0x01a00000UL

#define H713_MIPS_SHMEM_ADDR		0x4e300000UL
#define H713_MIPS_SHMEM_SIZE		0x00500000UL
#define H713_MIPS_SHMEM_MAGIC		0xdeadbeef
#define H713_MIPS_SHMEM_MAGIC1_OFF	0x00000090UL
#define H713_MIPS_SHMEM_MAX_CPU_OFF	0x00004cd8UL
#define H713_MIPS_SHMEM_ARM_FLAG_OFF	0x00004cdcUL
#define H713_MIPS_SHMEM_MIPS_FLAG_OFF	0x00004ce0UL
#define H713_MIPS_SHMEM_MAGIC2_OFF	0x000075b8UL
#define H713_MIPS_SHMEM_ARM_READY	(BIT(0) | BIT(2))
#define H713_MIPS_SHMEM_MIPS_READY	BIT(0)
#define H713_MIPS_SHMEM_MIPS_APP_READY	BIT(2)
#define H713_MIPS_SHMEM_LOCK_COUNT	12
#define H713_MIPS_SHMEM_LOCK_SIZE	12
#define H713_MIPS_SHMEM_LOCK_FREE	2
#define H713_MIPS_SHMEM_LOCK_THREAD_NONE	0x000000ff
#define H713_MIPS_SHMEM_CALL_VERSION_OFF	0x000075c0UL
#define H713_MIPS_SHMEM_CALL_COUNT_OFF	0x000075c4UL
#define H713_MIPS_SHMEM_CALL_TABLE_OFF	0x000075c8UL
#define H713_MIPS_SHMEM_CALL_ENTRY_COUNT	1224
#define H713_MIPS_SHMEM_CALL_ENTRY_SIZE	96
#define H713_MIPS_SHMEM_CALL_NEXT_OFF	92
/*
 * One second was chosen when the firmware stalled in its first few hundred
 * milliseconds. With the config and TSE artifacts staged it now runs its whole
 * instrumented startup, so give an uninstrumented run the same budget as a
 * traced one before calling readiness a failure.
 */
#define H713_MIPS_READY_TIMEOUT_US	10000000
#define H713_MIPS_TRACE_TIMEOUT_US	10000000
#define H713_MIPS_DISP_READY_TIMEOUT_US	4000000
#define H713_MIPS_TRACE_OFF		0x00040000UL
#define H713_MIPS_TRACE_MARKER_COUNT	88
#define H713_MIPS_TRACE_DBG_ADDR	88
#define H713_MIPS_TRACE_DBG_SIZE	89
#define H713_MIPS_TRACE_REG_COUNT	90
#define H713_MIPS_TRACE_REG_OBJECT	91
#define H713_MIPS_TRACE_REG_CALLBACK	92
#define H713_MIPS_TRACE_COUNT		93
#define H713_MIPS_COMM_TRACE_STAGE_OFF	H713_MIPS_TRACE_OFF
#define H713_MIPS_COMM_TRACE_MAGIC_OFF	(H713_MIPS_TRACE_OFF + 4)
#define H713_MIPS_COMM_TRACE_QUEUE_OFF	(H713_MIPS_TRACE_OFF + 8)
#define H713_MIPS_COMM_TRACE_CALL_OFF	(H713_MIPS_TRACE_OFF + 12)
#define H713_MIPS_COMM_TRACE_ACK_OFF	(H713_MIPS_TRACE_OFF + 16)
#define H713_MIPS_SOURCE_TRACE_STAGE_OFF	(H713_MIPS_TRACE_OFF + 0x20)
#define H713_MIPS_SOURCE_TRACE_EVENT_OFF	(H713_MIPS_TRACE_OFF + 0x24)
#define H713_MIPS_SOURCE_TRACE_NEW_OFF	(H713_MIPS_TRACE_OFF + 0x28)
#define H713_MIPS_SOURCE_TRACE_OLD_OFF	(H713_MIPS_TRACE_OFF + 0x2c)
#define H713_MIPS_SOURCE_TRACE_QUEUE_OFF	(H713_MIPS_TRACE_OFF + 0x30)
#define H713_MIPS_SOURCE_TRACE_WORKER_OFF (H713_MIPS_TRACE_OFF + 0x34)
#define H713_MIPS_VP_INIT_TRACE_OFF	(H713_MIPS_TRACE_OFF + 0x38)
#define H713_MIPS_COMM_TRACE_MAGIC	0x434f4d4d
#define H713_MIPS_STABILITY_SECONDS	60
#define H713_MIPS_DIAG_OFF		0x00041000UL
#define H713_MIPS_DIAG_HEARTBEAT_OFF	(H713_MIPS_DIAG_OFF + 0x00)
#define H713_MIPS_DIAG_EXCEPTION_OFF	(H713_MIPS_DIAG_OFF + 0x04)
#define H713_MIPS_DIAG_STATUS_OFF	(H713_MIPS_DIAG_OFF + 0x08)
#define H713_MIPS_DIAG_CAUSE_OFF	(H713_MIPS_DIAG_OFF + 0x0c)
#define H713_MIPS_DIAG_EPC_OFF		(H713_MIPS_DIAG_OFF + 0x10)
#define H713_MIPS_DIAG_BADVADDR_OFF	(H713_MIPS_DIAG_OFF + 0x14)
#define H713_MIPS_DIAG_EXCEPTION_GENERAL	1
#define H713_MIPS_DIAG_EXCEPTION_CACHE	2

/*
 * Display-fabric prerequisites surrounding MIPS release. Several blocks wedge
 * the interconnect if accessed before the complete parent/module clock tree is
 * enabled. This exact subset was verified with MIPS running and across a Linux
 * handoff with Panfrost enabled.
 */
#define H713_DISPLAY_PLL_VIDEO2_REG	0x02001050UL
#define H713_DISPLAY_DEINT_CLK_REG	0x02001db0UL
#define H713_DISPLAY_PANEL_CLK_REG	0x02001db4UL
#define H713_DISPLAY_SVP_DTL_CLK_REG	0x02001db8UL
#define H713_DISPLAY_AFBD_CLK_REG	0x02001dc0UL
#define H713_DISPLAY_BGR_REG		0x02001dd8UL
#define H713_DISPLAY_TOP_REG		0x05700000UL
#define H713_DISPLAY_MIXER_CTRL_REG	0x0525c038UL

#define H713_DISPLAY_MIXER_CTRL_VALUE	0x00000100

#define H713_TVCAP_TCD3_CLK_REG		0x02001d6cUL
#define H713_TVCAP_VINCAP_DMA_CLK_REG	0x02001d74UL
#define H713_TVCAP_BUS_CLK_REG		0x02001d80UL
#define H713_TVCAP_HDMI_AUDIO_CLK_REG	0x02001d84UL
#define H713_TVCAP_BGR_REG		0x02001d88UL

/*
 * The TV power domains: the HDMI-RX PHY (0x06840000) and INCAP sit in TVCAP,
 * and the firmware initialises its receiver the moment it runs.
 *
 * Model: the kernel's sun50i-h713-ppu.c, the stock DTB (power-management@
 * ff000000 reg <0x07001000 0x400>, children pd_gpu@0 pd_tvfe@1 pd_tvcap@2
 * pd_ve@3 pd_av1@4) and the stock driver (sunxi_pd_power/tv303_pmu in the
 * HY310 vmlinux): register = base + offset + (domain_id << 7), PWR_CTRL 0x20
 * (1 = on, 2 = off), STATUS 0x24 with bit 3 set while the sequencer is busy,
 * bit 1 once a transition completed and bits [17:16] = 01b when on. The
 * PPU's own bus gate and reset are R_CCU 0x1ac bits 0/16, which is where the
 * stock DTB's "ppu" clock (index 15) and "ppu_rst" reset (index 6) land.
 */
#define H713_R_PPU_BGR_REG		0x070101acUL
#define H713_PPU_BASE			0x07001000UL
#define H713_PPU_DOMAIN_REG(d, off)	(H713_PPU_BASE + (off) + ((d) << 7))
#define H713_PPU_WAIT_MODE(d)		H713_PPU_DOMAIN_REG(d, 0x14)
#define H713_PPU_PWR_OFF_DELAY(d)	H713_PPU_DOMAIN_REG(d, 0x18)
#define H713_PPU_PWR_ON_DELAY(d)	H713_PPU_DOMAIN_REG(d, 0x1c)
#define H713_PPU_PWR_CTRL(d)		H713_PPU_DOMAIN_REG(d, 0x20)
#define H713_PPU_STATUS(d)		H713_PPU_DOMAIN_REG(d, 0x24)
#define H713_PPU_CMD_ON			1
#define H713_PPU_STATUS_DONE		BIT(1)
#define H713_PPU_STATUS_BUSY		BIT(3)
#define H713_PPU_STATUS_STATE_MASK	(BIT(17) | BIT(16))
#define H713_PPU_STATUS_STATE_ON	BIT(16)
#define H713_PPU_WAIT_MODE_INIT		0x00000008
#define H713_PPU_DELAY_INIT		0x00080808
#define H713_PPU_DOMAIN_TVFE		1
#define H713_PPU_DOMAIN_TVCAP		2
#define H713_PPU_TIMEOUT_US		10000

/*
 * Board B's panel description, as stock assembles it.
 *
 * Stock parses its runtime DT into a flat 35-entry u32 array, then overwrites
 * the same array from /panel_config.ini -- the two name tables at stock
 * 0x4a05ae30 (DT) and 0x4a05a4fc (INI) are index-for-index parallel, which is
 * what makes the override work. The values below are that merged result:
 * panel_config.ini (Reserve0_a, sha256 7bffff88..., extracted to
 * local/mips-display/board-b-mips/panel_config.ini) over the runtime TOC1 DT.
 *
 * Only two fields actually differ between the two sources -- dual_port and
 * ssc_en -- and both are recorded here at their post-INI value.
 *
 * PanelLvds0Pol/PanelLvds1Pol are absent from *both* sources, so stock leaves
 * their array slots untouched and its patch helper skips them. They are
 * deliberately not modelled here.
 */
struct h713_panel_cfg {
	u32 mapping;		/* DT panel_protocol      */
	u32 color_depth;		/* DT panel_bitwidth      */
	u32 odd_even;		/* DT panel_data_swap     */
	u32 dual_port;		/* DT 1 -> INI 0          */
	u32 mirror_mode;
	u32 inv_de, inv_hsync, inv_vsync, inv_dclk;
	u32 de_current, odd_current, even_current;
	u32 ssc_en;		/* DT 1 -> INI 0          */
	u32 pll_n_plus_1;	/* display PLL 0x058c0014[15:8] + 1 */
	/*
	 * The raster totals in panel_config.ini convention, i.e. the raw
	 * HTotal/VTotal the ini states. Both rows of the table below are
	 * written that way; the two consumers apply their own convention:
	 *
	 *  - the mixer and DE records, 0x0525c000 and 0x0524c010, hold total
	 *    MINUS ONE, so the patch table subtracts one there. A live HY310
	 *    reads 0x045f084f = 2127/1119 for an ini of 2128/1120, and the
	 *    vendor's own de[0] record carries 04640897 = 2199x1124 for the
	 *    standard 1080p60 raster of 2200x1125.
	 *  - the TCON, 0x05880020, takes the raw totals: the vendor's own
	 *    timing record for the 1280x720 panel writes 02f80550 for an ini
	 *    of 1360/760.
	 *
	 * Keeping one convention in the rows is what the board B repair needed:
	 * its row stated the raw 1360/760 while the HY310's stated the already
	 * decremented 2127/1119, so its mixer and DE ran 1361x761 against a
	 * TCON at 1360x760 (A10 1.3).
	 */
	u32 htotal, vtotal, hsync, vsync, hbp, vbp, width;
	/*
	 * PanelHsyncPol / PanelVsyncPol of panel_config.ini: 1 on the HY310,
	 * 0 on the 1280x720 panel. They drive bit 31 of the TCON's 0x0588002c
	 * and 0x05880030, which the latch below used to hardcode as set for
	 * every board it wrote. What that bit does is still an open question;
	 * the only claim made here is stock's own: the vendor timing record of
	 * a 0/0 panel writes both words with bit 31 clear (00000014 /
	 * 00010003), and an HY310 (1/1) is left running them set.
	 */
	u32 hsync_pol, vsync_pol;
	/*
	 * Active height. Not derivable from the fields above -- the register
	 * table never carries it -- but the OSD surface has to be sized
	 * somehow, so name it.
	 */
	u32 height;
	/*
	 * 0x05800000[4:3]. This used to be fed from color_depth, which cannot
	 * be what the field wants: 8 & 3 is zero on any board, so the write
	 * carried no information. It reads as an encoding selector like the
	 * XML's lvds_format. Board B keeps the zero it has been running;
	 * 1 is what an HY310's stock bootloader leaves there.
	 */
	u32 lvds_bitsel;
	/* 0x0528008c, the layer's pixel X origin. */
	u32 layer_x;
	/*
	 * 0x058c0018, the spread-spectrum waveform, and the mask that says
	 * whether this panel has one at all. A zero mask means the record is
	 * left exactly as the vendor tables have it, which is the only honest
	 * default for a board whose value has never been read.
	 */
	u32 ssc_mask;
	u32 ssc_reg;
	/*
	 * Whether this panel writes the active height into 0x05280084[31:16].
	 * Zero leaves the record alone, which is what board B's own note asks
	 * for: "Do not change it on this reasoning alone."
	 */
	u32 layer_h_mask;
};

static const struct h713_panel_cfg h713_panel_cfg_board_b = {
	.mapping = 0, .color_depth = 8, .odd_even = 0,
	.dual_port = 0, .mirror_mode = 0,
	.inv_de = 0, .inv_hsync = 0, .inv_vsync = 0, .inv_dclk = 1,
	.de_current = 47, .odd_current = 7, .even_current = 7,
	.ssc_en = 0,
	/*
	 * The vendor table leaves the display PLL at N+1 = 43, which is
	 * 24 * 43 = 1032 MHz and, through the measured /14, 73.71 MHz of DCLK
	 * against the 62 MHz panel_config.ini asks for -- 18.9% fast. Sweeping
	 * the PLL with the chroma checker found the panel decodes cleanly at
	 * N+1 = 36: 864 MHz, 61.71 MHz of DCLK, 0.46% low, 59.71 Hz. At 43 the
	 * projected image is a uniform blur; at 36 it is a crisp checkerboard.
	 * That sweep is also what established K = 14 in the first place.
	 */
	.pll_n_plus_1 = 36,
	/* ini convention: HTotal 1360, VTotal 760; the DE pair gets 1359/759. */
	.htotal = 1360, .vtotal = 760, .hsync = 20, .vsync = 2,
	.hbp = 40, .vbp = 20, .width = 1280, .height = 720,
	/* panel_config.ini: PanelHsyncPol 0, PanelVsyncPol 0. */
	.hsync_pol = 0, .vsync_pol = 0,
	/* All three chosen so this board's registers do not move. */
	.lvds_bitsel = 0, .layer_x = 0, .ssc_mask = 0, .ssc_reg = 0,
	.layer_h_mask = 0,
};

/*
 * The HY310's panel: 1920x1080, project ID 0x30.
 *
 * Everything below is read off a live stock bootloader on that board while
 * its logo was on the wall. That is possible because its boot0 and ours sit
 * in different places on the eMMC, so both can be resident and one 32 KiB
 * write to LBA 16 switches between them.
 *
 *	0525c000  045f084f   2127 / 1119, i.e. 2128 x 1120 minus one
 *	0525c004  00002c05   hsync 44, vsync 5
 *	0525c01c  07800084   132 = 44 + 88
 *	0525c020  04380019    25 =  5 + 20
 *	0528008c  00000037   layer X origin 55
 *	05800000  01e0a40c   the [4:3] selector is 1
 *	058c0014  b9002800   N = 40, and bit 24 -- ssc_en -- set
 *	058c0018  c8d0362f   the spread-spectrum waveform
 *
 * The device's own display_cfg.xml agrees: hde 1920, vde 1080, htotal
 * typical 2128 (min 2044, max 2208), vtotal typical 1120 (min 1100, max
 * 1150), hs 44, vs 5, h_back_porch 88, pclk typical 143001600. It gives a
 * range and the firmware picks within it.
 *
 * The TCON meanwhile runs 2200 x 1125, in stock as much as here. The mixer
 * and the TCON are not meant to agree.
 *
 * PLL: 24 * 41 = 984 MHz. The stock boot log states it outright -- "ssc
 * percent:10 wave bottom:0x362f, wave step:0x8d, n:41, ssc_freq:31500
 * reg_value:0xc8d0362f" -- and spread spectrum is on, which is why ssc_reg
 * is carried rather than left at the vendor default.
 *
 * dual_port is 1 here where board B has 0.
 */
static const struct h713_panel_cfg h713_panel_cfg_hy310 = {
	.mapping = 0, .color_depth = 8, .odd_even = 0,
	.dual_port = 1, .mirror_mode = 0,
	.inv_de = 0, .inv_hsync = 0, .inv_vsync = 0, .inv_dclk = 1,
	.de_current = 47, .odd_current = 7, .even_current = 7,
	.ssc_en = 1,
	.pll_n_plus_1 = 41,
	/*
	 * ini convention, as the row above: display_cfg.xml states htotal 2128
	 * and vtotal 1120, and the patch table's minus-one puts the 2127/1119
	 * this board has always run into 0x0525c000 and 0x0524c010. The row
	 * used to carry the decremented pair, which is the convention slip
	 * board B's row fell over.
	 */
	.htotal = 2128, .vtotal = 1120, .hsync = 44, .vsync = 5,
	.hbp = 88, .vbp = 20, .width = 1920, .height = 1080,
	/* panel_config.ini: PanelHsyncPol 1, PanelVsyncPol 1. */
	.hsync_pol = 1, .vsync_pol = 1,
	.lvds_bitsel = 1, .layer_x = 55,
	.ssc_mask = 0xffffffff, .ssc_reg = 0xc8d0362f,
	.layer_h_mask = 0xffff,
};

/*
 * The panel, by declared project ID.
 *
 * The ID is what a board says about itself, in panel_config.ini and in the
 * name of the ProjectID_*.TSE group its firmware loads, and it is the only
 * thing on the device that names the panel. The display.bin digest does not:
 * one image, 22a7df11..., ships on the 720p HY300 T08 and on the 1080p HY350,
 * which declare 0x34 and 0x30 (doku/121 2, finding 3). The digest identifies a
 * firmware revision -- its size and its HDCP wait site -- and that is all.
 *
 * 0x34 is the 1280x720 single-port panel: this is the bench board's, and the
 * HY300 T08's panel_config.ini states exactly the same raster -- HTotal 1360,
 * VTotal 760, Hsync 20, Vsync 2, HBP 40, VBP 20, both polarities 0, DCLK 62
 * MHz, which is the 61.71 MHz the PLL sweep settled on. Its backlight is PWM
 * channel 5 at 40 kHz where the HY310 uses channel 2 at 25 kHz; nothing here
 * models that, and h713_disp_backlight_set() is still pinned to PWM2 on PB4.
 *
 * 0x30 is the 1920x1080 dual-port panel, as read off a live HY310.
 *
 * What the table does NOT claim is that a project ID fixes the raster across
 * boards: the HY350 declares 0x30 as well, and its panel_config.ini asks for
 * 2200x1125 at 148.5 MHz where the HY310 asks for 2128x1120 at 143.0 MHz.
 * Same resolution, same port count, different blanking. A board we have never
 * driven gets a profile of its own before it gets an entry here.
 */
static const struct {
	u32 project;
	const struct h713_panel_cfg *panel;
} h713_disp_panels[] = {
	{ 0x30, &h713_panel_cfg_hy310 },
	{ 0x34, &h713_panel_cfg_board_b },
};

static const struct h713_panel_cfg *h713_panel_by_project(u32 project)
{
	uint i;

	for (i = 0; i < ARRAY_SIZE(h713_disp_panels); i++)
		if (h713_disp_panels[i].project == project)
			return h713_disp_panels[i].panel;

	return NULL;
}

/*
 * The panel in force. Set from the table above once a display command knows
 * its project ID; board B's until then, so the diagnostics that run before any
 * selection keep the geometry they were written against.
 */
static const struct h713_panel_cfg *h713_disp_panel = &h713_panel_cfg_board_b;

/*
 * Everything that varies with the display.bin revision, in one place.
 *
 * These were three separate constants -- an expected size, a pinned digest and
 * a patch address -- so a board carrying a different firmware failed three
 * times in a row, each with its own message and each needing its own edit.
 * They are properties of the image, so keep them together and pick the row by
 * digest.
 */
struct h713_mips_fw_rev {
	/*
	 * A row describes an *image*, and nothing else. The name is for people
	 * -- the vendor device tree carries only model = "sun50iw12" on every
	 * board in this family, so the silkscreen is all there is to call them
	 * by -- and the three numbers are what the code can act on.
	 *
	 * What used to live here as well, the project ID and the panel, are
	 * properties of the board. They left because one image serves two of
	 * them: 22a7df11... runs the 720p HY300 T08 and the 1080p HY350. See
	 * the panel table above.
	 *
	 * hdcp_wait_va zero means the site has never been located in this
	 * image. It is searched for at run time either way; a non-zero value
	 * is the cross-check, not the input.
	 */
	const char *board;
	ulong size;
	ulong hdcp_wait_va;
	u8 digest[SHA256_SUM_LEN];
};

static const struct h713_mips_fw_rev h713_mips_fw_revs[] = {
	{
		/* Read from this board's own stock bootloader partition. */
		.board = "HY200 QZ713DF_A1",
		.size = 0x132910,
		.hdcp_wait_va = 0x4b13d6f8,
		.digest = {
			0x43, 0x80, 0xf1, 0xb3, 0xed, 0x7b, 0x62, 0xaa,
			0x50, 0x58, 0x2e, 0x7c, 0xb1, 0x6a, 0x87, 0xbd,
			0xfa, 0xce, 0x1b, 0x43, 0x00, 0x57, 0x8f, 0xe3,
			0x63, 0x1a, 0x41, 0x63, 0x54, 0xda, 0x30, 0xce,
		},
	},
	{
		/*
		 * The revision the note above called "a different firmware
		 * revision in a captured dump". It is indeed different, and it
		 * is what ships on an HY310: read off that board's own eMMC,
		 * matching its ProjectID_0x0030 TSE group. The HDCP wait
		 * address is the one already documented for it.
		 */
		.board = "HY310 (QZ713 V3.1)",
		.size = 0x132b18,
		.hdcp_wait_va = 0x4b13d0a4,
		.digest = {
			0x16, 0xc7, 0x4a, 0x28, 0x18, 0x7f, 0x34, 0x2d,
			0xe6, 0x57, 0x82, 0x8f, 0xab, 0x65, 0x14, 0x5b,
			0x14, 0x0a, 0xc9, 0x41, 0x1c, 0x40, 0xcc, 0xcc,
			0x02, 0xee, 0xd2, 0x50, 0x47, 0x47, 0x2e, 0xe9,
		},
	},
	{
		/*
		 * The ADT-3 family's image, from the HY300 T08 and HY350
		 * firmware images of 2026-09-14: byte-identical in both, and
		 * the reason a row cannot name a panel -- the T08 is 720p and
		 * the HY350 1080p. It carries 13 ProjectID descriptors, 0x30
		 * and 0x34 among them.
		 *
		 * Both ship it in vendor:/etc/display/mips/, not in
		 * boot-resource, and Android copies it to bootloader_a/_b
		 * (doku/121 2, finding 2).
		 *
		 * The wait site was located offline by the same search this
		 * code runs, over the image itself (umbau/work/A5).
		 */
		.board = "ADT-3 2024",
		.size = 0x131b20,
		.hdcp_wait_va = 0x4b13d1f0,
		.digest = {
			0x22, 0xa7, 0xdf, 0x11, 0x3f, 0xce, 0x3f, 0xa1,
			0x82, 0x92, 0x62, 0x68, 0xde, 0x8c, 0x75, 0x51,
			0xa1, 0x07, 0xf0, 0xc3, 0xbc, 0x29, 0x32, 0xf0,
			0x94, 0x0b, 0xd5, 0x8b, 0x8f, 0x42, 0x48, 0x35,
		},
	},
	{
		/*
		 * The HY300 Pro's, build 24-5-7, from the size and SHA-256 its
		 * owner reported in issue #1. Nobody here has the image; the
		 * wait site is the one the probe found on his device
		 * (issue #1, probe run of 2026-09-14), so the run-time search
		 * now has something to agree with.
		 *
		 * Without this row that board is refused on its size alone,
		 * before anything has been read -- which is the refusal
		 * doku/120 set out to remove.
		 */
		.board = "HY300 Pro",
		.size = 0x131f10,
		.hdcp_wait_va = 0x4b13d1b0,
		.digest = {
			0xcf, 0x96, 0x49, 0xbc, 0xc8, 0x4a, 0x11, 0x1c,
			0xe5, 0x90, 0xfc, 0x7a, 0xcd, 0xe7, 0x23, 0xc2,
			0x55, 0x57, 0xfd, 0x23, 0x32, 0xab, 0xbb, 0x9f,
			0xd1, 0x02, 0x25, 0x90, 0x5a, 0xae, 0x13, 0xa2,
		},
	},
	{
		/*
		 * From one of the two vendor "HY300 Pro+" images of 2025, the
		 * 0710 one: an LPDDR3 board at 720 MHz. Its display.bin is
		 * the image, and the image is all we have -- it is a firmware
		 * download, not a device anybody here or in issue #1 owns, so
		 * the name is the image's and not a board's.
		 *
		 * The wait site was located offline by the same search this
		 * code runs at run time, over the extracted file (file offset
		 * 0x3d538), so the search has something to agree with.
		 *
		 * Note what this row is *not*: it is not the HY300 Pro above.
		 * That board is DDR3 at 636 MHz and carries cf9649bc...; this
		 * is a different image on different memory, and the two share
		 * only a marketing name.
		 */
		.board = "HY300 Pro+ 2025 (LPDDR3)",
		.size = 0x1328f0,
		.hdcp_wait_va = 0x4b13d538,
		.digest = {
			0x46, 0x28, 0xcb, 0xaf, 0x8c, 0x93, 0xea, 0x12,
			0xec, 0x59, 0xe8, 0x71, 0xe4, 0x25, 0x0f, 0xac,
			0xf0, 0x77, 0x3d, 0xaa, 0xe7, 0x22, 0xb2, 0x51,
			0xc7, 0x3f, 0x1f, 0xad, 0x81, 0xe8, 0x0c, 0x2c,
		},
	},
};

/* Set by h713_mips_verify() once the image is identified. */
static const struct h713_mips_fw_rev *h713_mips_fw;

/*
 * Probe mode. The table above can only ever describe boards someone has held.
 * A third device meets it as three refusals in a row -- unknown size, unknown
 * digest, unknown patch site -- and every one of them fires before anything
 * has been learned, so the owner is told "no" and nobody finds out what the
 * board is. Under this flag those become findings instead: the command reads,
 * hashes and reports, and refuses everything that would touch the panel or
 * start the coprocessor. Probing describes, it does not experiment.
 */
static bool h713_probe_mode;

/*
 * The size is needed before the image can be hashed, so it starts at the
 * first row's value and is replaced by the file's own size once that is
 * known. h713_mips_clear_workspace() erases from the end of the firmware, and
 * a value that is too small takes the tail of the image with it.
 */
static ulong h713_mips_fw_size = H713_MIPS_FW_SIZE;

/*
 * Accept any size a known revision declares -- all five of them -- and let the
 * digest decide which one it is. Refusing here on one revision's size turns an
 * identity check into a size check. Both loaders had arrived at the same
 * conclusion separately, in two copies of the same loop, so they now ask the
 * same question in one place.
 *
 * This is the first gate a new board meets, so a row that exists only as a
 * size and a digest (the HY300 Pro's) still earns its keep: without it that
 * board is turned away before anything has been read from it.
 */
static int h713_mips_accept_size(ulong len)
{
	uint i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_fw_revs); i++) {
		if (len != h713_mips_fw_revs[i].size)
			continue;
		h713_mips_fw_size = len;
		return 0;
	}

	if (!h713_probe_mode) {
		printf("H713 MIPS: rejected size 0x%lx, no revision declares it\n",
		       len);
		return -EINVAL;
	}

	printf("H713 MIPS: size 0x%lx is not in the table -- new revision\n",
	       len);
	h713_mips_fw_size = len;

	return 0;
}

/*
 * Known stock boot logos. The asset is board-specific in geometry as well as
 * content, and the geometry is simply each board's own panel. A single pinned
 * size plus a single pinned digest rejected the other board twice before
 * anything ever looked at the file.
 *
 * The table identifies, it does not gate: a logo that matches no row is still
 * published, because the product boot has to show whatever the owner put at
 * /boot/bootlogo.bmp on a board nobody has hashed.
 */
static const struct {
	const char *board;
	u8 digest[SHA256_SUM_LEN];
} h713_vendor_bootlogos[] = {
	{
		/* bootloader_a/bootlogo.bmp, 2026-07-05 full-board dump. */
		"HY200 QZ713DF_A1", {
			0xe8, 0x12, 0xcd, 0x92, 0x8c, 0x67, 0xb8, 0x96,
			0x08, 0xc7, 0x42, 0x4a, 0xc8, 0x00, 0x66, 0xc1,
			0x96, 0x33, 0xe7, 0x6b, 0x15, 0x03, 0x78, 0xab,
			0xf4, 0xde, 0x9d, 0x62, 0x69, 0x8e, 0xb2, 0x2c,
		},
	},
	{
		/* mips/bootlogo.bmp on an HY310: 1920x1080, 6220854 bytes. */
		"HY310 (QZ713 V3.1)", {
			0x96, 0x84, 0xef, 0x71, 0x48, 0x3e, 0xb1, 0x99,
			0x01, 0xad, 0xf2, 0x5e, 0x55, 0xad, 0x61, 0x7e,
			0xb2, 0xb4, 0x1d, 0x70, 0x65, 0xd9, 0xe2, 0x57,
			0xe1, 0x08, 0x1c, 0x5c, 0x26, 0x09, 0x50, 0x33,
		},
	},
};

static bool h713_display_prepared;
static bool h713_panel_test_ran;

/*
 * Set only by fb-band, which screens candidate offset registers and needs
 * 0x0528008c holding its power-on value for the control step. A flag rather
 * than a parameter because h713_disp_run and h713_disp_panel_test are already
 * long in the tooth for arguments, and this is a diagnostic exception rather
 * than a mode of operation.
 */
static bool h713_disp_keep_layer_xoff;
static bool h713_comm_trace_active;

/* Previous value of every trace slot, so streaming reports only changes. */
static u32 trace_shadow[H713_MIPS_TRACE_COUNT];

struct h713_mips_patch {
	ulong addr;
	u32 expected;
	u32 replacement;
};

/*
 * Gate-2 stability instrumentation for the authenticated 4380f1b3... image.
 *
 * The timer trampoline preserves the displaced tick increment and store, then
 * publishes the new ThreadX tick through the MIPS uncached 0xae34xxxx alias.
 * The two exception trampolines record CP0 state before entering the stock
 * fatal handlers. EBase is set to 0x8b101000 at raw +0xb11ac..+0xb11b4, so
 * raw +0x1100 and +0x1180 are the live cache/general exception vectors.
 */
static const struct h713_mips_patch h713_mips_stability_patches[] = {
	/* Timer cave at raw +0x300. */
	{ 0x4b100300, 0x00000000, 0x26730001 }, /* addiu s3, s3, 1 */
	{ 0x4b100304, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100308, 0x00000000, 0xaf531000 }, /* sw s3, 0x1000(k0) */
	{ 0x4b10030c, 0x00000000, 0x0ac412e1 }, /* j 0x8b104b84 */
	{ 0x4b100310, 0x00000000, 0xac532cc0 }, /* sw s3, 0x2cc0(v0) */
	{ 0x4b104b7c, 0x26730001, 0x0ac400c0 }, /* j 0x8b100300 */

	/* General-exception cave at raw +0x320; type 1 is stored last. */
	{ 0x4b100320, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100324, 0x00000000, 0x401b6000 }, /* mfc0 k1, Status */
	{ 0x4b100328, 0x00000000, 0xaf5b1008 },
	{ 0x4b10032c, 0x00000000, 0x401b6800 }, /* mfc0 k1, Cause */
	{ 0x4b100330, 0x00000000, 0xaf5b100c },
	{ 0x4b100334, 0x00000000, 0x401b7000 }, /* mfc0 k1, EPC */
	{ 0x4b100338, 0x00000000, 0xaf5b1010 },
	{ 0x4b10033c, 0x00000000, 0x401b4000 }, /* mfc0 k1, BadVAddr */
	{ 0x4b100340, 0x00000000, 0xaf5b1014 },
	{ 0x4b100344, 0x00000000, 0x341b0001 },
	{ 0x4b100348, 0x00000000, 0xaf5b1004 },
	{ 0x4b10034c, 0x00000000, 0x0ac56f4a }, /* j 0x8b15bd28 */
	{ 0x4b100350, 0x00000000, 0x00000000 },
	{ 0x4b101180, 0x0ac56f4a, 0x0ac400c8 }, /* j 0x8b100320 */

	/* Cache-error cave at raw +0x360; EPC field receives ErrorEPC. */
	{ 0x4b100360, 0x00000000, 0x3c1aae34 }, /* lui k0, 0xae34 */
	{ 0x4b100364, 0x00000000, 0x401b6000 }, /* mfc0 k1, Status */
	{ 0x4b100368, 0x00000000, 0xaf5b1008 },
	{ 0x4b10036c, 0x00000000, 0x401b6800 }, /* mfc0 k1, Cause */
	{ 0x4b100370, 0x00000000, 0xaf5b100c },
	{ 0x4b100374, 0x00000000, 0x401bf000 }, /* mfc0 k1, ErrorEPC */
	{ 0x4b100378, 0x00000000, 0xaf5b1010 },
	{ 0x4b10037c, 0x00000000, 0x401b4000 }, /* mfc0 k1, BadVAddr */
	{ 0x4b100380, 0x00000000, 0xaf5b1014 },
	{ 0x4b100384, 0x00000000, 0x341b0002 },
	{ 0x4b100388, 0x00000000, 0xaf5b1004 },
	{ 0x4b10038c, 0x00000000, 0x0ac56f79 }, /* j 0x8b15bde4 */
	{ 0x4b100390, 0x00000000, 0x00000000 },
	{ 0x4b101100, 0x0ac56f79, 0x0ac400d8 }, /* j 0x8b100360 */
};

/*
 * Legacy trace for display.bin 16c74a28..., retained only as a reverse-
 * engineering record. The board's 4380f1b3... image moved the instrumented
 * functions, so none of these sites may be installed in executable DRAM.
 */
#if 0
/*
 * Volatile display.bin trace used only by "probe-trace". The pristine image is
 * authenticated before these words are installed in DRAM. Three tiny caves
 * and two inline stores mark:
 *
 *  1. share address absent (the polling path)
 *  2. share address and size accepted
 *  3. ARM CPU_READY accepted
 *  4. CPU_COMM hardware spinlock 0 acquired
 *  5. slave-side CPU_COMM initialization entered
 *  6. ThreadX application entry reached
 *  7. early application wrapper entered
 *  8. application byte-pool creation succeeded
 *  9. share-register reader called
 * 10. C runtime handed off to platform initialization
 * 11. constructor-table initialization completed
 * 12. early OS initialization completed
 * 13. platform service initialization completed
 * 14. application/thread construction entered
 * 15. application/thread construction completed
 * 16-25. successive calls within application/thread construction entered
 * 26. display object allocation entered
 * 27. display object construction entered
 * 28. display object resource discovery entered
 * 29-31. resource discovery's first three calls entered
 * 32-42. successive early system-initialization calls entered
 * 43-61. remaining early system-initialization calls entered
 * 62-67. device-manager allocation and factory calls entered
 * 68-83. HDMI receiver factory and constructor calls entered
 * 84-92. first HDMI receiver port's construction calls entered
 * 93-96. HDMI receiver MMIO address validation and first byte load
 * 97-102. HDMI receiver helper returns, tail calls, write, and timer entry
 * 103-106. polling-loop timer entry, return, current tick, and initial tick
 * 107-110. platform-registration callback targets and table positions
 * 111-112. marker-49 singleton allocation and construction
 * 130-132. the four-registration group's ids 0x130, 3, and 8 returning
 *
 * The marker stores target the firmware's uncached 0xae340000 alias, visible
 * to the ARM at H713_MIPS_SHMEM_ADDR + H713_MIPS_TRACE_OFF.
 */
static const struct h713_mips_patch h713_mips_trace_patches[] = {
	/* Cave 0x8b1002c4: marker 1, then return after the skipped log call. */
	{ 0x4b1002c4, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002c8, 0x00000000, 0x341b0001 },
	{ 0x4b1002cc, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002d0, 0x00000000, 0x0ac48e38 },
	{ 0x4b1002d4, 0x00000000, 0x00000000 },
	{ 0x4b1238d8, 0x0ec5401a, 0x0ac400b1 },

	/* Cave 0x8b100500: marker 2, then return after the skipped log call. */
	{ 0x4b100500, 0x00000000, 0x3c1aae34 },
	{ 0x4b100504, 0x00000000, 0x341b0002 },
	{ 0x4b100508, 0x00000000, 0xaf5b0004 },
	{ 0x4b10050c, 0x00000000, 0x0ac48e59 },
	{ 0x4b100510, 0x00000000, 0x00000000 },
	{ 0x4b12395c, 0x0ec5401a, 0x0ac40140 },

	/* Marker 3 in the disposable argument setup for the return log. */
	{ 0x4b12397c, 0x2687fd18, 0x3c1aae34 },
	{ 0x4b123980, 0x2666fa64, 0x341b0003 },
	{ 0x4b123988, 0x3c028b1f, 0xaf5b0008 },
	{ 0x4b12398c, 0x2442e040, 0x00000000 },
	{ 0x4b123990, 0xafa20014, 0x00000000 },

	/* Marker 4 immediately after CPU_COMM hardware spinlock 0 succeeds. */
	{ 0x4b11b07c, 0x8fc20090, 0x3c1aae34 },
	{ 0x4b11b080, 0x8fc475b8, 0x341b0004 },
	{ 0x4b11b084, 0x02e03825, 0xaf5b000c },

	/* Cave 0x8b100520: marker 5, then tail-call the original function. */
	{ 0x4b100520, 0x00000000, 0x3c1aae34 },
	{ 0x4b100524, 0x00000000, 0x341b0005 },
	{ 0x4b100528, 0x00000000, 0xaf5b0010 },
	{ 0x4b10052c, 0x00000000, 0x0ac466a2 },
	{ 0x4b100530, 0x00000000, 0x00000000 },
	{ 0x4b11b220, 0x0ec466a2, 0x0ec40148 },

	/* Cave 0x8b100540: marker 6, then tail-call the original timer read. */
	{ 0x4b100540, 0x00000000, 0x3c1aae34 },
	{ 0x4b100544, 0x00000000, 0x341b0006 },
	{ 0x4b100548, 0x00000000, 0xaf5b0014 },
	{ 0x4b10054c, 0x00000000, 0x0ac56f65 },
	{ 0x4b100550, 0x00000000, 0x00000000 },
	{ 0x4b1525a4, 0x0ec56f65, 0x0ec40150 },

	/* Cave 0x8b100560: marker 7, then enter the original wrapper. */
	{ 0x4b100560, 0x00000000, 0x3c1aae34 },
	{ 0x4b100564, 0x00000000, 0x341b0007 },
	{ 0x4b100568, 0x00000000, 0xaf5b0018 },
	{ 0x4b10056c, 0x00000000, 0x0ac4908f },
	{ 0x4b100570, 0x00000000, 0x00000000 },
	{ 0x4b1525ac, 0x0ec4908f, 0x0ec40158 },

	/* Cave 0x8b100590: marker 8 on the byte-pool success path. */
	{ 0x4b100590, 0x00000000, 0x3c1aae34 },
	{ 0x4b100594, 0x00000000, 0x341b0008 },
	{ 0x4b100598, 0x00000000, 0xaf5b001c },
	{ 0x4b10059c, 0x00000000, 0x0ac490c6 },
	{ 0x4b1005a0, 0x00000000, 0x00000000 },
	{ 0x4b124310, 0x0ec5401a, 0x0ac40164 },

	/* Cave 0x8b1005b0: marker 9, then call the share-register reader. */
	{ 0x4b1005b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005b4, 0x00000000, 0x341b0009 },
	{ 0x4b1005b8, 0x00000000, 0xaf5b0020 },
	{ 0x4b1005bc, 0x00000000, 0x0ac48e0a },
	{ 0x4b1005c0, 0x00000000, 0x00000000 },
	{ 0x4b12431c, 0x0ec48e0a, 0x0ec4016c },

	/* Caves 0x8b1005d0..0x8b100670 trace the pre-scheduler calls. */
	{ 0x4b1005d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005d4, 0x00000000, 0x341b000a },
	{ 0x4b1005d8, 0x00000000, 0xaf5b0024 },
	{ 0x4b1005dc, 0x00000000, 0x0ac407c1 },
	{ 0x4b1005e0, 0x00000000, 0x00000000 },
	{ 0x4b1b0940, 0x0100f809, 0x0ec40174 },

	{ 0x4b1005f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005f4, 0x00000000, 0x341b000b },
	{ 0x4b1005f8, 0x00000000, 0xaf5b0028 },
	{ 0x4b1005fc, 0x00000000, 0x0ac40b86 },
	{ 0x4b100600, 0x00000000, 0x00000000 },
	{ 0x4b101f24, 0x0ec40b86, 0x0ec4017c },

	{ 0x4b100610, 0x00000000, 0x3c1aae34 },
	{ 0x4b100614, 0x00000000, 0x341b000c },
	{ 0x4b100618, 0x00000000, 0xaf5b002c },
	{ 0x4b10061c, 0x00000000, 0x0ac51e54 },
	{ 0x4b100620, 0x00000000, 0x00000000 },
	{ 0x4b101f2c, 0x0ec51e54, 0x0ec40184 },

	{ 0x4b100630, 0x00000000, 0x3c1aae34 },
	{ 0x4b100634, 0x00000000, 0x341b000d },
	{ 0x4b100638, 0x00000000, 0xaf5b0030 },
	{ 0x4b10063c, 0x00000000, 0x0ac61091 },
	{ 0x4b100640, 0x00000000, 0x00000000 },
	{ 0x4b101f34, 0x0ec61091, 0x0ec4018c },

	{ 0x4b100650, 0x00000000, 0x3c1aae34 },
	{ 0x4b100654, 0x00000000, 0x341b000e },
	{ 0x4b100658, 0x00000000, 0xaf5b0034 },
	{ 0x4b10065c, 0x00000000, 0x0ac54b37 },
	{ 0x4b100660, 0x00000000, 0x00000000 },
	{ 0x4b101f3c, 0x0ec54b37, 0x0ec40194 },

	{ 0x4b100670, 0x00000000, 0x3c1aae34 },
	{ 0x4b100674, 0x00000000, 0x341b000f },
	{ 0x4b100678, 0x00000000, 0xaf5b0038 },
	{ 0x4b10067c, 0x00000000, 0x0ac415e4 },
	{ 0x4b100680, 0x00000000, 0x00000000 },
	{ 0x4b101f4c, 0x0ec415e4, 0x0ec4019c },

	/* Caves 0x8b100690..0x8b1007b0 trace construction's calls. */
	{ 0x4b100690, 0x00000000, 0x3c1aae34 },
	{ 0x4b100694, 0x00000000, 0x341b0010 },
	{ 0x4b100698, 0x00000000, 0xaf5b003c },
	{ 0x4b10069c, 0x00000000, 0x0ac672b9 },
	{ 0x4b1006a0, 0x00000000, 0x00000000 },
	{ 0x4b152ce4, 0x0ec672b9, 0x0ec401a4 },

	{ 0x4b1006b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006b4, 0x00000000, 0x341b0011 },
	{ 0x4b1006b8, 0x00000000, 0xaf5b0040 },
	{ 0x4b1006bc, 0x00000000, 0x0ac439dc },
	{ 0x4b1006c0, 0x00000000, 0x00000000 },
	{ 0x4b152cf0, 0x0ec439dc, 0x0ec401ac },

	{ 0x4b1006d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006d4, 0x00000000, 0x341b0012 },
	{ 0x4b1006d8, 0x00000000, 0xaf5b0044 },
	{ 0x4b1006dc, 0x00000000, 0x0ac43503 },
	{ 0x4b1006e0, 0x00000000, 0x00000000 },
	{ 0x4b152cf8, 0x0ec43503, 0x0ec401b4 },

	{ 0x4b1006f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006f4, 0x00000000, 0x341b0013 },
	{ 0x4b1006f8, 0x00000000, 0xaf5b0048 },
	{ 0x4b1006fc, 0x00000000, 0x0ac43511 },
	{ 0x4b100700, 0x00000000, 0x00000000 },
	{ 0x4b152d00, 0x0ec43511, 0x0ec401bc },

	{ 0x4b100710, 0x00000000, 0x3c1aae34 },
	{ 0x4b100714, 0x00000000, 0x341b0014 },
	{ 0x4b100718, 0x00000000, 0xaf5b004c },
	{ 0x4b10071c, 0x00000000, 0x0ac56698 },
	{ 0x4b100720, 0x00000000, 0x00000000 },
	{ 0x4b152d10, 0x0ec56698, 0x0ec401c4 },

	{ 0x4b100730, 0x00000000, 0x3c1aae34 },
	{ 0x4b100734, 0x00000000, 0x341b0015 },
	{ 0x4b100738, 0x00000000, 0xaf5b0050 },
	{ 0x4b10073c, 0x00000000, 0x0ac54e20 },
	{ 0x4b100740, 0x00000000, 0x00000000 },
	{ 0x4b152d18, 0x0ec54e20, 0x0ec401cc },

	{ 0x4b100750, 0x00000000, 0x3c1aae34 },
	{ 0x4b100754, 0x00000000, 0x341b0016 },
	{ 0x4b100758, 0x00000000, 0xaf5b0054 },
	{ 0x4b10075c, 0x00000000, 0x0ac5440b },
	{ 0x4b100760, 0x00000000, 0x00000000 },
	{ 0x4b152d20, 0x0ec5440b, 0x0ec401d4 },

	{ 0x4b100770, 0x00000000, 0x3c1aae34 },
	{ 0x4b100774, 0x00000000, 0x341b0017 },
	{ 0x4b100778, 0x00000000, 0xaf5b0058 },
	{ 0x4b10077c, 0x00000000, 0x0ac60af9 },
	{ 0x4b100780, 0x00000000, 0x00000000 },
	{ 0x4b152d28, 0x0ec60af9, 0x0ec401dc },

	{ 0x4b100790, 0x00000000, 0x3c1aae34 },
	{ 0x4b100794, 0x00000000, 0x341b0018 },
	{ 0x4b100798, 0x00000000, 0xaf5b005c },
	{ 0x4b10079c, 0x00000000, 0x0ac54acb },
	{ 0x4b1007a0, 0x00000000, 0x00000000 },
	{ 0x4b152d30, 0x0ec54acb, 0x0ec401e4 },

	{ 0x4b1007b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007b4, 0x00000000, 0x341b0019 },
	{ 0x4b1007b8, 0x00000000, 0xaf5b0060 },
	{ 0x4b1007bc, 0x00000000, 0x0ac56fed },
	{ 0x4b1007c0, 0x00000000, 0x00000000 },
	{ 0x4b152d54, 0x0ec56fed, 0x0ec401ec },

	/* Trace the allocation and construction calls below marker 17. */
	{ 0x4b1007d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007d4, 0x00000000, 0x341b001a },
	{ 0x4b1007d8, 0x00000000, 0xaf5b0064 },
	{ 0x4b1007dc, 0x00000000, 0x0ac6b54d },
	{ 0x4b1007e0, 0x00000000, 0x00000000 },
	{ 0x4b10e7b0, 0x0ec6b54d, 0x0ec401f4 },

	{ 0x4b1007f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007f4, 0x00000000, 0x341b001b },
	{ 0x4b1007f8, 0x00000000, 0xaf5b0068 },
	{ 0x4b1007fc, 0x00000000, 0x0ac439c1 },
	{ 0x4b100800, 0x00000000, 0x00000000 },
	{ 0x4b10e7c0, 0x0ec439c1, 0x0ec401fc },

	{ 0x4b100810, 0x00000000, 0x3c1aae34 },
	{ 0x4b100814, 0x00000000, 0x341b001c },
	{ 0x4b100818, 0x00000000, 0xaf5b006c },
	{ 0x4b10081c, 0x00000000, 0x0ac43554 },
	{ 0x4b100820, 0x00000000, 0x00000000 },
	{ 0x4b10e73c, 0x0ec43554, 0x0ec40204 },

	/* Trace the first resource-discovery calls below marker 28. */
	{ 0x4b100830, 0x00000000, 0x3c1aae34 },
	{ 0x4b100834, 0x00000000, 0x341b001d },
	{ 0x4b100838, 0x00000000, 0xaf5b0070 },
	{ 0x4b10083c, 0x00000000, 0x0ac452a3 },
	{ 0x4b100840, 0x00000000, 0x00000000 },
	{ 0x4b10d584, 0x0ec452a3, 0x0ec4020c },

	{ 0x4b100850, 0x00000000, 0x3c1aae34 },
	{ 0x4b100854, 0x00000000, 0x341b001e },
	{ 0x4b100858, 0x00000000, 0xaf5b0074 },
	{ 0x4b10085c, 0x00000000, 0x0ac4573b },
	{ 0x4b100860, 0x00000000, 0x00000000 },
	{ 0x4b10d598, 0x0ec4573b, 0x0ec40214 },

	{ 0x4b100870, 0x00000000, 0x3c1aae34 },
	{ 0x4b100874, 0x00000000, 0x341b001f },
	{ 0x4b100878, 0x00000000, 0xaf5b0078 },
	{ 0x4b10087c, 0x00000000, 0x0ac45030 },
	{ 0x4b100880, 0x00000000, 0x00000000 },
	{ 0x4b10d5a4, 0x0ec45030, 0x0ec4021c },

	/* Trace the early system-initialization calls below marker 24. */
	{ 0x4b100890, 0x00000000, 0x3c1aae34 },
	{ 0x4b100894, 0x00000000, 0x341b0020 },
	{ 0x4b100898, 0x00000000, 0xaf5b007c },
	{ 0x4b10089c, 0x00000000, 0x0ac5ff44 },
	{ 0x4b1008a0, 0x00000000, 0x00000000 },
	{ 0x4b152b64, 0x0ec5ff44, 0x0ec40224 },

	{ 0x4b1008b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008b4, 0x00000000, 0x341b0021 },
	{ 0x4b1008b8, 0x00000000, 0xaf5b0080 },
	{ 0x4b1008bc, 0x00000000, 0x0ac43500 },
	{ 0x4b1008c0, 0x00000000, 0x00000000 },
	{ 0x4b152b74, 0x0ec43500, 0x0ec4022c },

	{ 0x4b1008d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008d4, 0x00000000, 0x341b0022 },
	{ 0x4b1008d8, 0x00000000, 0xaf5b0084 },
	{ 0x4b1008dc, 0x00000000, 0x00400008 },
	{ 0x4b1008e0, 0x00000000, 0x00000000 },
	{ 0x4b152b90, 0x0040f809, 0x0ec40234 },

	{ 0x4b1008f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008f4, 0x00000000, 0x341b0023 },
	{ 0x4b1008f8, 0x00000000, 0xaf5b0088 },
	{ 0x4b1008fc, 0x00000000, 0x0ac43500 },
	{ 0x4b100900, 0x00000000, 0x00000000 },
	{ 0x4b152b98, 0x0ec43500, 0x0ec4023c },

	{ 0x4b100910, 0x00000000, 0x3c1aae34 },
	{ 0x4b100914, 0x00000000, 0x341b0024 },
	{ 0x4b100918, 0x00000000, 0xaf5b008c },
	{ 0x4b10091c, 0x00000000, 0x00400008 },
	{ 0x4b100920, 0x00000000, 0x00000000 },
	{ 0x4b152bb4, 0x0040f809, 0x0ec40244 },

	{ 0x4b100930, 0x00000000, 0x3c1aae34 },
	{ 0x4b100934, 0x00000000, 0x341b0025 },
	{ 0x4b100938, 0x00000000, 0xaf5b0090 },
	{ 0x4b10093c, 0x00000000, 0x0ac5ff56 },
	{ 0x4b100940, 0x00000000, 0x00000000 },
	{ 0x4b152bc4, 0x0ec5ff56, 0x0ec4024c },

	{ 0x4b100950, 0x00000000, 0x3c1aae34 },
	{ 0x4b100954, 0x00000000, 0x341b0026 },
	{ 0x4b100958, 0x00000000, 0xaf5b0094 },
	{ 0x4b10095c, 0x00000000, 0x0ac5ff56 },
	{ 0x4b100960, 0x00000000, 0x00000000 },
	{ 0x4b152bd4, 0x0ec5ff56, 0x0ec40254 },

	{ 0x4b100970, 0x00000000, 0x3c1aae34 },
	{ 0x4b100974, 0x00000000, 0x341b0027 },
	{ 0x4b100978, 0x00000000, 0xaf5b0098 },
	{ 0x4b10097c, 0x00000000, 0x0ac625f6 },
	{ 0x4b100980, 0x00000000, 0x00000000 },
	{ 0x4b152be0, 0x0ec625f6, 0x0ec4025c },

	{ 0x4b100990, 0x00000000, 0x3c1aae34 },
	{ 0x4b100994, 0x00000000, 0x341b0028 },
	{ 0x4b100998, 0x00000000, 0xaf5b009c },
	{ 0x4b10099c, 0x00000000, 0x0ac62345 },
	{ 0x4b1009a0, 0x00000000, 0x00000000 },
	{ 0x4b152be8, 0x0ec62345, 0x0ec40264 },

	{ 0x4b1009b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009b4, 0x00000000, 0x341b0029 },
	{ 0x4b1009b8, 0x00000000, 0xaf5b00a0 },
	{ 0x4b1009bc, 0x00000000, 0x0ac43500 },
	{ 0x4b1009c0, 0x00000000, 0x00000000 },
	{ 0x4b152bf0, 0x0ec43500, 0x0ec4026c },

	{ 0x4b1009d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009d4, 0x00000000, 0x341b002a },
	{ 0x4b1009d8, 0x00000000, 0xaf5b00a4 },
	{ 0x4b1009dc, 0x00000000, 0x00400008 },
	{ 0x4b1009e0, 0x00000000, 0x00000000 },
	{ 0x4b152c0c, 0x0040f809, 0x0ec40274 },

	/* Trace the remaining system-initialization calls below marker 42. */
	{ 0x4b1009f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009f4, 0x00000000, 0x341b002b },
	{ 0x4b1009f8, 0x00000000, 0xaf5b00a8 },
	{ 0x4b1009fc, 0x00000000, 0x0ac440d3 },
	{ 0x4b100a00, 0x00000000, 0x00000000 },
	{ 0x4b152c1c, 0x0ec440d3, 0x0ec4027c },

	{ 0x4b100a10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a14, 0x00000000, 0x341b002c },
	{ 0x4b100a18, 0x00000000, 0xaf5b00ac },
	{ 0x4b100a1c, 0x00000000, 0x0ac54a97 },
	{ 0x4b100a20, 0x00000000, 0x00000000 },
	{ 0x4b152c24, 0x0ec54a97, 0x0ec40284 },

	{ 0x4b100a30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a34, 0x00000000, 0x341b002d },
	{ 0x4b100a38, 0x00000000, 0xaf5b00b0 },
	{ 0x4b100a3c, 0x00000000, 0x0ac62571 },
	{ 0x4b100a40, 0x00000000, 0x00000000 },
	{ 0x4b152c2c, 0x0ec62571, 0x0ec4028c },

	{ 0x4b100a50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a54, 0x00000000, 0x341b002e },
	{ 0x4b100a58, 0x00000000, 0xaf5b00b4 },
	{ 0x4b100a5c, 0x00000000, 0x00400008 },
	{ 0x4b100a60, 0x00000000, 0x00000000 },
	{ 0x4b152c40, 0x0040f809, 0x0ec40294 },

	{ 0x4b100a70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a74, 0x00000000, 0x341b002f },
	{ 0x4b100a78, 0x00000000, 0xaf5b00b8 },
	{ 0x4b100a7c, 0x00000000, 0x0ac69cb7 },
	{ 0x4b100a80, 0x00000000, 0x00000000 },
	{ 0x4b152c48, 0x0ec69cb7, 0x0ec4029c },

	{ 0x4b100a90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a94, 0x00000000, 0x341b0030 },
	{ 0x4b100a98, 0x00000000, 0xaf5b00bc },
	{ 0x4b100a9c, 0x00000000, 0x0ac55f86 },
	{ 0x4b100aa0, 0x00000000, 0x00000000 },
	{ 0x4b152c50, 0x0ec55f86, 0x0ec402a4 },

	{ 0x4b100ab0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ab4, 0x00000000, 0x341b0031 },
	{ 0x4b100ab8, 0x00000000, 0xaf5b00c0 },
	{ 0x4b100abc, 0x00000000, 0x0ac4a0d3 },
	{ 0x4b100ac0, 0x00000000, 0x00000000 },
	{ 0x4b152c58, 0x0ec4a0d3, 0x0ec402ac },

	{ 0x4b100ad0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ad4, 0x00000000, 0x341b0032 },
	{ 0x4b100ad8, 0x00000000, 0xaf5b00c4 },
	{ 0x4b100adc, 0x00000000, 0x0ac6778c },
	{ 0x4b100ae0, 0x00000000, 0x00000000 },
	{ 0x4b152c60, 0x0ec6778c, 0x0ec402b4 },

	{ 0x4b100af0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100af4, 0x00000000, 0x341b0033 },
	{ 0x4b100af8, 0x00000000, 0xaf5b00c8 },
	{ 0x4b100afc, 0x00000000, 0x0ac5714f },
	{ 0x4b100b00, 0x00000000, 0x00000000 },
	{ 0x4b152c68, 0x0ec5714f, 0x0ec402bc },

	{ 0x4b100b10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b14, 0x00000000, 0x341b0034 },
	{ 0x4b100b18, 0x00000000, 0xaf5b00cc },
	{ 0x4b100b1c, 0x00000000, 0x0ac5eaab },
	{ 0x4b100b20, 0x00000000, 0x00000000 },
	{ 0x4b152c70, 0x0ec5eaab, 0x0ec402c4 },

	{ 0x4b100b30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b34, 0x00000000, 0x341b0035 },
	{ 0x4b100b38, 0x00000000, 0xaf5b00d0 },
	{ 0x4b100b3c, 0x00000000, 0x0ac60e77 },
	{ 0x4b100b40, 0x00000000, 0x00000000 },
	{ 0x4b152c78, 0x0ec60e77, 0x0ec402cc },

	{ 0x4b100b50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b54, 0x00000000, 0x341b0036 },
	{ 0x4b100b58, 0x00000000, 0xaf5b00d4 },
	{ 0x4b100b5c, 0x00000000, 0x0ac61003 },
	{ 0x4b100b60, 0x00000000, 0x00000000 },
	{ 0x4b152c80, 0x0ec61003, 0x0ec402d4 },

	{ 0x4b100b70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b74, 0x00000000, 0x341b0037 },
	{ 0x4b100b78, 0x00000000, 0xaf5b00d8 },
	{ 0x4b100b7c, 0x00000000, 0x0ac5f0d9 },
	{ 0x4b100b80, 0x00000000, 0x00000000 },
	{ 0x4b152c88, 0x0ec5f0d9, 0x0ec402dc },

	{ 0x4b100b90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b94, 0x00000000, 0x341b0038 },
	{ 0x4b100b98, 0x00000000, 0xaf5b00dc },
	{ 0x4b100b9c, 0x00000000, 0x0ac6768e },
	{ 0x4b100ba0, 0x00000000, 0x00000000 },
	{ 0x4b152c90, 0x0ec6768e, 0x0ec402e4 },

	{ 0x4b100bb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bb4, 0x00000000, 0x341b0039 },
	{ 0x4b100bb8, 0x00000000, 0xaf5b00e0 },
	{ 0x4b100bbc, 0x00000000, 0x0ac6af28 },
	{ 0x4b100bc0, 0x00000000, 0x00000000 },
	{ 0x4b152c98, 0x0ec6af28, 0x0ec402ec },

	{ 0x4b100bd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bd4, 0x00000000, 0x341b003a },
	{ 0x4b100bd8, 0x00000000, 0xaf5b00e4 },
	{ 0x4b100bdc, 0x00000000, 0x0ac54dad },
	{ 0x4b100be0, 0x00000000, 0x00000000 },
	{ 0x4b152ca0, 0x0ec54dad, 0x0ec402f4 },

	{ 0x4b100bf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bf4, 0x00000000, 0x341b003b },
	{ 0x4b100bf8, 0x00000000, 0xaf5b00e8 },
	{ 0x4b100bfc, 0x00000000, 0x0ac41bdf },
	{ 0x4b100c00, 0x00000000, 0x00000000 },
	{ 0x4b152cb0, 0x0ec41bdf, 0x0ec402fc },

	{ 0x4b100c10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c14, 0x00000000, 0x341b003c },
	{ 0x4b100c18, 0x00000000, 0xaf5b00ec },
	{ 0x4b100c1c, 0x00000000, 0x0ac41adf },
	{ 0x4b100c20, 0x00000000, 0x00000000 },
	{ 0x4b152cb8, 0x0ec41adf, 0x0ec40304 },

	{ 0x4b100c30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c34, 0x00000000, 0x341b003d },
	{ 0x4b100c38, 0x00000000, 0xaf5b00f0 },
	{ 0x4b100c3c, 0x00000000, 0x0ac423ed },
	{ 0x4b100c40, 0x00000000, 0x00000000 },
	{ 0x4b152cc0, 0x0ec423ed, 0x0ec4030c },

	/* Trace device-manager allocation and its four factory calls. */
	{ 0x4b100c50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c54, 0x00000000, 0x341b003e },
	{ 0x4b100c58, 0x00000000, 0xaf5b00f4 },
	{ 0x4b100c5c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100c60, 0x00000000, 0x00000000 },
	{ 0x4b183a10, 0x0ec6b54d, 0x0ec40314 },

	{ 0x4b100c70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c74, 0x00000000, 0x341b003f },
	{ 0x4b100c78, 0x00000000, 0xaf5b00f8 },
	{ 0x4b100c7c, 0x00000000, 0x0ac60c33 },
	{ 0x4b100c80, 0x00000000, 0x00000000 },
	{ 0x4b183a1c, 0x0ec60c33, 0x0ec4031c },

	{ 0x4b100c90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c94, 0x00000000, 0x341b0040 },
	{ 0x4b100c98, 0x00000000, 0xaf5b00fc },
	{ 0x4b100c9c, 0x00000000, 0x0ac51818 },
	{ 0x4b100ca0, 0x00000000, 0x00000000 },
	{ 0x4b18310c, 0x0ec51818, 0x0ec40324 },

	{ 0x4b100cb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cb4, 0x00000000, 0x341b0041 },
	{ 0x4b100cb8, 0x00000000, 0xaf5b0100 },
	{ 0x4b100cbc, 0x00000000, 0x0ac51bba },
	{ 0x4b100cc0, 0x00000000, 0x00000000 },
	{ 0x4b18311c, 0x0ec51bba, 0x0ec4032c },

	{ 0x4b100cd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cd4, 0x00000000, 0x341b0042 },
	{ 0x4b100cd8, 0x00000000, 0xaf5b0104 },
	{ 0x4b100cdc, 0x00000000, 0x0ac4c6df },
	{ 0x4b100ce0, 0x00000000, 0x00000000 },
	{ 0x4b18312c, 0x0ec4c6df, 0x0ec40334 },

	{ 0x4b100cf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cf4, 0x00000000, 0x341b0043 },
	{ 0x4b100cf8, 0x00000000, 0xaf5b0108 },
	{ 0x4b100cfc, 0x00000000, 0x0ac4c22b },
	{ 0x4b100d00, 0x00000000, 0x00000000 },
	{ 0x4b18313c, 0x0ec4c22b, 0x0ec4033c },

	/* Trace the HDMI receiver factory and constructor calls. */
	{ 0x4b100d10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d14, 0x00000000, 0x341b0044 },
	{ 0x4b100d18, 0x00000000, 0xaf5b010c },
	{ 0x4b100d1c, 0x00000000, 0x0ac5401a },
	{ 0x4b100d20, 0x00000000, 0x00000000 },
	{ 0x4b131bb8, 0x0ec5401a, 0x0ec40344 },

	{ 0x4b100d30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d34, 0x00000000, 0x341b0045 },
	{ 0x4b100d38, 0x00000000, 0xaf5b0110 },
	{ 0x4b100d3c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100d40, 0x00000000, 0x00000000 },
	{ 0x4b131bc0, 0x0ec6b54d, 0x0ec4034c },

	{ 0x4b100d50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d54, 0x00000000, 0x341b0046 },
	{ 0x4b100d58, 0x00000000, 0xaf5b0114 },
	{ 0x4b100d5c, 0x00000000, 0x0ac4c656 },
	{ 0x4b100d60, 0x00000000, 0x00000000 },
	{ 0x4b131bcc, 0x0ec4c656, 0x0ec40354 },

	{ 0x4b100d70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d74, 0x00000000, 0x341b0047 },
	{ 0x4b100d78, 0x00000000, 0xaf5b0118 },
	{ 0x4b100d7c, 0x00000000, 0x0ac60b3e },
	{ 0x4b100d80, 0x00000000, 0x00000000 },
	{ 0x4b13197c, 0x0ec60b3e, 0x0ec4035c },

	{ 0x4b100d90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d94, 0x00000000, 0x341b0048 },
	{ 0x4b100d98, 0x00000000, 0xaf5b011c },
	{ 0x4b100d9c, 0x00000000, 0x0ac5401a },
	{ 0x4b100da0, 0x00000000, 0x00000000 },
	{ 0x4b131a2c, 0x0ec5401a, 0x0ec40364 },

	{ 0x4b100db0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100db4, 0x00000000, 0x341b0049 },
	{ 0x4b100db8, 0x00000000, 0xaf5b0120 },
	{ 0x4b100dbc, 0x00000000, 0x0ac6b54d },
	{ 0x4b100dc0, 0x00000000, 0x00000000 },
	{ 0x4b131a38, 0x0ec6b54d, 0x0ec4036c },

	{ 0x4b100dd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100dd4, 0x00000000, 0x341b004a },
	{ 0x4b100dd8, 0x00000000, 0xaf5b0124 },
	{ 0x4b100ddc, 0x00000000, 0x0ac4cc04 },
	{ 0x4b100de0, 0x00000000, 0x00000000 },
	{ 0x4b131a48, 0x0ec4cc04, 0x0ec40374 },

	{ 0x4b100df0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100df4, 0x00000000, 0x341b004b },
	{ 0x4b100df8, 0x00000000, 0xaf5b0128 },
	{ 0x4b100dfc, 0x00000000, 0x0ac6b54d },
	{ 0x4b100e00, 0x00000000, 0x00000000 },
	{ 0x4b131a54, 0x0ec6b54d, 0x0ec4037c },

	{ 0x4b100e10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e14, 0x00000000, 0x341b004c },
	{ 0x4b100e18, 0x00000000, 0xaf5b012c },
	{ 0x4b100e1c, 0x00000000, 0x0ac4c8fb },
	{ 0x4b100e20, 0x00000000, 0x00000000 },
	{ 0x4b131a64, 0x0ec4c8fb, 0x0ec40384 },

	{ 0x4b100e30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e34, 0x00000000, 0x341b004d },
	{ 0x4b100e38, 0x00000000, 0xaf5b0130 },
	{ 0x4b100e3c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100e40, 0x00000000, 0x00000000 },
	{ 0x4b131a78, 0x0ec6b54d, 0x0ec4038c },

	{ 0x4b100e50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e54, 0x00000000, 0x341b004e },
	{ 0x4b100e58, 0x00000000, 0xaf5b0134 },
	{ 0x4b100e5c, 0x00000000, 0x0ac4e02d },
	{ 0x4b100e60, 0x00000000, 0x00000000 },
	{ 0x4b131a8c, 0x0ec4e02d, 0x0ec40394 },

	{ 0x4b100e70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e74, 0x00000000, 0x341b004f },
	{ 0x4b100e78, 0x00000000, 0xaf5b0138 },
	{ 0x4b100e7c, 0x00000000, 0x0ac4c958 },
	{ 0x4b100e80, 0x00000000, 0x00000000 },
	{ 0x4b131a9c, 0x0ec4c958, 0x0ec4039c },

	{ 0x4b100e90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e94, 0x00000000, 0x341b0050 },
	{ 0x4b100e98, 0x00000000, 0xaf5b013c },
	{ 0x4b100e9c, 0x00000000, 0x0ac56fed },
	{ 0x4b100ea0, 0x00000000, 0x00000000 },
	{ 0x4b131ac8, 0x0ec56fed, 0x0ec403a4 },

	{ 0x4b100eb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100eb4, 0x00000000, 0x341b0051 },
	{ 0x4b100eb8, 0x00000000, 0xaf5b0140 },
	{ 0x4b100ebc, 0x00000000, 0x0ac56fed },
	{ 0x4b100ec0, 0x00000000, 0x00000000 },
	{ 0x4b131ae4, 0x0ec56fed, 0x0ec403ac },

	{ 0x4b100ed0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ed4, 0x00000000, 0x341b0052 },
	{ 0x4b100ed8, 0x00000000, 0xaf5b0144 },
	{ 0x4b100edc, 0x00000000, 0x0ac56fed },
	{ 0x4b100ee0, 0x00000000, 0x00000000 },
	{ 0x4b131b00, 0x0ec56fed, 0x0ec403b4 },

	{ 0x4b100ef0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ef4, 0x00000000, 0x341b0053 },
	{ 0x4b100ef8, 0x00000000, 0xaf5b0148 },
	{ 0x4b100efc, 0x00000000, 0x0ac5401a },
	{ 0x4b100f00, 0x00000000, 0x00000000 },
	{ 0x4b131b28, 0x0ec5401a, 0x0ec403bc },

	/* Trace construction of the first HDMI receiver port. */
	{ 0x4b100f10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f14, 0x00000000, 0x341b0054 },
	{ 0x4b100f18, 0x00000000, 0xaf5b014c },
	{ 0x4b100f1c, 0x00000000, 0x0ac4e781 },
	{ 0x4b100f20, 0x00000000, 0x00000000 },
	{ 0x4b1380e0, 0x0ec4e781, 0x0ec403c4 },

	{ 0x4b100f24, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f28, 0x00000000, 0x341b0055 },
	{ 0x4b100f2c, 0x00000000, 0xaf5b0150 },
	{ 0x4b100f30, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f34, 0x00000000, 0x00000000 },
	{ 0x4b13810c, 0x0ec6b54d, 0x0ec403c9 },

	{ 0x4b100f38, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f3c, 0x00000000, 0x341b0056 },
	{ 0x4b100f40, 0x00000000, 0xaf5b0154 },
	{ 0x4b100f44, 0x00000000, 0x0ac4d495 },
	{ 0x4b100f48, 0x00000000, 0x00000000 },
	{ 0x4b138118, 0x0ec4d495, 0x0ec403ce },

	{ 0x4b100f4c, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f50, 0x00000000, 0x341b0057 },
	{ 0x4b100f54, 0x00000000, 0xaf5b0158 },
	{ 0x4b100f58, 0x00000000, 0x00400008 },
	{ 0x4b100f5c, 0x00000000, 0x00000000 },
	{ 0x4b138224, 0x0040f809, 0x0ec403d3 },

	{ 0x4b100f60, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f64, 0x00000000, 0x341b0058 },
	{ 0x4b100f68, 0x00000000, 0xaf5b015c },
	{ 0x4b100f6c, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f70, 0x00000000, 0x00000000 },
	{ 0x4b138234, 0x0ec6b54d, 0x0ec403d8 },

	{ 0x4b100f74, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f78, 0x00000000, 0x341b0059 },
	{ 0x4b100f7c, 0x00000000, 0xaf5b0160 },
	{ 0x4b100f80, 0x00000000, 0x0ac4e7c9 },
	{ 0x4b100f84, 0x00000000, 0x00000000 },
	{ 0x4b138244, 0x0ec4e7c9, 0x0ec403dd },

	{ 0x4b100f88, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f8c, 0x00000000, 0x341b005a },
	{ 0x4b100f90, 0x00000000, 0xaf5b0164 },
	{ 0x4b100f94, 0x00000000, 0x0ac6b54d },
	{ 0x4b100f98, 0x00000000, 0x00000000 },
	{ 0x4b138250, 0x0ec6b54d, 0x0ec403e2 },

	{ 0x4b100f9c, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fa0, 0x00000000, 0x341b005b },
	{ 0x4b100fa4, 0x00000000, 0xaf5b0168 },
	{ 0x4b100fa8, 0x00000000, 0x0ac4e9d9 },
	{ 0x4b100fac, 0x00000000, 0x00000000 },
	{ 0x4b138260, 0x0ec4e9d9, 0x0ec403e7 },

	{ 0x4b100fb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fb4, 0x00000000, 0x341b005c },
	{ 0x4b100fb8, 0x00000000, 0xaf5b016c },
	{ 0x4b100fbc, 0x00000000, 0x0ac5401a },
	{ 0x4b100fc0, 0x00000000, 0x00000000 },
	{ 0x4b138178, 0x0ec5401a, 0x0ec403ec },

	/*
	 * Marker 93 enters the address validator used by the first HDMI-RX
	 * register read, then tail-calls the original function.
	 */
	{ 0x4b100fd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fd4, 0x00000000, 0x341b005d },
	{ 0x4b100fd8, 0x00000000, 0xaf5b0170 },
	{ 0x4b100fdc, 0x00000000, 0x0ac5fe3b },
	{ 0x4b100fe0, 0x00000000, 0x00000000 },
	{ 0x4b17fa6c, 0x0ec5fe3b, 0x0ec403f4 },

	/*
	 * Marker 94 records a successful validator return and reproduces the
	 * original branch to the rejected-address or MMIO-read paths.
	 */
	{ 0x4b100ff0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ff4, 0x00000000, 0x341b005e },
	{ 0x4b100ff8, 0x00000000, 0xaf5b0174 },
	{ 0x4b100ffc, 0x00000000, 0x10400003 },
	{ 0x4b101000, 0x00000000, 0x00000000 },
	{ 0x4b101004, 0x00000000, 0x0ac5fea8 },
	{ 0x4b101008, 0x00000000, 0x00000000 },
	{ 0x4b10100c, 0x00000000, 0x0ac5fe9f },
	{ 0x4b101010, 0x00000000, 0x00000000 },
	{ 0x4b17fa74, 0x1440000a, 0x0ac403fc },

	/*
	 * Markers 95 and 96 surround the byte load itself. If only marker 95
	 * appears, the access to physical HDMI-RX register 0x06840093 wedged.
	 */
	{ 0x4b101020, 0x00000000, 0x3c1aae34 },
	{ 0x4b101024, 0x00000000, 0x341b005f },
	{ 0x4b101028, 0x00000000, 0xaf5b0178 },
	{ 0x4b10102c, 0x00000000, 0x90820000 },
	{ 0x4b101030, 0x00000000, 0x341b0060 },
	{ 0x4b101034, 0x00000000, 0xaf5b017c },
	{ 0x4b101038, 0x00000000, 0x03e00008 },
	{ 0x4b10103c, 0x00000000, 0x00000000 },
	{ 0x4b17fa88, 0x90820000, 0x0ec40408 },

	/*
	 * Marker 97 records the return from the first indirect helper and
	 * reproduces its original zero/nonzero branch.
	 */
	{ 0x4b101040, 0x00000000, 0x3c1aae34 },
	{ 0x4b101044, 0x00000000, 0x341b0061 },
	{ 0x4b101048, 0x00000000, 0xaf5b0180 },
	{ 0x4b10104c, 0x00000000, 0x10400003 },
	{ 0x4b101050, 0x00000000, 0x00000000 },
	{ 0x4b101054, 0x00000000, 0x0ac4e7d5 },
	{ 0x4b101058, 0x00000000, 0x00000000 },
	{ 0x4b10105c, 0x00000000, 0x0ac4e7dd },
	{ 0x4b101060, 0x00000000, 0x00000000 },
	{ 0x4b139f4c, 0x10400009, 0x0ac40410 },

	/* Marker 98 enters the second indirect target. */
	{ 0x4b101070, 0x00000000, 0x3c1aae34 },
	{ 0x4b101074, 0x00000000, 0x341b0062 },
	{ 0x4b101078, 0x00000000, 0xaf5b0184 },
	{ 0x4b10107c, 0x00000000, 0x03200008 },
	{ 0x4b101080, 0x00000000, 0x00000000 },
	{ 0x4b139f6c, 0x03200008, 0x0ac4041c },
	{ 0x4b139fc4, 0x03200008, 0x0ac4041c },

	/*
	 * Marker 99 records the second target's initial byte-read return and
	 * reproduces its original bit-test branch.
	 */
	{ 0x4b101090, 0x00000000, 0x3c1aae34 },
	{ 0x4b101094, 0x00000000, 0x341b0063 },
	{ 0x4b101098, 0x00000000, 0xaf5b0188 },
	{ 0x4b10109c, 0x00000000, 0x10400003 },
	{ 0x4b1010a0, 0x00000000, 0x00000000 },
	{ 0x4b1010a4, 0x00000000, 0x0ac4f409 },
	{ 0x4b1010a8, 0x00000000, 0x00000000 },
	{ 0x4b1010ac, 0x00000000, 0x0ac4f40c },
	{ 0x4b1010b0, 0x00000000, 0x00000000 },
	{ 0x4b13d01c, 0x10400004, 0x0ac40424 },

	/* Marker 100 enters the second target's read/modify/write fallback. */
	{ 0x4b1010c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1010c4, 0x00000000, 0x341b0064 },
	{ 0x4b1010c8, 0x00000000, 0xaf5b018c },
	{ 0x4b1010cc, 0x00000000, 0x0ac5febf },
	{ 0x4b1010d0, 0x00000000, 0x00000000 },
	{ 0x4b13d03c, 0x0ac5febf, 0x0ac40430 },

	/* Marker 101 enters the first target's byte-write helper. */
	{ 0x4b1010e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1010e4, 0x00000000, 0x341b0065 },
	{ 0x4b1010e8, 0x00000000, 0xaf5b0190 },
	{ 0x4b1010ec, 0x00000000, 0x0ac5fead },
	{ 0x4b1010f0, 0x00000000, 0x00000000 },
	{ 0x4b13d06c, 0x0ec5fead, 0x0ec40438 },

	/* Marker 102 proves the first target's byte write returned. */
	{ 0x4b101110, 0x00000000, 0x3c1aae34 },
	{ 0x4b101114, 0x00000000, 0x341b0066 },
	{ 0x4b101118, 0x00000000, 0xaf5b0194 },
	{ 0x4b10111c, 0x00000000, 0x0ac56f65 },
	{ 0x4b101120, 0x00000000, 0x00000000 },
	{ 0x4b13d074, 0x0ec56f65, 0x0ec40444 },

	/* Marker 103 enters the polling loop's second timer read. */
	{ 0x4b101130, 0x00000000, 0x3c1aae34 },
	{ 0x4b101134, 0x00000000, 0x341b0067 },
	{ 0x4b101138, 0x00000000, 0xaf5b0198 },
	{ 0x4b10113c, 0x00000000, 0x0ac56f65 },
	{ 0x4b101140, 0x00000000, 0x00000000 },
	{ 0x4b13d098, 0x0ec56f65, 0x0ec4044c },

	/*
	 * Marker 104 proves that timer read returned. The next two trace words
	 * capture its current tick and the initial tick used by the wait loop.
	 */
	{ 0x4b101150, 0x00000000, 0x3c1aae34 },
	{ 0x4b101154, 0x00000000, 0x341b0068 },
	{ 0x4b101158, 0x00000000, 0xaf5b019c },
	{ 0x4b10115c, 0x00000000, 0xaf4201a0 },
	{ 0x4b101160, 0x00000000, 0xaf5101a4 },
	{ 0x4b101164, 0x00000000, 0x00511823 },
	{ 0x4b101168, 0x00000000, 0x2c630033 },
	{ 0x4b10116c, 0x00000000, 0x0ac4f42a },
	{ 0x4b101170, 0x00000000, 0x00000000 },
	{ 0x4b13d0a0, 0x00511823, 0x0ac40454 },
	{ 0x4b13d0a4, 0x2c630033, 0x00000000 },

	/*
	 * Record the current callback and table position before each indirect
	 * call in platform registration. A blocking callback leaves its target
	 * as the final value in the corresponding trace pair.
	 */
	{ 0x4b101190, 0x00000000, 0x3c1aae34 },
	{ 0x4b101194, 0x00000000, 0xaf4201a8 },
	{ 0x4b101198, 0x00000000, 0xaf5001ac },
	{ 0x4b10119c, 0x00000000, 0x00400008 },
	{ 0x4b1011a0, 0x00000000, 0x00000000 },
	{ 0x4b18427c, 0x0040f809, 0x0ec40464 },

	{ 0x4b1011b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011b4, 0x00000000, 0xaf4201b0 },
	{ 0x4b1011b8, 0x00000000, 0xaf5001b4 },
	{ 0x4b1011bc, 0x00000000, 0x00400008 },
	{ 0x4b1011c0, 0x00000000, 0x00000000 },
	{ 0x4b1842c0, 0x0040f809, 0x0ec4046c },

	/* Marker 111 enters the marker-49 singleton allocation. */
	{ 0x4b1011d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011d4, 0x00000000, 0x341b006f },
	{ 0x4b1011d8, 0x00000000, 0xaf5b01b8 },
	{ 0x4b1011dc, 0x00000000, 0x0ac6b54d },
	{ 0x4b1011e0, 0x00000000, 0x00000000 },
	{ 0x4b128380, 0x0ec6b54d, 0x0ec40474 },

	/*
	 * Markers 130..132 bisect the four-registration group at 0x8b128020.
	 * It is a virtual-call loop of the form
	 *
	 *	fn = (*obj)[0xc]; fn(obj, id, s2, 0)
	 *
	 * issued for ids 0x12e, 0x130, 3, and 8 in that order. Marker 130 used
	 * to end in a spin, which halted the run before id 3 ever executed; it
	 * now falls through so the last two calls are reached.
	 *
	 * Each marker sits immediately after its call returns, so the highest
	 * one reported names the last registration that completed and the next
	 * id in the list is the one that hangs. Marker 132 doubles as the
	 * probe's completion sentinel because it lands in the final slot.
	 */
	{ 0x4b1011e8, 0x00000000, 0x3c1aae34 },
	{ 0x4b1011ec, 0x00000000, 0x341b0082 },
	{ 0x4b1011f0, 0x00000000, 0xaf5b0250 },
	{ 0x4b1011f4, 0x00000000, 0x0ac4a017 },
	{ 0x4b1011f8, 0x00000000, 0x8e020000 },
	{ 0x4b128058, 0x8e020000, 0x0ac4047a },

	/* Marker 131: registration id 3 returned. */
	{ 0x4b1002d8, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002dc, 0x00000000, 0x341b0083 },
	{ 0x4b1002e0, 0x00000000, 0xaf5b0254 },
	{ 0x4b1002e4, 0x00000000, 0x0ac4a01e },
	{ 0x4b1002e8, 0x00000000, 0x8e020000 },
	{ 0x4b128074, 0x8e020000, 0x0ac400b6 },

	/* Marker 132: registration id 8 returned; the whole group completed. */
	{ 0x4b1002ec, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002f0, 0x00000000, 0x341b0084 },
	{ 0x4b1002f4, 0x00000000, 0xaf5b0258 },
	{ 0x4b1002f8, 0x00000000, 0x0ac4a025 },
	{ 0x4b1002fc, 0x00000000, 0x8fb00040 },
	{ 0x4b128090, 0x8fb00040, 0x0ac400bb },
};
#endif

/*
 * Minimal CPU_COMM readiness trace rebuilt against display.bin 4380f1b3....
 *
 * Each guarded call redirects through a zero-filled code cave, stores its
 * marker through the firmware's uncached 0xae340000 alias, then either resumes
 * after a disposable log call (markers 1 and 2) or tail-calls the original
 * function. Markers 1-9 cover the chain needed for MIPS READY:
 *
 *  1. share address absent (polling path)
 *  2. share address and size accepted
 *  3. ARM CPU_READY accepted
 *  4. CPU_COMM hardware spinlock 0 acquired
 *  5. slave-side CPU_COMM initialization entered
 *  6. ThreadX application entry reached
 *  7. CPU_COMM initialization entered
 *  8. application byte-pool creation succeeded
 *  9. share-register reader called
 *
 * Markers 10-16 cover reset handoff through the ThreadX scheduler:
 *
 * 10. reset code handed off to the C runtime
 * 11-14. successive pre-application initialization calls entered
 * 15. application/thread construction entered
 * 16. ThreadX scheduler entry called
 *
 * Markers 17-26 cover every direct call in application/thread construction;
 * marker 26 is the ThreadX thread-creation call whose entry argument is the
 * application function traced by marker 6.
 * Marker 21's extended cave also records the sys:dbg_buf address and size
 * passed to memset in trace slots 88 and 89.
 *
 * Markers 27-56 cover every call inside marker 25's early-system-
 * initialization function at 0x8b15340c.
 *
 * Markers 57-61 cover all five calls in marker 38's tse_init function at
 * 0x8b110478: the global enable write, InitTFDMemory virtual call,
 * tse_init_data, and its two fatal-log paths.
 *
 * Markers 62-66 cover setCPUReady(), its spinlock acquire/unlock calls, and
 * its return to InitCommMem. Markers 67-71 cover the five calls after
 * InitCommMem returns and before setCPUAppReady(). Marker 72 enters the first
 * registration group. Markers 73-77 trace its CPU_COMM request through call-
 * table insertion, result lookup, and the insertion spinlock. Markers 78-82
 * retain later registration groups and final calls in hal_adapter_init().
 * Its repeated registration helper records a call count plus its latest
 * object/callback pointers in trace slots 90-92. Markers 84-86 cover the
 * helper's formatting, request construction, and CPU_COMM request call.
 * Marker 87 covers the final hal-adapter registration lock; marker 88 covers
 * setCPUAppReady() entry.
 */
static const struct h713_mips_patch h713_mips_trace_patches[] = {
	/* Marker 1: share address absent; skip its log call and resume. */
	{ 0x4b1002c4, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002c8, 0x00000000, 0x341b0001 },
	{ 0x4b1002cc, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002d0, 0x00000000, 0x0ac48e83 },
	{ 0x4b1002d4, 0x00000000, 0x00000000 },
	{ 0x4b123a04, 0x0ec54252, 0x0ac400b1 },

	/* Marker 2: share registers accepted; skip its log call and resume. */
	{ 0x4b100500, 0x00000000, 0x3c1aae34 },
	{ 0x4b100504, 0x00000000, 0x341b0002 },
	{ 0x4b100508, 0x00000000, 0xaf5b0004 },
	{ 0x4b10050c, 0x00000000, 0x0ac48ea4 },
	{ 0x4b100510, 0x00000000, 0x00000000 },
	{ 0x4b123a88, 0x0ec54252, 0x0ac40140 },

	/* Marker 3: ARM CPU_READY accepted, then preserve the original log. */
	{ 0x4b1005d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005d4, 0x00000000, 0x341b0003 },
	{ 0x4b1005d8, 0x00000000, 0xaf5b0008 },
	{ 0x4b1005dc, 0x00000000, 0x0ac54252 },
	{ 0x4b1005e0, 0x00000000, 0x00000000 },
	{ 0x4b123ad4, 0x0ec54252, 0x0ec40174 },

	/* Marker 4: hardware spinlock acquired, then preserve the log. */
	{ 0x4b1005f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005f4, 0x00000000, 0x341b0004 },
	{ 0x4b1005f8, 0x00000000, 0xaf5b000c },
	{ 0x4b1005fc, 0x00000000, 0x0ac54252 },
	{ 0x4b100600, 0x00000000, 0x00000000 },
	{ 0x4b11b1e0, 0x0ec54252, 0x0ec4017c },

	/* Marker 5: enter slave-side CPU_COMM initialization. */
	{ 0x4b100520, 0x00000000, 0x3c1aae34 },
	{ 0x4b100524, 0x00000000, 0x341b0005 },
	{ 0x4b100528, 0x00000000, 0xaf5b0010 },
	{ 0x4b10052c, 0x00000000, 0x0ac466ed },
	{ 0x4b100530, 0x00000000, 0x00000000 },
	{ 0x4b11b34c, 0x0ec466ed, 0x0ec40148 },

	/* Marker 6: ThreadX application entry reached. */
	{ 0x4b100540, 0x00000000, 0x3c1aae34 },
	{ 0x4b100544, 0x00000000, 0x341b0006 },
	{ 0x4b100548, 0x00000000, 0xaf5b0014 },
	{ 0x4b10054c, 0x00000000, 0x0ac5719d },
	{ 0x4b100550, 0x00000000, 0x00000000 },
	{ 0x4b152e84, 0x0ec5719d, 0x0ec40150 },

	/* Marker 7: enter CPU_COMM initialization. */
	{ 0x4b100560, 0x00000000, 0x3c1aae34 },
	{ 0x4b100564, 0x00000000, 0x341b0007 },
	{ 0x4b100568, 0x00000000, 0xaf5b0018 },
	{ 0x4b10056c, 0x00000000, 0x0ac490da },
	{ 0x4b100570, 0x00000000, 0x00000000 },
	{ 0x4b152e8c, 0x0ec490da, 0x0ec40158 },

	/* Marker 8: CPU_COMM byte-pool creation succeeded. */
	{ 0x4b100590, 0x00000000, 0x3c1aae34 },
	{ 0x4b100594, 0x00000000, 0x341b0008 },
	{ 0x4b100598, 0x00000000, 0xaf5b001c },
	{ 0x4b10059c, 0x00000000, 0x0ac54252 },
	{ 0x4b1005a0, 0x00000000, 0x00000000 },
	{ 0x4b12443c, 0x0ec54252, 0x0ec40164 },

	/* Marker 9: enter the share-register reader. */
	{ 0x4b1005b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005b4, 0x00000000, 0x341b0009 },
	{ 0x4b1005b8, 0x00000000, 0xaf5b0020 },
	{ 0x4b1005bc, 0x00000000, 0x0ac48e55 },
	{ 0x4b1005c0, 0x00000000, 0x00000000 },
	{ 0x4b124448, 0x0ec48e55, 0x0ec4016c },

	/* Marker 10: reset code hands off to the C runtime at 0x8b101f04. */
	{ 0x4b100610, 0x00000000, 0x3c1aae34 },
	{ 0x4b100614, 0x00000000, 0x341b000a },
	{ 0x4b100618, 0x00000000, 0xaf5b0024 },
	{ 0x4b10061c, 0x00000000, 0x0ac407c1 },
	{ 0x4b100620, 0x00000000, 0x00000000 },
	{ 0x4b1b1220, 0x0100f809, 0x0ec40184 },

	/* Markers 11-16: every C-runtime call through scheduler entry. */
	{ 0x4b100630, 0x00000000, 0x3c1aae34 },
	{ 0x4b100634, 0x00000000, 0x341b000b },
	{ 0x4b100638, 0x00000000, 0xaf5b0028 },
	{ 0x4b10063c, 0x00000000, 0x0ac419a4 },
	{ 0x4b100640, 0x00000000, 0x00000000 },
	{ 0x4b101f1c, 0x0ec419a4, 0x0ec4018c },

	{ 0x4b100650, 0x00000000, 0x3c1aae34 },
	{ 0x4b100654, 0x00000000, 0x341b000c },
	{ 0x4b100658, 0x00000000, 0xaf5b002c },
	{ 0x4b10065c, 0x00000000, 0x0ac40b86 },
	{ 0x4b100660, 0x00000000, 0x00000000 },
	{ 0x4b101f24, 0x0ec40b86, 0x0ec40194 },

	{ 0x4b100670, 0x00000000, 0x3c1aae34 },
	{ 0x4b100674, 0x00000000, 0x341b000d },
	{ 0x4b100678, 0x00000000, 0xaf5b0030 },
	{ 0x4b10067c, 0x00000000, 0x0ac5203a },
	{ 0x4b100680, 0x00000000, 0x00000000 },
	{ 0x4b101f2c, 0x0ec5203a, 0x0ec4019c },

	{ 0x4b100690, 0x00000000, 0x3c1aae34 },
	{ 0x4b100694, 0x00000000, 0x341b000e },
	{ 0x4b100698, 0x00000000, 0xaf5b0034 },
	{ 0x4b10069c, 0x00000000, 0x0ac612c9 },
	{ 0x4b1006a0, 0x00000000, 0x00000000 },
	{ 0x4b101f34, 0x0ec612c9, 0x0ec401a4 },

	{ 0x4b1006b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006b4, 0x00000000, 0x341b000f },
	{ 0x4b1006b8, 0x00000000, 0xaf5b0038 },
	{ 0x4b1006bc, 0x00000000, 0x0ac54d6f },
	{ 0x4b1006c0, 0x00000000, 0x00000000 },
	{ 0x4b101f3c, 0x0ec54d6f, 0x0ec401ac },

	{ 0x4b1006d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006d4, 0x00000000, 0x341b0010 },
	{ 0x4b1006d8, 0x00000000, 0xaf5b003c },
	{ 0x4b1006dc, 0x00000000, 0x0ac415e4 },
	{ 0x4b1006e0, 0x00000000, 0x00000000 },
	{ 0x4b101f4c, 0x0ec415e4, 0x0ec401b4 },

	/* Markers 17-26: each application/thread construction call. */
	{ 0x4b1006f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006f4, 0x00000000, 0x341b0011 },
	{ 0x4b1006f8, 0x00000000, 0xaf5b0040 },
	{ 0x4b1006fc, 0x00000000, 0x0ac674f1 },
	{ 0x4b100700, 0x00000000, 0x00000000 },
	{ 0x4b1535c4, 0x0ec674f1, 0x0ec401bc },

	{ 0x4b100710, 0x00000000, 0x3c1aae34 },
	{ 0x4b100714, 0x00000000, 0x341b0012 },
	{ 0x4b100718, 0x00000000, 0xaf5b0044 },
	{ 0x4b10071c, 0x00000000, 0x0ac43a27 },
	{ 0x4b100720, 0x00000000, 0x00000000 },
	{ 0x4b1535d0, 0x0ec43a27, 0x0ec401c4 },

	{ 0x4b100730, 0x00000000, 0x3c1aae34 },
	{ 0x4b100734, 0x00000000, 0x341b0013 },
	{ 0x4b100738, 0x00000000, 0xaf5b0048 },
	{ 0x4b10073c, 0x00000000, 0x0ac4354e },
	{ 0x4b100740, 0x00000000, 0x00000000 },
	{ 0x4b1535d8, 0x0ec4354e, 0x0ec401cc },

	{ 0x4b100750, 0x00000000, 0x3c1aae34 },
	{ 0x4b100754, 0x00000000, 0x341b0014 },
	{ 0x4b100758, 0x00000000, 0xaf5b004c },
	{ 0x4b10075c, 0x00000000, 0x0ac4355c },
	{ 0x4b100760, 0x00000000, 0x00000000 },
	{ 0x4b1535e0, 0x0ec4355c, 0x0ec401d4 },

	{ 0x4b100770, 0x00000000, 0x3c1aae34 },
	{ 0x4b100774, 0x00000000, 0x341b0015 },
	{ 0x4b100778, 0x00000000, 0xaf5b0050 },
	{ 0x4b10077c, 0x00000000, 0xaf440160 },
	{ 0x4b100780, 0x00000000, 0xaf460164 },
	{ 0x4b100784, 0x00000000, 0x0ac568d0 },
	{ 0x4b100788, 0x00000000, 0x00000000 },
	{ 0x4b1535f0, 0x0ec568d0, 0x0ec401dc },

	{ 0x4b100790, 0x00000000, 0x3c1aae34 },
	{ 0x4b100794, 0x00000000, 0x341b0016 },
	{ 0x4b100798, 0x00000000, 0xaf5b0054 },
	{ 0x4b10079c, 0x00000000, 0x0ac55058 },
	{ 0x4b1007a0, 0x00000000, 0x00000000 },
	{ 0x4b1535f8, 0x0ec55058, 0x0ec401e4 },

	{ 0x4b1007b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007b4, 0x00000000, 0x341b0017 },
	{ 0x4b1007b8, 0x00000000, 0xaf5b0058 },
	{ 0x4b1007bc, 0x00000000, 0x0ac54643 },
	{ 0x4b1007c0, 0x00000000, 0x00000000 },
	{ 0x4b153600, 0x0ec54643, 0x0ec401ec },

	{ 0x4b1007d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007d4, 0x00000000, 0x341b0018 },
	{ 0x4b1007d8, 0x00000000, 0xaf5b005c },
	{ 0x4b1007dc, 0x00000000, 0x0ac60d31 },
	{ 0x4b1007e0, 0x00000000, 0x00000000 },
	{ 0x4b153608, 0x0ec60d31, 0x0ec401f4 },

	{ 0x4b1007f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007f4, 0x00000000, 0x341b0019 },
	{ 0x4b1007f8, 0x00000000, 0xaf5b0060 },
	{ 0x4b1007fc, 0x00000000, 0x0ac54d03 },
	{ 0x4b100800, 0x00000000, 0x00000000 },
	{ 0x4b153610, 0x0ec54d03, 0x0ec401fc },

	{ 0x4b100810, 0x00000000, 0x3c1aae34 },
	{ 0x4b100814, 0x00000000, 0x341b001a },
	{ 0x4b100818, 0x00000000, 0xaf5b0064 },
	{ 0x4b10081c, 0x00000000, 0x0ac57225 },
	{ 0x4b100820, 0x00000000, 0x00000000 },
	{ 0x4b153634, 0x0ec57225, 0x0ec40204 },

	/* Markers 27-56: every call inside 0x8b15340c. */
	{ 0x4b100830, 0x00000000, 0x3c1aae34 },
	{ 0x4b100834, 0x00000000, 0x341b001b },
	{ 0x4b100838, 0x00000000, 0xaf5b0068 },
	{ 0x4b10083c, 0x00000000, 0x0ac6017c },
	{ 0x4b100840, 0x00000000, 0x00000000 },
	{ 0x4b153444, 0x0ec6017c, 0x0ec4020c },

	{ 0x4b100850, 0x00000000, 0x3c1aae34 },
	{ 0x4b100854, 0x00000000, 0x341b001c },
	{ 0x4b100858, 0x00000000, 0xaf5b006c },
	{ 0x4b10085c, 0x00000000, 0x0ac4354b },
	{ 0x4b100860, 0x00000000, 0x00000000 },
	{ 0x4b153454, 0x0ec4354b, 0x0ec40214 },

	{ 0x4b100870, 0x00000000, 0x3c1aae34 },
	{ 0x4b100874, 0x00000000, 0x341b001d },
	{ 0x4b100878, 0x00000000, 0xaf5b0070 },
	{ 0x4b10087c, 0x00000000, 0x00400008 },
	{ 0x4b100880, 0x00000000, 0x00000000 },
	{ 0x4b153470, 0x0040f809, 0x0ec4021c },

	{ 0x4b100890, 0x00000000, 0x3c1aae34 },
	{ 0x4b100894, 0x00000000, 0x341b001e },
	{ 0x4b100898, 0x00000000, 0xaf5b0074 },
	{ 0x4b10089c, 0x00000000, 0x0ac4354b },
	{ 0x4b1008a0, 0x00000000, 0x00000000 },
	{ 0x4b153478, 0x0ec4354b, 0x0ec40224 },

	{ 0x4b1008b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008b4, 0x00000000, 0x341b001f },
	{ 0x4b1008b8, 0x00000000, 0xaf5b0078 },
	{ 0x4b1008bc, 0x00000000, 0x00400008 },
	{ 0x4b1008c0, 0x00000000, 0x00000000 },
	{ 0x4b153494, 0x0040f809, 0x0ec4022c },

	{ 0x4b1008d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008d4, 0x00000000, 0x341b0020 },
	{ 0x4b1008d8, 0x00000000, 0xaf5b007c },
	{ 0x4b1008dc, 0x00000000, 0x0ac6018e },
	{ 0x4b1008e0, 0x00000000, 0x00000000 },
	{ 0x4b1534a4, 0x0ec6018e, 0x0ec40234 },

	{ 0x4b1008f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008f4, 0x00000000, 0x341b0021 },
	{ 0x4b1008f8, 0x00000000, 0xaf5b0080 },
	{ 0x4b1008fc, 0x00000000, 0x0ac6018e },
	{ 0x4b100900, 0x00000000, 0x00000000 },
	{ 0x4b1534b4, 0x0ec6018e, 0x0ec4023c },

	{ 0x4b100910, 0x00000000, 0x3c1aae34 },
	{ 0x4b100914, 0x00000000, 0x341b0022 },
	{ 0x4b100918, 0x00000000, 0xaf5b0084 },
	{ 0x4b10091c, 0x00000000, 0x0ac6282e },
	{ 0x4b100920, 0x00000000, 0x00000000 },
	{ 0x4b1534c0, 0x0ec6282e, 0x0ec40244 },

	{ 0x4b100930, 0x00000000, 0x3c1aae34 },
	{ 0x4b100934, 0x00000000, 0x341b0023 },
	{ 0x4b100938, 0x00000000, 0xaf5b0088 },
	{ 0x4b10093c, 0x00000000, 0x0ac6257d },
	{ 0x4b100940, 0x00000000, 0x00000000 },
	{ 0x4b1534c8, 0x0ec6257d, 0x0ec4024c },

	{ 0x4b100950, 0x00000000, 0x3c1aae34 },
	{ 0x4b100954, 0x00000000, 0x341b0024 },
	{ 0x4b100958, 0x00000000, 0xaf5b008c },
	{ 0x4b10095c, 0x00000000, 0x0ac4354b },
	{ 0x4b100960, 0x00000000, 0x00000000 },
	{ 0x4b1534d0, 0x0ec4354b, 0x0ec40254 },

	{ 0x4b100970, 0x00000000, 0x3c1aae34 },
	{ 0x4b100974, 0x00000000, 0x341b0025 },
	{ 0x4b100978, 0x00000000, 0xaf5b0090 },
	{ 0x4b10097c, 0x00000000, 0x00400008 },
	{ 0x4b100980, 0x00000000, 0x00000000 },
	{ 0x4b1534ec, 0x0040f809, 0x0ec4025c },

	{ 0x4b100990, 0x00000000, 0x3c1aae34 },
	{ 0x4b100994, 0x00000000, 0x341b0026 },
	{ 0x4b100998, 0x00000000, 0xaf5b0094 },
	{ 0x4b10099c, 0x00000000, 0x0ac4411e },
	{ 0x4b1009a0, 0x00000000, 0x00000000 },
	{ 0x4b1534fc, 0x0ec4411e, 0x0ec40264 },

	{ 0x4b1009b0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009b4, 0x00000000, 0x341b0027 },
	{ 0x4b1009b8, 0x00000000, 0xaf5b0098 },
	{ 0x4b1009bc, 0x00000000, 0x0ac54ccf },
	{ 0x4b1009c0, 0x00000000, 0x00000000 },
	{ 0x4b153504, 0x0ec54ccf, 0x0ec4026c },

	{ 0x4b1009d0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009d4, 0x00000000, 0x341b0028 },
	{ 0x4b1009d8, 0x00000000, 0xaf5b009c },
	{ 0x4b1009dc, 0x00000000, 0x0ac627a9 },
	{ 0x4b1009e0, 0x00000000, 0x00000000 },
	{ 0x4b15350c, 0x0ec627a9, 0x0ec40274 },

	{ 0x4b1009f0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009f4, 0x00000000, 0x341b0029 },
	{ 0x4b1009f8, 0x00000000, 0xaf5b00a0 },
	{ 0x4b1009fc, 0x00000000, 0x00400008 },
	{ 0x4b100a00, 0x00000000, 0x00000000 },
	{ 0x4b153520, 0x0040f809, 0x0ec4027c },

	{ 0x4b100a10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a14, 0x00000000, 0x341b002a },
	{ 0x4b100a18, 0x00000000, 0xaf5b00a4 },
	{ 0x4b100a1c, 0x00000000, 0x0ac69eef },
	{ 0x4b100a20, 0x00000000, 0x00000000 },
	{ 0x4b153528, 0x0ec69eef, 0x0ec40284 },

	{ 0x4b100a30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a34, 0x00000000, 0x341b002b },
	{ 0x4b100a38, 0x00000000, 0xaf5b00a8 },
	{ 0x4b100a3c, 0x00000000, 0x0ac561be },
	{ 0x4b100a40, 0x00000000, 0x00000000 },
	{ 0x4b153530, 0x0ec561be, 0x0ec4028c },

	{ 0x4b100a50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a54, 0x00000000, 0x341b002c },
	{ 0x4b100a58, 0x00000000, 0xaf5b00ac },
	{ 0x4b100a5c, 0x00000000, 0x0ac4a11e },
	{ 0x4b100a60, 0x00000000, 0x00000000 },
	{ 0x4b153538, 0x0ec4a11e, 0x0ec40294 },

	{ 0x4b100a70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a74, 0x00000000, 0x341b002d },
	{ 0x4b100a78, 0x00000000, 0xaf5b00b0 },
	{ 0x4b100a7c, 0x00000000, 0x0ac679c4 },
	{ 0x4b100a80, 0x00000000, 0x00000000 },
	{ 0x4b153540, 0x0ec679c4, 0x0ec4029c },

	{ 0x4b100a90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a94, 0x00000000, 0x341b002e },
	{ 0x4b100a98, 0x00000000, 0xaf5b00b4 },
	{ 0x4b100a9c, 0x00000000, 0x0ac57387 },
	{ 0x4b100aa0, 0x00000000, 0x00000000 },
	{ 0x4b153548, 0x0ec57387, 0x0ec402a4 },

	{ 0x4b100ab0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ab4, 0x00000000, 0x341b002f },
	{ 0x4b100ab8, 0x00000000, 0xaf5b00b8 },
	{ 0x4b100abc, 0x00000000, 0x0ac5ece3 },
	{ 0x4b100ac0, 0x00000000, 0x00000000 },
	{ 0x4b153550, 0x0ec5ece3, 0x0ec402ac },

	{ 0x4b100ad0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ad4, 0x00000000, 0x341b0030 },
	{ 0x4b100ad8, 0x00000000, 0xaf5b00bc },
	{ 0x4b100adc, 0x00000000, 0x0ac610af },
	{ 0x4b100ae0, 0x00000000, 0x00000000 },
	{ 0x4b153558, 0x0ec610af, 0x0ec402b4 },

	{ 0x4b100af0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100af4, 0x00000000, 0x341b0031 },
	{ 0x4b100af8, 0x00000000, 0xaf5b00c0 },
	{ 0x4b100afc, 0x00000000, 0x0ac6123b },
	{ 0x4b100b00, 0x00000000, 0x00000000 },
	{ 0x4b153560, 0x0ec6123b, 0x0ec402bc },

	{ 0x4b100b10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b14, 0x00000000, 0x341b0032 },
	{ 0x4b100b18, 0x00000000, 0xaf5b00c4 },
	{ 0x4b100b1c, 0x00000000, 0x0ac5f311 },
	{ 0x4b100b20, 0x00000000, 0x00000000 },
	{ 0x4b153568, 0x0ec5f311, 0x0ec402c4 },

	{ 0x4b100b30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b34, 0x00000000, 0x341b0033 },
	{ 0x4b100b38, 0x00000000, 0xaf5b00c8 },
	{ 0x4b100b3c, 0x00000000, 0x0ac678c6 },
	{ 0x4b100b40, 0x00000000, 0x00000000 },
	{ 0x4b153570, 0x0ec678c6, 0x0ec402cc },

	{ 0x4b100b50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b54, 0x00000000, 0x341b0034 },
	{ 0x4b100b58, 0x00000000, 0xaf5b00cc },
	{ 0x4b100b5c, 0x00000000, 0x0ac6b160 },
	{ 0x4b100b60, 0x00000000, 0x00000000 },
	{ 0x4b153578, 0x0ec6b160, 0x0ec402d4 },

	{ 0x4b100b70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b74, 0x00000000, 0x341b0035 },
	{ 0x4b100b78, 0x00000000, 0xaf5b00d0 },
	{ 0x4b100b7c, 0x00000000, 0x0ac54fe5 },
	{ 0x4b100b80, 0x00000000, 0x00000000 },
	{ 0x4b153580, 0x0ec54fe5, 0x0ec402dc },

	{ 0x4b100b90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100b94, 0x00000000, 0x341b0036 },
	{ 0x4b100b98, 0x00000000, 0xaf5b00d4 },
	{ 0x4b100b9c, 0x00000000, 0x0ac41bdf },
	{ 0x4b100ba0, 0x00000000, 0x00000000 },
	{ 0x4b153590, 0x0ec41bdf, 0x0ec402e4 },

	{ 0x4b100bb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bb4, 0x00000000, 0x341b0037 },
	{ 0x4b100bb8, 0x00000000, 0xaf5b00d8 },
	{ 0x4b100bbc, 0x00000000, 0x0ac41adf },
	{ 0x4b100bc0, 0x00000000, 0x00000000 },
	{ 0x4b153598, 0x0ec41adf, 0x0ec402ec },

	{ 0x4b100bd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bd4, 0x00000000, 0x341b0038 },
	{ 0x4b100bd8, 0x00000000, 0xaf5b00dc },
	{ 0x4b100bdc, 0x00000000, 0x0ac423ed },
	{ 0x4b100be0, 0x00000000, 0x00000000 },
	{ 0x4b1535a0, 0x0ec423ed, 0x0ec402f4 },

	/* Markers 57-61: every call inside tse_init at 0x8b110478. */
	{ 0x4b100bf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100bf4, 0x00000000, 0x341b0039 },
	{ 0x4b100bf8, 0x00000000, 0xaf5b00e0 },
	{ 0x4b100bfc, 0x00000000, 0x0ac627c9 },
	{ 0x4b100c00, 0x00000000, 0x00000000 },
	{ 0x4b1104a0, 0x0ec627c9, 0x0ec402fc },

	{ 0x4b100c10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c14, 0x00000000, 0x341b003a },
	{ 0x4b100c18, 0x00000000, 0xaf5b00e4 },
	{ 0x4b100c1c, 0x00000000, 0x00400008 },
	{ 0x4b100c20, 0x00000000, 0x00000000 },
	{ 0x4b1104b8, 0x0040f809, 0x0ec40304 },

	{ 0x4b100c30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c34, 0x00000000, 0x341b003b },
	{ 0x4b100c38, 0x00000000, 0xaf5b00e8 },
	{ 0x4b100c3c, 0x00000000, 0x0ac43aa4 },
	{ 0x4b100c40, 0x00000000, 0x00000000 },
	{ 0x4b1104c8, 0x0ec43aa4, 0x0ec4030c },

	{ 0x4b100c50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c54, 0x00000000, 0x341b003c },
	{ 0x4b100c58, 0x00000000, 0xaf5b00ec },
	{ 0x4b100c5c, 0x00000000, 0x0ac54252 },
	{ 0x4b100c60, 0x00000000, 0x00000000 },
	{ 0x4b110518, 0x0ec54252, 0x0ec40314 },

	{ 0x4b100c70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c74, 0x00000000, 0x341b003d },
	{ 0x4b100c78, 0x00000000, 0xaf5b00f0 },
	{ 0x4b100c7c, 0x00000000, 0x0ac54252 },
	{ 0x4b100c80, 0x00000000, 0x00000000 },
	{ 0x4b110568, 0x0ec54252, 0x0ec4031c },

	/* Marker 62: enter setCPUReady(getCurCPUID()). */
	{ 0x4b100c90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100c94, 0x00000000, 0x341b003e },
	{ 0x4b100c98, 0x00000000, 0xaf5b00f4 },
	{ 0x4b100c9c, 0x00000000, 0x0ac468f7 },
	{ 0x4b100ca0, 0x00000000, 0x00000000 },
	{ 0x4b11b364, 0x0ec468f7, 0x0ec40324 },

	/* Marker 63: setCPUReady enters comm_SpinLock(3). */
	{ 0x4b100cb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cb4, 0x00000000, 0x341b003f },
	{ 0x4b100cb8, 0x00000000, 0xaf5b00f8 },
	{ 0x4b100cbc, 0x00000000, 0x0ac496f1 },
	{ 0x4b100cc0, 0x00000000, 0x00000000 },
	{ 0x4b11a56c, 0x0ec496f1, 0x0ec4032c },

	/* Marker 64: READY stores completed; enter comm_SpinUnlock(3). */
	{ 0x4b100cd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cd4, 0x00000000, 0x341b0040 },
	{ 0x4b100cd8, 0x00000000, 0xaf5b00fc },
	{ 0x4b100cdc, 0x00000000, 0x0ac496fa },
	{ 0x4b100ce0, 0x00000000, 0x00000000 },
	{ 0x4b11a5a4, 0x0ec496fa, 0x0ec40334 },

	/* Marker 65: unlock returned; enter setCPUReady's final log call. */
	{ 0x4b100cf0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100cf4, 0x00000000, 0x341b0041 },
	{ 0x4b100cf8, 0x00000000, 0xaf5b0100 },
	{ 0x4b100cfc, 0x00000000, 0x0ac54252 },
	{ 0x4b100d00, 0x00000000, 0x00000000 },
	{ 0x4b11a5cc, 0x0ec54252, 0x0ec4033c },

	/* Marker 66: setCPUReady returned to InitCommMem. */
	{ 0x4b100d10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d14, 0x00000000, 0x341b0042 },
	{ 0x4b100d18, 0x00000000, 0xaf5b0104 },
	{ 0x4b100d1c, 0x00000000, 0x0ac49318 },
	{ 0x4b100d20, 0x00000000, 0x00000000 },
	{ 0x4b11b36c, 0x0ec49318, 0x0ec40344 },

	/* Markers 67-71: calls after InitCommMem and before app-ready. */
	{ 0x4b100d30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d34, 0x00000000, 0x341b0043 },
	{ 0x4b100d38, 0x00000000, 0xaf5b0108 },
	{ 0x4b100d3c, 0x00000000, 0x0ac5719d },
	{ 0x4b100d40, 0x00000000, 0x00000000 },
	{ 0x4b152ec8, 0x0ec5719d, 0x0ec4034c },

	{ 0x4b100d50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d54, 0x00000000, 0x341b0044 },
	{ 0x4b100d58, 0x00000000, 0xaf5b010c },
	{ 0x4b100d5c, 0x00000000, 0x0ac42b6e },
	{ 0x4b100d60, 0x00000000, 0x00000000 },
	{ 0x4b152ed0, 0x0ec42b6e, 0x0ec40354 },

	{ 0x4b100d70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d74, 0x00000000, 0x341b0045 },
	{ 0x4b100d78, 0x00000000, 0xaf5b0110 },
	{ 0x4b100d7c, 0x00000000, 0x0ac5719d },
	{ 0x4b100d80, 0x00000000, 0x00000000 },
	{ 0x4b152ed8, 0x0ec5719d, 0x0ec4035c },

	{ 0x4b100d90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100d94, 0x00000000, 0x341b0046 },
	{ 0x4b100d98, 0x00000000, 0xaf5b0114 },
	{ 0x4b100d9c, 0x00000000, 0x0ac52161 },
	{ 0x4b100da0, 0x00000000, 0x00000000 },
	{ 0x4b152ee4, 0x0ec52161, 0x0ec40364 },

	{ 0x4b100db0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100db4, 0x00000000, 0x341b0047 },
	{ 0x4b100db8, 0x00000000, 0xaf5b0118 },
	{ 0x4b100dbc, 0x00000000, 0x0ac52132 },
	{ 0x4b100dc0, 0x00000000, 0x00000000 },
	{ 0x4b152eec, 0x0ec52132, 0x0ec4036c },

	/* Marker 72: first registration group in hal_adapter_init(). */
	{ 0x4b100dd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100dd4, 0x00000000, 0x341b0048 },
	{ 0x4b100dd8, 0x00000000, 0xaf5b011c },
	{ 0x4b100ddc, 0x00000000, 0x0ac49246 },
	{ 0x4b100de0, 0x00000000, 0x00000000 },
	{ 0x4b10ade4, 0x0ec49246, 0x0ec40374 },

	/* Marker 73: registration request enters the shared call-table insert. */
	{ 0x4b100df0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100df4, 0x00000000, 0x341b0049 },
	{ 0x4b100df8, 0x00000000, 0xaf5b0120 },
	{ 0x4b100dfc, 0x00000000, 0x0ac4717c },
	{ 0x4b100e00, 0x00000000, 0x00000000 },
	{ 0x4b1225dc, 0x0ec4717c, 0x0ec4037c },

	/* Marker 74: call-table insert returned; resolve its result object. */
	{ 0x4b100e10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e14, 0x00000000, 0x341b004a },
	{ 0x4b100e18, 0x00000000, 0xaf5b0124 },
	{ 0x4b100e1c, 0x00000000, 0x0ac47382 },
	{ 0x4b100e20, 0x00000000, 0x00000000 },
	{ 0x4b122614, 0x0ec47382, 0x0ec40384 },

	/* Marker 75: result absent; enter the alternate cleanup/removal call. */
	{ 0x4b100e30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e34, 0x00000000, 0x341b004b },
	{ 0x4b100e38, 0x00000000, 0xaf5b0128 },
	{ 0x4b100e3c, 0x00000000, 0x0ac4738d },
	{ 0x4b100e40, 0x00000000, 0x00000000 },
	{ 0x4b122658, 0x0ec4738d, 0x0ec4038c },

	/* Marker 76: call-table insert enters comm_SpinLock(2). */
	{ 0x4b100e50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e54, 0x00000000, 0x341b004c },
	{ 0x4b100e58, 0x00000000, 0xaf5b012c },
	{ 0x4b100e5c, 0x00000000, 0x0ac496f1 },
	{ 0x4b100e60, 0x00000000, 0x00000000 },
	{ 0x4b11c7a8, 0x0ec496f1, 0x0ec40394 },

	/* Marker 77: entry copied and counted; enter comm_SpinUnlock(2). */
	{ 0x4b100e70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e74, 0x00000000, 0x341b004d },
	{ 0x4b100e78, 0x00000000, 0xaf5b0130 },
	{ 0x4b100e7c, 0x00000000, 0x0ac496fa },
	{ 0x4b100e80, 0x00000000, 0x00000000 },
	{ 0x4b11c838, 0x0ec496fa, 0x0ec4039c },

	/* Markers 78-80: later registration groups in hal_adapter_init(). */
	{ 0x4b100e90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100e94, 0x00000000, 0x341b004e },
	{ 0x4b100e98, 0x00000000, 0xaf5b0134 },
	{ 0x4b100e9c, 0x00000000, 0x0ac49246 },
	{ 0x4b100ea0, 0x00000000, 0x00000000 },
	{ 0x4b10aeb4, 0x0ec49246, 0x0ec403a4 },

	{ 0x4b100eb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100eb4, 0x00000000, 0x341b004f },
	{ 0x4b100eb8, 0x00000000, 0xaf5b0138 },
	{ 0x4b100ebc, 0x00000000, 0x0ac49246 },
	{ 0x4b100ec0, 0x00000000, 0x00000000 },
	{ 0x4b10aee0, 0x0ec49246, 0x0ec403ac },

	{ 0x4b100ed0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ed4, 0x00000000, 0x341b0050 },
	{ 0x4b100ed8, 0x00000000, 0xaf5b013c },
	{ 0x4b100edc, 0x00000000, 0x0ac49246 },
	{ 0x4b100ee0, 0x00000000, 0x00000000 },
	{ 0x4b10af0c, 0x0ec49246, 0x0ec403b4 },

	/* Markers 81-82: final registration and hal-adapter return log. */
	{ 0x4b100ef0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100ef4, 0x00000000, 0x341b0051 },
	{ 0x4b100ef8, 0x00000000, 0xaf5b0140 },
	{ 0x4b100efc, 0x00000000, 0x0ac48de2 },
	{ 0x4b100f00, 0x00000000, 0x00000000 },
	{ 0x4b10af1c, 0x0ec48de2, 0x0ec403bc },

	{ 0x4b100f10, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f14, 0x00000000, 0x341b0052 },
	{ 0x4b100f18, 0x00000000, 0xaf5b0144 },
	{ 0x4b100f1c, 0x00000000, 0x0ac54252 },
	{ 0x4b100f20, 0x00000000, 0x00000000 },
	{ 0x4b10af5c, 0x0ec54252, 0x0ec403c4 },

	/* Repeated helper entry: count calls and retain its latest arguments. */
	{ 0x4b100f30, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f34, 0x00000000, 0x8f5b0168 },
	{ 0x4b100f38, 0x00000000, 0x277b0001 },
	{ 0x4b100f3c, 0x00000000, 0xaf5b0168 },
	{ 0x4b100f40, 0x00000000, 0xaf44016c },
	{ 0x4b100f44, 0x00000000, 0xaf450170 },
	{ 0x4b100f48, 0x00000000, 0x0ac489ed },
	{ 0x4b100f4c, 0x00000000, 0x00000000 },
	{ 0x4b124984, 0x0ec489ed, 0x0ec403cc },

	/* Markers 84-86: format, construct, and submit a registration. */
	{ 0x4b100f50, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f54, 0x00000000, 0x341b0054 },
	{ 0x4b100f58, 0x00000000, 0xaf5b014c },
	{ 0x4b100f5c, 0x00000000, 0x0ac56cf3 },
	{ 0x4b100f60, 0x00000000, 0x00000000 },
	{ 0x4b1249a4, 0x0ec56cf3, 0x0ec403d4 },

	{ 0x4b100f70, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f74, 0x00000000, 0x341b0055 },
	{ 0x4b100f78, 0x00000000, 0xaf5b0150 },
	{ 0x4b100f7c, 0x00000000, 0x0ac491c1 },
	{ 0x4b100f80, 0x00000000, 0x00000000 },
	{ 0x4b1249b4, 0x0ec491c1, 0x0ec403dc },

	{ 0x4b100f90, 0x00000000, 0x3c1aae34 },
	{ 0x4b100f94, 0x00000000, 0x341b0056 },
	{ 0x4b100f98, 0x00000000, 0xaf5b0154 },
	{ 0x4b100f9c, 0x00000000, 0x0ac48955 },
	{ 0x4b100fa0, 0x00000000, 0x00000000 },
	{ 0x4b12486c, 0x0ec48955, 0x0ec403e4 },

	/* Marker 87: final hal-adapter registration enters spinlock 1. */
	{ 0x4b100fb0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fb4, 0x00000000, 0x341b0057 },
	{ 0x4b100fb8, 0x00000000, 0xaf5b0158 },
	{ 0x4b100fbc, 0x00000000, 0x0ac496f1 },
	{ 0x4b100fc0, 0x00000000, 0x00000000 },
	{ 0x4b123828, 0x0ec496f1, 0x0ec403ec },

	/* Marker 88: enter the later setCPUAppReady() wrapper. */
	{ 0x4b100fd0, 0x00000000, 0x3c1aae34 },
	{ 0x4b100fd4, 0x00000000, 0x341b0058 },
	{ 0x4b100fd8, 0x00000000, 0xaf5b015c },
	{ 0x4b100fdc, 0x00000000, 0x0ac4912c },
	{ 0x4b100fe0, 0x00000000, 0x00000000 },
	{ 0x4b152ef4, 0x0ec4912c, 0x0ec403f4 },
};

/*
 * CPU_COMM RETURN progress trace for the authenticated 4380f1b3... image.
 *
 * Each trampoline writes a progress-stage value to the uncached shared-memory
 * alias and then preserves the displaced control flow.
 * The normal startup trace uses the same code caves, so the two modes are
 * deliberately mutually exclusive.
 *
 *   a001  worker entered FreeCall recycling
 *   a002  release semaphore acquired; pre-commit log entered
 *   a003  pre-commit log returned; requesting FIFO write slot
 *   a004  committing the recycled slot to FreeCall
 *   a005  posting the release semaphore
 *   a105  release-semaphore post returned an error
 *   a006  release-semaphore post succeeded
 *   a007  final release log entered
 *   b001  routine lookup entered
 *   b002  routine lookup returned
 *   b003  handler arguments prepared; pre-call log entered
 *   b004  component handler entered
 *   c001  component handler returned to the worker
 *   c002  worker called SendComm2CPUEx
 *   c003  SendComm2CPUEx entered the send-semaphore wait
 *   c103  send-semaphore wait returned an error
 *   c004  send semaphore acquired
 *   c005  receiver staging-FIFO space is being checked
 *   c006  receiver staging FIFO has space
 *   c007  FreeReturn allocation entered
 *   c008  RETURN publication entered
 *   c009  waiting for RETURN_ACK
 *   d001  RETURN_ACK interrupt queued its deferred action
 *   d005  queueAction returned to the RETURN_ACK interrupt handler
 *   d002  deferred RETURN_ACK action reached the wakeup path
 *   d003  deferred action is posting the sender wait semaphore
 *   d103  sender wait-semaphore post returned an error
 *   d004  sender wait-semaphore post succeeded
 *   c010  SendComm2CPUEx resumed after RETURN_ACK
 *   c011  SendComm2CPUEx is releasing its send semaphore
 *   c111  send-semaphore release returned an error
 *   c012  send semaphore released successfully
 *   c013  SendComm2CPUEx returned to the CALL worker
 *
 * A separate CALL-action stage at trace+0x0c avoids races with the worker and
 * ACK stages above. trace+0x08 records osa_queue_send's unmodified return.
 *
 *   e001  CALL interrupt is invoking queueAction
 *   e004  queueAction returned to the CALL interrupt handler
 *   e005  CALL HISR wrapper is invoking the dispatcher
 *   e006  command_action entered for CALL
 *   e007  command_action is enqueueing the high-priority CALL worker
 *   e008  command_action is sending CALL_ACK
 *   e009  SendAckLow returned
 *   e010  command_action is returning to the CALL HISR wrapper
 *   e011  CALL HISR dispatcher returned
 *
 * A third persistent stage at trace+0x10 proves that the first RETURN_ACK
 * callback and its HISR wrapper actually returned to the shared consumer:
 *
 *   f001  ack_action is returning to the RETURN_ACK HISR wrapper
 *   f002  RETURN_ACK HISR dispatcher returned
 *   f003  RETURN_ACK HISR wrapper is returning
 *
 * The final three trampolines follow THal_Vp_SetSource below CPU_COMM. They
 * publish through the same uncached alias because the stock source globals
 * live in cached MIPS KSEG0 and therefore cannot be inspected reliably from
 * ARM with fwmd:
 *
 *   5101  source callback received an event
 *   5102  source callback returned from its nonblocking queue send
 *   5201  source worker dequeued a source-change event
 *   5202  source worker found new source equal to its current source
 *   5203  source worker completed the source transition
 *
 * THal_Vp_Init's registered adapter gets one more persistent slot.  Its
 * three stock inputs are [0, 1, physical address of a 0xd800-byte buffer].
 * These markers distinguish an internal initializer failure from a bad bulk
 * destination or the callback-install tail without changing any arguments:
 *
 *   7101  adapter entered; calling the local state reset
 *   7102  local state reset returned; calling the VP initializer
 *   7103  VP initializer returned; loading the caller's physical buffer
 *   7104  0xd800-byte copy returned
 *   7105  output prepared; entering callback installation
 */
static const struct h713_mips_patch h713_mips_comm_trace_patches[] = {
	/* Release semaphore acquired -> marker a002 -> original logger. */
	{ 0x4b1004c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1004c4, 0x00000000, 0x341ba002 },
	{ 0x4b1004c8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1004cc, 0x00000000, 0x0ac54252 },
	{ 0x4b1004d0, 0x00000000, 0x00000000 },
	{ 0x4b118e5c, 0x0ec54252, 0x0ec40130 },

	/* Pre-commit log returned -> marker a003 -> FIFO slot request. */
	{ 0x4b1004e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1004e4, 0x00000000, 0x341ba003 },
	{ 0x4b1004e8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1004ec, 0x00000000, 0x0ac46097 },
	{ 0x4b1004f0, 0x00000000, 0x00000000 },
	{ 0x4b118e64, 0x0ec46097, 0x0ec40138 },

	/* FIFO commit -> marker a004 -> fifo_requestItemWr. */
	{ 0x4b100500, 0x00000000, 0x3c1aae34 },
	{ 0x4b100504, 0x00000000, 0x341ba004 },
	{ 0x4b100508, 0x00000000, 0xaf5b0000 },
	{ 0x4b10050c, 0x00000000, 0x0ac4610c },
	{ 0x4b100510, 0x00000000, 0x00000000 },
	{ 0x4b118e9c, 0x0ec4610c, 0x0ec40140 },

	/* Release-semaphore post -> marker a005 -> osal_semaphore_set. */
	{ 0x4b100520, 0x00000000, 0x3c1aae34 },
	{ 0x4b100524, 0x00000000, 0x341ba005 },
	{ 0x4b100528, 0x00000000, 0xaf5b0000 },
	{ 0x4b10052c, 0x00000000, 0x0ac57074 },
	{ 0x4b100530, 0x00000000, 0x00000000 },
	{ 0x4b118eac, 0x0ec57074, 0x0ec40148 },

	/* Semaphore-post error log -> marker a105 -> original logger. */
	{ 0x4b100540, 0x00000000, 0x3c1aae34 },
	{ 0x4b100544, 0x00000000, 0x341ba105 },
	{ 0x4b100548, 0x00000000, 0xaf5b0000 },
	{ 0x4b10054c, 0x00000000, 0x0ac54252 },
	{ 0x4b100550, 0x00000000, 0x00000000 },
	{ 0x4b118eec, 0x0ec54252, 0x0ec40150 },

	/* Semaphore-post success log -> marker a006 -> original logger. */
	{ 0x4b100560, 0x00000000, 0x3c1aae34 },
	{ 0x4b100564, 0x00000000, 0x341ba006 },
	{ 0x4b100568, 0x00000000, 0xaf5b0000 },
	{ 0x4b10056c, 0x00000000, 0x0ac54252 },
	{ 0x4b100570, 0x00000000, 0x00000000 },
	{ 0x4b118f64, 0x0ec54252, 0x0ec40158 },

	/* Final release log -> marker a007 -> original logger. */
	{ 0x4b100580, 0x00000000, 0x3c1aae34 },
	{ 0x4b100584, 0x00000000, 0x341ba007 },
	{ 0x4b100588, 0x00000000, 0xaf5b0000 },
	{ 0x4b10058c, 0x00000000, 0x0ac54252 },
	{ 0x4b100590, 0x00000000, 0x00000000 },
	{ 0x4b118f8c, 0x0ec54252, 0x0ec40160 },

	/* FreeCall recycle -> marker a001 -> original release helper. */
	{ 0x4b100400, 0x00000000, 0x3c1aae34 },
	{ 0x4b100404, 0x00000000, 0x341ba001 },
	{ 0x4b100408, 0x00000000, 0xaf5b0000 },
	{ 0x4b10040c, 0x00000000, 0x0ac4632c },
	{ 0x4b100410, 0x00000000, 0x00000000 },
	{ 0x4b123dec, 0x0ec4632c, 0x0ec40100 },

	/* Routine lookup -> marker b001 -> original lookup helper. */
	{ 0x4b100420, 0x00000000, 0x3c1aae34 },
	{ 0x4b100424, 0x00000000, 0x341bb001 },
	{ 0x4b100428, 0x00000000, 0xaf5b0000 },
	{ 0x4b10042c, 0x00000000, 0x0ac4717a },
	{ 0x4b100430, 0x00000000, 0x00000000 },
	{ 0x4b123df8, 0x0ec4717a, 0x0ec40108 },

	/*
	 * Routine-lookup return -> marker b002, then reproduce the original
	 * beqz: zero is the found path at 0x8b123eb0; non-zero is the error
	 * path at 0x8b123e08.
	 */
	{ 0x4b100440, 0x00000000, 0x3c1aae34 },
	{ 0x4b100444, 0x00000000, 0x341bb002 },
	{ 0x4b100448, 0x00000000, 0xaf5b0000 },
	{ 0x4b10044c, 0x00000000, 0x10400003 },
	{ 0x4b100450, 0x00000000, 0x00000000 },
	{ 0x4b100454, 0x00000000, 0x0ac48f82 },
	{ 0x4b100458, 0x00000000, 0x00000000 },
	{ 0x4b10045c, 0x00000000, 0x0ac48fac },
	{ 0x4b100460, 0x00000000, 0x00000000 },
	{ 0x4b123e00, 0x1040002b, 0x0ac40110 },

	/* Pre-handler log -> marker b003 -> original logger. */
	{ 0x4b100480, 0x00000000, 0x3c1aae34 },
	{ 0x4b100484, 0x00000000, 0x341bb003 },
	{ 0x4b100488, 0x00000000, 0xaf5b0000 },
	{ 0x4b10048c, 0x00000000, 0x0ac54252 },
	{ 0x4b100490, 0x00000000, 0x00000000 },
	{ 0x4b123f3c, 0x0ec54252, 0x0ec40120 },

	/* Dynamic handler call -> marker b004 -> original target in v0. */
	{ 0x4b1004a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1004a4, 0x00000000, 0x341bb004 },
	{ 0x4b1004a8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1004ac, 0x00000000, 0x00400008 },
	{ 0x4b1004b0, 0x00000000, 0x00000000 },
	{ 0x4b123f4c, 0x0040f809, 0x0ec40128 },

	/* Handler-return branch -> marker c001 -> original branch target. */
	{ 0x4b1002c4, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002c8, 0x00000000, 0x341bc001 },
	{ 0x4b1002cc, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002d0, 0x00000000, 0x0ac48f8f },
	{ 0x4b1002d4, 0x00000000, 0x00000000 },
	{ 0x4b123f54, 0x1000ffb9, 0x0ac400b1 },

	/* Worker SendComm2CPUEx call -> marker c002 -> original callee. */
	{ 0x4b1002e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1002e4, 0x00000000, 0x341bc002 },
	{ 0x4b1002e8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1002ec, 0x00000000, 0x0ac48232 },
	{ 0x4b1002f0, 0x00000000, 0x00000000 },
	{ 0x4b123e68, 0x0ec48232, 0x0ec400b8 },

	/* Send semaphore wait -> marker c003 -> osal_semaphore_get. */
	{ 0x4b100300, 0x00000000, 0x3c1aae34 },
	{ 0x4b100304, 0x00000000, 0x341bc003 },
	{ 0x4b100308, 0x00000000, 0xaf5b0000 },
	{ 0x4b10030c, 0x00000000, 0x0ac5703f },
	{ 0x4b100310, 0x00000000, 0x00000000 },
	{ 0x4b12002c, 0x0ec5703f, 0x0ec400c0 },

	/* First success-path log -> marker c004 -> original logger. */
	{ 0x4b100320, 0x00000000, 0x3c1aae34 },
	{ 0x4b100324, 0x00000000, 0x341bc004 },
	{ 0x4b100328, 0x00000000, 0xaf5b0000 },
	{ 0x4b10032c, 0x00000000, 0x0ac54252 },
	{ 0x4b100330, 0x00000000, 0x00000000 },
	{ 0x4b1200e8, 0x0ec54252, 0x0ec400c8 },

	/* Staging-space test -> marker c005 -> fifo_isNearlyFull. */
	{ 0x4b100340, 0x00000000, 0x3c1aae34 },
	{ 0x4b100344, 0x00000000, 0x341bc005 },
	{ 0x4b100348, 0x00000000, 0xaf5b0000 },
	{ 0x4b10034c, 0x00000000, 0x0ac46017 },
	{ 0x4b100350, 0x00000000, 0x00000000 },
	{ 0x4b120164, 0x0ec46017, 0x0ec400d0 },

	/* FIFO fall-through -> marker c006, restore sll, resume at +0x18c. */
	{ 0x4b100360, 0x00000000, 0x3c1aae34 },
	{ 0x4b100364, 0x00000000, 0x341bc006 },
	{ 0x4b100368, 0x00000000, 0xaf5b0000 },
	{ 0x4b10036c, 0x00000000, 0x00161040 },
	{ 0x4b100370, 0x00000000, 0x0ac48063 },
	{ 0x4b100374, 0x00000000, 0x00000000 },
	{ 0x4b120184, 0x00161040, 0x0ac400d8 },

	/* FreeReturn allocation -> marker c007 -> original allocator. */
	{ 0x4b100380, 0x00000000, 0x3c1aae34 },
	{ 0x4b100384, 0x00000000, 0x341bc007 },
	{ 0x4b100388, 0x00000000, 0xaf5b0000 },
	{ 0x4b10038c, 0x00000000, 0x0ac462fa },
	{ 0x4b100390, 0x00000000, 0x00000000 },
	{ 0x4b12065c, 0x0ec462fa, 0x0ec400e0 },

	/* RETURN publication -> marker c008 -> SendLow. */
	{ 0x4b1003a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1003a4, 0x00000000, 0x341bc008 },
	{ 0x4b1003a8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1003ac, 0x00000000, 0x0ac47e7b },
	{ 0x4b1003b0, 0x00000000, 0x00000000 },
	{ 0x4b12046c, 0x0ec47e7b, 0x0ec400e8 },

	/* RETURN_ACK wait -> marker c009 -> osal_semaphore_get. */
	{ 0x4b1003c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1003c4, 0x00000000, 0x341bc009 },
	{ 0x4b1003c8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1003cc, 0x00000000, 0x0ac5703f },
	{ 0x4b1003d0, 0x00000000, 0x00000000 },
	{ 0x4b1204b4, 0x0ec5703f, 0x0ec400f0 },

	/* ACK interrupt queued action -> marker d001 -> queueAction. */
	{ 0x4b1005a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005a4, 0x00000000, 0x341bd001 },
	{ 0x4b1005a8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1005ac, 0x00000000, 0x0ac47bc0 },
	{ 0x4b1005b0, 0x00000000, 0x00000000 },
	{ 0x4b11f9dc, 0x0ec47bc0, 0x0ec40168 },

	/* queueAction returned -> marker d005 -> original handler tail. */
	{ 0x4b100660, 0x00000000, 0x3c1aae34 },
	{ 0x4b100664, 0x00000000, 0x341bd005 },
	{ 0x4b100668, 0x00000000, 0xaf5b0000 },
	{ 0x4b10066c, 0x00000000, 0x0ac47e42 },
	{ 0x4b100670, 0x00000000, 0x00000000 },
	{ 0x4b11f9e4, 0x1000ffc8, 0x0ac40198 },

	/* ACK action wakeup path -> marker d002 -> original logger. */
	{ 0x4b1005c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005c4, 0x00000000, 0x341bd002 },
	{ 0x4b1005c8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1005cc, 0x00000000, 0x0ac54252 },
	{ 0x4b1005d0, 0x00000000, 0x00000000 },
	{ 0x4b11eda4, 0x0ec54252, 0x0ec40170 },

	/* Sender wakeup -> marker d003 -> osal_semaphore_set. */
	{ 0x4b1005e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1005e4, 0x00000000, 0x341bd003 },
	{ 0x4b1005e8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1005ec, 0x00000000, 0x0ac57074 },
	{ 0x4b1005f0, 0x00000000, 0x00000000 },
	{ 0x4b11edb4, 0x0ec57074, 0x0ec40178 },

	/* Wakeup error log -> marker d103 -> original logger. */
	{ 0x4b100600, 0x00000000, 0x3c1aae34 },
	{ 0x4b100604, 0x00000000, 0x341bd103 },
	{ 0x4b100608, 0x00000000, 0xaf5b0000 },
	{ 0x4b10060c, 0x00000000, 0x0ac54252 },
	{ 0x4b100610, 0x00000000, 0x00000000 },
	{ 0x4b11edf4, 0x0ec54252, 0x0ec40180 },

	/* Wakeup success log -> marker d004 -> original logger. */
	{ 0x4b100620, 0x00000000, 0x3c1aae34 },
	{ 0x4b100624, 0x00000000, 0x341bd004 },
	{ 0x4b100628, 0x00000000, 0xaf5b0000 },
	{ 0x4b10062c, 0x00000000, 0x0ac54252 },
	{ 0x4b100630, 0x00000000, 0x00000000 },
	{ 0x4b11eed0, 0x0ec54252, 0x0ec40188 },

	/* ACK wait resumed -> marker c010 -> original logger. */
	{ 0x4b100640, 0x00000000, 0x3c1aae34 },
	{ 0x4b100644, 0x00000000, 0x341bc010 },
	{ 0x4b100648, 0x00000000, 0xaf5b0000 },
	{ 0x4b10064c, 0x00000000, 0x0ac54252 },
	{ 0x4b100650, 0x00000000, 0x00000000 },
	{ 0x4b12052c, 0x0ec54252, 0x0ec40190 },

	/* Send-semaphore release -> marker c011 -> osal_semaphore_set. */
	{ 0x4b100680, 0x00000000, 0x3c1aae34 },
	{ 0x4b100684, 0x00000000, 0x341bc011 },
	{ 0x4b100688, 0x00000000, 0xaf5b0000 },
	{ 0x4b10068c, 0x00000000, 0x0ac57074 },
	{ 0x4b100690, 0x00000000, 0x00000000 },
	{ 0x4b120588, 0x0ec57074, 0x0ec401a0 },

	/* Send-semaphore release error -> marker c111 -> original logger. */
	{ 0x4b100700, 0x00000000, 0x3c1aae34 },
	{ 0x4b100704, 0x00000000, 0x341bc111 },
	{ 0x4b100708, 0x00000000, 0xaf5b0000 },
	{ 0x4b10070c, 0x00000000, 0x0ac54252 },
	{ 0x4b100710, 0x00000000, 0x00000000 },
	{ 0x4b1203ec, 0x0ec54252, 0x0ec401c0 },

	/* Send-semaphore release success -> marker c012 -> original logger. */
	{ 0x4b1006a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006a4, 0x00000000, 0x341bc012 },
	{ 0x4b1006a8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1006ac, 0x00000000, 0x0ac54252 },
	{ 0x4b1006b0, 0x00000000, 0x00000000 },
	{ 0x4b1205c0, 0x0ec54252, 0x0ec401a8 },

	/*
	 * SendComm2CPUEx returned -> marker c013, then reproduce the original
	 * branch-likely. The displaced success-only load is harmless on the error
	 * path and remains in the original jump's delay slot.
	 */
	{ 0x4b1006c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1006c4, 0x00000000, 0x341bc013 },
	{ 0x4b1006c8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1006cc, 0x00000000, 0x10400003 },
	{ 0x4b1006d0, 0x00000000, 0x00000000 },
	{ 0x4b1006d4, 0x00000000, 0x0ac48f9e },
	{ 0x4b1006d8, 0x00000000, 0x00000000 },
	{ 0x4b1006dc, 0x00000000, 0x0ac48f47 },
	{ 0x4b1006e0, 0x00000000, 0x00000000 },
	{ 0x4b123e70, 0x5040ffaa, 0x0ac401b0 },

	/* Preserve osa_queue_send's result, then emulate osa_hisr_activate. */
	{ 0x4b100720, 0x00000000, 0x3c1aae34 },
	{ 0x4b100724, 0x00000000, 0xaf420008 },
	{ 0x4b100728, 0x00000000, 0x24020001 },
	{ 0x4b10072c, 0x00000000, 0x0ac48749 },
	{ 0x4b100730, 0x00000000, 0x00000000 },
	{ 0x4b121d1c, 0x24020001, 0x0ac401c8 },

	/* CALL interrupt -> marker e001 -> queueAction. */
	{ 0x4b100740, 0x00000000, 0x3c1aae34 },
	{ 0x4b100744, 0x00000000, 0x341be001 },
	{ 0x4b100748, 0x00000000, 0xaf5b000c },
	{ 0x4b10074c, 0x00000000, 0x0ac47bc0 },
	{ 0x4b100750, 0x00000000, 0x00000000 },
	{ 0x4b11f350, 0x0ec47bc0, 0x0ec401d0 },

	/* queueAction returned -> marker e004 -> original CALL-handler tail. */
	{ 0x4b100780, 0x00000000, 0x3c1aae34 },
	{ 0x4b100784, 0x00000000, 0x341be004 },
	{ 0x4b100788, 0x00000000, 0xaf5b000c },
	{ 0x4b10078c, 0x00000000, 0x0ac47cc1 },
	{ 0x4b100790, 0x00000000, 0x00000000 },
	{ 0x4b11f358, 0x1000ffea, 0x0ac401e0 },

	/* CALL HISR wrapper -> marker e005 -> dispatcher. */
	{ 0x4b1007a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007a4, 0x00000000, 0x341be005 },
	{ 0x4b1007a8, 0x00000000, 0xaf5b000c },
	{ 0x4b1007ac, 0x00000000, 0x0ac484b3 },
	{ 0x4b1007b0, 0x00000000, 0x00000000 },
	{ 0x4b1213c4, 0x0ec484b3, 0x0ec401e8 },

	/* Dispatcher CALL arm -> marker e006 -> command_action. */
	{ 0x4b1007c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007c4, 0x00000000, 0x341be006 },
	{ 0x4b1007c8, 0x00000000, 0xaf5b000c },
	{ 0x4b1007cc, 0x00000000, 0x0ac482f0 },
	{ 0x4b1007d0, 0x00000000, 0x00000000 },
	{ 0x4b1212e4, 0x0ac482f0, 0x0ac401f0 },

	/* CALL worker enqueue -> marker e007 -> Comm_Add2Call2WQ. */
	{ 0x4b1007e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1007e4, 0x00000000, 0x341be007 },
	{ 0x4b1007e8, 0x00000000, 0xaf5b000c },
	{ 0x4b1007ec, 0x00000000, 0x0ac47551 },
	{ 0x4b1007f0, 0x00000000, 0x00000000 },
	{ 0x4b1212b0, 0x0ec47551, 0x0ec401f8 },

	/* CALL_ACK send -> marker e008 -> SendAckLow. */
	{ 0x4b100800, 0x00000000, 0x3c1aae34 },
	{ 0x4b100804, 0x00000000, 0x341be008 },
	{ 0x4b100808, 0x00000000, 0xaf5b000c },
	{ 0x4b10080c, 0x00000000, 0x0ac48234 },
	{ 0x4b100810, 0x00000000, 0x00000000 },
	{ 0x4b120fac, 0x0ec48234, 0x0ec40200 },

	/* SendAckLow returned -> marker e009 -> original command-action tail. */
	{ 0x4b100820, 0x00000000, 0x3c1aae34 },
	{ 0x4b100824, 0x00000000, 0x341be009 },
	{ 0x4b100828, 0x00000000, 0xaf5b000c },
	{ 0x4b10082c, 0x00000000, 0x0ac48373 },
	{ 0x4b100830, 0x00000000, 0x00000000 },
	{ 0x4b120fb4, 0x1000ff85, 0x0ac40208 },

	/* command_action return -> marker e010 -> original caller. */
	{ 0x4b100840, 0x00000000, 0x3c1aae34 },
	{ 0x4b100844, 0x00000000, 0x341be010 },
	{ 0x4b100848, 0x00000000, 0xaf5b000c },
	{ 0x4b10084c, 0x00000000, 0x03e00008 },
	{ 0x4b100850, 0x00000000, 0x00000000 },
	{ 0x4b120e18, 0x03e00008, 0x0ac40210 },

	/* CALL dispatcher returned -> marker e011 -> original logger. */
	{ 0x4b100860, 0x00000000, 0x3c1aae34 },
	{ 0x4b100864, 0x00000000, 0x341be011 },
	{ 0x4b100868, 0x00000000, 0xaf5b000c },
	{ 0x4b10086c, 0x00000000, 0x0ac54252 },
	{ 0x4b100870, 0x00000000, 0x00000000 },
	{ 0x4b1213ec, 0x0ec54252, 0x0ec40218 },

	/* ack_action return -> marker f001 -> original caller. */
	{ 0x4b100880, 0x00000000, 0x3c1aae34 },
	{ 0x4b100884, 0x00000000, 0x341bf001 },
	{ 0x4b100888, 0x00000000, 0xaf5b0010 },
	{ 0x4b10088c, 0x00000000, 0x03e00008 },
	{ 0x4b100890, 0x00000000, 0x00000000 },
	{ 0x4b11eef0, 0x03e00008, 0x0ac40220 },

	/* RETURN_ACK dispatcher returned -> marker f002 -> original logger. */
	{ 0x4b1008a0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008a4, 0x00000000, 0x341bf002 },
	{ 0x4b1008a8, 0x00000000, 0xaf5b0010 },
	{ 0x4b1008ac, 0x00000000, 0x0ac54252 },
	{ 0x4b1008b0, 0x00000000, 0x00000000 },
	{ 0x4b121620, 0x0ec54252, 0x0ec40228 },

	/* RETURN_ACK wrapper return -> marker f003 -> original caller. */
	{ 0x4b1008c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1008c4, 0x00000000, 0x341bf003 },
	{ 0x4b1008c8, 0x00000000, 0xaf5b0010 },
	{ 0x4b1008cc, 0x00000000, 0x03e00008 },
	{ 0x4b1008d0, 0x00000000, 0x00000000 },
	{ 0x4b12163c, 0x03e00008, 0x0ac40230 },

	/* Semaphore failure log -> marker c103 -> original logger. */
	{ 0x4b1003e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1003e4, 0x00000000, 0x341bc103 },
	{ 0x4b1003e8, 0x00000000, 0xaf5b0000 },
	{ 0x4b1003ec, 0x00000000, 0x0ac54252 },
	{ 0x4b1003f0, 0x00000000, 0x00000000 },
	{ 0x4b12006c, 0x0ec54252, 0x0ec400f8 },

	/* Source callback entry: capture its event and tail-call queue send. */
	{ 0x4b1008e0, 0x00000000, 0x3c18ae34 }, /* lui t8, 0xae34 */
	{ 0x4b1008e4, 0x00000000, 0x34195101 }, /* ori t9, zero, 0x5101 */
	{ 0x4b1008e8, 0x00000000, 0xaf190020 }, /* sw t9, 0x20(t8) */
	{ 0x4b1008ec, 0x00000000, 0x8cb90000 }, /* lw t9, 0(a1) */
	{ 0x4b1008f0, 0x00000000, 0xaf190024 }, /* sw t9, 0x24(t8) */
	{ 0x4b1008f4, 0x00000000, 0x8cb90004 }, /* lw t9, 4(a1) */
	{ 0x4b1008f8, 0x00000000, 0xaf190028 }, /* sw t9, 0x28(t8) */
	{ 0x4b1008fc, 0x00000000, 0x0ac57118 }, /* j 0x8b15c460 */
	{ 0x4b100900, 0x00000000, 0x00000000 },
	{ 0x4b107580, 0x0ec57118, 0x0ec40238 }, /* jal 0x8b1008e0 */

	/* Source callback return: retain raw queue status before bool conversion. */
	{ 0x4b100920, 0x00000000, 0x3c18ae34 }, /* lui t8, 0xae34 */
	{ 0x4b100924, 0x00000000, 0xaf020030 }, /* sw v0, 0x30(t8) */
	{ 0x4b100928, 0x00000000, 0x34195102 }, /* ori t9, zero, 0x5102 */
	{ 0x4b10092c, 0x00000000, 0xaf190020 }, /* sw t9, 0x20(t8) */
	{ 0x4b100930, 0x00000000, 0x8fbf0014 }, /* lw ra, 0x14(sp) */
	{ 0x4b100934, 0x00000000, 0x2c420001 }, /* sltiu v0, v0, 1 */
	{ 0x4b100938, 0x00000000, 0x0ac41d64 }, /* j 0x8b107590 */
	{ 0x4b10093c, 0x00000000, 0x00000000 },
	{ 0x4b107588, 0x8fbf0014, 0x0ac40248 }, /* j 0x8b100920 */
	{ 0x4b10758c, 0x2c420001, 0x00000000 },

	/* Source-worker event zero: publish requested and previous source. */
	{ 0x4b100960, 0x00000000, 0x3c18ae34 }, /* lui t8, 0xae34 */
	{ 0x4b100964, 0x00000000, 0x34195201 }, /* ori t9, zero, 0x5201 */
	{ 0x4b100968, 0x00000000, 0xaf190034 }, /* sw t9, 0x34(t8) */
	{ 0x4b10096c, 0x00000000, 0xaf1e0028 }, /* sw fp, 0x28(t8) */
	{ 0x4b100970, 0x00000000, 0xaf16002c }, /* sw s6, 0x2c(t8) */
	{ 0x4b100974, 0x00000000, 0x13d60003 }, /* beq fp, s6, equal */
	{ 0x4b100978, 0x00000000, 0x00000000 },
	{ 0x4b10097c, 0x00000000, 0x0ac42557 }, /* j 0x8b10955c */
	{ 0x4b100980, 0x00000000, 0x00000000 },
	{ 0x4b100984, 0x00000000, 0x34195202 }, /* equal: source unchanged */
	{ 0x4b100988, 0x00000000, 0xaf190034 },
	{ 0x4b10098c, 0x00000000, 0x0ac424e8 }, /* j 0x8b1093a0 */
	{ 0x4b100990, 0x00000000, 0x24020001 }, /* original branch delay */
	{ 0x4b109554, 0x8fbe00d0, 0x0ac40258 }, /* j 0x8b100960 */
	{ 0x4b109558, 0x13d6ff91, 0x8fbe00d0 }, /* delay: lw fp, 0xd0(sp) */

	/* The general source-transition path returned to the worker loop. */
	{ 0x4b1009a0, 0x00000000, 0x3c18ae34 }, /* lui t8, 0xae34 */
	{ 0x4b1009a4, 0x00000000, 0x34195203 }, /* ori t9, zero, 0x5203 */
	{ 0x4b1009a8, 0x00000000, 0xaf190034 }, /* sw t9, 0x34(t8) */
	{ 0x4b1009ac, 0x00000000, 0x0ac424cf }, /* j 0x8b10933c */
	{ 0x4b1009b0, 0x00000000, 0x00000000 },
	{ 0x4b1095a0, 0x1000ff66, 0x0ac40268 }, /* j 0x8b1009a0 */

	/* THal_Vp_Init entry -> marker 7101 -> local state reset. */
	{ 0x4b1009c0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009c4, 0x00000000, 0x341b7101 },
	{ 0x4b1009c8, 0x00000000, 0xaf5b0038 },
	{ 0x4b1009cc, 0x00000000, 0x0ec5275c }, /* jal 0x8b149d70 */
	{ 0x4b1009d0, 0x00000000, 0x00000000 },
	{ 0x4b1009d4, 0x00000000, 0x0ac427c8 }, /* j 0x8b109f20 */
	{ 0x4b1009d8, 0x00000000, 0x00000000 },
	{ 0x4b109f18, 0x0ec5275c, 0x0ac40270 }, /* j 0x8b1009c0 */

	/* State reset returned -> marker 7102 -> local VP initializer. */
	{ 0x4b1009e0, 0x00000000, 0x3c1aae34 },
	{ 0x4b1009e4, 0x00000000, 0x341b7102 },
	{ 0x4b1009e8, 0x00000000, 0xaf5b0038 },
	{ 0x4b1009ec, 0x00000000, 0x0ec525d5 }, /* jal 0x8b149754 */
	{ 0x4b1009f0, 0x00000000, 0x00000000 },
	{ 0x4b1009f4, 0x00000000, 0x0ac427ca }, /* j 0x8b109f28 */
	{ 0x4b1009f8, 0x00000000, 0x00000000 },
	{ 0x4b109f20, 0x0ec525d5, 0x0ac40278 }, /* j 0x8b1009e0 */

	/* VP initializer returned -> marker 7103 -> original buffer load. */
	{ 0x4b100a00, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a04, 0x00000000, 0x341b7103 },
	{ 0x4b100a08, 0x00000000, 0xaf5b0038 },
	{ 0x4b100a0c, 0x00000000, 0x8e24000c }, /* lw a0, 0xc(s1) */
	{ 0x4b100a10, 0x00000000, 0x03e00008 }, /* jr ra */
	{ 0x4b100a14, 0x00000000, 0x00000000 },
	{ 0x4b109f28, 0x8e24000c, 0x0ec40280 }, /* jal 0x8b100a00 */

	/* Bulk copy returned -> marker 7104 -> original result setup. */
	{ 0x4b100a20, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a24, 0x00000000, 0x341b7104 },
	{ 0x4b100a28, 0x00000000, 0xaf5b0038 },
	{ 0x4b100a2c, 0x00000000, 0x24020001 }, /* addiu v0, zero, 1 */
	{ 0x4b100a30, 0x00000000, 0x03e00008 }, /* jr ra */
	{ 0x4b100a34, 0x00000000, 0x00000000 },
	{ 0x4b109f50, 0x24020001, 0x0ec40288 }, /* jal 0x8b100a20 */

	/* Output prepared -> marker 7105 -> original callback-install tail. */
	{ 0x4b100a40, 0x00000000, 0x3c1aae34 },
	{ 0x4b100a44, 0x00000000, 0x341b7105 },
	{ 0x4b100a48, 0x00000000, 0xaf5b0038 },
	{ 0x4b100a4c, 0x00000000, 0x0ac5305d }, /* j 0x8b14c174 */
	{ 0x4b100a50, 0x00000000, 0x00000000 },
	{ 0x4b109f74, 0x0ac5305d, 0x0ac40290 }, /* j 0x8b100a40 */
};

static void h713_mips_print_digest(const u8 *digest)
{
	int i;

	for (i = 0; i < SHA256_SUM_LEN; i++)
		printf("%02x", digest[i]);
}

static void h713_mips_stop(void)
{
	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_CLK_DISABLED, H713_MIPS_CLK_REG);
	mdelay(12);
	h713_display_prepared = false;
}

/*
 * Make the display blocks safe to touch. Nothing else.
 *
 * SPL leaves the display bus clock/reset enabled, but the module clocks and
 * their video2 parent are gated. Accessing the mixer or LVDS blocks before
 * enabling this complete tree wedges the interconnect.
 *
 * Preserve the cold-boot divider and mux fields. On the bench they yield
 * deint/panel=150 MHz, svp-dtl=200 MHz, and afbd=600 MHz.
 *
 * Split out of h713_display_prepare() so teardown can use it, which is what the
 * vendor does: its shutdown log reads
 * "ge2d 5240000.ge2d: acquire tvdisp clock on emergency shutdown" -- it turns
 * the display clock ON in order to turn the display off. That is the answer to
 * a problem our teardown had: it opens by READING the AFBD control register,
 * so it could only ever run when the sequence had already completed.
 *
 * Every register here is in the CCU at 0x02001xxx, which is always clocked, so
 * this is safe to call from any state. The gated blocks are the 0x05xxxxxx
 * ones, and this is what ungates them. setbits is idempotent, so calling it
 * when the tree is already up costs two delays and changes nothing.
 */
static void h713_display_clocks_on(void)
{
	setbits_le32((void *)H713_DISPLAY_PLL_VIDEO2_REG, BIT(31));
	mdelay(12);
	setbits_le32((void *)H713_DISPLAY_DEINT_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_PANEL_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_SVP_DTL_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_AFBD_CLK_REG, BIT(31));
	setbits_le32((void *)H713_DISPLAY_BGR_REG, BIT(16) | BIT(0));
	mdelay(12);
}

static void h713_display_prepare(void)
{
	h713_display_clocks_on();

	/*
	 * Recovered 1080p TVTOP routing. These values were bench-verified before
	 * the first mixer access; the initial SPL values open only part of the
	 * display fabric.
	 */
	writel(0x00000001, H713_DISPLAY_TOP_REG + 0x04);
	writel(0x11111111, H713_DISPLAY_TOP_REG + 0x44);
	writel(0x11111111, H713_DISPLAY_TOP_REG + 0x88);
	writel(0xfff11111, H713_DISPLAY_TOP_REG + 0x00);
	writel(0x00011111, H713_DISPLAY_TOP_REG + 0x40);
	writel(0x00001111, H713_DISPLAY_TOP_REG + 0x80);
	writel(0xfff000ef, H713_DISPLAY_TOP_REG + 0x84);
	mdelay(12);

	/*
	 * The factory state machine performs this mixer write in its earlier
	 * phase, before the final MIPS reset sequence.
	 */
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);
	mdelay(12);
	h713_display_prepared = true;
	printf("H713 MIPS: display clocks/routing prepared\n");
}

/*
 * Bring up the capture block before the MIPS leaves reset.
 *
 * The firmware writes 0xc0 to HDMI-RX register 0x06840093 and polls bit 0 for
 * a reset/lock acknowledgement (0x4b13d044). It never comes back, and the
 * timeout cannot rescue it: the tick is a software counter driven by the CP0
 * Compare ISR, and interrupts are still masked this early in startup, so the
 * wait loop spins forever and its caller never returns.
 *
 * An earlier attempt released TVCAP from the poll loop once marker 14 appeared
 * and wedged the interconnect. Doing it here instead matches the factory
 * ordering, where the capture clocks and reset are up before the coprocessor
 * runs at all.
 *
 * 0x02001d80's two bus gates are bits 31 (bus-hdmi-audio) and 30
 * (bus-cap-300m) per the vendor clock table; it reads 0xc0000000 out of the
 * boot loader, and bits 0/1, set here before, never latched. Callers print.
 */
static void h713_tvcap_prepare(void)
{
	setbits_le32((void *)H713_TVCAP_TCD3_CLK_REG, BIT(31));
	setbits_le32((void *)H713_TVCAP_VINCAP_DMA_CLK_REG, BIT(31));
	setbits_le32((void *)H713_TVCAP_HDMI_AUDIO_CLK_REG, BIT(31));
	mdelay(12);
	setbits_le32((void *)H713_TVCAP_BUS_CLK_REG, BIT(31) | BIT(30));
	mdelay(12);
	setbits_le32((void *)H713_TVCAP_BGR_REG, BIT(16) | BIT(0));
	mdelay(12);
}

/* Poll a domain's STATUS until (status & mask) == want; say so if it never does. */
static int h713_ppu_wait(uint domain, const char *name, const char *what,
			 u32 mask, u32 want)
{
	u32 status = readl(H713_PPU_STATUS(domain));
	int waited;

	for (waited = 0; (status & mask) != want &&
	     waited < H713_PPU_TIMEOUT_US; waited += 10) {
		udelay(10);
		status = readl(H713_PPU_STATUS(domain));
	}
	if ((status & mask) == want)
		return 0;
	printf("H713 PPU: %s %s after %d ms (status 0x%08x)\n",
	       name, what, H713_PPU_TIMEOUT_US / 1000, status);
	return -ETIMEDOUT;
}

/*
 * Switch one TV domain on the way the stock and mainline drivers do: wait for
 * the sequencer to go idle, command ON, wait for the transition to complete,
 * clear the sticky status bits by writing them back, then confirm the state.
 * Silent when the domain is already on. Returns 0 once it is on.
 */
static int h713_ppu_power_on(uint domain, const char *name)
{
	if ((readl(H713_PPU_STATUS(domain)) & H713_PPU_STATUS_STATE_MASK) ==
	    H713_PPU_STATUS_STATE_ON)
		return 0;

	writel(H713_PPU_WAIT_MODE_INIT, H713_PPU_WAIT_MODE(domain));
	writel(H713_PPU_DELAY_INIT, H713_PPU_PWR_ON_DELAY(domain));
	writel(H713_PPU_DELAY_INIT, H713_PPU_PWR_OFF_DELAY(domain));
	if (h713_ppu_wait(domain, name, "sequencer still busy",
			  H713_PPU_STATUS_BUSY, 0))
		return -ETIMEDOUT;
	writel(H713_PPU_CMD_ON, H713_PPU_PWR_CTRL(domain));
	if (h713_ppu_wait(domain, name, "power-on not complete",
			  H713_PPU_STATUS_DONE, H713_PPU_STATUS_DONE))
		return -ETIMEDOUT;
	writel(readl(H713_PPU_STATUS(domain)), H713_PPU_STATUS(domain));
	if (h713_ppu_wait(domain, name, "not reporting ON",
			  H713_PPU_STATUS_STATE_MASK, H713_PPU_STATUS_STATE_ON))
		return -ETIMEDOUT;
	printf("H713 PPU: %s switched on\n", name);
	return 0;
}

/*
 * The product path's capture bring-up, in the factory order: power domains,
 * then the capture clocks and reset, then (the caller) the coprocessor.
 * Without it the firmware runs its HDMI-RX init against an unpowered block
 * and its HDCP key-load wait on 0x06840093 can never be answered.
 *
 * A domain that will not switch on is reported and the run goes on: that is
 * the state every boot before this step had, and a firmware that then hangs
 * on the receiver says more than an aborted init would.
 */
static void h713_capture_prepare(void)
{
	int tvfe, tvcap;

	setbits_le32((void *)H713_R_PPU_BGR_REG, BIT(16) | BIT(0));
	tvfe = h713_ppu_power_on(H713_PPU_DOMAIN_TVFE, "TVFE");
	tvcap = h713_ppu_power_on(H713_PPU_DOMAIN_TVCAP, "TVCAP");
	h713_tvcap_prepare();
	if (tvfe || tvcap)
		printf("H713 MIPS: capture clocks up before release, but a TV "
		       "domain is NOT powered (see above)\n");
	else
		printf("H713 MIPS: TVFE/TVCAP powered, capture clocks up before "
		       "release\n");
}

/*
 * Zero everything the firmware may read except the two staged windows, so a
 * run does not depend on what the previous boot left in DRAM. Identical cold
 * boots have otherwise diverged by more than ten markers.
 */
static void h713_mips_clear_workspace(void)
{
	ulong tail = H713_MIPS_FW_ADDR + h713_mips_fw_size;

	memset((void *)tail, 0, H713_MIPS_CFG_ADDR - tail);
	memset((void *)H713_MIPS_FB_ADDR, 0, H713_MIPS_FB_SIZE);

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_CFG_ADDR - H713_MIPS_FW_ADDR);
	flush_cache(H713_MIPS_FB_ADDR, H713_MIPS_FB_SIZE);

	printf("H713 MIPS: workspace cleared (cfg@0x%08lx, tse@0x%08lx kept)\n",
	       H713_MIPS_CFG_ADDR, H713_MIPS_TSE_ADDR);
}

static u32 h713_mips_read_witness(void)
{
	invalidate_dcache_range(H713_MIPS_WITNESS_ADDR,
				H713_MIPS_WITNESS_ADDR +
				CONFIG_SYS_CACHELINE_SIZE);

	return readl(H713_MIPS_WITNESS_ADDR);
}

static void h713_mips_seed_witness(void)
{
	writel(H713_MIPS_WITNESS_SEED, H713_MIPS_WITNESS_ADDR);
	flush_cache(H713_MIPS_WITNESS_ADDR, CONFIG_SYS_CACHELINE_SIZE);
}

static u32 h713_mips_read_shmem(ulong offset)
{
	ulong addr = H713_MIPS_SHMEM_ADDR + offset;
	ulong start = addr & ~(CONFIG_SYS_CACHELINE_SIZE - 1);

	invalidate_dcache_range(start, start + CONFIG_SYS_CACHELINE_SIZE);

	return readl(addr);
}

/*
 * Read a word of the firmware's own memory through its MIPS virtual address.
 *
 * display.bin runs from KSEG0/KSEG1, so a VA maps to physical by masking the
 * segment off, and the MIPS's physical 0 is the ARM's 0x40000000 -- the same
 * conversion the firmware applies to itself when it publishes an ARM-visible
 * pointer (0x8b1197d4 builds base_addr as (VA & 0x1fffffff) + 0x40000000).
 *
 * The invalidate is not optional. A plain `md` of firmware BSS returns the
 * zeros U-Boot left there before the coprocessor started: that is why
 * `md.l 0x4b253e24` read all zeros on a boot where the FIFO indices, read
 * through this path, plainly showed the MIPS had written.
 */
static u32 h713_mips_read_fw(ulong va)
{
	ulong addr = (va & 0x1fffffffUL) + 0x40000000UL;
	ulong start = addr & ~(CONFIG_SYS_CACHELINE_SIZE - 1);

	invalidate_dcache_range(start, start + CONFIG_SYS_CACHELINE_SIZE);

	return readl(addr);
}

/*
 * The rest of the CPU_COMM master init, transcribed from the firmware.
 *
 * display.bin carries the ARM master's init as well as its own slave path.
 * getCurCPUID at 0x8b1227b4 hardcodes 1, so the master block at 0x8b11ace8 is
 * compiled in but never executed by the firmware -- it is the vendor's
 * specification of what the ARM must build. comm_InitSpinLock and the
 * call-table sentinels above came from the same block and were both correct on
 * hardware, which is the basis for trusting the rest of it.
 *
 * Three structures remain. Everything else the master path does either is
 * already published above or touches only firmware BSS (0x8b124ba4,
 * 0x8b119bb4) and imposes no obligation on the ARM.
 */
#define H713_COMM_MSGBOX_VERSION 0x03003810UL	/* User2 sub0 +0x10           */
#define H713_COMM_MSGBOX_COUNT	0x03003864UL	/* User2 sub0 +0x60 + 4*port  */
/*
 * The other direction. The firmware's msgbox send at 0x8b12199c forms its
 * addresses as base + (chan << 2) + (field << 10) + (user << 8) from the
 * handler registered by 0x8b121d98 (user 0, chan 1, dir 1), which lands on
 * 0x03003164/0x03003174 -- verified live: after our first accepted CALL the
 * count read 1 and the word read 0x00000002, a CALL_ACK.
 */
#define H713_COMM_MSGBOX_RX_COUNT 0x03003164UL	/* MIPS -> ARM, count         */
#define H713_COMM_MSGBOX_RX_DATA  0x03003174UL	/* MIPS -> ARM, MSG_DATA      */
#define H713_COMM_MSGBOX_BGR	0x0200171cUL	/* bit 0 gate, bit 16 reset   */
#define H713_COMM_MSGBOX_TX_IRQ_EN 0x03003830UL	/* User2 sub0 +0x30           */

#define H713_MIPS_SEQ_BASE_OFF		0x00000098UL
#define H713_MIPS_SEQ_STRIDE		2440
#define H713_MIPS_SEQ_PER_DIR		4880
#define H713_MIPS_SEQ_PER_CPU		9760
#define H713_MIPS_SEQ_SLOTS		20
#define H713_MIPS_SEQ_SLOT_SIZE		104
#define H713_MIPS_SEQ_RING_OFF		0x0c0
#define H713_MIPS_SEQ_SLOTS_OFF		0x168
#define H713_MIPS_SEQ_FIFO_OFF		0x078
#define H713_MIPS_SEQ_FIFO_CAPACITY	21

/*
 * Receiver-owned CallCmd/ReturnCmd backing rings for the ARM half of each
 * share_seq. The MIPS builds the corresponding cpu=1 rings in its own BSS,
 * but it cannot build the cpu=0 rings: their base_addr must be meaningful to
 * the ARM. SendComm2CPUEx checks the receiver's staging FIFO at +0x20 for
 * space before it allocates a FreeCall/FreeReturn slot, so leaving its
 * capacity zero makes the sender spin forever in fifo_isNearlyFull().
 */
static u32 h713_comm_staging_ring[2][2][H713_MIPS_SEQ_FIFO_CAPACITY];

/*
 * 0x8b1197d4(cpu, dir), addressed by 0x8b1193d8 as
 * shared + 0x98 + 9760*cpu + 4880*dir + 2440*idx. Eight structs of 0x988
 * bytes; the array ends exactly where max_cpu begins at 0x4cd8, which is the
 * check that the formula is right.
 *
 * The block at +0x78 is the vendor's comm_fifo, and the twenty 104-byte
 * message slots at +0x168 end exactly at 0x988. Each slot is stamped with its
 * index and an invalid session, then pushed onto the ring as its ARM-physical
 * address -- so the ring starts full of free slots, twenty in a capacity of
 * twenty-one, the spare being the full-detect slot.
 */
static void h713_mips_init_share_seq(uint cpu, uint dir, uint idx)
{
	ulong base = H713_MIPS_SHMEM_ADDR + H713_MIPS_SEQ_BASE_OFF +
		     cpu * H713_MIPS_SEQ_PER_CPU + dir * H713_MIPS_SEQ_PER_DIR +
		     idx * H713_MIPS_SEQ_STRIDE;
	ulong staging = base + 0x20;
	ulong slots = base + H713_MIPS_SEQ_SLOTS_OFF;
	ulong ring = base + H713_MIPS_SEQ_RING_OFF;
	ulong fifo = base + H713_MIPS_SEQ_FIFO_OFF;
	uint i;

	writeb(cpu, base + 0x00);
	writeb(dir, base + 0x01);
	writeb(idx, base + 0x02);
	writeb(0,   base + 0x08);
	writeb(H713_MIPS_SEQ_SLOTS, base + 0x10);
	writel(~0U, base + 0x14);
	writeb(H713_MIPS_SEQ_SLOTS, base + 0x68);
	writeb(0,   base + 0x69);
	writel(~0U, base + 0x6c);

	/*
	 * InitCommSeqMem owns the receive-side staging FIFO. U-Boot is the
	 * cpu=0 master, so initialise its four CallCmd/ReturnCmd headers and
	 * give each a distinct ARM-local backing ring. The MIPS will initialise
	 * the cpu=1 headers after release with MIPS-local base addresses.
	 */
	if (!cpu) {
		u32 *staging_ring = h713_comm_staging_ring[dir][idx];

		memset(staging_ring, 0,
		       sizeof(h713_comm_staging_ring[dir][idx]));
		writel(0, staging + 0x00);		/* rd_idx     */
		writel(0, staging + 0x04);		/* wr_idx     */
		writel(0, staging + 0x08);		/* peak_count */
		writel(1, staging + 0x0c);		/* track      */
		writel(H713_MIPS_SEQ_FIFO_CAPACITY, staging + 0x10);
		writel(4, staging + 0x14);		/* item_size  */
		writel((u32)(uintptr_t)staging_ring, staging + 0x18);
		writel(0, staging + 0x1c);
		strncpy((char *)(base + 0x40),
			idx ? "ReturnCmd" : "CallCmd", 0x20);
		writel(0, base + 0x60);
	}

	writel(0, fifo + 0x00);				/* rd_idx     */
	writel(H713_MIPS_SEQ_SLOTS, fifo + 0x04);	/* wr_idx     */
	/*
	 * The vendor initializer starts with wr=0 and pushes all twenty slots
	 * through fifo_requestItemWr(). With tracking enabled, those pushes leave
	 * peak_count=20. We populate the ring directly, so reproduce that side
	 * effect explicitly. A zero peak makes the first recycled slot restore a
	 * count of 20, trip fifo_getCount's count > peak+1 assertion, and spin in
	 * Comm_ReleaseFreeCall after wr has visibly wrapped 20 -> 0.
	 */
	writel(H713_MIPS_SEQ_SLOTS, fifo + 0x08);	/* peak_count */
	writel(1, fifo + 0x0c);				/* track      */
	writel(H713_MIPS_SEQ_FIFO_CAPACITY, fifo + 0x10);
	writel(4, fifo + 0x14);				/* item_size  */
	writel(ring, fifo + 0x18);			/* base_addr  */
	writel(0, fifo + 0x1c);

	/*
	 * The name is selected by idx, not dir: 0x8b1197d4 runs its body twice
	 * per (cpu, dir) -- idx 0 names the FIFO "FreeCall" at 0x8b119b84, then
	 * the tail at 0x8b119bac sets idx to 1 and repeats for "FreeReturn".
	 * That is why the addressing formula has an idx term and why all eight
	 * structures are live.
	 */
	strncpy((char *)(base + 0x98), idx ? "FreeReturn" : "FreeCall", 0x20);
	writel(0, base + 0xb8);

	for (i = 0; i < H713_MIPS_SEQ_SLOTS; i++) {
		ulong slot = slots + i * H713_MIPS_SEQ_SLOT_SIZE;

		writew(i, slot + 0x04);		/* comm_msg.slot_index */
		writel(~0U, slot + 0x0c);	/* comm_msg.session_id */
		writel(slot, ring + i * 4);	/* free slot, ARM-physical */
	}
}

static void h713_mips_init_share_seqs(void)
{
	uint cpu, dir, idx;

	for (cpu = 0; cpu < 2; cpu++)
		for (dir = 0; dir < 2; dir++)
			for (idx = 0; idx < 2; idx++)
				h713_mips_init_share_seq(cpu, dir, idx);
}

/*
 * Five empty circular lists, each {self, 0, self, 0}, written at 0x8b11af1c
 * onwards. The offsets are irregular because other state sits between them.
 */
static void h713_mips_init_lists(void)
{
	static const ulong off[] = {
		0x4d10, 0x4d28, 0x4d58, 0x4d70, 0x4d88,
	};
	uint i;

	for (i = 0; i < ARRAY_SIZE(off); i++) {
		ulong l = H713_MIPS_SHMEM_ADDR + off[i];

		writel(l, l + 0x00);
		writel(0, l + 0x04);
		writel(l, l + 0x08);
		writel(0, l + 0x0c);
	}
}

/*
 * 256 records of 0x28 bytes from 0x4db0 to 0x75b0 -- the loop at 0x8b11af70,
 * which lands exactly on magic2 at 0x75b8. Each gets the same self-referencing
 * head, then 0x8b1234c8(2, 0, record) appends it to the free list rooted at
 * 0x4d80: tail at +0x10, back link at record+0x20, anchor 0x4d88 at
 * record+0x18, and a u16 count at 0x4d82.
 */
#define H713_MIPS_REC_BASE_OFF	0x4db0UL
#define H713_MIPS_REC_END_OFF	0x75b0UL
#define H713_MIPS_REC_STRIDE	0x28
#define H713_MIPS_REC_LIST_OFF	0x4d80UL
#define H713_MIPS_REC_ANCHOR	0x4d88UL

/*
 * SMM (shared-memory allocator) heap, mirrored from Trid_SMM_Init() in the
 * arm64 cpu_comm driver. The heap region begins at shared+0x2ccf0 and runs to
 * the end of shared memory; its header is page-aligned up from there. The
 * MIPS smmMalloc() (display.bin 0x8b123040) locates a heap through the
 * descriptor slot at shared+0x4d00 + 8*index: {phys base, size}, and returns
 * NULL when both words read zero ("heap[%d] is not initialized!").
 */
#define H713_MIPS_SMM_HEAP_OFF		0x2ccf0UL
#define H713_MIPS_SMM_SLOT_OFF		0x4d00UL

static void h713_mips_init_record_pool(void)
{
	ulong list = H713_MIPS_SHMEM_ADDR + H713_MIPS_REC_LIST_OFF;
	ulong anchor = H713_MIPS_SHMEM_ADDR + H713_MIPS_REC_ANCHOR;
	ulong prev = 0;
	ulong off;
	u16 count = 0;

	for (off = H713_MIPS_REC_BASE_OFF; off < H713_MIPS_REC_END_OFF;
	     off += H713_MIPS_REC_STRIDE) {
		ulong r = H713_MIPS_SHMEM_ADDR + off;

		writel(r, r + 0x00);
		writel(0, r + 0x04);
		writel(r, r + 0x08);
		writel(0, r + 0x0c);

		writel(prev, r + 0x20);
		writel(anchor, r + 0x18);
		writel(0, r + 0x1c);
		writel(0, r + 0x24);
		if (prev)
			writel(0, prev + 0x04);

		prev = r + 0x18;
		writel(prev, list + 0x10);
		writel(0, list + 0x14);
		count++;
	}
	writew(count, list + 0x02);
}

/*
 * Lay down the SMM heap before the coprocessor runs.
 *
 * The firmware's hal_adapter_init() (0x8b10ad78) calls smmMalloc() for a
 * 44-byte signal-info buffer during app bring-up and stores the result in
 * sgp_hal_signal_info (0x8b253628) unchecked. The SignalChange adapter
 * (0x8b109fb0) later memcpy()s 44 bytes to that pointer BEFORE its null
 * check, on the first HDMI source switch. With no heap, smmMalloc() returns
 * NULL, the store to address 0 faults the MIPS, and its exception handler
 * (misaligned store, recursion) eats DRAM until the SoC is gone.
 *
 * Stock and the legacy arm32 stack never saw this because mipsloader started
 * the core from Linux, after Trid_SMM_Init() had run. Here U-Boot starts the
 * core first, and the arm64 driver's adoption path deliberately leaves
 * MIPS-shared state alone -- so the heap has to exist before this function
 * returns. Values are those of Trid_SMM_Init() (cpu_comm_mem.c) and match
 * analyse/hdmi-seq/smm_init.py; the mechanism is proven A/B in doku/74.
 *
 * Every pointer in the header is PHYSICAL: the MIPS reads the slot, masks to
 * 0x1fffffff and ORs 0xa0000000 for its uncached kseg1 view. The header
 * region is already zero from the memset in the caller.
 */
static void h713_mips_init_smm_heap(void)
{
	ulong heap = (H713_MIPS_SHMEM_ADDR + H713_MIPS_SMM_HEAP_OFF + 4095) &
		     ~4095UL;
	u32 size = (u32)(H713_MIPS_SHMEM_SIZE - 1 - H713_MIPS_SMM_HEAP_OFF);
	u32 page_count = (size + 4095) >> 12;
	u32 pt_phy = (u32)(heap + 4096);
	u32 data_start = ((12 * page_count + 4095) & ~4095U) + 4096;
	ulong slot = H713_MIPS_SHMEM_ADDR + H713_MIPS_SMM_SLOT_OFF;

	writel(pt_phy, heap + 0x00);		/* free_list_head */
	writel(pt_phy, heap + 0x04);		/* free_list_cur */
	writel(page_count, heap + 0x08);
	writel(data_start, heap + 0xb4);	/* data_start_offset */
	writel(size, heap + 0xb8);		/* raw capacity */
	writel((u32)heap, heap + 0xbc);		/* phys base of this header */

	/* descriptor slot 0 last: this is the word that arms the heap */
	writel(size, slot + 4);
	writel((u32)heap, slot + 0);

	printf("H713 MIPS: SMM heap prepared (base 0x%08lx, %u pages, "
	       "slot 0x%08lx)\n", heap, page_count, slot);
}

static void h713_mips_prepare_ready_probe(void)
{
	int i;

	memset((void *)H713_MIPS_SHMEM_ADDR, 0, H713_MIPS_SHMEM_SIZE);
	h713_comm_trace_active = false;

	/*
	 * InitCommMem takes the slave path when U-Boot publishes valid magic
	 * words. That path assumes the ARM master has already run
	 * comm_InitSpinLock(). Reproduce its exact 12-byte entry layout:
	 * type=free, status=free, mutex=0, owner=free, refcount=0, and no
	 * assigned thread. Leaving these bytes zero makes comm_SpinLock(3)
	 * wait forever before setCPUReady(1) can publish the MIPS flag.
	 */
	for (i = 0; i < H713_MIPS_SHMEM_LOCK_COUNT; i++) {
		u8 *lock = (u8 *)(H713_MIPS_SHMEM_ADDR +
				 i * H713_MIPS_SHMEM_LOCK_SIZE);

		lock[0] = H713_MIPS_SHMEM_LOCK_FREE;
		lock[1] = H713_MIPS_SHMEM_LOCK_FREE;
		lock[2] = 0;
		lock[3] = H713_MIPS_SHMEM_LOCK_FREE;
		*(u32 *)(lock + 4) = 0;
		*(u32 *)(lock + 8) = H713_MIPS_SHMEM_LOCK_THREAD_NONE;
	}

	/*
	 * The ARM master also clears the call-entry region, then seeds the
	 * per-entry link field with -1. The exact image does this at raw
	 * display.bin+0x1ae4c..0x1ae88: the table begins at shared+0x75c8,
	 * entries are 0x60 bytes, and the loop writes -1 at entry+0x5c until
	 * shared+0x240c4 (1224 entries). A zero link makes the MIPS insertion
	 * routine reject the first free slot before it can queue a request.
	 */
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_VERSION_OFF);
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_COUNT_OFF);
	for (i = 0; i < H713_MIPS_SHMEM_CALL_ENTRY_COUNT; i++)
		writel(~0U, H713_MIPS_SHMEM_ADDR +
		       H713_MIPS_SHMEM_CALL_TABLE_OFF +
		       i * H713_MIPS_SHMEM_CALL_ENTRY_SIZE +
		       H713_MIPS_SHMEM_CALL_NEXT_OFF);

	writel(3, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAX_CPU_OFF);
	writel(H713_MIPS_SHMEM_ARM_READY,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_ARM_FLAG_OFF);
	writel(0, H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	h713_mips_init_share_seqs();
	h713_mips_init_lists();
	h713_mips_init_record_pool();
	h713_mips_init_smm_heap();

	/*
	 * Bring the msgbox up before the coprocessor starts, not when a message
	 * is finally sent.
	 *
	 * Its bus gate and reset at 0x0200171c read zero from cold -- Linux
	 * takes CLK_BUS_MSGBOX/RST_BUS_MSGBOX, U-Boot never did. Enabling it at
	 * send time is too late: the firmware configures its own receive side
	 * during startup, and with the block gated those writes went nowhere.
	 * That is consistent with what the bench showed -- the doorbell reached
	 * the FIFO, the count went to one, and the MIPS never drained it.
	 */
	setbits_le32((void *)H713_COMM_MSGBOX_BGR, BIT(0) | BIT(16));
	udelay(20);

	/*
	 * Publish the magic words last. The MIPS firmware treats both markers,
	 * the ARM ready/app-ready flag, and an unlocked hardware spinlock 0 as
	 * the handoff from the ARM-side CPU_COMM master.
	 */
	writel(H713_MIPS_SHMEM_MAGIC,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAGIC1_OFF);
	writel(H713_MIPS_SHMEM_MAGIC,
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_MAGIC2_OFF);
	flush_cache(H713_MIPS_SHMEM_ADDR, H713_MIPS_SHMEM_SIZE);

	printf("H713 MIPS: readiness probe shared memory prepared "
	       "(12 spinlocks, 1224 call entries, 8 share_seq, "
	       "5 lists, 256 records)\n");
}

static int h713_mips_apply_trace(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_trace_patches[i];

		if (readl(patch->addr) != patch->expected) {
			printf("H713 MIPS: trace site 0x%08lx is not pristine\n",
			       patch->addr);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(h713_mips_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_trace_patches[i];

		writel(patch->replacement, patch->addr);
	}
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: volatile handshake trace installed\n");

	return 0;
}

static const char *h713_mips_comm_trace_stage_name(u32 stage)
{
	switch (stage) {
	case 0:
		return "no instrumented stage reached";
	case 0xa001:
		return "recycling FreeCall";
	case 0xa002:
		return "release semaphore acquired; pre-commit log entered";
	case 0xa003:
		return "requesting FreeCall FIFO write slot";
	case 0xa004:
		return "committing recycled FreeCall slot";
	case 0xa005:
		return "posting release semaphore";
	case 0xa105:
		return "release semaphore post failed";
	case 0xa006:
		return "release semaphore post succeeded";
	case 0xa007:
		return "final FreeCall release log entered";
	case 0xb001:
		return "routine lookup entered";
	case 0xb002:
		return "routine lookup returned";
	case 0xb003:
		return "handler arguments ready; pre-call log entered";
	case 0xb004:
		return "component handler entered";
	case 0xc001:
		return "component handler returned";
	case 0xc002:
		return "SendComm2CPUEx entered";
	case 0xc003:
		return "waiting for send semaphore";
	case 0xc103:
		return "send semaphore wait failed";
	case 0xc004:
		return "send semaphore acquired";
	case 0xc005:
		return "checking ReturnCmd FIFO space";
	case 0xc006:
		return "ReturnCmd FIFO has space";
	case 0xc007:
		return "allocating FreeReturn";
	case 0xc008:
		return "publishing RETURN";
	case 0xc009:
		return "waiting for RETURN_ACK";
	case 0xd001:
		return "RETURN_ACK interrupt queued deferred action";
	case 0xd005:
		return "RETURN_ACK queueAction returned";
	case 0xd002:
		return "RETURN_ACK action reached sender wakeup";
	case 0xd003:
		return "posting sender wait semaphore";
	case 0xd103:
		return "sender wait-semaphore post failed";
	case 0xd004:
		return "sender wait-semaphore post succeeded";
	case 0xc010:
		return "RETURN_ACK woke SendComm2CPUEx";
	case 0xc011:
		return "releasing SendComm2CPUEx send semaphore";
	case 0xc111:
		return "send-semaphore release failed";
	case 0xc012:
		return "send semaphore released";
	case 0xc013:
		return "SendComm2CPUEx returned to CALL worker";
	case 0xe001:
		return "CALL interrupt invoking queueAction";
	case 0xe004:
		return "queueAction returned to CALL interrupt";
	case 0xe005:
		return "CALL HISR invoking dispatcher";
	case 0xe006:
		return "command_action entered for CALL";
	case 0xe007:
		return "enqueueing high-priority CALL worker";
	case 0xe008:
		return "sending CALL_ACK";
	case 0xe009:
		return "SendAckLow returned";
	case 0xe010:
		return "command_action returning to CALL HISR";
	case 0xe011:
		return "CALL HISR dispatcher returned";
	case 0xf001:
		return "ack_action returning to RETURN_ACK HISR";
	case 0xf002:
		return "RETURN_ACK HISR dispatcher returned";
	case 0xf003:
		return "RETURN_ACK HISR wrapper returning";
	default:
		return "unknown stage";
	}
}

static const char *h713_mips_source_trace_stage_name(u32 stage)
{
	switch (stage) {
	case 0:
		return "not observed";
	case 0x5101:
		return "source callback received event";
	case 0x5102:
		return "source callback queue send returned";
	case 0x5201:
		return "source worker dequeued change";
	case 0x5202:
		return "source worker skipped unchanged source";
	case 0x5203:
		return "source worker completed transition";
	default:
		return "unknown source stage";
	}
}

static const char *h713_mips_vp_init_trace_stage_name(u32 stage)
{
	switch (stage) {
	case 0:
		return "not entered";
	case 0x7101:
		return "calling local state reset";
	case 0x7102:
		return "calling local VP initializer";
	case 0x7103:
		return "loading output-buffer physical address";
	case 0x7104:
		return "bulk state copy returned";
	case 0x7105:
		return "entering callback installation";
	default:
		return "unknown VP-init stage";
	}
}

static void h713_mips_print_comm_trace(void)
{
	u32 magic = h713_mips_read_shmem(H713_MIPS_COMM_TRACE_MAGIC_OFF);
	u32 stage = h713_mips_read_shmem(H713_MIPS_COMM_TRACE_STAGE_OFF);
	u32 call_stage = h713_mips_read_shmem(H713_MIPS_COMM_TRACE_CALL_OFF);
	u32 queue_status = h713_mips_read_shmem(H713_MIPS_COMM_TRACE_QUEUE_OFF);
	u32 ack_stage = h713_mips_read_shmem(H713_MIPS_COMM_TRACE_ACK_OFF);
	u32 source_stage = h713_mips_read_shmem(
		H713_MIPS_SOURCE_TRACE_STAGE_OFF);
	u32 source_event = h713_mips_read_shmem(
		H713_MIPS_SOURCE_TRACE_EVENT_OFF);
	u32 source_new = h713_mips_read_shmem(H713_MIPS_SOURCE_TRACE_NEW_OFF);
	u32 source_old = h713_mips_read_shmem(H713_MIPS_SOURCE_TRACE_OLD_OFF);
	u32 source_queue = h713_mips_read_shmem(
		H713_MIPS_SOURCE_TRACE_QUEUE_OFF);
	u32 source_worker = h713_mips_read_shmem(
		H713_MIPS_SOURCE_TRACE_WORKER_OFF);
	u32 vp_init_stage = h713_mips_read_shmem(H713_MIPS_VP_INIT_TRACE_OFF);

	if (magic != H713_MIPS_COMM_TRACE_MAGIC) {
		printf("H713 comm trace: not installed for this boot "
		       "(magic=%08x)\n", magic);
		return;
	}

	printf("H713 comm trace: stage=0x%04x (%s)\n", stage,
	       h713_mips_comm_trace_stage_name(stage));
	printf("H713 comm trace: CALL=0x%04x (%s), HISR queue send=%08x%s\n",
	       call_stage, h713_mips_comm_trace_stage_name(call_stage),
	       queue_status, queue_status == 0 ? " (success)" : "");
	printf("H713 comm trace: RETURN_ACK=0x%04x (%s)\n", ack_stage,
	       h713_mips_comm_trace_stage_name(ack_stage));
	printf("H713 VP-init trace: stage=0x%04x (%s)\n", vp_init_stage,
	       h713_mips_vp_init_trace_stage_name(vp_init_stage));
	printf("H713 source trace: callback=0x%04x (%s), worker=0x%04x (%s), "
	       "event=%u new=%u old=%u queue=%08x%s\n", source_stage,
	       h713_mips_source_trace_stage_name(source_stage), source_worker,
	       h713_mips_source_trace_stage_name(source_worker), source_event,
	       source_new, source_old, source_queue,
	       source_stage >= 0x5102 && source_queue == 0 ? " (success)" : "");
}

static int h713_mips_apply_comm_trace(void)
{
	ulong trace = H713_MIPS_SHMEM_ADDR + H713_MIPS_TRACE_OFF;
	int i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_comm_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_comm_trace_patches[i];

		if (readl(patch->addr) != patch->expected) {
			printf("H713 MIPS: comm trace site 0x%08lx is not pristine\n",
			       patch->addr);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(h713_mips_comm_trace_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_comm_trace_patches[i];

		writel(patch->replacement, patch->addr);
	}
	writel(0, trace);
	writel(H713_MIPS_COMM_TRACE_MAGIC, trace + 4);
	writel(~0U, trace + 8);
	writel(0, trace + 12);
	writel(0, trace + 16);
	writel(0, trace + 0x20);
	writel(~0U, trace + 0x24);
	writel(~0U, trace + 0x28);
	writel(~0U, trace + 0x2c);
	writel(~0U, trace + 0x30);
	writel(0, trace + 0x34);
	writel(0, trace + 0x38);
	flush_cache(trace, CONFIG_SYS_CACHELINE_SIZE);
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	h713_comm_trace_active = true;
	printf("H713 MIPS: CPU_COMM RETURN progress trace installed\n");

	return 0;
}

static int h713_mips_apply_stability(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(h713_mips_stability_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_stability_patches[i];

		if (readl(patch->addr) != patch->expected) {
			printf("H713 MIPS: stability site 0x%08lx is not pristine\n",
			       patch->addr);
			return -EINVAL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(h713_mips_stability_patches); i++) {
		const struct h713_mips_patch *patch =
			&h713_mips_stability_patches[i];

		writel(patch->replacement, patch->addr);
	}
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: uncached heartbeat/exception diagnostics installed\n");

	return 0;
}

/*
 * Report every trace slot that changed since the previous scan, as it changes.
 * The firmware's stalls wedge the interconnect often enough that a dump taken
 * after the poll loop is the one thing a failing run cannot produce. Streaming
 * costs one UART line per marker and makes the last line printed before a hang
 * the answer.
 */
static void h713_mips_stream_trace(u32 *shadow)
{
	ulong base = H713_MIPS_SHMEM_ADDR + H713_MIPS_TRACE_OFF;
	ulong start = base & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
	ulong end = ALIGN(base + H713_MIPS_TRACE_COUNT * sizeof(u32),
			  CONFIG_SYS_CACHELINE_SIZE);
	int i;

	invalidate_dcache_range(start, end);

	for (i = 0; i < H713_MIPS_TRACE_COUNT; i++) {
		u32 value = readl(base + i * sizeof(u32));

		if (value == shadow[i])
			continue;

		shadow[i] = value;
		if (i == H713_MIPS_TRACE_DBG_ADDR)
			printf("H713 MIPS: sys:dbg_buf=0x%08x\n", value);
		else if (i == H713_MIPS_TRACE_DBG_SIZE)
			printf("H713 MIPS: sys:dbg_buf_size=0x%08x\n", value);
		else if (i == H713_MIPS_TRACE_REG_COUNT)
			printf("H713 MIPS: hal registration count=%u\n", value);
		else if (i == H713_MIPS_TRACE_REG_OBJECT)
			printf("H713 MIPS: hal registration object=0x%08x\n",
			       value);
		else if (i == H713_MIPS_TRACE_REG_CALLBACK)
			printf("H713 MIPS: hal registration callback=0x%08x\n",
			       value);
		else
			printf("H713 MIPS: trace[%d]=%u\n", i, value);
	}
}

static void h713_mips_print_trace(void)
{
	int i;

	printf("H713 MIPS: handshake trace");
	for (i = 0; i < H713_MIPS_TRACE_MARKER_COUNT; i++)
		printf(" %u", h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
						   i * sizeof(u32)));
	printf("\n");
	printf("H713 MIPS: debug buffer addr=0x%08x size=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_DBG_ADDR * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_DBG_SIZE * sizeof(u32)));
	printf("H713 MIPS: hal registrations=%u object=0x%08x "
	       "callback=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_COUNT * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_OBJECT * sizeof(u32)),
	       h713_mips_read_shmem(H713_MIPS_TRACE_OFF +
				   H713_MIPS_TRACE_REG_CALLBACK * sizeof(u32)));
	printf("H713 MIPS: call table version=%u count=%u first-next=0x%08x\n",
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_VERSION_OFF),
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_COUNT_OFF),
	       h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_TABLE_OFF +
				   H713_MIPS_SHMEM_CALL_NEXT_OFF));
}

/*
 * The whole ladder in one place: clock, then bus-reset, cold-reset, bus gate,
 * and finally soft-reset with the boot address already written. The stages and
 * their order are the vendor kernel's mips_reset() exactly; see the reset
 * register's own comment for which bit is which.
 */
static int h713_mips_release_reset(bool publish_shmem)
{
	writel(H713_MIPS_CLK_VALUE, H713_MIPS_CLK_REG);
	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE1, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE2, H713_MIPS_RESET_REG);
	mdelay(12);
	writel(H713_MIPS_RESET_STAGE3, H713_MIPS_RESET_REG);
	mdelay(12);

	/*
	 * These two and the boot address below are the hand-over contract.
	 *
	 * The vendor kernel's sunxi-mipsloader reads exactly them back as
	 * base[9], base[10] and base[12] -- 0x03061024, 0x03061028,
	 * 0x03061030 -- and mips_powerdown()/mips_reset() then carry them
	 * around unchanged; it never re-derives them. So whatever stands here
	 * when we hand over is what the driver believes for the rest of the
	 * session, and it has to keep agreeing with the memory map in
	 * display_cfg.xml. The read-back below is not paranoia: if publication
	 * failed, the driver would inherit whatever was in the register.
	 *
	 * publish_shmem is false only on the paths that start the firmware
	 * without the CPU_COMM handshake -- "h713_mips start" and the bare
	 * h713_disp run. Those are bench commands. The boot path passes true.
	 */
	if (publish_shmem) {
		writel(H713_MIPS_SHMEM_ADDR, H713_MIPS_SHARE_ADDR_REG);
		writel(H713_MIPS_SHMEM_SIZE, H713_MIPS_SHARE_SIZE_REG);
		if (readl(H713_MIPS_SHARE_ADDR_REG) != H713_MIPS_SHMEM_ADDR ||
		    readl(H713_MIPS_SHARE_SIZE_REG) != H713_MIPS_SHMEM_SIZE) {
			printf("H713 MIPS: share-register publication failed\n");
			return -EIO;
		}
	}

	writel(H713_MIPS_FW_ADDR, H713_MIPS_BOOTADDR_REG);
	writel(H713_MIPS_RESET_RELEASED, H713_MIPS_RESET_REG);

	return 0;
}

static int h713_mips_verify(void)
{
	u8 digest[SHA256_SUM_LEN];

	uint i;

	/* A previous load's identity must not survive into this one. */
	h713_mips_fw = NULL;

	sha256_csum_wd((const u8 *)H713_MIPS_FW_ADDR, h713_mips_fw_size,
		       digest, CHUNKSZ_SHA256);

	printf("H713 MIPS: display.bin SHA-256 ");
	h713_mips_print_digest(digest);
	printf("\n");

	for (i = 0; i < ARRAY_SIZE(h713_mips_fw_revs); i++) {
		if (memcmp(digest, h713_mips_fw_revs[i].digest,
			   SHA256_SUM_LEN))
			continue;

		h713_mips_fw = &h713_mips_fw_revs[i];
		printf("H713 MIPS: firmware identity accepted (%s)\n",
		       h713_mips_fw->board);
		return 0;
	}

	if (h713_probe_mode) {
		printf("H713 MIPS: firmware identity unknown -- a revision this\n"
		       "           build has never seen. Size 0x%lx and the\n"
		       "           digest above are what a table row needs.\n",
		       h713_mips_fw_size);
		return 0;
	}

	printf("H713 MIPS: firmware identity rejected\n");
	return -EPERM;
}

static int h713_mips_load(const char *ifname, const char *dev,
			  const char *path)
{
	loff_t file_size;
	loff_t len_read;
	int ret;

	ret = fs_set_blk_dev(ifname, dev, FS_TYPE_ANY);
	if (ret) {
		printf("H713 MIPS: cannot select %s %s\n", ifname, dev);
		return ret;
	}

	ret = fs_size(path, &file_size);
	if (ret) {
		printf("H713 MIPS: cannot stat %s\n", path);
		return ret;
	}

	ret = h713_mips_accept_size((ulong)file_size);
	if (ret)
		return ret;

	h713_mips_stop();
	memset((void *)H713_MIPS_FW_ADDR, 0, H713_MIPS_FW_WINDOW_SIZE);

	ret = fs_set_blk_dev(ifname, dev, FS_TYPE_ANY);
	if (ret)
		return ret;

	len_read = file_size;
	ret = fs_read(path, H713_MIPS_FW_ADDR, 0, file_size, &len_read);
	if (ret || len_read != file_size) {
		printf("H713 MIPS: read failed (%d, 0x%llx/0x%llx bytes)\n",
		       ret, len_read, file_size);
		h713_mips_stop();
		return ret ? ret : -EIO;
	}

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	printf("H713 MIPS: loaded %s to 0x%08lx (0x%llx bytes)\n",
	       path, H713_MIPS_FW_ADDR, len_read);

	ret = h713_mips_verify();
	if (ret)
		h713_mips_stop();

	return ret;
}

static int h713_mips_start(void)
{
	u32 status;
	int ret;

	ret = h713_mips_verify();
	if (ret) {
		h713_mips_stop();
		return ret;
	}

	/*
	 * loady and other generic loaders do not clear the firmware's runtime
	 * tail. Match the filesystem load path so BSS/heap never inherit stale
	 * DRAM from an earlier boot.
	 */
	memset((void *)(H713_MIPS_FW_ADDR + h713_mips_fw_size), 0,
	       H713_MIPS_FW_WINDOW_SIZE - h713_mips_fw_size);
	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);

	/*
	 * The firmware entry point clears BSS from 0x4b232c00 through
	 * 0x4bac7c28 before entering C code. Seed a word within that range but
	 * just beyond the ARM-cleared load window. Observing zero here proves
	 * that the MIPS executed a firmware-owned store; CPU status alone only
	 * proves reset release.
	 */
	h713_mips_seed_witness();
	h713_display_prepare();

	ret = h713_mips_release_reset(false);
	if (ret) {
		h713_mips_stop();
		return ret;
	}
	mdelay(300);

	status = readl(H713_MIPS_STATUS_REG);
	printf("H713 MIPS: reset released, CPU status=0x%08x\n", status);

	if (status != H713_MIPS_STATUS_RELEASED) {
		printf("H713 MIPS: unexpected CPU status; returning to reset\n");
		h713_mips_stop();
		return -EIO;
	}

	status = h713_mips_read_witness();
	printf("H713 MIPS: BSS witness@0x%08lx=0x%08x\n",
	       H713_MIPS_WITNESS_ADDR, status);
	if (status) {
		printf("H713 MIPS: no execution witness; returning to reset\n");
		h713_mips_stop();
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware execution proven by BSS clear\n");
	printf("H713 MIPS: higher-level firmware readiness is not yet proven\n");

	return 0;
}

/*
 * Release the MIPS the way stock fastlogo does, without touching the display
 * fabric.
 *
 * Stock configures the fabric from LogoRegData.bin, writes INCAP and the LVDS
 * enable, releases the coprocessor, waits 300 ms and finalises LVDS. Replaying
 * that needs a release step that leaves the vendor register state alone --
 * h713_mips_start() would re-apply h713_display_prepare() over the top of it.
 *
 * The cache flush is the part that is easy to miss: fastboot staging writes
 * through the ARM cache, so a verify reads back correct while DRAM still holds
 * stale bytes and the MIPS fetches garbage.
 */
/*
 * Rx_HDCP14_LoadKey polls HDMI-RX 0x06840093 and gives up after 0x33 ticks.
 * The tick is a ThreadX software counter driven by the CP0 Compare ISR, and
 * interrupts are masked this early, so the timeout cannot expire and a failure
 * the firmware is built to survive becomes a hang. Rewriting the loop bound to
 * zero makes the wait give up on its first pass, taking the same path a real
 * timeout would. Applied after the hash check, so the image is authenticated
 * before it is modified.
 *
 * The address is firmware-specific: the same loop sits at 0x4b13d0a4 in the
 * revision found in an earlier captured dump and at 0x4b13d6f8 in the image
 * this board actually carries.
 *
 * One constant per revision does not reach past the boards we own. A third
 * device cannot be helped until someone with that device reads the address
 * out, and reading it out needs the very command the missing address blocks.
 * So find the site in the image instead. The instruction is "sltiu v1,v1,0x33"
 * (0x2c630033), and on this board's image it occurs twice -- once in
 * Rx_HDCP14_LoadKey, once in an unrelated function. Context tells them apart:
 * only the real site has the polled register 0x06840093 built nearby, as the
 * halves a MIPS lui/ori pair leaves in the instruction stream. Measured on
 * display.bin 16c74a28...: the hit at 0x4b13d0a4 has both halves within 1 KiB,
 * the one at 0x4b161478 has neither. The pinned addresses stay in the table,
 * as a check on the search rather than as its input.
 */
#define H713_MIPS_HDCP_WAIT_ORIG	0x2c630033
#define H713_MIPS_HDCP_WAIT_NONE	0x2c630000
/* Halves of the HDMI-RX register 0x06840093 the loop polls, and how far
 * either side of a candidate to look for them. */
#define H713_MIPS_HDCP_CTX_HI		0x0684
#define H713_MIPS_HDCP_CTX_LO		0x0093
#define H713_MIPS_HDCP_CTX_WIN		0x400UL

static bool h713_mips_hdcp_context(ulong va)
{
	ulong first = H713_MIPS_FW_ADDR;
	ulong last = H713_MIPS_FW_ADDR + h713_mips_fw_size;
	ulong from = (va - first > H713_MIPS_HDCP_CTX_WIN)
		     ? va - H713_MIPS_HDCP_CTX_WIN : first;
	ulong to = min(va + H713_MIPS_HDCP_CTX_WIN, last);
	bool hi = false, lo = false;
	ulong p;

	for (p = from; p + 2 <= to; p += 2) {
		u16 half = readw(p);

		if (half == H713_MIPS_HDCP_CTX_HI)
			hi = true;
		else if (half == H713_MIPS_HDCP_CTX_LO)
			lo = true;
	}

	return hi && lo;
}

/* Zero on failure, with the reason printed: no caller can act on it. */
static ulong h713_mips_find_hdcp_wait(void)
{
	ulong found = 0;
	uint hits = 0;
	ulong p;

	for (p = H713_MIPS_FW_ADDR;
	     p + 4 <= H713_MIPS_FW_ADDR + h713_mips_fw_size; p += 4) {
		if (readl(p) != H713_MIPS_HDCP_WAIT_ORIG)
			continue;
		hits++;
		if (!h713_mips_hdcp_context(p))
			continue;
		if (found) {
			printf("H713 MIPS: HDCP wait site is ambiguous -- "
			       "0x%08lx and 0x%08lx both look right\n",
			       found, p);
			return 0;
		}
		found = p;
	}

	if (!found)
		printf("H713 MIPS: no HDCP wait site found (%u candidate%s "
		       "had the instruction, none the context)\n",
		       hits, hits == 1 ? "" : "s");

	return found;
}

static int h713_mips_release_raw(bool skip_hdcp_wait, bool publish_shmem,
				 bool trace, bool stability, bool comm_trace)
{
	u32 status, witness;
	int elapsed;
	int ret;

	ret = h713_mips_verify();
	if (ret)
		return ret;

	if ((trace || stability || comm_trace) && !publish_shmem)
		return -EINVAL;
	if ((trace && stability) || (trace && comm_trace) ||
	    (stability && comm_trace))
		return -EINVAL;

	if (skip_hdcp_wait) {
		ulong pinned = h713_mips_fw ? h713_mips_fw->hdcp_wait_va : 0;
		ulong va = h713_mips_find_hdcp_wait();

		if (!va)
			return -EINVAL;
		if (pinned && pinned != va)
			printf("H713 MIPS: HDCP wait site found at 0x%08lx but "
			       "the table says 0x%08lx -- using the search\n",
			       va, pinned);
		writel(H713_MIPS_HDCP_WAIT_NONE, va);
		printf("H713 MIPS: HDCP key-load wait defeated at 0x%08lx\n",
		       va);
	}

	if (publish_shmem)
		h713_mips_prepare_ready_probe();

	if (trace) {
		ret = h713_mips_apply_trace();
		if (ret)
			return ret;
		memset(trace_shadow, 0, sizeof(trace_shadow));
	}
	if (stability) {
		ret = h713_mips_apply_stability();
		if (ret)
			return ret;
	}
	if (comm_trace) {
		ret = h713_mips_apply_comm_trace();
		if (ret)
			return ret;
	}

	flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
	h713_mips_seed_witness();

	ret = h713_mips_release_reset(publish_shmem);
	if (ret)
		return ret;
	if (trace) {
		for (elapsed = 0; elapsed < 300000; elapsed += 1000) {
			h713_mips_stream_trace(trace_shadow);
			udelay(1000);
		}
		h713_mips_stream_trace(trace_shadow);
	} else {
		mdelay(300);
	}

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	printf("H713 MIPS: released, status=0x%08x witness=0x%08x\n",
	       status, witness);

	if (status != H713_MIPS_STATUS_RELEASED) {
		printf("H713 MIPS: unexpected CPU status\n");
		return -EIO;
	}
	/*
	 * The seed sits inside the firmware's BSS (0x4b232c00..0x4bac7c40), so
	 * startup zeroes it -- but the firmware then *uses* that memory, and
	 * with elog buffering enabled it stores a pointer there. Demanding a
	 * zero read therefore reports a false failure on a perfectly healthy
	 * run. Any value other than the seed proves the MIPS wrote to it.
	 */
	if (witness == H713_MIPS_WITNESS_SEED) {
		printf("H713 MIPS: seed intact -- firmware not executing\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware execution proven (witness overwritten)\n");
	return 0;
}

static int h713_mips_monitor_stability(void)
{
	u32 previous = h713_mips_read_shmem(H713_MIPS_DIAG_HEARTBEAT_OFF);
	bool advanced = true;
	int second;

	printf("H713 MIPS: starting %d-second stability window, tick=%u\n",
	       H713_MIPS_STABILITY_SECONDS, previous);
	for (second = 1; second <= H713_MIPS_STABILITY_SECONDS; second++) {
		u32 exception;
		u32 heartbeat;
		u32 mips_flag;
		u32 status;

		mdelay(1000);
		heartbeat =
			h713_mips_read_shmem(H713_MIPS_DIAG_HEARTBEAT_OFF);
		exception =
			h713_mips_read_shmem(H713_MIPS_DIAG_EXCEPTION_OFF);
		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		status = readl(H713_MIPS_STATUS_REG);

		printf("H713 MIPS: stability %2ds tick=%u delta=%u "
		       "status=%08x MIPS=%08x exception=%u\n",
		       second, heartbeat, heartbeat - previous, status,
		       mips_flag, exception);

		if (exception) {
			printf("H713 MIPS: %s exception status=%08x cause=%08x "
			       "epc=%08x badvaddr=%08x\n",
			       exception == H713_MIPS_DIAG_EXCEPTION_CACHE ?
			       "cache" : "general",
			       h713_mips_read_shmem(H713_MIPS_DIAG_STATUS_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_CAUSE_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_EPC_OFF),
			       h713_mips_read_shmem(H713_MIPS_DIAG_BADVADDR_OFF));
			return -EFAULT;
		}
		if (heartbeat == previous)
			advanced = false;
		if (status != H713_MIPS_STATUS_RELEASED ||
		    (mips_flag & (H713_MIPS_SHMEM_MIPS_READY |
				  H713_MIPS_SHMEM_MIPS_APP_READY)) !=
		    (H713_MIPS_SHMEM_MIPS_READY |
		     H713_MIPS_SHMEM_MIPS_APP_READY)) {
			printf("H713 MIPS: readiness/status changed during "
			       "stability window\n");
			return -EIO;
		}
		previous = heartbeat;
	}

	if (!advanced) {
		printf("H713 MIPS: stability failed: heartbeat stalled during "
		       "the observation window\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: stability passed: heartbeat advanced for %d seconds "
	       "with no recorded exception\n",
	       H713_MIPS_STABILITY_SECONDS);
	return 0;
}

static int h713_mips_wait_ready(int timeout_us, bool trace)
{
	u32 arm_flag;
	u32 magic1;
	u32 magic2;
	u32 mips_flag = 0;
	u32 status;
	u32 witness;
	int elapsed;

	for (elapsed = 0; elapsed < timeout_us; elapsed += 1000) {
		if (trace)
			h713_mips_stream_trace(trace_shadow);
		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		if ((mips_flag & (H713_MIPS_SHMEM_MIPS_READY |
				 H713_MIPS_SHMEM_MIPS_APP_READY)) ==
		    (H713_MIPS_SHMEM_MIPS_READY |
		     H713_MIPS_SHMEM_MIPS_APP_READY))
			break;
		udelay(1000);
	}

	if (trace) {
		h713_mips_stream_trace(trace_shadow);
		h713_mips_print_trace();
	}

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	magic1 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF);
	magic2 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC2_OFF);
	arm_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_ARM_FLAG_OFF);
	mips_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	printf("H713 MIPS: readiness status=0x%08x witness=0x%08x\n",
	       status, witness);
	printf("H713 MIPS: CPU_COMM magic=%08x/%08x ARM=%08x MIPS=%08x\n",
	       magic1, magic2, arm_flag, mips_flag);

	if (status != H713_MIPS_STATUS_RELEASED ||
	    witness == H713_MIPS_WITNESS_SEED ||
	    magic1 != H713_MIPS_SHMEM_MAGIC ||
	    magic2 != H713_MIPS_SHMEM_MAGIC ||
	    arm_flag != H713_MIPS_SHMEM_ARM_READY ||
	    !(mips_flag & H713_MIPS_SHMEM_MIPS_READY)) {
		printf("H713 MIPS: firmware readiness not proven; core left running\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: firmware readiness proven by MIPS READY\n");
	if (!(mips_flag & H713_MIPS_SHMEM_MIPS_APP_READY)) {
		printf("H713 MIPS: application readiness not proven; "
		       "core left running\n");
		return -ETIMEDOUT;
	}

	printf("H713 MIPS: application readiness proven\n");
	return 0;
}

static int h713_mips_probe_ready(bool trace, bool release_tvcap,
				 bool skip_wait)
{
	u32 tvcap_saved_tcd3 = 0;
	u32 tvcap_saved_vincap = 0;
	u32 tvcap_saved_bus = 0;
	u32 tvcap_saved_hdmi_audio = 0;
	u32 tvcap_saved_bgr = 0;
	u32 arm_flag;
	u32 magic1;
	u32 magic2;
	u32 mips_flag = 0;
	u32 status;
	u32 witness;
	bool tvcap_released = false;
	int elapsed;
	int ret;
	int timeout = trace ? H713_MIPS_TRACE_TIMEOUT_US :
			      H713_MIPS_READY_TIMEOUT_US;

	ret = h713_mips_verify();
	if (ret) {
		h713_mips_stop();
		return ret;
	}

	h713_mips_clear_workspace();
	h713_mips_seed_witness();
	h713_mips_prepare_ready_probe();
	if (trace) {
		ret = h713_mips_apply_trace();
		if (ret)
			goto out_stop;
	}
	/*
	 * Rx_HDCP14_LoadKey polls HDMI-RX 0x06840093 bit 0 for a key-load
	 * acknowledgement and gives up after 0x33 ticks with "time out!". The
	 * tick is a software counter driven by the CP0 Compare ISR, so while
	 * interrupts are masked this early the timeout can never expire and a
	 * failure the firmware is designed to survive becomes a hang.
	 *
	 * Rewrite the loop bound inside marker 104's cave so the wait gives up
	 * on its first iteration, taking the same path a real timeout would.
	 */
	if (skip_wait) {
		writel(0x2c630000, 0x4b101168);
		flush_cache(H713_MIPS_FW_ADDR, H713_MIPS_FW_WINDOW_SIZE);
		printf("H713 MIPS: tick wait loops forced to expire at once\n");
	}

	h713_display_prepare();

	/* Capture the cold values before enabling, so stop() can undo this. */
	if (release_tvcap) {
		tvcap_saved_tcd3 = readl(H713_TVCAP_TCD3_CLK_REG);
		tvcap_saved_vincap = readl(H713_TVCAP_VINCAP_DMA_CLK_REG);
		tvcap_saved_bus = readl(H713_TVCAP_BUS_CLK_REG);
		tvcap_saved_hdmi_audio =
			readl(H713_TVCAP_HDMI_AUDIO_CLK_REG);
		tvcap_saved_bgr = readl(H713_TVCAP_BGR_REG);

		h713_tvcap_prepare();
		printf("H713 MIPS: TVCAP clocks/reset prepared before release\n");
		tvcap_released = true;
	}

	ret = h713_mips_release_reset(true);
	if (ret)
		goto out_stop;

	if (trace)
		memset(trace_shadow, 0, sizeof(trace_shadow));

	for (elapsed = 0; elapsed < timeout; elapsed += 1000) {
		if (trace)
			h713_mips_stream_trace(trace_shadow);


		mips_flag =
			h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);
		if (mips_flag & H713_MIPS_SHMEM_MIPS_READY)
			break;
		udelay(1000);
	}

	if (trace)
		h713_mips_stream_trace(trace_shadow);

	status = readl(H713_MIPS_STATUS_REG);
	witness = h713_mips_read_witness();
	magic1 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF);
	magic2 = h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC2_OFF);
	arm_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_ARM_FLAG_OFF);
	mips_flag = h713_mips_read_shmem(H713_MIPS_SHMEM_MIPS_FLAG_OFF);

	if (trace)
		h713_mips_print_trace();
	printf("H713 MIPS: readiness probe status=0x%08x witness=0x%08x\n",
	       status, witness);
	printf("H713 MIPS: CPU_COMM magic=%08x/%08x ARM=%08x MIPS=%08x\n",
	       magic1, magic2, arm_flag, mips_flag);

	if (status != H713_MIPS_STATUS_RELEASED || witness ||
	    magic1 != H713_MIPS_SHMEM_MAGIC ||
	    magic2 != H713_MIPS_SHMEM_MAGIC ||
	    arm_flag != H713_MIPS_SHMEM_ARM_READY ||
	    !(mips_flag & H713_MIPS_SHMEM_MIPS_READY)) {
		printf("H713 MIPS: firmware readiness not proven\n");
		ret = -ETIMEDOUT;
	} else {
		printf("H713 MIPS: firmware readiness proven by MIPS READY\n");
		ret = 0;
	}

out_stop:
	h713_mips_stop();
	if (tvcap_released) {
		writel(tvcap_saved_bgr, H713_TVCAP_BGR_REG);
		writel(tvcap_saved_hdmi_audio,
		       H713_TVCAP_HDMI_AUDIO_CLK_REG);
		writel(tvcap_saved_bus, H713_TVCAP_BUS_CLK_REG);
		writel(tvcap_saved_vincap, H713_TVCAP_VINCAP_DMA_CLK_REG);
		writel(tvcap_saved_tcd3, H713_TVCAP_TCD3_CLK_REG);
		printf("H713 MIPS: TVCAP trace state restored\n");
	}
	printf("H713 MIPS: readiness probe returned the core to reset\n");

	return ret;
}


/*
 * Scan the coprocessor's workspace for its own log output.
 *
 * display.bin keeps an "elog" buffer whose location moves between firmware
 * revisions, so rather than hardcode an address that goes stale, walk the
 * region and print runs of printable text. After a run this surfaces whatever
 * the firmware said about its own startup, which beats inferring from the
 * outside.
 */
#define H713_LOG_MIN_RUN	12
#define H713_LOG_MAX_LINES	200

static int h713_mips_log(ulong start, ulong end)
{
	ulong a = start;
	uint printed = 0;

	if (end <= start)
		return -EINVAL;

	while (a < end && printed < H713_LOG_MAX_LINES) {
		ulong run = a;
		uint len = 0;

		while (run + len < end) {
			u8 c = readb(run + len);

			if (c == '\n' || c == '\t' || (c >= 0x20 && c < 0x7f))
				len++;
			else
				break;
		}

		if (len >= H713_LOG_MIN_RUN) {
			uint i;

			printf("0x%08lx: ", run);
			for (i = 0; i < len; i++) {
				u8 c = readb(run + i);

				putc(c == '\n' ? ' ' : c);
			}
			printf("\n");
			printed++;
		}

		a = run + (len ? len : 1);
	}

	printf("H713 MIPS: %u text run(s) in 0x%08lx..0x%08lx%s\n",
	       printed, start, end,
	       printed == H713_LOG_MAX_LINES ? " (truncated)" : "");

	return 0;
}

static void h713_mips_status(void)
{
	u32 witness = h713_mips_read_witness();

	printf("H713 MIPS: clk=0x%08x reset=0x%08x status=0x%08x ",
	       readl(H713_MIPS_CLK_REG), readl(H713_MIPS_RESET_REG),
	       readl(H713_MIPS_STATUS_REG));
	printf("bootaddr=0x%08x\n", readl(H713_MIPS_BOOTADDR_REG));
	printf("H713 MIPS: BSS=0x%08lx..0x%08lx witness@0x%08lx=0x%08x\n",
	       H713_MIPS_BSS_START, H713_MIPS_BSS_END,
	       H713_MIPS_WITNESS_ADDR, witness);
	printf("H713 MIPS: display prerequisites=%s\n",
	       h713_display_prepared ? "applied" : "not applied");
}

static int do_h713_mips(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	/* Default range covers the firmware's data and BSS working set. */
	if (!strcmp(argv[1], "log")) {
		ulong a = argc > 2 ? hextoul(argv[2], NULL) : 0x4b232000;
		ulong b = argc > 3 ? hextoul(argv[3], NULL) : 0x4bd00000;

		return h713_mips_log(a, b) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "status")) {
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "stop")) {
		h713_mips_stop();
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "verify")) {
		ret = h713_mips_verify();
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * Prepare the display clock tree without releasing the MIPS. Several
	 * blocks are only ARM-accessible once this tree is up, so isolating a
	 * register probe from firmware activity needs prep on its own.
	 */
	if (!strcmp(argv[1], "prepare")) {
		h713_display_prepare();
		h713_mips_status();
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "start")) {
		ret = h713_mips_start();
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "release")) {
		ret = h713_mips_release_raw(false, false, false, false, false);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "probe-ready")) {
		ret = h713_mips_probe_ready(false, false, false);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * The ARM leaves TVCAP alone by default; "probe-trace tvcap" opts into
	 * releasing it, which is known to wedge the board.
	 */
	if (!strcmp(argv[1], "probe-trace")) {
		bool tvcap = argc == 3 && !strcmp(argv[2], "tvcap");
		bool no_wait = argc == 3 && !strcmp(argv[2], "no-wait");

		if (argc > 3 || (argc == 3 && !tvcap && !no_wait))
			return CMD_RET_USAGE;

		ret = h713_mips_probe_ready(true, tvcap, no_wait);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "load") || !strcmp(argv[1], "boot")) {
		if (argc != 5)
			return CMD_RET_USAGE;

		ret = h713_mips_load(argv[2], argv[3], argv[4]);
		if (!ret && !strcmp(argv[1], "boot"))
			ret = h713_mips_start();

		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(h713_mips, 5, 0, do_h713_mips,
	   "manually manage the H713 display MIPS firmware",
	   "status\n"
	   "h713_mips log [start] [end]\n"
	   "h713_mips stop\n"
	   "h713_mips verify\n"
	   "h713_mips prepare\n"
	   "h713_mips start\n"
	   "h713_mips release\n"
	   "h713_mips probe-ready\n"
	   "h713_mips probe-trace [tvcap|no-wait]\n"
	   "h713_mips load <interface> <dev[:part]> <path>\n"
	   "h713_mips boot <interface> <dev[:part]> <path>"
);

/*
 * LogoRegData.bin replay.
 *
 * Stock U-Boot drives the panel itself for its boot logo: it parses
 * mips/LogoRegData.bin and applies the register table, then blits a bitmap.
 * The file is a container of 16-byte {address, value, mask, type} records.
 *
 * The record semantics below are read off the vendor applier itself, at
 * 0x4a025164 in this board's stock U-Boot 2018.05 (Thumb-2, load base
 * 0x4a000000). It walks a {?, records, count} descriptor and switches on the
 * type word at +0xc:
 *
 *   type <= 4    masked read-modify-write: *addr = (*addr & ~mask) | value.
 *                Stock uses a 32-bit access for every type in that range --
 *                the width field is vestigial, and this container only ever
 *                uses 4 anyway.
 *   type 0xfe    pulse the masked bits: *addr = cur & ~mask, then
 *                *addr = cur | mask, where cur is one read taken up front.
 *                The value word is not used.
 *   type 0xff    delay, in MICROSECONDS -- it calls the udelay thunk at
 *                0x4a000d74, and the delay core busy-waits val * 24 arch-timer
 *                ticks at 24 MHz. The separate x1000 mdelay wrapper next to it
 *                is not what the walker uses.
 *
 * The container holds several alternative tables -- a CCU/TVTOP prologue, a
 * set of timing blocks, and seven DE/mixer/LVDS blocks for different modes --
 * and which combination this panel needs is not yet established. So this
 * command applies an explicit byte range rather than trying to pick for you;
 * bisecting on the bench is cheaper than guessing statically.
 *
 * h713_display_prepare() is, register for register, the opening of the first
 * table.
 */
/*
 * Stock imposes no ceiling; this only bounds a walk that has fallen into
 * garbage. The largest delay in the container is 15000 us, so 100 ms of
 * headroom cannot truncate a genuine record.
 */
#define H713_LOGO_MAX_DELAY_US	100000

struct h713_logo_rec {
	u32 addr;
	u32 val;
	u32 mask;
	u32 type;
};

/*
 * `setenv h713_trace 1` makes the applying walk name every record on the
 * console *before* it touches the bus. A write into a block whose clock or
 * reset is not up hangs silently, and the console is synchronous, so the last
 * line a stalled log carries is the record that hung -- no rebuild, no
 * debugger, and a remote owner can produce it. Off by default: it is one line
 * per record, which is some 90 lines for a full bring-up.
 *
 * Every wait the walk can enter is bounded: the delay record is capped at
 * H713_LOGO_MAX_DELAY_US, the pulse is two stores with no read-back loop, and
 * the resync step only ever advances `off` towards `end`. There is no
 * poll-until-ready anywhere in here, so a stall is always the bus, never us.
 */
static bool h713_logo_tracing(void)
{
	const char *set = env_get("h713_trace");

	return set && *set && *set != '0';
}

static bool h713_logo_reg_sane(u32 addr)
{
	/* CCU, PIO/MIPS control, and the display/capture blocks only. */
	return (addr >= 0x02000000 && addr < 0x03100000) ||
	       (addr >= 0x04000000 && addr < 0x07000000);
}

static int h713_logo_walk(ulong base, ulong start, ulong end, bool apply)
{
	const bool trace = apply && h713_logo_tracing();
	ulong off;
	int written = 0, skipped = 0, delayed = 0, pulsed = 0;

	/*
	 * Records are 16 bytes but the container only aligns them to 4 -- the
	 * first run starts at 0x17c -- so do not demand 16-byte offsets.
	 */
	if ((start | end) & 3 || end <= start) {
		printf("H713 logo: range must be 4-byte aligned and non-empty\n");
		return -EINVAL;
	}

	/*
	 * Records are 16 bytes but the container inserts other data between
	 * runs, so the stream shifts phase -- the run at 0xbdc is 0x1d8 from
	 * the one at 0xa04, which is not a multiple of 16. Resynchronise by
	 * stepping four bytes until a record parses, then consume sixteen.
	 * Walking a fixed lattice silently drops entries: it applied 167 of
	 * the 292 records in the 0xa04..0x1fdc section.
	 */
	for (off = start; off + sizeof(struct h713_logo_rec) <= end; ) {
		struct h713_logo_rec r;
		ulong reg;

		r.addr = readl(base + off);
		r.val  = readl(base + off + 4);
		r.mask = readl(base + off + 8);
		r.type = readl(base + off + 12);

		/*
		 * Container addresses are 32-bit; widen once here so the MMIO
		 * accessors are not handed a narrower-than-pointer integer.
		 */
		reg = r.addr;

		if (!r.addr && r.type == 0xff) {
			off += sizeof(struct h713_logo_rec);
			u32 us = min_t(u32, r.val, H713_LOGO_MAX_DELAY_US);

			if (trace)
				printf("  T +0x%04lx  delay %u us\n",
				       off - sizeof(struct h713_logo_rec), us);
			if (apply)
				udelay(us);
			else
				printf("  +0x%04lx  delay %u us\n",
				       off - sizeof(struct h713_logo_rec), us);
			delayed++;
			continue;
		}

		if (h713_logo_reg_sane(r.addr) && r.type == 0xfe) {
			u32 cur;

			off += sizeof(struct h713_logo_rec);

			/*
			 * All four in this container pulse bit 31 of the
			 * display PLL at 0x058c0014 -- that is PLL_ENABLE, so
			 * this is a PLL restart, and the one inside timing
			 * block 6 sits between two 15 ms waits in the middle
			 * of a divider reprogram. Dropping it left everything
			 * after it in the block applied to a PLL that had
			 * never been re-locked.
			 */
			if (!apply) {
				printf("  +0x%04lx  0x%08x pulse mask 0x%08x\n",
				       off - sizeof(struct h713_logo_rec),
				       r.addr, r.mask);
				pulsed++;
				continue;
			}

			if (trace)
				printf("  T +0x%04lx  %08x pulse mask %08x\n",
				       off - sizeof(struct h713_logo_rec),
				       r.addr, r.mask);
			cur = readl(reg);
			writel(cur & ~r.mask, reg);
			writel(cur | r.mask, reg);
			pulsed++;
			continue;
		}

		if (!h713_logo_reg_sane(r.addr) ||
		    (r.type != 1 && r.type != 2 && r.type != 4)) {
			off += 4;
			skipped++;
			continue;
		}
		off += sizeof(struct h713_logo_rec);

		if (!apply) {
			printf("  +0x%04lx  0x%08x <- 0x%08x mask 0x%08x w%u\n",
			       off - sizeof(struct h713_logo_rec),
			       r.addr, r.val, r.mask, r.type);
			written++;
			continue;
		}

		if (trace)
			printf("  T +0x%04lx  %08x <- %08x mask %08x w%u\n",
			       off - sizeof(struct h713_logo_rec), r.addr,
			       r.val, r.mask, r.type);

		/* Masked read-modify-write at the record's access width. */
		switch (r.type) {
		case 1:
			writeb((readb(reg) & ~(u8)r.mask) |
			       ((u8)r.val & (u8)r.mask), reg);
			break;
		case 2:
			writew((readw(reg) & ~(u16)r.mask) |
			       ((u16)r.val & (u16)r.mask), reg);
			break;
		default:
			writel((readl(reg) & ~r.mask) | (r.val & r.mask), reg);
			break;
		}
		written++;
	}

	printf("H713 logo: %s %d record(s), %d pulse(s), %d delay(s), "
	       "%d resync step(s)\n", apply ? "applied" : "listed", written,
	       pulsed, delayed, skipped);

	return 0;
}

static int do_h713_logo(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong base, start, end;
	bool apply;

	if (argc != 5)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "apply"))
		apply = true;
	else if (!strcmp(argv[1], "dump"))
		apply = false;
	else
		return CMD_RET_USAGE;

	base  = hextoul(argv[2], NULL);
	start = hextoul(argv[3], NULL);
	end   = hextoul(argv[4], NULL);

	return h713_logo_walk(base, start, end, apply) ?
	       CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_logo, 5, 0, do_h713_logo,
	   "replay a range of the vendor LogoRegData.bin register table",
	   "dump  <blob-addr> <start-off> <end-off>\n"
	   "h713_logo apply <blob-addr> <start-off> <end-off>"
);

/*
 * Bit-banged I2C scan on TWI1's pins.
 *
 * The stock ge2d driver contains an optional DLPC3435 path at 0x1b, so this
 * probe originally tested whether Board B populated that controller. Hardware
 * now proves that it does not answer on the live panel boot; do not infer a
 * DLPC3435 merely from the generic driver's normal_i2c list.
 * TWI1 is the only I2C bus the vendor device tree enables: 0x02502400 at
 * 100 kHz on PH2/PH3. Board B answers at 0x18, matching the enabled STK8BA58
 * accelerometer node and providing the useful bus-positive control. The DT
 * also lists other mutually exclusive accelerometer choices; their absence is
 * not a bus failure.
 *
 * Bit-banging rather than bringing up mvtwsi keeps this self-contained: no
 * device-tree node, no CCU gate, nothing that can be silently wrong.
 */
/*
 * H713 uses CONFIG_SUNXI_NEW_PINCTRL, so banks are 0x30 apart and the pull
 * registers sit at +0x24 -- not the 0x24 stride and +0x1c pull of the older
 * layout. Take the stride from the header rather than restating it.
 */
#define H713_PIO_BASE		0x02000000UL
#define H713_PIO_BANK_B		1
#define H713_PIO_BANK_F		5
#define H713_PIO_BANK_H		7
#define H713_PB_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_B * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PB_DATA		(H713_PB_CFG0 + 0x10)
#define H713_PF_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_F * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PF_DATA		(H713_PF_CFG0 + 0x10)
#define H713_PH_CFG0		(H713_PIO_BASE + \
				 H713_PIO_BANK_H * SUNXI_PINCTRL_BANK_SIZE)
#define H713_PH_DATA		(H713_PH_CFG0 + 0x10)
#define H713_PH_PULL0		(H713_PH_CFG0 + 0x24)

#define H713_I2C_DELAY_US	5		/* ~100 kHz */

/* Selectable so a wrong guess costs a retype, not a rebuild. */
static uint h713_i2c_scl = 2;
static uint h713_i2c_sda = 3;
#define H713_I2C_SCL_PIN	h713_i2c_scl
#define H713_I2C_SDA_PIN	h713_i2c_sda

/* Drive low by becoming an output; release to high-Z and let the pull-up win. */
static void h713_i2c_set(uint pin, bool high)
{
	u32 cfg = readl(H713_PH_CFG0) & ~(0xfu << (pin * 4));

	if (high) {
		writel(cfg, H713_PH_CFG0);		/* input: high-Z */
	} else {
		clrbits_le32((void *)H713_PH_DATA, BIT(pin));
		writel(cfg | (1u << (pin * 4)), H713_PH_CFG0);	/* output low */
	}
	udelay(H713_I2C_DELAY_US);
}

static int h713_i2c_get_sda(void)
{
	return !!(readl(H713_PH_DATA) & BIT(H713_I2C_SDA_PIN));
}

static void h713_i2c_start(void)
{
	h713_i2c_set(H713_I2C_SDA_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, false);
	h713_i2c_set(H713_I2C_SCL_PIN, false);
}

static void h713_i2c_stop(void)
{
	h713_i2c_set(H713_I2C_SDA_PIN, false);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, true);
}

/* Returns true when the slave pulled SDA low for ACK. */
static bool h713_i2c_write_byte(u8 byte)
{
	bool ack;
	int i;

	for (i = 7; i >= 0; i--) {
		h713_i2c_set(H713_I2C_SDA_PIN, !!(byte & BIT(i)));
		h713_i2c_set(H713_I2C_SCL_PIN, true);
		h713_i2c_set(H713_I2C_SCL_PIN, false);
	}

	h713_i2c_set(H713_I2C_SDA_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	ack = !h713_i2c_get_sda();
	h713_i2c_set(H713_I2C_SCL_PIN, false);

	return ack;
}

static u8 h713_i2c_read_byte(bool ack)
{
	u8 v = 0;
	int i;

	h713_i2c_set(H713_I2C_SDA_PIN, true);		/* release for the slave */

	for (i = 7; i >= 0; i--) {
		h713_i2c_set(H713_I2C_SCL_PIN, true);
		if (h713_i2c_get_sda())
			v |= BIT(i);
		h713_i2c_set(H713_I2C_SCL_PIN, false);
	}

	/* ACK to continue, NACK to end the transfer. */
	h713_i2c_set(H713_I2C_SDA_PIN, !ack);
	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SCL_PIN, false);
	h713_i2c_set(H713_I2C_SDA_PIN, true);

	return v;
}

/*
 * Plain read with no preceding register write: identifying an unknown chip
 * must not risk changing its state.
 */
static int h713_i2c_dump(uint addr, uint count)
{
	uint i;

	h713_i2c_start();
	if (!h713_i2c_write_byte((u8)((addr << 1) | 1))) {
		h713_i2c_stop();
		printf("H713 i2c: 0x%02x did not ACK its read address\n", addr);
		return -EIO;
	}

	printf("H713 i2c: 0x%02x read:", addr);
	for (i = 0; i < count; i++)
		printf(" %02x", h713_i2c_read_byte(i + 1 < count));
	printf("\n");

	h713_i2c_stop();
	return 0;
}

static int do_h713_i2c(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	int addr, found = 0;

	/* Enable the internal pull-ups the vendor pinmux asks for. */
	clrsetbits_le32((void *)H713_PH_PULL0,
			(3u << (H713_I2C_SCL_PIN * 2)) |
			(3u << (H713_I2C_SDA_PIN * 2)),
			(1u << (H713_I2C_SCL_PIN * 2)) |
			(1u << (H713_I2C_SDA_PIN * 2)));

	h713_i2c_set(H713_I2C_SCL_PIN, true);
	h713_i2c_set(H713_I2C_SDA_PIN, true);

	if (argc == 4 && !strcmp(argv[1], "read")) {
		uint a = hextoul(argv[2], NULL);
		uint n = dectoul(argv[3], NULL);

		if (a > 0x7f || !n || n > 32) {
			printf("H713 i2c: address 0..0x7f, count 1..32\n");
			return CMD_RET_FAILURE;
		}
		return h713_i2c_dump(a, n) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc != 2 && argc != 4)
		return CMD_RET_USAGE;
	if (strcmp(argv[1], "scan"))
		return CMD_RET_USAGE;

	if (argc == 4) {
		h713_i2c_scl = dectoul(argv[2], NULL);
		h713_i2c_sda = dectoul(argv[3], NULL);
		if (h713_i2c_scl > 31 || h713_i2c_sda > 31 ||
		    h713_i2c_scl == h713_i2c_sda) {
			printf("H713 i2c: bad PH pin numbers\n");
			return CMD_RET_FAILURE;
		}
	}


	/*
	 * With SDA stuck low every ACK read returns zero and the scan reports
	 * a device at every address. Refuse to run rather than print 112 lies.
	 */
	if (!h713_i2c_get_sda()) {
		printf("H713 i2c: SDA (PH%d) reads low with the bus idle -- "
		       "wrong pins, no pull-up, or the line is held\n",
		       H713_I2C_SDA_PIN);
		return CMD_RET_FAILURE;
	}

	printf("H713 i2c: scanning PH%d/PH%d\n",
	       H713_I2C_SCL_PIN, H713_I2C_SDA_PIN);

	for (addr = 0x08; addr < 0x78; addr++) {
		bool ack;

		h713_i2c_start();
		ack = h713_i2c_write_byte((u8)(addr << 1));
		h713_i2c_stop();

		if (ack) {
			printf("  0x%02x ACK%s\n", addr,
			       addr == 0x18 ? "   <-- stk8ba58 (bus works)" :
			       addr == 0x1b ? "   <-- optional DLPC3435" :
			       addr == 0x6a ? "   <-- alternate lsm6dsr" : "");
			found++;
		}
	}

	printf("H713 i2c: %d device(s) responded\n", found);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_i2c, 4, 0, do_h713_i2c,
	   "bit-banged I2C scan on a pair of PH pins",
	   "scan            - scan using PH2/PH3 (vendor twi1 pins)\n"
	   "h713_i2c scan <scl> <sda> - scan using the given PH pin numbers\n"
	   "h713_i2c read <addr> <count> - read bytes, no register write"
);

/*
 * One-shot replay of stock U-Boot's fastlogo display bring-up.
 *
 * LogoRegData.bin is indexed: descriptors of 0x18 bytes from offset 0x10 (how
 * many is in the header -- 15 on the HY310's file, 13 on the HY300 Pro's),
 * each starting with a project ID that matches the ProjectID_*.TSE names. Two
 * of its words select which tables that project uses --
 *
 *   word3 & 0xffff  prologue variant, 1-based (three exist)
 *   word3 >> 16     timing variant,   0-based (eleven exist)
 *   word4 & 0xffff  DE/mixer variant, 0-based (seven exist)
 *
 * -- so a working configuration is a *consistent triple*, not three ranges
 * picked by eye. Doing it by hand produced combinations no project uses.
 *
 * Stock's order is: prologue, timing, LVDS FIFO reset, mixer write, DE table,
 * then clocks/INCAP/LVDS, the coprocessor release, and the LVDS finalise.
 *
 * Where those tables *are* comes from the file, never from constants. Behind
 * the descriptor table the container is a chain of blocks:
 *
 *   +0x00 u32 index   +0x04 u32 payload length   then the payload
 *
 * The index counts up inside a class and restarts at zero when the next class
 * begins: head block, prologue 1..3, timing 0..10, DE 0..7. That is exactly
 * the numbering the descriptor words use -- and it is why the prologue
 * variant is 1-based: index 0 of its class is the file's own head block.
 *
 * This used to be three tables of byte offsets measured on the HY310's
 * 15652-byte file, which has 15 descriptors. The HY300 Pro's firmware
 * generation carries 13, so every block sits 0x30 lower, and its timing and
 * DE tables are sized differently again. On that file the fixed DE range for
 * project 0x34 (0x361c..0x39a4) began in the middle of DE variant 7 and ran
 * 368 bytes past the end of the file, into whatever DRAM held before the blob
 * was loaded -- an arbitrary write into the 0x02000000/0x04000000 windows,
 * which is a silent bus hang. Deriving the boundaries reproduces all 22 of
 * the old HY310 constants exactly; see umbau/work/E4.
 */
struct h713_disp_block { u32 start, end; };

#define H713_DISP_DESC_OFF	0x10
#define H713_DISP_DESC_SIZE	0x18
#define H713_DISP_HDR_TABLE_LEN	8	/* u16: descriptor table bytes */
#define H713_DISP_HDR_HEAD_LEN	0x0a	/* u16: head block payload bytes */
#define H713_DISP_HDR_REST	0x0c	/* u32: block bytes after that */
#define H713_DISP_BLOCK_HDR	8

struct h713_disp_sel {
	u32 project, prologue, timing, de;
	struct h713_disp_block prologue_range, timing_range, de_range;
};

/*
 * The container's length, from the four words of its own header:
 *   0x10 + 0x0168 + 0x24 + 0x3b88 = 15652, the HY310's file
 *   0x10 + 0x0138 + 0x24 + 0x36c8 = 14388, the HY300 Pro's
 * Zero if this is not a LogoRegData container or the length is not credible.
 * Every offset the replay uses is bounded by this, so a file we do not
 * understand yields no groups at all rather than a walk over open DRAM --
 * which is the fault this whole change is about.
 */
#define H713_DISP_HDR_MAGIC	0x6f676f6cU	/* "logo" */
#define H713_DISP_MAX_SIZE	0x100000UL	/* both known files are ~15 KiB */

static ulong h713_disp_size(ulong blob)
{
	ulong size = H713_DISP_DESC_OFF +
		     readw(blob + H713_DISP_HDR_TABLE_LEN) +
		     readw(blob + H713_DISP_HDR_HEAD_LEN) +
		     readl(blob + H713_DISP_HDR_REST);

	if (readl(blob) != H713_DISP_HDR_MAGIC || size > H713_DISP_MAX_SIZE)
		return 0;

	return size;
}

/*
 * Resolve one group: walk the block chain, count a class boundary wherever
 * the block index stops rising, and hand back the payload of the block whose
 * index is the wanted variant. `end` is the payload end; the old constants
 * carried the next block's 8-byte header as well, which the walker never
 * reads (it needs off + 16 <= end and every payload is a multiple of 16).
 */
static int h713_disp_group(ulong blob, uint want_class, uint index,
			   struct h713_disp_block *out)
{
	ulong size = h713_disp_size(blob);
	ulong off = H713_DISP_DESC_OFF + readw(blob + H713_DISP_HDR_TABLE_LEN);
	bool first = true;
	uint cls = 0;
	u32 prev = 0;

	while (off + H713_DISP_BLOCK_HDR <= size) {
		u32 idx = readl(blob + off);
		u32 len = readl(blob + off + 4);

		if (!len || off + H713_DISP_BLOCK_HDR + len > size)
			break;
		if (!first && idx <= prev)
			cls++;
		first = false;
		prev = idx;

		if (cls == want_class && idx == index) {
			out->start = off + H713_DISP_BLOCK_HDR;
			out->end = out->start + len;
			return 0;
		}
		off += H713_DISP_BLOCK_HDR + len;
	}

	return -ENOENT;
}

/*
 * Descriptor count is in the header, not fixed: this board's file carries 15
 * where an earlier revision had 13. Reading it keeps the command correct
 * across firmware versions.
 */
static uint h713_disp_desc_count(ulong blob)
{
	uint bytes = readw(blob + H713_DISP_HDR_TABLE_LEN);

	return bytes / H713_DISP_DESC_SIZE;
}

/* Set once a sequence has run, so the dump knows the blocks are clocked. */
static bool h713_disp_configured;

static int h713_disp_lookup(ulong blob, u32 project,
			    struct h713_disp_sel *sel)
{
	const struct h713_panel_cfg *panel;
	int i;

	for (i = 0; i < h713_disp_desc_count(blob); i++) {
		ulong d = blob + H713_DISP_DESC_OFF + i * H713_DISP_DESC_SIZE;
		u32 id = readl(d);
		u32 w3, w4;

		if (id != project)
			continue;

		w3 = readl(d + 8);
		w4 = readl(d + 12);
		sel->project  = id;
		sel->prologue = w3 & 0xffff;
		sel->timing   = w3 >> 16;
		sel->de       = w4 & 0xffff;

		if (!sel->prologue ||
		    h713_disp_group(blob, 0, sel->prologue,
				    &sel->prologue_range) ||
		    h713_disp_group(blob, 1, sel->timing, &sel->timing_range) ||
		    h713_disp_group(blob, 2, sel->de, &sel->de_range)) {
			printf("H713 disp: project 0x%02x selects tables this "
			       "file does not carry (%u/%u/%u)\n", id,
			       sel->prologue, sel->timing, sel->de);
			return -EINVAL;
		}

		/*
		 * Say where they landed. A boot log that stops inside the
		 * replay then names the group it was walking, without a
		 * rebuild and without the owner's copy of the file.
		 */
		printf("H713 disp: groups from the file (%lu bytes): prologue "
		       "0x%04x..0x%04x, timing 0x%04x..0x%04x, de "
		       "0x%04x..0x%04x\n", h713_disp_size(blob),
		       sel->prologue_range.start, sel->prologue_range.end,
		       sel->timing_range.start, sel->timing_range.end,
		       sel->de_range.start, sel->de_range.end);

		/*
		 * The declared project ID is what names the panel, and it is
		 * the ID this run was given -- from the environment, from
		 * panel_config.ini, or from the operator. Not the display.bin
		 * digest: one image serves both panels, so choosing by digest
		 * would drive an HY350 with the HY310's raster the moment
		 * their shared firmware was identified. Everything downstream
		 * reads the panel from here: the register patch table, the OSD
		 * geometry, the logo.
		 */
		panel = h713_panel_by_project(id);
		if (panel) {
			h713_disp_panel = panel;
			printf("H713 disp: project 0x%02x, panel %ux%u %s-port\n",
			       id, panel->width, panel->height,
			       panel->dual_port ? "dual" : "single");
		} else if (h713_probe_mode) {
			/*
			 * Nothing downstream may run on a guessed panel: the
			 * register patches, the OSD geometry and the logo are
			 * all sized from it.
			 */
			printf("H713 disp: no panel known for this image -- "
			       "probe reports, it does not drive a panel\n");
			return -ENOENT;
		} else {
			printf("H713 disp: project 0x%02x has no panel of its "
			       "own -- keeping %ux%u\n", id,
			       h713_disp_panel->width, h713_disp_panel->height);
		}
		return 0;
	}

	printf("H713 disp: no descriptor for project 0x%02x\n", project);
	return -ENOENT;
}

static void h713_disp_list(ulong blob)
{
	int i;

	printf("H713 disp: project  prologue  timing  de\n");
	for (i = 0; i < h713_disp_desc_count(blob); i++) {
		ulong d = blob + H713_DISP_DESC_OFF + i * H713_DISP_DESC_SIZE;
		u32 w3 = readl(d + 8);

		printf("             0x%02x       %u       %2u   %u\n",
		       readl(d), w3 & 0xffff, w3 >> 16, readl(d + 12) & 0xffff);
	}
}

/* Clocks, capture block and INCAP, as stock issues them before LVDS. */
static void h713_disp_clocks(void)
{
	writel(0x22ffff22, 0x02000150);		/* PH mux; PH0/1 stay UART0 */
	mdelay(12);
	writel(0x00010001, 0x02001d88);
	mdelay(12);
	/*
	 * 0x02001020 is deliberately absent. Stock writes PLL_PERIPH0 there,
	 * but it is a no-op in stock's boot context and a live reconfiguration
	 * in ours -- MMC and the buses run from it. The other two PLLs are
	 * disabled in our cold state, so enabling them is safe.
	 */
	writel(0xb8003501, 0x02001040);
	writel(0x80000305, 0x02001d6c);
	mdelay(12);
	writel(0xb8002f01, 0x02001068);
	writel(0x81000001, 0x02001d74);
	mdelay(12);
	writel(0x80000000, 0x02001d84);
	mdelay(12);
	writel(0xc0000000, 0x02001d80);
	mdelay(12);

	writel(0x01111117, 0x06e00004);
	writel(0x00000404, 0x06e00008);
	writel(0x00111111, 0x06e00000);
	mdelay(12);
	writel(0, 0x06e00004);
	writel(0, 0x06e00008);
	writel(0, 0x06e00000);
	mdelay(12);
	writel(0x01111117, 0x06e00004);
	writel(0x00000404, 0x06e00008);
	writel(0x00111111, 0x06e00000);
	mdelay(12);

	/*
	 * No module-clock writes here on purpose. h713_display_prepare() sets
	 * PLL_VIDEO2 and the deint/panel/SVP-DTL/AFBD gates for the
	 * h713_mips start path, but the vendor prologue block already does the
	 * same work on this one -- prologue 3 writes 0x02001dc0 <- 0x80000005
	 * and enables PLL_VIDEO2 across four masked records. Calling the
	 * helper here would be redundant, not protective; the clock state is
	 * reported in the post-readiness dump instead, which is the only place
	 * it could still have changed.
	 */
}

/* Stock's reset: pulse bit 8 of the FIFO control, then re-latch the config. */
static void h713_disp_fifo_reset(void)
{
	u32 status = readl(0x05880fe0);
	u32 cfg;
	u32 ctl;

	if (!status) {
		printf("H713 disp: FIFO status clear; reset skipped\n");
		return;
	}

	cfg = readl(0x0588000c);
	ctl = readl(0x05700088);
	writel(ctl & ~0x100, 0x05700088);
	writel(ctl | 0x100, 0x05700088);
	writel(cfg, 0x0588000c);
	printf("H713 disp: FIFO status 0x%08x; reset applied\n", status);
}

/*
 * Stock fastlogo applies all LogoRegData groups, waits poweron_delay1
 * (550 ms), then invokes its panel-power method. The board-B runtime TOC1 DT
 * identifies panel_power_en as PF6, not the PH19 found in the older board-A
 * dump. It identifies panel_gpio_0 as PH16. Enable PF6 and pulse PH16 low for
 * 2 ms, then high for 5 ms. The state
 * machine waits poweron_delay0 (20 ms) before continuing into the display
 * clocks and MIPS release.
 *
 * Both GPIOs carry GPIO_PULL_DOWN in the stock DT. Set the data latch before
 * selecting output mode so the power line cannot glitch low as it is enabled.
 *
 * Do not use the legacy gpio_set_value() compatibility wrapper here. This
 * build uses DM_GPIO, whose compatibility wrapper reconstructs a descriptor
 * without its previous output-direction flags; the second value change then
 * does not reach this driver's set_flags() output path. Accessing the PF/PH
 * data latches directly also lets the diagnostic verify the exact registers.
 */
static int h713_disp_stock_panel_power(void)
{
	const uint power = SUNXI_GPF(6);
	const uint reset = SUNXI_GPH(16);
	u32 pf_dat;
	u32 ph_dat;

	printf("H713 panel: stock power pre-delay 550 ms\n");
	mdelay(550);

	sunxi_gpio_set_pull(power, SUNXI_GPIO_PULL_DOWN);
	sunxi_gpio_set_pull(reset, SUNXI_GPIO_PULL_DOWN);

	setbits_le32((void *)H713_PF_DATA, BIT(6));
	sunxi_gpio_set_cfgpin(power, SUNXI_GPIO_OUTPUT);

	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	sunxi_gpio_set_cfgpin(reset, SUNXI_GPIO_OUTPUT);
	mdelay(2);
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(5);

	/*
	 * The other four panel lines the vendor bootloader drives.
	 *
	 * Its device tree, tvtop@1, names six and gives every one of them
	 * <phandle bank pin mux pull drive data> with mux 1 (output) and
	 * data 1:
	 *
	 *	panel_power_en  PH19    panel_gpio_0    PH16
	 *	panel_bl_en     PB5     panel_gpio_1    PH15
	 *	                        panel_gpio_2    PH8
	 *	                        panel_gpio_3    PH9
	 *
	 * Three of those are already driven -- PB5, PH16, and PF6, which
	 * appears in neither the bootloader nor the kernel device tree and so
	 * belongs to board B alone. On an HY310 the remaining four were left
	 * unconfigured: `gpio status PH19` read back "func" with no direction
	 * at all, and the image came up minutes late and then flickered away.
	 * Panel power is not something to leave to a pin's reset default.
	 */
	if (h713_disp_panel == &h713_panel_cfg_hy310) {
		static const u8 pins[] = { 19, 15, 8, 9 };
		uint i;

		for (i = 0; i < ARRAY_SIZE(pins); i++) {
			setbits_le32((void *)H713_PH_DATA, BIT(pins[i]));
			sunxi_gpio_set_cfgpin(SUNXI_GPH(pins[i]),
					      SUNXI_GPIO_OUTPUT);
		}
		mdelay(5);
		printf("H713 panel: vendor panel lines PH19/PH15/PH8/PH9 "
		       "driven high (PH_DAT=%08x)\n", readl(H713_PH_DATA));
	}

	pf_dat = readl(H713_PF_DATA);
	ph_dat = readl(H713_PH_DATA);
	printf("H713 panel: stock GPIO phase complete: "
	       "PF6 cfg=%x latch=%d, PH16 cfg=%x latch=%d, "
	       "PF_DAT=%08x PH_DAT=%08x\n",
	       sunxi_gpio_get_cfgpin(power), !!(pf_dat & BIT(6)),
	       sunxi_gpio_get_cfgpin(reset), !!(ph_dat & BIT(16)),
	       pf_dat, ph_dat);
	mdelay(20);

	return 0;
}

/*
 * The panel power-down sequence lives further down, next to the AFBD and LVDS
 * register definitions it needs: see h713_disp_teardown().
 */


/*
 * A note about project 0x33, kept because the results that rest on it are
 * still quoted. It is about the bench board, board B, and about nothing else:
 * this used to introduce a compiled-in H713_DISP_BOARD_PROJECT_ID, which was
 * that board's 0x34 and wrong on every other one. The constant went in
 * 218792f; the sentence that called it "this board's" outlived it. Which
 * project a board declares is now read from the board -- h713_project, or its
 * panel_config.ini -- and the panel follows from it.
 *
 * Board B declares 0x34, in the panel_config.ini the struct above transcribes
 * ("ProjectID = 52") and in stock's own log -- "Project id:0x34
 * version:25-1-6-3", then mips/ProjectID_0x0034.TSE. Its bring-up here ran
 * 0x33 throughout, on no evidence, so 0x33 was a guess.
 *
 * It is a guess that cost nothing, which was checked rather than assumed: an
 * A/B on 2026-08-06 found 0x33 and 0x34 select the same prologue and timing
 * blocks, differing only in the DE block, and there only in 0x0525c038 once our
 * patch table has run. The panel is indistinguishable. So every 0x33 result
 * stands; see docs/mips-display-recovery.md.
 *
 * Prefer 0x34 for new work on that board anyway. The equivalence covers the
 * ARM's LogoRegData replay, and the ProjectID_*.TSE payloads -- which differ by
 * 2688 bytes and feed the MIPS -- are not exercised by any test we have.
 */

struct h713_panel_patch {
	u32 reg;
	u8  shift;
	u32 fieldmask;
	u32 value;
};

/*
 * The vendor tables carry another panel's defaults; stock rewrites these
 * fields from the merged config before its applier touches hardware. Sites
 * and bit positions are transcribed from stock U-Boot's patch function at
 * 0x4a0248fc, whose helper at 0x4a024894 is a plain bitfield insert:
 *
 *     record.value = (record.value & ~(fieldmask << shift))
 *                  | ((value & fieldmask) << shift)
 *
 * guarded by (fieldmask << shift) being a subset of the record's own mask.
 *
 * Two sites were deliberately omitted, on the grounds that stock's literal
 * zero at 0x05280084[31:16] and 0x0528008c[15:0] might be a decode artefact
 * rather than a real store, and that an unjustifiable zero was worse than the
 * vendor default.
 *
 * 0x0528008c[15:0] is now restored: that omission was the whole framebuffer
 * fault. The vendor default is another panel's and it is 123, which is exactly
 * the pale left band and, through the source advance, the shear. The static
 * analysis was right and the caution was wrong; what it lacked was hardware,
 * and test_32 and test_33 supply it.
 *
 * 0x05280084[31:16] is still omitted and is now *more* suspicious, not less.
 * It currently holds 720 and stock appears to zero it. Do not change it on
 * this reasoning alone -- it earned a two-sided perturbation of its own, the
 * way 0x0528008c did.
 */
static int h713_disp_panel_patch(ulong blob, const struct h713_disp_sel *sel)
{
	const struct h713_panel_cfg *c = h713_disp_panel;
	const struct h713_panel_patch tbl[] = {
		/* LVDS lane/map: protocol, bit width, swap and inversions */
		/* Display PLL N: the panel decodes only near 864 MHz. */
		{ 0x058c0014,  8, 0xff,   c->pll_n_plus_1 - 1 },
		/*
		 * The spread-spectrum waveform, if this panel declares one.
		 * Board B's mask is zero, so the entry is skipped there and
		 * its record keeps whatever the vendor tables hold -- which
		 * has never been read on that board and must not be guessed.
		 */
		{ 0x058c0018,  0, c->ssc_mask, c->ssc_reg },
		{ 0x05800000,  6, 0x3,    c->mapping },
		{ 0x05800000,  3, 0x3,    c->lvds_bitsel },
		{ 0x05800000, 14, 0x1,    c->odd_even },
		{ 0x05800000, 16, 0x1,    c->inv_hsync },
		{ 0x05800000, 17, 0x1,    c->inv_vsync },
		{ 0x05800000, 18, 0x1,    c->inv_de },
		{ 0x05800000, 24, 0x1,    c->inv_dclk },
		/* display PLL: LVDS drive currents and spread spectrum */
		{ 0x058c0020, 24, 0x3f,   c->de_current },
		{ 0x058c0020,  0, 0x7,    c->odd_current },
		{ 0x058c0024, 24, 0x3f,   c->de_current },
		{ 0x058c0024,  0, 0x7,    c->even_current },
		{ 0x058c0014, 24, 0x1,    c->ssc_en },
		/* TCON control: single/dual port */
		{ 0x0588000c, 12, 0x3,    c->dual_port },
		/* AFBD fetch: mirror mode */
		{ 0x05600140,  2, 0x3,    c->mirror_mode },
		/*
		 * Mixer geometry and blanking. 0x0525c000 holds total MINUS
		 * ONE -- the conversion lives here, in the one place that
		 * knows the register, and not in the panel rows, which state
		 * the ini's raw totals (see struct h713_panel_cfg). The HY310
		 * keeps the 2127/1119 it has always written; board B stops
		 * driving a 1361x761 mixer raster into a 1360x760 TCON.
		 */
		{ 0x0525c000, 16, 0xffff, c->vtotal - 1 },
		{ 0x0525c000,  0, 0xffff, c->htotal - 1 },
		{ 0x0525c004,  8, 0xff,   c->hsync },
		{ 0x0525c004,  0, 0xff,   c->vsync },
		{ 0x0525c01c,  0, 0xffff, c->hsync + c->hbp },
		{ 0x0525c020,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0525c030,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0525c034, 16, 0xffff, c->width },
		{ 0x0525c034,  0, 0xffff, c->hsync + c->hbp },
		/*
		 * Stock's compare at +0x24e12/+0x24e26 matches *either*
		 * 0x0524c010 or 0x0525c000 and patches whichever record it
		 * found, so the DE's copy of the geometry takes the same
		 * value as the mixer's. Omitting it left the DE composing
		 * 1440x741 beneath a mixer at 1360x760 -- visible in a live
		 * dump as de +0x10 holding the unpatched 0x02e4059f.
		 *
		 * Same minus-one convention as 0x0525c000 above.
		 */
		{ 0x0524c010, 16, 0xffff, c->vtotal - 1 },
		{ 0x0524c010,  0, 0xffff, c->htotal - 1 },
		/* display engine */
		{ 0x0524c004, 16, 0xffff, c->width },
		{ 0x0524c004,  0, 0xffff, c->vsync + c->vbp },
		{ 0x0524c014,  8, 0xff,   c->vsync },
		{ 0x05280084,  0, 0xffff, c->width },
		/*
		 * The upper half, which the note above leaves alone because
		 * stock appears to zero it there. On an HY310 a live stock
		 * bootloader reads 04380780, i.e. the active height. Those two
		 * observations are about different boards and need not agree.
		 *
		 * The same note asks not to change this on reasoning alone, so
		 * board B's mask is zero and its record is untouched until a
		 * perturbation on that hardware says otherwise. Only the panel
		 * that was measured writes it.
		 */
		{ 0x05280084, 16, c->layer_h_mask, c->height },
		{ 0x05280088,  0, 0xffff, c->vsync + c->vbp },
		/*
		 * The layer's pixel X origin, restored 2026-08-04.
		 *
		 * The zero above came from board B's own sweep -- test_32
		 * showed it puts content at column 0 there -- so it is that
		 * panel's answer and it stays. A live stock bootloader on an
		 * HY310 reads 0x37 = 55 for its panel. Two boards, two values;
		 * the register is per-panel and not a constant either way.
		 * The vendor default is another panel's, and it is 123 -- which
		 * is precisely the pale band that sat at the left of every
		 * framebuffer photograph in this bring-up, and, through the
		 * source advance, the shear as well.
		 *
		 * The static analysis was right; the caution was wrong. What it
		 * lacked was hardware: test_32 drove this register 1:1 (0 puts
		 * content at column 0 across the full 1280, 400 puts it at 406,
		 * four other candidates moved it by a pixel) and test_33 showed
		 * the stride reads S = V once it is zero.
		 *
		 * Patching the record rather than writing the register after
		 * the fact also survives h713_disp_reassert_osd, which replays
		 * DE block 5 from this same blob.
		 */
		{ 0x0528008c,  0, 0xffff, c->layer_x },
	};
	const struct h713_disp_block *ranges[] = {
		&sel->prologue_range, &sel->timing_range, &sel->de_range,
	};
	int patched = 0, guarded = 0;
	uint i, r;

	for (r = 0; r < ARRAY_SIZE(ranges); r++) {
		ulong off;

		for (off = ranges[r]->start;
		     off + sizeof(struct h713_logo_rec) <= ranges[r]->end; ) {
			struct h713_logo_rec rec;

			rec.addr = readl(blob + off);
			rec.val  = readl(blob + off + 4);
			rec.mask = readl(blob + off + 8);
			rec.type = readl(blob + off + 12);

			/*
			 * Record validity and the 4-byte resync step must
			 * mirror h713_logo_walk() exactly. If this pass
			 * framed the stream differently it would rewrite
			 * bytes the walker then reads as a different record.
			 */
			bool is_delay = !rec.addr && rec.type == 0xff;
			bool is_pulse = h713_logo_reg_sane(rec.addr) &&
					rec.type == 0xfe;
			bool is_write = h713_logo_reg_sane(rec.addr) &&
					(rec.type == 1 || rec.type == 2 ||
					 rec.type == 4);

			if (!is_delay && !is_pulse && !is_write) {
				off += 4;
				continue;
			}

			for (i = 0; is_write && i < ARRAY_SIZE(tbl); i++) {
				u32 window, updated;

				if (tbl[i].reg != rec.addr)
					continue;
				/* No field: this panel does not patch it. */
				if (!tbl[i].fieldmask)
					continue;

				window = tbl[i].fieldmask << tbl[i].shift;
				if (window & ~rec.mask) {
					guarded++;
					continue;
				}

				updated = (rec.val & ~window) |
					  ((tbl[i].value & tbl[i].fieldmask)
					   << tbl[i].shift);
				if (updated == rec.val)
					continue;

				printf("  +0x%04lx  %08x  %08x -> %08x  "
				       "shift %u mask 0x%x\n", off, rec.addr,
				       rec.val, updated, tbl[i].shift,
				       tbl[i].fieldmask);
				writel(updated, blob + off + 4);
				rec.val = updated;
				patched++;
			}

			off += sizeof(struct h713_logo_rec);
		}
	}

	printf("H713 panel: config applied, %d record field(s) patched, "
	       "%d guarded by record mask\n", patched, guarded);
	return 0;
}

/*
 * Sample the three words the DE replay is known to clear, at each ownership
 * boundary of a single boot.
 *
 * Two consecutive boots showed `0x051c0014` 18000005->18000000,
 * `0x051c0028` 1f300030->00000030 and `0x05140054` 40000080->40000000 across
 * the replay. Whether the firmware or the ARM's own record blocks set those
 * bits in the first place was never established: static analysis of the
 * coprocessor image resolves 604 of its 845 register-helper call sites and
 * none of them touch these three, but the unresolved remainder computes its
 * addresses from runtime tables, so that is not proof either way.
 *
 * This probe answers it directly and needs no disassembly. If the bits are
 * already set before the MIPS is released, the ARM's record blocks own them
 * and the firmware is irrelevant to the clobber. If they appear only after
 * readiness, the firmware owns them.
 */
/*
 * Measure the real raster rate, and from it the real DCLK.
 *
 * The clock tree computes as follows. PLL_VIDEO2 at 0x02001050 reads
 * 0xb9002a00: N+1 = 43, M+1 = 1, and the H616-compatible fixed post-divider of
 * 4, giving pll-video2-4x = 1032 MHz. PANEL_CLK at 0x02001db4 reads 0x80000001:
 * gated on, mux 0 (pll-video2-4x its only parent), M+1 = 2. So the panel clock
 * is 516 MHz. Against a 7:1 LVDS serialiser that is 73.71 MHz of DCLK, where
 * panel_config.ini asks for 62 MHz -- 18.9% high, and a 71.3 Hz refresh instead
 * of 60.
 *
 * Worse, 62 MHz is not reachable through this tree at all: 1032/16 = 64.50 and
 * 1032/17 = 60.71 bracket it, and no integer divider hits it. Either the
 * requested value is not what the hardware targets, the serialisation ratio is
 * not 7:1, or a divider exists that has not been located -- 0x058c0020 and
 * 0x058c0024 are the candidates, since the panel config patches write both.
 *
 * Rather than pick between those, measure it. 0x05880000 carries two 16-bit
 * raster counters; sampling it in a tight loop and counting the decreases in
 * each half gives the line and frame rates directly, and DCLK = line_rate * HT.
 * That also settles the register's semantics, which have never been pinned:
 * observed samples put values above 760 in both halves, so the obvious
 * "x in one, y in the other" reading of a 1360x760 raster cannot be right.
 */
#define H713_DISP_SCAN_REG_RAW	0x05880000UL	/* == H713_DISP_LVDS_SCAN_REG */

/*
 * 0x05880000 holds one free-running 10-bit counter presented twice with a fixed
 * offset -- three consecutive samples gave high-low = -642 every time, and a
 * register sweep found nothing else in the block moving at all. It is not a
 * raster position. It is, however, a usable clock witness: its wrap rate
 * measured 18000 Hz, so the counter clock is 18000 * 1024 = 18.432 MHz, and
 * 516/28 = 18.4286 MHz. That is 0.02% away, which independently confirms the
 * 516 MHz panel clock computed from PLL_VIDEO2 and PANEL_CLK.
 */
#define H713_PLL_VIDEO2_REG	0x02001050UL
#define H713_PLL_VIDEO2_LOCK	BIT(28)
#define H713_PANEL_CLK_DIV	2	/* PANEL_CLK M+1, from 0x02001db4 */
#define H713_LVDS_SER_RATIO	7	/* assumed 7:1 serialisation */
#define H713_COUNTER_DIV	28	/* panel_clk / counter clock, measured */

static ulong h713_disp_counter_khz(uint window_us)
{
	u32 prev, cur;
	ulong t0, elapsed;
	uint wraps = 0;

	prev = readl(H713_DISP_SCAN_REG_RAW) >> 16;
	t0 = timer_get_us();
	do {
		cur = readl(H713_DISP_SCAN_REG_RAW) >> 16;
		if (cur < prev)
			wraps++;
		prev = cur;
		elapsed = timer_get_us() - t0;
	} while (elapsed < window_us);

	if (!elapsed)
		return 0;
	/* Ten-bit counter, so each wrap is 1024 counter clocks. */
	return (ulong)wraps * 1024 * 1000 / elapsed;
}

/*
 * Retune PLL_VIDEO2 so the panel actually receives the DCLK it asks for.
 *
 * The vendor table leaves N+1 = 43, giving pll-video2-4x = 1032 MHz, a 516 MHz
 * panel clock and -- at 7:1 -- 73.71 MHz of DCLK where panel_config.ini asks for
 * 62. That is 18.9% high and a 71.3 Hz refresh instead of 60.
 *
 * Exact 62 MHz is not synthesisable here: DCLK = 24*(N+1)/((M+1)*7) and no
 * integer solution exists within the PLL's rate limits. N+1 = 36 lands at
 * 61.71 MHz, 0.46% low, which is well inside any panel's tolerance.
 *
 * The change verifies itself. If the counter really is panel_clk/28 its rate
 * must fall from 18432 kHz to 864/(2*28) = 15428 kHz. If it does not move
 * proportionally then the clock did not change the way this reasoning assumes,
 * and no visual result from the run means anything.
 */
static int h713_disp_pll_video2_set_n(uint n_plus_1)
{
	u32 saved = readl(H713_PLL_VIDEO2_REG);
	u32 val = (saved & ~(0xFFU << 8)) | (((n_plus_1 - 1) & 0xFF) << 8);
	int i;

	writel(val, H713_PLL_VIDEO2_REG);
	dmb();
	for (i = 0; i < 2000; i++) {
		if (readl(H713_PLL_VIDEO2_REG) & H713_PLL_VIDEO2_LOCK)
			break;
		udelay(100);
	}
	printf("H713 pll: PLL_VIDEO2 %08x -> %08x (N+1=%u), lock %s after %d us\n",
	       saved, readl(H713_PLL_VIDEO2_REG), n_plus_1,
	       (readl(H713_PLL_VIDEO2_REG) & H713_PLL_VIDEO2_LOCK) ?
	       "acquired" : "NOT acquired", i * 100);

	return (readl(H713_PLL_VIDEO2_REG) & H713_PLL_VIDEO2_LOCK) ? 0 : -EIO;
}

/*
 * Identify which register actually controls the pixel clock, by perturbing one
 * candidate field at a time and watching the free-running counter.
 *
 * Retuning the CCU's PLL_VIDEO2 proved that it does not: the N field write read
 * back correctly as 0xb9002300 and the counter did not move by a single kHz.
 * The display block carries its own PLL -- 0x058c0014 reads 0xb8002a00, the
 * same 0x2a in bits 15:8 and near-identical control bits to PLL_VIDEO2's
 * 0xb9002a00 -- and the panel config patches write both 0x058c0014 and
 * 0x058c0024 with panel-derived values. That is the likely clock path.
 *
 * Rather than guess field meanings one build at a time, perturb each candidate,
 * measure, and restore. Whichever moves the counter is in the chain. Each field
 * is restored before the next is tried and the counter is re-measured
 * afterwards, so a candidate that fails to restore is reported rather than
 * silently poisoning the rest of the sweep.
 *
 * Run with the MIPS quiesced and the raster still clocked. Power-cycle after,
 * as with every other mode here.
 */
struct h713_clk_candidate {
	ulong reg;
	u8 shift;
	u8 width;
	u32 alt;
	const char *what;
};

/*
 * SAFETY, learned the hard way: the first version of this swept all candidates
 * in one go with large perturbations -- N 43 -> 36, dividers halved -- and hung
 * the board. Stopping or wildly moving a clock that feeds the display fabric can
 * wedge the bus on the next MMIO read, and nothing after that point runs.
 *
 * So: one candidate per invocation, chosen by index; the smallest useful
 * perturbation rather than a large one; and the candidate is printed before the
 * write, so if the board does hang, the serial log names exactly which field did
 * it. Nothing here persists across a power cycle, so a hang costs a reboot.
 */
static const struct h713_clk_candidate h713_clk_candidates[] = {
	{ 0x058c0014,  8, 8, 0x29, "disp-PLL N   0x058c0014[15:8]  0x2a->0x29" },
	{ 0x058c0024,  0, 3, 0x06, "divider?     0x058c0024[2:0]   7->6" },
	{ 0x058c0024, 24, 6, 0x2e, "divider?     0x058c0024[29:24] 0x2f->0x2e" },
	{ 0x058c0020,  0, 8, 0x7e, "unknown      0x058c0020[7:0]   0x7f->0x7e" },
	{ 0x058c0018,  0, 8, 0x1d, "unknown      0x058c0018[7:0]   0x1e->0x1d" },
	{ 0x058c002c,  0, 8, 0x11, "unknown      0x058c002c[7:0]   0x10->0x11" },
	{ 0x058c0028,  0, 8, 0x01, "unknown      0x058c0028[7:0]   0x00->0x01" },
};

static int h713_disp_clk_find(int which)
{
	ulong base_khz;
	const struct h713_clk_candidate *c;
	u32 mask, saved, val;
	ulong khz, back;
	long delta;
	uint i;

	if (which < 0) {
		printf("H713 clkfind: candidates (run one at a time with an index)\n");
		for (i = 0; i < ARRAY_SIZE(h713_clk_candidates); i++)
			printf("  %u  %-42s currently %08x\n", i,
			       h713_clk_candidates[i].what,
			       readl(h713_clk_candidates[i].reg));
		printf("H713 clkfind: each write is minimal and restored; a hang "
		       "costs a power cycle, not the board\n");
		return 0;
	}

	if (which >= (int)ARRAY_SIZE(h713_clk_candidates)) {
		printf("H713 clkfind: index %d out of range (0..%u)\n", which,
		       (uint)ARRAY_SIZE(h713_clk_candidates) - 1);
		return -EINVAL;
	}

	base_khz = h713_disp_counter_khz(100000);
	printf("H713 clkfind: baseline counter %lu kHz\n", base_khz);
	if (!base_khz) {
		printf("H713 clkfind: counter is not running; nothing to measure\n");
		return -EIO;
	}

	c = &h713_clk_candidates[which];
	mask = (u32)(((1ULL << c->width) - 1) << c->shift);
	saved = readl(c->reg);
	val = (saved & ~mask) | ((c->alt << c->shift) & mask);

	if (val == saved) {
		printf("H713 clkfind: %s already holds the alternate value\n",
		       c->what);
		return 0;
	}

	/* Announce before writing: if this hangs, the log names the culprit. */
	printf("H713 clkfind: perturbing %s, %08x -> %08x ...\n",
	       c->what, saved, val);

	writel(val, c->reg);
	dmb();
	mdelay(50);
	khz = h713_disp_counter_khz(100000);

	writel(saved, c->reg);
	dmb();
	mdelay(50);
	back = h713_disp_counter_khz(100000);

	delta = ((long)khz - (long)base_khz) * 1000 / (long)base_khz;
	printf("H713 clkfind: counter %lu -> %lu kHz (%+ld.%ld%%), restored to %lu\n",
	       base_khz, khz, delta / 10, (delta < 0 ? -delta : delta) % 10, back);

	if (!khz)
		printf("H713 clkfind: the perturbation STOPPED the counter -- "
		       "this field gates or sources the clock\n");
	else if (delta > 10 || delta < -10)
		printf("H713 clkfind: IN THE CLOCK PATH\n");
	else
		printf("H713 clkfind: no effect on the pixel clock\n");

	if (back > base_khz + base_khz / 50 || back < base_khz - base_khz / 50)
		printf("H713 clkfind: WARNING: counter did not return to "
		       "baseline; power-cycle before trusting another run\n");

	return 0;
}

/*
 * Find the registers that actually move, and what they count.
 *
 * 0x05880000 was long described as the raster position within the programmed
 * 1360x760, and that reading is wrong. Measured, both of its 16-bit halves are
 * 10-bit counters -- max exactly 1023 -- wrapping at the same ~18 kHz. A raster
 * counter would max at HT-1 and VT-1 and its two rates would differ by a factor
 * of VT. Everything in this project that cited "scan=..." as proof the raster
 * was live was citing that misreading.
 *
 * This sweeps a register range, sampling each word over a short window, and
 * reports the ones that change together with their observed minimum, maximum
 * and change count. The real horizontal and vertical counters are identifiable
 * by their maxima: HT-1 and VT-1 for the programmed timing, printed alongside.
 * Anything maxing at a power of two minus one is a free-running counter, not a
 * raster position.
 *
 * Read-only, so it is safe to run repeatedly and after any other mode.
 */
static int h713_disp_reg_scan(ulong base, uint words)
{
	u32 total = readl(0x05880020);
	u32 vt = total >> 16, ht = total & 0xffff;
	uint i;

	printf("H713 regscan: %u word(s) from 0x%08lx, timing HT=%u VT=%u "
	       "(raster counters would max at %u and %u)\n",
	       words, base, ht, vt, ht ? ht - 1 : 0, vt ? vt - 1 : 0);

	for (i = 0; i < words; i++) {
		ulong reg = base + i * 4;
		u32 first = readl(reg);
		u32 lo = first, hi = first, prev = first;
		uint changes = 0, n;

		for (n = 0; n < 20000; n++) {
			u32 v = readl(reg);

			if (v != prev)
				changes++;
			if (v < lo)
				lo = v;
			if (v > hi)
				hi = v;
			prev = v;
		}

		if (!changes)
			continue;

		printf("  +0x%03x  min=%08x max=%08x changes=%-6u",
		       i * 4, lo, hi, changes);
		if ((hi & 0xffff) == ht - 1 || (hi >> 16) == ht - 1 ||
		    (hi & 0xffff) == vt - 1 || (hi >> 16) == vt - 1)
			printf("  <== matches a raster total");
		else if (((hi & 0xffff) & ((hi & 0xffff) + 1)) == 0 ||
			 ((hi >> 16) & ((hi >> 16) + 1)) == 0)
			printf("  (free-running, 2^n-1)");
		printf("\n");
	}

	return 0;
}

static int h713_disp_scan_rate(void)
{
	u32 total = readl(0x05880020);
	u32 vt = total >> 16, ht = total & 0xffff;
	u32 prev, cur, hi_max = 0, lo_max = 0;
	ulong t0, elapsed;
	uint hi_wraps = 0, lo_wraps = 0, samples = 0;
	u32 fast, slow;

	if (!ht || !vt) {
		printf("H713 scan: timing register reads %08x; no active raster\n",
		       total);
		return -ENODEV;
	}

	prev = readl(H713_DISP_SCAN_REG_RAW);
	t0 = timer_get_us();
	do {
		cur = readl(H713_DISP_SCAN_REG_RAW);
		if ((cur >> 16) < (prev >> 16))
			hi_wraps++;
		if ((cur & 0xffff) < (prev & 0xffff))
			lo_wraps++;
		if ((cur >> 16) > hi_max)
			hi_max = cur >> 16;
		if ((cur & 0xffff) > lo_max)
			lo_max = cur & 0xffff;
		prev = cur;
		samples++;
		elapsed = timer_get_us() - t0;
	} while (elapsed < 200000);

	printf("H713 scan: %u samples in %lu us (%lu kHz sampling), "
	       "timing HT=%u VT=%u\n", samples, elapsed,
	       (ulong)samples * 1000 / elapsed, ht, vt);
	printf("H713 scan: high half max %u, %u wrap(s) -> %lu Hz\n",
	       hi_max, hi_wraps, (ulong)hi_wraps * 1000000 / elapsed);
	printf("H713 scan: low  half max %u, %u wrap(s) -> %lu Hz\n",
	       lo_max, lo_wraps, (ulong)lo_wraps * 1000000 / elapsed);

	fast = hi_wraps > lo_wraps ? hi_wraps : lo_wraps;
	slow = hi_wraps > lo_wraps ? lo_wraps : hi_wraps;
	if (!fast) {
		printf("H713 scan: no counter movement; raster is not running\n");
		return -EIO;
	}

	printf("H713 scan: line rate %lu Hz -> DCLK %lu.%02lu MHz "
	       "(panel_config asks 62.00)\n",
	       (ulong)fast * 1000000 / elapsed,
	       (ulong)fast * ht / elapsed,
	       ((ulong)fast * ht * 100 / elapsed) % 100);
	if (slow)
		printf("H713 scan: frame rate %lu Hz (line/VT would be %lu Hz)\n",
		       (ulong)slow * 1000000 / elapsed,
		       (ulong)fast * 1000000 / elapsed / vt);
	else
		printf("H713 scan: slower counter never wrapped in the window\n");

	return 0;
}

static void h713_disp_probe_contested(const char *when)
{
	printf("H713 probe [%-22s] 051c0014=%08x 051c0028=%08x 05140054=%08x\n",
	       when, readl(0x051c0014), readl(0x051c0028), readl(0x05140054));
}

/*
 * The layer's pixel X origin. Comes up holding 123, which is exactly the pale
 * band that sat at the left of every framebuffer photograph in this bring-up.
 * test_32 proved it 1:1: 0 puts content at column 0 across the full 1280, 400
 * puts it at 406. It also caused the apparent stride deficit -- with the origin
 * at 0 the stride register measures S = V, correct as it always was.
 */
#define H713_DISP_LAYER_XOFF_REG	0x0528008cUL


/*
 * A backstop, no longer the fix.
 *
 * The real correction is the 0x0528008c record patch in
 * h713_disp_panel_patch, which is what stock does and which therefore applies
 * through the ordinary sequence, on every path, and survives the DE replay.
 *
 * This stays because it costs nothing and it *reports*. If the patch lands,
 * the register already reads the panel's value and these calls are silent. If
 * they ever print, the record patch did not take -- most likely the record's
 * own mask no longer admits [15:0] -- and that is worth knowing on the console
 * rather than rediscovering from a photograph.
 *
 * It enforces the *panel's* origin, not a literal zero. Zero is board B's
 * value; an HY310 wants 55. Hard-coding the zero here quietly undid the record
 * patch on that board after every DE replay, which showed up as a narrow
 * bright band at the right-hand edge of the projection.
 */
static void h713_disp_enforce_layer_xoff(const char *when)
{
	u32 want = h713_disp_panel->layer_x;
	u32 was;

	if (h713_disp_keep_layer_xoff)
		return;

	was = readl(H713_DISP_LAYER_XOFF_REG);
	if (was == want)
		return;

	writel(want, H713_DISP_LAYER_XOFF_REG);
	dmb();
	printf("H713 panel: layer X origin 0x%08lx was %08x %s -- the record "
	       "patch did not take; forced to %08x\n",
	       H713_DISP_LAYER_XOFF_REG, was, when,
	       readl(H713_DISP_LAYER_XOFF_REG));
}

static int h713_disp_run(ulong blob, u32 project, bool skip_hdcp_wait,
			 bool prove_ready, bool trace, bool stability,
			 bool comm_trace, bool stock_panel_power,
			 bool release_mips)
{
	struct h713_disp_sel sel;
	int ret;

	ret = h713_disp_lookup(blob, project, &sel);
	if (ret)
		return ret;

	printf("H713 disp: project 0x%02x -> prologue %u, timing %u, de %u\n",
	       sel.project, sel.prologue, sel.timing, sel.de);

	ret = h713_disp_panel_patch(blob, &sel);
	if (ret)
		return ret;

	ret = h713_logo_walk(blob, sel.prologue_range.start,
			     sel.prologue_range.end, true);
	if (ret)
		return ret;
	ret = h713_logo_walk(blob, sel.timing_range.start,
			     sel.timing_range.end, true);
	if (ret)
		return ret;

	h713_disp_fifo_reset();
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);

	ret = h713_logo_walk(blob, sel.de_range.start, sel.de_range.end,
			     true);
	if (ret)
		return ret;

	if (stock_panel_power) {
		ret = h713_disp_stock_panel_power();
		if (ret)
			return ret;
	}

	h713_disp_clocks();
	if (stock_panel_power)
		printf("H713 panel: post-stock GPIO mux: "
		       "PF6 cfg=%x latch=%d, PH16 cfg=%x latch=%d\n",
		       sunxi_gpio_get_cfgpin(SUNXI_GPF(6)),
		       !!(readl(H713_PF_DATA) & BIT(6)),
		       sunxi_gpio_get_cfgpin(SUNXI_GPH(16)),
		       !!(readl(H713_PH_DATA) & BIT(16)));

	writel(1, 0x06940000);			/* INCAP */
	mdelay(12);
	writel(0x01800045, 0x051c0010);		/* LVDS enable */
	mdelay(12);

	/*
	 * Everything the display path needs -- TCON timing, TVTOP routing, the
	 * display PLL, mixer, DE, AFBD, panel power, INCAP and LVDS -- comes
	 * from the vendor tables and the ARM sequence above. The coprocessor
	 * programs none of it; what it demonstrably does do is overwrite the
	 * TCON timing with 1080p once it runs.
	 *
	 * So holding it in reset is a real experiment, not a degraded run: if
	 * pixels appear without it, the firmware is what suppresses them and
	 * the fault is composition ownership rather than our register
	 * programming. It also leaves the panel on the tables' native 720p and
	 * lifts the one-launch-per-power-cycle rule, since no launch happens.
	 */
	h713_disp_probe_contested("ARM records applied");

	if (release_mips) {
		/*
		 * Domains, clocks, reset, then the MIPS: the firmware's first
		 * act is its HDMI-RX init, and that block is in TVCAP.
		 */
		h713_capture_prepare();
		ret = h713_mips_release_raw(skip_hdcp_wait, prove_ready, trace,
					    stability, comm_trace);
		if (ret)
			return ret;
	} else {
		printf("H713 disp: MIPS held in reset (noboot); display driven "
		       "by the ARM sequence alone\n");
	}

	h713_disp_probe_contested(release_mips ? "MIPS ready" : "MIPS held off");

	writel(0x45, 0x051c0010);		/* LVDS finalise */

	/*
	 * The one register group stock's fastlogo writes and this replay never
	 * did. Enumerating every MMIO literal in stock's fastlogo function
	 * (0x4a0228d4) and diffing against our sequence leaves exactly one
	 * omission: 0x051c00d4..0x051c00e0, written as a barriered group at
	 * raw +0x22cca.
	 *
	 * Stock has two branches for it, selected on a config field at +0xf8.
	 * When that field is unset it writes the constants below; otherwise it
	 * read-modify-writes the same registers with values derived from
	 * config +0xec/+0xf0. The fields sit past the 35-entry panel array and
	 * neither the DT nor panel_config.ini supplies them, so the constant
	 * path is the one this board takes. Either way stock writes this
	 * group and we did not.
	 *
	 * These live in the LVDS PHY block alongside the firmware's
	 * hardware-blue-screen source at +0xb0/+0xb4/+0xb8 -- which does reach
	 * the panel -- so this is the right neighbourhood for a pixel path
	 * that is configured, clocked, enabled and still delivering nothing.
	 */
	dmb();
	writel(0, 0x051c00d4);
	dmb();
	writel(0, 0x051c00d8);
	dmb();
	writel(0x08000800, 0x051c00dc);
	dmb();
	writel(0x08000000, 0x051c00e0);
	dmb();
	printf("H713 disp: LVDS PHY tail applied: %08x %08x %08x %08x\n",
	       readl(0x051c00d4), readl(0x051c00d8),
	       readl(0x051c00dc), readl(0x051c00e0));
	h713_disp_configured = true;
	printf("H713 disp: sequence complete, LVDS FIFO status=0x%08x\n",
	       readl(0x05880fe0));
	h713_disp_enforce_layer_xoff("at the end of the sequence");

	/*
	 * Readiness is a property of a released coprocessor. With the MIPS
	 * held in reset there is nothing to wait for -- CPU_COMM was never
	 * published, so the magic words read as uninitialised DRAM -- and
	 * failing the run on that would abort the very test the flag exists
	 * to perform.
	 */
	if (prove_ready && release_mips) {
		ret = h713_mips_wait_ready(H713_MIPS_DISP_READY_TIMEOUT_US,
					   trace);
		if (ret)
			return ret;
		if (stability)
			return h713_mips_monitor_stability();
	}

	return 0;
}

static const struct { ulong base; uint words; const char *name; } h713_disp_regs[] = {
	{ 0x05700000, 16, "tvtop"  }, { 0x05800000, 12, "lvds-lane" },
	{ 0x05880000, 16, "lvds"   }, { 0x058c0000, 12, "disp-pll"  },
	{ 0x051c0000,  8, "lvds-phy" }, { 0x0525c000, 16, "mixer"   },
	/* The selected DE block also writes these previously-undumped ranges. */
	{ 0x05140050,  4, "display-route" },
	{ 0x051c0020, 36, "lvds-phy-mid" },
	/*
	 * The stock ge2d_dev plane descriptor names these as the vblender,
	 * channel-0 OSD, channel-0/channel-1 DE2 companion, and AFBD source-mux
	 * windows.  The fastlogo table only writes a small subset, so a four-word
	 * layer dump could not expose a firmware-selected opaque plane or mux.
	 */
	{ 0x05200000, 32, "vblender" },
	{ 0x05248000, 32, "osd-ch0" },
	{ 0x05240000,  8, "de-top" },
	{ 0x05280000, 40, "de-layers" },
	{ 0x05288000, 16, "de2-ch0" },
	{ 0x0529c000, 16, "de2-ch1" },
	/*
	 * The firmware's blue-screen source lives at +0xb0/+0xb4/+0xb8 and the
	 * group stock writes at the end of fastlogo at +0xd4..+0xe0. Neither
	 * was ever dumped.
	 */
	{ 0x051c00b0, 16, "lvds-phy2" },
	{ 0x0524c000, 32, "de"     }, { 0x05600140, 16, "afbd"      },
	/*
	 * DE block 5 writes 0x80000020 to 0x05600000, AFBD's top-level
	 * control, but nothing has ever read it back. If the fetch unit is
	 * globally disabled, that is where it shows.
	 */
	{ 0x05600000,  4, "afbd-top" },
	{ 0x05600040, 16, "afbd-global" },
	{ 0x05600300, 16, "afbd-mux" },
	/*
	 * The vendor prologue enables PLL_VIDEO2 and the display module
	 * clocks, so they are not gated going in. Read them back after the
	 * coprocessor has run: it is the one actor that could since have
	 * changed them, and an unclocked fetch datapath behind a clocked
	 * register file would look exactly like the blank screen we have.
	 */
	{ 0x02001050,  4, "pll-video2" },
	{ 0x02001db0,  8, "disp-modclk" },
};

/*
 * Dump the blocks the vendor tables write. A register that does not hold what
 * was written to it means its block is gated or absent, which is far more
 * useful than guessing at semantics.
 */
static void h713_disp_dump(bool force)
{
	int i;

	/*
	 * These blocks are gated until the sequence runs; reading them cold
	 * stalls the interconnect and hangs the board. Refuse rather than
	 * wedge, unless the caller insists.
	 */
	if (!h713_disp_configured && !force) {
		printf("H713 disp: display not configured this boot -- reading "
		       "these blocks now would hang.\n"
		       "           run the sequence first, or 'dump force'\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(h713_disp_regs); i++) {
		uint w;

		printf("%s @0x%08lx:", h713_disp_regs[i].name,
		       h713_disp_regs[i].base);
		for (w = 0; w < h713_disp_regs[i].words; w++) {
			if (!(w % 8))
				printf("\n  +0x%02x:", w * 4);
			printf(" %08x", readl(h713_disp_regs[i].base + w * 4));
		}
		printf("\n");
	}
}


/*
 * Load the vendor display artifacts straight off the board's own eMMC.
 *
 * They all live in the stock FAT bootloader partition, which is where stock
 * U-Boot reads them from -- the "bootloader" partition lookup in its fastlogo
 * path does exactly this. Reading them here removes the fastboot staging dance
 * entirely, survives reboots, and keeps proprietary blobs out of the U-Boot
 * image, which the project's rules require.
 */
#define H713_DISP_FS_IF		"mmc"
#define H713_DISP_FS_DEV	"1:2"
#define H713_DISP_FS_PATH	"mips"
#define H713_DISP_MMC_DEV	1

/*
 * The A/B slot, as the vendor U-Boot reads it.
 *
 * Its part_get_partno() (stock 0x4a004514) does not take the name it is given:
 * slotify_name() (0x4a004454) first checks the name against the env list
 * ab_partition_list -- which ships as "bootloader,env,boot,vendor_boot,dtbo,
 * vbmeta,..." -- and, for a name in that list, appends the active slot before
 * the GPT is ever walked. The slot comes from the misc partition, where Android
 * keeps its bootloader_control block at offset 2048: four bytes of slot suffix
 * ("_a"/"_b"), then the magic 0x42414342. There is no cross-slot fallback: if
 * misc says _b, stock reads bootloader_b and nothing else (A0 report, 1a).
 *
 * A device with no misc partition, or a misc without a valid control block, is
 * not an A/B device in any way we can read, so "_a" is the answer -- that is
 * also what env.fex ships as slot_suffix.
 */
#define H713_DISP_BOOT_CTRL_OFF		2048
#define H713_DISP_BOOT_CTRL_MAGIC	0x42414342

static struct blk_desc *h713_disp_blk(void)
{
	return blk_get_devnum_by_uclass_id(UCLASS_MMC, H713_DISP_MMC_DEV);
}

static const char *h713_disp_read_slot(void)
{
	static u8 sector[512] __aligned(ARCH_DMA_MINALIGN);
	struct blk_desc *desc = h713_disp_blk();
	struct disk_partition info;

	if (!desc || part_get_info_by_name(desc, "misc", &info) < 0)
		return "_a";
	if (!info.blksz || info.blksz > sizeof(sector))
		return "_a";
	if (blk_dread(desc, info.start + H713_DISP_BOOT_CTRL_OFF / info.blksz,
		      1, sector) != 1)
		return "_a";
	if (get_unaligned_le32(sector + 4) != H713_DISP_BOOT_CTRL_MAGIC)
		return "_a";

	return (sector[0] == '_' && sector[1] == 'b') ? "_b" : "_a";
}

/*
 * Asked once. Finding a partition by name walks the GPT entry by entry, and
 * every entry re-reads the header and the whole entry array, so each lookup
 * costs on the order of a hundred transfers -- and this one sits behind every
 * other lookup. The slot cannot change while we are running: Android rewrites
 * misc, and then the board reboots.
 */
static const char *h713_disp_slot_suffix(void)
{
	static const char *cached;

	if (!cached)
		cached = h713_disp_read_slot();

	return cached;
}

/*
 * Which partition holds the display artifacts is a property of the layout, and
 * we now know three of them. "mmc 1:2" was this board's bootloader_b by index,
 * and an index is exactly what a second layout does not share: on the HY300 T08
 * and the HY350 the same number is a different partition, and on a device this
 * port was installed on there is no vendor FAT at all (doku/109).
 *
 * So ask for the partition by name, the way stock does, and let the GPT say
 * where it is:
 *
 *   bootloader_a / bootloader_b  stock, per the slot in misc
 *   bootloader                   a layout without A/B
 *   hy310-boot                   ours
 *
 * Stock first, because a device that still has both is a stock device that has
 * been installed onto -- and then the vendor artifacts in the vendor partition
 * are the ones stock itself would read. The environment overrides all of it
 * (h713_mips_dev), and our own boot script sets it, so on an installed HY310
 * none of this runs.
 *
 * The result is cached: part_get_info_by_name() walks the GPT entry by entry,
 * and a load does five reads. An mmc rescan onto a differently partitioned card
 * therefore keeps the first answer; set h713_mips_dev if that ever matters.
 */
struct h713_disp_part { const char *name; bool slotted; };

static const struct h713_disp_part h713_disp_parts[] = {
	{ "bootloader", true },
	{ "hy310-boot", false },
};

/*
 * "1#<name>" for the first form of this partition the GPT actually has, or
 * NULL. A slotted name is tried with the misc slot appended first, as stock
 * does, and then bare, because a layout without A/B carries the bare name.
 */
static const char *h713_disp_part_dev(const struct h713_disp_part *part,
				      char *out, size_t outlen)
{
	struct blk_desc *desc = h713_disp_blk();
	struct disk_partition info;
	char name[PART_NAME_LEN];

	if (!desc)
		return NULL;

	if (part->slotted) {
		snprintf(name, sizeof(name), "%s%s", part->name,
			 h713_disp_slot_suffix());
		if (part_get_info_by_name(desc, name, &info) >= 0) {
			snprintf(out, outlen, "%d#%s", H713_DISP_MMC_DEV, name);
			return out;
		}
	}

	if (part_get_info_by_name(desc, part->name, &info) < 0)
		return NULL;

	snprintf(out, outlen, "%d#%s", H713_DISP_MMC_DEV, part->name);

	return out;
}

static const char *h713_disp_resolve_dev(void)
{
	static char resolved[24];
	uint i;

	if (resolved[0])
		return resolved;

	for (i = 0; i < ARRAY_SIZE(h713_disp_parts); i++) {
		if (!h713_disp_part_dev(&h713_disp_parts[i], resolved,
					sizeof(resolved)))
			continue;
		printf("H713 disp: artifacts on %s %s, found by name\n",
		       H713_DISP_FS_IF, resolved);
		return resolved;
	}

	return H713_DISP_FS_DEV;
}

/*
 * Where those files live is a property of the installation, not of the SoC.
 * On a stock device they sit in the vendor FAT partition; an installation that
 * has given that partition to something else puts them on its own filesystem
 * and says so in the environment. fs_read() takes FS_TYPE_ANY, so ext4 works
 * exactly like FAT, and "1#hy310-boot" addresses a partition by name.
 *
 *   h713_mips_dev	device[:part] or device#partname	default: by name
 *   h713_mips_path	directory holding the files		default "mips"
 */
static const char *h713_disp_fs_dev(void)
{
	const char *s = env_get("h713_mips_dev");

	return s && *s ? s : h713_disp_resolve_dev();
}

static const char *h713_disp_fs_path(void)
{
	const char *s = env_get("h713_mips_path");

	return s && *s ? s : H713_DISP_FS_PATH;
}

/*
 * The declared project ID: which ProjectID_*.TSE group the firmware loads,
 * and -- through the panel table -- which panel is fitted.
 *
 * It is a property of the board, and two places on the board state it:
 *
 *  1. the environment, h713_project. On our layout that is the answer. The
 *     installer writes it at install time from the device's own
 *     panel_config.ini, and the boot script hands it to "h713_disp init".
 *  2. panel_config.ini itself, which the vendor keeps on Reserve0 and mirrors
 *     to media_data (/oem) -- "ProjectID = 52", in decimal, where the TSE file
 *     names and this code are hexadecimal. This is the stock case and the
 *     probe's: a device nobody has installed onto has no environment of ours,
 *     and still knows what it is.
 *
 * Deliberately not from the display.bin digest. One image serves both panels:
 * 22a7df11... is the firmware of the 720p HY300 T08 *and* of the 1080p HY350,
 * which declare 0x34 and 0x30 (doku/121 2, finding 3). The image identifies a
 * firmware revision, not a board.
 */
#define H713_DISP_PANEL_INI	"panel_config.ini"
#define H713_DISP_INI_MAX	4096

static const struct h713_disp_part h713_disp_ini_parts[] = {
	{ "Reserve0", true },
	{ "media_data", false },	/* unslotted in all three vendor GPTs */
};

static int h713_disp_ini_project(const char *dev, u32 *project)
{
	static u8 ini[H713_DISP_INI_MAX] __aligned(ARCH_DMA_MINALIGN);
	loff_t len = 0;
	ulong p, q;
	u32 val;

	if (fs_set_blk_dev(H713_DISP_FS_IF, dev, FS_TYPE_ANY))
		return -ENODEV;
	if (fs_read(H713_DISP_PANEL_INI, (ulong)ini, 0, sizeof(ini), &len))
		return -ENOENT;

	/*
	 * Line-oriented and deliberately dumb: the key has to start a line, so
	 * a commented-out one cannot be mistaken for the setting. Everything
	 * else in the file is somebody else's business.
	 */
	for (p = 0; p + sizeof("ProjectID") <= (ulong)len; p++) {
		if (p && ini[p - 1] != '\n' && ini[p - 1] != '\r')
			continue;
		if (memcmp(&ini[p], "ProjectID", strlen("ProjectID")))
			continue;

		q = p + strlen("ProjectID");
		while (q < (ulong)len && (ini[q] == ' ' || ini[q] == '\t'))
			q++;
		if (q >= (ulong)len || ini[q] != '=')
			continue;
		q++;
		while (q < (ulong)len && (ini[q] == ' ' || ini[q] == '\t'))
			q++;
		if (q >= (ulong)len || ini[q] < '0' || ini[q] > '9')
			continue;

		for (val = 0; q < (ulong)len && ini[q] >= '0' && ini[q] <= '9';
		     q++)
			val = val * 10 + (ini[q] - '0');
		*project = val;

		return 0;
	}

	return -EINVAL;
}

/*
 * Where the last h713_disp_declared_project() found its answer: "env", a
 * "1#<partition>" that panel_config.ini was read from, or "" for nowhere.
 */
static char h713_disp_project_src[24];

static int h713_disp_declared_project(u32 *project)
{
	const char *s = env_get("h713_project");
	char dev[24];
	uint i;

	h713_disp_project_src[0] = '\0';

	if (s && *s) {
		*project = hextoul(s, NULL);
		strlcpy(h713_disp_project_src, "env",
			sizeof(h713_disp_project_src));
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(h713_disp_ini_parts); i++) {
		if (!h713_disp_part_dev(&h713_disp_ini_parts[i], dev,
					sizeof(dev)))
			continue;
		if (h713_disp_ini_project(dev, project))
			continue;

		strlcpy(h713_disp_project_src, dev,
			sizeof(h713_disp_project_src));
		printf("H713 disp: project 0x%02x, from %s on %s %s\n",
		       *project, H713_DISP_PANEL_INI, H713_DISP_FS_IF, dev);
		return 0;
	}

	printf("H713 disp: this board does not say which project it is.\n"
	       "           h713_project is unset and no %s was readable.\n"
	       "           Give it on the command line: h713_disp init <id>\n",
	       H713_DISP_PANEL_INI);

	return -ENOENT;
}
/*
 * Above the framebuffer window (which ends at 0x4d941000) and below the
 * CPU_COMM share region at 0x4e300000. Parking it inside the framebuffer
 * means h713_mips_clear_workspace() erases it the moment it is loaded.
 */
#define H713_DISP_LOGO_ADDR	0x4e000000UL

/*
 * "alt" means this is an alternative source being tried before the usual one:
 * say where a file came from when it comes from there, and say nothing at all
 * when it is not there, because a miss is the normal case and not an error.
 */
static int h713_disp_read_from(const char *dev, const char *name, ulong addr,
			       loff_t *len, bool alt)
{
	char path[64];
	int ret;

	snprintf(path, sizeof(path), "%s/%s", h713_disp_fs_path(), name);

	ret = fs_set_blk_dev(H713_DISP_FS_IF, dev, FS_TYPE_ANY);
	if (ret) {
		if (!alt)
			printf("H713 disp: cannot select %s %s\n",
			       H713_DISP_FS_IF, dev);
		return ret;
	}

	ret = fs_read(path, addr, 0, 0, len);
	if (ret) {
		if (!alt)
			printf("H713 disp: cannot read %s\n", path);
		return ret;
	}

	printf("  %-28s -> 0x%08lx  %llu bytes%s%s\n", path, addr, *len,
	       alt ? "  <- " : "", alt ? dev : "");

	return 0;
}

static int h713_disp_read(const char *name, ulong addr, loff_t *len)
{
	return h713_disp_read_from(h713_disp_fs_dev(), name, addr, len, false);
}

/*
 * The TSE group, and only it, has a second source and a last resort.
 *
 * Stock tries each of the four files three times (its loop at 0x4a0232f0):
 * partition media_data -- mounted as /oem, and it prints "loading tse from oem
 * success" when that works -- then the bootloader partition, and for the
 * project file only a hard-coded ProjectID_0x0012.TSE from the bootloader
 * partition. display.bin and display_cfg.xml have no /oem fallback at all
 * (A0 report, 1a).
 *
 * The precedence is not cosmetic: /oem is where a per-unit override lands, so
 * a device tuned in the factory carries picture data there that a read of the
 * bootloader partition alone would silently miss. On our layout there is no
 * media_data partition, so the attempt costs one GPT lookup and prints nothing.
 */
/* Unslotted in all three vendor layouts, unlike Reserve0 next to it. */
static const struct h713_disp_part h713_disp_oem_part = {
	"media_data", false
};
#define H713_DISP_TSE_LAST_RESORT	"ProjectID_0x0012.TSE"

/* Also asked once, and for the same reason: it is a GPT walk per load. */
static const char *h713_disp_oem_dev(void)
{
	static char dev[24];
	static bool asked;

	if (!asked) {
		asked = true;
		if (!h713_disp_part_dev(&h713_disp_oem_part, dev, sizeof(dev)))
			dev[0] = '\0';
	}

	return dev[0] ? dev : NULL;
}

static int h713_disp_read_tse(const char *oem, const char *name, ulong addr,
			      loff_t *len)
{
	if (oem && !h713_disp_read_from(oem, name, addr, len, true))
		return 0;

	return h713_disp_read(name, addr, len);
}

/*
 * The TSE window takes the vendor databases concatenated, project file last.
 *
 * The order is stock's, read off its own fastlogo log rather than guessed:
 * database.TSE (0x44f60) to 0x4be41000, pq_custom.TSE (0x3aa8) to 0x4be85f60,
 * projecttable.TSE (0x568) to 0x4be89a08, then ProjectID_0x0034.TSE (0x4398)
 * to 0x4be89f70 -- each address being the previous one plus its size, which is
 * what pins the sequence.
 *
 * This used to read database, projecttable, ProjectID, pq_custom, so the
 * comment above described stock while the code did something else.
 *
 * That was not harmless. Putting the variable-sized project file third moved
 * every blob after it: pq_custom landed at 0x4be8b2e0 under project 0x33 and
 * 0x4be8a860 under 0x34, differing by the 0xa80 the two ProjectID files differ
 * by. Stock's order keeps the fixed-size databases at stable addresses and
 * lets only the last file move. The firmware noticed -- its own allocations at
 * afbd-mux +0x20 tracked the placement, and returned to their project-0x33
 * values once this matched stock. The display rendered correctly throughout,
 * so this was a latent divergence rather than a bug, but "the blobs are
 * self-describing so order cannot matter" was wrong.
 */
static int h713_disp_load_tse(u32 project)
{
	static const char *const fixed[] = {
		"database.TSE", "pq_custom.TSE",
		"projecttable.TSE",
	};
	const char *oem = h713_disp_oem_dev();
	char pid[40];
	ulong addr = H713_MIPS_TSE_ADDR;
	loff_t len;
	int i, ret;

	memset((void *)H713_MIPS_TSE_ADDR, 0, H713_MIPS_TSE_SIZE);

	for (i = 0; i < ARRAY_SIZE(fixed); i++) {
		ret = h713_disp_read_tse(oem, fixed[i], addr, &len);
		if (ret)
			return ret;
		addr += len;
	}

	snprintf(pid, sizeof(pid), "ProjectID_0x%04x.TSE", project);
	ret = h713_disp_read_tse(oem, pid, addr, &len);
	if (ret) {
		printf("H713 disp: no %s -- falling back to %s, as stock does\n",
		       pid, H713_DISP_TSE_LAST_RESORT);
		ret = h713_disp_read(H713_DISP_TSE_LAST_RESORT, addr, &len);
		if (ret)
			return ret;
	}
	addr += len;

	if (addr > H713_MIPS_TSE_ADDR + H713_MIPS_TSE_SIZE) {
		printf("H713 disp: TSE data overruns its window\n");
		return -ENOSPC;
	}

	/*
	 * The MIPS reads this through its own uncached mapping. Filesystem reads
	 * populate ARM cacheable DRAM, so publish both the payload and zero tail
	 * before reset release instead of depending on incidental eviction.
	 */
	flush_cache(H713_MIPS_TSE_ADDR, H713_MIPS_TSE_SIZE);

	return 0;
}

static int h713_disp_load(u32 project)
{
	loff_t len;
	int ret;

	printf("H713 disp: loading vendor artifacts from %s %s:%s\n",
	       H713_DISP_FS_IF, h713_disp_fs_dev(), h713_disp_fs_path());

	/* Clear first: the workspace wipe must not run over what we load. */
	h713_mips_clear_workspace();
	memset((void *)H713_MIPS_CFG_ADDR, 0, H713_MIPS_CFG_SIZE);

	ret = h713_disp_read("display.bin", H713_MIPS_FW_ADDR, &len);
	if (ret)
		return ret;
	/*
	 * Taking the size from the file rather than from a constant also keeps
	 * h713_mips_clear_workspace() from erasing the tail of a larger image
	 * on the next run.
	 */
	ret = h713_mips_accept_size((ulong)len);
	if (ret)
		return ret;
	/*
	 * Identify here, not at release time: the panel is chosen from the
	 * image's identity in h713_disp_lookup(), and that runs first. Hashing
	 * 1.2 MB costs a few milliseconds, and the digest is worth printing on
	 * every load anyway. h713_mips_verify() runs again before release; it
	 * is idempotent.
	 */
	ret = h713_mips_verify();
	if (ret)
		return ret;

	ret = h713_disp_read("display_cfg.xml", H713_MIPS_CFG_ADDR, &len);
	if (ret)
		return ret;
	if (len > H713_MIPS_CFG_SIZE) {
		printf("H713 disp: display_cfg.xml overruns its window\n");
		return -ENOSPC;
	}
	/*
	 * The firmware's early sys:* lookups run before its scheduler. Make the
	 * XML and its zero-filled terminator visible to the non-coherent MIPS.
	 */
	flush_cache(H713_MIPS_CFG_ADDR, H713_MIPS_CFG_SIZE);

	ret = h713_disp_load_tse(project);
	if (ret)
		return ret;
	printf("H713 disp: config/TSE windows published for MIPS\n");

	ret = h713_disp_read("LogoRegData.bin", H713_DISP_LOGO_ADDR, &len);
	if (ret)
		return ret;

	return 0;
}


/*
 * Single-command test run.
 *
 * The firmware wedges the interconnect some seconds after the sequence
 * completes, so anything typed by hand afterwards races that failure and the
 * result depends on how fast the operator types. Do the whole experiment --
 * load, config patches, sequence, timed sampling, log dump -- inside one
 * command so the timing is fixed.
 *
 * display_cfg.xml offsets are byte positions of single ASCII digits, so each
 * patch is one character and cannot change the document's length.
 */
#define H713_CFG_OFF_SOURCE_ID	0x0e48
#define H713_CFG_OFF_ELOG_MODE	0x1222
#define H713_CFG_OFF_ELOG_LEVEL	0x123c
#define H713_CFG_OFF_ELOG_ASYNC	0x125e

static int h713_cfg_set(ulong off, char want, const char *what)
{
	ulong a = H713_MIPS_CFG_ADDR + off;
	u8 cur = readb(a);

	if (cur < '0' || cur > '9') {
		printf("H713 disp: %s at +0x%04lx reads '%c', not a digit -- "
		       "config layout differs, skipping\n", what, off, cur);
		return -EINVAL;
	}

	writeb(want, a);
	printf("  %-14s '%c' -> '%c'\n", what, cur, want);

	return 0;
}

/*
 * Set one <tag val='N' /> in display_cfg.xml by NAME, not by byte offset.
 *
 * The fixed offsets this file used to carry were wrong for this board: on
 * 2026-09-01 they landed on route/mode/level instead of mode/level/async --
 * off by one field and a few bytes, because the XML moves between firmware
 * revisions. The digit guard caught it and skipped, which is why nothing was
 * corrupted, but nothing was set either. Searching for the tag survives a
 * reflow of the file; a byte offset does not.
 *
 * Deliberately not a real parser: find "<tag", then the next "val=", then the
 * quote, then one digit. That is the shape every entry in this file has.
 */
static int h713_cfg_set_tag(const char *tag, char want, const char *what)
{
	ulong base = H713_MIPS_CFG_ADDR;
	ulong end = base + H713_MIPS_CFG_SIZE;
	size_t taglen = strlen(tag);
	ulong a;

	for (a = base; a + taglen + 12 < end; a++) {
		ulong p;
		uint i;

		if (readb(a) != '<')
			continue;
		for (i = 0; i < taglen; i++)
			if (readb(a + 1 + i) != (u8)tag[i])
				break;
		if (i != taglen)
			continue;
		/* Only a full tag, not a prefix of a longer one. */
		if (readb(a + 1 + taglen) != ' ' && readb(a + 1 + taglen) != '\t')
			continue;

		for (p = a + taglen; p + 6 < end && readb(p) != '>'; p++) {
			if (readb(p) != 'v' || readb(p + 1) != 'a' ||
			    readb(p + 2) != 'l' || readb(p + 3) != '=')
				continue;
			p += 4;
			if (readb(p) == '\'' || readb(p) == '"')
				p++;
			return h713_cfg_set(p - base, want, what);
		}
	}

	printf("H713 disp: %s -- <%s val='N'> not found in display_cfg.xml\n",
	       what, tag);
	return -ENOENT;
}

static void h713_disp_sample(void)
{
	static const uint at_ms[] = { 0, 100, 500, 1000, 2000, 4000 };
	uint i, elapsed = 0;

	printf("H713 disp: LVDS FIFO over time\n");
	for (i = 0; i < ARRAY_SIZE(at_ms); i++) {
		if (at_ms[i] > elapsed) {
			mdelay(at_ms[i] - elapsed);
			elapsed = at_ms[i];
		}
		printf("  t=%4u ms  fifo=0x%08x  status=0x%08x\n",
		       elapsed, readl(0x05880fe0),
		       readl(H713_MIPS_STATUS_REG));
	}
}

#define H713_DISP_OSD_FB_ADDR		0x6c100000UL
/*
 * The OSD surface is the active panel, not a constant. It was 1280x720 here
 * because that is board B's panel.
 */
#define H713_DISP_OSD_WIDTH		(h713_disp_panel->width)
#define H713_DISP_OSD_HEIGHT		(h713_disp_panel->height)
#define H713_DISP_OSD_STRIDE		(H713_DISP_OSD_WIDTH * sizeof(u32))
#define H713_DISP_OSD_SIZE		(H713_DISP_OSD_STRIDE * \
					 H713_DISP_OSD_HEIGHT)
#define H713_DISP_OSD_CTRL_REG		0x0524c000UL
#define H713_DISP_OSD_OPEN_REG		0x0524c01cUL
#define H713_DISP_AFBD_CTRL_REG		0x05600140UL
#define H713_DISP_AFBD_READY_REG	0x05600144UL
#define H713_DISP_AFBD_STATUS_REG	0x05600168UL
#define H713_DISP_AFBD_STRIDE_REG	0x05600170UL
/*
 * AFBD +0x38, the source address. DE block 5 writes 0x6c100000 here, and
 * fb-anim proved AFBD streams from it live rather than latching the surface at
 * submission -- so this register is the page-flip lever for double buffering.
 */
#define H713_DISP_AFBD_SRC_REG		0x05600178UL
/*
 * The back buffer, one megabyte-aligned surface after the front. On board B
 * that is 4 MiB and works out to the 0x6c500000 this used to hard-code; a
 * 1920x1080 surface is 8294400 bytes and the back buffer lands at 0x6c900000.
 *
 * patches/kernel/0024 reserves uboot-scanout@6c100000 at 0x800000, which
 * covers both 720p surfaces but only *one* 1080p surface. Double-buffered
 * 1080p needs that raised to 0x1000000. The single-buffered path, which is
 * what the boot logo uses, is unaffected.
 */
#define H713_DISP_OSD_FB_ADDR_B		(H713_DISP_OSD_FB_ADDR + \
					 ALIGN(H713_DISP_OSD_SIZE, 0x100000UL))

/*
 * The panel's declared power-down timings, from panel_config.ini:
 *
 *	PanelOnTiming0/1/2  = 20 / 550 / 75
 *	PanelOffTiming0/1/2 = 20 / 250 / 75
 *
 * The on-side mapping is confirmed by the code above: OnTiming1 (550) is the
 * pre-delay before the GPIO phase, and OnTiming0 (20) is the settle after it,
 * before the display clocks and the MIPS release. OnTiming2 (75) is not
 * accounted for on the on side either.
 *
 * The off-side assignment below is therefore SYMMETRY, NOT KNOWLEDGE. Off0
 * mirrors On0 as the settle after the signal stops, Off1 mirrors On1 as the
 * delay bracketing the power rail, and Off2 fills the gap between dropping
 * reset and dropping power. If a panel datasheet or the stock power-down path
 * ever contradicts this, believe them, not this comment.
 */
#define H713_PANEL_OFF_T0_MS	20
#define H713_PANEL_OFF_T1_MS	250
#define H713_PANEL_OFF_T2_MS	75

/*
 * Put the display back where a fresh bring-up can pick it up.
 *
 * Every mode has, until now, ended by abandoning the hardware: panel powered,
 * MIPS in reset, clocks running. That is why "power-cycle before another run"
 * exists, why a second run in one boot produces a dark panel with an
 * identical-looking console, and why that trap cost four results before the
 * refusal guard made it visible.
 *
 * The likely reason a second run fails is right here in the GPIOs. The bring-up
 * powers the panel by driving PF6 high and pulsing PH16; if the panel is still
 * powered from the previous run, that "power on" is a no-op and the panel never
 * re-runs its own init. Dropping both properly is what should make a second
 * bring-up behave like a first.
 *
 * Ordering is the general LVDS rule -- backlight off, then signal, then VCC --
 * because it is the one part of this with real hardware risk. Panels can latch
 * up when the data lanes outlive the rail, and repeated violations shorten
 * their life.
 *
 * Two deliberate omissions:
 *
 *   - The backlight is not touched. Which PWM drives it is still unresolved:
 *     display_cfg.xml says channel 0, panel_config.ini says channel 5, and the
 *     dimmer we implemented drives PWM2 on PB4, which matches neither. Turning
 *     off the wrong channel would be theatre.
 *
 *   - The display PLLs and mod clocks are left running. Disabling a block's
 *     clock while something still reaches for it is the documented way to wedge
 *     this interconnect, and the acceptance test does not need it: the bring-up
 *     replays LogoRegData, which rewrites this state absolutely. Leaving them
 *     up is the conservative choice, not an oversight.
 *
 * The firmware cannot do any of this for us. THal_Vp_Deinit was tested on
 * hardware and changes four words inside AFBD, leaving LVDS, the PHY, the PLL,
 * the mixer and the DE byte-identical.
 */
/*
 * The panel dimmer, per the stock runtime DTB's tvtop panel block:
 *
 *	panel_pwm_ch = 2      PWM2, which the h713 pinctrl driver muxes on
 *	                      PB4 at function 2
 *	panel_pwm_freq = 25000
 *	panel_pwm_pol = 0     active high
 *	panel_pwm_min = 0, panel_pwm_max = 100, panel_backlight = 75
 *
 * Register layout from our own sun8i 8-channel PWM driver (patch 0007).
 * 24 MHz HOSC with both dividers at 1 gives 24e6/25e3 = 960 cycles a period,
 * which fits the 16-bit fields with room for 100% duty.
 *
 * This exists to retest a parked negative. The earlier bench note -- "a running
 * 25 kHz PWM on PB4 changed brightness not at all, so PB4/PWM2 is not the
 * control" -- was taken when the display path did not work, and the standing
 * explanation was that the panel ignores its dim input until fastlogo has run
 * its serial init. That init now demonstrably happens: the panel renders. So
 * the test is worth repeating against a panel that is actually up, which was
 * never possible before.
 */
/*
 * This is the mainline `pwm-sun20i-d1` register map, the second-generation
 * sunxi PWM IP. Do not "correct" it against `pwm-sun8i.c` (patch 0007) -- that
 * was tried on 2026-08-05 and it is wrong, in a way this hardware proved:
 *
 *   - patch 0007 calls 0x040 dead-zone control and 0x060 ENABLE. Under this
 *     map 0x040 is the per-channel CLK_GATE and 0x080 is ENABLE. After the
 *     switch, a full block dump read 0x040 = 0 and 0x080 = 0 -- the channel
 *     ungated and disabled -- and CNT(2) at 0x02000d48 read 0, i.e. the
 *     counter was not running. Under patch 0007's map every field was set
 *     correctly and the counter should have been counting. It was not.
 *   - patch 0007 puts CLK_CFG SRC at [1:0], DIV_M at [6:4] and a gate at bit
 *     7. Here SRC is [8:7] and DIV_M is [3:0], which is exactly the 0x18f
 *     mask below, and the gating lives in the separate CLK_GATE register.
 *   - patch 0007 puts active cycles at [31:16]. Here [31:16] is entire-1.
 *
 * The "verified from live hardware" note in patch 0007 -- PERIOD2 = 0x03BF03C0
 * -- does not discriminate: 0x3bf = 959 and 0x3c0 = 960, so it reads as 25 kHz
 * at ~100% duty under *either* field order. It is a degenerate sample and must
 * not be cited as proof of the layout again.
 *
 * Whether patch 0007 is also wrong for the R_PWM instance it was DMM-validated
 * against is open; that block may be a different generation. Nothing here
 * depends on the answer.
 */
#define H713_PWM_BASE		0x02000c00UL
#define H713_PWM_PCCR(pair)	(H713_PWM_BASE + 0x020 + (pair) * 4)
#define H713_PWM_CLK_GATE	(H713_PWM_BASE + 0x040)
#define H713_PWM_ENABLE		(H713_PWM_BASE + 0x080)
#define H713_PWM_CTL(ch)	(H713_PWM_BASE + 0x100 + (ch) * 0x20)
#define H713_PWM_PERIOD(ch)	(H713_PWM_BASE + 0x104 + (ch) * 0x20)
#define H713_PWM_CNT(ch)	(H713_PWM_BASE + 0x108 + (ch) * 0x20)

#define H713_BL_PWM_CH		2
/*
 * Matches the vendor DTB. Note this must drop to <= 2.5 kHz if the planned
 * inline MOSFET dimmer is ever fitted -- see docs/backlight-investigation.md,
 * "The chosen path". Left at 25 kHz until then so the value keeps matching the
 * measurement it is documented by (counter wraps at 960).
 */
#define H713_BL_PWM_HZ		25000
/*
 * PB4's pwm2 function is mux **3**, not 2. Corrected 2026-08-05 after a sweep
 * with a verifiably correct PWM -- CLK_CFG=0x80, PERIOD=03c003c0..000003c0,
 * EN bit 2 set -- changed brightness not at all, because the waveform was
 * being driven into a pad muxed to a different function.
 *
 * Three independent sources say 3: board B's own stock U-Boot DTB
 * (`pwm2@0 { allwinner,pins = "PB4"; allwinner,muxsel = <0x03>; }`), the
 * upstream mainline H616 pinctrl table, and patch 0018. The only source
 * saying 2 is patch 0002, our own H713 pinctrl transcription -- which is also
 * the table the kernel binds, so the Linux pwm-backlight test of 2026-07-24
 * had the same fault and its debugfs "function pwm2" was true at the
 * abstraction level and wrong at the register level. One transcription error,
 * both null results.
 *
 * Patch 0002 still needs the same correction for Linux.
 */
#define H713_BL_PWM_MUX		3

/*
 * The PWM block is gated and held in reset out of cold boot, and nothing in
 * the display path ungates it. The first sweep wrote CTL, PERIOD and ENABLE
 * and read all three back as zero -- the writes went nowhere, so the run said
 * nothing about the panel.
 *
 * CCU 0x7ac carries both, from our own H713 CCU driver: BIT(0) is bus-pwm's
 * gate and BIT(16) is RST_BUS_PWM. Gate first, then release reset, which is
 * the ordering the sunxi clock code uses.
 */
#define H713_CCU_PWM_BGR	0x020017acUL

static void h713_disp_pwm_bus_enable(void)
{
	setbits_le32((void *)H713_CCU_PWM_BGR, BIT(0));
	dmb();
	udelay(20);
	setbits_le32((void *)H713_CCU_PWM_BGR, BIT(16));
	dmb();
	udelay(20);
}

/*
 * Modulate the light's supply enable on PB5 by hand.
 *
 * PB5 has no PWM function in this SoC's pinmux -- gpio_in/gpio_out only -- so
 * toggling is the only way to modulate it. That is worth doing because the
 * light is not fed a raw rail: the LED header measures 52.6 V against a 36 V
 * input, so a boost converter sits between them, and PWM-on-enable is a
 * standard dimming technique for LED boost drivers. If this converter honours
 * it, brightness control needs no new hardware at all.
 *
 * Shell loops cannot test this. A recursive "run" gives a duty set by command
 * dispatch time (roughly 90% on) at an uncontrolled frequency, which is why
 * bit-banging from the prompt showed nothing.
 *
 * Bounded by construction: it runs for a fixed time and always restores PB5
 * high on exit. Ctrl-C aborts a command, not a loop, so a U-Boot experiment
 * that cannot end by itself is one power cycle away from being a nuisance.
 *
 * PB5 is shared with fan power. Low duty slows the fan, so prefer short runs
 * and start high; the light being off for the same period makes this thermally
 * safe, but a stalled fan is still worth avoiding.
 */
static void h713_disp_bl_gpio_pwm(uint freq_hz, uint duty_pct, uint secs)
{
	ulong period_us, hi_us, lo_us, cycles, i;

	if (freq_hz < 1 || freq_hz > 20000) {
		printf("bl-gpio: frequency %u out of range (1..20000 Hz)\n",
		       freq_hz);
		return;
	}
	if (duty_pct > 100)
		duty_pct = 100;
	if (secs < 1 || secs > 30)
		secs = 3;

	period_us = 1000000UL / freq_hz;
	hi_us = period_us * duty_pct / 100;
	lo_us = period_us - hi_us;
	cycles = (ulong)freq_hz * secs;

	printf("bl-gpio: PB5 %u Hz %u%% for %u s (%lu us high, %lu us low)\n",
	       freq_hz, duty_pct, secs, hi_us, lo_us);

	sunxi_gpio_set_cfgpin(SUNXI_GPB(5), SUNXI_GPIO_OUTPUT);

	for (i = 0; i < cycles; i++) {
		if (hi_us) {
			setbits_le32((void *)H713_PB_DATA, BIT(5));
			udelay(hi_us);
		}
		if (lo_us) {
			clrbits_le32((void *)H713_PB_DATA, BIT(5));
			udelay(lo_us);
		}
	}

	/* Never leave the fan/backlight interlock deasserted. */
	setbits_le32((void *)H713_PB_DATA, BIT(5));
	printf("bl-gpio: done, PB5 restored high\n");
}

static void h713_disp_backlight_set(uint percent)
{
	const uint ch = H713_BL_PWM_CH;
	u32 period = 24000000u / H713_BL_PWM_HZ;	/* 960 cycles at 25 kHz */
	u32 act;
	u32 cnt_a, cnt_b;

	if (percent > 100)
		percent = 100;
	act = period * percent / 100;

	h713_disp_pwm_bus_enable();
	sunxi_gpio_set_cfgpin(SUNXI_GPB(4), H713_BL_PWM_MUX);

	/* HOSC, divide by one: clear both CLK_SRC[8:7] and DIV_M[3:0]. */
	clrbits_le32((void *)H713_PWM_PCCR(ch / 2), 0x18f);
	/* Prescaler 0, ACT_STA high to match panel_pwm_pol = 0. */
	writel(BIT(8), H713_PWM_CTL(ch));
	/* [31:16] entire cycles - 1, [15:0] active cycles. */
	writel(((period - 1) << 16) | act, H713_PWM_PERIOD(ch));
	setbits_le32((void *)H713_PWM_CLK_GATE, BIT(ch));
	setbits_le32((void *)H713_PWM_ENABLE, BIT(ch));
	dmb();

	/*
	 * The counter is the only witness that the channel is actually running
	 * rather than merely configured. Two reads a few microseconds apart
	 * must differ -- a full 960-cycle period is 40 us, so a static value
	 * means no clock is reaching the channel.
	 */
	cnt_a = readl(H713_PWM_CNT(ch));
	udelay(7);
	cnt_b = readl(H713_PWM_CNT(ch));

	printf("H713 backlight: PWM%u %u%% duty (%u/%u cycles at %u Hz), "
	       "PB4 mux=%d, BGR=%08x PCCR=%08x CTL=%08x PERIOD=%08x "
	       "GATE=%08x EN=%08x CNT=%08x->%08x\n",
	       ch, percent, act, period, H713_BL_PWM_HZ,
	       sunxi_gpio_get_cfgpin(SUNXI_GPB(4)),
	       readl(H713_CCU_PWM_BGR),
	       readl(H713_PWM_PCCR(ch / 2)),
	       readl(H713_PWM_CTL(ch)), readl(H713_PWM_PERIOD(ch)),
	       readl(H713_PWM_CLK_GATE), readl(H713_PWM_ENABLE),
	       cnt_a, cnt_b);

	/* Check this channel's bit, not the register being nonzero. */
	if (!(readl(H713_PWM_ENABLE) & BIT(ch)))
		printf("H713 backlight: PWM%u enable bit did not stick -- the "
		       "block is not accepting writes, so this step says "
		       "nothing about the panel\n", ch);
	if (cnt_a == cnt_b)
		printf("H713 backlight: PWM%u counter is static -- the channel "
		       "is configured but not running, so this step says "
		       "nothing about the panel\n", ch);
}


static void h713_disp_teardown(const char *why)
{
	u32 ctrl;

	printf("H713 teardown: %s\n", why);

	/*
	 * 0. Make the display blocks reachable before touching one.
	 *
	 * The idea is the vendor's -- its shutdown log reads "ge2d: acquire
	 * tvdisp clock on emergency shutdown", turning the display clock ON in
	 * order to turn the display off. Step 1 below READS the AFBD control
	 * register, and those blocks wedge the interconnect when read gated, so
	 * without this teardown could only run after a completed sequence.
	 *
	 * The clocks alone are NOT enough, which cost a hang on 2026-08-07:
	 * h713_display_clocks_on() ran, and the very next AFBD read still took
	 * the board. h713_display_prepare()'s own comment says why -- "the
	 * initial SPL values open only part of the display fabric" -- so the
	 * TVTOP routing at 0x05700000 is part of what makes these blocks
	 * reachable, not merely part of configuring them.
	 *
	 * So call the whole of prepare(). It is not a guess: every normal run
	 * calls it cold before any display access, so it is the one primitive
	 * proven safe from this exact state. Only when the sequence has not
	 * run -- on the warm path everything is already up and re-running it
	 * would churn TVTOP and the mixer for nothing.
	 */
	if (!h713_disp_configured)
		h713_display_prepare();

	/* 1. Stop scanout before anything downstream of it goes away. */
	ctrl = readl(H713_DISP_AFBD_CTRL_REG);
	writel(ctrl & ~BIT(0), H713_DISP_AFBD_CTRL_REG);
	dmb();

	/*
	 * 2. Park the coprocessor. Also clears h713_display_prepared. The TV
	 *    domains and capture clocks stay up, as teardown always left them.
	 */
	h713_mips_stop();

	/*
	 * 3. Signal off, back to the values every cold probe in this bring-up
	 *    has recorded before init: 051c0014=18000000, 051c0028=00000030,
	 *    05140054=40000000.
	 */
	writel(0x00000000, 0x051c00d4);
	writel(0x00000000, 0x051c00d8);
	writel(0x00000000, 0x051c00dc);
	writel(0x00000000, 0x051c00e0);
	writel(0x18000000, 0x051c0014);
	writel(0x00000030, 0x051c0028);
	writel(0x40000000, 0x05140054);
	dmb();

	mdelay(H713_PANEL_OFF_T0_MS);

	/* 4. Reset asserted, then the rail, then let it discharge. */
	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	dmb();
	mdelay(H713_PANEL_OFF_T2_MS);

	clrbits_le32((void *)H713_PF_DATA, BIT(6));
	dmb();
	mdelay(H713_PANEL_OFF_T1_MS);

	printf("H713 teardown: panel down (PF_DAT=%08x PH_DAT=%08x), "
	       "PHY %08x/%08x, route %08x, MIPS reset=%08x\n",
	       readl(H713_PF_DATA), readl(H713_PH_DATA),
	       readl(0x051c0014), readl(0x051c0028), readl(0x05140054),
	       readl(H713_MIPS_RESET_REG));
}

/*
 * The one place a failed display run cleans up after itself.
 *
 * Every error path in h713_disp_panel_test, h713_disp_init_only and
 * h713_disp_test used to be a bare "return ret", leaving the panel powered and
 * the MIPS live. Not hypothetical: it is exactly what vendor-logo-late did when
 * its bootlogo hash refusal fired, and the next run then initialised into a
 * half-torn-down state and came back dark with a console that looked perfect.
 *
 * Deliberately a shared failure handler rather than the single "goto out" the
 * plan called for, because SUCCESS must not tear down. panel-test leaving the
 * display up is load-bearing: it is what lets "panel-test ... ; boot" hand a
 * live frame to Linux, which is how the handoff was verified on 2026-08-07.
 * A single exit covering both would have silently broken that, and the same
 * applies to init_only, whose whole purpose is to leave the display up for
 * scanrate/regscan/clkfind.
 *
 * This used to be guarded on h713_disp_configured, because teardown opens by
 * reading the AFBD control register and those blocks hang the interconnect when
 * read gated -- so a failure before the sequence completed could not be cleaned
 * up at all. h713_disp_teardown() now acquires the display clocks first, the
 * way the vendor's own shutdown does, so it works from any state and the guard
 * is gone.
 */
static int h713_disp_fail(int ret)
{
	h713_disp_teardown("run failed, tearing down");
	return ret;
}

/*
 * Refuse a cold read of the display blocks rather than hang the board.
 *
 * h713_disp_dump has always done this; scanrate, regscan and clkfind never did,
 * and "h713_disp scanrate" on a cold boot wedged the board on 2026-08-07 --
 * h713_disp_scan_rate() reads 0x05880000 three times and that block is gated
 * until the sequence runs.
 *
 * Deliberately a refusal and not the ungate that teardown got. Teardown has
 * real work to do cold: drop the rails, park the MIPS. These three only
 * *measure*, and every one of them measures something driven by the display
 * PLL -- so with the clocks merely ungated and no sequence run they would
 * return a confident number that means nothing. A refusal is the honest answer.
 */
static bool h713_disp_readable(const char *what)
{
	if (h713_disp_configured)
		return true;

	printf("H713 disp: display not configured this boot -- %s reads blocks "
	       "that are still gated, and would hang.\n"
	       "           run 'panel-test' or 'init' first\n", what);
	return false;
}



/*
 * The measured fetch stride, from test_30: the edge sweep's five photographed
 * steps put the source advance at 1237 px/row, and a search over every stride
 * from 2 to 4000 has no other solution -- a stripe count fixes S mod P, and the
 * five pitches together leave one candidate. See docs/claude-display-handoff.md.
 *
 * Fill and sweep spans have to cover the largest stride any mode drives, not
 * 1280: the frame consumes stride * 720 words, so 1296 px/row reads 933120
 * words where the nominal buffer holds 921600. 1360 covers every register
 * value fb-fix drives even if the unit slope turns out to be wrong and the
 * stride tracks the register outright, and ends at 0x6c4ec000, clear of
 * everything in the memory map -- the next region down ends at 0x4e800000.
 */
#define H713_DISP_EDGE_PITCH		1237
/* Board B's htotal. The edge sweeps were written against that panel. */
#define H713_DISP_EDGE_SPAN_PX		1360
#define H713_DISP_EDGE_SPAN_WORDS	(H713_DISP_EDGE_SPAN_PX * \
					 H713_DISP_OSD_HEIGHT)
#define H713_DISP_LVDS_SCAN_REG		0x05880000UL
#define H713_DISP_LVDS_LANE_REG		0x05800000UL
/*
 * Moved up from 0x6d000000: a double-buffered 1080p pair reaches 0x6d100000,
 * which would have run into the staging area.
 */
#define H713_DISP_VENDOR_BMP_ADDR	0x6e000000UL
#define H713_DISP_VENDOR_BMP_SIZE	2764854UL
/* A 1920x1080 24-bit BMP is 6220854 bytes and does not fit in 4 MiB. */
#define H713_DISP_VENDOR_BMP_MAX	0x00800000UL
#define H713_DISP_PANEL_BLUE_RGB0	0x051c00b0UL
#define H713_DISP_PANEL_BLUE_RGB1	0x051c00b4UL
#define H713_DISP_PANEL_BLUE_CTRL	0x051c00b8UL
#define H713_DISP_TCON_CTRL_REG		0x0588000cUL
#define H713_DISP_TCON_MODE_REG		0x0588001cUL
#define H713_DISP_TCON_PATTERN_SIZE_REG	0x05880038UL
#define H713_DISP_TCON_PATTERN_RGB0_REG	0x0588003cUL
#define H713_DISP_TCON_PATTERN_RGB1_REG	0x05880040UL

/*
 * Every visible phase holds for this long. One second per frame is not enough
 * time to photograph by hand, and these runs are judged by eye: an image the
 * operator cannot capture is not evidence.
 *
 * The bar pattern cycles eight colours, so its rotation repeats with period
 * eight -- eight frames cover every distinct image and anything beyond that
 * is a duplicate. The previous 15x1s run was therefore seven redundant frames,
 * each too brief to catch.
 */
#define H713_DISP_OSD_FRAMES		8
#define H713_DISP_OSD_DWELL_MS		5000
#define H713_DISP_STATIC_FRAME_DWELL_MS	15000

/*
 * LogoRegData.bin DE block 5 writes 0x6c100000 to AFBD +0x38
 * (0x05600178), 0x1400 to +0x30, and 0x02cf04ff to +0x10. Together those
 * identify a linear 1280x720, 32-bit OSD buffer with a 5120-byte stride.
 * Stock U-Boot blits bootlogo.bmp into that buffer after applying the table;
 * our register-only replay must provide pixels explicitly.
 */
/*
 * Find the display's true line pitch by matching the pattern to it.
 *
 * Every framebuffer probe is explained by one model: the hardware fetches
 * P = 1280 + d pixels per display line where we write 1280, so each display row
 * starts d pixels further into the buffer than the last.
 *
 *   fb-vprobe  horizontal bands  row Y shows source row ~Y(1+d/1280); for d~10
 *                                a 0.8% compression, invisible -- looks correct
 *   fb-hprobe  vertical bands    the row start walks, cycling the eight bands
 *                                every 1280/d rows -> fine horizontal rainbow
 *   fb-quad    quadrants         each row lands wholly in one quadrant colour ->
 *                                solid full-width stripes, halves swapping
 *   bootlogo   36-row text       sheared into a thin diagonal streak
 *
 * It also explains why fbcheck finds the framebuffer byte-exact. It is. The
 * fault is entirely in how the display walks it.
 *
 * Counting rainbow cycles in the fb-hprobe capture gives d ~ 9.7 px, but blur
 * merges stripes and undercounts, so that is worth +-4 px at best. Rather than
 * guess a register from a soft number, sweep the pattern instead: fill the
 * buffer so the eight bands repeat every P pixels in LINEAR index order, for a
 * series of candidate P. When P equals the hardware's true pitch the bands stand
 * up straight and stop sliding; every other value leaves them sloped or striped.
 *
 * The criterion is unambiguous and needs no register knowledge -- and once P is
 * known, d = P - 1280 says exactly how much a stride or width field is out.
 */
/*
 * Chase the edge artefact by sweeping the horizontal back porch.
 *
 * A pale vertical band has sat at one side of the projection in every capture in
 * this log. It is not the framebuffer bug: it appears in the TCON generator
 * tests too, which bypass framebuffer, AFBD, OSD and DE entirely, so it lives in
 * the TCON timing or the panel. Worth removing on its own account -- one fewer
 * variable before the pitch question is settled.
 *
 * 0x05880028 reads 00140028 against panel_config hsync=20 and hbp=40, so the
 * halves are those fields. With HT=1360 and HA=1280 the front porch is
 * 1360-1280-20-40 = 20. If the panel wants a different back porch the active
 * window sits at the wrong offset and the remainder shows as a band at one edge.
 *
 * Sweep it with the style-8 checker running, which renders correctly, so any
 * shift of the image is obvious. If the band moves with the value it is a porch
 * error and the centring value is the answer; if it does not move at all, the
 * band belongs to the panel and the timing is exonerated.
 */
static const u16 h713_hbp_sweep[] = { 40, 20, 30, 50, 60, 80 };

static const u16 h713_pitch_sweep[] = { 1280, 1284, 1288, 1292, 1296, 1300 };

/*
 * All six of the narrow sweep came back as solid per-row colour, which means the
 * slide exceeds one band (160 px) at every one of them: the pitch is nowhere
 * near 1280, and the d ~ 9.7 px read off the fb-hprobe capture was moire -- the
 * same aliasing trap already flagged for fb-quad and then walked into anyway.
 *
 * The leading candidate is that the hardware walks HTOTAL rather than the active
 * width. 0x05880020 reads 02f80550: 760 total lines by 1360 total columns
 * against a 1280x720 active area. P = 1360 predicts a half-band slide per row,
 * an eight-band cycle every 16 rows and ~45 cycles down the frame -- which is
 * the fine rainbow observed -- while compressing fb-vprobe by only 1.0625, still
 * reading as correct. It was not in the narrow list.
 *
 * This wider list brackets that and the usual alignment roundings. If 1360 is
 * the one that stands up, the fix is to feed the framebuffer a 1360-pixel stride
 * (5440 bytes) or to correct whichever field is handing HTOTAL to the fetch.
 */
static const u16 h713_pitch_sweep_wide[] = {
	1360, 1366, 1408, 1440, 1536, 1920,
};

/*
 * Below 1280, which both earlier sweeps missed.
 *
 * The operator reports that the TCON generator patterns -- checkerboards and
 * solids -- fill the whole screen, while the framebuffer patterns are truncated
 * with a pale band at one side. That contradicts what this log claimed and what
 * was repeated here: the band is NOT a panel or TCON artefact, it belongs to the
 * framebuffer path, and it is part of this bug rather than a separate one.
 *
 * It also measures the fault directly. If the fetch supplies P pixels for a
 * 1280-pixel display line, the content occupies P/1280 of the width and the
 * remainder is the band. Two captures agree: content is 92.8% and 92.7% of the
 * projection, so P is about 1187 -- below 1280, where neither sweep looked,
 * because both were chasing an HTOTAL theory that had the sign wrong.
 *
 * P = 1187 also fits everything else: a -93 px/row slide gives ~52 rainbow
 * cycles down the frame, and fb-vprobe stretches by only 1.08, which still reads
 * as correct.
 *
 * The band gives a far better success criterion than band verticality: at the
 * correct pitch the content fills the full width and the band disappears. That
 * is unmistakable in a blurry photograph, where judging whether fine bands are
 * quite vertical is not.
 */
static const u16 h713_pitch_sweep_low[] = {
	1180, 1184, 1188, 1192, 1196, 1200,
};

static void h713_disp_fill_pitch(u32 pitch)
{
	static const u32 hues[] = {
		0xffff0000, 0xffff8000, 0xffffff00, 0xff00ff00,
		0xff00ffff, 0xff0000ff, 0xff8000ff, 0xffff00ff,
	};
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	/*
	 * Cover pitch * height, not width * height. If the hardware walks P
	 * pixels per row then row Y starts at Y*P, and a buffer of only
	 * 1280*720 leaves the lower frame reading unwritten memory for every
	 * P > 1280 -- 6% of the frame at 1360, a third of it at 1920. The first
	 * sweep was testing garbage over exactly the region a larger pitch
	 * pushes into.
	 */
	u32 total = pitch * H713_DISP_OSD_HEIGHT;
	u32 band = pitch / ARRAY_SIZE(hues);
	u32 i;

	for (i = 0; i < total; i++) {
		u32 b = (i % pitch) / band;

		fb[i] = hues[b < ARRAY_SIZE(hues) ? b : ARRAY_SIZE(hues) - 1];
	}
	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
}

/*
 * Eight vertical bands. The exact complement of fb-vprobe.
 *
 * fb-vprobe varies only down the frame and renders correctly. fb-quad varies on
 * both axes and comes back as horizontal stripes, each a solid colour spanning
 * the full width -- which no single source row can produce, since each quadrant
 * colour occupies only half the width. So a display line is being drawn from
 * part of a source line, and horizontal variation is being turned into vertical
 * structure.
 *
 * That is as far as the quadrants go. The visible stripe period is ~38 display
 * rows, far too coarse for a per-row effect, and at 1.8 display rows per photo
 * pixel the capture is almost certainly showing moire rather than the true
 * period. No stride ratio can honestly be read off it.
 *
 * These bands vary only ACROSS the frame, isolating the axis fb-vprobe could
 * not test. 160 px each, wide enough to survive blur:
 *
 *   eight vertical bands, correct order   horizontal addressing is fine, and the
 *                                         quad failure is an interaction effect
 *   horizontal stripes instead            the axes are transposed or interleaved
 *   more than eight bands                 the line is repeating; the count gives
 *                                         the true pitch as a ratio, which is the
 *                                         measurement the moire denied us
 *   eight bands, wrong order              the line is being read out permuted
 *
 * Note the left/right column artefact is not this bug. It appears in the TCON
 * generator tests too, which bypass the framebuffer entirely, so it belongs to
 * the TCON/LVDS/panel layer -- most likely a horizontal offset or porch error --
 * and will not be fixed by whatever is wrong here.
 */
static void h713_disp_fill_vbands(void)
{
	static const u32 hues[] = {
		0xffff0000, 0xffff8000, 0xffffff00, 0xff00ff00,
		0xff00ffff, 0xff0000ff, 0xff8000ff, 0xffff00ff,
	};
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	const u32 W = H713_DISP_OSD_WIDTH, H = H713_DISP_OSD_HEIGHT;
	u32 band_w = W / ARRAY_SIZE(hues);
	u32 x, y;

	for (y = 0; y < H; y++) {
		for (x = 0; x < W; x++) {
			u32 band = x / band_w;

			fb[y * W + x] = hues[band < ARRAY_SIZE(hues) ?
					     band : ARRAY_SIZE(hues) - 1];
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: %u vertical bands of %u px published at 0x%08lx "
	       "(red orange yellow green cyan blue purple magenta, left to "
	       "right)\n", (uint)ARRAY_SIZE(hues), band_w,
	       H713_DISP_OSD_FB_ADDR);
}

/*
 * Four solid quadrants. The blur-proof geometry test.
 *
 * fb-grid showed the answer -- a dense repeating lattice where a sparse 10x6
 * grid was drawn, and 80x80 corner squares arriving as full-width bands, which
 * is the image tiling rather than scaling. But its fine features could not be
 * measured from a handheld photograph: autocorrelation on the capture decayed
 * monotonically from the shortest lag, reading camera blur rather than any
 * period. A pattern is only as good as the photograph it has to survive.
 *
 * So: four quadrants, each a solid 640x360 block. Red top-left, green
 * top-right, blue bottom-left, yellow bottom-right. Nothing else. These stay
 * legible through blur, defocus and keystone, and the colour sequence encodes
 * the addressing directly:
 *
 *   four quadrants, correct colours    geometry is right
 *   quadrants present but transposed   flip, mirror or rotation
 *   horizontal stripes cycling
 *     red green red green ...          each output row is drawing from a
 *                                      different source row -- a row-pitch
 *                                      mismatch, and the number of complete
 *                                      colour cycles down the frame gives the
 *                                      ratio between the true pitch and the
 *                                      5120 bytes written
 *   vertical stripes                   pixel-size or per-pixel stride error
 *
 * The stripe count is the measurement fb-grid could not deliver: counting four
 * or five wide bands in a blurry photo is reliable in a way that counting
 * lattice dots is not.
 */
static void h713_disp_fill_quads(void)
{
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	const u32 W = H713_DISP_OSD_WIDTH, H = H713_DISP_OSD_HEIGHT;
	u32 x, y;

	for (y = 0; y < H; y++) {
		for (x = 0; x < W; x++) {
			u32 c;

			if (y < H / 2)
				c = (x < W / 2) ? 0xffff0000 : 0xff00ff00;
			else
				c = (x < W / 2) ? 0xff0000ff : 0xffffff00;
			fb[y * W + x] = c;
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: four solid quadrants published at 0x%08lx -- "
	       "red=TL green=TR blue=BL yellow=BR, each %ux%u\n",
	       H713_DISP_OSD_FB_ADDR, W / 2, H / 2);
}

/*
 * A framebuffer pattern that cannot hide an addressing error on either axis.
 *
 * fb-vprobe's eight horizontal bands were claimed to prove the framebuffer path.
 * They do not. Every row of a band is uniform across the full width, so a
 * horizontal stride error leaves the bands looking perfectly correct -- the same
 * structural blindness as scoring a checkerboard with a column-only metric, or
 * judging a logo by a row-luminance profile. Three instruments in this
 * investigation have now reported on an axis they could not see.
 *
 * The vendor logo shows what the bands missed: a 545x36 text block centred at
 * (368,343), verified byte-exact in the framebuffer by fbcheck, projects as a
 * streak spanning nearly the full width and only a few rows tall.
 *
 * This pattern is sensitive to everything the bands were not:
 *
 *   2 px white border      any scale or crop error moves or loses an edge
 *   both diagonals         a stride error changes their slope; they are the
 *                          single most sensitive feature to row pitch
 *   centre crosshair       shows translation
 *   corner squares         red TL, green TR, blue BL, yellow BR -- names any
 *                          flip, rotation or mirror unambiguously
 *   1 px grid every 128    a repeating reference for measuring scale directly
 *
 * Diagonals matter most. If the hardware reads with a row pitch different from
 * the 5120 bytes written, a corner-to-corner diagonal comes back at a visibly
 * different angle, and the angle gives the true pitch.
 */
static void h713_disp_fill_grid(void)
{
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	const u32 W = H713_DISP_OSD_WIDTH, H = H713_DISP_OSD_HEIGHT;
	u32 x, y;

	for (y = 0; y < H; y++)
		for (x = 0; x < W; x++)
			fb[y * W + x] = 0xff000000;

	for (y = 0; y < H; y++) {
		for (x = 0; x < W; x++) {
			u32 *p = &fb[y * W + x];

			if (x < 2 || x >= W - 2 || y < 2 || y >= H - 2)
				*p = 0xffffffff;			/* border    */
			else if (!(x % 128) || !(y % 128))
				*p = 0xff404040;			/* grid      */
			if (x * H / W == y)
				*p = 0xff00ffff;			/* diag TL-BR*/
			if ((W - 1 - x) * H / W == y)
				*p = 0xffff00ff;			/* diag TR-BL*/
			if ((x >= W / 2 - 60 && x < W / 2 + 60 && y >= H / 2 - 1 &&
			     y < H / 2 + 1) ||
			    (y >= H / 2 - 60 && y < H / 2 + 60 && x >= W / 2 - 1 &&
			     x < W / 2 + 1))
				*p = 0xffffffff;			/* crosshair */
			if (x < 80 && y < 80)
				*p = 0xffff0000;			/* TL red    */
			if (x >= W - 80 && y < 80)
				*p = 0xff00ff00;			/* TR green  */
			if (x < 80 && y >= H - 80)
				*p = 0xff0000ff;			/* BL blue   */
			if (x >= W - 80 && y >= H - 80)
				*p = 0xffffff00;			/* BR yellow */
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: geometry grid published at 0x%08lx -- 2px border, "
	       "both diagonals, centre crosshair, 128px grid, corners "
	       "red=TL green=TR blue=BL yellow=BR\n",
	       H713_DISP_OSD_FB_ADDR);
}

/*
 * Read the framebuffer back and report where the non-black content actually
 * landed, so the BMP-to-framebuffer conversion can be checked without the
 * optical path in the way.
 *
 * fb-vprobe proved the framebuffer path itself: eight synthetic bands render in
 * the right order at full height. So if a converted image looks wrong, the fault
 * is between the file and the framebuffer, not downstream of it. Photographs
 * cannot settle that -- a projection shot at an angle is keystoned, and an
 * axis-aligned measurement of a trapezoid produces margins that look like shear
 * whether or not any exists.
 *
 * For bootlogo.bmp the expected answer is exact and known from the file:
 * bright content confined to rows 343..378 and columns 368..912 of 1280x720,
 * with equal margins. Any row outside that range holding content means the
 * conversion is wrong -- a stride mismatch shears, a wrong row order flips, a
 * wrong pixel stride stretches -- and the per-row extents say which.
 */
static void h713_disp_verify_fb(uint expect_y0, uint expect_y1,
				uint expect_x0, uint expect_x1, bool red_only)
{
	const u32 *fb = (const u32 *)H713_DISP_OSD_FB_ADDR;
	uint y, x, bad_rows = 0;
	uint first_y = H713_DISP_OSD_HEIGHT, last_y = 0;
	uint min_x = H713_DISP_OSD_WIDTH, max_x = 0;

	printf("H713 fbcheck: scanning %ux%u at 0x%08lx; expecting content in "
	       "rows %u..%u, columns %u..%u\n",
	       H713_DISP_OSD_WIDTH, H713_DISP_OSD_HEIGHT,
	       H713_DISP_OSD_FB_ADDR, expect_y0, expect_y1,
	       expect_x0, expect_x1);

	for (y = 0; y < H713_DISP_OSD_HEIGHT; y++) {
		const u32 *row = fb + y * H713_DISP_OSD_WIDTH;
		uint rx0 = H713_DISP_OSD_WIDTH, rx1 = 0, n = 0;

		for (x = 0; x < H713_DISP_OSD_WIDTH; x++) {
			u32 p = row[x];
			uint r = (p >> 16) & 0xff, g = (p >> 8) & 0xff;
			uint b = p & 0xff;

			if (red_only ? (r > 60 && b <= 60) :
				       (r > 60 || g > 60 || b > 60)) {
				if (x < rx0)
					rx0 = x;
				if (x > rx1)
					rx1 = x;
				n++;
			}
		}
		if (!n)
			continue;

		if (y < first_y)
			first_y = y;
		last_y = y;
		if (rx0 < min_x)
			min_x = rx0;
		if (rx1 > max_x)
			max_x = rx1;

		if (y < expect_y0 || y > expect_y1) {
			if (bad_rows < 8)
				printf("  row %3u OUTSIDE expected range: %u px, "
				       "x %u..%u\n", y, n, rx0, rx1);
			bad_rows++;
		} else if ((y - expect_y0) % 8 == 0) {
			printf("  row %3u  %4u px  x %u..%u\n", y, n, rx0, rx1);
		}
	}

	printf("H713 fbcheck: content rows %u..%u (expected %u..%u), "
	       "columns %u..%u (expected %u..%u)\n",
	       first_y, last_y, expect_y0, expect_y1,
	       min_x, max_x, expect_x0, expect_x1);
	if (first_y > last_y)
		printf("H713 fbcheck: NO MATCHING PIXEL ANYWHERE. The bounds "
		       "above are the untouched initial values, not a "
		       "measurement. Whatever the framebuffer holds, it is not "
		       "what this check was told to look for -- fix that before "
		       "reading anything into the panel.\n");
	else if (bad_rows)
		printf("H713 fbcheck: %u row(s) outside the expected range -- the "
		       "conversion is wrong, not the display\n", bad_rows);
	else if (first_y == expect_y0 && last_y == expect_y1 &&
		 min_x == expect_x0 && max_x == expect_x1)
		printf("H713 fbcheck: EXACT match; the framebuffer holds the "
		       "image the file describes\n");
	else
		printf("H713 fbcheck: within range but not exact; compare the "
		       "bounds above against the file\n");
}

/*
 * Eight horizontal bands, 90 rows each, in eight distinct hues.
 *
 * The existing fill_pattern draws vertical bars, which probe horizontal
 * addressing. With the link now correct -- the TCON generator renders a clean
 * checkerboard -- the remaining fault is that a framebuffer image collapses
 * vertically: the whole 1280x720 vendor logo arrives as two thin lines of
 * glyph-like structure on a field that should be black.
 *
 * Horizontal bands discriminate the causes, which vertical bars cannot:
 *
 *   all eight bands, in order, full height  -> vertical addressing is fine and
 *                                              the logo fault is elsewhere
 *   bands squeezed into a thin strip        -> vertical scale is wrong by a
 *                                              large factor
 *   one or two hues filling the frame       -> only a few source rows are read
 *                                              and repeated
 *   bands out of order or interleaved       -> stride or tiling mismatch, which
 *                                              is what feeding linear ARGB to a
 *                                              decoder expecting a tiled or
 *                                              compressed layout would look like
 *
 * Hues rather than greys, because the optical path answers chroma far more
 * reliably than luminance.
 */
static void h713_disp_fill_hbands(void)
{
	static const u32 hues[] = {
		0xffff0000,	/* red     */
		0xffff8000,	/* orange  */
		0xffffff00,	/* yellow  */
		0xff00ff00,	/* green   */
		0xff00ffff,	/* cyan    */
		0xff0000ff,	/* blue    */
		0xff8000ff,	/* purple  */
		0xffff00ff,	/* magenta */
	};
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	uint band_h = H713_DISP_OSD_HEIGHT / ARRAY_SIZE(hues);
	uint x, y;

	for (y = 0; y < H713_DISP_OSD_HEIGHT; y++) {
		uint band = y / band_h;
		u32 colour = hues[band < ARRAY_SIZE(hues) ?
				  band : ARRAY_SIZE(hues) - 1];

		for (x = 0; x < H713_DISP_OSD_WIDTH; x++)
			*fb++ = colour;
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: %u horizontal bands of %u rows published at "
	       "0x%08lx (red orange yellow green cyan blue purple magenta, "
	       "top to bottom)\n", (uint)ARRAY_SIZE(hues), band_h,
	       H713_DISP_OSD_FB_ADDR);
}

static void h713_disp_fill_pattern(uint phase)
{
	static const u32 colours[] = {
		0xffff0000,	/* red */
		0xff00ff00,	/* green */
		0xff0000ff,	/* blue */
		0xffffffff,	/* white */
		0xff00ffff,	/* cyan */
		0xffff00ff,	/* magenta */
		0xffffff00,	/* yellow */
		0xff000000,	/* black */
	};
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	uint bar, x, y;

	for (y = 0; y < H713_DISP_OSD_HEIGHT; y++) {
		for (bar = 0; bar < ARRAY_SIZE(colours); bar++) {
			u32 colour = colours[(bar + phase) %
					     ARRAY_SIZE(colours)];

			for (x = 0; x < H713_DISP_OSD_WIDTH /
				     ARRAY_SIZE(colours); x++)
				*fb++ = colour;
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: pattern %u published at 0x%08lx "
	       "(%ux%u ARGB8888, stride 0x%x)\n",
	       phase, H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_WIDTH,
	       H713_DISP_OSD_HEIGHT, (uint)H713_DISP_OSD_STRIDE);
}

/*
 * Reproduce the Board-B stock fastlogo pixel path, independently of U-Boot's
 * video uclass. The source is bootlogo.bmp from this board's own bootloader
 * FAT partition, not a generated or board-A substitute. Its stock identity is
 * one row of h713_vendor_bootlogos[], from the 2026-07-05 full-board dump.
 *
 * Stock's 24-bit blitter reads BMP B,G,R bytes, supplies alpha 0xff and writes
 * one little-endian 0xffRRGGBB word per destination pixel. Positive-height
 * BMPs are bottom-up. Keeping that conversion literal makes this a control
 * for both the test-pattern contents and the presumed framebuffer byte order.
 */
/*
 * Convert a 24-bit BMP at VENDOR_BMP_ADDR, in the panel's own geometry, into
 * the OSD framebuffer.
 *
 * The file is not hashed here. The vendor table h713_vendor_bootlogos[] is
 * identification only, and on the product boot path even that costs: SHA-256
 * over the HY310's 6.2 MB logo takes about 105 ms, and "init <id> logo"
 * publishes twice (before the firmware starts and after it is ready), so a
 * boot paid 0.2 s to print a board name (measured 15.09.2026,
 * umbau/test-20260915/boottime-dev2-193348.log). Stock hashes nothing. The
 * one caller that needs the identity, panel-test vendor-logo, asks
 * h713_disp_bmp_is_vendor() itself, and only then are the bytes hashed.
 *
 * What protects the framebuffer is the BMP *format* check, and that runs
 * either way: the panel's own width and height, one plane, 24 bpp,
 * uncompressed. An image that is not exactly that is refused, not mangled.
 */
/*
 * Length of the BMP last read into H713_DISP_VENDOR_BMP_ADDR. A caller that
 * publishes again without loading (init ... logo, after the firmware is up)
 * must hash the bytes that were read, not a fixed size: the fixed size is one
 * board's asset, and hashing the HY310's 6.2 MB logo over it refused the
 * vendor's own file on the second publish (device run 2, 15.09.2026).
 */
static loff_t h713_disp_bmp_loaded_len;

/*
 * Whether the BMP last read into H713_DISP_VENDOR_BMP_ADDR is one row of
 * h713_vendor_bootlogos[]. Hashes the bytes that were read, on demand: only
 * panel-test vendor-logo asks, because that mode is a control whose
 * framebuffer bounds are measured from the stock file. No size check: the
 * digest is the identity, and each board's asset is its own panel's size.
 */
static bool h713_disp_bmp_is_vendor(void)
{
	u8 digest[SHA256_SUM_LEN];
	loff_t len = h713_disp_bmp_loaded_len ? h713_disp_bmp_loaded_len :
					       H713_DISP_VENDOR_BMP_SIZE;
	uint i;

	sha256_csum_wd((const u8 *)H713_DISP_VENDOR_BMP_ADDR, len, digest,
		       CHUNKSZ_SHA256);
	printf("H713 panel: bootlogo.bmp SHA-256 ");
	h713_mips_print_digest(digest);
	printf("\n");
	for (i = 0; i < ARRAY_SIZE(h713_vendor_bootlogos); i++)
		if (!memcmp(digest, h713_vendor_bootlogos[i].digest,
			    SHA256_SUM_LEN)) {
			printf("H713 panel: stock logo recognised (%s)\n",
			       h713_vendor_bootlogos[i].board);
			return true;
		}
	printf("H713 panel: logo not in the vendor table\n");
	return false;
}

static int h713_disp_publish_bmp(bool load, const char *path, bool chroma)
{
	struct bmp_header *hdr = (struct bmp_header *)H713_DISP_VENDOR_BMP_ADDR;
	u8 *pixels;
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	loff_t len = h713_disp_bmp_loaded_len ? h713_disp_bmp_loaded_len :
					       H713_DISP_VENDOR_BMP_SIZE;
	u32 data_offset, compression, row_bytes;
	s32 width, height;
	u16 planes, bpp;
	uint x, y;
	int ret;

	if (load) {
		/*
		 * The logo lives where the display artifacts live: the stock
		 * FAT on a stock device, hy310-boot on an installed one. The
		 * fixed "1:2" this used to name is the raw U-Boot partition on
		 * our layout, which has no filesystem at all.
		 */
		const char *dev = h713_disp_fs_dev();

		ret = fs_set_blk_dev(H713_DISP_FS_IF, dev, FS_TYPE_ANY);
		if (ret) {
			printf("H713 panel: cannot select %s %s for %s\n",
			       H713_DISP_FS_IF, dev, path);
			return ret;
		}
		ret = fs_read(path, H713_DISP_VENDOR_BMP_ADDR, 0,
			      H713_DISP_VENDOR_BMP_MAX, &len);
		if (ret) {
			printf("H713 panel: cannot read %s\n", path);
			return ret;
		}
		printf("  %-28s -> 0x%08lx  %llu bytes\n", path,
		       H713_DISP_VENDOR_BMP_ADDR, len);
		h713_disp_bmp_loaded_len = len;
	}

	if (hdr->signature[0] != 'B' || hdr->signature[1] != 'M') {
		printf("H713 panel: %s has no BMP signature\n", path);
		return -EINVAL;
	}
	data_offset = get_unaligned_le32(&hdr->data_offset);
	width = (s32)get_unaligned_le32(&hdr->width);
	height = (s32)get_unaligned_le32(&hdr->height);
	planes = get_unaligned_le16(&hdr->planes);
	bpp = get_unaligned_le16(&hdr->bit_count);
	compression = get_unaligned_le32(&hdr->compression);
	if (width != H713_DISP_OSD_WIDTH ||
	    (height != H713_DISP_OSD_HEIGHT &&
	     height != -H713_DISP_OSD_HEIGHT) ||
	    planes != 1 || bpp != 24 || compression != BMP_BI_RGB) {
		printf("H713 panel: %s: unsupported BMP layout %dx%d, %u "
		       "plane(s), %u bpp, compression %u -- need 1 plane, "
		       "24 bpp, uncompressed, %ux%u\n",
		       path, width, height, planes, bpp, compression,
		       H713_DISP_OSD_WIDTH, H713_DISP_OSD_HEIGHT);
		return -EINVAL;
	}

	row_bytes = ALIGN(H713_DISP_OSD_WIDTH * 3, BMP_DATA_ALIGN);
	if (data_offset > len ||
	    (u64)row_bytes * H713_DISP_OSD_HEIGHT > len - data_offset) {
		printf("H713 panel: %s pixel array is truncated\n", path);
		return -EINVAL;
	}
	pixels = (u8 *)H713_DISP_VENDOR_BMP_ADDR + data_offset;

	for (y = 0; y < H713_DISP_OSD_HEIGHT; y++) {
		u8 *src = pixels + (height > 0 ?
			(H713_DISP_OSD_HEIGHT - 1 - y) : y) * row_bytes;

		for (x = 0; x < H713_DISP_OSD_WIDTH; x++) {
			u8 *p = src + x * 3;

			/*
			 * The stock logo is 99% black and its lit pixels are
			 * pure grey -- chroma |R-B| is exactly 0 across the
			 * whole file. This optical path normalises luminance
			 * away, so as shipped it photographs as an unlit panel
			 * whether the framebuffer path works or not. Keep the
			 * geometry, replace the palette: lit -> red, unlit ->
			 * blue. Then a shear or an offset is visible.
			 */
			if (chroma)
				*fb++ = (p[0] + p[1] + p[2] > 96) ?
					0xffff0000 : 0xff0000ff;
			else
				*fb++ = 0xff000000 | (p[2] << 16) |
					(p[1] << 8) | p[0];
		}
	}

	flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	printf("H713 panel: %s published at 0x%08lx (24-bit BMP -> %s)\n",
	       path, H713_DISP_OSD_FB_ADDR,
	       chroma ? "red where lit, blue where not" : "0xffRRGGBB");
	return 0;
}

/* The stock asset by name. Thin wrapper, so callers read well. */
static int h713_disp_publish_vendor_bootlogo(bool load, bool chroma)
{
	return h713_disp_publish_bmp(load, "bootlogo.bmp", chroma);
}

/*
 * The authenticated display.bin implements SetHWBlueScreenColorPanel at raw
 * +0x17a68. It writes the same three 10-bit components to panel registers
 * +0xb0 and +0xb4. EnableHWBlueScreenPanel at raw +0x16600 then sets
 * +0xb8[6:0] to 0x7f; its disable peer clears +0xb8[5:0].
 *
 * Reproduce that firmware-owned test source only after application readiness,
 * then restore all three registers exactly. This bypasses the OSD framebuffer,
 * AFBD and DE, giving a software-only split between scanout and the
 * panel/LVDS side of the pipeline.
 */
static u32 h713_disp_panel_blue_colour(u32 c0, u32 c1, u32 c2)
{
	return ((c0 & 0x3ff) << 20) |
	       ((c2 & 0x3ff) << 10) |
	       (c1 & 0x3ff);
}

static void h713_disp_panel_blue_test(void)
{
	static const u16 colours[][3] = {
		{ 0x3ff, 0x000, 0x000 },
		{ 0x000, 0x3ff, 0x000 },
		{ 0x000, 0x000, 0x3ff },
		{ 0x3ff, 0x3ff, 0x3ff },
	};
	u32 saved_rgb0 = readl(H713_DISP_PANEL_BLUE_RGB0);
	u32 saved_rgb1 = readl(H713_DISP_PANEL_BLUE_RGB1);
	u32 saved_ctrl = readl(H713_DISP_PANEL_BLUE_CTRL);
	uint i;

	printf("H713 panel: phase 1, 720p firmware hardware-blue-screen source, "
	       "%u colours x %u ms\n",
	       (uint)ARRAY_SIZE(colours), H713_DISP_OSD_DWELL_MS);
	printf("H713 panel: blue-screen baseline: %08x %08x %08x\n",
	       saved_rgb0, saved_rgb1, saved_ctrl);

	for (i = 0; i < ARRAY_SIZE(colours); i++) {
		u32 colour = h713_disp_panel_blue_colour(colours[i][0],
							 colours[i][1],
							 colours[i][2]);

		writel(colour, H713_DISP_PANEL_BLUE_RGB0);
		writel(colour, H713_DISP_PANEL_BLUE_RGB1);
		writel((readl(H713_DISP_PANEL_BLUE_CTRL) & ~0x7f) | 0x7f,
		       H713_DISP_PANEL_BLUE_CTRL);
		printf("H713 panel: hardware colour %u/4: %08x %08x %08x\n",
		       i + 1, readl(H713_DISP_PANEL_BLUE_RGB0),
		       readl(H713_DISP_PANEL_BLUE_RGB1),
		       readl(H713_DISP_PANEL_BLUE_CTRL));
		mdelay(H713_DISP_OSD_DWELL_MS);
	}

	writel(saved_rgb0, H713_DISP_PANEL_BLUE_RGB0);
	writel(saved_rgb1, H713_DISP_PANEL_BLUE_RGB1);
	writel(saved_ctrl, H713_DISP_PANEL_BLUE_CTRL);
	printf("H713 panel: blue-screen registers restored: %08x %08x %08x\n",
	       readl(H713_DISP_PANEL_BLUE_RGB0),
	       readl(H713_DISP_PANEL_BLUE_RGB1),
	       readl(H713_DISP_PANEL_BLUE_CTRL));
}

static void h713_disp_print_panel_gpio(const char *phase)
{
	u32 pb = readl(H713_PB_DATA);
	u32 pf = readl(H713_PF_DATA);
	u32 ph = readl(H713_PH_DATA);

	printf("H713 panel: %s: PB4 cfg=%x latch=%d, "
	       "PB5 cfg=%x latch=%d; PH16 cfg=%x latch=%d, "
	       "PF6 cfg=%x latch=%d\n",
	       phase,
	       sunxi_gpio_get_cfgpin(SUNXI_GPB(4)), !!(pb & BIT(4)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPB(5)), !!(pb & BIT(5)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPH(16)), !!(ph & BIT(16)),
	       sunxi_gpio_get_cfgpin(SUNXI_GPF(6)), !!(pf & BIT(6)));
}

/*
 * Exercise only controls identified by the stock DT, with the internal white
 * source active and the panel timing already at 1280x720.
 *
 * PB5 is shared by the LED backlight and cooling fan. It is forced high and
 * never toggled. PH16 is the active-high panel reset/enable and PF6 is panel
 * power. Do not exercise PB4 as brightness here: it is PWM2 from the
 * pre-override DT, while board B's panel_config.ini selects PWM5. The PWM5 pin
 * route is not proven.
 *
 * The PF6 power cycle is performed with PH16 asserted low, then the stock
 * low-to-high reset sequence is replayed after power returns. This makes a
 * visible opacity/flash change useful evidence that the named GPIOs reach live
 * panel hardware even if the LVDS image remains absent.
 */
static void h713_disp_panel_control_test(void)
{
	const uint fan_bl = SUNXI_GPB(5);
	const uint reset = SUNXI_GPH(16);
	const uint power = SUNXI_GPF(6);
	u32 saved_rgb0 = readl(H713_DISP_PANEL_BLUE_RGB0);
	u32 saved_rgb1 = readl(H713_DISP_PANEL_BLUE_RGB1);
	u32 saved_ctrl = readl(H713_DISP_PANEL_BLUE_CTRL);
	u32 white = h713_disp_panel_blue_colour(0x3ff, 0x3ff, 0x3ff);

	/* Preserve the fan/backlight safety interlock throughout this test. */
	setbits_le32((void *)H713_PB_DATA, BIT(5));
	sunxi_gpio_set_cfgpin(fan_bl, SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_cfgpin(reset, SUNXI_GPIO_OUTPUT);
	sunxi_gpio_set_cfgpin(power, SUNXI_GPIO_OUTPUT);

	writel(white, H713_DISP_PANEL_BLUE_RGB0);
	writel(white, H713_DISP_PANEL_BLUE_RGB1);
	writel((readl(H713_DISP_PANEL_BLUE_CTRL) & ~0x7f) | 0x7f,
	       H713_DISP_PANEL_BLUE_CTRL);

	printf("H713 panel: phase 2, power/enable readiness test; "
	       "internal white remains selected\n");
	h713_disp_print_panel_gpio("control baseline");
	mdelay(2000);

	printf("H713 panel: PH16 reset/enable LOW for 3 seconds\n");
	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	h713_disp_print_panel_gpio("PH16 low");
	mdelay(3000);

	printf("H713 panel: PH16 reset/enable HIGH for 3 seconds\n");
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	h713_disp_print_panel_gpio("PH16 high");
	mdelay(3000);

	printf("H713 panel: PF6 panel power OFF for 3 seconds; "
	       "PH16 held LOW, PB5 stays HIGH\n");
	clrbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(100);
	clrbits_le32((void *)H713_PF_DATA, BIT(6));
	h713_disp_print_panel_gpio("PF6 low");
	mdelay(3000);

	printf("H713 panel: PF6 panel power ON; replaying stock PH16 release\n");
	setbits_le32((void *)H713_PF_DATA, BIT(6));
	mdelay(2);
	setbits_le32((void *)H713_PH_DATA, BIT(16));
	mdelay(5);
	h713_disp_print_panel_gpio("panel repowered");
	mdelay(5000);

	writel(saved_rgb0, H713_DISP_PANEL_BLUE_RGB0);
	writel(saved_rgb1, H713_DISP_PANEL_BLUE_RGB1);
	writel(saved_ctrl, H713_DISP_PANEL_BLUE_CTRL);
	printf("H713 panel: control test complete; blue-screen registers "
	       "restored: %08x %08x %08x\n",
	       readl(H713_DISP_PANEL_BLUE_RGB0),
	       readl(H713_DISP_PANEL_BLUE_RGB1),
	       readl(H713_DISP_PANEL_BLUE_CTRL));
}

/*
 * Restore the panel timing that project 0x33's timing block 6 programmed
 * before the MIPS replaced it with 1080p, then pulse the same +0x0c latch used
 * by the vendor table.
 *
 * Every word written below comes from the panel row: the TCON's own convention
 * is the raw totals (unlike the mixer's minus-one), and bit 31 of +0x2c/+0x30
 * comes from the ini's PanelHsyncPol/PanelVsyncPol. It used to say it carried
 * "the live bit-31 enables" and in fact hardcoded them set, on a panel whose
 * ini asks for 0/0 and whose own vendor timing record writes them clear
 * (A10 1.2). The low half of +0x30 stays literal because nobody has explained
 * it: [15:0] = 3 where vsync is 2, so it is not a plain sync width.
 */
static void h713_disp_latch_panel_timing(void)
{
	const struct h713_panel_cfg *c = h713_disp_panel;
	const bool board_b = (c == &h713_panel_cfg_board_b);
	u32 live_total  = readl(0x05880020);
	u32 live_active = readl(0x05880024);
	u32 ctl  = readl(0x0588000c);
	u32 mode = readl(0x0588001c);

	printf("H713 panel: firmware left %ux%u active, %ux%u total\n",
	       live_active & 0xffff, live_active >> 16,
	       live_total & 0xffff, live_total >> 16);

	/*
	 * Overwriting the timing is board B's repair: its panel is 1280x720,
	 * the firmware programs 1080p regardless, so the values have to be
	 * forced back afterwards. On a panel that really is 1920x1080 the
	 * firmware's values are the correct ones and board B's would drive a
	 * 720p raster into a 1080p panel.
	 *
	 * The latch pulse below belongs to neither board in particular: it is
	 * what re-commits the TCON once the coprocessor is parked. Skip the
	 * values, never the commit -- returning early from here to avoid the
	 * 720p values takes the latch with it, and the panel goes from a wrong
	 * picture to no picture at all.
	 */
	if (board_b) {
		writel((mode & ~0x7) | 0x4, 0x0588001c);
		/* Raw totals here, not the mixer's total minus one. */
		writel((c->vtotal << 16) | c->htotal, 0x05880020);
		writel((c->height << 16) | c->width, 0x05880024);
		writel((c->vbp << 16) | c->hbp, 0x05880028);
		writel((c->hsync_pol ? BIT(31) : 0) | c->hsync, 0x0588002c);
		writel((c->vsync_pol ? BIT(31) : 0) | 0x00010003, 0x05880030);
	} else {
		printf("H713 panel: TCON 1c=%08x 20=%08x 24=%08x 28=%08x "
		       "2c=%08x 30=%08x; mixer %08x (these differ by design)\n",
		       mode, live_total, live_active, readl(0x05880028),
		       readl(0x0588002c), readl(0x05880030),
		       readl(0x0525c000));
	}

	writel(ctl | BIT(0), 0x0588000c);
	udelay(1);
	writel(ctl & ~BIT(0), 0x0588000c);

	/*
	 * The second half of the line is what the next owner's log needs so the
	 * two rasters can be compared without a register dump: the polarities
	 * the row asks for, the raw totals, and what the minus-one convention
	 * makes of them in the mixer and DE records -- read back, not assumed.
	 */
	printf("H713 panel: timing latched: %08x %08x %08x %08x %08x %08x; "
	       "sync pol h%u v%u; DE totals %ux%u written as %ux%u "
	       "(mixer %08x de %08x)\n",
	       readl(0x0588001c), readl(0x05880020), readl(0x05880024),
	       readl(0x05880028), readl(0x0588002c), readl(0x05880030),
	       c->hsync_pol, c->vsync_pol, c->htotal, c->vtotal,
	       c->htotal - 1, c->vtotal - 1,
	       readl(0x0525c000), readl(0x0524c010));
}

/*
 * Remove the firmware as a live display owner without resetting or gating any
 * display block.  h713_mips_stop() is intentionally not used here: it also
 * disables the MIPS module clock and clears the display-prepared flag, which
 * would turn this compositor diagnostic into another cold/gated run.
 *
 * The core reset is asserted only after application readiness has proved that
 * the firmware completed its display initialization.  Most callers then
 * re-latch the panel timing and replay the authenticated DE block; the native
 * timing diagnostic deliberately preserves the firmware's final timing.  Two
 * scan samples report whether the hardware raster survived the ownership
 * handoff; no visual conclusion is valid if both samples are zero or equal.
 */
static void h713_disp_quiesce_mips_owner(void)
{
	u32 before = readl(H713_DISP_LVDS_SCAN_REG);
	u32 after_reset;
	u32 after_wait;

	writel(H713_MIPS_RESET_ASSERTED, H713_MIPS_RESET_REG);
	dmb();
	mdelay(20);
	after_reset = readl(H713_DISP_LVDS_SCAN_REG);
	mdelay(20);
	after_wait = readl(H713_DISP_LVDS_SCAN_REG);

	printf("H713 panel: MIPS core quiesced, display clocks retained; "
	       "reset=%08x status=%08x scan=%08x->%08x->%08x\n",
	       readl(H713_MIPS_RESET_REG), readl(H713_MIPS_STATUS_REG),
	       before, after_reset, after_wait);
	if (!after_reset || after_reset == after_wait)
		printf("H713 panel: WARNING: raster did not prove live after MIPS "
		       "quiesce; visual result is not a compositor verdict\n");
}

/*
 * Publish a newly written OSD frame using the exact order recovered from the
 * stock H713 ge2d_dev.ko.  tgd_put_plane_info() first sets bit 0 in the
 * selected AFBD channel control at +0x00. osd_ready_for_update() then writes
 * literal 1 to the channel ready register at +0x04. Its read-only lookup table
 * contains { 0x05600100, 0x05600140 }, proving that the second write is AFBD
 * channel 1 +0x04 -- not OSD channel 1 +0x00.
 *
 * The earlier related-platform inference that 0x0524c000[0] was a second
 * frame-ready bit was wrong. Do not mutate that word during submission. The
 * AFBD ready bit may self-clear after the raster accepts it, so the readback
 * is diagnostic rather than an error check.
 */
struct h713_disp_commit_stat {
	u32  pending, cleared, done;
	uint wait_us;
	bool completed;		/* BIT(1) observed before the timeout */
};

/*
 * The silent half, so a caller committing hundreds of frames is not really
 * measuring the console. One printf of the line below is ~130 bytes, which at
 * 115200 baud is ~11 ms -- comparable to a whole frame at 59.71 Hz, so printing
 * per commit would dominate exactly the timing an animation is trying to
 * measure. h713_disp_anim_run() reports aggregates instead.
 */
static void h713_disp_commit_osd_frame_quiet(struct h713_disp_commit_stat *st)
{
	u32 ctrl = readl(H713_DISP_AFBD_CTRL_REG);
	u32 pending = readl(H713_DISP_AFBD_STATUS_REG);
	u32 done;
	uint wait_us;

	/*
	 * +0x168 is channel 1's write-one-to-clear IRQ status.  Stock's
	 * osd_afbd_irq handler writes every non-zero status back before it
	 * submits another frame; without an ARM IRQ handler the old completion
	 * otherwise remains pending forever.
	 */
	if (pending)
		writel(pending, H713_DISP_AFBD_STATUS_REG);
	st->pending = pending;
	st->cleared = readl(H713_DISP_AFBD_STATUS_REG);

	writel(ctrl | BIT(0), H713_DISP_AFBD_CTRL_REG);
	writel(1, H713_DISP_AFBD_READY_REG);

	/* Bit 1 is the completion bit checked by the stock AFBD hard IRQ. */
	for (wait_us = 0; wait_us < 50000; wait_us += 100) {
		done = readl(H713_DISP_AFBD_STATUS_REG);
		if (done & BIT(1))
			break;
		udelay(100);
	}
	st->done = readl(H713_DISP_AFBD_STATUS_REG);
	st->wait_us = wait_us;
	st->completed = !!(st->done & BIT(1));
}

static void h713_disp_commit_osd_frame(void)
{
	struct h713_disp_commit_stat st;

	h713_disp_commit_osd_frame_quiet(&st);

	printf("H713 panel: frame commit irq=%08x->%08x->%08x wait=%uus "
	       "AFBD-ctrl=%08x ready=%08x OSD=%08x scan=%08x\n",
	       st.pending, st.cleared, st.done, st.wait_us,
	       readl(H713_DISP_AFBD_CTRL_REG),
	       readl(H713_DISP_AFBD_READY_REG),
	       readl(H713_DISP_OSD_CTRL_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
}

/*
 * Isolate the selected OSD plane from the final mix without changing its
 * geometry, format, address or routing. The stock H713 ge2d_dev.ko initializes
 * a disabled OSD plane with bit 31 set in its channel control word; its active
 * update path changes that same bit, and board B's authenticated DE table
 * leaves 0x0524c000 at 0x00fc0202 with bit 31 clear.
 *
 * Run this only after the MIPS owner has been quiesced. Hold the stock disable
 * state long enough to observe, then restore the complete board-B word. Each
 * change is followed by the stock AFBD submission sequence so the comparison
 * crosses a frame boundary. If the panel changes only while bit 31 is set,
 * channel 1 reaches the final mix and the remaining fault is at or before its
 * AFBD pixels. No visible change means the final mixer is not using this plane.
 */
static void h713_disp_plane_gate_test(void)
{
	u32 saved = readl(H713_DISP_OSD_CTRL_REG);

	printf("H713 panel: selected-plane gate test; disable for 5 seconds, "
	       "then restore for 5 seconds\n");
	writel(saved | BIT(31), H713_DISP_OSD_CTRL_REG);
	h713_disp_commit_osd_frame();
	printf("H713 panel: plane DISABLED OSD=%08x\n",
	       readl(H713_DISP_OSD_CTRL_REG));
	mdelay(5000);

	writel(saved, H713_DISP_OSD_CTRL_REG);
	h713_disp_commit_osd_frame();
	printf("H713 panel: plane RESTORED OSD=%08x\n",
	       readl(H713_DISP_OSD_CTRL_REG));
	mdelay(5000);
}

/*
 * Split the selected OSD plane from its AFBD input without changing either
 * plane geometry or the AFBD format/address.
 * Stock tgd_put_plane_info() sets AFBD channel-control bit 0 immediately
 * before writing ready=1, and the stock disabled channel value has bit 0
 * clear. Therefore bit 0 is the narrow, reversible channel gate.
 *
 * Do not call h713_disp_commit_osd_frame() while disabled because that helper
 * deliberately re-enables the channel. Restore the complete saved control
 * word and submit one stock-ordered frame afterwards. A visible AFBD-gate
 * transition proves the OSD plane consumes this decoder's output; no change
 * after the OSD-plane gate did change localizes the missing link between AFBD
 * and OSD rather than in the final mixer.
 */
static void h713_disp_afbd_gate_test(void)
{
	u32 saved = readl(H713_DISP_AFBD_CTRL_REG);
	u32 pending = readl(H713_DISP_AFBD_STATUS_REG);

	if (pending)
		writel(pending, H713_DISP_AFBD_STATUS_REG);

	printf("H713 panel: AFBD channel-1 gate test; disable for 5 seconds, "
	       "then restore for 5 seconds\n");
	writel(saved & ~BIT(0), H713_DISP_AFBD_CTRL_REG);
	mdelay(50);
	printf("H713 panel: AFBD DISABLED ctrl=%08x ready=%08x status=%08x "
	       "scan=%08x\n",
	       readl(H713_DISP_AFBD_CTRL_REG),
	       readl(H713_DISP_AFBD_READY_REG),
	       readl(H713_DISP_AFBD_STATUS_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);

	writel(saved, H713_DISP_AFBD_CTRL_REG);
	h713_disp_commit_osd_frame();
	printf("H713 panel: AFBD RESTORED ctrl=%08x ready=%08x status=%08x\n",
	       readl(H713_DISP_AFBD_CTRL_REG),
	       readl(H713_DISP_AFBD_READY_REG),
	       readl(H713_DISP_AFBD_STATUS_REG));
	mdelay(5000);
}

/*
 * Board B's own ge2d_dev.ko is authoritative for this gate. Its
 * tgd_is_plane_open() selects 0x0524c000 for plane 1, reads base + 0x1c and
 * returns bit 0. The earlier board-A inference that 0x0524c000[31] was the
 * plane gate was wrong; toggling it did not exercise stock visibility.
 *
 * Publish one known frame first, then clear exactly the stock plane-open bit
 * across a fresh frame boundary. Restore the complete saved word afterwards.
 * Do not repeat the old +0x00 gate or the already-proven AFBD-enable gate.
 */
static void h713_disp_boardb_plane_gate_test(void)
{
	u32 saved = readl(H713_DISP_OSD_OPEN_REG);

	printf("H713 panel: STATIC BARS for exact Board-B plane-open gate; "
	       "baseline 5 seconds\n");
	h713_disp_fill_pattern(0);
	h713_disp_commit_osd_frame();
	printf("H713 panel: BOARD-B PLANE BASELINE open=%08x scan=%08x\n",
	       readl(H713_DISP_OSD_OPEN_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	if (!(saved & BIT(0)))
		printf("H713 panel: WARNING: Board-B driver already considers "
		       "the selected plane closed\n");
	mdelay(5000);

	writel(saved & ~BIT(0), H713_DISP_OSD_OPEN_REG);
	dmb();
	h713_disp_commit_osd_frame();
	printf("H713 panel: BOARD-B PLANE CLOSED open=%08x scan=%08x; "
	       "holding 5 seconds\n",
	       readl(H713_DISP_OSD_OPEN_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);

	writel(saved, H713_DISP_OSD_OPEN_REG);
	dmb();
	h713_disp_commit_osd_frame();
	printf("H713 panel: BOARD-B PLANE RESTORED open=%08x scan=%08x; "
	       "holding 5 seconds\n",
	       readl(H713_DISP_OSD_OPEN_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);
}

/*
 * Reproduce the style-selection tail of tgd_set_checkboard_style() from Board
 * B's own unstripped ge2d_dev.ko. This is the stock driver's TCON-local checker
 * generator, so it bypasses the framebuffer, OSD, AFBD, DE and vblender. The
 * function's preceding writes merely derive the TCON timing words from cached
 * panel geometry; the caller has just latched those authenticated 720p words.
 * The literal tail selects TCON mode 5, enables control bit 3, chooses 128x128
 * cells, and supplies the driver's two packed colour words.
 *
 * Save and restore every touched word exactly. Run only with the MIPS core
 * quiesced; otherwise firmware display ownership could rewrite these registers
 * during the observation interval.
 */
/*
 * The generator's colour words are three 10-bit components, R at [29:20],
 * G at [19:10], B at [9:0]. Stock style 3 writes 0x3fffffff for white, which
 * fixes all three field widths; the top two bits of the word are unimplemented
 * and read back clear.
 */
#define H713_TCON_RGB_RED	0x3ff00000
#define H713_TCON_RGB_GREEN	0x000ffc00
#define H713_TCON_RGB_BLUE	0x000003ff

/* Stock's own cell size, kept fixed so only the colour words vary. */
#define H713_TCON_CELL_STOCK	0x00800080

static void h713_disp_tcon_pattern_apply(u32 ctrl, u32 size, u32 rgb0, u32 rgb1)
{
	/* Preserve the exact store order in Board B's stock function. */
	writel(5, H713_DISP_TCON_MODE_REG);
	writel(ctrl | BIT(3), H713_DISP_TCON_CTRL_REG);
	writel(size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	dmb();
}

static void h713_disp_boardb_tcon_style_apply(u32 ctrl, uint style)
{
	u32 size;
	u32 rgb0;
	u32 rgb1;

	switch (style) {
	case 1:                         /* 128x128 black/white checker */
		size = 0x00800080;
		rgb0 = 0x00000000;
		rgb1 = 0x3fffffff;
		break;
	case 2:                         /* solid black */
		size = 0x00000000;
		rgb0 = 0x00000000;
		rgb1 = 0x00000000;
		break;
	case 3:                         /* solid white */
		size = 0x00000000;
		rgb0 = 0x3fffffff;
		rgb1 = 0x3fffffff;
		break;
	case 8:                         /* 128x128 red/blue checker */
	default:
		size = 0x00800080;
		rgb0 = 0xff000000;
		rgb1 = 0x000000ff;
		break;
	}

	h713_disp_tcon_pattern_apply(ctrl, size, rgb0, rgb1);
}

/*
 * Turn the generator back off without disturbing the two colour words, so a
 * caller can blink it on and off and still restore the exact saved state once
 * at the end.
 */
static void h713_disp_boardb_tcon_generator_off(u32 saved_ctrl, u32 saved_mode)
{
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	dmb();
}

/*
 * Emit a countable optical marker: `blinks` pulses of solid white, then a
 * quiet gap before the phase that follows.
 *
 * The serial labels cannot be seen by a camera pointed at the screen, so a
 * recording cannot be aligned to the phases from the transcript alone. The
 * test_14 video had to be aligned by inference, and that inference is what
 * made its result ambiguous. These markers make each recording self-labelling:
 * count the blinks immediately before a phase to know which phase it is.
 *
 * A marker is only visible when the generator reaches the panel, so it is an
 * alignment aid, not a liveness proof. That job belongs to the positive
 * control below.
 */
#define H713_DISP_MARKER_ON_MS		250
#define H713_DISP_MARKER_OFF_MS		250
#define H713_DISP_MARKER_GAP_MS		1000

static void h713_disp_optical_marker(uint blinks)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);
	uint i;

	printf("H713 panel: MARKER %u blink(s) ->\n", blinks);
	for (i = 0; i < blinks; i++) {
		h713_disp_boardb_tcon_style_apply(saved_ctrl, 3);
		mdelay(H713_DISP_MARKER_ON_MS);
		h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
		mdelay(H713_DISP_MARKER_OFF_MS);
	}

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	dmb();
	mdelay(H713_DISP_MARKER_GAP_MS);
}

/*
 * Prove the panel is still decoding the LVDS pixel stream at this instant.
 *
 * Style 8 is the one generator state with a recorded, operator-confirmed
 * optical response on this hardware: the test_13 run produced three vertical
 * colour bands. It is therefore the available positive control. Without one,
 * a null result cannot distinguish "this phase changed nothing" from "the link
 * was already dead before the phase began" -- exactly the ambiguity that made
 * the test_14 solid comparison uninterpretable.
 *
 * Run this immediately before and after any single-variable change, and treat
 * the whole run as invalid if the pre-change control is already blank.
 */
#define H713_DISP_CONTROL_MS		5000

static void h713_disp_tcon_positive_control(const char *label)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);

	h713_disp_boardb_tcon_style_apply(saved_ctrl, 8);
	printf("H713 panel: POSITIVE CONTROL (%s) style 8 ACTIVE: lane=%08x "
	       "ctrl=%08x mode=%08x rgb=%08x/%08x scan=%08x; expect three "
	       "vertical colour bands, holding %u ms\n",
	       label, readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_TCON_CTRL_REG),
	       readl(H713_DISP_TCON_MODE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG), H713_DISP_CONTROL_MS);
	mdelay(H713_DISP_CONTROL_MS);

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
	printf("H713 panel: POSITIVE CONTROL (%s) restored, scan=%08x\n",
	       label, readl(H713_DISP_LVDS_SCAN_REG));
}

static void h713_disp_boardb_tcon_test(uint style)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);

	printf("H713 panel: exact Board-B TCON %s; holding 15 seconds\n",
	       style == 9 ? "solid black/white styles 2/3" :
	       style == 1 ? "black/white checker style 1" :
			    "red/blue checker style 8");
	printf("H713 panel: TCON checker saved: ctrl=%08x mode=%08x "
	       "size=%08x rgb=%08x/%08x\n",
	       saved_ctrl, saved_mode, saved_size, saved_rgb0, saved_rgb1);

	if (style == 9) {
		h713_disp_boardb_tcon_style_apply(saved_ctrl, 2);
		printf("H713 panel: BOARD-B TCON SOLID BLACK ACTIVE: ctrl=%08x "
		       "mode=%08x size=%08x rgb=%08x/%08x scan=%08x; "
		       "observe now\n",
		       readl(H713_DISP_TCON_CTRL_REG),
		       readl(H713_DISP_TCON_MODE_REG),
		       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS / 2);

		h713_disp_boardb_tcon_style_apply(saved_ctrl, 3);
		printf("H713 panel: BOARD-B TCON SOLID WHITE ACTIVE: ctrl=%08x "
		       "mode=%08x size=%08x rgb=%08x/%08x scan=%08x; "
		       "observe now\n",
		       readl(H713_DISP_TCON_CTRL_REG),
		       readl(H713_DISP_TCON_MODE_REG),
		       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS / 2);
	} else {
		h713_disp_boardb_tcon_style_apply(saved_ctrl, style);
		printf("H713 panel: BOARD-B TCON CHECKER ACTIVE: ctrl=%08x "
		       "mode=%08x size=%08x rgb=%08x/%08x scan=%08x; "
		       "photograph now\n",
		       readl(H713_DISP_TCON_CTRL_REG),
		       readl(H713_DISP_TCON_MODE_REG),
		       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	}

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	dmb();
	printf("H713 panel: BOARD-B TCON CHECKER RESTORED: ctrl=%08x "
	       "mode=%08x size=%08x rgb=%08x/%08x scan=%08x; "
	       "holding 5 seconds\n",
	       readl(H713_DISP_TCON_CTRL_REG),
	       readl(H713_DISP_TCON_MODE_REG),
	       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);
}

/*
 * Hold one spatially uniform generator code while changing only the LVDS
 * sampling-edge selection. Board B's panel_config.ini requests inverted DCLK,
 * and the stock U-Boot patch function maps that setting to 0x05800000[24].
 * The first and final phases explicitly select the stock value so the middle
 * phase has an unambiguous A/B/A comparison even if the saved register ever
 * differs from the expected value.
 */
static void h713_disp_boardb_tcon_dclk_test(void)
{
	u32 saved_lane = readl(H713_DISP_LVDS_LANE_REG);
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);
	u32 stock_lane = saved_lane | BIT(24);
	u32 normal_lane = stock_lane & ~BIT(24);

	printf("H713 panel: DCLK edge A/B/A with fixed TCON all-zero source; "
	       "7.5 seconds for A/B, then 5-second A restore\n");
	printf("H713 panel: saved lane=%08x ctrl=%08x mode=%08x "
	       "size=%08x rgb=%08x/%08x\n",
	       saved_lane, saved_ctrl, saved_mode, saved_size,
	       saved_rgb0, saved_rgb1);

	h713_disp_boardb_tcon_style_apply(saved_ctrl, 2);
	writel(stock_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK A INVERTED ACTIVE: lane=%08x "
	       "rgb=%08x/%08x scan=%08x; observe now\n",
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(H713_DISP_STATIC_FRAME_DWELL_MS / 2);

	writel(normal_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK B NORMAL ACTIVE: lane=%08x "
	       "rgb=%08x/%08x scan=%08x; observe now\n",
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(H713_DISP_STATIC_FRAME_DWELL_MS / 2);

	writel(stock_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK A INVERTED RESTORED: lane=%08x "
	       "rgb=%08x/%08x scan=%08x; holding 5 seconds\n",
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	writel(saved_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK TEST RESTORED: lane=%08x ctrl=%08x "
	       "mode=%08x scan=%08x\n",
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_TCON_CTRL_REG),
	       readl(H713_DISP_TCON_MODE_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
}

/*
 * Chroma diagnostics.
 *
 * Across five generator styles and four bench runs the optical response
 * correlates perfectly with whether a style's two colour words differ in
 * CHROMA, and not at all with luminance or cell size:
 *
 *   style 8  red/blue checker  chroma     visible
 *   style 1  b/w checker       luminance  no change
 *   style 2  solid black       luminance  no change
 *   style 3  solid white       luminance  no change
 *   marker   white blink       luminance  never detected in any video
 *
 * A light engine with dynamic contrast or auto-brightness would behave exactly
 * this way: it normalises a luminance change away and cannot normalise a hue
 * change. Every luminance-based result in this project is therefore suspect,
 * including the solid black/white comparisons and the white blink markers.
 *
 * These diagnostics use only chroma-differing sources. The cell size is pinned
 * to stock's 0x00800080 for the solids so that the sole difference between a
 * solid phase and a checker phase is the pair of colour words.
 */
#define H713_DISP_CHROMA_MARK_MS	300
#define H713_DISP_CHROMA_GAP_MS		800
#define H713_DISP_CHROMA_PHASE_MS	6000

static void h713_disp_chroma_marker(uint blinks)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	uint i;

	printf("H713 panel: CHROMA MARKER %u blink(s) ->\n", blinks);
	for (i = 0; i < blinks; i++) {
		h713_disp_tcon_pattern_apply(saved_ctrl, H713_TCON_CELL_STOCK,
					     H713_TCON_RGB_RED,
					     H713_TCON_RGB_RED);
		mdelay(H713_DISP_CHROMA_MARK_MS);
		h713_disp_tcon_pattern_apply(saved_ctrl, H713_TCON_CELL_STOCK,
					     H713_TCON_RGB_BLUE,
					     H713_TCON_RGB_BLUE);
		mdelay(H713_DISP_CHROMA_MARK_MS);
	}
	h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
	mdelay(H713_DISP_CHROMA_GAP_MS);
}

static void h713_disp_chroma_phase(const char *label, u32 ctrl, u32 size,
				   u32 rgb0, u32 rgb1)
{
	h713_disp_tcon_pattern_apply(ctrl, size, rgb0, rgb1);
	printf("H713 panel: CHROMA %s ACTIVE: size=%08x rgb=%08x/%08x "
	       "lane=%08x scan=%08x; observe now\n", label,
	       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(H713_DISP_CHROMA_PHASE_MS);
}

static void h713_disp_hbp_sweep(void)
{
	u32 saved_porch = readl(0x05880028);
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);
	uint i;

	printf("H713 hbp: 0x05880028 = %08x (hsync %u, back porch %u); sweeping "
	       "the back porch with the checker running\n",
	       saved_porch, saved_porch >> 16, saved_porch & 0xffff);

	for (i = 0; i < ARRAY_SIZE(h713_hbp_sweep); i++) {
		u32 hbp_val = h713_hbp_sweep[i];
		u32 val = (saved_porch & 0xffff0000) | hbp_val;

		writel(val, 0x05880028);
		dmb();
		h713_disp_chroma_marker(i + 1);
		h713_disp_tcon_pattern_apply(saved_ctrl, H713_TCON_CELL_STOCK,
					     H713_TCON_RGB_RED,
					     H713_TCON_RGB_BLUE);
		printf("H713 hbp: step %u, back porch %u (0x05880028=%08x), "
		       "front porch becomes %d; observe the band\n",
		       i + 1, hbp_val, readl(0x05880028),
		       /* htotal is the raw total now, so no +1 here. */
		       (int)h713_disp_panel->htotal -
		       (int)h713_disp_panel->width -
		       (int)(saved_porch >> 16) - (int)hbp_val);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
		h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
	}

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	writel(saved_porch, 0x05880028);
	dmb();
	printf("H713 hbp: restored 0x05880028=%08x\n", readl(0x05880028));
}

/*
 * One edge instead of eight bands.
 *
 * The pitch sweep was scored on two criteria and one of them was impossible.
 * The pale band is the hardware fetching fewer pixels per display line than the
 * line shows; the sweep only changes the pattern written into memory, so no
 * candidate can alter the band, and it did not. Only verticality can respond --
 * and judging whether eight fine bands are quite vertical, through blur and
 * keystone, is a poor way to decide anything.
 *
 * So: two halves, red then blue, repeating every P pixels in linear order.
 *
 * Row Y begins at word Y*P_hw, so its phase is (Y*D) mod P for D = P_hw - P.
 * The boundary therefore slides D pixels per row -- and, crucially, it *wraps*.
 * It crosses the full width once every P/|D| rows, so a 720-row frame shows
 *
 *	N = 720*|D| / P
 *
 * diagonal red/blue stripes, and inverting that gives the answer from any step:
 *
 *	|P_hw - P| = N * P / 720,  about 1.64 px per stripe at these pitches.
 *
 * That is the real scoring rule, and it is much stronger than "look for the
 * vertical one": every step measures the pitch independently, and the six must
 * agree. Expect a single edge only from a step within ~1.6 px of the truth --
 * with 4 px steps at most one can be, and the rest will show 2, 3, 5, 8 stripes
 * rather than the one boundary an earlier note here promised. Do not read a
 * multi-stripe step as a failed step or as moire; it is the measurement.
 *
 * Fewest stripes is closest, and two steps leaning opposite ways bracket the
 * answer between them.
 */
static void h713_disp_fill_edge(u32 pitch)
{
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	/*
	 * Fill the full span, not pitch * height and not even the nominal
	 * allocation. The hardware walks its own stride, so a 720-row frame
	 * consumes stride * 720 words; at the measured 1237 that is 890640,
	 * and the stride sweep drives it to 1296, which needs 933120 against
	 * the 921600 the buffer nominally holds. Anything short leaves the
	 * bottom of the frame fetching unwritten memory -- and an unexplained
	 * artefact at a frame edge has cost this bring-up a session before.
	 * The pattern is periodic, so covering the span costs one short loop.
	 */
	u32 total = H713_DISP_EDGE_SPAN_WORDS;
	u32 i;

	for (i = 0; i < total; i++)
		fb[i] = (i % pitch) < pitch / 2 ? 0xffff0000 : 0xff0000ff;

	flush_cache(H713_DISP_OSD_FB_ADDR, total * sizeof(u32));
}

/*
 * The animated test signal: one red bar on blue, moving left to right.
 *
 * Everything rendered in this bring-up so far has been a single frame held
 * still, so sustained commits, frame timing and liveness are all untested.
 * h713_disp_commit_osd_frame() hand-manages what stock does in an IRQ handler,
 * and at most six chained commits had ever been exercised before this existed.
 *
 * Deliberately not decoded video. A decoder brings its own buffers, format
 * conversion and timing, so a stall would have five candidate causes instead of
 * one -- and this bring-up has already spent days modelling a symptom measured
 * downstream of a fault as a fault of its own. Video is the integration test
 * once this passes, and this becomes the reference DECD is scored against.
 *
 * Chroma, not luminance: the optical path normalises luminance away, which is
 * why every photographable signal here is red against blue.
 *
 * The bar's position encodes the frame number. x = (frame * STEP) mod WIDTH, so
 * a photograph gives frame mod (WIDTH / STEP) = mod 80, and the console prints
 * the committed count. If the predicted x and the photographed x agree, the
 * panel is showing the frame the ARM believes it committed -- which is the
 * whole point, and is not something a static frame can establish.
 */
#define H713_DISP_ANIM_BAR_PX		64
#define H713_DISP_ANIM_STEP_PX		16
#define H713_DISP_ANIM_FRAMES		600
#define H713_DISP_ANIM_REPORT		60

static void h713_disp_fill_bar(u32 x, ulong addr)
{
	u32 *fb = (u32 *)addr;
	/*
	 * Same reasoning as h713_disp_fill_edge(): cover the whole span the
	 * hardware may walk, not width * height, so no row ever fetches
	 * unwritten memory.
	 */
	u32 total = H713_DISP_EDGE_SPAN_WORDS;
	u32 row, i;

	for (i = 0; i < total; i++)
		fb[i] = 0xff0000ff;

	/*
	 * Row-addressed, which is only legitimate because S = V is settled:
	 * 0x05600170 is a plain byte stride and the layer X origin is 0, so row
	 * r begins at word r * 1280. Do not combine this mode with a stride
	 * override -- the bar would shear and the position would stop meaning
	 * the frame number.
	 */
	for (row = 0; row < H713_DISP_OSD_HEIGHT; row++) {
		u32 base = row * H713_DISP_OSD_WIDTH;
		u32 k;

		for (k = 0; k < H713_DISP_ANIM_BAR_PX; k++)
			fb[base + (x + k) % H713_DISP_OSD_WIDTH] = 0xffff0000;
	}

	flush_cache(addr, total * sizeof(u32));
}

/*
 * Double buffering, added 2026-08-07 because fb-anim's first run tore.
 *
 * The single-buffered path rewrites the surface AFBD is streaming from, so the
 * raster sees a half-updated frame -- the bar arrives broken during motion and
 * whole once it stops. That same observation is what proves AFBD does not latch
 * the surface at submission: a snapshotted frame could not tear afterwards.
 *
 * So write the frame the hardware is *not* showing, point 0x05600178 at it, and
 * commit. Completion is vsync-locked (measured to 0.04% of a frame period), so
 * the flip lands on a frame boundary.
 *
 * fb-anim keeps the single-buffered behaviour deliberately. It is the recorded
 * baseline, and keeping it makes fb-anim vs fb-anim-db a one-variable A/B on the
 * same instrument rather than a comparison against a remembered result.
 */
static void h713_disp_anim_run(u32 frames, bool double_buffered)
{
	struct h713_disp_commit_stat st;
	u32 f, x = 0, completed = 0, timeouts = 0, first_timeout = 0;
	u64 wait_sum = 0, fill_sum = 0;
	uint wait_min = ~0U, wait_max = 0, fill_min = ~0U, fill_max = 0;
	ulong t_start, t_end;
	ulong front = H713_DISP_OSD_FB_ADDR;
	bool stopped = false;

	if (!frames)
		frames = H713_DISP_ANIM_FRAMES;

	printf("H713 anim: %u frames, %u px bar stepping %u px/frame, wrapping "
	       "every %u frames, %s.\n"
	       "H713 anim: SMOOTH MOTION means sustained commits work. A STALL "
	       "after k frames localises the handshake break -- the frame number "
	       "is reported. TEARING shows as a horizontal discontinuity in the "
	       "bar. Ctrl-C stops.\n",
	       frames, H713_DISP_ANIM_BAR_PX, H713_DISP_ANIM_STEP_PX,
	       H713_DISP_OSD_WIDTH / H713_DISP_ANIM_STEP_PX,
	       double_buffered ? "DOUBLE-buffered (expect no tear)" :
				 "SINGLE-buffered (expect a tear)");

	t_start = timer_get_us();

	for (f = 0; f < frames; f++) {
		ulong t0;
		ulong target;
		uint fill_us;

		x = (f * H713_DISP_ANIM_STEP_PX) % H713_DISP_OSD_WIDTH;

		/*
		 * Single-buffered draws into the live surface, which is the
		 * whole point of the baseline. Double-buffered draws into the
		 * one the hardware is not reading.
		 */
		target = !double_buffered ? H713_DISP_OSD_FB_ADDR :
			 front == H713_DISP_OSD_FB_ADDR ?
			 H713_DISP_OSD_FB_ADDR_B : H713_DISP_OSD_FB_ADDR;

		t0 = timer_get_us();
		h713_disp_fill_bar(x, target);
		fill_us = (uint)(timer_get_us() - t0);

		/*
		 * Publish the address before the submission, so the commit that
		 * the raster completes at vsync is the one carrying the new
		 * surface.
		 */
		if (double_buffered) {
			writel((u32)target, H713_DISP_AFBD_SRC_REG);
			dmb();
		}

		h713_disp_commit_osd_frame_quiet(&st);
		front = target;

		fill_sum += fill_us;
		if (fill_us < fill_min)
			fill_min = fill_us;
		if (fill_us > fill_max)
			fill_max = fill_us;

		wait_sum += st.wait_us;
		if (st.wait_us < wait_min)
			wait_min = st.wait_us;
		if (st.wait_us > wait_max)
			wait_max = st.wait_us;

		if (st.completed) {
			completed++;
		} else {
			/*
			 * Report the first one immediately and keep going:
			 * whether it recovers or stays wedged is the actual
			 * diagnostic, and stopping here would discard it.
			 */
			if (!timeouts++) {
				first_timeout = f;
				printf("H713 anim: FIRST COMMIT TIMEOUT at "
				       "frame %u (irq=%08x->%08x->%08x, waited "
				       "%u us). Continuing to see whether it "
				       "recovers.\n", f, st.pending, st.cleared,
				       st.done, st.wait_us);
			}
		}

		if (f && !(f % H713_DISP_ANIM_REPORT))
			printf("H713 anim: frame %u, bar x=%u, committed %u, "
			       "timeouts %u, last wait %u us, fill %u us, "
			       "front=%08lx\n",
			       f, x, completed, timeouts, st.wait_us, fill_us,
			       front);

		if (ctrlc()) {
			printf("H713 anim: interrupted at frame %u\n", f);
			frames = f + 1;
			stopped = true;
			break;
		}
	}

	t_end = timer_get_us();

	printf("H713 anim: %s after %u frame(s) in %lu ms\n",
	       stopped ? "STOPPED" : "complete", frames,
	       (t_end - t_start) / 1000);
	printf("H713 anim: committed %u, timeouts %u%s\n",
	       completed, timeouts, timeouts ? "" : " -- sustained commits OK");
	if (timeouts)
		printf("H713 anim: first timeout was frame %u; %u of %u frames "
		       "completed, so the handshake %s\n", first_timeout,
		       completed, frames,
		       completed > first_timeout ? "recovered at least once" :
		       "did not recover -- suspect the missing ARM IRQ handler");
	printf("H713 anim: commit wait min/mean/max %u/%lu/%u us; "
	       "fill min/mean/max %u/%lu/%u us\n",
	       wait_min, (ulong)(wait_sum / frames), wait_max,
	       fill_min, (ulong)(fill_sum / frames), fill_max);
	/*
	 * Leave the hardware where every other mode expects to find it. They
	 * all write H713_DISP_OSD_FB_ADDR and none of them touch the source
	 * register, so finishing on the back buffer would silently break the
	 * next command in the session -- exactly the class of cross-run trap
	 * that cost this bring-up four results before teardown existed.
	 * Re-render the same frame into the front buffer so the picture does
	 * not change while the state does.
	 */
	if (double_buffered && front != H713_DISP_OSD_FB_ADDR) {
		struct h713_disp_commit_stat rst;

		h713_disp_fill_bar(x, H713_DISP_OSD_FB_ADDR);
		writel((u32)H713_DISP_OSD_FB_ADDR, H713_DISP_AFBD_SRC_REG);
		dmb();
		h713_disp_commit_osd_frame_quiet(&rst);
		printf("H713 anim: front buffer restored to %08lx, reads back "
		       "%08x\n", H713_DISP_OSD_FB_ADDR,
		       readl(H713_DISP_AFBD_SRC_REG));
	}

	printf("H713 anim: PHOTOGRAPH NOW. Final bar x=%u (frame %u mod %u = "
	       "%u). If the bar is not there, the panel is not showing the "
	       "frame the ARM committed.\n",
	       x, frames - 1, H713_DISP_OSD_WIDTH / H713_DISP_ANIM_STEP_PX,
	       (frames - 1) % (H713_DISP_OSD_WIDTH / H713_DISP_ANIM_STEP_PX));
	if (double_buffered)
		printf("H713 anim: SCORING -- compare against 'fb-anim' on the "
		       "same boot. The bar tearing during motion there and not "
		       "here is the result; a still photograph cannot show it.\n");
}

static void h713_disp_edge_sweep(const u16 *list, uint count)
{
	uint i;

	printf("H713 edge: COUNT THE DIAGONAL RED/BLUE STRIPES in each step. "
	       "N stripes down the frame means the pitch is wrong by "
	       "N*P/height px "
	       "-- so every step measures it, and the six must agree. Fewest "
	       "stripes is closest; opposite leans bracket the answer. Several "
	       "stripes is the measurement, not a failed step. The pale band "
	       "cannot change and is not a criterion.\n");

	for (i = 0; i < count; i++) {
		u32 p = list[i];

		h713_disp_fill_edge(p);
		h713_disp_chroma_marker(i + 1);
		h713_disp_commit_osd_frame();
		printf("H713 edge: step %u, assumed pitch %u px; each stripe = "
		       "%u.%02u px of error, so |true - %u| = stripes * that\n",
		       i + 1, p, p * 100 / H713_DISP_OSD_HEIGHT / 100,
	       p * 100 / H713_DISP_OSD_HEIGHT % 100, p);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
	}
	printf("H713 edge: sweep complete\n");
}

/*
 * Confirm the stride to the pixel.
 *
 * test_30 put it at 1237 from five steps that were all 37..57 px away, so the
 * value comes from a slope, not from a step that matched. These six sit within
 * 5 px of it, and at S = 1237 the predicted counts are
 *
 *	1232 -> 2.9   1234 -> 1.7   1236 -> 0.58
 *	1237 -> 0     1238 -> 0.58  1240 -> 1.7
 *
 * a V with one minimum. The 1237 step is the only one that can show a single
 * edge standing truly vertical; +-1 px still traverses 58% of the width and
 * reads as clearly slanted. If the minimum lands anywhere but step 4, the
 * stride is whatever that step says and 1237 was wrong.
 */
static const u16 h713_pitch_sweep_fine[] = {
	1232, 1234, 1236, 1237, 1238, 1240,
};

/*
 * Does anything actually read the AFBD line-stride register?
 *
 * 0x05600170 holds 0x1400 -- 5120 bytes, 1280 px -- while the hardware fetches
 * 1237. So either it is not consulted, or it is consulted through something
 * that is not the identity. A pitch sweep cannot tell those apart; only writing
 * it can.
 *
 * Hold the pattern at 1237, where the unmodified register gives a single
 * vertical edge, and step the register instead. If it is live with unit slope,
 * driving it to 1280+k px moves the fetch to 1237+k and the frame picks up
 * 720*k/1237 stripes:
 *
 *	1280 -> 0    1284 -> 2.3   1288 -> 4.7
 *	1292 -> 7.0  1296 -> 9.3   1272 -> 4.7, leaning the other way
 *
 * If it is inert, all six steps are the same single vertical edge. Those two
 * outcomes are not subtle and neither needs a register read to score.
 *
 * The last step is the tilt control: it predicts the same count as 1288 with
 * the opposite lean, so it separates a real response from a count that happens
 * to grow.
 */
static const u16 h713_stride_sweep_px[] = {
	1280, 1284, 1288, 1292, 1296, 1272,
};

static void h713_disp_stride_sweep(void)
{
	u32 saved = readl(H713_DISP_AFBD_STRIDE_REG);
	uint i;

	printf("H713 stride: pattern held at %u px, AFBD stride 0x%08lx swept. "
	       "Register reads %08x (%u px) while the fetch measures %u, so "
	       "this asks whether the register is consulted at all. LIVE: the "
	       "steps pick up 0/2.3/4.7/7.0/9.3 stripes and the last leans the "
	       "other way. INERT: six identical single vertical edges.\n",
	       H713_DISP_EDGE_PITCH, H713_DISP_AFBD_STRIDE_REG,
	       saved, saved / 4, H713_DISP_EDGE_PITCH);

	h713_disp_fill_edge(H713_DISP_EDGE_PITCH);

	for (i = 0; i < ARRAY_SIZE(h713_stride_sweep_px); i++) {
		u32 px = h713_stride_sweep_px[i];

		writel(px * 4, H713_DISP_AFBD_STRIDE_REG);
		dmb();
		h713_disp_chroma_marker(i + 1);
		h713_disp_commit_osd_frame();
		printf("H713 stride: step %u, register %08x (%u px), reads back "
		       "%08x; predicted %u stripe(s) if live, 0 if inert\n",
		       i + 1, px * 4, px, readl(H713_DISP_AFBD_STRIDE_REG),
		       H713_DISP_OSD_HEIGHT *
		       (px > H713_DISP_OSD_WIDTH ? px - H713_DISP_OSD_WIDTH
						 : H713_DISP_OSD_WIDTH - px) /
		       H713_DISP_EDGE_PITCH);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
	}

	writel(saved, H713_DISP_AFBD_STRIDE_REG);
	dmb();
	printf("H713 stride: sweep complete, restored 0x%08lx=%08x\n",
	       H713_DISP_AFBD_STRIDE_REG, readl(H713_DISP_AFBD_STRIDE_REG));
}

/*
 * Kept as the run that closed the framebuffer path, and as the correction to
 * how it was understood.
 *
 * The sweep was built expecting S = V - 42, from test_31, and predicting a
 * minimum near V = 1322. test_33 falsified that outright. With the layer X
 * origin zeroed, the five swept steps come back at S = 1300.1, 1309.8, 1319.4,
 * 1329.1, 1338.7 against registers 1300..1340 -- so
 *
 *	S = V
 *
 * and the stride register is a plain byte stride that was correct all along.
 * The control step at the stock 0x1400 rendered one dead-vertical red/blue
 * edge across the full width, drifting 0.0 px per row.
 *
 * So there was never a second fault. The stride deficit measured twice as
 * 1237-1238 was a *consequence* of 0x0528008c holding 123; zeroing the origin
 * removed the shear as well as the band. Two symptoms, one register.
 *
 * That is the same mistake the old log made when it read the band as a stride,
 * one level up: a number measured downstream of a fault was modelled as an
 * independent fault of its own. Neither test_30 nor test_31 was wrong about
 * what it measured -- S really was 1238 while the origin was 123 -- and both
 * were wrong about what it meant.
 *
 * The mechanism, why an origin of 123 costs 42 px of advance per row, is not
 * understood and is no longer load-bearing.
 */
static const u16 h713_stride_fix_sweep_px[] = {
	1300, 1310, 1320, 1330, 1340, 1280,
};

/*
 * The other fault: content starts ~110 px into every display line.
 *
 * Measured across all eleven photographs of test_30 and test_31: left pad 105
 * +- 5 px, right pad 1.4, top pad **0.0**, bottom 5. So it is horizontal only,
 * and it is at the head of the line -- not the tail, which is what "the fetch
 * runs out" implied and what the old log assumed. It does not respond to the
 * stride (24 px of sweep moved the band by 4) or to the pattern pitch.
 *
 * A zero top pad also kills the tidiest hypothesis: 0x05280088 and 0x0528008c
 * hold 22 and 123, which read like a (y, x) origin, and 123 is within a few px
 * of the band. But an origin with y = 22 would blank 22 rows at the top, and
 * nothing is blank at the top. 0x0528008c stays on the list -- 123 is still the
 * closest value in the dump -- but not as half of a pair.
 *
 * No candidate is convincing enough to sweep on its own, so screen several. A
 * solid fill is the right pattern here: it is immune to the stride bug, so the
 * band edge stands as one crisp vertical boundary with nothing else moving, and
 * every step is scored by one number. Each step zeroes an offset-shaped field
 * and restores it; the top candidate also gets a large positive value, so a
 * register whose zero happens to sit near the current band cannot read as
 * inert.
 *
 * Registers that hold the same value in pairs are written together. Changing
 * one half of a producer/consumer pair can be a no-op that looks like a null.
 *
 * The control step is worth a photograph on its own account. If the band stays
 * pale against a saturated fill it is not showing framebuffer content at all;
 * if it turns red, it is, and this is a fetch or addressing fault rather than a
 * geometry one. Nothing else in the run distinguishes those.
 */
struct h713_band_probe {
	const char *what;
	u32 addr[2];
	u32 mask;
	u32 value;
};

static const struct h713_band_probe h713_band_probes[] = {
	{ "control, nothing written",
	  { 0, 0 }, 0, 0 },
	{ "de-layers 0x0528008c 123 -> 0",
	  { 0x0528008c, 0 }, 0xffffffff, 0 },
	{ "de-layers 0x0528008c 123 -> 400",
	  { 0x0528008c, 0 }, 0xffffffff, 400 },
	{ "mixer 0x0525c01c+034 low 60 -> 0",
	  { 0x0525c01c, 0x0525c034 }, 0xffff, 0 },
	{ "de 0x0524c004 low 22 -> 0",
	  { 0x0524c004, 0 }, 0xffff, 0 },
	{ "vblender 0x0520000c+024 low 49 -> 0",
	  { 0x0520000c, 0x05200024 }, 0xffff, 0 },
};

static void h713_disp_fill_solid(u32 argb)
{
	u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
	u32 total = H713_DISP_EDGE_SPAN_WORDS;
	u32 i;

	for (i = 0; i < total; i++)
		fb[i] = argb;

	flush_cache(H713_DISP_OSD_FB_ADDR, total * sizeof(u32));
}

/*
 * Hold a full-white field and step the dimmer.
 *
 * White because the question is whether the backlight changes, and a bright
 * uniform field makes a duty change unmistakable while a dark one hides it.
 * No chroma markers between steps: they drive the TCON generator, which would
 * replace the field and change apparent brightness by itself. The console
 * prints the step instead, and the operator watches continuously.
 */
static void h713_disp_backlight_sweep(void)
{
	static const u8 duty[] = { 100, 75, 50, 25, 0, 100 };
	uint i;

	printf("H713 backlight: full-white field, stepping PWM2 duty on PB4 "
	       "mux 3. Each step prints CNT twice; if those two values differ "
	       "the channel is genuinely running, and the panel's response (or "
	       "lack of one) is evidence about the panel rather than about this "
	       "code. As of 2026-08-05 the counter runs, the duty steps, and "
	       "brightness does not change -- put a DMM on PB4 to find out "
	       "whether the waveform actually reaches the pad.\n");

	h713_disp_fill_solid(0xffffffff);
	h713_disp_commit_osd_frame();

	for (i = 0; i < ARRAY_SIZE(duty); i++) {
		printf("H713 backlight: step %u of %u -> %u%%\n",
		       i + 1, (uint)ARRAY_SIZE(duty), duty[i]);
		h713_disp_backlight_set(duty[i]);
		mdelay(4000);
	}
	printf("H713 backlight: sweep complete, left at %u%%\n",
	       duty[ARRAY_SIZE(duty) - 1]);
}

static void h713_disp_band_sweep(void)
{
	uint i, j;

	printf("H713 band: solid red fill, so the only thing to read is where "
	       "content starts. Baseline is ~110 px of pale at the LEFT, 0 at "
	       "the top. Score every step by that one number: a step whose band "
	       "moves names the register. All six the same means none of these "
	       "carries it, and the fault is not one of these offsets.\n");

	h713_disp_fill_solid(0xffff0000);

	for (i = 0; i < ARRAY_SIZE(h713_band_probes); i++) {
		const struct h713_band_probe *p = &h713_band_probes[i];
		u32 saved[2] = { 0, 0 };

		for (j = 0; j < 2; j++) {
			if (!p->addr[j])
				continue;
			saved[j] = readl(p->addr[j]);
			writel((saved[j] & ~p->mask) | (p->value & p->mask),
			       p->addr[j]);
		}
		dmb();

		h713_disp_chroma_marker(i + 1);
		h713_disp_commit_osd_frame();

		printf("H713 band: step %u, %s", i + 1, p->what);
		for (j = 0; j < 2; j++)
			if (p->addr[j])
				printf("; 0x%08x was %08x now %08x",
				       p->addr[j], saved[j],
				       readl(p->addr[j]));
		printf("\n");
		mdelay(H713_DISP_CHROMA_PHASE_MS);

		for (j = 0; j < 2; j++)
			if (p->addr[j])
				writel(saved[j], p->addr[j]);
		dmb();
	}
	printf("H713 band: sweep complete, all probed registers restored\n");
}

static void h713_disp_stride_fix_sweep(void)
{
	u32 saved = readl(H713_DISP_AFBD_STRIDE_REG);
	uint i;

	printf("H713 fix: pattern written at the natural %u px, AFBD stride "
	       "swept. S = V, so expect 11.3/16.9/22.5/28.1/33.8 stripes and "
	       "the last step -- the stock %u px -- to render ONE vertical edge. "
	       "That last step is the whole framebuffer path, correct.\n",
	       H713_DISP_OSD_WIDTH, saved / 4);

	h713_disp_fill_edge(H713_DISP_OSD_WIDTH);

	for (i = 0; i < ARRAY_SIZE(h713_stride_fix_sweep_px); i++) {
		u32 px = h713_stride_fix_sweep_px[i];

		writel(px * 4, H713_DISP_AFBD_STRIDE_REG);
		dmb();
		h713_disp_chroma_marker(i + 1);
		h713_disp_commit_osd_frame();
		printf("H713 fix: step %u, register %08x (%u px), reads back "
		       "%08x; S = %u, so %u.%u stripe(s) expected\n",
		       i + 1, px * 4, px,
		       readl(H713_DISP_AFBD_STRIDE_REG), px,
		       720 * (px > H713_DISP_OSD_WIDTH ?
			      px - H713_DISP_OSD_WIDTH :
			      H713_DISP_OSD_WIDTH - px) / H713_DISP_OSD_WIDTH,
		       (7200 * (px > H713_DISP_OSD_WIDTH ?
				px - H713_DISP_OSD_WIDTH :
				H713_DISP_OSD_WIDTH - px) /
			H713_DISP_OSD_WIDTH) % 10);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
	}

	writel(saved, H713_DISP_AFBD_STRIDE_REG);
	dmb();
	printf("H713 fix: sweep complete, restored 0x%08lx=%08x\n",
	       H713_DISP_AFBD_STRIDE_REG, readl(H713_DISP_AFBD_STRIDE_REG));
}

static void h713_disp_pitch_sweep(const u16 *list, uint count)
{
	uint i;

	printf("H713 pitch: sweeping the assumed line pitch; the step whose "
	       "bands stand up straight is the hardware's true pitch\n");

	for (i = 0; i < count; i++) {
		u32 p = list[i];

		h713_disp_fill_pitch(p);
		h713_disp_chroma_marker(i + 1);
		h713_disp_commit_osd_frame();
		printf("H713 pitch: step %u, assumed pitch %u px (d = %+d); "
		       "vertical bands mean this is correct\n",
		       i + 1, p, (int)p - H713_DISP_OSD_WIDTH);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
	}

	printf("H713 pitch: sweep complete\n");
}

/*
 * Three full-scale primaries, then the same red/blue pair as a checker at two
 * cell sizes a factor of four apart.
 *
 * The solids establish that the link carries a colour at all and that all three
 * components are separately controllable. The checkers then ask the question
 * that matters: does ANY spatial frequency survive? test_16 showed a style-8
 * checker arriving as a uniform field with adjacent-column contrast lower than
 * baseline, so a checker that stays uniform at both sizes means the link
 * carries no positional information, while a visible difference between 128 and
 * 32 would put a resolvable limit on it.
 */
static void h713_disp_boardb_tcon_chroma_test(void)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);

	printf("H713 panel: chroma generator sweep; count the blinks before "
	       "each phase\n");
	printf("H713 panel: TCON saved: ctrl=%08x mode=%08x size=%08x "
	       "rgb=%08x/%08x\n", saved_ctrl, saved_mode, saved_size,
	       saved_rgb0, saved_rgb1);

	h713_disp_chroma_marker(1);
	h713_disp_chroma_phase("SOLID RED", saved_ctrl, H713_TCON_CELL_STOCK,
			       H713_TCON_RGB_RED, H713_TCON_RGB_RED);

	h713_disp_chroma_marker(2);
	h713_disp_chroma_phase("SOLID GREEN", saved_ctrl, H713_TCON_CELL_STOCK,
			       H713_TCON_RGB_GREEN, H713_TCON_RGB_GREEN);

	h713_disp_chroma_marker(3);
	h713_disp_chroma_phase("SOLID BLUE", saved_ctrl, H713_TCON_CELL_STOCK,
			       H713_TCON_RGB_BLUE, H713_TCON_RGB_BLUE);

	h713_disp_chroma_marker(4);
	h713_disp_chroma_phase("CHECKER RED/BLUE 128px", saved_ctrl,
			       0x00800080, H713_TCON_RGB_RED,
			       H713_TCON_RGB_BLUE);

	h713_disp_chroma_marker(5);
	h713_disp_chroma_phase("CHECKER RED/BLUE 32px", saved_ctrl,
			       0x00200020, H713_TCON_RGB_RED,
			       H713_TCON_RGB_BLUE);

	h713_disp_chroma_marker(6);
	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	dmb();
	printf("H713 panel: CHROMA RESTORED: ctrl=%08x mode=%08x size=%08x "
	       "rgb=%08x/%08x scan=%08x; holding 5 seconds\n",
	       readl(H713_DISP_TCON_CTRL_REG), readl(H713_DISP_TCON_MODE_REG),
	       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);
}

/*
 * Sweep the display PLL and watch for the checker to resolve.
 *
 * clkfind settled that 0x058c0014 is the display PLL -- N+1 from 43 to 42 moved
 * the free-running counter -2.28% against a predicted -2.326% -- so the PLL runs
 * at 24 MHz * (N+1) = 1032 MHz and the counter is PLL/56.
 *
 * It did not settle the ratio between the PLL and DCLK, and cannot: the two
 * fields the panel config writes into 0x058c0024 both left the counter flat,
 * which is what a divider downstream of the counter's tap point looks like. So
 * DCLK is 1032/14 = 73.7, 1032/7 = 147.4, or 18.4 MHz, and no measurement
 * available here distinguishes them.
 *
 * Searching is cheaper than deducing. The chroma checker now reproduces across
 * boots -- test_17, test_18 and test_19 all separated the solids cleanly -- so
 * it is a usable instrument, and the PLL is a verified knob. Step N across a
 * range that brackets 62 MHz under the 1032/14 reading, show the red/blue
 * checker at each step, and look for cells to appear.
 *
 * N+1 from 32 to 52 keeps the PLL between 768 and 1248 MHz, near its working
 * point, which is deliberately a narrower range than the retune that hung the
 * board. Under the /14 reading that spans 54.9 to 89.1 MHz of DCLK and brackets
 * the panel's 62. If the checker resolves at any step, that step's N gives both
 * the right DCLK and the ratio; if none does, the /14 reading is wrong and the
 * remaining two candidates need a different approach.
 *
 * Each step verifies itself: the counter must track N proportionally, and a
 * step where it does not is reported and its visual result disclaimed.
 */
#define H713_DISP_PLL_REG	0x058c0014UL
#define H713_DISP_PLL_N_NOW	43

static const u8 h713_n_sweep[] = { 32, 36, 40, 43, 47, 52 };

/*
 * The first sweep found banding at N+1 = 32, 36 and 52, nothing at 40, 43 and
 * 47, and the strongest response at 52 -- which was the top of the range. The
 * optimum may well lie beyond it, so this list extends upward. 1440 MHz is 40%
 * above the working point and well inside the rate limits the sibling CCU video
 * PLLs document, and every step restores.
 */
static const u8 h713_n_sweep_hi[] = { 50, 52, 54, 56, 58, 60 };

static void h713_disp_tcon_n_sweep(const u8 *list, uint count)
{
	u32 saved_pll = readl(H713_DISP_PLL_REG);
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);
	ulong base = h713_disp_counter_khz(100000);
	uint i;

	printf("H713 nsweep: PLL %08x, counter %lu kHz at N+1=%u; stepping N "
	       "with the red/blue 128px checker at each step\n",
	       saved_pll, base, H713_DISP_PLL_N_NOW);

	for (i = 0; i < count; i++) {
		uint np1 = list[i];
		u32 val = (saved_pll & ~(0xFFU << 8)) |
			  (((np1 - 1) & 0xFF) << 8);
		ulong khz, expect;
		long err;

		printf("H713 nsweep: step %u, N+1=%u, PLL %08x -> %08x ...\n",
		       i + 1, np1, saved_pll, val);
		writel(val, H713_DISP_PLL_REG);
		dmb();
		mdelay(100);

		khz = h713_disp_counter_khz(50000);
		expect = base * np1 / H713_DISP_PLL_N_NOW;
		err = expect ? ((long)khz - (long)expect) * 100 / (long)expect : 0;
		printf("H713 nsweep: counter %lu kHz, expected %lu (%+ld%%)%s\n",
		       khz, expect, err,
		       (err > 3 || err < -3) ?
		       "   <== did not track; this step's image means nothing" :
		       "");

		h713_disp_chroma_marker(i + 1);
		h713_disp_tcon_pattern_apply(saved_ctrl, 0x00800080,
					     H713_TCON_RGB_RED,
					     H713_TCON_RGB_BLUE);
		printf("H713 nsweep: CHECKER AT N+1=%u ACTIVE; observe now\n",
		       np1);
		mdelay(H713_DISP_CHROMA_PHASE_MS);
		h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
	}

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	writel(saved_pll, H713_DISP_PLL_REG);
	dmb();
	mdelay(100);
	printf("H713 nsweep: restored PLL %08x, counter %lu kHz\n",
	       readl(H713_DISP_PLL_REG), h713_disp_counter_khz(100000));
}

/*
 * Run the chroma sweep with the panel clock retuned to the DCLK the panel
 * actually asks for, and prove the retune took effect before believing
 * anything optical.
 */
#define H713_PLL_N_STOCK	43	/* 1032 MHz 4x -> 73.71 MHz DCLK */
#define H713_PLL_N_62M		36	/*  864 MHz 4x -> 61.71 MHz DCLK */

static void h713_disp_chroma_test_at_62m(void)
{
	u32 saved_pll = readl(H713_PLL_VIDEO2_REG);
	ulong before, after, expect;
	int ret;

	before = h713_disp_counter_khz(100000);
	printf("H713 pll: counter %lu kHz before retune "
	       "(expect 18432 for the stock 516 MHz panel clock)\n", before);

	ret = h713_disp_pll_video2_set_n(H713_PLL_N_62M);
	mdelay(50);

	expect = 24000UL * H713_PLL_N_62M / H713_PANEL_CLK_DIV /
		 H713_COUNTER_DIV;
	after = h713_disp_counter_khz(100000);
	printf("H713 pll: counter %lu kHz after retune (expect %lu)\n",
	       after, expect);
	printf("H713 pll: implied DCLK %lu.%02lu MHz "
	       "(panel_config asks 62.00)\n",
	       after * H713_COUNTER_DIV / H713_LVDS_SER_RATIO / 1000,
	       (after * H713_COUNTER_DIV / H713_LVDS_SER_RATIO / 10) % 100);

	if (ret) {
		printf("H713 pll: PLL did not relock; restoring and aborting\n");
	} else if (!after || after > expect + expect / 20 ||
		   after < expect - expect / 20) {
		printf("H713 pll: WARNING: counter did not track the retune "
		       "within 5%%. The clock did not change as modelled and no "
		       "visual result from this run is meaningful.\n");
		h713_disp_boardb_tcon_chroma_test();
	} else {
		printf("H713 pll: retune confirmed by the clock witness; "
		       "running the chroma sweep at %lu.%02lu MHz DCLK\n",
		       after * H713_COUNTER_DIV / H713_LVDS_SER_RATIO / 1000,
		       (after * H713_COUNTER_DIV / H713_LVDS_SER_RATIO / 10) % 100);
		h713_disp_boardb_tcon_chroma_test();
	}

	writel(saved_pll, H713_PLL_VIDEO2_REG);
	dmb();
	mdelay(50);
	printf("H713 pll: PLL_VIDEO2 restored to %08x, counter %lu kHz\n",
	       readl(H713_PLL_VIDEO2_REG), h713_disp_counter_khz(100000));
}

/*
 * The solid all-zero / all-one pair, each preceded by its own optical marker
 * so a recording can be scored without guessing where the phases fall. The
 * generator is switched off between the two codes: going zero -> off -> one
 * rather than zero -> one means a null result cannot be blamed on the
 * receiver holding its last decoded value across a same-geometry change.
 */
static void h713_disp_boardb_tcon_test_solid_marked(void)
{
	u32 saved_ctrl = readl(H713_DISP_TCON_CTRL_REG);
	u32 saved_mode = readl(H713_DISP_TCON_MODE_REG);
	u32 saved_size = readl(H713_DISP_TCON_PATTERN_SIZE_REG);
	u32 saved_rgb0 = readl(H713_DISP_TCON_PATTERN_RGB0_REG);
	u32 saved_rgb1 = readl(H713_DISP_TCON_PATTERN_RGB1_REG);
	uint i;

	printf("H713 panel: TCON solid saved: ctrl=%08x mode=%08x "
	       "size=%08x rgb=%08x/%08x\n",
	       saved_ctrl, saved_mode, saved_size, saved_rgb0, saved_rgb1);

	for (i = 0; i < 2; i++) {
		uint style = i ? 3 : 2;

		h713_disp_optical_marker(3 + i);
		h713_disp_boardb_tcon_style_apply(saved_ctrl, style);
		printf("H713 panel: BOARD-B TCON SOLID %s ACTIVE: ctrl=%08x "
		       "mode=%08x size=%08x rgb=%08x/%08x scan=%08x; "
		       "observe now\n", style == 2 ? "BLACK" : "WHITE",
		       readl(H713_DISP_TCON_CTRL_REG),
		       readl(H713_DISP_TCON_MODE_REG),
		       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
		       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS / 2);
		h713_disp_boardb_tcon_generator_off(saved_ctrl, saved_mode);
	}

	writel(saved_size, H713_DISP_TCON_PATTERN_SIZE_REG);
	writel(saved_rgb0, H713_DISP_TCON_PATTERN_RGB0_REG);
	writel(saved_rgb1, H713_DISP_TCON_PATTERN_RGB1_REG);
	writel(saved_mode, H713_DISP_TCON_MODE_REG);
	writel(saved_ctrl, H713_DISP_TCON_CTRL_REG);
	dmb();
	printf("H713 panel: TCON solid restored: ctrl=%08x mode=%08x "
	       "size=%08x rgb=%08x/%08x scan=%08x\n",
	       readl(H713_DISP_TCON_CTRL_REG),
	       readl(H713_DISP_TCON_MODE_REG),
	       readl(H713_DISP_TCON_PATTERN_SIZE_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB0_REG),
	       readl(H713_DISP_TCON_PATTERN_RGB1_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
}

/*
 * Select the normal DCLK edge, let the receiver settle, then run the all-zero
 * and all-one generator codes entirely under that edge.
 *
 * The first version of this test was uninterpretable. Its recording showed a
 * featureless field across every phase, but with no in-run liveness evidence
 * there was no way to tell whether normal DCLK had silenced the link or
 * whether the link was already silent before the edge was touched. Frame
 * analysis of that video put the panel's last structured output roughly
 * thirteen seconds ahead of the DCLK write, which favours the second reading
 * and leaves the DCLK hypothesis untested.
 *
 * So this version brackets the single variable with positive controls and
 * labels every phase optically:
 *
 *   1 blink   style 8 under the stock inverted edge   (link alive before?)
 *   2 blinks  style 8 under the normal edge           (did the edge kill it?)
 *   3 blinks  solid all-zero  under the normal edge
 *   4 blinks  solid all-one   under the normal edge
 *   5 blinks  everything restored
 *
 * Read it as: control 1 blank means the run is invalid and nothing downstream
 * counts. Control 1 banded and control 2 blank means the normal edge breaks
 * the link, and inverted DCLK -- which is also what Board B's panel_config.ini
 * requests -- is correct. Both controls banded makes the solid comparison
 * meaningful for the first time.
 */
static void h713_disp_boardb_tcon_normal_solid_test(void)
{
	u32 saved_lane = readl(H713_DISP_LVDS_LANE_REG);
	u32 normal_lane = saved_lane & ~BIT(24);

	printf("H713 panel: DCLK normal-edge solid test with bracketing "
	       "positive controls; count the blinks before each phase\n");

	h713_disp_optical_marker(1);
	h713_disp_tcon_positive_control("stock inverted DCLK");

	h713_disp_optical_marker(2);
	writel(normal_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK NORMAL selected: lane=%08x (saved %08x), "
	       "settling 1 second\n",
	       readl(H713_DISP_LVDS_LANE_REG), saved_lane);
	mdelay(1000);
	h713_disp_tcon_positive_control("normal DCLK");

	h713_disp_boardb_tcon_test_solid_marked();

	h713_disp_optical_marker(5);
	writel(saved_lane, H713_DISP_LVDS_LANE_REG);
	dmb();
	printf("H713 panel: DCLK NORMAL solid test complete; lane restored "
	       "to %08x, scan=%08x\n",
	       readl(H713_DISP_LVDS_LANE_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
	mdelay(5000);
}

static void h713_disp_animate_pattern(uint frames, uint dwell_ms, uint phase)
{
	uint i;

	for (i = 0; i < frames; i++) {
		h713_disp_fill_pattern(phase + i);
		h713_disp_commit_osd_frame();
		printf("H713 panel: animation %u/%u, holding %u ms\n",
		       i + 1, frames, dwell_ms);
		mdelay(dwell_ms);
	}
}

/*
 * Re-apply the selected DE block after the coprocessor has settled.
 *
 * The firmware demonstrably reprograms display hardware after our replay: it
 * rewrites the TCON timing to 1080p, and 0x05600168 -- written 0x000003b2 by
 * DE block 5 -- reads back zero once it has run. Re-imposing the TCON timing
 * post-readiness is already known to stick (h713_disp_latch_panel_timing), so
 * the OSD path gets the same treatment: replay the block, which re-asserts
 * AFBD's format, size, stride, buffer address and enable bit together with the
 * mixer and DE registers the same block owns.
 *
 * The block is replayed rather than hand-written so it applies exactly what
 * the records say, including the panel-config patches already made to them.
 * DE block 5 touches only 0x0560xxxx, 0x0525cxxx, 0x0524cxxx and 0x05280xxx,
 * so it cannot disturb the TCON timing latched just before it.
 *
 * This is a diagnostic. If the OSD appears only after the re-assert, the
 * firmware owns the fetch path and the real work is in how ownership is handed
 * over -- not in another framebuffer pattern.
 */
static int h713_disp_reassert_osd(ulong blob, u32 project)
{
	struct h713_disp_sel sel;
	int ret;

	ret = h713_disp_lookup(blob, project, &sel);
	if (ret)
		return ret;

	printf("H713 panel: re-asserting DE block %u after MIPS readiness\n",
	       sel.de);

	/* Stock writes the mixer control ahead of the DE table; keep parity. */
	writel(H713_DISPLAY_MIXER_CTRL_VALUE, H713_DISPLAY_MIXER_CTRL_REG);

	ret = h713_logo_walk(blob, sel.de_range.start, sel.de_range.end,
			     true);
	if (ret)
		return ret;

	/*
	 * AFBD's buffer address was just rewritten. The caller owns the surface
	 * contents: quiesce publishes bars, while vendor-logo must retain only
	 * the authenticated BMP. Do not inject a pattern from this shared helper.
	 */
	return 0;
}

/*
 * 0x05600144 is the last record DE block 5 applies -- a single-bit write under
 * mask 1 -- and it reads back zero afterwards. The stock driver identifies it
 * as AFBD channel 1's ready register; self-clearing after acceptance is normal.
 *
 * Write the bit and read it straight back, repeatedly. Sample two neighbours
 * at the same time: 0x05600168, channel 1's write-one-to-clear AFBD IRQ status
 * whose bit 1 means writeback complete, and 0x05880000, whose two halves track
 * the raster position within the programmed 1360x760 and therefore prove the
 * TCON is still scanning.
 *
 * If the enable is briefly observable set, or either neighbour reacts, the
 * write lands. If nothing anywhere moves, it is being swallowed.
 */
static void h713_disp_afbd_enable_probe(void)
{
	uint i;

	printf("H713 panel: AFBD enable probe\n");
	printf("  baseline    ctrl=%08x ready=%08x status=%08x scan=%08x\n",
	       readl(H713_DISP_AFBD_CTRL_REG),
	       readl(H713_DISP_AFBD_READY_REG),
	       readl(H713_DISP_AFBD_STATUS_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));

	/* Exact stock order: channel enable first, then literal ready = 1. */
	setbits_le32((void *)H713_DISP_AFBD_CTRL_REG, BIT(0));
	writel(1, H713_DISP_AFBD_READY_REG);

	for (i = 0; i < 4; i++)
		printf("  read %u      ctrl=%08x ready=%08x status=%08x "
		       "scan=%08x\n", i,
		       readl(H713_DISP_AFBD_CTRL_REG),
		       readl(H713_DISP_AFBD_READY_REG),
		       readl(H713_DISP_AFBD_STATUS_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));

	mdelay(50);
	printf("  +50ms       ctrl=%08x ready=%08x status=%08x scan=%08x\n",
	       readl(H713_DISP_AFBD_CTRL_REG),
	       readl(H713_DISP_AFBD_READY_REG),
	       readl(H713_DISP_AFBD_STATUS_REG),
	       readl(H713_DISP_LVDS_SCAN_REG));
}

/*
 * CPU_COMM routine identities.
 *
 * The firmware registers its HAL routines by name and the transport addresses
 * them by a hashed id. The hash is an Allwinner CRC32 variant over
 * "<name>_<cpu_id>_<pid_low12>", seed 0x123456, cpu_id 1 for MIPS-side
 * routines. Reproducing it against ten independently documented ids matched
 * 10/10, and the 85 THal_Vp_ and MipsHalCallback_ name strings in the
 * authenticated display.bin match the 82 registrations counted on hardware.
 *
 * These are the ids worth recognising in a live table. THal_Vp_*BlackScreen is
 * first because a firmware that powers on with black-screen asserted would
 * explain every observation to date: the raster runs, the LVDS PHY is
 * configured, PHY-injected colours are visible because they inject downstream
 * of composition -- and the composited output is forced black.
 */
static const struct { u32 id; const char *name; } h713_comm_routines[] = {
	{ 0xb66041d8, "THal_Vp_DisableBlackScreen"  },
	{ 0xa30d4c6b, "THal_Vp_EnableBlackScreen"   },
	{ 0x143ffc87, "THal_Vp_DisableScreenCover"  },
	{ 0x0152f134, "THal_Vp_EnableScreenCover"   },
	{ 0x396f16bf, "THal_Vp_SetImageBufferAddr"  },
	{ 0x2f02f7dd, "THal_Vp_GetImageBufferAddr"  },
	{ 0xeaf13de5, "THal_Vp_SetSource"           },
	{ 0x24efc7c9, "THal_Vp_GetSource"           },
	{ 0x83a878bf, "THal_Vp_SetPictureMode"      },
	{ 0x2d8338c3, "THal_Vp_GetPictureMode"      },
	{ 0x1c6ff747, "THal_Vp_Init"                },
	{ 0x3ab1d1dc, "THal_Vp_DisableVideoFreeze"  },
	{ 0x7bbd5772, "THal_Vp_Wce_GetActiveWindow" },
	{ 0x51ad877e, "Thal_Vp_SetBacklightLevel"   },
	{ 0xb46ce545, "Thal_Vp_SetBacklightPwmInfo" },
};

/*
 * Send one CPU_COMM CALL and poll for its reply.
 *
 * Every step below is transcribed from display.bin rather than inferred:
 *
 *   queue      0x8b1195ec/0x8b1195b8 are mirrors of the share_seq addressing,
 *              so a message from X to Y lives in share_seq(Y, X, idx). For
 *              ARM(0) -> MIPS(1) CALL that is share_seq(1,0,0), and the reply
 *              comes back in share_seq(0,1,1).
 *   allocate   0x8b118be8 takes share_seq+0x78 and pops via 0x8b1180ec, which
 *              returns base_addr + rd_idx*item_size and NULL when rd == wr.
 *              The popped entry holds the slot's ARM-physical address, and the
 *              slot carries its own index at +0x04, bounds-checked below 20.
 *   fill       memcpy(slot, msg, 104) at 0x8b1201b8, then slot[+0x04] = index
 *              and slot[+0x0A] = 2, then an assertion that the slot address is
 *              exactly share_seq + 0x168 + 104*index.
 *   publish    0x8b11f9ec writes share_seq[+0x10] = index, [+0x08] = flags,
 *              [+0x14] = session, [+0x18] = wait pointer, bumps [+0x04].
 *              The call site at 0x8b12046c sources the state byte from
 *              msg[+0x06] & 0xff, so the flags halfword is what lands there.
 *   doorbell   a single write to 0x03003874; the pulse workaround in the Linux
 *              tree belongs to the ARISC path, not this one.
 *
 * The doorbell word is the message type and nothing else. The msgbox LISR at
 * 0x8b121698 drains the port-1 FIFO and calls the registered channel callback
 * with the raw word in $a0; that callback (0x8b121d78) forwards it verbatim to
 * cpu_comm_cb at 0x8b122678 with $a1 hardcoded to zero. cpu_comm_cb switches
 * on the whole 32-bit value:
 *
 *      0 -> 0x8b11f0a4  CALL          2 -> 0x8b11f61c  CALL_ACK
 *      1 -> 0x8b11f360  RETURN        3 -> 0x8b11f804  RETURN_ACK
 *
 * and silently drops anything else. Each handler then calls queueAction with
 * the matching index, which activates HISR[cpu][index] -- the four records
 * named "%s CALL HISR" .. "%s RETURN_ACK HISR" that Trid_CPUComm_Init builds
 * at 0x8b1221e0. So a CALL is doorbell 0x00000000. There is no msg_type<<16
 * field and no intr_type field; the Linux tree's packing is not what this
 * firmware parses.
 *
 * The presence gate is share_seq[+0x08] bit 2. comm_handle_call resolves
 * share_seq(1,0,0) at 0x8b11f18c, tests that bit at 0x8b11f19c, and returns
 * immediately when it is clear -- which is every send this project has made.
 * When it is set the handler clears it (0x8b11f2ec) and queues the HISR, so
 * the bit going 4 -> 0 in shared memory is the on-hardware proof of receipt.
 *
 * Note the ordering hazard this implies: once bit 2 is set, share_seq[+0x10]
 * must hold a slot index below 20, or the assert at 0x8b11f24c spins the
 * firmware forever. The empty marker written by init is exactly 20, so the
 * flag must never be raised without a matching published index.
 *
 * The wait pointer is published as zero deliberately. The receiver stores it
 * at share_seq[+0x70]/[+0x74] without dereferencing, and the signal primitive
 * at 0x8b15c27c null-checks and returns an error rather than writing. U-Boot
 * polls, so it does not need to be signalled. The residual risk is behavioural
 * -- the firmware may log an error path -- not memory corruption.
 *
 * This is the first traffic in either direction. It writes into a live
 * coprocessor's queues, so it is a separate opt-in command and never runs as
 * part of panel-test.
 */
#define H713_COMM_CALL_SEQ_OFF	0x000026b8UL	/* share_seq(1,0,0) FreeCall  */
#define H713_COMM_CALL_ACK_SEQ_OFF 0x000013a8UL /* share_seq(0,1,0) ACK fields*/
#define H713_COMM_RET_SEQ_OFF	0x00001d30UL	/* share_seq(0,1,1) FreeReturn*/
#define H713_COMM_RET_ACK_SEQ_OFF 0x00003040UL	/* share_seq(1,0,1) ack fields*/
#define H713_COMM_MSG_SIZE	104
#define H713_COMM_MAX_PARAMS	10
#define H713_COMM_SET_PICTURE_MODE_ID	0x83a878bf
#define H713_COMM_GET_PICTURE_MODE_ID	0x2d8338c3
#define H713_COMM_SET_PICTURE_MODE_HANDLER 0x8b10a8c0
#define H713_COMM_GET_PICTURE_MODE_HANDLER 0x8b10a8ec
#define H713_COMM_DOORBELL	0x03003874UL	/* User2 sub0 port1 MSG_DATA  */
#define H713_COMM_DOORBELL_CALL	0x00000000	/* cpu_comm_cb type 0 = CALL  */
#define H713_COMM_DOORBELL_RETURN_ACK 0x00000003
#define H713_COMM_MSG_FLAG_SENT	4		/* msg[+0x06] bit 2, the gate */

struct h713_comm_reply {
	u16 nparams;
	u32 param[H713_COMM_MAX_PARAMS];
};

/*
 * Guard the real-handler marshalling test against a different call-table
 * layout or firmware build. In the authenticated board-B display.bin,
 * GetPictureMode is a read of the live 0x8b272988 state word. SetPictureMode
 * first compares its argument with that same word and performs no hardware
 * update when they match. The test below deliberately exercises that
 * same-value path, but only after the live table proves both ids, owner, and
 * exact adapter entry points.
 */
static int h713_comm_validate_picture_mode_routines(u32 pid)
{
	u32 count = h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_COUNT_OFF);
	bool found_get = false, found_set = false;
	uint i;

	if (!count || count > H713_MIPS_SHMEM_CALL_ENTRY_COUNT) {
		printf("H713 comm: call table is not populated consistently\n");
		return -ENODEV;
	}

	for (i = 0; i < H713_MIPS_SHMEM_CALL_ENTRY_COUNT; i++) {
		ulong off = H713_MIPS_SHMEM_CALL_TABLE_OFF +
			    i * H713_MIPS_SHMEM_CALL_ENTRY_SIZE;
		u32 id = h713_mips_read_shmem(off + 0x08);
		u32 expected_handler;
		const char *name;
		bool *found;

		if (id == H713_COMM_GET_PICTURE_MODE_ID) {
			expected_handler = H713_COMM_GET_PICTURE_MODE_HANDLER;
			name = "THal_Vp_GetPictureMode";
			found = &found_get;
		} else if (id == H713_COMM_SET_PICTURE_MODE_ID) {
			expected_handler = H713_COMM_SET_PICTURE_MODE_HANDLER;
			name = "THal_Vp_SetPictureMode";
			found = &found_set;
		} else {
			continue;
		}

		if (*found) {
			printf("H713 comm: duplicate %s registration -- refusing test\n",
			       name);
			return -EPROTO;
		}
		if (h713_mips_read_shmem(off + 0x04) != pid ||
		    h713_mips_read_shmem(off + 0x50) != expected_handler) {
			printf("H713 comm: %s entry %u does not match pid/handler "
			       "guard\n", name, i);
			return -EPROTO;
		}

		*found = true;
		printf("H713 comm: guarded %s entry %u handler=%08x\n",
		       name, i, expected_handler);
	}

	if (!found_get || !found_set) {
		printf("H713 comm: picture-mode getter/setter registration missing\n");
		return -ENOENT;
	}

	return 0;
}

static int h713_comm_call(u32 comp_id, const u32 *params, uint nparams,
			  u32 chan, u32 pid, struct h713_comm_reply *reply)
{
	ulong seq = H713_MIPS_SHMEM_ADDR + H713_COMM_CALL_SEQ_OFF;
	ulong call_ack_seq = H713_MIPS_SHMEM_ADDR +
			     H713_COMM_CALL_ACK_SEQ_OFF;
	ulong ret_seq = H713_MIPS_SHMEM_ADDR + H713_COMM_RET_SEQ_OFF;
	ulong ret_ack_seq = H713_MIPS_SHMEM_ADDR + H713_COMM_RET_ACK_SEQ_OFF;
	ulong fifo = seq + H713_MIPS_SEQ_FIFO_OFF;
	u32 rd, wr, cap, isz, base;
	ulong entry, slot, expect;
	u8 msg[H713_COMM_MSG_SIZE];
	uint index, i;
	int waited;
	uint rx = 0;
	bool drained = false;
	bool accepted = false;
	u16 returned_raw_nparams = 0, returned_nparams = 0;
	u32 returned_param[H713_COMM_MAX_PARAMS] = { 0 };

	if (reply)
		memset(reply, 0, sizeof(*reply));
	if (nparams > H713_COMM_MAX_PARAMS) {
		printf("H713 comm: at most %u parameters\n",
		       H713_COMM_MAX_PARAMS);
		return -EINVAL;
	}
	if (h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF) !=
	    H713_MIPS_SHMEM_MAGIC) {
		printf("H713 comm: shared memory not published this boot\n");
		return -ENODEV;
	}

	rd   = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x00);
	wr   = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x04);
	cap  = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x10);
	isz  = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x14);
	base = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x18);

	printf("H713 comm: FreeCall @+0x%05lx  rd=%u wr=%u cap=%u isz=%u "
	       "base=%08x\n", H713_COMM_CALL_SEQ_OFF, rd, wr, cap, isz, base);

	if (cap != H713_MIPS_SEQ_FIFO_CAPACITY || isz != 4 ||
	    rd >= cap || wr >= cap ||
	    base != seq + H713_MIPS_SEQ_RING_OFF) {
		printf("H713 comm: FreeCall ring is inconsistent -- refusing to send\n");
		return -EINVAL;
	}

	if (rd == wr) {
		printf("H713 comm: no free slot (ring empty)\n");
		return -EBUSY;
	}

	/* 0x8b1180ec: entry = base_addr + rd_idx * item_size */
	entry = base + rd * isz;
	invalidate_dcache_range(entry & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				(entry & ~(CONFIG_SYS_CACHELINE_SIZE - 1)) +
				CONFIG_SYS_CACHELINE_SIZE);
	slot = readl(entry);

	/* 0x8b118be8: the slot carries its own index, bounded by 20. */
	invalidate_dcache_range(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1)) +
				CONFIG_SYS_CACHELINE_SIZE);
	index = readw(slot + 0x04);

	expect = seq + H713_MIPS_SEQ_SLOTS_OFF + index * H713_COMM_MSG_SIZE;
	printf("H713 comm: slot %u @0x%08lx (index %u, expect 0x%08lx)\n",
	       rd, slot, index, expect);

	if (index >= H713_MIPS_SEQ_SLOTS || slot != expect) {
		printf("H713 comm: slot inconsistent -- refusing to send\n");
		return -EINVAL;
	}

	/*
	 * Build the message. Layout confirmed from the consumer, command_action
	 * at 0x8b120bc0: dst_cpu +0x02, slot_index +0x04, flags +0x06, parameter
	 * count in cmd_type +0x08, session +0x0C, comp_id +0x28, params +0x2C.
	 *
	 * flags carries bit 2 -- MSG_FLAG_SENT. The publish helper copies this
	 * halfword's low byte into share_seq[+0x08], and comm_handle_call at
	 * 0x8b11f0a4 returns without touching the queue unless that bit is set.
	 * Every send before this one published zero here, which is why the
	 * doorbell always drained and the message was never read.
	 */
	memset(msg, 0, sizeof(msg));
	/*
	 * chan is the full u16 at +0x00, not a byte. 0x8b11d544 compares this
	 * halfword against the channel record's +0x02 at 0x8b11d63c, so a
	 * stray 1 in the high byte fails the channel match and the CALL is
	 * rejected before command_action can even complete -- tried on
	 * hardware, and it regressed a working path.
	 */
	*(u16 *)(msg + 0x00) = (u16)chan;		/* chan / src_cpu   */
	*(u16 *)(msg + 0x02) = 1;			/* dst_cpu = MIPS   */
	*(u16 *)(msg + 0x04) = (u16)index;
	*(u16 *)(msg + 0x06) = H713_COMM_MSG_FLAG_SENT;	/* flags            */
	*(u32 *)(msg + 0x10) = pid;			/* pid              */
	*(u16 *)(msg + 0x08) = (u16)nparams;		/* cmd_type = count */
	*(u32 *)(msg + 0x0c) = 0x00000001;		/* session id       */
	*(u32 *)(msg + 0x28) = comp_id;
	for (i = 0; i < nparams; i++)
		*(u32 *)(msg + 0x2c + i * 4) = params[i];

	memcpy((void *)slot, msg, sizeof(msg));
	writew((u16)index, slot + 0x04);
	writew(2, slot + 0x0a);				/* CALL marker      */
	flush_cache(slot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
		    H713_COMM_MSG_SIZE + CONFIG_SYS_CACHELINE_SIZE);

	/* Consume the ring entry. */
	writel((rd + 1) % cap, fifo + 0x00);

	/*
	 * 0x8b11f9ec, with the wait pointer deliberately zero. The index at
	 * +0x10 must be in range before the flag at +0x08 goes up: the firmware
	 * asserts and spins on an index of 20 or more, and 20 is exactly what
	 * init leaves there. Both land in the same cache line and the flush
	 * below precedes the doorbell, so the MIPS never observes a raised flag
	 * over a stale index.
	 */
	writeb((u8)index, seq + 0x10);
	writeb(H713_COMM_MSG_FLAG_SENT, seq + 0x08);
	writel(0x00000001, seq + 0x14);			/* session          */
	writel(0, seq + 0x18);				/* wait ptr lo      */
	writel(0, seq + 0x1c);				/* wait ptr hi      */
	writel(h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x04) + 1,
	       seq + 0x04);
	flush_cache(seq & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
		    0x80 + CONFIG_SYS_CACHELINE_SIZE);

	printf("H713 comm: published index %u, comp_id 0x%08x, %u param(s), "
	       "chan=0x%x pid=0x%08x (key 0x%08x, channel slot %u)\n",
	       index, comp_id, nparams, chan, pid,
	       (pid << 4) | (chan & 0xf),
	       ((pid & 3) << 2) | (chan & 3));

	/*
	 * The msgbox has its own bus gate and reset, which Linux takes via
	 * CLK_BUS_MSGBOX/RST_BUS_MSGBOX and U-Boot has never touched. An
	 * unclocked block swallows the doorbell silently, so enable it and
	 * prove it is alive before writing: the per-sub-block version register
	 * reads 0x00020000 on a live msgbox.
	 */
	printf("H713 comm: msgbox BGR %08x, version %08x (expect 00020000), "
	       "fifo count %u\n",
	       readl(H713_COMM_MSGBOX_BGR), readl(H713_COMM_MSGBOX_VERSION),
	       readl(H713_COMM_MSGBOX_COUNT));

	/*
	 * H713's msgbox is edge-triggered, unlike H6's level-triggered one, so
	 * writing MSG_DATA alone leaves the message sitting in the FIFO without
	 * waking the receiver -- observed directly: the count went 0 -> 1 and
	 * the MIPS never drained it. The receiver needs a TX_IRQ_EN pulse.
	 *
	 * TX_IRQ_EN is at sub-block +0x30 and the bit is BIT(2*port + 1); the
	 * Linux tree's ARISC path pulses BIT(7) for port 3, which is the same
	 * rule. MIPS is port 1, so BIT(3).
	 */
	writel(H713_COMM_DOORBELL_CALL, H713_COMM_DOORBELL);
	writel(BIT(3), H713_COMM_MSGBOX_TX_IRQ_EN);
	udelay(10);
	writel(0, H713_COMM_MSGBOX_TX_IRQ_EN);
	udelay(100);
	printf("H713 comm: doorbell 0x%08x rung + IRQ pulsed; fifo count now "
	       "%u\n", H713_COMM_DOORBELL_CALL,
	       readl(H713_COMM_MSGBOX_COUNT));

	/* Poll the return transport rather than waiting to be signalled. */
	for (waited = 0; waited < 2000; waited++) {
		u32 ridx = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x10) &
			   0xff;
		u32 rstate = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x08) &
			     0xff;
		u32 fc = readl(H713_COMM_MSGBOX_COUNT);
		u32 state = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x08) &
			    0xff;

		if (!fc && !drained) {
			printf("H713 comm: MIPS drained the FIFO after %d ms\n",
			       waited);
			drained = true;
		}

		/*
		 * comm_handle_call clears bit 2 at 0x8b11f2ec once it accepts
		 * the message. That is the first thing the firmware does with
		 * our shared memory, and it happens well before any reply, so
		 * report it on its own -- a run that gets this far but never
		 * replies has still proven the message was read.
		 */
		if (!accepted && !(state & H713_COMM_MSG_FLAG_SENT)) {
			printf("H713 comm: firmware accepted the message after "
			       "%d ms (state bit 2 cleared)\n", waited);
			accepted = true;
		}

		/*
		 * Drain the inbound mailbox. The firmware acknowledges over the
		 * msgbox using the same bare-type encoding: 0x8b11eadc maps a
		 * received CALL to 2 (CALL_ACK) and a RETURN to 3 (RETURN_ACK),
		 * and sends it through 0x8b121ddc. Nothing on the ARM side ever
		 * read this FIFO, so the first CALL_ACK sat here unnoticed.
		 * It has capacity 8; leaving it full would eventually block the
		 * firmware's sender, which polls for space at 0x8b121a68.
		 */
		while (readl(H713_COMM_MSGBOX_RX_COUNT)) {
			u32 w = readl(H713_COMM_MSGBOX_RX_DATA);
			static const char * const t[] = {
				"CALL", "RETURN", "CALL_ACK", "RETURN_ACK"
			};

			printf("H713 comm: <- msgbox 0x%08x (%s) after %d ms\n",
			       w, w < 4 ? t[w] : "unknown", waited);
			rx++;

			/*
			 * Mirror comm_handle_CPU2_callACK. SendAckLow publishes ACK
			 * metadata in share_seq(0,1,0) at +0x68..+0x74,
			 * raises +0x69 bit 2, then sends mailbox type 2. The stock ARM
			 * interrupt handler clears that bit before queueing ack_action.
			 *
			 * Merely draining the mailbox leaves the MIPS publication live.
			 * Its next SendAckLow checks +0x69 at 0x8b120964 and assert-spins
			 * when bit 2 is still set -- exactly why CALL #2 reached the
			 * "sending CALL_ACK" trace marker but emitted no ACK. U-Boot
			 * polls instead of sleeping on the published wait pointer, so
			 * consuming the validated publication is the complete inline
			 * CALL_ACK action.
			 */
			if (w == 2) {
				u32 ack_index = h713_mips_read_shmem(
					H713_COMM_CALL_ACK_SEQ_OFF + 0x68) & 0xff;
				u32 ack_state = h713_mips_read_shmem(
					H713_COMM_CALL_ACK_SEQ_OFF + 0x69) & 0xff;
				u32 ack_session = h713_mips_read_shmem(
					H713_COMM_CALL_ACK_SEQ_OFF + 0x6c);

				printf("H713 comm: CALL_ACK metadata index=%u "
				       "session=%08x state=%02x\n", ack_index,
				       ack_session, ack_state);
				if (ack_index != H713_MIPS_SEQ_SLOTS ||
				    ack_session != 1 ||
				    !(ack_state & H713_COMM_MSG_FLAG_SENT)) {
					printf("H713 comm: CALL_ACK metadata is "
					       "inconsistent; refusing to consume it\n");
					return -EPROTO;
				}

				writeb(ack_state & ~H713_COMM_MSG_FLAG_SENT,
				       call_ack_seq + 0x69);
				flush_cache((call_ack_seq + 0x69) &
					    ~(CONFIG_SYS_CACHELINE_SIZE - 1),
					    CONFIG_SYS_CACHELINE_SIZE);
				printf("H713 comm: CALL_ACK publication consumed\n");
			}
		}

		if (ridx < H713_MIPS_SEQ_SLOTS &&
		    (rstate & H713_COMM_MSG_FLAG_SENT)) {
			ulong rslot = ret_seq + H713_MIPS_SEQ_SLOTS_OFF +
				      ridx * H713_COMM_MSG_SIZE;
			u32 session, returned_comp;
			ulong staging = ret_seq + 0x20;
			u32 srd, swr, scap, sisz, sbase, snext, speak, scount;
			ulong sentry;
			ulong rfifo = ret_seq + H713_MIPS_SEQ_FIFO_OFF;
			u32 rrd, rwr, rcap, risz, rbase, next;
			u32 ack_wait_lo, ack_wait_hi;
			ulong rent;
			int ack_waited, action_waited;

			invalidate_dcache_range(
				rslot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				(rslot & ~(CONFIG_SYS_CACHELINE_SIZE - 1)) +
				H713_COMM_MSG_SIZE + CONFIG_SYS_CACHELINE_SIZE);
			session = readl(rslot + 0x0c);
			returned_comp = readl(rslot + 0x28);
			returned_raw_nparams = readw(rslot + 0x08);
			returned_nparams = returned_raw_nparams == 0xf ? 0 :
					   returned_raw_nparams;
			printf("H713 comm: reply after %d ms, slot %u\n",
			       waited, ridx);
			printf("  session=%08x comp_id=%08x nret=%u%s\n",
			       session, returned_comp, returned_nparams,
			       returned_raw_nparams == 0xf ?
			       " (raw 0xf no-output sentinel)" : "");

			if (session != 1 || returned_comp != comp_id) {
				printf("H713 comm: RETURN does not match this CALL; "
				       "refusing to acknowledge or recycle it\n");
				return -EPROTO;
			}
			if (returned_nparams > H713_COMM_MAX_PARAMS) {
				printf("H713 comm: RETURN has too many parameters; "
				       "refusing to acknowledge or recycle it\n");
				return -EPROTO;
			}
			for (i = 0; i < returned_nparams; i++) {
				returned_param[i] = readl(rslot + 0x2c + i * 4);
				printf("  ret[%u]=%08x\n", i, returned_param[i]);
			}

			/*
			 * Mirror comm_handle_return + command_action. AddReturn2Fifo
			 * publishes the received slot into the ARM-owned ReturnCmd FIFO.
			 * The ARM return action normally consumes that FIFO entry before
			 * acknowledging it. U-Boot handles the return synchronously, so it
			 * must advance its own read index after the ACK succeeds; the MIPS
			 * sender does not own or update this receiver-local FIFO.
			 */
			srd = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x20);
			swr = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x24);
			scap = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x30);
			sisz = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x34);
			sbase = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x38);
			if (scap != H713_MIPS_SEQ_FIFO_CAPACITY || sisz != 4 ||
			    srd >= scap || swr >= scap ||
			    sbase != (u32)(uintptr_t)h713_comm_staging_ring[1][1]) {
				printf("H713 comm: ReturnCmd FIFO is inconsistent; "
				       "refusing RETURN slot %u\n", ridx);
				return -EINVAL;
			}
			snext = (swr + 1) % scap;
			if (snext == srd) {
				printf("H713 comm: ReturnCmd FIFO is full; "
				       "refusing RETURN slot %u\n", ridx);
				return -ENOSPC;
			}
			speak = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF + 0x28);
			scount = snext >= srd ? snext - srd : snext + scap - srd;
			sentry = sbase + swr * sisz;
			writel(rslot, sentry);
			flush_cache(sentry & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    CONFIG_SYS_CACHELINE_SIZE);
			writew(readw(rslot + 0x0a) | 0x10, rslot + 0x0a);
			flush_cache(rslot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    H713_COMM_MSG_SIZE + CONFIG_SYS_CACHELINE_SIZE);
			writel(snext, staging + 0x04);
			if (scount > speak)
				writel(scount, staging + 0x08);
			flush_cache(staging & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    CONFIG_SYS_CACHELINE_SIZE);
			printf("H713 comm: ReturnCmd queued slot %u "
			       "(rd=%u wr=%u -> %u)\n", ridx, srd, swr, snext);

			/*
			 * Accept the published RETURN and restore the empty index
			 * sentinel. SendAckLow writes acknowledgement metadata into the
			 * opposite-direction share_seq at +0x68..+0x74 and raises
			 * doorbell type 3. The wait pointer belongs to the MIPS sender;
			 * copying it back lets ack_action wake SendComm2CPUEx.
			 */
			writeb(rstate & ~H713_COMM_MSG_FLAG_SENT, ret_seq + 0x08);
			writeb(H713_MIPS_SEQ_SLOTS, ret_seq + 0x10);
			flush_cache(ret_seq & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    0x80 + CONFIG_SYS_CACHELINE_SIZE);

			ack_wait_lo = h713_mips_read_shmem(
				H713_COMM_RET_SEQ_OFF + 0x18);
			ack_wait_hi = h713_mips_read_shmem(
				H713_COMM_RET_SEQ_OFF + 0x1c);
			printf("H713 comm: RETURN_ACK metadata session=%08x "
			       "wait=%08x:%08x\n", session, ack_wait_hi,
			       ack_wait_lo);

			writeb(H713_MIPS_SEQ_SLOTS, ret_ack_seq + 0x68);
			writeb(rstate & ~H713_COMM_MSG_FLAG_SENT,
			       ret_ack_seq + 0x69);
			writel(session, ret_ack_seq + 0x6c);
			writel(ack_wait_lo, ret_ack_seq + 0x70);
			writel(ack_wait_hi, ret_ack_seq + 0x74);
			flush_cache(ret_ack_seq, 0x80);
			writeb(rstate | H713_COMM_MSG_FLAG_SENT,
			       ret_ack_seq + 0x69);
			flush_cache(ret_ack_seq + 0x40, 0x40);

			writel(H713_COMM_DOORBELL_RETURN_ACK, H713_COMM_DOORBELL);
			writel(BIT(3), H713_COMM_MSGBOX_TX_IRQ_EN);
			udelay(10);
			writel(0, H713_COMM_MSGBOX_TX_IRQ_EN);

			for (ack_waited = 0; ack_waited < 100; ack_waited++) {
				if (!(h713_mips_read_shmem(
					      H713_COMM_RET_ACK_SEQ_OFF + 0x69) &
				      H713_COMM_MSG_FLAG_SENT))
					break;
				mdelay(1);
			}
			printf("H713 comm: RETURN_ACK publication %s after %d ms\n",
			       ack_waited < 100 ? "consumed by interrupt handler" :
			       "not consumed", ack_waited);
			if (ack_waited >= 100) {
				printf("H713 comm: preserving the unreconciled "
				       "ReturnCmd and FreeReturn state\n");
				return -ETIMEDOUT;
			}

			/*
			 * Clearing +0x69 happens in the low-level interrupt handler,
			 * before queueAction runs the RETURN_ACK action that posts the
			 * sender's wait semaphore. In comm-trace mode, wait for the
			 * latter event and the sender's post-ACK cleanup. This is both a
			 * sharper diagnostic and
			 * prevents a falsely successful first call from hiding a MIPS
			 * sender that is still blocked when the second CALL is queued.
			 */
			if (h713_comm_trace_active) {
				u32 action_stage = 0;

				for (action_waited = 0; action_waited < 1000;
				     action_waited++) {
					action_stage = h713_mips_read_shmem(
						H713_MIPS_COMM_TRACE_STAGE_OFF);
					if (action_stage == 0xc013 ||
					    action_stage == 0xd103 ||
					    action_stage == 0xc111)
						break;
					mdelay(1);
				}
				if (action_stage != 0xc013) {
					printf("H713 comm: MIPS sender did not complete "
					       "RETURN after %d ms\n",
					       action_waited);
					h713_mips_print_comm_trace();
					printf("H713 comm: preserving the "
					       "unreconciled ReturnCmd and "
					       "FreeReturn state\n");
					return (action_stage == 0xd103 ||
						action_stage == 0xc111) ? -EIO :
						-ETIMEDOUT;
				}
				printf("H713 comm: MIPS sender completed RETURN "
				       "after %d ms\n", action_waited);
			}

			/* Complete the receiver-owned ARM ReturnCmd action. */
			writel(snext, staging + 0x00);
			flush_cache(staging & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    CONFIG_SYS_CACHELINE_SIZE);
			printf("H713 comm: ReturnCmd consumed slot %u "
			       "(rd=%u -> %u wr=%u)\n", ridx, srd, snext, snext);

			/* Return the consumed slot to the FreeReturn ring. */
			rrd = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF +
						    H713_MIPS_SEQ_FIFO_OFF + 0x00);
			rwr = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF +
						    H713_MIPS_SEQ_FIFO_OFF + 0x04);
			rcap = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF +
						     H713_MIPS_SEQ_FIFO_OFF + 0x10);
			risz = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF +
						     H713_MIPS_SEQ_FIFO_OFF + 0x14);
			rbase = h713_mips_read_shmem(H713_COMM_RET_SEQ_OFF +
						      H713_MIPS_SEQ_FIFO_OFF + 0x18);
			if (rcap != H713_MIPS_SEQ_FIFO_CAPACITY || risz != 4 ||
			    rrd >= rcap || rwr >= rcap ||
			    rbase != ret_seq + H713_MIPS_SEQ_RING_OFF) {
				printf("H713 comm: FreeReturn ring is inconsistent; "
				       "refusing to recycle slot %u\n", ridx);
				return -EINVAL;
			}
			if ((rwr + 1) % rcap == rrd) {
				printf("H713 comm: FreeReturn ring is full; "
				       "cannot recycle slot %u\n", ridx);
				return -ENOSPC;
			}
			next = (rwr + 1) % rcap;
			rent = rbase + rwr * risz;
			writew(0, rslot + 0x0a);
			flush_cache(rslot & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    H713_COMM_MSG_SIZE + CONFIG_SYS_CACHELINE_SIZE);
			writel(rslot, rent);
			flush_cache(rent & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    CONFIG_SYS_CACHELINE_SIZE);
			writel(next, rfifo + 0x04);
			flush_cache(rfifo & ~(CONFIG_SYS_CACHELINE_SIZE - 1),
				    CONFIG_SYS_CACHELINE_SIZE);
			printf("H713 comm: FreeReturn slot %u recycled "
			       "(rd=%u wr=%u -> %u)\n", ridx, rrd, rwr, next);
			if (reply) {
				reply->nparams = returned_nparams;
				memcpy(reply->param, returned_param,
				       returned_nparams * sizeof(returned_param[0]));
			}

			return 0;
		}
		mdelay(1);
	}

	{
		u32 pidx = h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x10) &
			   0xff;

		printf("H713 comm: no reply within 2000 ms (fifo count %u, %s; "
		       "message %s; %u inbound msgbox word(s); "
		       "published idx %s)\n",
		       readl(H713_COMM_MSGBOX_COUNT),
		       drained ? "was drained" : "never drained",
		       accepted ? "accepted" : "never accepted", rx,
		       pidx == index ? "still ours -- not consumed"
				     : "changed -- consumed");
	}
	printf("  FreeCall  rd=%u wr=%u  idx=%02x state=%02x\n",
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x00),
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF +
				    H713_MIPS_SEQ_FIFO_OFF + 0x04),
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x10) & 0xff,
	       h713_mips_read_shmem(H713_COMM_CALL_SEQ_OFF + 0x08) & 0xff);
	if (h713_comm_trace_active)
		h713_mips_print_comm_trace();
	return -ETIMEDOUT;
}

/*
 * Exercise input and output marshalling without intentionally changing the
 * display. The getter returns the current live picture-mode word. Passing that
 * exact word to the setter takes its statically verified equality branch, so
 * no hardware update is issued. A final getter proves both the input arrived
 * intact and the output path remains usable after the setter transaction.
 */
static int h713_comm_picture_mode_test(u32 chan, u32 pid)
{
	struct h713_comm_reply before, set_reply, after;
	u32 picture_mode;
	int ret;

	ret = h713_comm_validate_picture_mode_routines(pid);
	if (ret)
		return ret;

	ret = h713_comm_call(H713_COMM_GET_PICTURE_MODE_ID, NULL, 0,
			     chan, pid, &before);
	if (ret)
		return ret;
	if (before.nparams != 1) {
		printf("H713 comm: GetPictureMode returned %u word(s), expected 1\n",
		       before.nparams);
		return -EPROTO;
	}
	picture_mode = before.param[0];
	printf("H713 comm: current picture mode = 0x%08x; "
	       "sending the same value back\n", picture_mode);

	ret = h713_comm_call(H713_COMM_SET_PICTURE_MODE_ID, &picture_mode, 1,
			     chan, pid, &set_reply);
	if (ret)
		return ret;
	if (set_reply.nparams != 0) {
		printf("H713 comm: SetPictureMode returned %u word(s), expected 0\n",
		       set_reply.nparams);
		return -EPROTO;
	}

	ret = h713_comm_call(H713_COMM_GET_PICTURE_MODE_ID, NULL, 0,
			     chan, pid, &after);
	if (ret)
		return ret;
	if (after.nparams != 1 || after.param[0] != picture_mode) {
		printf("H713 comm: picture mode did not round-trip: "
		       "nret=%u value=%08x expected=%08x\n",
		       after.nparams, after.param[0], picture_mode);
		return -EPROTO;
	}

	printf("H713 comm: picture-mode marshalling PASS "
	       "(get=%08x set-same get=%08x)\n",
	       picture_mode, after.param[0]);
	return 0;
}

/*
 * Read-only inspection of the eight share_seq transports.
 *
 * U-Boot builds each one with rd_idx=0 and wr_idx=20 -- a ring holding twenty
 * free slots in a capacity of twenty-one. If the firmware has allocated or
 * consumed any, those indices will have moved, which is the difference between
 * "the firmware tolerated our structures" and "the firmware is using them".
 *
 * Makes no writes and sends no messages, so it does not consume the
 * one-launch-per-power-cycle budget.
 */
/*
 * Read the firmware's channel table.
 *
 * A received CALL is looked up by Comm_GetCallbyChannel, 0x8b11ccec:
 *
 *   key  = (pid << 4) | (chan & 0xf)          pid = msg[+0x10], chan = msg[+0x00]
 *   idx  = ((key >> 2) & 0xc) | (key & 3)     i.e. (pid & 3) << 2 | (chan & 3)
 *   rec  = pcpu_comm_dev + 0x18 + 0x90 + 48*idx
 *   hit  = rec[+0x0c] == key
 *
 * and 0x8b11d544 then re-checks rec[+0x02] against chan and rec[+0x08] against
 * pid before accepting. Channels are created by 0x8b122554 from the routine
 * registration path at 0x8b12486c, so the firmware's own 82 registrations
 * should have populated this table -- with whatever chan/pid they chose. That
 * is the value we cannot derive statically and can read here.
 *
 * Read-only, no messages, does not consume a launch.
 */
#define H713_COMM_DEV_PTR	0x8b22efe4UL	/* -> pcpu_comm_dev           */
#define H713_COMM_CHAN_OFF	0xa8UL		/* dev + 0x18 + 0x90          */
#define H713_COMM_CHAN_STRIDE	48
#define H713_COMM_CHAN_SLOTS	16

static int h713_disp_comm_dev(void)
{
	ulong dev, tbl;
	uint i, live = 0;

	if (h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF) !=
	    H713_MIPS_SHMEM_MAGIC) {
		printf("H713 comm: shared memory not published this boot\n");
		return -ENODEV;
	}

	/*
	 * Prove the KSEG reader before trusting anything it says. getCurCPUID
	 * at 0x8b1227b4 is two instructions we know byte for byte from the
	 * authenticated display.bin: `jr $ra` then `addiu $v0, $zero, 1`. If
	 * these do not match, every other number below is meaningless.
	 */
	{
		u32 a = h713_mips_read_fw(0x8b1227b4UL);
		u32 b = h713_mips_read_fw(0x8b1227b8UL);
		bool ok = a == 0x03e00008 && b == 0x24020001;

		printf("H713 comm: reader self-test @0x8b1227b4 -> %08x %08x "
		       "(expect 03e00008 24020001) %s\n",
		       a, b, ok ? "OK" : "*** FAILED ***");
		if (!ok)
			printf("H713 comm: KSEG reads are not landing where "
			       "expected -- treat the dump below as void\n");
	}

	dev = h713_mips_read_fw(H713_COMM_DEV_PTR);
	printf("H713 comm: pcpu_comm_dev = 0x%08lx (via 0x%08lx)\n",
	       dev, H713_COMM_DEV_PTR);

	if (dev < 0x80000000UL || dev >= 0xa0000000UL) {
		printf("H713 comm: dev pointer not a KSEG address -- "
		       "firmware not up?\n");
		return -ENODEV;
	}

	tbl = dev + H713_COMM_CHAN_OFF;
	printf("H713 comm: channel table at 0x%08lx (ARM 0x%08lx), "
	       "%u slots x %u bytes\n", tbl,
	       (tbl & 0x1fffffffUL) + 0x40000000UL,
	       H713_COMM_CHAN_SLOTS, H713_COMM_CHAN_STRIDE);

	for (i = 0; i < H713_COMM_CHAN_SLOTS; i++) {
		ulong r = tbl + i * H713_COMM_CHAN_STRIDE;
		u32 w0 = h713_mips_read_fw(r + 0x00);
		u32 chan = (w0 >> 16) & 0xffff;
		u32 pid = h713_mips_read_fw(r + 0x08);
		u32 key = h713_mips_read_fw(r + 0x0c);
		/* 0xffffffff is the free sentinel, same as the call table. */
		bool used = key != 0xffffffff && (key || pid || w0);

		if (used)
			live++;

		/*
		 * Everything here prints as hex because `commcall chan=/pid=`
		 * parses hex. They were decimal once and it cost a bench run:
		 * pid 0x8b8f32b0 printed as 2341417648, which hextoul then
		 * read back as 0x41417648.
		 */
		printf("  [%2u] key=%08x  chan=%04x pid=%08x  "
		       "+00=%08x +04=%08x sem=%08x%s\n",
		       i, key, chan, pid, w0,
		       h713_mips_read_fw(r + 0x04),
		       h713_mips_read_fw(r + 0x10),
		       used ? "  <== LIVE" : "");

		/*
		 * record[+0x10] is the channel's call semaphore. 0x8b11d544
		 * posts it via osal_semaphore_set (0x8b15c1d0) the moment the
		 * channel lookup succeeds, and if that post returns non-zero
		 * the caller spins forever at 0x8b11d850. So the object behind
		 * this handle decides whether a correctly addressed CALL
		 * completes or wedges the receive thread -- dump enough of it
		 * to tell. ThreadX tags a semaphore with 'SEMA' at +0x00 and
		 * keeps the count at +0x08.
		 */
		if (used) {
			ulong sem = h713_mips_read_fw(r + 0x10);
			uint k;

			if (sem >= 0x80000000UL && sem < 0xa0000000UL) {
				/*
				 * Dump a window rather than three named fields.
				 * The first attempt printed id/name/count as
				 * three reads and got the pointer's own value
				 * back from all of them, which is either a
				 * bogus object or a broken read -- a window
				 * tells those apart, because a broken read
				 * repeats and a real structure varies.
				 */
				printf("       sem@%08lx:", sem);
				for (k = 0; k < 12; k++) {
					if (k && !(k % 6))
						printf("\n                 ");
					printf(" %08x",
					       h713_mips_read_fw(sem + k * 4));
				}
				printf("\n");
			} else {
				printf("       sem handle 0x%08lx is not a "
				       "KSEG address\n", sem);
			}
		}
	}

	printf("H713 comm: %u of %u channel slot(s) live "
	       "(0xffffffff = free)\n", live, H713_COMM_CHAN_SLOTS);
	printf("H713 comm: retry a CALL with "
	       "'commcall <id> chan=<chan> pid=<pid>' using a LIVE row above; "
	       "all values hex\n");
	return 0;
}

/*
 * Dump firmware memory by MIPS virtual address, with the cache invalidated.
 *
 * `md` cannot do this job: it reads through the ARM's cache and returns the
 * zeros U-Boot left before the coprocessor started. This is the general form
 * of what commdev does for one structure, so that chasing a pointer into the
 * firmware does not cost a reflash each time.
 *
 * Read-only. Any KSEG0/KSEG1 address is accepted; nothing else is.
 */
static int h713_disp_fw_md(ulong va, uint words)
{
	uint i;

	if (va < 0x80000000UL || va >= 0xa0000000UL) {
		printf("H713 fw: 0x%08lx is not a KSEG address "
		       "(expect 0x8xxxxxxx/0x9xxxxxxx)\n", va);
		return -EINVAL;
	}
	if (!words || words > 256)
		words = 16;

	printf("H713 fw: %u word(s) at 0x%08lx (ARM 0x%08lx)\n",
	       words, va, (va & 0x1fffffffUL) + 0x40000000UL);

	for (i = 0; i < words; i++) {
		if (!(i % 6))
			printf("%s0x%08lx:", i ? "\n" : "", va + i * 4);
		printf(" %08x", h713_mips_read_fw(va + i * 4));
	}
	printf("\n");
	return 0;
}

static int h713_disp_comm_state(void)
{
	uint cpu, dir, idx, live = 0;

	if (h713_mips_read_shmem(H713_MIPS_SHMEM_MAGIC1_OFF) !=
	    H713_MIPS_SHMEM_MAGIC) {
		printf("H713 comm: shared memory not published this boot\n");
		return -ENODEV;
	}

	printf("H713 comm: share_seq transports "
	       "(as built: rd=0 wr=%u cap=%u)\n",
	       H713_MIPS_SEQ_SLOTS, H713_MIPS_SEQ_FIFO_CAPACITY);

	for (cpu = 0; cpu < 2; cpu++)
	for (dir = 0; dir < 2; dir++)
	for (idx = 0; idx < 2; idx++) {
		ulong off = H713_MIPS_SEQ_BASE_OFF +
			    cpu * H713_MIPS_SEQ_PER_CPU +
			    dir * H713_MIPS_SEQ_PER_DIR +
			    idx * H713_MIPS_SEQ_STRIDE;
		ulong f = off + H713_MIPS_SEQ_FIFO_OFF;
		u32 rd = h713_mips_read_shmem(f + 0x00);
		u32 wr = h713_mips_read_shmem(f + 0x04);
		u32 peak = h713_mips_read_shmem(f + 0x08);
		u32 cap = h713_mips_read_shmem(f + 0x10);
		u32 base = h713_mips_read_shmem(f + 0x18);
		char name[0x14];
		uint i;
		bool moved;

		for (i = 0; i < sizeof(name) - 1; i++)
			name[i] = (char)(h713_mips_read_shmem(off + 0x98 +
					 (i & ~3)) >> ((i & 3) * 8));
		name[sizeof(name) - 1] = 0;

		moved = rd != 0 || wr != H713_MIPS_SEQ_SLOTS;
		if (moved)
			live++;

		printf("  cpu=%u dir=%u idx=%u %-11s @+0x%05lx  "
		       "rd=%-3u wr=%-3u peak=%-3u cap=%-3u base=%08x%s\n",
		       cpu, dir, idx, name, off, rd, wr, peak, cap, base,
		       moved ? "  <== MOVED" : "");

		/*
		 * The staging FIFO at share_seq+0x20 -- "CallCmd" / "ReturnCmd"
		 * in the Linux driver's terms, name at +0x40. This is the queue
		 * 0x8b11d544 allocates from when command_action hands a received
		 * CALL to the firmware's worker, and it is separate from the
		 * FreeCall ring at +0x78. Its base_addr is the *receiver's* own
		 * virtual address, so for share_seq(1,0,0) it should point into
		 * MIPS memory and only the MIPS can populate it. A capacity of
		 * zero means 0x8b11825c fails and the CALL is dropped -- and
		 * 0x8b1212a0 never checks that return, so the CALL_ACK still
		 * goes out. Print it so that case is visible instead of silent.
		 */
		{
			ulong s = off + 0x20;
			u32 srd = h713_mips_read_shmem(s + 0x00);
			u32 swr = h713_mips_read_shmem(s + 0x04);
			u32 scap = h713_mips_read_shmem(s + 0x10);
			u32 sisz = h713_mips_read_shmem(s + 0x14);
			u32 sbase = h713_mips_read_shmem(s + 0x18);
			char sname[0x14];

			for (i = 0; i < sizeof(sname) - 1; i++)
				sname[i] = (char)(h713_mips_read_shmem(off +
						  0x40 + (i & ~3)) >>
						  ((i & 3) * 8));
			sname[sizeof(sname) - 1] = 0;

			printf("      staging +0x20 %-11s rd=%-3u wr=%-3u "
			       "cap=%-3u isz=%-3u base=%08x%s\n",
			       sname[0] ? sname : "(unnamed)",
			       srd, swr, scap, sisz, sbase,
			       scap ? "" : "  <== UNINITIALISED");
		}
	}

	printf("H713 comm: %u of 8 transport(s) show movement\n", live);
	if (!live)
		printf("H713 comm: firmware accepted the structures but has not "
		       "used them; a send would be the first traffic\n");
	return 0;
}

/*
 * Read-only inspection of the live call table. This makes no writes and sends
 * no messages, so it does not count as a second MIPS launch: run it at the
 * prompt after panel-test or mips-test, on the same boot, while the firmware
 * is still up.
 *
 * Two outputs. First, any entry word matching a known routine id, with the
 * offset it was found at -- that locates the id field within the 0x60-byte
 * entry and confirms the hash convention against this firmware. Second, raw
 * hex for the first few populated entries, so the layout can be worked out
 * offline even if nothing matches.
 */
static int h713_disp_call_table(uint raw_entries)
{
	u32 version, count;
	uint i, w, shown = 0, populated = 0, matched = 0;
	char name[H713_MIPS_SHMEM_CALL_ENTRY_SIZE - 12];

	version = h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_VERSION_OFF);
	count   = h713_mips_read_shmem(H713_MIPS_SHMEM_CALL_COUNT_OFF);

	printf("H713 comm: call table @0x%08lx  version=%u count=%u\n",
	       H713_MIPS_SHMEM_ADDR + H713_MIPS_SHMEM_CALL_TABLE_OFF,
	       version, count);

	if (!count || count > H713_MIPS_SHMEM_CALL_ENTRY_COUNT) {
		printf("H713 comm: table not populated -- run the firmware "
		       "first, on this boot\n");
		return -ENODEV;
	}

	for (i = 0; i < H713_MIPS_SHMEM_CALL_ENTRY_COUNT; i++) {
		ulong off = H713_MIPS_SHMEM_CALL_TABLE_OFF +
			    i * H713_MIPS_SHMEM_CALL_ENTRY_SIZE;
		u32 word[H713_MIPS_SHMEM_CALL_ENTRY_SIZE / 4];
		bool any = false;

		for (w = 0; w < ARRAY_SIZE(word); w++) {
			word[w] = h713_mips_read_shmem(off + w * 4);
			/* The free sentinel is not content. */
			if (word[w] && !(w * 4 == H713_MIPS_SHMEM_CALL_NEXT_OFF &&
					 word[w] == ~0U))
				any = true;
		}
		if (!any)
			continue;
		populated++;

		/*
		 * The entry names itself. Layout, settled from the first good
		 * dump on 2026-08-04:
		 *
		 *   +0x00  0x00010000        version/flags
		 *   +0x04  pid               0x8b8f32b0 for every entry so far
		 *   +0x08  routine id        what commcall takes
		 *   +0x0c  ASCII name, NUL-terminated ("THal_Vp_Deinit_1_000")
		 *   +0x50  handler VA
		 *   +0x5c  0xffffffff        free sentinel
		 *
		 * Reading the name out of the entry beats matching against a
		 * built-in list: the firmware exports 1224 slots and the list
		 * knew fifteen, so everything else dumped as anonymous hex and
		 * the ids we actually needed -- Deinit, the backlight pair --
		 * were sitting in plain ASCII the whole time.
		 */
		name[0] = '\0';
		for (w = 0; w < sizeof(name) - 1; w++) {
			u8 c = (word[3 + w / 4] >> ((w % 4) * 8)) & 0xff;

			if (!c)
				break;
			name[w] = (c >= 0x20 && c < 0x7f) ? c : '?';
		}
		name[w] = '\0';

		if (name[0]) {
			printf("  entry %4u  id %08x  handler %08x  %s\n",
			       i, word[2], word[20], name);
			matched++;
		}

		if (shown < raw_entries) {
			printf("  entry %4u raw:", i);
			for (w = 0; w < ARRAY_SIZE(word); w++) {
				if (!(w % 8))
					printf("\n    +0x%02x:", w * 4);
				printf(" %08x", word[w]);
			}
			printf("\n");
			shown++;
		}
	}

	printf("H713 comm: %u populated entr%s, %u named\n",
	       populated, populated == 1 ? "y" : "ies", matched);
	if (!matched)
		printf("H713 comm: nothing named -- the raw dumps above are the "
		       "input for working out the entry layout offline\n");
	return 0;
}

/*
 * Bring the display up and stop, leaving everything clocked.
 *
 * The register diagnostics -- scanrate, regscan, clkfind -- all need a live,
 * clocked display, which until now meant running a full panel-test first and
 * sitting through its generator phases just to reach a usable prompt. That also
 * conflated two unrelated things: the bring-up sequence under test, and the
 * measurement being taken.
 *
 * This runs the bring-up alone. `quiesce` additionally parks the MIPS core the
 * way the generator modes do, which is what the register diagnostics want:
 * with the firmware halted, nothing rewrites display registers underneath a
 * measurement or a perturbation.
 */
/*
 * elog_level >= 0 turns the coprocessor's own logging on before it is
 * released. It is opt-in because it is not free: the firmware's ring is
 * 100 KiB and it stops logging once full, so a raised level without a reader
 * draining it is worse than the default -- see doku/63-mips-elog.md.
 *
 * Mode is forced to 1, the 100 KiB ring, which is the one that can be read
 * back from Linux. Mode 2 is deliberately not offered: the knowledge base
 * records that enabling it may break MIPS init.
 */
/*
 * This is the hand-over point, and what it owes the next stage is exact.
 *
 * The vendor driver's first mips_reset() takes an "already powered" branch: it
 * touches no reset line, records mips_status = RUNNING, and asks no questions.
 * So the state we leave behind is the state Linux believes in, and there are
 * two halves to leaving it right.
 *
 * The core must be running. Hand over with it in reset and the driver still
 * reports RUNNING, and the first ioctl start returns 0 without doing anything
 * -- a trap with no error message anywhere. That is what the default path
 * gives: release_mips true, no quiesce.
 *
 * And 0x03061024/0x03061028/0x03061030 must hold the share address, the share
 * size and the firmware's entry point, because the driver reads those three
 * back rather than deriving them (A7 report, 5). They are written and verified
 * in h713_mips_release_reset(), which this path reaches with publish_shmem
 * true: h713_disp_run() is called below with prove_ready set.
 *
 * "noboot" and "quiesce" deliberately break both halves. They are bench modes
 * for the register diagnostics -- nothing may follow them but a power cycle,
 * and in particular not a kernel.
 */
/*
 * logo: put the boot logo on the panel WITHOUT parking the MIPS.
 *
 * `auto <id> logo` renders the logo and then quiesces the coprocessor as its
 * last act, which is why an installed HY310 boots with `init` and a blank
 * panel: Linux needs the firmware alive for cpu_comm, SetSource and the HDMI
 * switch (doku/67, doku/124). This path does what the KMS driver does a
 * second later with its console: fill the OSD framebuffer, and after the
 * firmware has proven readiness re-arm AFBD channel 1 with the same status
 * clear, control bit and READY write. No DE replay, no quiesce, no timing
 * latch beyond the one init already does. The buffer is filled before the
 * firmware starts so the first frame DE block 5 scans out is already the
 * logo, not whatever DRAM held.
 *
 * A missing or refused logo is a warning, never a failed init: the boot must
 * go on exactly as without the word "logo".
 */
static int h713_disp_init_only(u32 project, bool release_mips, bool quiesce,
			       int elog_level, bool logo, const char *logo_file)
{
	bool logo_up = false;
	int ret;

	/*
	 * This path launches the firmware too, so it owes the same one-launch
	 * marker that panel-test and auto set. Without it a following `auto`
	 * finds the flag clear, skips the teardown, and loads display.bin on
	 * top of a running coprocessor -- which keeps writing into the image
	 * while it is being hashed, so the digest matches no build at all.
	 */
	h713_panel_test_ran = true;

	ret = h713_disp_load(project);
	if (ret)
		return h713_disp_fail(ret);

	if (elog_level >= 0) {
		printf("H713 disp: firmware log level %d, ring mode 1\n",
		       elog_level);
		/*
		 * Only two fields are touched, and mode stays at 1 on purpose.
		 *
		 * display_cfg.xml documents mode as 0 sync / 1 async / 2 buf,
		 * but the value goes straight into the firmware's dispatch byte
		 * at 0x8B48BE9B: 1 selects elog_mode1_ring100k_write, the
		 * 100 KiB ring at 0x8B272D9C that tools/mipslog.c reads. 0 ends
		 * up in elog_default_dispatch and a different 120 KiB ring
		 * whose address we have not located, and 2 is the 2 MiB linear
		 * buffer the knowledge base says may break MIPS init. "sync"
		 * does not mean UART here -- every mode writes to memory.
		 *
		 * So: make sure output is on, raise the level, leave the rest.
		 */
		h713_cfg_set_tag("output", '1', "elog output");
		h713_cfg_set_tag("mode", '1', "elog mode");
		h713_cfg_set_tag("level", '0' + elog_level, "elog level");
	}

	/*
	 * Select the panel before anything is sized from it: the BMP check
	 * and the OSD geometry read h713_disp_panel, which until the lookup
	 * still points at the default board B (1280x720). Publishing before
	 * the lookup refused the HY310's own 1920x1080 logo on the first
	 * device run of this path (15.09.2026). h713_disp_run() repeats the
	 * lookup; it is a table walk with no side effect but the selection.
	 */
	{
		struct h713_disp_sel sel;

		ret = h713_disp_lookup(H713_DISP_LOGO_ADDR, project, &sel);
		if (ret)
			return h713_disp_fail(ret);
	}

	if (logo) {
		if (logo_file)
			ret = h713_disp_publish_bmp(true, logo_file, false);
		else
			ret = h713_disp_publish_vendor_bootlogo(true, false);
		if (ret)
			printf("H713 disp: WARNING: no boot logo (%d); "
			       "continuing without one\n", ret);
		else
			logo_up = true;
	}
	if (!logo_up) {
		/*
		 * No logo: black, not whatever DRAM holds. The records point
		 * AFBD channel 1 at this buffer, and with the firmware up the
		 * panel scans it until Linux publishes its console -- white on
		 * a cold start, noise after a warm one.
		 */
		u32 *fb = (u32 *)H713_DISP_OSD_FB_ADDR;
		uint n = H713_DISP_OSD_SIZE / sizeof(u32);

		while (n--)
			*fb++ = 0xff000000;
		flush_cache(H713_DISP_OSD_FB_ADDR, H713_DISP_OSD_SIZE);
	}

	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true, false,
			    false, false, true, release_mips);
	if (ret)
		return h713_disp_fail(ret);

	if (quiesce) {
		h713_disp_quiesce_mips_owner();
		h713_disp_probe_contested("MIPS quiesced");
	}
	h713_disp_latch_panel_timing();

	if (logo_up) {
		/*
		 * Publish once more from the BMP still at its load address: if
		 * the record replay or the firmware start touched the OSD buffer,
		 * the panel would otherwise show DRAM until Linux. Then the same
		 * re-arm the KMS driver does for its console.
		 */
		if (logo_file)
			h713_disp_publish_bmp(false, logo_file, false);
		else
			h713_disp_publish_vendor_bootlogo(false, false);
		h713_disp_commit_osd_frame();
		printf("H713 disp: boot logo up at 0x%08lx with the firmware "
		       "%s; MIPS reset=%08x status=%08x scan=%08x\n",
		       H713_DISP_OSD_FB_ADDR,
		       quiesce ? "quiesced" : "running",
		       readl(H713_MIPS_RESET_REG), readl(H713_MIPS_STATUS_REG),
		       readl(H713_DISP_LVDS_SCAN_REG));
	}

	printf("H713 disp: display initialised%s; scanrate, regscan and "
	       "clkfind can run now. Power-cycle before another init.\n",
	       quiesce ? " with the MIPS core quiesced" :
	       release_mips ? " with the firmware running" :
			      " from the ARM sequence alone");
	return 0;
}

static int h713_disp_panel_test(u32 project, bool release_mips, bool full,
				bool quiesce, bool vendor_logo, bool vendor_chroma,
				bool plane_gate,
				uint tcon_checker_style,
				bool preserve_mips_timing, bool hbands,
				bool grid, bool quads, bool vbands,
				bool pitch, bool pitch_wide, bool hbp,
				bool pitch_low, bool edge, bool edge_fine,
				bool stride, bool stride_fix, bool band,
				bool bl_sweep, bool vendor_early,
				u32 stride_override, bool anim, u32 anim_frames,
				bool anim_db)
{
	int ret;

	/*
	 * A second run in one boot used to be refused, because every mode ended
	 * by abandoning the hardware and the next one then initialised into a
	 * torn-down state: the sequence re-reported success, the register dumps
	 * came back byte-identical, and nothing reached the panel. Undetectable
	 * from the log, and it cost four results before the refusal made it
	 * visible.
	 *
	 * Now there is a teardown, so tear down and carry on instead. Whether
	 * that is enough is the whole acceptance test for it: if this second
	 * run renders, the teardown is correct; if it comes back dark, it is
	 * not, and the message below is what says which run you are looking at.
	 */
	if (h713_panel_test_ran)
		h713_disp_teardown("second run this boot, tearing down first");
	h713_panel_test_ran = true;

	/*
	 * fb-band screens candidate offset registers and scores every step
	 * against 0x0528008c's power-on value, so it is the one mode that must
	 * see 123 rather than the corrected 0.
	 */
	h713_disp_keep_layer_xoff = band;

	ret = h713_disp_load(project);
	if (ret)
		return h713_disp_fail(ret);

	/*
	 * Load the logo AFTER the display sequence, not before it.
	 *
	 * Loading before was the whole reason vendor-logo never put anything
	 * on the panel, across five runs. Every fb-* mode seeds with a plain
	 * memory write and renders; vendor-logo instead selected a block
	 * device, read 2.7 MB off FAT to 0x6d000000 and hashed it, all between
	 * h713_disp_load() and h713_disp_run() -- and the panel then stayed
	 * dark, with every register dump byte-identical to a run that worked
	 * and the TCON marker equally absent. Moving that work after init, and
	 * changing nothing else, renders the logo correctly.
	 *
	 * Note h713_disp_load() already reads this same filesystem in every
	 * mode, so FAT access before the sequence is not the problem by
	 * itself. What is new here is the extra ~2.7 MB read plus a SHA-256
	 * over it -- seconds of work, and a lot of cache traffic, in a window
	 * where nothing else does any. The mechanism is not established.
	 *
	 * vendor-logo-early reproduces the broken ordering for whoever chases
	 * it. Do not make it the default again without a photograph.
	 */
	if (vendor_logo && vendor_early) {
		ret = h713_disp_publish_vendor_bootlogo(true, vendor_chroma);
		if (ret)
			return h713_disp_fail(ret);
	} else {
		h713_disp_fill_pattern(0);
	}
	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true, false,
			    false, false, true, release_mips);
	if (ret)
		return h713_disp_fail(ret);

	if (quiesce) {
		h713_disp_quiesce_mips_owner();
		h713_disp_probe_contested("MIPS quiesced");
	}
	if (preserve_mips_timing) {
		printf("H713 panel: firmware TCON timing preserved: %08x %08x "
		       "%08x %08x %08x %08x\n",
		       readl(0x0588001c), readl(0x05880020),
		       readl(0x05880024), readl(0x05880028),
		       readl(0x0588002c), readl(0x05880030));
	} else {
		h713_disp_latch_panel_timing();
	}

	/*
	 * The long-form phases are opt-in. Each returned the same answer on
	 * roughly eight consecutive runs, so by default they only cost MIPS
	 * uptime -- which matters, because the late uncommanded shutdown is
	 * uncharacterised and Gate 2 only ever validated sixty seconds.
	 *
	 *   blue-screen colours   settled: LVDS, panel power and backlight all
	 *                         work; one colour is enough as a liveness check
	 *   GPIO control test     settled: PF6 and PH16 are proven, and it
	 *                         needlessly cycles panel power
	 *   DE re-assert + 2nd dump
	 *                         settled: the two dumps were byte-identical
	 *                         every time, so the firmware does not tear the
	 *                         OSD layer down
	 *
	 * `full` brings them all back if a result ever needs re-establishing.
	 */
	if (full) {
		h713_disp_panel_blue_test();
		h713_disp_panel_control_test();
	}

	printf("H713 panel: OSD state %s\n",
	       quiesce ? "after firmware init, with MIPS core quiesced" :
	       release_mips ? "as the firmware left it" :
			      "from the ARM sequence alone (MIPS in reset)");
	h713_disp_dump(false);

	/*
	 * The DE block 5 replay destroys firmware PHY and routing state. A
	 * four-point probe across one boot showed the coprocessor setting
	 * 0x051c0014[2:0]=5 and 0x051c0028[28:16]=0x1f30 by MIPS readiness, and
	 * 0x05140054[7] during application readiness -- and the replay clearing
	 * all three back to their pre-release values.
	 *
	 * The TCON pattern generator takes nothing from the OSD/AFBD path, so
	 * for the generator modes the replay is pure damage: it has run before
	 * every generator test performed so far. Skip it there and leave the
	 * firmware's configuration standing.
	 *
	 * This is not by itself an explanation of the generator's
	 * boot-to-boot irreproducibility: test_13 produced colour bands on this
	 * same code path, replay included. Removing a known destructive step is
	 * correct regardless of whether it restores that response.
	 */
	if (tcon_checker_style && (full || quiesce) && !preserve_mips_timing)
		printf("H713 panel: DE replay skipped for the TCON generator; "
		       "firmware PHY/routing state retained\n");

	if ((full || quiesce) && !preserve_mips_timing && !tcon_checker_style) {
		/*
		 * The replay is still needed here: it re-asserts DE block 5,
		 * which is what configures the OSD/AFBD path the framebuffer
		 * modes depend on. But it also clears three words the firmware
		 * set -- 0x051c0014[2:0], 0x051c0028[28:16] and 0x05140054[7] --
		 * so capture them first and put them back afterwards. The
		 * generator modes sidestep this by skipping the replay entirely;
		 * the framebuffer modes cannot.
		 */
		u32 phy14 = readl(0x051c0014);
		u32 phy28 = readl(0x051c0028);
		u32 route54 = readl(0x05140054);

		ret = h713_disp_reassert_osd(H713_DISP_LOGO_ADDR, project);
		if (ret)
			return h713_disp_fail(ret);

		writel(phy14, 0x051c0014);
		writel(phy28, 0x051c0028);
		writel(route54, 0x05140054);
		dmb();
		printf("H713 panel: firmware PHY/routing restored after the DE "
		       "replay: %08x %08x %08x\n",
		       readl(0x051c0014), readl(0x051c0028),
		       readl(0x05140054));
		h713_disp_probe_contested("after DE replay");
		printf("H713 panel: OSD state after %s re-assert\n",
		       quiesce ? "post-quiesce" : "full-test");
		h713_disp_dump(false);
	}

	/*
	 * Applied after the DE replay, which rewrites the AFBD block, so the
	 * override has to land here rather than during init or it is undone.
	 * Lets the value the fix sweep picks be tried against vendor-logo the
	 * same evening, without a rebuild between.
	 */
	if (stride_override) {
		writel(stride_override, H713_DISP_AFBD_STRIDE_REG);
		dmb();
		printf("H713 panel: AFBD stride overridden to %08x (%u px), "
		       "reads back %08x\n", stride_override,
		       stride_override / 4,
		       readl(H713_DISP_AFBD_STRIDE_REG));
	}

	/*
	 * The left band, fixed. test_32 settled it in one run: 0x0528008c is a
	 * plain pixel X origin for the layer, 1:1 and with nothing else in it.
	 * Zeroed, content starts at column 0 and fills all 1280; set to 400,
	 * it starts at 406. The four other candidates screened alongside moved
	 * it by 1 px.
	 *
	 * Whatever writes 123 does so before this point -- it survives the DE
	 * replay, so it is either LogoRegData or the firmware itself -- which
	 * is why this is a write here rather than a patched record. Applied to
	 * every mode except fb-band, which needs the untouched value for its
	 * own control step.
	 */
	h713_disp_enforce_layer_xoff("after the DE replay");

	if (anim || anim_db) {
		h713_disp_anim_run(anim_frames, anim_db);
	} else if (bl_sweep) {
		h713_disp_backlight_sweep();
	} else if (band) {
		h713_disp_band_sweep();
	} else if (stride_fix) {
		h713_disp_stride_fix_sweep();
	} else if (stride) {
		h713_disp_stride_sweep();
	} else if (edge_fine) {
		h713_disp_edge_sweep(h713_pitch_sweep_fine,
				     ARRAY_SIZE(h713_pitch_sweep_fine));
	} else if (edge) {
		h713_disp_edge_sweep(h713_pitch_sweep_low,
				     ARRAY_SIZE(h713_pitch_sweep_low));
	} else if (pitch_low) {
		h713_disp_pitch_sweep(h713_pitch_sweep_low,
				      ARRAY_SIZE(h713_pitch_sweep_low));
	} else if (pitch_wide) {
		h713_disp_pitch_sweep(h713_pitch_sweep_wide,
				      ARRAY_SIZE(h713_pitch_sweep_wide));
	} else if (pitch) {
		h713_disp_pitch_sweep(h713_pitch_sweep,
				      ARRAY_SIZE(h713_pitch_sweep));
	} else if (vbands) {
		h713_disp_fill_vbands();
		printf("H713 panel: VERTICAL BAND PROBE at the panel's own timing; "
		       "one frame, holding %u ms\n",
		       H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_commit_osd_frame();
		printf("H713 panel: VERTICAL BANDS COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else if (quads) {
		h713_disp_fill_quads();
		printf("H713 panel: QUADRANT TEST at the panel's own timing; one "
		       "frame, holding %u ms\n", H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_commit_osd_frame();
		printf("H713 panel: QUADRANTS COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else if (grid) {
		h713_disp_fill_grid();
		printf("H713 panel: GEOMETRY GRID at the panel's own timing; one "
		       "frame, holding %u ms\n", H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_commit_osd_frame();
		printf("H713 panel: GRID COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else if (hbands) {
		h713_disp_fill_hbands();
		printf("H713 panel: HORIZONTAL BAND PROBE at the panel's own timing; "
		       "one frame, holding %u ms\n",
		       H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_commit_osd_frame();
		printf("H713 panel: BANDS COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else if (vendor_logo) {
		/*
		 * Two markers, bracketing the publish, so one run says where
		 * the panel dies rather than only that it did.
		 *
		 *   1 then 2 blinks -> panel alive throughout; the fault is in
		 *     the frame content or the commit
		 *   1 blink only    -> publishing the logo kills it, which is
		 *     what the pre-run ordering used to do
		 *   neither         -> the panel never came up in this mode,
		 *     and everything after init is beside the point
		 *
		 * The marker drives the TCON generator, which reaches the panel
		 * without touching the framebuffer path at all.
		 */
		h713_disp_chroma_marker(1);
		/*
		 * Load here unless the early variant already did. Getting this
		 * wrong once hashed whatever DRAM happened to sit at
		 * 0x6d000000, refused it, and returned before either marker.
		 */
		ret = h713_disp_publish_vendor_bootlogo(!vendor_early,
						       vendor_chroma);
		if (ret)
			return h713_disp_fail(ret);
		/*
		 * The one caller that needs the digest, as a gate. This mode
		 * is a control: h713_disp_verify_fb() below reads rows
		 * 343..378 and columns 368..912, bounds measured from the
		 * stock file, and "the logo is where it should be" means
		 * nothing about some other picture. The product boot path
		 * hashes nothing (a board whose logo nobody has hashed must
		 * still get a logo, and a boot must not pay for a log line),
		 * so the identity is computed here and only here.
		 */
		if (!h713_disp_bmp_is_vendor()) {
			printf("H713 panel: vendor-logo is a control and needs "
			       "the stock bootlogo.bmp; refusing this one\n");
			return h713_disp_fail(-EKEYREJECTED);
		}
		h713_disp_chroma_marker(2);
		/* Bounds measured from the file: rows 343..378, cols 368..912. */
		h713_disp_verify_fb(343, 378, 368, 912, vendor_chroma);
		printf("H713 panel: EXACT VENDOR LOGO TEST at the panel's own timing; "
		       "one frame, holding %u ms\n",
		       H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_commit_osd_frame();
		printf("H713 panel: EXACT VENDOR LOGO COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else if (plane_gate) {
		/* One frame and one downstream gate; no settled upstream probes. */
		h713_disp_boardb_plane_gate_test();
	} else if (tcon_checker_style) {
		/* Board-B hardware pattern; independent of the OSD pixel path. */
		if (tcon_checker_style == 16)
			h713_disp_hbp_sweep();
		else if (tcon_checker_style == 15)
			h713_disp_tcon_n_sweep(h713_n_sweep_hi,
					       ARRAY_SIZE(h713_n_sweep_hi));
		else if (tcon_checker_style == 14)
			h713_disp_tcon_n_sweep(h713_n_sweep,
					       ARRAY_SIZE(h713_n_sweep));
		else if (tcon_checker_style == 13)
			h713_disp_chroma_test_at_62m();
		else if (tcon_checker_style == 12)
			h713_disp_boardb_tcon_chroma_test();
		else if (tcon_checker_style == 10)
			h713_disp_boardb_tcon_dclk_test();
		else if (tcon_checker_style == 11)
			h713_disp_boardb_tcon_normal_solid_test();
		else
			h713_disp_boardb_tcon_test(tcon_checker_style);
	} else if (quiesce) {
		/* Cheap, and its scan column is the only raster-liveness signal. */
		h713_disp_fill_pattern(0);
		h713_disp_afbd_enable_probe();
		h713_disp_plane_gate_test();
		h713_disp_afbd_gate_test();

		/*
		 * Keep this ownership diagnostic to one unmistakable publication.
		 * Rotating phases add no information until a first bar frame is
		 * visible, and made hand-recorded gate observations ambiguous.
		 */
		printf("H713 panel: STATIC BAR TEST at the panel's own timing; "
		       "one frame, holding %u ms\n",
		       H713_DISP_STATIC_FRAME_DWELL_MS);
		h713_disp_fill_pattern(0);
		h713_disp_commit_osd_frame();
		printf("H713 panel: STATIC BARS COMMITTED; photograph now\n");
		mdelay(H713_DISP_STATIC_FRAME_DWELL_MS);
	} else {
		/* Cheap, and its scan column is the only raster-liveness signal. */
		h713_disp_afbd_enable_probe();
		printf("H713 panel: moving OSD at the panel's own timing, "
		       "%u frames x %u ms\n",
		       full ? H713_DISP_OSD_FRAMES : 4,
		       full ? H713_DISP_OSD_DWELL_MS : 3000);
		h713_disp_animate_pattern(full ? H713_DISP_OSD_FRAMES : 4,
					  full ? H713_DISP_OSD_DWELL_MS : 3000,
					  1);
	}

	if (!release_mips) {
		printf("H713 panel: pattern test complete; MIPS never released, "
		       "so this run may be repeated without a power cycle\n");
		return 0;
	}

	printf("H713 panel: pattern test complete\n");
	return 0;

}

/*
 * The elog buffer is a small ring and it wraps.
 *
 * The first capture proved it: the same TMpegNR error appears at both ends of
 * the dumped region, and two truncated fragments ("...ionModuleID",
 * "...30030008") sit between them. About 2.6 KB of ring, already overwritten
 * by the time we read it -- so at BUF mode a raised level does not get us more
 * history, it gets us the same tail flooded sooner.
 *
 * Hence the mode argument. Mode 0 is ELOG_OUTPUT_MODE_SYNC, which writes
 * straight out through `route` (0 = uart) with no buffer to overflow. The
 * formatter is fully present for every level -- the A/E/W/I/D/V tag table and
 * its colour table are contiguous in the firmware -- so if call sites exist
 * above ERROR, sync mode is what will show them.
 *
 * Default stays 2 (BUF), which is what has been used so far and is known to
 * produce readable output.
 */
static int h713_disp_test(u32 project, u32 source_id, u32 level, u32 mode)
{
	int ret;

	ret = h713_disp_load(project);
	if (ret)
		return h713_disp_fail(ret);

	printf("H713 disp: config patches\n");
	h713_cfg_set(H713_CFG_OFF_SOURCE_ID, '0' + source_id, "source_id");
	h713_cfg_set(H713_CFG_OFF_ELOG_MODE, '0' + mode, "elog mode");
	h713_cfg_set(H713_CFG_OFF_ELOG_ASYNC, '0', "elog async");
	h713_cfg_set(H713_CFG_OFF_ELOG_LEVEL, '0' + level, "elog level");

	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, false, false, false,
			    false, false, false, true);
	if (ret)
		return h713_disp_fail(ret);

	h713_disp_sample();

	printf("H713 disp: firmware log\n");
	h713_mips_log(0x4b232000, 0x4bd00000);

	return 0;
}

/*
 * Publish the vendor boot logo and leave it on the panel. The product's
 * boot-logo step, and it is opt-in: `auto <id> logo`, not the `auto` default.
 *
 * The design call (2026-08-07): a projector should show a boot logo, and Linux
 * preserves the U-Boot frame (verified 2026-08-07), so a logo published here
 * persists through boot with no Linux display driver. But `auto` is not on the
 * boot path -- the default bootcmd is `mmc read; bootm` -- so making render the
 * default would only add a bootlogo.bmp dependency and ~1.5 s to a diagnostic
 * command for no boot-behaviour gain. Product images add `h713_disp auto <id>
 * logo` before `bootm`; the dev standalone boot stays fast and artifact-free.
 *
 * This is the `panel-test <id> vendor-logo` sequence with the diagnostics
 * removed, not a new path. That mode runs with quiesce=true, so it does
 * exactly: run (panel powered), quiesce the MIPS, latch the panel timing,
 * re-assert the DE block with the firmware PHY/routing saved across it, apply
 * the layer X-origin fix, then publish and commit the real logo. Item 4's
 * successful Linux handoff used precisely this state -- MIPS quiesced, logo up
 * -- which is why it is replicated rather than simplified. What is dropped is
 * output only: the register dumps, the chroma markers, fbcheck, and the 15 s
 * diagnostic dwell (fatal on a boot path).
 */
static int h713_disp_auto_logo(u32 project, const char *logo_file)
{
	u32 phy14, phy28, route54;
	int ret;

	/* Same rerun protection as panel-test: a second sequence this boot
	 * must tear the first down or it initialises into a half-torn state. */
	if (h713_panel_test_ran)
		h713_disp_teardown("second run this boot, tearing down first");
	h713_panel_test_ran = true;

	ret = h713_disp_load(project);
	if (ret)
		return h713_disp_fail(ret);

	/* Panel powered (stock_panel_power=true), MIPS released. */
	ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true, false,
			    false, false, true, true);
	if (ret)
		return h713_disp_fail(ret);

	h713_disp_quiesce_mips_owner();
	h713_disp_latch_panel_timing();

	/* The DE replay re-asserts the OSD/AFBD path the framebuffer needs, but
	 * clears three words the firmware set; save and restore them, exactly
	 * as panel-test does. */
	phy14 = readl(0x051c0014);
	phy28 = readl(0x051c0028);
	route54 = readl(0x05140054);
	ret = h713_disp_reassert_osd(H713_DISP_LOGO_ADDR, project);
	if (ret)
		return h713_disp_fail(ret);
	writel(phy14, 0x051c0014);
	writel(phy28, 0x051c0028);
	writel(route54, 0x05140054);
	dmb();

	h713_disp_enforce_layer_xoff("after the DE replay");

	/*
	 * Publish the real artwork (chroma=false). The default hashes the stock
	 * asset to name it in the log, a custom file is not looked up at all,
	 * and neither is refused on its digest. Format is validated either way
	 * -- the panel's own size, 24 bpp, uncompressed.
	 */
	if (logo_file)
		ret = h713_disp_publish_bmp(true, logo_file, false);
	else
		ret = h713_disp_publish_vendor_bootlogo(true, false);
	if (ret)
		return h713_disp_fail(ret);
	h713_disp_commit_osd_frame();

	printf("H713 disp: boot logo published and committed\n");
	return 0;
}

static int do_h713_disp(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	ulong blob;

	if ((argc == 2 || argc == 3) && !strcmp(argv[1], "dump")) {
		bool force = argc == 3 && !strcmp(argv[2], "force");

		if (argc == 3 && !force)
			return CMD_RET_USAGE;
		h713_disp_dump(force);
		return CMD_RET_SUCCESS;
	}

	/* Everything in one command, so operator timing is not a variable. */
	if (argc >= 3 && !strcmp(argv[1], "test")) {
		u32 project = hextoul(argv[2], NULL);
		u32 src = argc > 3 ? dectoul(argv[3], NULL) : 2;
		u32 lvl = argc > 4 ? dectoul(argv[4], NULL) : 3;
		u32 mode = argc > 5 ? dectoul(argv[5], NULL) : 2;

		if (argc > 6 || src > 9 || lvl > 5 || mode > 2)
			return CMD_RET_USAGE;
		return h713_disp_test(project, src, lvl, mode) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/*
	 * Reproduce stock's panel-power phase, publish cache-coherent pixels,
	 * prove full firmware readiness, then compare the firmware-selected
	 * timing with the panel's timing.
	 */
	/*
	 * Opt-in raw call into the live coprocessor queues. GetImageBufferAddr
	 * (0x2f02f7dd) is the safest
	 * first target: it takes no parameters and its registered handler in the
	 * authenticated board-B image is a two-instruction no-op.
	 */
	if (argc >= 3 && !strcmp(argv[1], "commcall")) {
		u32 comp_id = hextoul(argv[2], NULL);
		u32 chan = 0, pid = 0;
		u32 p[10];
		uint n = 0, k;

		for (k = 3; k < (uint)argc; k++) {
			if (!strncmp(argv[k], "db=", 3)) {
				printf("H713 comm: db= is no longer supported; "
				       "commcall always sends a CALL (doorbell 0)\n");
				return CMD_RET_FAILURE;
			}
			/*
			 * chan/pid select the firmware channel the CALL is
			 * addressed to. Comm_GetCallbyChannel rejects the
			 * message outright when no channel matches, so these
			 * have to agree with what `commdev` reports.
			 */
			if (!strncmp(argv[k], "chan=", 5)) {
				chan = hextoul(argv[k] + 5, NULL);
				continue;
			}
			if (!strncmp(argv[k], "pid=", 4)) {
				pid = hextoul(argv[k] + 4, NULL);
				continue;
			}
			if (n >= ARRAY_SIZE(p))
				return CMD_RET_USAGE;
			p[n++] = hextoul(argv[k], NULL);
		}

		return h713_comm_call(comp_id, p, n, chan, pid, NULL) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc >= 2 && !strcmp(argv[1], "comm-pq-test")) {
		u32 chan = 0, pid = 0;
		bool have_chan = false, have_pid = false;
		uint k;

		for (k = 2; k < (uint)argc; k++) {
			if (!strncmp(argv[k], "chan=", 5)) {
				chan = hextoul(argv[k] + 5, NULL);
				have_chan = true;
				continue;
			}
			if (!strncmp(argv[k], "pid=", 4)) {
				pid = hextoul(argv[k] + 4, NULL);
				have_pid = true;
				continue;
			}
			return CMD_RET_USAGE;
		}
		if (!have_chan || !have_pid) {
			printf("H713 comm: comm-pq-test requires explicit "
			       "chan=<hex> and pid=<hex> from commdev\n");
			return CMD_RET_FAILURE;
		}

		return h713_comm_picture_mode_test(chan, pid) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "commstate"))
		return h713_disp_comm_state() ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;

	if (argc == 5 && !strcmp(argv[1], "bl-gpio")) {
		h713_disp_bl_gpio_pwm(dectoul(argv[2], NULL),
				      dectoul(argv[3], NULL),
				      dectoul(argv[4], NULL));
		return CMD_RET_SUCCESS;
	}

	if (argc >= 2 && argc <= 7 && !strcmp(argv[1], "init")) {
		bool noboot = false, quiesce = false, have_id = false;
		bool logo = false;
		const char *logo_file = NULL;
		u32 project = 0;
		int elog = -1;
		int i;

		/*
		 * The project ID is optional now, and the first token that is
		 * not a flag. "h713_disp init ${h713_project}" with the
		 * variable unset expands to exactly this, so a device whose
		 * environment has not been told what it is asks the board.
		 */
		for (i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "noboot"))
				noboot = true;
			else if (!strcmp(argv[i], "quiesce"))
				quiesce = true;
			else if (!strncmp(argv[i], "elog=", 5))
				elog = (int)dectoul(argv[i] + 5, NULL);
			else if (!strcmp(argv[i], "logo"))
				logo = true;
			else if (logo && !logo_file)
				/* A token after "logo" is a custom BMP on the
				 * artifact filesystem, published without the
				 * vendor hash. The project ID goes before it. */
				logo_file = argv[i];
			else if (!have_id) {
				project = hextoul(argv[i], NULL);
				have_id = true;
			} else {
				return CMD_RET_USAGE;
			}
		}
		if (elog > 5) {
			printf("H713 disp: elog level %d out of range "
			       "(0 assert .. 5 verbose)\n", elog);
			return CMD_RET_USAGE;
		}
		if (!have_id && h713_disp_declared_project(&project))
			return CMD_RET_FAILURE;

		return h713_disp_init_only(project, !noboot, quiesce, elog,
					   logo, logo_file) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if ((argc == 2 || argc == 3) && !strcmp(argv[1], "clkfind")) {
		int which = argc == 3 ? (int)dectoul(argv[2], NULL) : -1;

		/* Perturbs a clock and watches the display witness to score it. */
		if (!h713_disp_readable("clkfind"))
			return CMD_RET_FAILURE;

		return h713_disp_clk_find(which) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "teardown")) {
		/*
		 * Safe from a cold boot: teardown acquires the display clocks
		 * before it reads anything. Briefly guarded here instead, until
		 * the vendor's "acquire tvdisp clock on emergency shutdown"
		 * showed that ungating is the fix rather than refusing.
		 */
		h713_disp_teardown("requested from the prompt");
		return CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "scanrate")) {
		if (!h713_disp_readable("scanrate"))
			return CMD_RET_FAILURE;
		return h713_disp_scan_rate() ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if ((argc >= 2 && argc <= 4) && !strcmp(argv[1], "regscan")) {
		ulong base = argc >= 3 ? hextoul(argv[2], NULL) : 0x05880000UL;
		uint words = argc >= 4 ? dectoul(argv[3], NULL) : 32;

		/*
		 * Only the display window needs the guard; regscan is happy to
		 * walk the CCU or anything else cold, and refusing that would
		 * take away a diagnostic that works.
		 */
		if (base >= 0x05000000UL && base < 0x06000000UL &&
		    !h713_disp_readable("regscan of the display window"))
			return CMD_RET_FAILURE;
		return h713_disp_reg_scan(base, words) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "commtrace")) {
		h713_mips_print_comm_trace();
		return CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "commdev"))
		return h713_disp_comm_dev() ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;

	if ((argc == 3 || argc == 4) && !strcmp(argv[1], "fwmd")) {
		ulong va = hextoul(argv[2], NULL);
		uint n = argc == 4 ? (uint)hextoul(argv[3], NULL) : 16;

		return h713_disp_fw_md(va, n) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if ((argc == 2 || argc == 3) && !strcmp(argv[1], "calltable")) {
		uint n = argc == 3 ? dectoul(argv[2], NULL) : 4;

		return h713_disp_call_table(n) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if ((argc >= 3 && argc <= 5) && !strcmp(argv[1], "panel-test")) {
		/*
		 * Optional argv[4] is a raw AFBD stride, in bytes, applied
		 * for the whole run. Keeps a value the fix sweep picks
		 * testable against any mode without a rebuild.
		 */
		const char *mode = argc >= 4 ? argv[3] : "";
		bool anim = !strcmp(mode, "fb-anim");
		bool anim_db = !strcmp(mode, "fb-anim-db");
		/*
		 * fb-anim takes a decimal frame count in the same slot. A
		 * stride override would shear the bar and break the position ->
		 * frame-number encoding, so the two are mutually exclusive by
		 * construction rather than by a warning nobody reads.
		 */
		u32 anim_frames = ((anim || anim_db) && argc == 5) ?
				   dectoul(argv[4], NULL) : 0;
		u32 stride_override = (!anim && !anim_db && argc == 5) ?
				       hextoul(argv[4], NULL) : 0;
		bool noboot = !strcmp(mode, "noboot");
		bool full   = !strcmp(mode, "full");
		bool quiesce = !strcmp(mode, "quiesce");
		/*
		 * Loading after the display sequence is the DEFAULT, because
		 * loading before it does not work: see h713_disp_panel_test.
		 * vendor-logo-late is kept as an alias of that default;
		 * vendor-logo-early reproduces the broken ordering for anyone
		 * chasing the mechanism.
		 */
		bool vendor_early = !strcmp(mode, "vendor-logo-early");
		bool vendor_chroma = !strcmp(mode, "vendor-logo-chroma") ||
				     !strcmp(mode, "vendor-logo-late") ||
				     vendor_early;
		bool vendor_logo = !strcmp(mode, "vendor-logo") ||
				   vendor_chroma;
		bool plane_gate = !strcmp(mode, "plane-gate");
		bool tcon_checker = !strcmp(mode, "tcon-checker");
		bool tcon_checker_mono = !strcmp(mode, "tcon-checker-mono");
		bool tcon_solid = !strcmp(mode, "tcon-solid");
		bool tcon_solid_native = !strcmp(mode, "tcon-solid-native");
		bool tcon_dclk = !strcmp(mode, "tcon-dclk");
		bool tcon_solid_dclk_normal = !strcmp(mode, "tcon-solid-dclk-normal");
		bool tcon_chroma = !strcmp(mode, "tcon-chroma");
		bool tcon_chroma_62m = !strcmp(mode, "tcon-chroma-62m");
		bool tcon_nsweep = !strcmp(mode, "tcon-nsweep");
		bool tcon_nsweep_hi = !strcmp(mode, "tcon-nsweep-hi");
		bool hbands = !strcmp(mode, "fb-vprobe");
		bool grid = !strcmp(mode, "fb-grid");
		bool quads = !strcmp(mode, "fb-quad");
		bool vbands = !strcmp(mode, "fb-hprobe");
		bool pitch = !strcmp(mode, "fb-pitch");
		bool pitch_wide = !strcmp(mode, "fb-pitch-wide");
		bool hbp = !strcmp(mode, "tcon-hbp");
		bool pitch_low = !strcmp(mode, "fb-pitch-low");
		bool edge = !strcmp(mode, "fb-edge");
		bool edge_fine = !strcmp(mode, "fb-edge-fine");
		bool stride = !strcmp(mode, "fb-stride");
		bool stride_fix = !strcmp(mode, "fb-fix");
		bool band = !strcmp(mode, "fb-band");
		bool bl_sweep = !strcmp(mode, "bl-sweep");

		if (argc >= 4 && !noboot && !full && !quiesce && !vendor_logo &&
		    !plane_gate && !tcon_checker && !tcon_checker_mono &&
		    !tcon_solid && !tcon_solid_native && !tcon_dclk &&
		    !tcon_solid_dclk_normal && !tcon_chroma && !tcon_chroma_62m &&
		    !tcon_nsweep && !tcon_nsweep_hi && !hbands && !grid && !quads && !vbands && !pitch && !pitch_wide && !hbp && !pitch_low && !edge && !edge_fine && !stride && !stride_fix && !band &&
		    !bl_sweep && !anim && !anim_db)
			return CMD_RET_USAGE;
		return h713_disp_panel_test(hextoul(argv[2], NULL), !noboot,
					    full,
					    quiesce || vendor_logo || plane_gate ||
					    tcon_checker || tcon_checker_mono ||
					    tcon_solid || tcon_solid_native ||
					    tcon_dclk || tcon_solid_dclk_normal ||
					    tcon_chroma || tcon_chroma_62m ||
					    tcon_nsweep || tcon_nsweep_hi || hbands || grid || quads || vbands || pitch || pitch_wide || hbp || pitch_low || edge || edge_fine || stride || stride_fix || band || bl_sweep || anim || anim_db,
					    vendor_logo, vendor_chroma, plane_gate,
					    hbp ? 16 :
					    tcon_nsweep_hi ? 15 :
					    tcon_nsweep ? 14 :
					    tcon_chroma_62m ? 13 :
					    tcon_chroma ? 12 :
					    tcon_solid_dclk_normal ? 11 :
					    tcon_dclk ? 10 :
					    (tcon_solid || tcon_solid_native) ? 9 :
					    tcon_checker_mono ? 1 :
					    tcon_checker ? 8 : 0,
					    tcon_solid_native, hbands, grid, quads, vbands, pitch, pitch_wide, hbp, pitch_low, edge, edge_fine, stride, stride_fix, band,
					    bl_sweep, vendor_early, stride_override,
				    anim, anim_frames, anim_db) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/* Load only, so display_cfg.xml can be patched before the run. */
	if (argc == 3 && !strcmp(argv[1], "load")) {
		if (h713_disp_load(hextoul(argv[2], NULL)))
			return CMD_RET_FAILURE;
		printf("H713 disp: loaded; run with 'h713_disp 0x%08lx <project>'\n",
		       H713_DISP_LOGO_ADDR);
		return CMD_RET_SUCCESS;
	}

	/*
	 * Full display launch plus the CPU_COMM handoff. The guarded HDCP wait
	 * override takes the firmware's own timeout path so its startup can
	 * reach the CPU_COMM thread while interrupts are still masked.
	 */
	if (argc == 3 && (!strcmp(argv[1], "mips-test") ||
			  !strcmp(argv[1], "mips-trace") ||
			  !strcmp(argv[1], "mips-comm-trace") ||
			  !strcmp(argv[1], "mips-stability"))) {
		u32 project = hextoul(argv[2], NULL);
		bool trace = !strcmp(argv[1], "mips-trace");
		bool comm_trace = !strcmp(argv[1], "mips-comm-trace");
		bool stability = !strcmp(argv[1], "mips-stability");
		int ret;

		if (h713_disp_load(project))
			return CMD_RET_FAILURE;
		ret = h713_disp_run(H713_DISP_LOGO_ADDR, project, true, true,
				    trace, stability, comm_trace, false, true);
		printf("H713 disp: MIPS test complete; power-cycle before "
		       "another MIPS run\n");
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	/* Load everything from eMMC, then run: one command from power-on. */
	if (argc >= 3 && !strcmp(argv[1], "auto")) {
		u32 project = hextoul(argv[2], NULL);
		bool nowait = false;
		bool logo = false;
		const char *logo_file = NULL;
		int i;

		for (i = 3; i < argc; i++) {
			if (!strcmp(argv[i], "nowait"))
				nowait = true;
			else if (!strcmp(argv[i], "logo"))
				logo = true;
			else if (logo && !logo_file)
				/* A token after "logo" is a custom BMP on the
				 * FAT, published without the vendor hash. */
				logo_file = argv[i];
			else
				return CMD_RET_USAGE;
		}

		/* Opt-in boot logo; the default stays prep-only. */
		if (logo)
			return h713_disp_auto_logo(project, logo_file) ?
			       CMD_RET_FAILURE : CMD_RET_SUCCESS;

		if (h713_disp_load(project))
			return CMD_RET_FAILURE;
		return h713_disp_run(H713_DISP_LOGO_ADDR, project, nowait,
				     false, false, false, false, false, true) ?
		       CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (argc == 3 && !strcmp(argv[1], "list")) {
		h713_disp_list(hextoul(argv[2], NULL));
		return CMD_RET_SUCCESS;
	}

	if (argc != 3 && argc != 4)
		return CMD_RET_USAGE;
	if (argc == 4 && strcmp(argv[3], "nowait"))
		return CMD_RET_USAGE;

	blob = hextoul(argv[1], NULL);

	return h713_disp_run(blob, hextoul(argv[2], NULL), argc == 4, false,
			     false, false, false, false, true) ?
	       CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_disp, 15, 0, do_h713_disp,
	   "run stock's fastlogo display sequence for a project ID",
	   "the project ID is the board's, not this build's: an HY310 declares\n"
	   "0x30, the HY200 bench board 0x34 (0x33 renders identically there).\n"
	   "h713_probe reads it off the device.\n"
	   "test <project-id> [source] [level] [mode] - load, patch, run, sample, log\n"
	   "                                      level 0..5 (ASSERT..VERBOSE), default 3\n"
	   "                                      mode 0=sync 1=async 2=buf, default 2\n"
	   "                                      buf is a ~2.6 KB ring and wraps; sync does not\n"
	   "h713_disp mips-test <project-id>    - run with CPU_COMM readiness proof\n"
	   "h713_disp mips-trace <project-id>   - stream full-launch startup markers\n"
	   "h713_disp mips-comm-trace <project-id> - trace CPU_COMM RETURN progress\n"
	   "h713_disp mips-stability <project-id> - run 60s heartbeat/exception test\n"
	   "h713_disp calltable [raw-entries]   - read the live CPU_COMM call table\n"
	   "h713_disp commstate                 - read the CPU_COMM transports\n"
	   "h713_disp commtrace                 - read the CPU_COMM trace stage\n"
	   "h713_disp init [project-id] [noboot|quiesce] [elog=<0-5>] [logo [file.bmp]]\n"
	   "                                      logo: boot logo on the panel with the firmware RUNNING (the\n"
	   "                                            product boot with a picture; a missing logo only warns)\n"
	   "    without an ID: env h713_project, else panel_config.ini on\n"
	   "    Reserve0 or media_data (ProjectID is decimal there).\n"
	   "    elog turns on the coprocessor's own log (ring mode 1).\n"
	   "    Needs a reader draining it, or it fills and stops.\n"
	   "                                    - bring the display up and stop, ready for diagnostics\n"
	   "                                      quiesce: park the MIPS core (use this for clkfind)\n"
	   "h713_disp teardown                  - stop scanout, park the MIPS, drop the panel rail\n"\
	   "                                      run it, then run panel-test again: if the second\n"\
	   "                                      render is correct the teardown is correct\n"
	   "h713_disp scanrate                  - measure line/frame rate and real DCLK\n"
	   "h713_disp regscan [base] [words]    - find which registers move, and what they count\n"
	   "h713_disp clkfind [index]           - list, or test ONE clock candidate (see docs)\n"
	   "h713_disp commdev                   - read the firmware channel table\n"
	   "h713_disp fwmd <mips-va> [words]    - dump firmware memory (cache-safe)\n"
	   "h713_disp commcall <id> [chan=<hex>] [pid=<hex>] [args..]\n"
	   "                                    - send one CPU_COMM CALL (writes!)\n"
	   "                                      chan=/pid= select the channel\n"
	   "h713_disp comm-pq-test chan=<hex> pid=<hex>\n"
	   "                                    - guarded picture-mode get/set-same/get\n"
	   "h713_disp panel-test <project-id> [noboot|full|quiesce|vendor-logo|plane-gate|tcon-checker|tcon-checker-mono|tcon-solid|tcon-solid-native|tcon-dclk|tcon-solid-dclk-normal]\n"
	   "                                    - panel-sized colours, power controls, OSD\n"
	   "                                      noboot: hold MIPS in reset, ARM only\n"
	   "                                      full:   add the settled long-form phases\n"
	   "                                      quiesce: init, reset MIPS core, test OSD\n"
	   "                                      vendor-logo: exact Board-B logo only\n"
	   "                                      plane-gate: exact Board-B plane-open bit\n"
	   "                                      tcon-checker: Board-B red/blue HW checker\n"
	   "                                      tcon-checker-mono: Board-B black/white checker\n"
	   "                                      tcon-solid: Board-B solid black then white\n"
	   "                                      tcon-solid-native: same, preserve firmware timing\n"
	   "                                      tcon-dclk: fixed zero source, DCLK edge A/B/A\n"
	   "                                      tcon-solid-dclk-normal: zero/one under normal DCLK\n"
	   "                                      tcon-chroma: RGB solids + red/blue checkers (128/32px)\n"
	   "                                      tcon-chroma-62m: same, with DCLK retuned 73.7 -> 61.7 MHz\n"
	   "                                      tcon-nsweep: step the display PLL, checker at each step\n"
	   "                                      tcon-nsweep-hi: same, N+1 50..60 (extends past the best so far)\n"
	   "                                      fb-vprobe: 8 horizontal colour bands (vertical order only)\n"
	   "                                      fb-grid: border+diagonals+corners (needs a sharp photo)\n"
	   "                                      fb-quad: four solid quadrants; survives blur, counts the tiling\n"
	   "                                      fb-hprobe: 8 vertical bands; isolates the horizontal axis\n"
	   "                                      fb-pitch: sweep the assumed line pitch 1280..1300\n"
	   "                                      fb-pitch-wide: sweep 1360..1920 (HTOTAL and alignment roundings)\n"
	   "                                      tcon-hbp: sweep the horizontal back porch to chase the edge band\n"
	   "                                      fb-pitch-low: sweep 1180..1200 with eight bands\n"
	   "                                      fb-edge: 1180..1200 red/blue edge; stripe count gives the stride (measured 1237)\n"
	   "                                      fb-edge-fine: 1232..1240, confirms the stride to the pixel\n"
	   "                                      fb-stride: pattern fixed, sweep AFBD 0x05600170 -- LIVE, unit slope, S = V - 42\n"
	   "                                      fb-fix: pattern at the natural 1280, sweep for the register that unshears it\n"
	   "                                      fb-band: solid fill, screen offset registers for the ~110px left band\n"
	   "                                      bl-sweep: white field, step PWM2/PB4 duty 100..0 (the stock dimmer)\n"
	   "                                      fb-anim: moving red bar on blue, SINGLE-buffered -- tears, and that is the baseline\n"
	   "                                               argv[5] is a decimal frame count (default 600); Ctrl-C stops\n"
	   "                                      fb-anim-db: same bar, DOUBLE-buffered via the AFBD source address -- should not tear\n"

	   "                                               run both on one boot; the A/B is the measurement\n"
	   "h713_disp bl-gpio <hz> <duty%> <secs> - bit-bang PB5, the light's supply enable, to test whether the\n"
	   "                                      on-board boost converter dims on PWM-of-enable. Self-terminating;\n"
	   "                                      always restores PB5 high. PB5 also powers the fan: keep runs short.\n"
	   "                                      vendor-logo-chroma: the stock logo, lit->red unlit->blue (it is pure grey, which this path cannot show)\n"
	   "                                      vendor-logo-late: alias of vendor-logo-chroma; loading late is now the default\n"
	   "                                      vendor-logo-early: loads before the display sequence -- known broken, kept to chase why\n"
	   "       h713_disp panel-test <id> <mode> <stride>  - same, with AFBD 0x05600170 forced to <stride> bytes (hex)\n"
	   "h713_disp auto <project-id> [nowait] [logo [file.bmp]] - load from eMMC and run\n"
	   "                                      logo: publish the boot logo and leave it up, but PARK the\n"
	   "                                            coprocessor as the last act -- Linux then finds a dead\n"
	   "                                            firmware and cannot switch to HDMI. NOT the product boot\n"
	   "                                            path: that is \"init <id> logo\", firmware left running\n"
	   "                                            optional file.bmp is a custom 24-bit logo the panel's own size;\n"
	   "                                            default is the vendor bootlogo.bmp; the file is not hashed or gated\n"
	   "h713_disp load <project-id>         - load from eMMC only\n"
	   "h713_disp <blob-addr> <project-id> [nowait] - run against a staged blob\n"
	   "h713_disp list <blob-addr>          - show every project's tables\n"
	   "h713_disp dump [force]              - dump the display register blocks"
);

/*
 * h713_probe -- say what this board is, and write nothing.
 *
 * Everything a board needs before it can be supported is one table row: the
 * display.bin size and digest, the project ID, the panel, and the DRAM the
 * vendor uses. All of it is readable. None of it needs a write, and none of
 * it needs the board to be in this room -- which is the point, because the
 * table can only ever describe boards someone has held, and the refusals it
 * produces fire before the owner learns anything.
 *
 * What this deliberately does not do is start the coprocessor or light the
 * panel. On an unidentified board the panel timing is a guess, and a guess
 * belongs on a bench, not in someone's living room.
 */

#define H713_PROBE_DRAM_OFF	0x38
#define H713_PROBE_DRAM_WORDS	24

static const char * const h713_probe_dram_names[H713_PROBE_DRAM_WORDS] = {
	"clk", "type", "zq", "odt_en", "para1", "para2",
	"mr0", "mr1", "mr2", "mr3",
	"tpr0", "tpr1", "tpr2", "tpr3", "tpr4", "tpr5",
	"tpr6", "tpr7", "tpr8", "tpr9", "tpr10", "tpr11", "tpr12", "tpr13",
};

static u8 h713_probe_sector[512] __aligned(ARCH_DMA_MINALIGN);

/*
 * Two things the sections below find and the profile row at the end has to
 * quote: the clock the vendor's own boot0 declares, and the partition the
 * display artifacts were actually read from. Both are zero when the search
 * for them came up empty, and the row says so rather than guessing.
 */
static u32 h713_probe_dram_clk;
static char h713_probe_dev_used[16];

/*
 * The vendor's own DRAM settings, from the boot0 header on the eMMC. Not from
 * the firmware image: the flash tool patches para2 and tpr13 on the way in, so
 * the copy on the device is the one that describes the running board. What our
 * builds ship is not this either -- tpr0..tpr2 are computed from the clock --
 * but zq, para1, the mode registers and tpr3..tpr12 are taken from here
 * verbatim, and tpr11/tpr12 are per-board PHY tuning that cannot be derived.
 *
 * The BootROM reads boot0 from LBA 16 and falls back to LBA 256, and a stock
 * device carries it in both. A device this port was installed on does not: LBA
 * 16 holds our SPL and LBA 256 went with the rest of the image (doku/109). Our
 * SPL has an eGON.BT0 header too, so the magic alone lets it through -- the
 * first run on an HY310 printed the SPL's device-tree name as DRAM settings.
 * Mainline marks its header with "SPL" at 0x14, where boot0 keeps its header
 * size, and a real DRAM block has a sane clock and type; check both.
 */
static const lbaint_t h713_probe_boot0_lbas[] = { 16, 256 };

static bool h713_probe_sector_empty(void)
{
	uint i;

	for (i = 0; i < sizeof(h713_probe_sector); i++)
		if (h713_probe_sector[i])
			return false;

	return true;
}

static bool h713_probe_dram_plausible(void)
{
	u32 clk = get_unaligned_le32(h713_probe_sector + H713_PROBE_DRAM_OFF);
	u32 type = get_unaligned_le32(h713_probe_sector +
				      H713_PROBE_DRAM_OFF + 4);

	/* DDR2, DDR3, LPDDR3; clocks the H713 DRAM code accepts. */
	return clk >= 300 && clk <= 1200 &&
	       (type == 2 || type == 3 || type == 7);
}

static void h713_probe_boot0(int devnum)
{
	struct blk_desc *desc;
	uint n, i;

	printf("\n-- vendor boot0 (mmc %d) --\n", devnum);

	desc = blk_get_devnum_by_uclass_id(UCLASS_MMC, devnum);
	if (!desc) {
		printf("   no mmc %d\n", devnum);
		return;
	}

	for (n = 0; n < ARRAY_SIZE(h713_probe_boot0_lbas); n++) {
		lbaint_t lba = h713_probe_boot0_lbas[n];

		if (blk_dread(desc, lba, 1, h713_probe_sector) != 1) {
			printf("   LBA %lu: read failed\n", (ulong)lba);
			continue;
		}
		if (memcmp(h713_probe_sector + 4, "eGON.BT0", 8)) {
			printf("   LBA %lu: %s\n", (ulong)lba,
			       h713_probe_sector_empty() ? "empty" :
			       "no boot0 header");
			continue;
		}
		if (!memcmp(h713_probe_sector + 0x14, SPL_SIGNATURE, 3)) {
			printf("   LBA %lu: a mainline U-Boot SPL, not the "
			       "vendor boot0\n", (ulong)lba);
			continue;
		}
		if (!h713_probe_dram_plausible()) {
			printf("   LBA %lu: boot0 header, but nothing at 0x%x "
			       "looks like DRAM settings\n", (ulong)lba,
			       H713_PROBE_DRAM_OFF);
			continue;
		}

		printf("   LBA %lu: vendor boot0\n", (ulong)lba);
		for (i = 0; i < H713_PROBE_DRAM_WORDS; i++) {
			u32 v = get_unaligned_le32(h713_probe_sector +
						   H713_PROBE_DRAM_OFF + i * 4);

			printf("   dram_%-6s 0x%08x", h713_probe_dram_names[i],
			       v);
			if (i == 0) {
				printf("   %u MHz", v);
				h713_probe_dram_clk = v;
			}
			printf("\n");
		}
		return;
	}

	printf("   no vendor boot0 left on this eMMC -- it has been replaced.\n"
	       "   The DRAM settings are still in the stock firmware image or\n"
	       "   in a dump taken before the replacement.\n");
}

/*
 * Where the vendor FAT sits is not the same on every layout: a stock device
 * keeps it on bootloader_b, bootloader_a holds a byte-identical copy, and a
 * device this port was installed on has the files on hy310-boot. Stock
 * first, since that is who the probe is for; each miss costs one failed open.
 * The first attempt uses whatever the environment resolves to, and that one
 * is not tried twice.
 */
static const char * const h713_probe_devs[] = { "1:2", "1:1", "1#hy310-boot" };

/* Set by the search below, consumed by the report. */
static loff_t h713_probe_fw_len;

static void h713_probe_report(void)
{
	loff_t len;
	ulong site;
	u32 project;

	printf("H713 probe: read from %s %s:%s\n", H713_DISP_FS_IF,
	       h713_disp_fs_dev(), h713_disp_fs_path());
	strlcpy(h713_probe_dev_used, h713_disp_fs_dev(),
		sizeof(h713_probe_dev_used));

	if (h713_mips_accept_size((ulong)h713_probe_fw_len))
		return;
	/* Prints the digest and either the board name or "new revision". */
	if (h713_mips_verify())
		return;

	/*
	 * Only located, never patched: the probe does not start the
	 * firmware, so there is no wait to defuse. "Not patched" read like
	 * a fault to the first person who saw it -- say what it is for.
	 */
	site = h713_mips_find_hdcp_wait();
	if (site)
		printf("H713 MIPS: HDCP 1.4 key-wait loop found at 0x%08lx "
		       "(normal boot defuses it; the probe only locates it)\n",
		       site);

	/*
	 * The panel follows the ID the board declares, so report the ID, where
	 * it was read, and what this build knows about it. An ID with no panel
	 * is the interesting case and the reason the probe exists: it is a row
	 * nobody has filled in yet, not a fault.
	 */
	if (!h713_disp_declared_project(&project)) {
		const struct h713_panel_cfg *panel =
			h713_panel_by_project(project);

		if (panel)
			printf("H713 probe: project 0x%02x, panel %ux%u "
			       "%s-port\n", project, panel->width,
			       panel->height,
			       panel->dual_port ? "dual" : "single");
		else
			printf("H713 probe: project 0x%02x is not in this "
			       "build's panel table -- the panel has to be "
			       "measured on the board, it cannot be guessed\n",
			       project);
	}

	if (h713_disp_read("LogoRegData.bin", H713_DISP_LOGO_ADDR, &len))
		return;
	h713_disp_list(H713_DISP_LOGO_ADDR);
}

static void h713_probe_display(void)
{
	const char *saved = env_get("h713_mips_dev");
	char keep[16] = "";
	char tried[16];
	bool found;
	uint i;

	if (saved)
		strlcpy(keep, saved, sizeof(keep));
	strlcpy(tried, h713_disp_fs_dev(), sizeof(tried));

	printf("\n-- display artifacts --\n");

	found = !h713_disp_read("display.bin", H713_MIPS_FW_ADDR,
				&h713_probe_fw_len);
	for (i = 0; !found && i < ARRAY_SIZE(h713_probe_devs); i++) {
		if (!strcmp(tried, h713_probe_devs[i]))
			continue;
		printf("H713 probe: trying %s %s instead\n",
		       H713_DISP_FS_IF, h713_probe_devs[i]);
		env_set("h713_mips_dev", h713_probe_devs[i]);
		found = !h713_disp_read("display.bin", H713_MIPS_FW_ADDR,
					&h713_probe_fw_len);
	}

	if (found)
		h713_probe_report();
	else
		printf("H713 probe: no display.bin found. Set h713_mips_dev "
		       "to the partition holding the vendor FAT and run "
		       "h713_probe again.\n");

	env_set("h713_mips_dev", keep[0] ? keep : NULL);
}

/*
 * The profile row.
 *
 * Everything above is written for a person watching a console. This is
 * written for the file that comes out of it. One key: value line per field,
 * the field named exactly as the installer's identify() names it, so a
 * stranger pastes his log into an issue and profiles/<id>.py is written from
 * it by hand, without anyone here ever holding the board.
 *
 * It repeats values the sections above already printed. That is deliberate:
 * the sections are prose and change when the prose is wrong, the row is a
 * record and the two must not be the same thing.
 */
#define H713_PROBE_SECSTORE_LBA		12288U
#define H713_PROBE_SECSTORE_MAGIC	0x17253948
#define H713_PROBE_SECSTORE_ITEM0	16U	/* +0x2000: the first store_object_t */

/*
 * The one region of this eMMC that is never written and never read out: the
 * sunxi secure storage from LBA 12288, holding the HDCP 1.4 and 2.2 keys, the
 * WLAN and Bluetooth MAC addresses and the serial number (doku/109 2.3,
 * doku/68). None of it is recoverable if it is lost, and none of it exists in
 * any firmware image.
 *
 * Its shape, as read off an HY310 (doku/68): the first 4 KiB item is the map,
 * a NUL-separated list of "name:size" entries in plain text; the items follow
 * from +0x2000, each a store_object_t that starts with the magic 0x17253948,
 * then an id and a 64-byte name. So the signature to look for is the first
 * item's magic, sixteen sectors in -- not the map, which carries none.
 *
 * The row reports the shape and nothing else: how many entries the map lists,
 * that the item magic is there, and what the first item calls itself. No
 * contents and no digest: what is there are keys. The point is to tell a
 * foreign board's owner where his keys are so his profile can lock the region,
 * and to show that the hole our layout leaves is in the right place. When the
 * signature is missing the row says only that; what another scheme keeps at
 * that address is unknown here, and unknown may mean keys.
 */
static void h713_probe_row_secure_storage(struct blk_desc *desc)
{
	static u8 item[512] __aligned(ARCH_DMA_MINALIGN);
	const u8 *map = h713_probe_sector;
	const u8 *name = item + 8;
	uint i, j, entries = 0;

	if (!desc ||
	    blk_dread(desc, H713_PROBE_SECSTORE_LBA, 1, h713_probe_sector) != 1 ||
	    blk_dread(desc, H713_PROBE_SECSTORE_LBA + H713_PROBE_SECSTORE_ITEM0,
		      1, item) != 1) {
		printf("secure_storage: unreadable at LBA %u\n",
		       H713_PROBE_SECSTORE_LBA);
		return;
	}

	/* The map: printable, NUL-terminated tokens until the first empty one. */
	for (i = 0; i < sizeof(h713_probe_sector) && map[i]; i = j + 1) {
		for (j = i; j < sizeof(h713_probe_sector) &&
		     map[j] >= 0x20 && map[j] <= 0x7e; j++)
			;
		if (j == i || j >= sizeof(h713_probe_sector) || map[j])
			break;
		entries++;
	}

	if (get_unaligned_le32(item) != H713_PROBE_SECSTORE_MAGIC) {
		printf("secure_storage: no sunxi item signature at LBA %u "
		       "(map at LBA %u lists %u entries)\n",
		       H713_PROBE_SECSTORE_LBA + H713_PROBE_SECSTORE_ITEM0,
		       H713_PROBE_SECSTORE_LBA, entries);
		return;
	}

	printf("secure_storage: sunxi at LBA %u (map lists %u entries; item "
	       "magic 0x%08x at LBA %u", H713_PROBE_SECSTORE_LBA, entries,
	       H713_PROBE_SECSTORE_MAGIC,
	       H713_PROBE_SECSTORE_LBA + H713_PROBE_SECSTORE_ITEM0);
	for (i = 0; i < 64 && name[i] >= 0x20 && name[i] <= 0x7e; i++)
		;
	if (i && i < 64 && !name[i])
		printf(", first item \"%s\"", (const char *)name);
	printf(")\n");
}

static void h713_probe_row_layout(struct blk_desc *desc)
{
	struct disk_partition info;
	int part, n = 0;

	if (!desc) {
		printf("layout.entries: unknown -- no mmc 1\n");
		return;
	}

	for (part = 1; part <= MAX_SEARCH_PARTITIONS; part++)
		if (!part_get_info(desc, part, &info))
			n++;

	printf("layout.entries: %d\n", n);
}

/*
 * A stock device keeps display.bin twice, on bootloader_a and bootloader_b,
 * and the two copies are not always the same file -- on an HY310 the
 * display_cfg.xml beside them differs. Which one the board is using is the
 * active slot, and whether the other agrees is the first thing a restore has
 * to know, so both are hashed. The scratch address is the logo buffer, which
 * the sections above are finished with by the time this runs.
 */
#define H713_PROBE_SLOT_MAX	0x300000UL	/* logo buffer to CPU_COMM */

static int h713_probe_slot_digest(const char *dev, u8 *digest, loff_t *len)
{
	char path[64];

	snprintf(path, sizeof(path), "%s/display.bin", h713_disp_fs_path());

	if (fs_set_blk_dev(H713_DISP_FS_IF, dev, FS_TYPE_ANY))
		return -ENODEV;
	if (fs_size(path, len) || !*len)
		return -ENOENT;
	/* Unknown board, unknown file: the buffer ends at the share region. */
	if ((ulong)*len > H713_PROBE_SLOT_MAX)
		return -EFBIG;
	if (fs_set_blk_dev(H713_DISP_FS_IF, dev, FS_TYPE_ANY))
		return -ENODEV;
	if (fs_read(path, H713_DISP_LOGO_ADDR, 0, 0, len) || !*len)
		return -ENOENT;

	sha256_csum_wd((const u8 *)H713_DISP_LOGO_ADDR, (uint)*len, digest,
		       CHUNKSZ_SHA256);
	return 0;
}

static void h713_probe_row_slot(const char *field, const char *dev,
				u8 *digest, loff_t *len, int *ret)
{
	*ret = h713_probe_slot_digest(dev, digest, len);

	printf("%s: ", field);
	if (*ret == -EFBIG) {
		printf("too large to hash here, %llu bytes (%s %s)\n", *len,
		       H713_DISP_FS_IF, dev);
		return;
	}
	if (*ret) {
		printf("absent (%s %s)\n", H713_DISP_FS_IF, dev);
		return;
	}
	h713_mips_print_digest(digest);
	printf(" %llu\n", *len);
}

/*
 * Where the board said what it is: the partition panel_config.ini was read
 * from by h713_disp_declared_project() above, found by name (Reserve0_<slot>,
 * Reserve0, media_data), or the environment. Nothing is searched twice.
 */
static void h713_probe_row_panel_ini(void)
{
	if (!h713_disp_project_src[0])
		printf("panel_config.ini: not found (%s by name, then %s)\n",
		       h713_disp_ini_parts[0].name, h713_disp_ini_parts[1].name);
	else if (!strcmp(h713_disp_project_src, "env"))
		printf("panel_config.ini: not read -- h713_project is set in "
		       "the environment\n");
	else
		printf("panel_config.ini: %s %s\n", H713_DISP_FS_IF,
		       h713_disp_project_src);
}

static void h713_probe_profile_row(void)
{
	u8 digest[SHA256_SUM_LEN], da[SHA256_SUM_LEN], db[SHA256_SUM_LEN];
	struct blk_desc *desc;
	loff_t la = 0, lb = 0;
	int ra, rb;
	ulong site;
	u32 project;

	desc = blk_get_devnum_by_uclass_id(UCLASS_MMC, 1);

	printf("\n-- profile row --\n");

	printf("board: %s\n", h713_mips_fw ? h713_mips_fw->board : "unknown");

	if (h713_probe_dram_clk)
		printf("dram_clk_mhz: %u\n", h713_probe_dram_clk);
	else
		printf("dram_clk_mhz: unknown -- no vendor boot0 on this "
		       "eMMC; this build ran at %u\n", (uint)CONFIG_DRAM_CLK);

	h713_probe_row_layout(desc);

	if (h713_probe_fw_len) {
		sha256_csum_wd((const u8 *)H713_MIPS_FW_ADDR,
			       h713_mips_fw_size, digest, CHUNKSZ_SHA256);
		printf("mips.display_bin.size: %llu\n", h713_probe_fw_len);
		printf("mips.display_bin.sha256: ");
		h713_mips_print_digest(digest);
		printf("\n");
	} else {
		printf("mips.display_bin.size: unknown -- none found\n");
		printf("mips.display_bin.sha256: unknown -- none found\n");
	}

	/*
	 * The declared ID is a property of the board, read the way the boot
	 * path reads it (env h713_project, else panel_config.ini by name on
	 * Reserve0 or media_data); the firmware table does not name one, since
	 * one image serves boards with different panels.
	 */
	if (!h713_disp_declared_project(&project))
		printf("panel.declared_project_id: 0x%02x\n", project);
	else
		printf("panel.declared_project_id: unknown -- h713_project unset "
		       "and no %s found\n", H713_DISP_PANEL_INI);

	site = h713_mips_find_hdcp_wait();
	if (site)
		printf("hdcp_wait_va: 0x%08lx\n", site);
	else
		printf("hdcp_wait_va: not found\n");

	/*
	 * active_slot is what misc says ("_a"/"_b"; "_a" when there is no misc
	 * or no control block), the same meaning the installer's identify()
	 * gives that key. Where the artifacts were actually read from is the
	 * next line, since on our layout that is neither slot.
	 */
	printf("active_slot: %s\n", h713_disp_slot_suffix());
	printf("mips.source: %s %s\n", H713_DISP_FS_IF,
	       h713_probe_dev_used[0] ? h713_probe_dev_used : "none");

	h713_probe_row_slot("mips.bootloader_a", "1#bootloader_a", da, &la, &ra);
	h713_probe_row_slot("mips.bootloader_b", "1#bootloader_b", db, &lb, &rb);
	printf("mips.bootloader_a_equals_b: %s\n",
	       ra || rb ? "unknown -- one of the two is missing" :
	       (la == lb && !memcmp(da, db, SHA256_SUM_LEN)) ? "yes" : "no");

	h713_probe_row_panel_ini();

	h713_probe_row_secure_storage(desc);

	printf("\nThe stock boot chain starts the MIPS in its logo path. This\n"
	       "probe never does: every line above was read, hashed or\n"
	       "counted with the coprocessor held in reset and the panel dark.\n");
}

static int do_h713_probe(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	bool was = h713_probe_mode;

	if (argc != 1)
		return CMD_RET_USAGE;

	printf("H713 probe: reading only -- nothing is written to the eMMC,\n"
	       "            the coprocessor stays in reset, the panel stays "
	       "dark.\n");

	h713_probe_mode = true;

	printf("\n-- partitions (mmc 1) --\n");
	run_command("part list mmc 1", 0);

	h713_probe_boot0(1);
	h713_probe_display();
	h713_probe_profile_row();

	h713_probe_mode = was;

	printf("\nH713 probe: done. Paste everything above into the issue.\n");

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_probe, 1, 0, do_h713_probe,
	   "report what this board is, without writing anything",
	   "\n"
	   "    Reads the vendor boot0 DRAM settings, the partition table and\n"
	   "    the display firmware, and prints what a support table row would\n"
	   "    need: size, SHA-256, project IDs, panel, HDCP wait site.\n"
	   "    An unknown firmware revision is a result here, not a refusal.\n"
	   "    Nothing is written and the panel is never driven."
);
