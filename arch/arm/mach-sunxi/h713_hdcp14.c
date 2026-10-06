// SPDX-License-Identifier: GPL-2.0+
/*
 * H713: hand this device's own HDCP 1.4 licence back to its receiver.
 *
 * The board stores its HDCP 1.4 key set in its own secure storage. Nothing in
 * software can read the key set itself, and nothing here tries: this file reads
 * the stored record, leaves it where our own EL3 service asks for it, and lets
 * the Crypto Engine do the rest. The key set goes from the engine straight into
 * the receiver's key sink and never appears in memory this code can read.
 *
 * The service takes no address and serves one sink write per boot: EL3 says
 * where the record goes, checks it again itself, and closes the service after
 * the first write (sunxi_h713_sip.c in the TF-A fork). When this file writes
 * nothing it closes the service itself - at every exit of
 * h713_hdcp14_install() that makes no write, and in arch_preboot_os() for boots
 * that never get there and start an OS with bootm. That holds against an OS
 * that cannot write BL31's memory. On this board BL31 sits in DRAM that nothing
 * protects from the normal world, so for root the lock is a hurdle, not a
 * boundary. Boots without "h713_disp init" that start their OS other than
 * through bootm (bootefi, go, bootelf) are not closed at all.
 *
 * Why it belongs in the boot loader: the display firmware polls the sink's
 * "key loaded" bit in Rx_HDCP14_LoadKey and gives up early, so the key has to be
 * in place BEFORE the firmware is released (doku/112, doku/145).
 *
 * Everything on the far side is ours as well - our service in BL31, no vendor
 * Trusted OS in the image (doku/147 ch. 7, doku/148).
 *
 * MEASURED FACTS THIS CODE RELIES ON (doku/148 §0, do not "simplify" them away):
 *
 *   - The engine refuses key-select 3 (the hardware key) towards readable DRAM
 *     with ESR 0x11, and accepts it towards the sink at 0x03041400. That is a
 *     design rule, not a bug: the first would be key export.
 *   - It also refuses it when the DMA source lies outside the Trusted OS window.
 *     Hence the stage in that window, which EL3 names - this is the one step
 *     that made the difference, after hours of looking for the fault elsewhere.
 *   - The stored bytes need no unwrapping on this device: the vendor's key box
 *     only decrypts when the item's "encrypted" flag is set, and here op 1 built
 *     no decrypt task at all. The stored 288 bytes are byte-identical to the
 *     source of the vendor's own engine task (0 of 288 differ). So they go to the
 *     sink unchanged, and no wrapping key is needed anywhere.
 *   - The key set is NOT device-bound: three secure-storage regions from two
 *     devices with two different serial numbers carry one and the same key set.
 *     It is still a DRM secret and must never be written into the source tree.
 *   - The sink survives a warm reset. An "Enabled" measured after an earlier push
 *     says nothing about the current one, so a measurement only counts as a
 *     pair of consecutive boots: wipe (h713_hdcp14_wipe=1) -> "Desired", then
 *     the default boot -> "Enabled". Each boot has one sink write, so the two
 *     steps cannot share one.
 *
 * Nothing in here may ever stop the boot. Every failure prints one line and
 * returns: a device without a usable key set must still come up, just without
 * protected playback.
 */

#include <command.h>
#include <blk.h>
#include <bootm.h>
#include <env.h>
#include <cpu_func.h>
#include <linux/arm-smccc.h>
#include <part.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/global_data.h>
#include <asm/unaligned.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Our own EL3 service. STAGE answers where the record goes (a1) and how much
 * room there is (a2); ONCE is the one sink write of this boot - the staged key
 * set, zeros, or nothing - and closes the service whatever it was. Neither
 * takes an address.
 */
#define H713_SIP_HDCP14_STAGE	0x8200001dUL
#define H713_SIP_HDCP14_ONCE	0x8200001eUL
#define HDCP14_KEY_SET		0UL
#define HDCP14_ZEROS		1UL
#define HDCP14_NOTHING		2UL
#define H713_SIP_E_CLOSED	0x17UL		/* already written or closed */

/*
 * The secure storage. Region and slot size are the device's layout; everything
 * inside a slot is found by name and length, never by a constant offset, so a
 * device that orders its items differently still works (doku/148 §1).
 */
#define SS_LBA			0x3000UL	/* region start, LBA */
#define SS_SLOT_BYTES		4096UL
#define SS_SLOT_BLOCKS		(SS_SLOT_BYTES / 512UL)
#define SS_SCAN_SLOTS		32UL		/* items live in the first few */
#define SS_STAGE_BYTES		(2 * SS_SLOT_BYTES)	/* original and copy */
#define SS_MAGIC		0x17253948UL
#define SS_NAME_OFF		0x08UL
#define SS_NAME_MAX		0x40UL
#define SS_OUTER_LEN_OFF	0x60UL
#define SS_OUTER_DATA_OFF	0x64UL
/* Blob items nest a second header in their payload. */
#define SS_INNER_LEN_OFF	0x40UL		/* relative to the payload */
#define SS_INNER_DATA_OFF	0x4cUL		/* relative to the payload */
#define SS_ITEM_NAME		"hdcpkey"
#define HDCP14_RECORD_LEN	320UL		/* 288 for the engine + 32 tail */
#define HDCP14_KEY_LEN		288UL		/* what the engine is handed */
#define MMC_DEV			1

/*
 * The watchdog as a safety net around the push. The engine task takes
 * microseconds; if it ever wedges the bus, a reset is far better than a board
 * parked at the logo with no way back (a real power cut is not available while
 * USB feeds the board). Armed only if nobody else is driving it - U-Boot's own
 * driver stays off on this board ("WDT: Not starting watchdog@2051000"), and
 * double-driving it would be worse than no net at all.
 *
 * Feeding needs a real off/on pair: rewriting the active MODE register does NOT
 * restart the counter (cost us four needless resets to work out).
 */
#define WDT_CFG			0x02051010UL
#define WDT_MODE		0x02051014UL
#define WDT_KEY			0x16aa0000UL
#define WDT_CFG_SYSTEM		(WDT_KEY | 0x1UL)
#define WDT_MODE_EN		0x1UL
#define WDT_INTERVAL_6S		(0x6UL << 4)

static bool wdt_armed_by_us;

static void wdt_guard_on(void)
{
	wdt_armed_by_us = false;
	if ((readl((void *)WDT_MODE) & WDT_MODE_EN) != 0UL)
		return;			/* somebody else owns it - hands off */
	writel(WDT_CFG_SYSTEM, (void *)WDT_CFG);
	writel(WDT_KEY | WDT_INTERVAL_6S | WDT_MODE_EN, (void *)WDT_MODE);
	wdt_armed_by_us = true;
}

static void wdt_guard_off(void)
{
	if (!wdt_armed_by_us)
		return;
	writel(WDT_KEY | WDT_INTERVAL_6S, (void *)WDT_MODE);
	wdt_armed_by_us = false;
}

/* Bounded compare of a NUL-terminated name field against a literal. */
static bool name_is(const u8 *slot, ulong off, const char *want)
{
	ulong i;

	for (i = 0; i < SS_NAME_MAX; i++) {
		u8 c = slot[off + i];

		if (want[i] == '\0')
			return c == 0;
		if (c != (u8)want[i])
			return false;
	}
	return false;			/* no terminator inside the field */
}

static bool all_same(const u8 *p, ulong len, u8 v)
{
	ulong i;

	for (i = 0; i < len; i++)
		if (p[i] != v)
			return false;
	return true;
}

/*
 * Pull the "hdcpkey" payload out of one slot, with every structural check from
 * doku/148 §1. Returns the payload length, or a negative error.
 */
static int slot_hdcp14_payload(const u8 *slot, const u8 **payload)
{
	u32 outer, inner;
	const u8 *data;

	if (get_unaligned_le32(slot) != SS_MAGIC)
		return -ENOENT;		/* empty slot, not an error */
	if (!name_is(slot, SS_NAME_OFF, SS_ITEM_NAME))
		return -ENOENT;		/* some other item */

	outer = get_unaligned_le32(slot + SS_OUTER_LEN_OFF);
	if (outer == 0U || SS_OUTER_DATA_OFF + outer > SS_SLOT_BYTES) {
		printf("H713 HDCP: item length %u is out of range\n", outer);
		return -EILSEQ;
	}
	data = slot + SS_OUTER_DATA_OFF;

	/* A blob item repeats its name and carries its own length. */
	if (!name_is(data, 0, SS_ITEM_NAME)) {
		printf("H713 HDCP: item is not the expected blob form\n");
		return -EILSEQ;
	}
	inner = get_unaligned_le32(data + SS_INNER_LEN_OFF);
	if (SS_INNER_DATA_OFF + inner != outer) {
		printf("H713 HDCP: inner length %u does not fit outer %u\n",
		       inner, outer);
		return -EILSEQ;
	}
	if (inner < HDCP14_KEY_LEN) {
		printf("H713 HDCP: key set is only %u bytes, need %lu\n",
		       inner, HDCP14_KEY_LEN);
		return -EILSEQ;
	}
	*payload = data + SS_INNER_DATA_OFF;
	return (int)inner;
}

/*
 * Find the key item in the secure storage and leave both stored copies - the
 * two slots, unchanged - at `stage`, where EL3 checks them again and takes the
 * key set from. The device keeps every item twice; both copies must agree,
 * otherwise the record is damaged and we would be pushing rubbish into the sink.
 */
static int read_key_set(u8 *stage)
{
	static u8 slot[SS_SLOT_BYTES] __aligned(64);
	const u8 *first = NULL;
	struct blk_desc *desc;
	ulong i;
	int ret = -ENOENT;

	desc = blk_get_devnum_by_uclass_id(UCLASS_MMC, MMC_DEV);
	if (!desc) {
		printf("H713 HDCP: mmc %d is not there\n", MMC_DEV);
		return -ENODEV;
	}

	for (i = 0; i < SS_SCAN_SLOTS; i++) {
		const u8 *payload = NULL;
		int len;

		if (blk_dread(desc, SS_LBA + i * SS_SLOT_BLOCKS,
			      SS_SLOT_BLOCKS, slot) != SS_SLOT_BLOCKS) {
			printf("H713 HDCP: cannot read slot %lu\n", i);
			ret = -EIO;
			break;
		}
		len = slot_hdcp14_payload(slot, &payload);
		if (len == -ENOENT)
			continue;
		if (len < 0) {
			ret = len;	/* structurally broken - say so, move on */
			continue;
		}
		if (all_same(payload, HDCP14_KEY_LEN, 0x00) ||
		    all_same(payload, HDCP14_KEY_LEN, 0xff)) {
			printf("H713 HDCP: slot %lu holds no key (all 0x%02x)\n",
			       i, payload[0]);
			ret = -EILSEQ;
			continue;
		}
		if (!first) {
			memcpy(stage, slot, SS_SLOT_BYTES);
			first = stage + (payload - slot);
			ret = -EAGAIN;	/* found one, still want the copy */
			continue;
		}
		if (memcmp(first, payload, HDCP14_KEY_LEN) != 0) {
			printf("H713 HDCP: the two stored copies differ - "
			       "refusing to push a damaged key set\n");
			ret = -EBADMSG;
			break;
		}
		memcpy(stage + SS_SLOT_BYTES, slot, SS_SLOT_BYTES);
		ret = 0;
		break;
	}

	if (ret == -EAGAIN) {
		printf("H713 HDCP: only one copy of the key set found\n");
		ret = -EBADMSG;
	}
	if (ret == -ENOENT)
		printf("H713 HDCP: no '%s' item in the secure storage\n",
		       SS_ITEM_NAME);

	memset(slot, 0, sizeof(slot));
	if (ret)
		memset(stage, 0, SS_STAGE_BYTES);
	return ret;
}

/*
 * Where EL3 wants the record: its answer, checked against the RAM U-Boot
 * knows, or 0 with the reason printed.
 */
static ulong hdcp14_stage(void)
{
	struct arm_smccc_res res = { 0 };

	arm_smccc_smc(H713_SIP_HDCP14_STAGE, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == (unsigned long)-1) {
		printf("H713 HDCP: this BL31 has no key-sink service\n");
		return 0;
	}
	if (res.a0 == H713_SIP_E_CLOSED) {
		printf("H713 HDCP: the key sink is closed for this boot\n");
		return 0;
	}
	if (res.a0 || res.a2 < SS_STAGE_BYTES || (res.a1 & 63) ||
	    res.a1 < gd->ram_base || res.a1 + SS_STAGE_BYTES > gd->ram_top) {
		printf("H713 HDCP: EL3 names no usable stage (0x%lx at 0x%lx)\n",
		       res.a0, res.a1);
		return 0;
	}
	return res.a1;
}

/*
 * The one sink write of this boot, under the watchdog: the staged key set,
 * zeros, or nothing. A stage, if given, is wiped afterwards.
 */
static int hdcp14_once(ulong what, ulong stage, unsigned long *verdict)
{
	struct arm_smccc_res res = { 0 };

	if (stage)
		flush_dcache_range(stage, stage + SS_STAGE_BYTES);
	wdt_guard_on();
	arm_smccc_smc(H713_SIP_HDCP14_ONCE, what, 0, 0, 0, 0, 0, 0, &res);
	wdt_guard_off();

	/* Our SiP answers with SMC_RET1: the verdict is in a0, not a1. */
	*verdict = res.a0;
	if (stage) {
		memset((void *)stage, 0, SS_STAGE_BYTES);
		flush_dcache_range(stage, stage + SS_STAGE_BYTES);
	}
	if (res.a0 == (unsigned long)-1) {
		printf("H713 HDCP: this BL31 has no key-sink service\n");
		return -ENOSYS;
	}
	if (res.a0 == H713_SIP_E_CLOSED) {
		printf("H713 HDCP: the key sink is closed for this boot\n");
		return -EALREADY;
	}
	return res.a0 ? -EIO : 0;
}

/*
 * Close the service for this boot without writing anything. It says so only
 * when it really closed it: after a write it is closed already (E_CLOSED), and
 * a BL31 without the service answers SMC_UNK.
 */
static void hdcp14_close(void)
{
	struct arm_smccc_res res = { 0 };

	arm_smccc_smc(H713_SIP_HDCP14_ONCE, HDCP14_NOTHING, 0, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0 == 0)
		printf("H713 HDCP: key sink closed for this boot, nothing "
		       "written\n");
}

int h713_hdcp14_install(void)
{
	unsigned long verdict = 0;
	ulong stage;
	int ret;

	if (env_get_yesno("h713_hdcp14") == 0) {
		printf("H713 HDCP: skipped on request (h713_hdcp14=0)\n");
		hdcp14_close();
		return 0;
	}

	/*
	 * Clearing the sink is the only way to get a defined starting point for
	 * a measurement, because the sink survives a warm reset. It is also the
	 * proof that our push mechanism works at all: a source must fall back
	 * to "Desired" afterwards. It is this boot's one sink write.
	 */
	if (env_get_yesno("h713_hdcp14_wipe") == 1) {
		ret = hdcp14_once(HDCP14_ZEROS, 0, &verdict);
		printf("H713 HDCP: sink overwritten with zeros (0x%lx)\n",
		       verdict);
		return ret;
	}

	stage = hdcp14_stage();
	if (!stage) {
		hdcp14_close();
		return -ENOSYS;
	}
	ret = read_key_set((u8 *)stage);
	if (ret) {
		printf("H713 HDCP: no key set installed, booting without "
		       "protected playback\n");
		hdcp14_close();
		return ret;
	}

	ret = hdcp14_once(HDCP14_KEY_SET, stage, &verdict);
	if (ret == -EIO)
		printf("H713 HDCP: the sink refused the key set (0x%lx)\n",
		       verdict);
	if (ret)
		return ret;
	printf("H713 HDCP: 1.4 key set handed to the receiver\n");
	return 0;
}

/*
 * The backstop for boots that never ran h713_hdcp14_install() - a role without
 * "h713_disp init" - and start an OS with bootm: close the service before it.
 * Every other boot has closed it already, and this stays silent.
 */
void arch_preboot_os(void)
{
	hdcp14_close();
}

static int do_h713_hdcp14(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	return h713_hdcp14_install() ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_hdcp14, 1, 0, do_h713_hdcp14,
	   "hand this device's HDCP 1.4 key set to its receiver",
	   " - read the stored key set and put it into the receiver's sink\n"
	   "   h713_hdcp14=0      skip it entirely\n"
	   "   h713_hdcp14_wipe=1 push 288 zero bytes instead (clears the sink)\n"
	   "   one sink write per boot; the boot path normally makes it\n");
