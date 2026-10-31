// SPDX-License-Identifier: GPL-2.0+
/*
 * H713: hand U-Boot's own console output to Linux on a board without a serial
 * console.
 *
 * With CONFIG_CONSOLE_RECORD every character U-Boot prints after relocation
 * also lands in gd->console_out. `h713_conlog <addr>` copies what has been
 * recorded so far to <addr> behind a 16-byte header and sets ${filesize} to
 * the whole length, so that the environment can put it somewhere Linux reads:
 *
 *	h713_conlog 0x45000000; mmc dev 1; mmc write 0x45000000 0x2800 0x400
 *
 * Header: "H713CLOG", u32 payload length, u32 flags (bit 0: the recording
 * overflowed and lost its tail). The copy drains the record, so a second call
 * returns only what was printed in between.
 */

#include <command.h>
#include <console.h>
#include <env.h>
#include <membuf.h>
#include <asm/global_data.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

#define H713_CONLOG_MAGIC	"H713CLOG"
#define H713_CONLOG_HDR		16

static int do_h713_conlog(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct membuf *mb = (struct membuf *)&gd->console_out;
	char *dst, *data;
	u32 len = 0, flags;
	int n;

	if (argc != 2)
		return CMD_RET_USAGE;
	if (!mb->start) {
		printf("h713_conlog: no console record buffer\n");
		return CMD_RET_FAILURE;
	}
	flags = (gd->flags & GD_FLG_RECORD_OVF) ? 1 : 0;
	dst = (char *)hextoul(argv[1], NULL);

	/* The buffer is circular: getraw hands back at most two runs. */
	while ((n = membuf_getraw(mb, -1, true, &data)) > 0) {
		memcpy(dst + H713_CONLOG_HDR + len, data, n);
		len += n;
	}
	memcpy(dst, H713_CONLOG_MAGIC, 8);
	memcpy(dst + 8, &len, 4);
	memcpy(dst + 12, &flags, 4);
	env_set_hex("filesize", H713_CONLOG_HDR + len);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(h713_conlog, 2, 0, do_h713_conlog,
	   "copy the recorded console output to memory",
	   "<addr> - header \"H713CLOG\", u32 length, u32 flags, then the text");
