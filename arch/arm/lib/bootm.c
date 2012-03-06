/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * Copyright (C) 2001  Erik Mouw (J.A.K.Mouw@its.tudelft.nl)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <common.h>
#include <command.h>
#include <image.h>
#include <u-boot/zlib.h>
#include <asm/byteorder.h>
#include <fdt.h>
#include <libfdt.h>
#include <fdt_support.h>

#include <android_image.h>
#include <mmc.h>
#include <asm/omap_common.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#define BIT(x)  (1<<(x))

#if defined (CONFIG_SETUP_MEMORY_TAGS) || \
    defined (CONFIG_CMDLINE_TAG) || \
    defined (CONFIG_INITRD_TAG) || \
    defined (CONFIG_SERIAL_TAG) || \
    defined (CONFIG_REVISION_TAG)
static void setup_start_tag (bd_t *bd);

# ifdef CONFIG_SETUP_MEMORY_TAGS
static void setup_memory_tags (bd_t *bd);
# endif
static void setup_commandline_tag (bd_t *bd, char *commandline);

# ifdef CONFIG_INITRD_TAG
static void setup_initrd_tag (bd_t *bd, ulong initrd_start,
			      ulong initrd_end);
# endif
static void setup_end_tag (bd_t *bd);

static struct tag *params;
#endif /* CONFIG_SETUP_MEMORY_TAGS || CONFIG_CMDLINE_TAG || CONFIG_INITRD_TAG */

static ulong get_sp(void);
#if defined(CONFIG_OF_LIBFDT)
static int bootm_linux_fdt(int machid, bootm_headers_t *images);
#endif

int get_boot_slot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
int do_booti(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static u64 get_start_sector(unsigned char *sector, char *ptn);
static u64 get_partition(struct mmc *mmc, char *ptn);

void arch_lmb_reserve(struct lmb *lmb)
{
	ulong sp;

	/*
	 * Booting a (Linux) kernel image
	 *
	 * Allocate space for command line and board info - the
	 * address should be as high as possible within the reach of
	 * the kernel (see CONFIG_SYS_BOOTMAPSZ settings), but in unused
	 * memory, which means far enough below the current stack
	 * pointer.
	 */
	sp = get_sp();
	debug("## Current stack ends at 0x%08lx ", sp);

	/* adjust sp by 1K to be safe */
	sp -= 1024;
	lmb_reserve(lmb, sp,
		    gd->bd->bi_dram[0].start + gd->bd->bi_dram[0].size - sp);
}

static void announce_and_cleanup(void)
{
	printf("\nStarting kernel ...\n\n");

#ifdef CONFIG_USB_DEVICE
	{
		extern void udc_disconnect(void);
		udc_disconnect();
	}
#endif
	cleanup_before_linux();
}

int do_bootm_linux(int flag, int argc, char *argv[], bootm_headers_t *images)
{
	bd_t	*bd = gd->bd;
	char	*s;
	int	machid = bd->bi_arch_number;
	void	(*kernel_entry)(int zero, int arch, uint params);

#ifdef CONFIG_CMDLINE_TAG
	char *commandline = getenv ("bootargs");
#endif

	if ((flag != 0) && (flag != BOOTM_STATE_OS_GO))
		return 1;

	s = getenv ("machid");
	if (s) {
		machid = simple_strtoul (s, NULL, 16);
		printf ("Using machid 0x%x from environment\n", machid);
	}

	show_boot_progress (15);

#ifdef CONFIG_OF_LIBFDT
	if (images->ft_len)
		return bootm_linux_fdt(machid, images);
#endif

	kernel_entry = (void (*)(int, int, uint))images->ep;

	debug ("## Transferring control to Linux (at address %08lx) ...\n",
	       (ulong) kernel_entry);

#if defined (CONFIG_SETUP_MEMORY_TAGS) || \
    defined (CONFIG_CMDLINE_TAG) || \
    defined (CONFIG_INITRD_TAG) || \
    defined (CONFIG_SERIAL_TAG) || \
    defined (CONFIG_REVISION_TAG)
	setup_start_tag (bd);
#ifdef CONFIG_SERIAL_TAG
	setup_serial_tag (&params);
#endif
#ifdef CONFIG_REVISION_TAG
	setup_revision_tag (&params);
#endif
#ifdef CONFIG_SETUP_MEMORY_TAGS
	setup_memory_tags (bd);
#endif
#ifdef CONFIG_CMDLINE_TAG
	setup_commandline_tag (bd, commandline);
#endif
#ifdef CONFIG_INITRD_TAG
	if (images->rd_start && images->rd_end)
		setup_initrd_tag (bd, images->rd_start, images->rd_end);
#endif
	setup_end_tag(bd);
#endif

	announce_and_cleanup();

	kernel_entry(0, machid, bd->bi_boot_params);
	/* does not return */

	return 1;
}

#if defined(CONFIG_OF_LIBFDT)
static int fixup_memory_node(void *blob)
{
	bd_t	*bd = gd->bd;
	int bank;
	u64 start[CONFIG_NR_DRAM_BANKS];
	u64 size[CONFIG_NR_DRAM_BANKS];

	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start[bank] = bd->bi_dram[bank].start;
		size[bank] = bd->bi_dram[bank].size;
	}

	return fdt_fixup_memory_banks(blob, start, size, CONFIG_NR_DRAM_BANKS);
}

static int bootm_linux_fdt(int machid, bootm_headers_t *images)
{
	ulong rd_len;
	void (*kernel_entry)(int zero, int dt_machid, void *dtblob);
	ulong of_size = images->ft_len;
	char **of_flat_tree = &images->ft_addr;
	ulong *initrd_start = &images->initrd_start;
	ulong *initrd_end = &images->initrd_end;
	struct lmb *lmb = &images->lmb;
	int ret;

	kernel_entry = (void (*)(int, int, void *))images->ep;

	boot_fdt_add_mem_rsv_regions(lmb, *of_flat_tree);

	rd_len = images->rd_end - images->rd_start;
	ret = boot_ramdisk_high(lmb, images->rd_start, rd_len,
				initrd_start, initrd_end);
	if (ret)
		return ret;

	ret = boot_relocate_fdt(lmb, of_flat_tree, &of_size);
	if (ret)
		return ret;

	debug("## Transferring control to Linux (at address %08lx) ...\n",
	       (ulong) kernel_entry);

	fdt_chosen(*of_flat_tree, 1);

	fixup_memory_node(*of_flat_tree);

	fdt_initrd(*of_flat_tree, *initrd_start, *initrd_end, 1);

	announce_and_cleanup();

	kernel_entry(0, machid, *of_flat_tree);
	/* does not return */

	return 1;
}
#endif

static u64 get_start_sector(unsigned char *sector, char *ptn)
{

	unsigned char name[72];
	int i, n;
	u64 start = 0;

	typedef struct {
		unsigned char type_uuid[16];
		unsigned char uniq_uuid[16];
		unsigned long long first_lba;
		unsigned long long last_lba;
		unsigned long long attr;
		unsigned short name[36];
	} efi_part_desc_t;

	efi_part_desc_t *entries = (efi_part_desc_t *) sector;

	for (i = 0; i < 4; i++, entries++) {
		n = 0;
		/* copying a simple unicode partition name */
		while (n < (sizeof(entries->name)-1)) {
			name[n] = entries->name[n];
			n++;
		}

		debug("booti: Name found %s\n", name);
		if (!strcmp((char *)name, ptn)) {
			start = entries->first_lba;
			break;
		}
	}

	return start;
}

static u64 get_partition(struct mmc *mmc, char *ptn)
{
	u64 start_sector = 0;
	unsigned char buffer[512];
	ulong block_count;
	int j;

	block_count = mmc->block_dev.block_read(1, 0, 1, &buffer);
	if (block_count == 0) {
		printf("booti: unable to read the MBR\n");
		return 0;
	}

	if (buffer[0x1fe] == 0x55 && buffer[0x1ff] == 0xaa) {
		debug("booti: MBR signature found\n");
	} else {
		printf("booti: MBR signature not found\n");
		return 0;
	}

	block_count = mmc->block_dev.block_read(1, 1, 1, &buffer);
	if (block_count == 0) {
		printf("booti: error reading primary GPT header\n");
		return 0;
	}

	if (memcmp(buffer, "EFI PART", 8)) {
		printf("booti: efi partition table not found\n");
		return 0;
	}

	/* searching for the boot partition */
	for (j = 2; j < 34; j++) {
		block_count = mmc->block_dev.block_read(1, j, 1, &buffer);
		if (block_count == 0) {
			printf("booti: error reading partition entries %d\n",
									j);
			return 0;
		}

		start_sector = get_start_sector(buffer, ptn);
		debug("booti: start sector %lld\n", start_sector);
		if (start_sector != 0)
			break;

	}
	return start_sector;
}

#define ALIGNPAGES(n, pagesz) ((n + (pagesz - 1)) & (~(pagesz - 1)))

int do_booti(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int err;
	unsigned address = 0;
	int mmc_controller = -1;
	char *ptn = "boot";
	unsigned char boothdr[512];
	struct andr_img_hdr *hdr = (void *) boothdr;
	bd_t *bd = gd->bd;
	ulong initrd_start, initrd_end;
	void (*theKernel)(int zero, int arch, uint params);

	if (argc < 2)
		return 1;

	if (!strcmp(argv[1], "mmc0"))
		mmc_controller = 0;
	else if (!strcmp(argv[1], "mmc1"))
		mmc_controller = 1;
	else
		address = simple_strtoul(argv[1], NULL, 16);

	if (argc > 2)
		ptn = argv[2];

	if (mmc_controller != -1) {

		ulong block_count;
		unsigned sector;
		u64 start_sector = 0;
		struct mmc *mmc;

		mmc = find_mmc_device(mmc_controller);
		if (!mmc) {
			printf("booti: mmc device not found!!\n");
			return 1;
		}

		err = mmc_init(mmc);
		if (err) {
			printf("booti: mmc init failed: err - %d\n", err);
			return 1;
		}

		/* allows the user to boot from recovery partition
		 but keep boot partition as the default */
		start_sector = get_partition(mmc, ptn);
		if (start_sector == 0) {
			printf("booti: Unable to find the boot partition\n");
			return 1;
		}

		/*
		reading from controller 1 and sector found earlier
		reading first sector, and storing it in *buffer
		*/
		block_count = mmc->block_dev.block_read(1, start_sector, 1,
								(void *)hdr);
		if (block_count == 0) {
			printf("booti: error loading boot image\n");
			return 1;
		}

		if (memcmp(hdr->magic, ANDR_BOOT_MAGIC, 8)) {
			printf("booti: bad boot image magic\n");
			return 1;
		}

		debug("booti: hdr->magic is = %s\n", hdr->magic);
		debug("booti: kernel addr and kern size = %u %d \n",
				hdr->kernel_addr, hdr->kernel_size);
		debug("booti: ramdisk addr and ramdisk size = %u %d \n",
				hdr->ramdisk_addr, hdr->ramdisk_size);

		sector = start_sector + (hdr->page_size / 512);

		/*
		reading from controller 1, start address = sector,
		reading kernel size and storing in kernel addr
		*/
		block_count = mmc->block_dev.block_read(1, sector,
		(hdr->kernel_size/512 + 1), (void *) hdr->kernel_addr);
		if (block_count == 0) {
			printf("booti: failed to load kernel : "
						"mmc blk read err\n");
			return 1;
		}

		sector += ALIGNPAGES(hdr->kernel_size, hdr->page_size) / 512;

		block_count = mmc->block_dev.block_read(1, sector,
		(hdr->ramdisk_size/512 + 1), (void *) hdr->ramdisk_addr);
		if (block_count == 0) {
			printf("booti: failed to read ramdisk : "
						"mmc blk read err\n");
			return 1;
		}

	} else {
		unsigned kernel_address;
		unsigned ramdisk_address;

		/* copy the header from the bootimg */
		memcpy(hdr, (void *) address, sizeof(*hdr));

		if (memcmp(hdr->magic, ANDR_BOOT_MAGIC, 8)) {
			printf("booti: bad boot image magic\n");
			return 1;
		}

		debug("booti: hdr->magic is = %s\n", hdr->magic);
		debug("booti: kernel addr and kern size = %u %d \n",
				hdr->kernel_addr, hdr->kernel_size);
		debug("booti: ramdisk addr and ramdisk size = %u %d \n",
				hdr->ramdisk_addr, hdr->ramdisk_size);

		kernel_address = address + hdr->page_size;
		ramdisk_address = kernel_address + ALIGN(hdr->kernel_size,
							hdr->page_size);

		memmove((void *) hdr->kernel_addr,  (void *) kernel_address,
							hdr->kernel_size);
		memmove((void *) hdr->ramdisk_addr, (void *) ramdisk_address,
							hdr->ramdisk_size);
	}

	printf("booti: kernel   @ %08x (%d)\n", hdr->kernel_addr,
							hdr->kernel_size);
	printf("booti: ramdisk  @ %08x (%d)\n", hdr->ramdisk_addr,
							hdr->ramdisk_size);

	theKernel = (void (*)(int, int, uint))(hdr->kernel_addr);

	initrd_start = hdr->ramdisk_addr;
	initrd_end = initrd_start + hdr->ramdisk_size;

	#ifdef CONFIG_SETUP_MEMORY_TAGS
	setup_start_tag(bd);
	#ifdef CONFIG_SERIAL_TAG
	setup_serial_tag(&params);
	#endif
	#ifdef CONFIG_REVISION_TAG
	setup_revision_tag(&params);
	#endif
	#ifdef CONFIG_SETUP_MEMORY_TAGS
	setup_memory_tags(bd);
	#endif
	#ifdef CONFIG_CMDLINE_TAG
	setup_commandline_tag(bd, hdr->cmdline);
	#endif
	#ifdef CONFIG_INITRD_TAG
	if (hdr->ramdisk_size)
		setup_initrd_tag(bd, initrd_start, initrd_end);
	#endif
	setup_end_tag(bd);
	#endif

	printf("\nStarting kernel ...\n\n");

	cleanup_before_linux();

	theKernel(0, bd->bi_arch_number, bd->bi_boot_params);

	return 1;
}

U_BOOT_CMD(
	booti,  3,      1,      do_booti,
	"booti   - boot android bootimg from memory\n",
	"<addr>\n    - boot application image stored in memory\n"
	"\t'addr' - address of boot image which contains zImage + ramdisk\n"
);

/**
*  get_boot_slot: Returns boot from SD or eMMC
* @ret: 1:SD   0:eMMC
*/
int get_boot_slot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc > 1)
		return 1;

	switch (omap_boot_device()) {
	case BOOT_DEVICE_MMC1:
		printf("current boot device is sd card\n");
		return 1;
		break;
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		printf("current boot device is emmc\n");
		return 0;
		break;
	}

	return 1;
}

U_BOOT_CMD(
	bootdevice,  3,      0,      get_boot_slot,
	"- get boot device we booted out from\n",
	"- returns 0 for eMMC and returns 1 for sd card boot\n"
);

#if defined (CONFIG_SETUP_MEMORY_TAGS) || \
    defined (CONFIG_CMDLINE_TAG) || \
    defined (CONFIG_INITRD_TAG) || \
    defined (CONFIG_SERIAL_TAG) || \
    defined (CONFIG_REVISION_TAG)
static void setup_start_tag (bd_t *bd)
{
	params = (struct tag *) bd->bi_boot_params;

	params->hdr.tag = ATAG_CORE;
	params->hdr.size = tag_size (tag_core);

	params->u.core.flags = 0;
	params->u.core.pagesize = 0;
	params->u.core.rootdev = 0;

	params = tag_next (params);
}


#ifdef CONFIG_SETUP_MEMORY_TAGS
static void setup_memory_tags (bd_t *bd)
{
	int i;

	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		params->hdr.tag = ATAG_MEM;
		params->hdr.size = tag_size (tag_mem32);

		params->u.mem.start = bd->bi_dram[i].start;
		params->u.mem.size = bd->bi_dram[i].size;

		params = tag_next (params);
	}
}
#endif /* CONFIG_SETUP_MEMORY_TAGS */


static void setup_commandline_tag (bd_t *bd, char *commandline)
{
	char *p;

	if (!commandline)
		return;

	/* eat leading white space */
	for (p = commandline; *p == ' '; p++);

	/* skip non-existent command lines so the kernel will still
	 * use its default command line.
	 */
	if (*p == '\0')
		return;

	params->hdr.tag = ATAG_CMDLINE;
	params->hdr.size =
		(sizeof (struct tag_header) + strlen (p) + 1 + 4) >> 2;

	strcpy (params->u.cmdline.cmdline, p);

	params = tag_next (params);
}


#ifdef CONFIG_INITRD_TAG
static void setup_initrd_tag (bd_t *bd, ulong initrd_start, ulong initrd_end)
{
	/* an ATAG_INITRD node tells the kernel where the compressed
	 * ramdisk can be found. ATAG_RDIMG is a better name, actually.
	 */
	params->hdr.tag = ATAG_INITRD2;
	params->hdr.size = tag_size (tag_initrd);

	params->u.initrd.start = initrd_start;
	params->u.initrd.size = initrd_end - initrd_start;

	params = tag_next (params);
}
#endif /* CONFIG_INITRD_TAG */

#ifdef CONFIG_SERIAL_TAG
void setup_serial_tag (struct tag **tmp)
{
	struct tag *params = *tmp;
	struct tag_serialnr serialnr;
	void get_board_serial(struct tag_serialnr *serialnr);

	get_board_serial(&serialnr);
	params->hdr.tag = ATAG_SERIAL;
	params->hdr.size = tag_size (tag_serialnr);
	params->u.serialnr.low = serialnr.low;
	params->u.serialnr.high= serialnr.high;
	params = tag_next (params);
	*tmp = params;
}
#endif

#ifdef CONFIG_REVISION_TAG
void setup_revision_tag(struct tag **in_params)
{
	u32 rev = 0;
	u32 get_board_rev(void);

	rev = get_board_rev();
	params->hdr.tag = ATAG_REVISION;
	params->hdr.size = tag_size (tag_revision);
	params->u.revision.rev = rev;
	params = tag_next (params);
}
#endif  /* CONFIG_REVISION_TAG */

static void setup_end_tag (bd_t *bd)
{
	params->hdr.tag = ATAG_NONE;
	params->hdr.size = 0;
}
#endif /* CONFIG_SETUP_MEMORY_TAGS || CONFIG_CMDLINE_TAG || CONFIG_INITRD_TAG */

static ulong get_sp(void)
{
	ulong ret;

	asm("mov %0, sp" : "=r"(ret) : );
	return ret;
}
