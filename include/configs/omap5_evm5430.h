/*
 * (C) Copyright 2010
 * Texas Instruments Incorporated.
 * Sricharan R	  <r.sricharan@ti.com>
 *
 * Derived from OMAP4 done by:
 *	Aneesh V <aneesh@ti.com>
 *
 * Configuration settings for the TI EVM5430 board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1	/* in a TI OMAP core */
#define CONFIG_OMAP54XX		1	/* which is a 54XX */
#define CONFIG_OMAP5430		1	/* which is in a 5430 */
#define CONFIG_5430EVM		1	/* working with EVM */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_MACH_OMAP_5430EVM 1

/* Get CPU defs */
#include <asm/arch/cpu.h>
#include <asm/arch/omap.h>

/* Display CPU and Board Info */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1

/* Clock Defines */
#define V_OSCK			19200000	/* Clock output from T2 */
#define V_SCLK                   V_OSCK
#define CONFIG_UBOOT_CLOCKS_ENABLE_ALL	1	/* Enable all clocks */
#define CONFIG_UBOOT_ENABLE_PADS_ALL	1	/* Enable all PADS for now */

#undef CONFIG_USE_IRQ				/* no support for IRQs */
#define CONFIG_MISC_INIT_R
#define CONFIG_OF_LIBFDT		1

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * Size of malloc() pool
 * Total Size Environment - 128k
 * Malloc - add 256k
 */
#define CONFIG_ENV_SIZE			(128 << 10)
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (256 << 10))
/* Vector Base */
#define CONFIG_SYS_CA9_VECTOR_BASE	SRAM_ROM_VECT_BASE

/*
 * Hardware drivers
 */

/*
 * serial port - NS16550 compatible
 */
#define V_NS16550_CLK			48000000

#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		V_NS16550_CLK
#define CONFIG_CONS_INDEX		3
#define CONFIG_SYS_NS16550_COM3		UART3_BASE

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
					115200}

/* OPP SETTING*/
#define OPP_NOM				1

/* I2C  */
#define CONFIG_HARD_I2C			1
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS		0
#define CONFIG_SYS_I2C_BUS_SELECT	1
#define CONFIG_DRIVER_OMAP34XX_I2C	1
#define CONFIG_I2C_MULTI_BUS		1

/* TWL6035 */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_TWL6035_POWER		1
#endif

/* MMC */
#define CONFIG_GENERIC_MMC		1
#define CONFIG_MMC			1
#define CONFIG_OMAP_HSMMC		1
#define CONFIG_SYS_MMC_SET_DEV		1

#define CONFIG_DOS_PARTITION		1
/* ENV related config options */
#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SLOT2: eMMC(1) */
#define CONFIG_ENV_OFFSET		0xE0000
#define CONFIG_CMD_SAVEENV

#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1
#endif
/* Flash */
#define CONFIG_SYS_NO_FLASH	1

/* USB UHH support options */
#define CONFIG_CMD_USB
#define CONFIG_USB_HOST
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_OMAP
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS 3

#define CONFIG_OMAP_EHCI_PHY1_RESET_GPIO 172
#define CONFIG_OMAP_EHCI_PHY2_RESET_GPIO 173

/* Enabled commands */
#define CONFIG_NET_MULTI
#define CONFIG_CMD_DHCP		/* DHCP Support			*/
#define CONFIG_CMD_NET		/* bootp, tftpboot, rarpboot	*/
#define CONFIG_CMD_NFS		/* NFS support			*/

/* USB Networking options */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_SMSC95XX

/* commands to include */
#include <config_cmd_default.h>

/* Enabled commands */
#define CONFIG_CMD_EXT2		/* EXT2 Support                 */
#define CONFIG_CMD_FAT		/* FAT support                  */
#define CONFIG_CMD_I2C		/* I2C serial bus support	*/
#define CONFIG_CMD_MMC		/* MMC support                  */
#define CONFIG_CMD_SAVEENV

/* Disabled commands */
#undef CONFIG_CMD_FPGA		/* FPGA configuration Support   */
#undef CONFIG_CMD_IMLS		/* List all found images        */

/*
 * Environment setup
 */
#define CONFIG_BOOTDELAY	3

#define CONFIG_ENV_OVERWRITE

/* This is the default case */
#ifndef CONFIG_OMAP_ANDROID
	#define CONFIG_EXTRA_ENV_SETTINGS \
		"usbethaddr=9e:77:39:1b:c4:2d\0" \
		"loadaddr=0x82000000\0" \
		"console=ttyO2,115200n8\0" \
		"usbtty=cdc_acm\0" \
		"vram=16M\0" \
		"mmcdev=0\0" \
		"mmcroot=/dev/mmcblk0p2 rw\0" \
		"mmcrootfstype=ext3 rootwait\0" \
		"mmcargs=setenv bootargs console=${console} " \
			"vram=${vram} " \
			"root=${mmcroot} " \
			"rootfstype=${mmcrootfstype}\0" \
		"loadbootscript=fatload mmc ${mmcdev} ${loadaddr} boot.scr\0" \
		"bootscript=echo Running bootscript from mmc${mmcdev} ...; " \
			"source ${loadaddr}\0" \
		"loaduimage=fatload mmc ${mmcdev} ${loadaddr} uImage\0" \
		"mmcboot=echo Booting from mmc${mmcdev} ...; " \
			"run mmcargs; " \
			"bootm ${loadaddr}\0" \

	#define CONFIG_BOOTCOMMAND \
		"if mmc rescan ${mmcdev}; then " \
			"if run loadbootscript; then " \
				"run bootscript; " \
			"else " \
				"if run loaduimage; then " \
					"run mmcboot; " \
				"fi; " \
			"fi; " \
		"fi "

#else

/* This is used when we build for ANDROID i.e. omap5_evm5430_android */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"if bootdevice; then " \
		"console=ttyO2,115200n8\0" \
		"vram=16M\0" \
		"emmcargs=setenv bootargs console=${console} " \
			"mem=456M@0x80000000 " \
			"mem=512M@0xA0000000 " \
			"omapfb.vram=0:4M " \
			"androidboot.console=ttyO2 " \
			"init=/init " \
			"vram=${vram}\0" \
		"emmcboot=echo Booting from emmc ...; " \
				"run emmcargs; " \
				"booti mmc1\0" \
	"else " \
		"usbethaddr=9e:77:39:1b:c4:2d\0" \
		"loadaddr=0x82000000\0" \
		"console=ttyO2,115200n8\0" \
		"usbtty=cdc_acm\0" \
		"vram=16M\0" \
		"mmcdev=0\0" \
		"mmcroot=/dev/mmcblk0p2 rw\0" \
		"mmcrootfstype=ext3 rootwait\0" \
		"mmcargs=setenv bootargs console=${console} " \
			"vram=${vram} " \
			"root=${mmcroot} " \
			"rootfstype=${mmcrootfstype}\0" \
		"loadbootscript=fatload mmc ${mmcdev} ${loadaddr} boot.scr\0" \
		"bootscript=echo Running bootscript from mmc${mmcdev} ...; " \
			"source ${loadaddr}\0" \
		"loaduimage=fatload mmc ${mmcdev} ${loadaddr} uImage\0" \
		"mmcboot=echo Booting from mmc${mmcdev} ...; " \
			"run mmcargs; " \
			"bootm ${loadaddr}\0" \
	"fi"

#define CONFIG_BOOTCOMMAND \
	"if bootdevice; then " \
		"run emmcboot; " \
	"else " \
		"if mmc rescan ${mmcdev}; then " \
			"if run loadbootscript; then " \
				"run bootscript; " \
			"else " \
				"if run loaduimage; then " \
					"run mmcboot; " \
				"fi; " \
			"fi; " \
		"fi; " \
	"fi"
#endif

#define CONFIG_AUTO_COMPLETE		1

/*
 * Miscellaneous configurable options
 */

#define CONFIG_SYS_LONGHELP	/* undef to save memory */
#define CONFIG_SYS_HUSH_PARSER	/* use "hush" command parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_PROMPT		"OMAP5430 EVM # "
#define CONFIG_SYS_CBSIZE		256
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		16
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		(CONFIG_SYS_CBSIZE)

/*
 * memtest setup
 */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + (32 << 20))

/* Default load address */
#define CONFIG_SYS_LOAD_ADDR		0x80000000

/* Use General purpose timer 1 */
#define CONFIG_SYS_TIMERBASE		GPT2_BASE
#define CONFIG_SYS_PTV			2	/* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/*
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 << 10)	/* Regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4 << 10)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4 << 10)	/* FIQ stack */
#endif

/*
 * SDRAM Memory Map
 * Even though we use two CS all the memory
 * is mapped to one contiguous block
 */
#define CONFIG_NR_DRAM_BANKS	1

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_INIT_RAM_ADDR	0x4030D800
#define CONFIG_SYS_INIT_RAM_SIZE	0x800
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + \
					 CONFIG_SYS_INIT_RAM_SIZE - \
					 GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS

/* Defines for SDRAM init */
#ifndef CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS
#define CONFIG_SYS_AUTOMATIC_SDRAM_DETECTION
#define CONFIG_SYS_DEFAULT_LPDDR2_TIMINGS
#endif

/* Defines for SPL */
#define CONFIG_SPL
#define CONFIG_SPL_TEXT_BASE		0x40300350
#define CONFIG_SPL_MAX_SIZE		0x20000	/* 128K */
#define CONFIG_SPL_STACK		LOW_LEVEL_SRAM_STACK

#define CONFIG_SPL_BSS_START_ADDR	0x80000000
#define CONFIG_SPL_BSS_MAX_SIZE		0x80000		/* 512 KB */

#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x300 /* address 0x60000 */
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	0x200 /* 256 KB */
#define CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION	1
#define CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME	"u-boot.img"

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBDISK_SUPPORT
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_FAT_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_LDSCRIPT "arch/arm/cpu/armv7/omap-common/u-boot-spl.lds"
#define CONFIG_BOOT_SCRIPT_FILE "boot.scr.omap.txt"

/*
 * 1MB into the SDRAM to allow for SPL's bss at the beginning of SDRAM
 * 64 bytes before this address should be set aside for u-boot.img's
 * header. That is 0x800FFFC0--0x80100000 should not be used for any
 * other needs.
 */
#define CONFIG_SYS_TEXT_BASE		0x80100000
#define CONFIG_SYS_SPL_MALLOC_START     0x80200000
#define CONFIG_SYS_SPL_MALLOC_SIZE      0x100000        /* 1 MB */

#endif /* __CONFIG_H */
