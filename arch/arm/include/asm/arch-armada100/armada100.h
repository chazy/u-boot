/*
 * (C) Copyright 2010
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 * Contributor: Mahavir Jain <mjain@marvell.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#ifndef _ASM_ARCH_ARMADA100_H
#define _ASM_ARCH_ARMADA100_H

#ifndef __ASSEMBLY__
#include <asm/types.h>
#include <asm/io.h>
#endif	/* __ASSEMBLY__ */

#if defined (CONFIG_ARMADA100)
#include <asm/arch/cpu.h>

/* Common APB clock register bit definitions */
#define APBC_APBCLK     (1<<0)  /* APB Bus Clock Enable */
#define APBC_FNCLK      (1<<1)  /* Functional Clock Enable */
#define APBC_RST        (1<<2)  /* Reset Generation */
/* Functional Clock Selection Mask */
#define APBC_FNCLKSEL(x)        (((x) & 0xf) << 4)

/* Fast Ethernet Controller Clock register definition */
#define FE_CLK_RST		0x1
#define FE_CLK_ENA		0x8

/* Register Base Addresses */
#define ARMD1_DRAM_BASE		0xB0000000
#define ARMD1_FEC_BASE		0xC0800000
#define ARMD1_TIMER_BASE	0xD4014000
#define ARMD1_APBC1_BASE	0xD4015000
#define ARMD1_APBC2_BASE	0xD4015800
#define ARMD1_UART1_BASE	0xD4017000
#define ARMD1_UART2_BASE	0xD4018000
#define ARMD1_GPIO_BASE		0xD4019000
#define ARMD1_SSP1_BASE		0xD401B000
#define ARMD1_SSP2_BASE		0xD401C000
#define ARMD1_MFPR_BASE		0xD401E000
#define ARMD1_SSP3_BASE		0xD401F000
#define ARMD1_SSP4_BASE		0xD4020000
#define ARMD1_SSP5_BASE		0xD4021000
#define ARMD1_UART3_BASE	0xD4026000
#define ARMD1_MPMU_BASE		0xD4050000
#define ARMD1_APMU_BASE		0xD4282800
#define ARMD1_CPU_BASE		0xD4282C00

/*
 * Main Power Management (MPMU) Registers
 * Refer Datasheet Appendix A.8
 */
struct armd1mpmu_registers {
	u8 pad0[0x08 - 0x00];
	u32 fccr;	/*0x0008*/
	u32 pocr;	/*0x000c*/
	u32 posr;	/*0x0010*/
	u32 succr;	/*0x0014*/
	u8 pad1[0x030 - 0x014 - 4];
	u32 gpcr;	/*0x0030*/
	u8 pad2[0x200 - 0x030 - 4];
	u32 wdtpcr;	/*0x0200*/
	u8 pad3[0x1000 - 0x200 - 4];
	u32 apcr;	/*0x1000*/
	u32 apsr;	/*0x1004*/
	u8 pad4[0x1020 - 0x1004 - 4];
	u32 aprr;	/*0x1020*/
	u32 acgr;	/*0x1024*/
	u32 arsr;	/*0x1028*/
};

/*
 * Application Subsystem Power Management
 * Refer Datasheet Appendix A.9
 */
struct armd1apmu_registers {
	u32 pcr;		/* 0x000 */
	u32 ccr;		/* 0x004 */
	u32 pad1;
	u32 ccsr;		/* 0x00C */
	u32 fc_timer;		/* 0x010 */
	u32 pad2;
	u32 ideal_cfg;		/* 0x018 */
	u8 pad3[0x04C - 0x018 - 4];
	u32 lcdcrc;		/* 0x04C */
	u32 cciccrc;		/* 0x050 */
	u32 sd1crc;		/* 0x054 */
	u32 sd2crc;		/* 0x058 */
	u32 usbcrc;		/* 0x05C */
	u32 nfccrc;		/* 0x060 */
	u32 dmacrc;		/* 0x064 */
	u32 pad4;
	u32 buscrc;		/* 0x06C */
	u8 pad5[0x07C - 0x06C - 4];
	u32 wake_clr;		/* 0x07C */
	u8 pad6[0x090 - 0x07C - 4];
	u32 core_status;	/* 0x090 */
	u32 rfsc;		/* 0x094 */
	u32 imr;		/* 0x098 */
	u32 irwc;		/* 0x09C */
	u32 isr;		/* 0x0A0 */
	u8 pad7[0x0B0 - 0x0A0 - 4];
	u32 mhst;		/* 0x0B0 */
	u32 msr;		/* 0x0B4 */
	u8 pad8[0x0C0 - 0x0B4 - 4];
	u32 msst;		/* 0x0C0 */
	u32 pllss;		/* 0x0C4 */
	u32 smb;		/* 0x0C8 */
	u32 gccrc;		/* 0x0CC */
	u8 pad9[0x0D4 - 0x0CC - 4];
	u32 smccrc;		/* 0x0D4 */
	u32 pad10;
	u32 xdcrc;		/* 0x0DC */
	u32 sd3crc;		/* 0x0E0 */
	u32 sd4crc;		/* 0x0E4 */
	u8 pad11[0x0F0 - 0x0E4 - 4];
	u32 cfcrc;		/* 0x0F0 */
	u32 mspcrc;		/* 0x0F4 */
	u32 cmucrc;		/* 0x0F8 */
	u32 fecrc;		/* 0x0FC */
	u32 pciecrc;		/* 0x100 */
	u32 epdcrc;		/* 0x104 */
};

/*
 * APB1 Clock Reset/Control Registers
 * Refer Datasheet Appendix A.10
 */
struct armd1apb1_registers {
	u32 uart1;	/*0x000*/
	u32 uart2;	/*0x004*/
	u32 gpio;	/*0x008*/
	u32 pwm1;	/*0x00c*/
	u32 pwm2;	/*0x010*/
	u32 pwm3;	/*0x014*/
	u32 pwm4;	/*0x018*/
	u8 pad0[0x028 - 0x018 - 4];
	u32 rtc;	/*0x028*/
	u32 twsi0;	/*0x02c*/
	u32 kpc;	/*0x030*/
	u32 timers;	/*0x034*/
	u8 pad1[0x03c - 0x034 - 4];
	u32 aib;	/*0x03c*/
	u32 sw_jtag;	/*0x040*/
	u32 timer1;	/*0x044*/
	u32 onewire;	/*0x048*/
	u8 pad2[0x050 - 0x048 - 4];
	u32 asfar;	/*0x050 AIB Secure First Access Reg*/
	u32 assar;	/*0x054 AIB Secure Second Access Reg*/
	u8 pad3[0x06c - 0x054 - 4];
	u32 twsi1;	/*0x06c*/
	u32 uart3;	/*0x070*/
	u8 pad4[0x07c - 0x070 - 4];
	u32 timer2;	/*0x07C*/
	u8 pad5[0x084 - 0x07c - 4];
	u32 ac97;	/*0x084*/
};

#endif /* CONFIG_ARMADA100 */
#endif /* _ASM_ARCH_ARMADA100_H */
