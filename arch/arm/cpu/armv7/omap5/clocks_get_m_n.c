/*
 * Program for finding M & N values for DPLLs
 * To be run on Host PC
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 * Sricharan R <r.sricharan@ti.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <stdlib.h>
#include <stdio.h>
typedef unsigned int u32;
#define MAX_N	127
int get_m_n_optimized(u32 target_freq_khz, u32 ref_freq_khz, u32 *m, u32 *n,
			u32 n_max_limit, u32 tolerance_khz)
{
	u32 min_freq = target_freq_khz - tolerance_khz;
	u32 max_freq = target_freq_khz;
	u32 freq, freq_old;
	*n = 1;
	while (1) {
		*m = min_freq / ref_freq_khz / 2 * (*n) ;
		freq_old = 0;
		while (1) {
			freq = ref_freq_khz * 2 * (*m) / (*n);
			if (abs(target_freq_khz - freq_old) <=
				abs(target_freq_khz - freq)) {
				freq = freq_old;
				(*m)--;
				break;
			}
			(*m)++;
			freq_old = freq;
		}
		if (freq >= min_freq && freq <= max_freq)
			break;
		(*n)++;
		if ((*n) > MAX_N + 1) {
			printf("ref %d m %d n %d target %d : ",
				ref_freq_khz, *m, *n, target_freq_khz);
			printf("can not find m & n - please consider"
				" increasing tolerance\n");
			return -1;
		}
	}
	(*n)--;
	printf("ref %d m %d n %d target %d locked %d\n",
		ref_freq_khz, *m, *n, target_freq_khz, freq);
	if ((ref_freq_khz / (*n + 1)) < 1000) {
		printf("\tREFCLK - CLKINP/(N+1) is less than 1 MHz - less than"
			" ideal, locking time will be high!\n");
	}
	return 0;
}

void main(void)
{
	u32 m, n;
	printf("\nMPU - 2000000\n");
	get_m_n_optimized(2000000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(2000000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(2000000, 16800, &m, &n, 128, 800);
	get_m_n_optimized(2000000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(2000000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(2000000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(2000000, 38400, &m, &n, 128, 0);

	printf("\nMPU - 1200000\n");
	get_m_n_optimized(1200000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1200000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1200000, 16800, &m, &n, 128, 800);
	get_m_n_optimized(1200000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1200000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1200000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1200000, 38400, &m, &n, 128, 0);

	printf("\nMPU - 1500000\n");
	get_m_n_optimized(1500000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1500000, 38400, &m, &n, 128, 0);

	printf("\nMPU - 1400000\n");
	get_m_n_optimized(1400000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1400000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1400000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1400000, 19200, &m, &n, 128, 200);
	get_m_n_optimized(1400000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1400000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1400000, 38400, &m, &n, 128, 200);

	printf("\nCore 1600000\n");
	get_m_n_optimized(1600000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1600000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1600000, 16800, &m, &n, 128, 200);
	get_m_n_optimized(1600000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1600000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1600000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1600000, 38400, &m, &n, 128, 0);

	printf("\nPER 1536000\n");
	get_m_n_optimized(1536000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1536000, 38400, &m, &n, 128, 0);

	printf("\nIVA 1862000\n");
	get_m_n_optimized(1862000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1862000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1862000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1862000, 19200, &m, &n, 128, 900);
	get_m_n_optimized(1862000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1862000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1862000, 38400, &m, &n, 128, 800);

	printf("\nABE 196608 sys clk\n");
	get_m_n_optimized(196608, 12000, &m, &n, 128, 700);
	get_m_n_optimized(196608, 13000, &m, &n, 128, 200);
	get_m_n_optimized(196608, 16800, &m, &n, 128, 700);
	get_m_n_optimized(196608, 19200, &m, &n, 128, 400);
	get_m_n_optimized(196608, 26000, &m, &n, 128, 200);
	get_m_n_optimized(196608, 27000, &m, &n, 128, 900);
	get_m_n_optimized(196608, 38400, &m, &n, 128, 0);

	printf("\nABE 196608 32K\n");
	get_m_n_optimized(196608000/4, 32768, &m, &n, 128, 0);

	printf("\nUSB 1920000\n");
	get_m_n_optimized(1920000, 12000, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 27000, &m, &n, 128, 0);
	get_m_n_optimized(1920000, 38400, &m, &n, 128, 0);

	printf("\nCore ES1 1523712\n");
	get_m_n_optimized(1524000, 12000, &m, &n, 128, 100);
	get_m_n_optimized(1524000, 13000, &m, &n, 128, 0);
	get_m_n_optimized(1524000, 16800, &m, &n, 128, 0);
	get_m_n_optimized(1524000, 19200, &m, &n, 128, 0);
	get_m_n_optimized(1524000, 26000, &m, &n, 128, 0);
	get_m_n_optimized(1524000, 27000, &m, &n, 128, 0);

	/* exact recommendation for SDPs */
	get_m_n_optimized(1523712, 38400, &m, &n, 128, 0);

}
