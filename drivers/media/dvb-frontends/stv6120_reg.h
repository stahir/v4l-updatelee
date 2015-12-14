/*
	STV6110(A) Silicon tuner driver

	Copyright (C) Manu Abraham <abraham.manu@gmail.com>

	Copyright (C) ST Microelectronics

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __STV6120_REG_H
#define __STV6120_REG_H

/*CTRL1*/
#define RSTV6120_CTRL1  0x0000
#define FSTV6120_K  0x000000f8
#define FSTV6120_RDIV  0x00000004
#define FSTV6120_OSHP  0x00000002
#define FSTV6120_MCLKDIV  0x00000001

/*CTRL2*/
#define RSTV6120_CTRL2  0x0001
#define FSTV6120_DCLOOPOFF_1  0x00010080
#define FSTV6120_SDOFF_1  0x00010040
#define FSTV6120_SYN_1  0x00010020
#define FSTV6120_REFOUTSEL_1  0x00010010
#define FSTV6120_BBGAIN_1  0x0001000f

/*CTRL3*/
#define RSTV6120_CTRL3  0x0002
#define FSTV6120_NDIV_1_LSB  0x000200ff

/*CTRL4*/
#define RSTV6120_CTRL4  0x0003
#define FSTV6120_F_1_L  0x000300fe
#define FSTV6120_NDIV_1_MSB  0x00030001

/*CTRL5*/
#define RSTV6120_CTRL5  0x0004
#define FSTV6120_F_1_M  0x000400ff

/*CTRL6*/
#define RSTV6120_CTRL6  0x0005
#define FSTV6120_ICP_1  0x00050070
#define FSTV6120_VCOILOW_1  0x00050008
#define FSTV6120_F_1_H  0x00050007

/*CTRL7*/
#define RSTV6120_CTRL7  0x0006
#define FSTV6120_RCCLFOFF_1  0x00060080
#define FSTV6120_PDIV_1  0x00060060
#define FSTV6120_CF_1  0x0006001f

/*CTRL8*/
#define RSTV6120_CTRL8  0x0007
#define FSTV6120_TCAL  0x000700c0
#define FSTV6120_CALTIME_1  0x00070020
#define FSTV6120_CFHF_1  0x0007001f

/*STAT1*/
#define RSTV6120_STAT1  0x0008
#define FSTV6120_CALVCOSTRT_1  0x00080004
#define FSTV6120_CALRCSTRT_1  0x00080002
#define FSTV6120_LOCK_1  0x00080001

/*CTRL9*/
#define RSTV6120_CTRL9  0x0009
#define FSTV6120_RFSEL_2  0x0009000c
#define FSTV6120_RFSEL_1  0x00090003

/*CTRL10*/
#define RSTV6120_CTRL10  0x000a
#define FSTV6120_LNADON  0x000a0020
#define FSTV6120_LNACON  0x000a0010
#define FSTV6120_LNABON  0x000a0008
#define FSTV6120_LNAAON  0x000a0004
#define FSTV6120_PATHON_2  0x000a0002
#define FSTV6120_PATHON_1  0x000a0001

/*CTRL11*/
#define RSTV6120_CTRL11  0x000b
#define FSTV6120_DCLOOPOFF_2  0x000b0080
#define FSTV6120_SDOFF_2  0x000b0040
#define FSTV6120_SYN_2  0x000b0020
#define FSTV6120_REFOUTSEL_2  0x000b0010
#define FSTV6120_BBGAIN_2  0x000b000f

/*CTRL12*/
#define RSTV6120_CTRL12  0x000c
#define FSTV6120_NDIV_2_LSB  0x000c00ff

/*CTRL13*/
#define RSTV6120_CTRL13  0x000d
#define FSTV6120_F_2_L  0x000d00fe
#define FSTV6120_NDIV_2_MSB  0x000d0001

/*CTRL14*/
#define RSTV6120_CTRL14  0x000e
#define FSTV6120_F_2_M  0x000e00ff

/*CTRL15*/
#define RSTV6120_CTRL15  0x000f
#define FSTV6120_ICP_2  0x000f0070
#define FSTV6120_VCOILOW_2  0x000f0008
#define FSTV6120_F_2_H  0x000f0007

/*CTRL16*/
#define RSTV6120_CTRL16  0x0010
#define FSTV6120_RCCLFOFF_2  0x00100080
#define FSTV6120_PDIV_2  0x00100060
#define FSTV6120_CF_2  0x0010001f

/*CTRL17*/
#define RSTV6120_CTRL17  0x0011
#define FSTV6120_CALTIME_2  0x00110020
#define FSTV6120_CFHF_2  0x0011001f

/*STAT2*/
#define RSTV6120_STAT2  0x0012
#define FSTV6120_CALVCOSTRT_2  0x00120004
#define FSTV6120_CALRCSTRT_2  0x00120002
#define FSTV6120_LOCK_2  0x00120001

/*CTRL18*/
#define RSTV6120_CTRL18  0x0013
#define FSTV6120_TEST1  0x001300ff

/*CTRL19*/
#define RSTV6120_CTRL19  0x0014
#define FSTV6120_TEST2  0x001400ff

/*CTRL20*/
#define RSTV6120_CTRL20  0x0015
#define FSTV6120_TEST3  0x001500ff

/*CTRL21*/
#define RSTV6120_CTRL21  0x0016
#define FSTV6120_TEST4  0x001600ff

/*CTRL22*/
#define RSTV6120_CTRL22  0x0017
#define FSTV6120_TEST5  0x001700ff

/*CTRL23*/
#define RSTV6120_CTRL23  0x0018
#define FSTV6120_TEST6  0x001800ff

#endif /* __STV6120_REG_H */
