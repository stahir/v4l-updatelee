/*
 * Driver for the ST STV0910 DVB-S/S2 demodulator.
 *
 * Copyright (C) 2014-2015 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *                         developed for Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <asm/div64.h>

#include "dvb_frontend.h"
#include "stv6110x.h"
#include "stv0910.h"
#include "stv0910.h"
#include "stv0910_regs.h"

#define TUNING_DELAY    200
#define BER_SRC_S    0x20
#define BER_SRC_S2   0x20

LIST_HEAD(stvlist);

enum FE_STV0910_frame_len { FE_LONGFRAME, FE_SHORTFRAME };

enum FE_STV0910_dmdstate {
	FE_SEARCHING,
	FE_DVBS2_PLH,
	FE_DVB_S2,
	FE_DVB_S
};

enum DVBS2_modcod {
	DVBS2_DUMMY_PLF, DVBS2_QPSK_1_4, DVBS2_QPSK_1_3, DVBS2_QPSK_2_5,
	DVBS2_QPSK_1_2, DVBS2_QPSK_3_5, DVBS2_QPSK_2_3,	DVBS2_QPSK_3_4,
	DVBS2_QPSK_4_5,	DVBS2_QPSK_5_6,	DVBS2_QPSK_8_9,	DVBS2_QPSK_9_10,
	DVBS2_8PSK_3_5,	DVBS2_8PSK_2_3,	DVBS2_8PSK_3_4,	DVBS2_8PSK_5_6,
	DVBS2_8PSK_8_9,	DVBS2_8PSK_9_10, DVBS2_16APSK_2_3, DVBS2_16APSK_3_4,
	DVBS2_16APSK_4_5, DVBS2_16APSK_5_6, DVBS2_16APSK_8_9, DVBS2_16APSK_9_10,
	DVBS2_32APSK_3_4, DVBS2_32APSK_4_5, DVBS2_32APSK_5_6, DVBS2_32APSK_8_9,
	DVBS2_32APSK_9_10
};

enum FE_STV0910_modcod {
	FE_DUMMY_PLF,
	FE_QPSK_14,
	FE_QPSK_13,
	FE_QPSK_25,
	FE_QPSK_12,
	FE_QPSK_35,
	FE_QPSK_23,
	FE_QPSK_34,
	FE_QPSK_45,
	FE_QPSK_56,
	FE_QPSK_89,
	FE_QPSK_910,
	FE_8PSK_35,
	FE_8PSK_23,
	FE_8PSK_34,
	FE_8PSK_56,
	FE_8PSK_89,
	FE_8PSK_910,
	FE_16APSK_23,
	FE_16APSK_34,
	FE_16APSK_45,
	FE_16APSK_56,
	FE_16APSK_89,
	FE_16APSK_910,
	FE_32APSK_34,
	FE_32APSK_45,
	FE_32APSK_56,
	FE_32APSK_89,
	FE_32APSK_910
};

static inline u32 MulDiv32(u32 a, u32 b, u32 c)
{
	u64 tmp64;

	tmp64 = (u64)a * (u64)b;
	do_div(tmp64, c);

	return (u32) tmp64;
}

struct stv_base {
	struct list_head     stvlist;

	u8                   adr;
	struct i2c_adapter  *i2c;
	struct mutex         i2c_lock;
	struct mutex         reg_lock;
	int                  count;

	u32                  extclk;
	u32                  mclk;
};

struct stv0910_state {
	struct stv_base     *base;
	struct dvb_frontend  frontend;
	const struct stv0910_cfg *config;
	int                  nr;
	u8                   i2crpt;
	u8                   tscfgh;
	u8                   tsspeed;

	enum stv0910_algo	algo;

	u32   LastBERNumerator;
	u32   LastBERDenominator;
	u8    BERScale;
};

struct SInitTable {
	u16  Address;
	u8   Data;
};

/* Max transfer size done by I2C transfer functions */
#define MAX_XFER_SIZE  64

static void extract_mask_pos(u32 label, u8 *mask, u8 *pos)
{
	u8 position = 0, i = 0;

	(*mask) = label & 0xff;

	while ((position == 0) && (i < 8)) {
		position = ((*mask) >> i) & 0x01;
		i++;
	}

	(*pos) = (i - 1);
}

static int stv0910_read_regs(struct stv0910_state *state, u16 reg, u8 *data, u8 len)
{
	int ret;
	u8 b0[] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[] = {
		{ .addr = state->base->adr, .flags = 0,        .buf = b0,   .len = 2 },
		{ .addr = state->base->adr, .flags = I2C_M_RD, .buf = data, .len = len }
	};

	ret = i2c_transfer(state->base->i2c, msg, 2);
	if (ret != 2) {
		printk("I2C Error\n");
		return -EREMOTEIO;
	}
	return 0;
}

static u8 stv0910_read_reg(struct stv0910_state *state, u16 reg)
{
	u8 data = 0x00;
	stv0910_read_regs(state, reg, &data, 1);
	return data;
}

static u8 stv0910_read_field(struct stv0910_state *state, u32 label)
{
	u8 mask, pos, data;

	extract_mask_pos(label, &mask, &pos);

	data = stv0910_read_reg(state, label >> 16);
	data = (data & mask) >> pos;

	return data;
}

static int stv0910_write_regs(struct stv0910_state *state, u16 reg, u8 *data, u8 len)
{
	int ret;
	u8 buf[MAX_XFER_SIZE];

	struct i2c_msg msg = {
		.addr  = state->base->adr,
		.flags = 0,
		.buf   = buf,
		.len   = len + 2
	};

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	memcpy(&buf[2], data, len);

	ret = i2c_transfer(state->base->i2c, &msg, 1);
	if (ret != 1) {
		printk("I2C Error\n");
		return -EREMOTEIO;
	}

	return 0;
}

static int stv0910_write_reg(struct stv0910_state *state, u16 reg, u8 data)
{
	return stv0910_write_regs(state, reg, &data, 1);
}

static int stv0910_write_field(struct stv0910_state *state, u32 label, u8 data)
{
	u8 reg, mask, pos;

	reg = stv0910_read_reg(state, (label >> 16) & 0xffff);
	extract_mask_pos(label, &mask, &pos);

	data = mask & (data << pos);
	reg = (reg & (~mask)) | data;

	return stv0910_write_reg(state, (label >> 16) & 0xffff, reg);
}

/*********************************************************************
Tracking carrier loop carrier QPSK 1/4 to 8PSK 9/10 long Frame
*********************************************************************/
static u8 S2CarLoop[] =	{
	/* Modcod  2MPon 2MPoff 5MPon 5MPoff 10MPon 10MPoff
	   20MPon 20MPoff 30MPon 30MPoff*/
	/* FE_QPSK_14  */
	0x0C,  0x3C,  0x0B,  0x3C,  0x2A,  0x2C,  0x2A,  0x1C,  0x3A,  0x3B,
	/* FE_QPSK_13  */
	0x0C,  0x3C,  0x0B,  0x3C,  0x2A,  0x2C,  0x3A,  0x0C,  0x3A,  0x2B,
	/* FE_QPSK_25  */
	0x1C,  0x3C,  0x1B,  0x3C,  0x3A,  0x1C,  0x3A,  0x3B,  0x3A,  0x2B,
	/* FE_QPSK_12  */
	0x0C,  0x1C,  0x2B,  0x1C,  0x0B,  0x2C,  0x0B,  0x0C,  0x2A,  0x2B,
	/* FE_QPSK_35  */
	0x1C,  0x1C,  0x2B,  0x1C,  0x0B,  0x2C,  0x0B,  0x0C,  0x2A,  0x2B,
	/* FE_QPSK_23  */
	0x2C,  0x2C,  0x2B,  0x1C,  0x0B,  0x2C,  0x0B,  0x0C,  0x2A,  0x2B,
	/* FE_QPSK_34  */
	0x3C,  0x2C,  0x3B,  0x2C,  0x1B,  0x1C,  0x1B,  0x3B,  0x3A,  0x1B,
	/* FE_QPSK_45  */
	0x0D,  0x3C,  0x3B,  0x2C,  0x1B,  0x1C,  0x1B,  0x3B,  0x3A,  0x1B,
	/* FE_QPSK_56  */
	0x1D,  0x3C,  0x0C,  0x2C,  0x2B,  0x1C,  0x1B,  0x3B,  0x0B,  0x1B,
	/* FE_QPSK_89  */
	0x3D,  0x0D,  0x0C,  0x2C,  0x2B,  0x0C,  0x2B,  0x2B,  0x0B,  0x0B,
	/* FE_QPSK_910 */
	0x1E,  0x0D,  0x1C,  0x2C,  0x3B,  0x0C,  0x2B,  0x2B,  0x1B,  0x0B,
	/* FE_8PSK_35  */
	0x28,  0x09,  0x28,  0x09,  0x28,  0x09,  0x28,  0x08,  0x28,  0x27,
	/* FE_8PSK_23  */
	0x19,  0x29,  0x19,  0x29,  0x19,  0x29,  0x38,  0x19,  0x28,  0x09,
	/* FE_8PSK_34  */
	0x1A,  0x0B,  0x1A,  0x3A,  0x0A,  0x2A,  0x39,  0x2A,  0x39,  0x1A,
	/* FE_8PSK_56  */
	0x2B,  0x2B,  0x1B,  0x1B,  0x0B,  0x1B,  0x1A,  0x0B,  0x1A,  0x1A,
	/* FE_8PSK_89  */
	0x0C,  0x0C,  0x3B,  0x3B,  0x1B,  0x1B,  0x2A,  0x0B,  0x2A,  0x2A,
	/* FE_8PSK_910 */
	0x0C,  0x1C,  0x0C,  0x3B,  0x2B,  0x1B,  0x3A,  0x0B,  0x2A,  0x2A,

	/**********************************************************************
	Tracking carrier loop carrier 16APSK 2/3 to 32APSK 9/10 long Frame
	**********************************************************************/
	/*Modcod 2MPon  2MPoff 5MPon 5MPoff 10MPon 10MPoff 20MPon
	  20MPoff 30MPon 30MPoff*/
	/* FE_16APSK_23  */
	0x0A,  0x0A,  0x0A,  0x0A,  0x1A,  0x0A,  0x39,  0x0A,  0x29,  0x0A,
	/* FE_16APSK_34  */
	0x0A,  0x0A,  0x0A,  0x0A,  0x0B,  0x0A,  0x2A,  0x0A,  0x1A,  0x0A,
	/* FE_16APSK_45  */
	0x0A,  0x0A,  0x0A,  0x0A,  0x1B,  0x0A,  0x3A,  0x0A,  0x2A,  0x0A,
	/* FE_16APSK_56  */
	0x0A,  0x0A,  0x0A,  0x0A,  0x1B,  0x0A,  0x3A,  0x0A,  0x2A,  0x0A,
	/* FE_16APSK_89  */
	0x0A,  0x0A,  0x0A,  0x0A,  0x2B,  0x0A,  0x0B,  0x0A,  0x3A,  0x0A,
	/* FE_16APSK_910 */
	0x0A,  0x0A,  0x0A,  0x0A,  0x2B,  0x0A,  0x0B,  0x0A,  0x3A,  0x0A,
	/* FE_32APSK_34  */
	0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,
	/* FE_32APSK_45  */
	0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,
	/* FE_32APSK_56  */
	0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,
	/* FE_32APSK_89  */
	0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,
	/* FE_32APSK_910 */
	0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,  0x09,
};

struct stv0910_table {
	s16  ret;
	u16  val;
};

struct stv0910_table stv0910_S1_SNR_lookup[] = {
	{  510,    425  },  /*C/N=51.0dB*/
	{  500,    426  },  /*C/N=50.0dB*/
	{  450,    430  },  /*C/N=45.0dB*/
	{  400,    445  },  /*C/N=40.0dB*/
	{  350,    490  },  /*C/N=33.0dB*/
	{  330,    526  },  /*C/N=33.0dB*/
	{  320,    550  },  /*C/N=32.0dB*/
	{  310,    579  },  /*C/N=31.0dB*/
	{  300,    613  },  /*C/N=30.0dB*/
	{  290,    653  },  /*C/N=29.0dB*/
	{  280,    702  },  /*C/N=28.0dB*/
	{  270,    758  },  /*C/N=27.0dB*/
	{  260,    826  },  /*C/N=26.0dB*/
	{  250,    900  },  /*C/N=25.0dB*/
	{  240,    992  },  /*C/N=24.0dB*/
	{  230,   1095  },  /*C/N=23.0dB*/
	{  220,   1213  },  /*C/N=22.0dB*/
	{  210,   1347  },  /*C/N=21.0dB*/
	{  200,   1499  },  /*C/N=20.0dB*/
	{  190,   1670  },  /*C/N=19.0dB*/
	{  180,   1862  },  /*C/N=18.0dB*/
	{  170,   2077  },  /*C/N=17.0dB*/
	{  160,   2318  },  /*C/N=16.0dB*/
	{  150,   2587  },  /*C/N=15.0dB*/
	{  145,   2733  },  /*C/N=14.5dB*/
	{  140,   2889  },  /*C/N=14.0dB*/
	{  135,   3052  },  /*C/N=13.5dB*/
	{  130,   3225  },  /*C/N=13.0dB*/
	{  125,   3406  },  /*C/N=12.5dB*/
	{  120,   3598  },  /*C/N=12.0dB*/
	{  115,   3800  },  /*C/N=11.5dB*/
	{  110,   4013  },  /*C/N=11.0dB*/
	{  105,   4236  },  /*C/N=10.5dB*/
	{  100,   4467  },  /*C/N=10.0dB*/
	{  95,    4709  },  /*C/N=9.5dB*/
	{  90,    4959  },  /*C/N=9.0dB*/
	{  85,    5224  },  /*C/N=8.5dB*/
	{  80,    5492  },  /*C/N=8.0dB*/
	{  75,    5768  },  /*C/N=7.5dB*/
	{  70,    6048  },  /*C/N=7.0dB*/
	{  65,    6330  },  /*C/N=6.5dB*/
	{  60,    6576  },  /*C/N=6.0dB*/
	{  55,    6861  },  /*C/N=5.5dB*/
	{  50,    7136  },  /*C/N=5.0dB*/
	{  45,    7405  },  /*C/N=4.5dB*/
	{  40,    7666  },  /*C/N=4.0dB*/
	{  35,    7908  },  /*C/N=3.5dB*/
	{  30,    8146  },  /*C/N=3.0dB*/
	{  25,    8366  },  /*C/N=2.5dB*/
	{  20,    8566  },  /*C/N=2.0dB*/
	{  15,    8780  },  /*C/N=1.5dB*/
	{  10,    8950  },  /*C/N=1.0dB*/
	{  05,    9105  },  /*C/N=0.5dB*/
	{   0,    9242  },  /*C/N=  0dB*/
};

struct stv0910_table stv0910_S2_SNR_lookup[] = {
	{  510,    463  },  /*C/N=51.0dB*/
	{  500,    464  },  /*C/N=50.0dB*/
	{  450,    466  },  /*C/N=45.0dB*/
	{  400,    480  },  /*C/N=40.0dB*/
	{  350,    517  },  /*C/N=35.0dB*/
	{  330,    550  },  /*C/N=33.0dB*/
	{  320,    575  },  /*C/N=32.0dB*/
	{  310,    602  },  /*C/N=31.0dB*/
	{  300,    635  },  /*C/N=30.0dB*/
	{  290,    671  },  /*C/N=29.0dB*/
	{  280,    718  },  /*C/N=28.0dB*/
	{  270,    772  },  /*C/N=27.0dB*/
	{  260,    836  },  /*C/N=26.0dB*/
	{  250,    910  },  /*C/N=25.0dB*/
	{  240,   1000  },  /*C/N=24.0dB*/
	{  230,   1100  },  /*C/N=23.0dB*/
	{  220,   1212  },  /*C/N=22.0dB*/
	{  210,   1340  },  /*C/N=21.0dB*/
	{  200,   1485  },  /*C/N=20.0dB*/
	{  190,   1650  },  /*C/N=19.0dB*/
	{  180,   1825  },  /*C/N=18.0dB*/
	{  170,   2035  },  /*C/N=17.0dB*/
	{  160,   2270  },  /*C/N=16.0dB*/
	{  150,   2535  },  /*C/N=15.0dB*/
	{  145,   2670  },  /*C/N=14.5dB*/
	{  140,   2820  },  /*C/N=14.0dB*/
	{  135,   2980  },  /*C/N=13.5dB*/
	{  130,   3140  },  /*C/N=13.0dB*/
	{  125,   3315  },  /*C/N=12.5dB*/
	{  120,   3570  },  /*C/N=12.0dB*/
	{  115,   3765  },  /*C/N=11.5dB*/
	{  110,   3980  },  /*C/N=11.0dB*/
	{  105,   4210  },  /*C/N=10.5dB*/
	{  100,   4425  },  /*C/N=10.0dB*/
	{   95,   4680  },  /*C/N= 9.5dB*/
	{   90,   4930  },  /*C/N= 9.0dB*/
	{   85,   5200  },  /*C/N= 8.5dB*/
	{   80,   5480  },  /*C/N= 8.0dB*/
	{   75,   5760  },  /*C/N= 7.5dB*/
	{   70,   6060  },  /*C/N= 7.0dB*/
	{   65,   6320  },  /*C/N= 6.5dB*/
	{   60,   6720  },  /*C/N= 6.0dB*/
	{   55,   7080  },  /*C/N= 5.5dB*/
	{   50,   7430  },  /*C/N= 5.0dB*/
	{   45,   7800  },  /*C/N= 4.5dB*/
	{   40,   8180  },  /*C/N= 4.0dB*/
	{   35,   8575  },  /*C/N= 3.5dB*/
	{   30,   8970  },  /*C/N= 3.0dB*/
	{   25,   9390  },  /*C/N= 2.5dB*/
	{   20,   9790  },  /*C/N= 2.0dB*/
	{   15,  10210  },  /*C/N= 1.5dB*/
	{   10,  10630  },  /*C/N= 1.0dB*/
	{   05,  11080  },  /*C/N= 0.5dB*/
	{    0,  11520  },  /*C/N=   0dB*/
	{  -05,  11900  },  /*C/N=-0.5dB*/
	{  -10,  12345  },  /*C/N=-1.0dB*/
	{  -15,  12760  },  /*C/N=-1.5dB*/
	{  -20,  13150  },  /*C/N=-2.0dB*/
	{  -25,  13580  },  /*C/N=-2.5dB*/
	{  -30,  13950  },  /*C/N=-3.0dB*/
};

/* RF level C/N lookup table */
struct stv0910_table stv0910_dbm_lookup[] = {
	{ -70, 0x07aa }, /* -70dBm */
	{ -65, 0x114f }, /* -65dBm */
	{ -60, 0x210d }, /* -60dBm */
	{ -55, 0x2d11 }, /* -55dBm */
	{ -50, 0x3a14 }, /* -50dBm */
	{ -45, 0x59be }, /* -45dBm */
	{ -40, 0x8389 }, /* -40dBm */
	{ -35, 0x98a8 }, /* -35dBm */
	{ -30, 0xa298 }, /* -30dBm */
	{ -25, 0xad5a }, /* -25dBm */
	{ -20, 0xb4bc }, /* -20dBm */
	{ -15, 0xbb08 }, /* -15dBm */
	{ -10, 0xc229 }, /* -10dBm */
	{  -5, 0xcaa1 }, /*  -5dBm */
};

struct stv0910_table stv0910_dbm_lookup_x100[] = {
	{ -7000, 0x07aa }, /* -70dBm */
	{ -6500, 0x114f }, /* -65dBm */
	{ -6000, 0x210d }, /* -60dBm */
	{ -5500, 0x2d11 }, /* -55dBm */
	{ -5000, 0x3a14 }, /* -50dBm */
	{ -4500, 0x59be }, /* -45dBm */
	{ -4000, 0x8389 }, /* -40dBm */
	{ -3500, 0x98a8 }, /* -35dBm */
	{ -3000, 0xa298 }, /* -30dBm */
	{ -2500, 0xad5a }, /* -25dBm */
	{ -2000, 0xb4bc }, /* -20dBm */
	{ -1500, 0xbb08 }, /* -15dBm */
	{ -1000, 0xc229 }, /* -10dBm */
	{  -500, 0xcaa1 }, /*  -5dBm */
};

static s16 stv0910_lookup(const struct stv0910_table *table, u8 max, u32 value)
{
	u8 min = 0;
	u8 i   = 0;
	u16 diff_val;
	s16 diff_ret;
	u16 correction;

	if (value <= table[min].val) {
		return table[min].ret;
	}
	if (value >= table[max].val) {
		return table[max].ret;
	}

	while (value >= table[i].val) {
		i++;
	}
	min = i - 1;
	max = i;

	diff_val = table[max].val - table[min].val;
	correction = ((value - table[min].val)* 100) / diff_val;
	diff_ret = table[max].ret - table[min].ret;

	return ((diff_ret * correction) / 100) + table[min].ret;
}

static inline s32 comp2(s32 __x, s32 __width)
{
	if (__width == 32)
		return __x;
	else
		return (__x >= (1 << (__width - 1))) ? (__x - (1 << __width)) : __x;
}

static u32 stv0910_get_SR(struct stv0910_state *state)
{
	u64 SFR;
	SFR = (STV0910_READ_REG(state, SFR3) << 24) | (STV0910_READ_REG(state, SFR2) << 16) | (STV0910_READ_REG(state, SFR1) << 8) | STV0910_READ_REG(state, SFR0);
	SFR = (SFR * state->base->mclk) >> 32;

	printk("%s: SR: %d \n", __func__, (int)SFR);
	return SFR;
}

static int stv0910_get_signal_parameters(struct stv0910_state *state)
{
	struct dvb_frontend *fe = &state->frontend;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	u8 FE_STV0910_fec_dvbs[] = {
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_1_2,
		FEC_3_5,
		FEC_2_3,
		FEC_3_4,
		FEC_NONE,
		FEC_5_6,
		FEC_6_7,
		FEC_7_8,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE
	};

	u8 FE_STV0910_fec_dvbs2[] = {
		FEC_NONE,
		FEC_1_4,
		FEC_1_3,
		FEC_2_5,
		FEC_1_2,
		FEC_3_5,
		FEC_2_3,
		FEC_3_4,
		FEC_4_5,
		FEC_5_6,
		FEC_8_9,
		FEC_9_10,
		FEC_3_5,
		FEC_2_3,
		FEC_3_4,
		FEC_5_6,
		FEC_8_9,
		FEC_9_10,
		FEC_2_3,
		FEC_3_4,
		FEC_4_5,
		FEC_5_6,
		FEC_8_9,
		FEC_9_10,
		FEC_3_4,
		FEC_4_5,
		FEC_5_6,
		FEC_8_9,
		FEC_9_10,
		FEC_NONE,
		FEC_NONE,
		FEC_NONE
	};

	u8 FE_STV0910_rolloff[] = {
		ROLLOFF_35,
		ROLLOFF_25,
		ROLLOFF_20,
		ROLLOFF_AUTO
	};

	u8 FE_STV0910_modulation[] = {
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		QPSK,
		PSK_8,
		PSK_8,
		PSK_8,
		PSK_8,
		PSK_8,
		PSK_8,
		APSK_16,
		APSK_16,
		APSK_16,
		APSK_16,
		APSK_16,
		APSK_16,
		APSK_32,
		APSK_32,
		APSK_32,
		APSK_32,
		APSK_32
	};

	printk("%s: demod: %d \n", __func__, state->nr);

	p->symbol_rate = stv0910_get_SR(state);
	p->modulation  = FE_STV0910_modulation[STV0910_READ_FIELD(state, DEMOD_MODCOD)];
	p->pilot       = STV0910_READ_FIELD(state, DEMOD_TYPE) & 0x01;
	p->frame_len   = (STV0910_READ_FIELD(state, DEMOD_TYPE) & 0x02) >> 1;
	p->rolloff     = FE_STV0910_rolloff[STV0910_READ_FIELD(state, ROLLOFF_STATUS)];
	p->matype      = (STV0910_READ_FIELD(state, MATYPE_CURRENT1) << 8) | STV0910_READ_FIELD(state, MATYPE_CURRENT0);
	p->inversion   = STV0910_READ_FIELD(state, IQINV);

	switch (STV0910_READ_FIELD(state, HEADER_MODE)) {
	case 2:
		p->delivery_system = SYS_DVBS2;
		p->fec_inner       = FE_STV0910_fec_dvbs2[STV0910_READ_FIELD(state, DEMOD_MODCOD)];
		break;
	case 3:
		if (STV0910_READ_FIELD(state, DSS_DVB)) {
			p->delivery_system = SYS_DSS;
		} else {
			p->delivery_system = SYS_DVBS;
		}
		p->fec_inner = FE_STV0910_fec_dvbs[STV0910_READ_FIELD(state, DEMOD_MODCOD)];
		break;
	default:
		p->delivery_system = SYS_UNDEFINED;
		break;
	}

	return 0;
}

static int stv0910_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d %s \n", __func__, state->nr, enable ? "ENABLE" : "DISABLE");

	if (enable)
		mutex_lock(&state->base->i2c_lock);

	if (enable)
		stv0910_write_reg(state, RSTV0910_P1_I2CRPT, 0xc8);
	else
		stv0910_write_reg(state, RSTV0910_P1_I2CRPT, 0x4a);

	if (!enable)
		mutex_unlock(&state->base->i2c_lock);
	return 0;
}

static int stv0910_init(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	return 0;
}

static int stv0910_set_mclock(struct stv0910_state *state, u32 MasterClock)
{
	u32 idf = 1;
	u32 odf = 4;
	u32 quartz = state->base->extclk / 1000000;
	u32 Fphi = MasterClock / 1000000;
	u32 ndiv = (Fphi * odf * idf) / quartz;
	u32 cp = 7;
	u32 fvco;
	printk("%s: demod: %d \n", __func__, state->nr);

	if (ndiv >= 7 && ndiv <= 71)
		cp = 7;
	else if (ndiv >=  72 && ndiv <=  79)
		cp = 8;
	else if (ndiv >=  80 && ndiv <=  87)
		cp = 9;
	else if (ndiv >=  88 && ndiv <=  95)
		cp = 10;
	else if (ndiv >=  96 && ndiv <= 103)
		cp = 11;
	else if (ndiv >= 104 && ndiv <= 111)
		cp = 12;
	else if (ndiv >= 112 && ndiv <= 119)
		cp = 13;
	else if (ndiv >= 120 && ndiv <= 127)
		cp = 14;
	else if (ndiv >= 128 && ndiv <= 135)
		cp = 15;
	else if (ndiv >= 136 && ndiv <= 143)
		cp = 16;
	else if (ndiv >= 144 && ndiv <= 151)
		cp = 17;
	else if (ndiv >= 152 && ndiv <= 159)
		cp = 18;
	else if (ndiv >= 160 && ndiv <= 167)
		cp = 19;
	else if (ndiv >= 168 && ndiv <= 175)
		cp = 20;
	else if (ndiv >= 176 && ndiv <= 183)
		cp = 21;
	else if (ndiv >= 184 && ndiv <= 191)
		cp = 22;
	else if (ndiv >= 192 && ndiv <= 199)
		cp = 23;
	else if (ndiv >= 200 && ndiv <= 207)
		cp = 24;
	else if (ndiv >= 208 && ndiv <= 215)
		cp = 25;
	else if (ndiv >= 216 && ndiv <= 223)
		cp = 26;
	else if (ndiv >= 224 && ndiv <= 225)
		cp = 27;

	stv0910_write_reg(state, RSTV0910_NCOARSE, (cp << 3) | idf);
	stv0910_write_reg(state, RSTV0910_NCOARSE2, odf);
	stv0910_write_reg(state, RSTV0910_NCOARSE1, ndiv);

	fvco = (quartz * 2 * ndiv) / idf;
	state->base->mclk = fvco / (2 * odf) * 1000000;

	return 0;
}

static int stv0910_get_dmdlock(struct stv0910_state *state)
{
	struct dvb_frontend *fe = &state->frontend;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u16 timeout = 0;
	u16 timer = 0;
	u8 lock = 0;

	printk("%s: demod: %d \n", __func__, state->nr);

	if (p->symbol_rate <= 1000000) {  /*SR <=1Msps*/
		timeout = 3000;
	} else if (p->symbol_rate <= 2000000) {  /*1Msps < SR <=2Msps*/
		timeout = 2500;
	} else if (p->symbol_rate <= 5000000) {  /*2Msps< SR <=5Msps*/
		timeout = 1000;
	} else if (p->symbol_rate <= 10000000) {  /*5Msps< SR <=10Msps*/
		timeout = 700;
	} else if (p->symbol_rate < 20000000) {  /*10Msps< SR <=20Msps*/
		timeout = 700;
	} else {  /*SR >=20Msps*/
		timeout = 700;
	}

	while (timer < timeout && !lock) {
		if (STV0910_READ_FIELD(state, HEADER_MODE) >= 2) {
			lock = STV0910_READ_FIELD(state, LOCK_DEFINITIF);
		}
		if (!lock)
			msleep(10);
		printk("%s: %s \n", __func__, lock ? "LOCKED" : "SEARCHING");
		timer += 10;
	}

	return lock;
}

static int stv0910_start(struct stv0910_state *state, struct dtv_frontend_properties *p)
{
	struct dvb_frontend *fe = &state->frontend;
	const struct stv0910_cfg *config = state->config;

	u16 SFR;
	int lock;
	u8  i;
	s64 CFR;
	s32 offset;

	printk("%s: demod: %d \n", __func__, state->nr);

	if (p->symbol_rate < 100000 || p->symbol_rate > 70000000)
		return -EINVAL;

	p->bandwidth_hz = 36000000 * 2;

start:
//	STV0910_WRITE_FIELD(state, RST_HWARE, 0x01);
//	STV0910_WRITE_FIELD(state, ALGOSWRST, 0x01);
	STV0910_WRITE_REG(state, AGC2O, 0x5B);
	STV0910_WRITE_REG(state, DMDISTATE, 0x5C); // Demod Stop

	stv0910_i2c_gate_ctrl(fe, 1);
	config->tuner_set_bandwidth(fe, p->bandwidth_hz);
	config->tuner_set_frequency(fe, p->frequency);
	stv0910_i2c_gate_ctrl(fe, 0);

	STV0910_WRITE_REG(state, SFRUP1, 0x83);  // SR = 65,000 Ksps
	STV0910_WRITE_REG(state, SFRUP0, 0xC0);
	STV0910_WRITE_REG(state, SFRLOW1, 0x82); // SR = 400 Ksps
	STV0910_WRITE_REG(state, SFRLOW0, 0xA0);

	/* Set the Init Symbol rate*/
	SFR = (p->symbol_rate * 65536) / state->base->mclk;
	STV0910_WRITE_REG(state, SFRINIT1, (SFR >> 8) & 0x7F);
	STV0910_WRITE_REG(state, SFRINIT0, SFR & 0xFF);

	/* FE_STV0910_SetSearchStandard */
	STV0910_WRITE_REG(state, DMDCFGMD, 0xD9);

	/* Disable DSS */
	STV0910_WRITE_REG(state, FECM, 0x00);
	STV0910_WRITE_REG(state, PRVIT, 0x2F);

	/* 8PSK 3/5, 8PSK 2/3 Poff tracking optimization WA*/
	STV0910_WRITE_REG(state, ACLC2S2Q, 0x0B);
	STV0910_WRITE_REG(state, ACLC2S28, 0x0A);
	STV0910_WRITE_REG(state, BCLC2S2Q, 0x84);
	STV0910_WRITE_REG(state, BCLC2S28, 0x84);
	STV0910_WRITE_REG(state, CARHDR, 0x1C);
	/* Reset demod */
	STV0910_WRITE_REG(state, DMDISTATE, 0x1F); // Reset demod state machine

	STV0910_WRITE_REG(state, CARCFG, 0x46);

	STV0910_WRITE_REG(state, CFRUP1, 0x10);
	STV0910_WRITE_REG(state, CFRUP0, 0x4e);
	/*CFR Low Setting*/
	STV0910_WRITE_REG(state, CFRLOW1, 0xef);
	STV0910_WRITE_REG(state, CFRLOW0, 0xb2);

	/* init the demod frequency offset to 0 */
	STV0910_WRITE_REG(state, CFRINIT1, 0);
	STV0910_WRITE_REG(state, CFRINIT0, 0);

	STV0910_WRITE_REG(state, DMDISTATE, 0x1F);
	STV0910_WRITE_REG(state, DMDISTATE, 0x01); // Blindsearch - best guess

	msleep(50);

	lock = 0;
	for (i = 0; i < 10; i++) {
		if (STV0910_READ_FIELD(state, TMGLOCK_QUALITY) >= 2) {
			lock++;
		}
	}
	printk("%s: course lock: %d \n", __func__, lock);

	CFR    = MAKEWORD16(STV0910_READ_REG(state, CFR2), STV0910_READ_REG(state, CFR1));
	CFR    = comp2(CFR, 16);
	CFR   *= (state->base->mclk / 1000);
	offset = CFR >> 16;
	printk("%s: freq: %d offset %d\n", __func__, p->frequency, offset);
	p->frequency   += offset;
	p->symbol_rate  = stv0910_get_SR(state);
	p->bandwidth_hz = (p->symbol_rate * 13) / 10;
	if (offset > 1000 || offset < -1000) {
		printk("%s: corrected frequency: %d RESTARTING\n", __func__, p->frequency);
		goto start;
	}
	printk("%s: corrected bandwidth: %d \n", __func__, p->bandwidth_hz);
	stv0910_i2c_gate_ctrl(fe, 1);
	config->tuner_set_bandwidth(fe, p->bandwidth_hz);
	stv0910_i2c_gate_ctrl(fe, 0);

	printk("%s: SR: %d \n", __func__, stv0910_get_SR(state));

	return stv0910_get_dmdlock(state);
}

static int stv0910_init_diseqc(struct stv0910_state *state)
{
	u8 Freq = ((state->base->mclk + 11000 * 32) / (22000 * 32));
	printk("%s: demod: %d \n", __func__, state->nr);

	/* Disable receiver */
	STV0910_WRITE_REG(state, DISRXCFG, 0x00);
	STV0910_WRITE_REG(state, DISTXCFG, 0xBA); /* Reset = 1 */
	STV0910_WRITE_REG(state, DISTXCFG, 0x3A); /* Reset = 0 */
	STV0910_WRITE_REG(state, DISTXF22, Freq);
	return 0;
}

static int stv0910_probe(struct stv0910_state *state)
{
	u8 id;
	printk("%s: demod: %d \n", __func__, state->nr);

	id = stv0910_read_reg(state, RSTV0910_MID);
	if (id != 0x51)
		return -EINVAL;
	pr_info("stv0910: found STV0910 id=0x%02x\n", id);

	 /* Configure the I2C repeater to off */
	stv0910_write_reg(state, RSTV0910_P1_I2CRPT, 0x24);
	/* Configure the I2C repeater to off */
	stv0910_write_reg(state, RSTV0910_P2_I2CRPT, 0x24);
	/* Set the I2C to oversampling ratio */
	stv0910_write_reg(state, RSTV0910_I2CCFG, 0x88);

	stv0910_write_reg(state, RSTV0910_GPIO5CFG,  0x04);
	stv0910_write_reg(state, RSTV0910_GPIO6CFG,  0x06);
	stv0910_write_reg(state, RSTV0910_P2_AGC1CN,  0x99);

	stv0910_write_reg(state, RSTV0910_OUTCFG,    0x00);  /* OUTCFG */
	stv0910_write_reg(state, RSTV0910_PADCFG,    0x05);  /* RF AGC Pads Dev = 05 */
	stv0910_write_reg(state, RSTV0910_SYNTCTRL,  0x02);  /* SYNTCTRL */
	stv0910_write_reg(state, RSTV0910_TSGENERAL, 0x00);  /* TSGENERAL */
	stv0910_write_reg(state, RSTV0910_CFGEXT,    0x02);  /* CFGEXT */
	stv0910_write_reg(state, RSTV0910_GENCFG,    0x15);  /* GENCFG */

	stv0910_write_reg(state, RSTV0910_TSTRES0, 0x80); /* LDPC Reset */
	stv0910_write_reg(state, RSTV0910_TSTRES0, 0x00);

	stv0910_set_mclock(state, 135000000);

	/* TS output */
	stv0910_write_reg(state, RSTV0910_P1_TSCFGH, state->tscfgh | 0x01);
	stv0910_write_reg(state, RSTV0910_P1_TSCFGH, state->tscfgh);
	stv0910_write_reg(state, RSTV0910_P1_TSCFGM, 0xC0);  /* Manual speed */
	stv0910_write_reg(state, RSTV0910_P1_TSCFGL, 0x20);

	/* Speed = 67.5 MHz */
	stv0910_write_reg(state, RSTV0910_P1_TSSPEED, state->tsspeed);

	stv0910_write_reg(state, RSTV0910_P2_TSCFGH, state->tscfgh | 0x01);
	stv0910_write_reg(state, RSTV0910_P2_TSCFGH, state->tscfgh);
	stv0910_write_reg(state, RSTV0910_P2_TSCFGM, 0xC0);  /* Manual speed */
	stv0910_write_reg(state, RSTV0910_P2_TSCFGL, 0x20);

	/* Speed = 67.5 MHz */
	stv0910_write_reg(state, RSTV0910_P2_TSSPEED, state->tsspeed);

	/* Reset stream merger */
	stv0910_write_reg(state, RSTV0910_P1_TSCFGH, state->tscfgh | 0x01);
	stv0910_write_reg(state, RSTV0910_P2_TSCFGH, state->tscfgh | 0x01);
	stv0910_write_reg(state, RSTV0910_P1_TSCFGH, state->tscfgh);
	stv0910_write_reg(state, RSTV0910_P2_TSCFGH, state->tscfgh);

	stv0910_write_reg(state, RSTV0910_P1_I2CRPT, 0x4a);
	stv0910_write_reg(state, RSTV0910_P2_I2CRPT, 0x4a);

	stv0910_init_diseqc(state);
	return 0;
}

static void stv0910_release(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	state->base->count--;
	if (state->base->count == 0) {
		list_del(&state->base->stvlist);
		kfree(state->base);
	}
	kfree(state);
}

static int stv0910_set_parameters(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	printk("%s: demod: %d \n", __func__, state->nr);
	printk("%s: freq: %d, bw: %d \n", __func__, p->frequency, p->bandwidth_hz);

	return stv0910_start(state, p);
}

static int stv0910_get_frontend_algo(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	if (state->algo == STV0910_NOTUNE) {
		return DVBFE_ALGO_NOTUNE;
	} else {
		return DVBFE_ALGO_CUSTOM;
	}
}

static int stv0910_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d tone:%s \n", __func__, state->nr, tone ? "ON" : "OFF");

	switch (tone) {
	case SEC_TONE_ON:
		return STV0910_WRITE_REG(state, DISTXCFG, 0x00);
	case SEC_TONE_OFF:
		return STV0910_WRITE_REG(state, DISTXCFG, 0x02);
	default:
		break;
	}
	return -EINVAL;
}

static enum dvbfe_search stv0910_search(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;

	if (state->algo == STV0910_NOTUNE) {
		return DVBFE_ALGO_SEARCH_FAILED;
	}

	printk("%s: demod: %d \n", __func__, state->nr);

	if (stv0910_set_parameters(fe))
		return DVBFE_ALGO_SEARCH_SUCCESS;
	else
		return DVBFE_ALGO_SEARCH_FAILED;
}

static int stv0910_set_property(struct dvb_frontend *fe,
				struct dtv_property *tvp)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	if (tvp->cmd == DTV_TUNE) {
		state->algo = STV0910_BLIND_SEARCH;
	}

	return 0;
}

static int stv0910_get_property(struct dvb_frontend *fe,
				struct dtv_property *tvp)
{
	return 0;
}

static int stv0910_set_frontend(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	state->algo = STV0910_BLIND_SEARCH;

	return 0;
}

static int stv0910_wait_dis(struct stv0910_state *state, u8 flag, u8 val)
{
	int i;
	u8 stat;
	printk("%s: demod: %d \n", __func__, state->nr);

	for (i = 0; i < 10; i++) {
		stat = STV0910_READ_REG(state, DISTXSTATUS);
		if ((stat & flag) == val)
			return 0;
		msleep(10);
	}
	return -1;
}

static int stv0910_send_master_cmd(struct dvb_frontend *fe,
			   struct dvb_diseqc_master_cmd *cmd)
{
	struct stv0910_state *state = fe->demodulator_priv;
	int i;
	printk("%s: demod: %d \n", __func__, state->nr);

	/*pr_info("master_cmd %02x %02x %02x %02x\n",
	  cmd->msg[0],  cmd->msg[1],  cmd->msg[2],  cmd->msg[3]);*/
	STV0910_WRITE_REG(state, DISTXCFG, 0x3E);
	for (i = 0; i < cmd->msg_len; i++) {
		stv0910_wait_dis(state, 0x40, 0x00);
		STV0910_WRITE_REG(state, DISTXFIFO, cmd->msg[i]);
	}
	STV0910_WRITE_REG(state, DISTXCFG, 0x3A);
	stv0910_wait_dis(state, 0x20, 0x20);
	return 0;
}

static int stv0910_recv_slave_reply(struct dvb_frontend *fe,
			    struct dvb_diseqc_slave_reply *reply)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);
	return 0;
}

static int stv0910_send_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);
#if 0
	u8 value;

	if (burst == SEC_MINI_A) {
		STV0910_WRITE_REG(state, DISTXCFG, 0x3F);
		value = 0x00;
	} else {
		STV0910_WRITE_REG(state, DISTXCFG, 0x3E);
		value = 0xFF;
	}
	wait_dis(state, 0x40, 0x00);
	STV0910_WRITE_REG(state, DISTXFIFO, value);
	STV0910_WRITE_REG(state, DISTXCFG, 0x3A);
	wait_dis(state, 0x20, 0x20);
#endif
	return 0;
}

static int stv0910_sleep(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	state->algo = STV0910_NOTUNE;
	STV0910_WRITE_REG(state, DMDISTATE, 0x5C); // Demod Stop

	return 0;
}

static int stv0910_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	return 0;
}

static int stv0910_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	return 0;
}

static int stv0910_read_dbm(struct dvb_frontend *fe, s16 *strength)
{
	struct stv0910_state *state = fe->demodulator_priv;
	u16 agc;

	agc = MAKEWORD16(STV0910_READ_REG(state, AGCIQIN1), STV0910_READ_REG(state, AGCIQIN0));
	*strength = stv0910_lookup(stv0910_dbm_lookup_x100, ARRAY_SIZE(stv0910_dbm_lookup_x100) - 1, agc);

	return 0;
}

static int stv0910_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	s16 dbm;
	stv0910_read_dbm(fe, &dbm);
	*strength = ((dbm * 100) + 100) * 0xFFFF / 100;
	return 0;
}

static int stv0910_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	return 0;
}

static int stv0910_get_stats(struct dvb_frontend *fe)
{
	struct stv0910_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u8 i;
	u32 value = 0;
	s16 dbm = 0;

	printk("%s: demod: %d \n", __func__, state->nr);

	switch (STV0910_READ_FIELD(state, HEADER_MODE)) {
	case FE_DVB_S2:
		for (i = 0; i < 10; i++) {
			value += MAKEWORD16(STV0910_READ_FIELD(state, NOSPLHT_NORMED1), STV0910_READ_FIELD(state, NOSPLHT_NORMED0));
		}
		value /= 10;
		p->cnr.stat[0].svalue = stv0910_lookup(stv0910_S2_SNR_lookup, ARRAY_SIZE(stv0910_S2_SNR_lookup) - 1, value) * 1000;
		p->cnr.stat[0].scale  = FE_SCALE_DECIBEL;
		break;
	case FE_DVB_S:
		for (i = 0; i < 10; i++) {
			value += MAKEWORD16(STV0910_READ_FIELD(state, NOSDATAT_NORMED1), STV0910_READ_FIELD(state, NOSDATAT_NORMED0));
		}
		value /= 10;
		p->cnr.stat[0].svalue = stv0910_lookup(stv0910_S1_SNR_lookup, ARRAY_SIZE(stv0910_S1_SNR_lookup) - 1, value) * 1000;
		p->cnr.stat[0].scale  = FE_SCALE_DECIBEL;
		break;
	default:
		break;
	}
	p->strength.stat[0].scale  = FE_SCALE_DECIBEL;
	stv0910_read_dbm(fe, &dbm);
	p->strength.stat[0].svalue = dbm * 100;

	return 0;
}

static int stv0910_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct stv0910_state *state = fe->demodulator_priv;
	printk("%s: demod: %d \n", __func__, state->nr);

	if (state->algo == STV0910_NOTUNE) {
		*status = FE_TIMEDOUT;
		return 0;
	}

	*status = 0;

	if (STV0910_READ_FIELD(state, CAR_LOCK)) {
		*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER;
	}

	switch (STV0910_READ_FIELD(state, HEADER_MODE)) {
	case FE_DVB_S:
		if (STV0910_READ_FIELD(state, LOCK_DEFINITIF)) {
			if (STV0910_READ_FIELD(state, LOCKEDVIT)) {
				*status |= FE_HAS_VITERBI;
				if (STV0910_READ_FIELD(state, TSFIFO_LINEOK)) {
					*status |= FE_HAS_SYNC | FE_HAS_LOCK;
				}

			}
		}
		stv0910_get_signal_parameters(state);
		break;
	case FE_DVB_S2:
		if (STV0910_READ_FIELD(state, LOCK_DEFINITIF)) {
			if (STV0910_READ_FIELD(state, PKTDELIN_LOCK)) {
				*status |= FE_HAS_VITERBI;
				if (STV0910_READ_FIELD(state, TSFIFO_LINEOK)) {
					*status |= FE_HAS_SYNC | FE_HAS_LOCK;
				}

			}

		}
		stv0910_get_signal_parameters(state);
		break;
	default:
		break;
	}

	stv0910_get_stats(fe);

	return 0;
}

static int stv0910_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	struct stv0910_state *state = fe->demodulator_priv;
	const struct stv0910_cfg *config = state->config;
	u32 x;

	printk("%s: demod: %d \n", __func__, state->nr);

	state->algo = STV0910_NOTUNE;

	STV0910_WRITE_REG(state, AGC2O, 0x5B);
	STV0910_WRITE_REG(state, AGC2REF, 0x38);
	STV0910_WRITE_REG(state, DMDISTATE, 0x5C); // Demod Stop

	stv0910_i2c_gate_ctrl(fe, 1);
	config->tuner_set_bandwidth(fe, 1000000);

	*s->type = SC_DBM;

	STV0910_WRITE_REG(state, DMDISTATE, 0x5C);
	for (x = 0 ; x < s->num_freq ; x++)
	{
		STV0910_WRITE_REG(state, DMDISTATE, 0x1C);

		config->tuner_set_frequency(fe, *(s->freq + x));

		msleep(10);

		stv0910_read_dbm(fe, (s->rf_level + x));
	}

	stv0910_i2c_gate_ctrl(fe, 0);

	return 0;
}

static int stv0910_get_consellation_samples(struct dvb_frontend *fe, struct dvb_fe_constellation_samples *s)
{
	struct stv0910_state *state = fe->demodulator_priv;
	u32 x;
	u8 buf[2];

	STV0910_WRITE_REG(state, IQCONST, s->options);

	for (x = 0 ; x < s->num ; x++)
	{
		STV0910_READ_REGS(state, ISYMB, buf, 2);
		s->samples[x].imaginary = buf[0];
		s->samples[x].real = buf[1];
	}
	return 0;
}

static struct dvb_frontend_ops stv0910_ops = {
	.delsys = { SYS_DSS, SYS_DVBS, SYS_DVBS2 },
	.info = {
		.name			= "STV0910",
		.frequency_min		= 950000,
		.frequency_max		= 2150000,
		.frequency_stepsize	= 0,
		.frequency_tolerance	= 0,
		.symbol_rate_min	= 1000000,
		.symbol_rate_max	= 67500000,
		.caps			= FE_CAN_INVERSION_AUTO |
					  FE_CAN_FEC_AUTO       |
					  FE_CAN_QPSK           |
					  FE_CAN_2G_MODULATION  |
					  FE_CAN_SPECTRUMSCAN   |
					  FE_CAN_IQ             |
					  FE_CAN_BLINDSEARCH
	},
	.init				= stv0910_init,
	.sleep				= stv0910_sleep,
	.release                        = stv0910_release,
	.i2c_gate_ctrl                  = stv0910_i2c_gate_ctrl,
	.get_frontend_algo              = stv0910_get_frontend_algo,
	.read_status			= stv0910_read_status,
	.set_tone			= stv0910_set_tone,

	.search				= stv0910_search,
	.set_property			= stv0910_set_property,
	.get_property			= stv0910_get_property,
	.set_frontend			= stv0910_set_frontend,

	.diseqc_send_master_cmd		= stv0910_send_master_cmd,
	.diseqc_send_burst		= stv0910_send_burst,
	.diseqc_recv_slave_reply	= stv0910_recv_slave_reply,

	.read_snr			= stv0910_read_snr,
	.read_ber			= stv0910_read_ber,
	.read_signal_strength		= stv0910_read_signal_strength,
	.read_ucblocks			= stv0910_read_ucblocks,

	.get_spectrum_scan		= stv0910_get_spectrum_scan,
	.get_constellation_samples	= stv0910_get_consellation_samples,
};

static struct stv_base *match_base(struct i2c_adapter  *i2c, u8 adr)
{
	struct stv_base *p;

	list_for_each_entry(p, &stvlist, stvlist)
		if (p->i2c == i2c && p->adr == adr)
			return p;
	return NULL;
}

struct dvb_frontend *stv0910_attach(struct i2c_adapter *i2c,
				    struct stv0910_cfg *cfg,
				    int nr)
{
	struct stv0910_state *state;
	struct stv_base *base;

	state = kzalloc(sizeof(struct stv0910_state), GFP_KERNEL);
	if (!state)
		return NULL;

	state->nr = nr;
	state->tscfgh  = 0x20 | (cfg->parallel ? 0 : 0x40);
	state->tsspeed = 0x40;

	base = match_base(i2c, cfg->adr);
	if (base) {
		base->count++;
		state->base = base;
	} else {
		base = kzalloc(sizeof(struct stv_base), GFP_KERNEL);
		if (!base)
			goto fail;
		base->i2c = i2c;
		base->adr = cfg->adr;
		base->count = 1;
		base->extclk = cfg->clk ? cfg->clk : 30000000;

		mutex_init(&base->i2c_lock);
		mutex_init(&base->reg_lock);
		state->base = base;
		if (stv0910_probe(state) < 0) {
			kfree(base);
			goto fail;
		}
		list_add(&base->stvlist, &stvlist);
	}
	state->config		          = cfg;
	state->frontend.ops               = stv0910_ops;
	state->frontend.demodulator_priv  = state;

	return &state->frontend;

fail:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL_GPL(stv0910_attach);

MODULE_DESCRIPTION("STV0910 driver");
MODULE_AUTHOR("Ralph Metzler, Manfred Voelkel");
MODULE_LICENSE("GPL");
