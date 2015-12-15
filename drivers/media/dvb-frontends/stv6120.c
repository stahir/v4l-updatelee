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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "dvb_frontend.h"

#include "stv6110x.h"
#include "stv6120_reg.h"
#include "stv6120.h"
#include "stv6120_priv.h"

static unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "Set Verbosity level");

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

static int stv6120_read_regs(struct stv6120_state *state, u16 reg, u8 *data, u8 len)
{
	int ret;
	u8 b0[] = { reg & 0xff };
	struct i2c_msg msg[] = {
		{ .addr = state->config->addr, .flags = 0,        .buf = b0,   .len = 1 },
		{ .addr = state->config->addr, .flags = I2C_M_RD, .buf = data, .len = len }
	};

	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret != 2) {
		printk("I2C Error\n");
		return -EREMOTEIO;
	}
	return 0;
}

static u8 stv6120_read_reg(struct stv6120_state *state, u16 reg)
{
	u8 data = 0x00;
	stv6120_read_regs(state, reg, &data, 1);
	return data;
}

static u8 stv6120_read_field(struct stv6120_state *state, u32 label)
{
	u8 mask, pos, data;

	extract_mask_pos(label, &mask, &pos);

	data = stv6120_read_reg(state, label >> 16);
	data = (data & mask) >> pos;

	return data;
}

static int stv6120_write_regs(struct stv6120_state *state, u16 reg, u8 *data, u8 len)
{
	int ret;
	u8 buf[MAX_XFER_SIZE];

	struct i2c_msg msg = {
		.addr  = state->config->addr,
		.flags = 0,
		.buf   = buf,
		.len   = len + 1
	};

	buf[0] = reg & 0xff;
	memcpy(&buf[1], data, len);

	ret = i2c_transfer(state->i2c, &msg, 1);
	if (ret != 1) {
		printk("I2C Error\n");
		return -EREMOTEIO;
	}

	return 0;
}

static int stv6120_write_reg(struct stv6120_state *state, u16 reg, u8 data)
{
	return stv6120_write_regs(state, reg, &data, 1);
}

static int stv6120_write_field(struct stv6120_state *state, u32 label, u8 data)
{
	u8 reg, mask, pos;

	reg = stv6120_read_reg(state, (label >> 16) & 0xffff);
	extract_mask_pos(label, &mask, &pos);

	data = mask & (data << pos);
	reg = (reg & (~mask)) | data;

	return stv6120_write_reg(state, (label >> 16) & 0xffff, reg);
}

static int stv6120_init(struct dvb_frontend *fe)
{
	struct stv6120_state *state = fe->tuner_priv;
//	const struct stv6120_config *config = state->config;

	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	stv6120_write_reg(state, RSTV6120_STAT1,  0x0E);
	stv6120_write_reg(state, RSTV6120_CTRL10, 0x03);
	stv6120_write_reg(state, RSTV6120_CTRL15, 0x0D);
	stv6120_write_reg(state, RSTV6120_STAT2,  0x0E);
	stv6120_write_reg(state, RSTV6120_CTRL18, 0x00);
	stv6120_write_reg(state, RSTV6120_CTRL19, 0x00);
	stv6120_write_reg(state, RSTV6120_CTRL20, 0x4C);
	stv6120_write_reg(state, RSTV6120_CTRL21, 0x00);
	stv6120_write_reg(state, RSTV6120_CTRL22, 0x00);
	stv6120_write_reg(state, RSTV6120_CTRL23, 0x4C);
	stv6120_write_reg(state, RSTV6120_CTRL10, 0x13);
	stv6120_write_reg(state, RSTV6120_CTRL10, 0x1B);

//	stv6120_write_field(state, FSTV6120_K, (config->refclk/1000000) - 16);

//	if (config->refclk >= 27000000) {
//		stv6120_write_field(state, FSTV6120_RDIV, 1);
//	} else {
//		stv6120_write_field(state, FSTV6120_RDIV, 0);
//	}

	return 0;
}

static int stv6120_set_frequency(struct dvb_frontend *fe, u32 frequency)
{
	struct stv6120_state *state = fe->tuner_priv;
	const struct stv6120_config *config = state->config;

	u32 Fxtl, Fvco, FRdiv, N, F;
	u8  P, PDiv, R, ICP;

	frequency /= 1000;

	printk(KERN_INFO "%s: tuner: %d, freq: %d \n", __func__, state->tuner, frequency);

	if (frequency >= 250) {
		P = 16;
		PDiv = 3;
	}
	if (frequency >= 299) {
		P = 8;
		PDiv = 2;
	}
	if (frequency >= 596) {
		P = 4;
		PDiv = 1;
	}
	if (frequency >= 1191) {
		P = 2;
		PDiv = 0;
	}

	Fvco = frequency * P;

	if (Fvco >= 2380) {
		ICP = 0;
	}
	if (Fvco >= 2473) {
		ICP = 1;
	}
	if (Fvco >= 2701) {
		ICP = 2;
	}
	if (Fvco >= 3022) {
		ICP = 3; // 3 or 4
	}
	if (Fvco >= 3388) {
		ICP = 5;
	}
	if (Fvco >= 3846) {
		ICP = 6;
	}
	if (Fvco >= 4395) {
		ICP = 7;
	}

	// fLO = fVCO / P = (fXTAL / R) * (N + F / 2^18) / P
	Fxtl  = config->refclk / 1000000; // 30mhz
	R     = config->clk_div;          // 2
	FRdiv = Fxtl / R;
	N     = Fvco / FRdiv;
	F     = ((Fvco % FRdiv) * 0x40000) / FRdiv;

	printk(KERN_INFO "%s: Fvco:%02x Fxtl:%02x R:%02x FRdiv:%02x ICP:%02x PDiv:%02x \n", __func__, Fvco, Fxtl, R, FRdiv, ICP, PDiv);
	printk(KERN_INFO "%s: N:%08x F:%08x \n", __func__, N, F);

	STV6120_WRITE_FIELD(state, ICP, ICP);
	STV6120_WRITE_FIELD(state, PDIV, PDiv);
	STV6120_WRITE_FIELD(state, NDIV_LSB, (N & 0xFF));
	STV6120_WRITE_FIELD(state, NDIV_MSB, ((N>>8) && 0x01));
	STV6120_WRITE_FIELD(state, F_H, ((F>>15) & 0x07));
	STV6120_WRITE_FIELD(state, F_M, ((F>>7) & 0xFF));
	STV6120_WRITE_FIELD(state, F_L, (F & 0x7F));
	STV6120_WRITE_FIELD(state, CALVCOSTRT, 1); // VCO Auto Calibration

	while (!STV6120_READ_FIELD(state, LOCK)) {
		msleep(10);
	}

	return 0;
}

static int stv6120_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	return 0;
}

static int stv6120_set_bandwidth(struct dvb_frontend *fe, u32 bandwidth)
{
	struct stv6120_state *state = fe->tuner_priv;
	u8 lpf;
	s32 i;

	printk(KERN_INFO "%s: tuner: %d, bw: %d \n", __func__, state->tuner, bandwidth);

	if ((bandwidth/2) > 36000000) // F[4:0] BW/2 max =31+5=36 mhz for F=31
		lpf = 31;
	else if ((bandwidth/2) < 5000000) // BW/2 min = 5Mhz for F=0
		lpf = 0;
	else // if 5 < BW/2 < 36
		lpf = (bandwidth/2)/1000000 - 5;

	STV6120_WRITE_FIELD(state, CF, lpf); // Set the LPF value
	STV6120_WRITE_FIELD(state, CALRCSTRT, 1); // Start LPF auto calibration

	i=0;
	while((i<10) && (STV6120_READ_FIELD(state, CALRCSTRT) != 0))
	{
		msleep(10); // wait for LPF auto calibration
		i++;
	}

	printk(KERN_INFO "%s: LPF:%d \n", __func__, lpf);
	return 0;
}

static int stv6120_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	return 0;
}

static int stv6120_get_bbgain(struct dvb_frontend *fe, u32 *gain)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	return 0;
}

static int stv6120_set_bbgain(struct dvb_frontend *fe, u32 gain)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	return 0;
}

static int stv6120_set_refclock(struct dvb_frontend *fe, u32 refclock)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	return 0;
}

static int stv6120_set_mode(struct dvb_frontend *fe, enum tuner_mode mode)
{
	struct stv6120_state *state = fe->tuner_priv;

	switch (mode) {
//	case TUNER_WAKE:
//		printk(KERN_INFO "%s: TUNER_WAKE\n", __func__);
//		if (state->tuner == TUNER_1) {
//			stv6120_write_field(state, SYN_1, 0);
//			stv6120_write_field(state, SDOFF_1, 1);
//			stv6120_write_field(state, PATHON_1, 0);
//		}
//		if (state->tuner == TUNER_2) {
//			stv6120_write_field(state, SYN_2, 0);
//			stv6120_write_field(state, SDOFF_1, 1);
//			stv6120_write_field(state, PATHON_1, 0);
//		}
		break;
	case TUNER_WAKE:
		printk(KERN_INFO "%s: tuner: %d TUNER_WAKE\n", __func__, state->tuner);

//		stv6120_write_field(state, LNABON, 1);
//		stv6120_write_field(state, LNACON, 1);

//		stv6120_write_field(state, SYN_1, 1);
//		stv6120_write_field(state, SDOFF_1, 0);
//		stv6120_write_field(state, PATHON_1, 1);
//		stv6120_write_field(state, SYN_2, 1);
//		stv6120_write_field(state, SDOFF_2, 0);
//		stv6120_write_field(state, PATHON_2, 1);
		break;
	case TUNER_SLEEP: // This actually TUNER_WAKE up the tuner, fix this
		printk(KERN_INFO "%s: tuner: %d TUNER_SLEEP\n", __func__, state->tuner);

//		stv6120_write_field(state, SYN_1, 1);
//		stv6120_write_field(state, SDOFF_1, 0);
//		stv6120_write_field(state, PATHON_1, 1);
//		stv6120_write_field(state, SYN_2, 1);
//		stv6120_write_field(state, SDOFF_2, 0);
//		stv6120_write_field(state, PATHON_2, 1);
		break;
	}

	return 0;
}

static int stv6120_sleep(struct dvb_frontend *fe)
{
	if (fe->tuner_priv)
		return stv6120_set_mode(fe, TUNER_SLEEP);

	return 0;
}

static int stv6120_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct stv6120_state *state = fe->tuner_priv;
	printk(KERN_INFO "%s: tuner: %d \n", __func__, state->tuner);

	if (STV6120_READ_FIELD(state, LOCK)) {
		*status = TUNER_PHASELOCKED;
	} else {
		*status = 0;
	}

	return 0;
}

static int stv6120_get_state(struct dvb_frontend *fe,
			     enum tuner_param param,
			     struct tuner_state *state)
{
	switch (param) {
	case DVBFE_TUNER_FREQUENCY:
		stv6120_get_frequency(fe, &state->frequency);
		break;
	case DVBFE_TUNER_TUNERSTEP:
		break;
	case DVBFE_TUNER_IFFREQ:
		break;
	case DVBFE_TUNER_BANDWIDTH:
		stv6120_get_bandwidth(fe, &state->bandwidth);
		break;
	case DVBFE_TUNER_REFCLOCK:
		break;
	default:
		break;
	}

	return 0;
}

static int stv6120_set_state(struct dvb_frontend *fe,
			     enum tuner_param param,
			     struct tuner_state *state)
{
	switch (param) {
	case DVBFE_TUNER_FREQUENCY:
		stv6120_set_frequency(fe, state->frequency);
		break;
	case DVBFE_TUNER_TUNERSTEP:
		break;
	case DVBFE_TUNER_IFFREQ:
		break;
	case DVBFE_TUNER_BANDWIDTH:
		stv6120_set_bandwidth(fe, state->bandwidth);
		break;
	case DVBFE_TUNER_REFCLOCK:
		break;
	default:
		break;
	}

	return 0;
}

static int stv6120_release(struct dvb_frontend *fe)
{
	struct stv6120_state *state = fe->tuner_priv;

	fe->tuner_priv = NULL;
	kfree(state);

	return 0;
}

static int stv6120_set_params(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct stv6120_state *state = fe->tuner_priv;

	printk(KERN_INFO "%s: tuner: %d\n", __func__, state->tuner);

	stv6120_set_bandwidth(fe, c->bandwidth_hz);
	stv6120_set_frequency(fe, c->frequency);

	return 0;
}

static struct dvb_tuner_ops stv6120_ops = {
	.info = {
		.name		= "STV6120 Silicon Tuner",
		.frequency_min	=  950000,
		.frequency_max	= 2150000,
		.frequency_step	= 0,
	},
	.sleep          = stv6120_sleep,
	.get_status	= stv6120_get_status,
	.get_state	= stv6120_get_state,
	.set_state	= stv6120_set_state,
	.release	= stv6120_release
};

static struct stv6120_devctl stv6120_ctl = {
	.tuner_sleep		= stv6120_sleep,
	.tuner_set_mode		= stv6120_set_mode,
	.tuner_set_frequency	= stv6120_set_frequency,
	.tuner_get_frequency	= stv6120_get_frequency,
	.tuner_set_bandwidth	= stv6120_set_bandwidth,
	.tuner_get_bandwidth	= stv6120_get_bandwidth,
	.tuner_set_bbgain	= stv6120_set_bbgain,
	.tuner_get_bbgain	= stv6120_get_bbgain,
	.tuner_set_refclk	= stv6120_set_refclock,
	.tuner_get_status	= stv6120_get_status,
	.tuner_set_params	= stv6120_set_params,
};

struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe,
					const struct stv6120_config *config,
					struct i2c_adapter *i2c)
{
	struct stv6120_state *state;

	state = kzalloc(sizeof (struct stv6120_state), GFP_KERNEL);
	if (!state)
		return NULL;

	state->i2c	= i2c;
	state->config	= config;
	state->devctl	= &stv6120_ctl;
	state->tuner  = config->tuner;

	fe->tuner_priv		= state;
	fe->ops.tuner_ops	= stv6120_ops;

	printk(KERN_INFO "%s: Attaching stv6120, tuner: %d\n", __func__, state->tuner);

	stv6120_set_mode(fe, TUNER_WAKE);
	stv6120_init(fe);

	return state->devctl;
}

EXPORT_SYMBOL(stv6120_attach);

MODULE_AUTHOR("Manu Abraham");
MODULE_DESCRIPTION("stv6120 Silicon tuner");
MODULE_LICENSE("GPL");
