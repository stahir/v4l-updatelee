/* DVB USB compliant Linux driver for the
 *  - GENPIX 8pks/qpsk/DCII USB2.0 DVB-S module
 *
 * Copyright (C) 2006,2007 Alan Nisota (alannisota@gmail.com)
 * Copyright (C) 2006,2007 Genpix Electronics (genpix@genpix-electronics.com)
 *
 * Thanks to GENPIX for the sample code used to implement this module.
 *
 * This module is based off the vp7045 and vp702x modules
 *
 *	This program is free software; you can redistribute it and/or modify it
 *	under the terms of the GNU General Public License as published by the Free
 *	Software Foundation, version 2.
 *
 * see Documentation/dvb/README.dvb-usb for more information
 */
#include "gp8psk.h"

struct gp8psk_fe_state {
	struct dvb_frontend fe;
	struct dvb_usb_device *d;
	u8 lock;
	u16 snr;
	unsigned long next_status_check;
	unsigned long status_check_interval;
};

static int gp8psk_tuned_to_DCII(struct dvb_frontend *fe)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;
	u8 status;
	gp8psk_usb_in_op(st->d, GET_8PSK_CONFIG, 0, 0, &status, 1);
	return status & bmDCtuned;
}

static int gp8psk_set_tuner_mode(struct dvb_frontend *fe, int mode)
{
	struct gp8psk_fe_state *state = fe->demodulator_priv;
	return gp8psk_usb_out_op(state->d, SET_8PSK_CONFIG, mode, 0, NULL, 0);
}

static int gp8psk_fe_update_status(struct gp8psk_fe_state *st)
{
	u8 buf[6];
	if (time_after(jiffies,st->next_status_check)) {
		gp8psk_usb_in_op(st->d, GET_SIGNAL_LOCK, 0,0,&st->lock,1);
		gp8psk_usb_in_op(st->d, GET_SIGNAL_STRENGTH, 0,0,buf,6);

		printk(KERN_INFO "%s: lock:%s\n", __func__, st->lock ? "locked" : "unlocked");

		st->snr = ((buf[1]) << 8 | buf[0]) << 4;
		st->next_status_check = jiffies + (st->status_check_interval*HZ)/1000;
	}
	return 0;
}

static int gp8psk_fe_read_status(struct dvb_frontend *fe,
				 enum fe_status *status)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct gp8psk_fe_state *st = fe->demodulator_priv;

	u8 buf[32];
	int frequency;
	int carrier_error;
	int carrier_offset;
	int rate_error;
	int rate_offset;
	int symbol_rate;

	int fe_gp8psk_system_return[] = {
		SYS_DVBS,
		SYS_TURBO,
		SYS_TURBO,
		SYS_TURBO,
		SYS_DCII,
		SYS_DCII,
		SYS_DCII,
		SYS_DCII,
		SYS_DSS,
		SYS_UNDEFINED
	};

	int fe_gp8psk_modulation_return[] = {
		QPSK,
		QPSK,
		PSK_8,
		QAM_16,
		C_QPSK,
		I_QPSK,
		Q_QPSK,
		C_OQPSK,
		QPSK,
		QPSK,
	};

	printk(KERN_INFO "%s\n", __func__);

	gp8psk_fe_update_status(st);

	if (st->lock)
		*status = FE_HAS_LOCK | FE_HAS_SYNC | FE_HAS_VITERBI | FE_HAS_SIGNAL | FE_HAS_CARRIER;
	else
		*status = 0;

	printk(KERN_INFO "%s: lock: %s\n", __func__, st->lock ? "yes" : "no");

	if (*status & FE_HAS_LOCK) {
		gp8psk_usb_in_op(st->d, GET_SIGNAL_STAT, 0, 0, buf, 32);
		frequency		= ((buf[11] << 24) + (buf[10] << 16) + (buf[9] << 8) + buf[8]) / 1000;
		carrier_error	= ((buf[15] << 24) + (buf[14] << 16) + (buf[13] << 8) + buf[12]) / 1000;
		carrier_offset	=  (buf[19] << 24) + (buf[18] << 16) + (buf[17] << 8) + buf[16];
		rate_error		=  (buf[23] << 24) + (buf[22] << 16) + (buf[21] << 8) + buf[20];
		rate_offset		=  (buf[27] << 24) + (buf[26] << 16) + (buf[25] << 8) + buf[24];
		symbol_rate		=  (buf[31] << 24) + (buf[30] << 16) + (buf[29] << 8) + buf[28];

		c->frequency		= frequency - carrier_error;
		c->symbol_rate		= symbol_rate + rate_error;

		switch (c->delivery_system) {
		case SYS_DSS:
		case SYS_DVBS:
			c->delivery_system	= fe_gp8psk_system_return[buf[1]];
			c->modulation		= fe_gp8psk_modulation_return[buf[1]];
			switch (buf[2]) {
			case 0:  c->fec_inner = FEC_1_2;  break;
			case 1:  c->fec_inner = FEC_2_3;  break;
			case 2:  c->fec_inner = FEC_3_4;  break;
			case 3:  c->fec_inner = FEC_5_6;  break;
			case 4:  c->fec_inner = FEC_6_7;  break;
			case 5:  c->fec_inner = FEC_7_8;  break;
			default: c->fec_inner = FEC_AUTO; break;
			}
			break;
		case SYS_TURBO:
			c->delivery_system	= fe_gp8psk_system_return[buf[1]];
			c->modulation		= fe_gp8psk_modulation_return[buf[1]];
			if (c->modulation == QPSK) {
				switch (buf[2]) {
				case 0:  c->fec_inner = FEC_7_8;  break;
				case 1:  c->fec_inner = FEC_1_2;  break;
				case 2:  c->fec_inner = FEC_3_4;  break;
				case 3:  c->fec_inner = FEC_2_3;  break;
				case 4:  c->fec_inner = FEC_5_6;  break;
				default: c->fec_inner = FEC_AUTO; break;
				}
			} else {
				switch (buf[2]) {
				case 0:  c->fec_inner = FEC_2_3;  break;
				case 1:  c->fec_inner = FEC_3_4;  break;
				case 2:  c->fec_inner = FEC_3_4;  break;
				case 3:  c->fec_inner = FEC_5_6;  break;
				case 4:  c->fec_inner = FEC_8_9;  break;
				default: c->fec_inner = FEC_AUTO; break;
				}
			}
			break;
		case SYS_DCII:
			c->modulation		= fe_gp8psk_modulation_return[buf[1]];
			switch (buf[2]) {
			case 0:  c->fec_inner = FEC_5_11; break;
			case 1:  c->fec_inner = FEC_1_2;  break;
			case 2:  c->fec_inner = FEC_3_5;  break;
			case 3:  c->fec_inner = FEC_2_3;  break;
			case 4:  c->fec_inner = FEC_3_4;  break;
			case 5:  c->fec_inner = FEC_4_5;  break;
			case 6:  c->fec_inner = FEC_5_6;  break;
			case 7:  c->fec_inner = FEC_7_8;  break;
			default: c->fec_inner = FEC_AUTO; break;
			}
			break;
		default:
			c->fec_inner = FEC_AUTO;
			break;
		}

		st->status_check_interval = 1000;
	} else {
		st->status_check_interval = 100;
	}
	return 0;
}

/* not supported by this Frontend */
static int gp8psk_fe_read_ber(struct dvb_frontend* fe, u32 *ber)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;
	u8 buf[4];

	gp8psk_usb_in_op(st->d, GET_BER_RATE, 0, 0, buf, 4);
	*ber = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];

	return 0;
}

/* not supported by this Frontend */
static int gp8psk_fe_read_unc_blocks(struct dvb_frontend* fe, u32 *unc)
{
	(void) fe;
	*unc = 0;
	return 0;
}

static int gp8psk_fe_read_snr(struct dvb_frontend* fe, u16 *snr)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;
	gp8psk_fe_update_status(st);
	/* snr is reported in dBu*256 */
	*snr = st->snr;
	return 0;
}

static int gp8psk_fe_read_signal_strength(struct dvb_frontend* fe, u16 *strength)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;
	gp8psk_fe_update_status(st);
	/* snr is reported in dBu*256 */
	/* snr / 38.4 ~= 100% strength */
	/* snr * 17 returns 100% strength as 65535 */
	if (st->snr > 0xf00)
		*strength = 0xffff;
	else
		*strength = (st->snr << 4) + st->snr; /* snr*17 */
	return 0;
}

static int gp8psk_fe_get_tune_settings(struct dvb_frontend* fe, struct dvb_frontend_tune_settings *tune)
{
	tune->min_delay_ms = 800;
	return 0;
}

static int gp8psk_fe_set_frontend(struct dvb_frontend *fe)
{
	struct gp8psk_fe_state *state = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u8 cmd[10];
	u32 freq = c->frequency * 1000;
	int gp_product_id = le16_to_cpu(state->d->udev->descriptor.idProduct);

	printk(KERN_INFO "%s: freq: %d, sr: %d\n", __func__, freq, c->symbol_rate);

	cmd[0] =  c->symbol_rate        & 0xff;
	cmd[1] = (c->symbol_rate >>  8) & 0xff;
	cmd[2] = (c->symbol_rate >> 16) & 0xff;
	cmd[3] = (c->symbol_rate >> 24) & 0xff;
	cmd[4] = freq         & 0xff;
	cmd[5] = (freq >> 8)  & 0xff;
	cmd[6] = (freq >> 16) & 0xff;
	cmd[7] = (freq >> 24) & 0xff;

	/* backwards compatibility: DVB-S2 used to be used for Turbo-FEC */
	if (c->delivery_system == SYS_DVBS2) {
		c->delivery_system = SYS_TURBO;
	}

	switch (c->delivery_system) {
	case SYS_DVBS:
		printk(KERN_INFO "%s: DVB-S QPSK delivery system selected w/fec %d\n", __func__, c->fec_inner);
		c->fec_inner = FEC_AUTO;
		cmd[8] = ADV_MOD_DVB_QPSK;
		cmd[9] = 5;
		break;
	case SYS_TURBO:
		printk(KERN_INFO "%s: Turbo-FEC delivery system selected\n", __func__);
		switch (c->modulation) {
		case QPSK:
			printk(KERN_INFO "%s: modulation QPSK selected w/fec %d\n", __func__, c->fec_inner);
			cmd[8] = ADV_MOD_TURBO_QPSK;
			switch (c->fec_inner) {
			case FEC_1_2:	cmd[9] = 1; break;
			case FEC_2_3:	cmd[9] = 3; break;
			case FEC_3_4:	cmd[9] = 2; break;
			case FEC_5_6:	cmd[9] = 4; break;
			default:	cmd[9] = 0; break;
			}
			break;
		case PSK_8:
			printk(KERN_INFO "%s: modulation 8PSK selected w/fec %d\n", __func__, c->fec_inner);
			cmd[8] = ADV_MOD_TURBO_8PSK;
			switch (c->fec_inner) {
			case FEC_2_3:	cmd[9] = 0; break;
			case FEC_3_4:	cmd[9] = 1; break;
			case FEC_3_5:	cmd[9] = 2; break;
			case FEC_5_6:	cmd[9] = 3; break;
			case FEC_8_9:	cmd[9] = 4; break;
			default:	cmd[9] = 0; break;
			}
			break;
		case QAM_16: /* QAM_16 is for compatibility with DN */
			printk(KERN_INFO "%s: modulation QAM_16 selected w/fec %d\n", __func__, c->fec_inner);
			cmd[8] = ADV_MOD_TURBO_16QAM;
			cmd[9] = 0;
			break;
		default: /* Unknown modulation */
			printk(KERN_INFO "%s: unsupported modulation selected (%d)\n", __func__, c->modulation);
			return -EOPNOTSUPP;
		}
		break;
	case SYS_DSS:
		printk(KERN_INFO "%s: DSS delivery system selected w/fec %d\n", __func__, c->fec_inner);
		cmd[8] = ADV_MOD_DSS_QPSK;
		switch (c->fec_inner) {
		case FEC_1_2:	cmd[9] = 0; break;
		case FEC_2_3:	cmd[9] = 1; break;
		case FEC_3_4:	cmd[9] = 2; break;
		case FEC_5_6:	cmd[9] = 3; break;
		case FEC_7_8:	cmd[9] = 4; break;
		case FEC_AUTO:	cmd[9] = 5; break;
		case FEC_6_7:	cmd[9] = 6; break;
		default:	cmd[9] = 5; break;
		}
		break;
	case SYS_DCII:
		printk(KERN_INFO "%s: DCII delivery system selected w/fec %d\n", __func__, c->fec_inner);
		switch (c->modulation) {
		case C_QPSK:
			cmd[8] = ADV_MOD_DCII_C_QPSK;
			break;
		case I_QPSK:
			cmd[8] = ADV_MOD_DCII_I_QPSK;
			break;
		case Q_QPSK:
			cmd[8] = ADV_MOD_DCII_Q_QPSK;
			break;
		case C_OQPSK:
		default:
			cmd[8] = ADV_MOD_DCII_C_OQPSK;
			break;
		}		
		switch (c->fec_inner) {
		case FEC_5_11: cmd[9] = 0; break;
		case FEC_1_2:  cmd[9] = 1; break;
		case FEC_3_5:  cmd[9] = 2; break;
		case FEC_2_3:  cmd[9] = 3; break;
		case FEC_3_4:  cmd[9] = 4; break;
		case FEC_4_5:  cmd[9] = 5; break;
		case FEC_5_6:  cmd[9] = 6; break;
		case FEC_7_8:  cmd[9] = 7; break;
		case FEC_AUTO: cmd[9] = 8; break;
		default:       cmd[9] = 8; break;
		}
		break;
	default:
		printk(KERN_INFO "%s: unsupported delivery system selected (%d)\n", __func__, c->delivery_system);
		return -EOPNOTSUPP;
	}

	if (gp_product_id == USB_PID_GENPIX_8PSK_REV_1_WARM) {
	    if (gp8psk_tuned_to_DCII(fe)) {
		gp8psk_bcm4500_reload(state->d);
	    }
	    gp8psk_set_tuner_mode(fe, 0);
	}
	gp8psk_usb_out_op(state->d, TUNE_8PSK, 0, 0, cmd, 10);

	state->lock = 0;
	state->next_status_check = jiffies;
	state->status_check_interval = 200;

	return 0;
}

static int gp8psk_fe_send_diseqc_msg (struct dvb_frontend* fe,
				    struct dvb_diseqc_master_cmd *m)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;

	printk(KERN_INFO "%s\n", __func__);

	if (gp8psk_usb_out_op(st->d,SEND_DISEQC_COMMAND, m->msg[0], 0,
			m->msg, m->msg_len)) {
		return -EINVAL;
	}
	return 0;
}

static int gp8psk_fe_send_diseqc_burst(struct dvb_frontend *fe,
				       enum fe_sec_mini_cmd burst)
{
	struct gp8psk_fe_state *st = fe->demodulator_priv;
	u8 cmd;

	printk(KERN_INFO "%s\n", __func__);

	/* These commands are certainly wrong */
	cmd = (burst == SEC_MINI_A) ? 0x00 : 0x01;

	if (gp8psk_usb_out_op(st->d,SEND_DISEQC_COMMAND, cmd, 0,
			&cmd, 0)) {
		return -EINVAL;
	}
	return 0;
}

static int gp8psk_fe_set_tone(struct dvb_frontend *fe,
			      enum fe_sec_tone_mode tone)
{
	struct gp8psk_fe_state* state = fe->demodulator_priv;

	printk(KERN_INFO "%s: tone:%s\n", __func__, tone == SEC_TONE_ON ? "ON" : "OFF");

	if (gp8psk_usb_out_op(state->d,SET_22KHZ_TONE,
		 (tone == SEC_TONE_ON), 0, NULL, 0)) {
		return -EINVAL;
	}
	return 0;
}

static int gp8psk_fe_set_voltage(struct dvb_frontend *fe,
				 enum fe_sec_voltage voltage)
{
	struct gp8psk_fe_state* state = fe->demodulator_priv;

	printk(KERN_INFO "%s: voltage:%s\n", __func__, voltage == SEC_VOLTAGE_18 ? "18v" : "13v");

	if (gp8psk_usb_out_op(state->d,SET_LNB_VOLTAGE,
			 voltage == SEC_VOLTAGE_18, 0, NULL, 0)) {
		return -EINVAL;
	}
	return 0;
}

static int gp8psk_fe_enable_high_lnb_voltage(struct dvb_frontend* fe, long onoff)
{
	struct gp8psk_fe_state* state = fe->demodulator_priv;
	return gp8psk_usb_out_op(state->d, USE_EXTRA_VOLT, onoff, 0,NULL,0);
}

static int gp8psk_fe_send_legacy_dish_cmd (struct dvb_frontend* fe, unsigned long sw_cmd)
{
	struct gp8psk_fe_state* state = fe->demodulator_priv;
	u8 cmd = sw_cmd & 0x7f;

	if (gp8psk_usb_out_op(state->d,SET_DN_SWITCH, cmd, 0,
			NULL, 0)) {
		return -EINVAL;
	}
	if (gp8psk_usb_out_op(state->d,SET_LNB_VOLTAGE, !!(sw_cmd & 0x80),
			0, NULL, 0)) {
		return -EINVAL;
	}

	return 0;
}

static void gp8psk_fe_release(struct dvb_frontend* fe)
{
	struct gp8psk_fe_state *state = fe->demodulator_priv;
	kfree(state);
}

static struct dvb_frontend_ops gp8psk_fe_ops;

struct dvb_frontend * gp8psk_fe_attach(struct dvb_usb_device *d)
{
	struct gp8psk_fe_state *s = kzalloc(sizeof(struct gp8psk_fe_state), GFP_KERNEL);
	if (s == NULL)
		goto error;

	s->d = d;
	memcpy(&s->fe.ops, &gp8psk_fe_ops, sizeof(struct dvb_frontend_ops));
	s->fe.demodulator_priv = s;

	goto success;
error:
	return NULL;
success:
	return &s->fe;
}

static int gp8psk_fe_get_spectrum_scan(struct dvb_frontend *fe, struct dvb_fe_spectrum_scan *s)
{
	struct gp8psk_fe_state* state = fe->demodulator_priv;

	int x;
	u8 cmd[10];
	u8 buf[2];
	unsigned short power;
	unsigned int freq = 950000000;
	unsigned int sr = 1000000;

	printk(KERN_INFO "%s\n", __func__);

	cmd[0] =  sr        & 0xff;
	cmd[1] = (sr >>  8) & 0xff;
	cmd[2] = (sr >> 16) & 0xff;
	cmd[3] = (sr >> 24) & 0xff;
	cmd[4] = freq         & 0xff;
	cmd[5] = (freq >> 8)  & 0xff;
	cmd[6] = (freq >> 16) & 0xff;
	cmd[7] = (freq >> 24) & 0xff;
	cmd[8] = ADV_MOD_DVB_QPSK;
	cmd[9] = 5;
	if (gp8psk_usb_out_op(state->d, TUNE_8PSK, 0, 0, cmd, 10)) {
		printk(KERN_INFO "%s: TUNE_8PSK error\n", __func__);
		return -EINVAL;
	}
	msleep(1000);

	for (x = 0 ; x < s->num_freq ; x++)
	{
		freq = *(s->freq + x) * 1000;
		cmd[0] = freq         & 0xff;
		cmd[1] = (freq >> 8)  & 0xff;
		cmd[2] = (freq >> 16) & 0xff;
		cmd[3] = (freq >> 24) & 0xff;

		if (gp8psk_usb_out_op(state->d, SET_FREQUENCY, 0, 0, cmd, 4)) {
			printk(KERN_INFO "%s: SET_FREQUENCY %d error\n", __func__, freq);
			return -EINVAL;
		}

		msleep(30);

		if (gp8psk_usb_in_op(state->d, GET_SIGNAL_POWER, 0, 0, buf, 2)) {
			printk(KERN_INFO "%s: GET_SIGNAL_POWER %x %x error\n", __func__, buf[0], buf[1]);
			return -EINVAL;
		}
		power = (buf[1] << 8) + buf[0];
		*(s->rf_level + x) = 0x7FFF - power;
	}

	return 0;
}

static struct dvb_frontend_ops gp8psk_fe_ops = {
	.delsys = { SYS_TURBO, SYS_DCII, SYS_DSS, SYS_DVBS },
	.info = {
		.name			= "Genpix DVB-S",
		.frequency_min		= 800000,
		.frequency_max		= 2250000,
		.frequency_stepsize	= 100,
		.symbol_rate_min        = 1000000,
		.symbol_rate_max        = 45000000,
		.symbol_rate_tolerance  = 500,  /* ppm */
		.caps = FE_CAN_INVERSION_AUTO | FE_CAN_SPECTRUMSCAN |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			/*
			 * FE_CAN_QAM_16 is for compatibility
			 * (Myth incorrectly detects Turbo-QPSK as plain QAM-16)
			 */
			FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_TURBO_FEC
	},

	.release = gp8psk_fe_release,

	.init = NULL,
	.sleep = NULL,

	.set_frontend = gp8psk_fe_set_frontend,

	.get_tune_settings = gp8psk_fe_get_tune_settings,

	.read_status = gp8psk_fe_read_status,
	.read_ber = gp8psk_fe_read_ber,
	.read_signal_strength = gp8psk_fe_read_signal_strength,
	.read_snr = gp8psk_fe_read_snr,
	.read_ucblocks = gp8psk_fe_read_unc_blocks,

	.diseqc_send_master_cmd = gp8psk_fe_send_diseqc_msg,
	.diseqc_send_burst = gp8psk_fe_send_diseqc_burst,
	.set_tone = gp8psk_fe_set_tone,
	.set_voltage = gp8psk_fe_set_voltage,
	.dishnetwork_send_legacy_command = gp8psk_fe_send_legacy_dish_cmd,
	.enable_high_lnb_voltage = gp8psk_fe_enable_high_lnb_voltage,
	.get_spectrum_scan		= gp8psk_fe_get_spectrum_scan,
};
