/*
 * cgminer SPI driver for Bitmine.ch T3 devices
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"
#include "crc.h"

#include "T3-common.h"
#include "driver-btm-DASH.h"

struct spi_config cfg[ASIC_CHAIN_NUM];
struct spi_ctx *spi[ASIC_CHAIN_NUM];
struct T3_chain *chain[ASIC_CHAIN_NUM];

#define DISABLE_CHIP_FAIL_THRESHOLD	3
#define LEAST_CORE_ONE_CHAIN	400
#define RESET_CHAIN_CNT	2


static uint8_t A1Pll1=0;  //120MHz
static uint8_t A1Pll2=0;  //120MHz
static uint8_t A1Pll3=0;  //120MHz
static uint8_t A1Pll4=0;  //120MHz
static uint8_t A1Pll5=0;  //120MHz
static uint8_t A1Pll6=0;  //120MHz
static int T2spi[10];

int opt_diff=15;

static const uint8_t difficult_Tbl[24][8] = {
	{0x1e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 1
	{0x1e, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 2
	{0x1e, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 4
	{0x1e, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 8
	{0x1e, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 16
	{0x1e, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 32
	{0x1e, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 64
	                   //**
	{0x1e, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff},	// 128
	{0x1e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},	// 256
	{0x1e, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 512
	{0x1e, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 1024
	{0x1e, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 2048
	{0x1e, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff},	// 4096
	{0x1e, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff},	// 8192
	{0x1e, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff},	// 16384
	{0x1e, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff},	// 32768
	{0x1e, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff}	// 65536
};

const unsigned short wCRCTalbeAbss[] ={
0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00,
0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400,};

struct device_drv bitmineT3_drv;

unsigned short CRC16_2(unsigned char* pchMsg, unsigned short wDataLen)
{
	volatile unsigned short wCRC = 0xFFFF;
	unsigned short i;
	unsigned char chChar;

	for (i = 0; i < wDataLen; i++)
	{
		chChar = *pchMsg++;
		wCRC = wCRCTalbeAbss[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);
		wCRC = wCRCTalbeAbss[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
	}

	return wCRC;
}

unsigned short crc16(unsigned char* pchMsg, unsigned short wDataLen)
{
	unsigned short wCRC = 0xFFFF;
	unsigned short i;
	unsigned char chChar;
	unsigned short len = wDataLen / 2;
	unsigned char *tmp;
	unsigned short src[32];

	memset(src, 0, sizeof(src));
	for(i = 0; i < len; i++)
	{
		src[i] = ((((unsigned short)pchMsg[2* i]) << 8) & 0xff00) | ((unsigned short)pchMsg[2*i + 1] & 0x00ff);
	}
	tmp = (unsigned char*)src;
	for (i = 0; i < wDataLen; i++)
	{
		chChar = tmp[i];
		wCRC = wCRCTalbeAbss[(chChar ^ wCRC) & 15] ^ (wCRC >> 4); 
		wCRC = wCRCTalbeAbss[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4); 
	}
	tmp = NULL;
	return wCRC;
}

int T3_ConfigT3PLLClock(int optPll)
{
	int i;
	int T3Pll;
	int PLL = optPll;

	if(PLL > 0)
	{
		T3Pll=0;
		if(PLL <= 120)
		{
			T3Pll = 0; //found
		}
		else
		{
			for(i = 1; i < T3_PLL_CLOCK_1400MHz; i++)
			{
				if((PLL < T3_REG_TO_CLOCK((i + 1))) && (PLL >= T3_REG_TO_CLOCK(i)))
				{
					T3Pll=i; //found
					break;
				}
			}
		}
	} else {
		T3Pll = 0;
	}

	return T3Pll;
}

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);
	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}


static void applog_hexdumpd(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_ERR, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdumpd(char *prefix, uint8_t *buff, int len)
{
	applog_hexdumpd(prefix, buff, len, LOG_ERR);
}

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

static void flush_spi(struct T3_chain *t3)
{
	memset(t3->spi_tx, 0, 64);
	spi_transfer(t3->spi_ctx, t3->spi_tx, t3->spi_rx, 64);
}


/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct T3_chain *t3,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t len,
			  uint8_t resp_len)
{
	int tx_len = 4 + len;
	memset(t3->spi_tx, 0, tx_len);
	t3->spi_tx[0] = cmd;
	t3->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(t3->spi_tx + 2, data, len);

	assert(spi_transfer(t3->spi_ctx, t3->spi_tx, t3->spi_rx, tx_len));
	hexdump("send: TX", t3->spi_tx, tx_len);
	hexdump("send: RX", t3->spi_rx, tx_len);

	int poll_len = resp_len;
	if (chip_id == 0) {
		if (t3->num_chips == 0) {
			applog(LOG_INFO, "%d: unknown chips in chain, "
			       "assuming 8", t3->chain_id);
			poll_len += 32;
		}
		poll_len += 4 * t3->num_chips;
	}
	else {
		poll_len += 4 * chip_id - 2;
	}
	assert(spi_transfer(t3->spi_ctx, NULL, t3->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", t3->spi_rx + tx_len, poll_len);
	int ack_len = tx_len + resp_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", t3->spi_rx + ack_pos, ack_len - 2);

	return (t3->spi_rx + ack_pos);

}


/********** T3 SPI commands */
static uint8_t *cmd_BIST_FIX_BCAST(struct T3_chain *t3)
{
	uint8_t *ret = exec_cmd(t3, T3_BIST_FIX, 0x00, NULL, 0, 0);
	if (ret == NULL || ret[0] != T3_BIST_FIX) {
		applog(LOG_ERR, "%d: cmd_BIST_FIX_BCAST failed", t3->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_BIST_FIX_NN(struct T3_chain *t3, int chip)
{
	uint8_t *ret = exec_cmd(t3, T3_BIST_FIX, chip, NULL, 0, 0);
	if (ret == NULL || ret[0] != T3_BIST_FIX) {
		applog(LOG_ERR, "%d: cmd_BIST_FIX_BCAST failed", t3->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_BIST_COLLECT_BCAST(struct T3_chain *t3)
{
	uint8_t *ret = exec_cmd(t3, T3_BIST_COLLECT, 0x00, NULL, 0, 0);
	if (ret == NULL || ret[0] != T3_BIST_COLLECT) {
		applog(LOG_ERR, "%d: cmd_BIST_COLLECT_BCAST failed", t3->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_RESET_BCAST(struct T3_chain *t3, uint8_t strategy)
{
	static uint8_t s[2];
	s[0] = strategy;
	s[1] = strategy;
	uint8_t *ret = exec_cmd(t3, T3_RESET, 0x00, s, 2, 0);
	if (ret == NULL || (ret[0] != T3_RESET_RES && t3->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", t3->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_RESET_IDUAL(struct T3_chain *t3, uint8_t chip_id)
{
	static uint8_t s[2];
	s[0] = 0xe5;
	s[1] = 0xe5;
	uint8_t *ret = exec_cmd(t3, T3_RESET, chip_id, s, 2, 0);
	if (ret == NULL || (ret[0] != T3_RESET && ret[1] != chip_id)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", t3->chain_id);
		return NULL;
	}
	return ret;
}

static uint8_t *cmd_READ_RESULT_BCAST(struct T3_chain *t3)
{
	int tx_len = 10;
	uint16_t clc_crc;
	uint16_t res_crc;
	memset(t3->spi_tx, 0, tx_len);
	t3->spi_tx[0] = T3_READ_RESULT;

	assert(spi_transfer(t3->spi_ctx, t3->spi_tx, t3->spi_rx, tx_len));
//	hexdump("send: TX", t3->spi_tx, tx_len);
//	hexdump("send: RX", t3->spi_rx, tx_len);

	int poll_len = tx_len + 4 * t3->num_chips;
	assert(spi_transfer(t3->spi_ctx, NULL, t3->spi_rx + tx_len, poll_len));
//	hexdump("poll: RX", t3->spi_rx + tx_len, poll_len);

	uint8_t *scan = t3->spi_rx;

	int i;
	for (i = 0; i < poll_len; i += 2) {
		if ((scan[i] & 0x0f) == T3_READ_RESULT && (scan[i] & 0xf0) != 0) {
			res_crc = (scan[i + ASIC_RESULT_LEN] << 8) + (scan[i + ASIC_RESULT_LEN+1] << 0);
			clc_crc = crc16(scan + i, ASIC_RESULT_LEN);
				if(clc_crc == res_crc)
					return scan + i;
		}
	}
	
	return NULL;
}

static uint8_t *cmd_WRITE_REG(struct T3_chain *t3, uint8_t chip, uint8_t *reg)
{	
	uint8_t buffer[14];
	uint8_t reg_crc[14];

	buffer[0] = 0x09;
	buffer[1] = chip;
	memcpy(buffer + 2, reg, 12);

	unsigned short crc = crc16(buffer, 14);

	memcpy(reg_crc, reg, 12);
	reg_crc[12] = (crc >> 8) & 0xff;
	reg_crc[13] = crc & 0xff;
	
	uint8_t *ret = exec_cmd(t3, T3_WRITE_REG, chip, reg_crc, 14, 0);
	if (ret == NULL || ret[0] != T3_WRITE_REG) {
		applog(LOG_ERR, "%d: cmd_WRITE_REG failed", t3->chain_id);
		return NULL;
	}

	return ret;
}

static uint8_t *cmd_READ_REG(struct T3_chain *t3, uint8_t chip)
{
	uint8_t *ret = exec_cmd(t3, T3_READ_REG, chip, NULL, 14, 18);
	unsigned short crc = ((((unsigned short)ret[14]) << 8) & 0xff00) | ((unsigned short)ret[15] & 0x00ff);
	if (ret == NULL || ret[0] != T3_READ_REG_RESP || ret[1] != chip || crc != crc16(ret, 14)) {
		applog(LOG_ERR, "%d: cmd_READ_REG chip %d failed",
		       t3->chain_id, chip);
		return NULL;
	}
	memcpy(t3->spi_rx, ret, 16);
	return ret;
}

uint8_t cmd_CHECK_BUSY(struct T3_chain *t3, uint8_t chip_id)
{
	//printf("[check busy] \r\n");
	
	if(cmd_READ_REG(t3, chip_id) && (t3->spi_rx[11] & 0x01) == 1)
	{
		//applog(LOG_WARNING, "chip %d is busy now", chip_id);
		return WORK_BUSY;
	}
	//applog(LOG_WARNING, "chip %d is free now", chip_id);
	return WORK_FREE;
}

static bool cmd_WRITE_JOB(struct T3_chain *t3, uint8_t chip_id,
			      uint8_t *job)
{
	/* ensure we push the SPI command to the last chip in chain */
	int tx_len = JOB_LENGTH + 2;
	memcpy(t3->spi_tx, job, JOB_LENGTH);
	memset(t3->spi_tx + JOB_LENGTH, 0, tx_len - JOB_LENGTH);

	assert(spi_transfer(t3->spi_ctx, t3->spi_tx, t3->spi_rx, tx_len));

	int poll_len = 4 * chip_id - 2;

	assert(spi_transfer(t3->spi_ctx, NULL, t3->spi_rx + tx_len, poll_len));

//	int ack_len = tx_len;
//	int ack_pos = tx_len + poll_len - ack_len;
//	hexdump("poll: ACK", t3->spi_rx + ack_pos, tx_len);

	printf("[write job] \r\n");
	hexdumpd("job:", t3->spi_tx, JOB_LENGTH);

	cgsleep_us(100000);

	if(cmd_CHECK_BUSY(t3, chip_id) != WORK_BUSY)
	{
		return false;
	}

	return true;

}

/********** T3 low level functions */
#define MAX_PLL_WAIT_CYCLES 25
#define PLL_CYCLE_WAIT_TIME 40
static bool check_chip_pll_lock(struct T3_chain *t3, int chip_id, uint8_t *wr)
{
	int n;
	for (n = 0; n < MAX_PLL_WAIT_CYCLES; n++) {
		/* check for PLL lock status */

		if (cmd_READ_REG(t3, chip_id) && (t3->spi_rx[4] & 0x01) == 1) {
			/* double check that we read back what we set before */

			return wr[0] == t3->spi_rx[2] && wr[1] == t3->spi_rx[3] && wr[3] == t3->spi_rx[5];
		}
	}
	applog(LOG_ERR, "%d: chip %d failed PLL lock", t3->chain_id, chip_id);
	return false;
}


static int chain_chips(struct T3_chain *t3)
{
	int tx_len = 6;
	int rx_len = 4;
	int cid = t3->chain_id;

	memset(t3->spi_tx, 0, tx_len);
	memset(t3->spi_rx, 0, rx_len); 
	t3->spi_tx[0] = T3_BIST_START;
	t3->spi_tx[1] = 0;

	struct spi_ctx *spi = t3->spi_ctx;

	if (!spi_transfer(t3->spi_ctx, t3->spi_tx, t3->spi_rx, 6))
		return 0;
	hexdump("TX", t3->spi_tx, 6);
	hexdump("RX", t3->spi_rx, 6);

	int i;
	int max_poll_words = MAX_CHAIN_LENGTH * 2;

	for(i = 0; i < max_poll_words; i++) {
		if (t3->spi_rx[0] == T3_BIST_START && t3->spi_rx[1] == 0) {
			spi_transfer(t3->spi_ctx, NULL, t3->spi_rx, 2);
			hexdump("TX", t3->spi_tx, 2);
			uint8_t n = t3->spi_rx[1];
			t3->num_chips = (i / 2) + 1;
			if(t3->num_chips != n) {
				applog(LOG_ERR, "%d: enumeration: %d <-> %d",cid, t3->num_chips, n);
				if(n != 0)
					t3->num_active_chips = n;
			}
			applog(LOG_WARNING, "%d: detected %d chips", cid, t3->num_chips);
			return t3->num_chips;
		}
		bool s = spi_transfer(t3->spi_ctx, NULL, t3->spi_rx, 2);
		hexdump("RX", t3->spi_rx, 2);
		if(!s)
			return 0;
	}
	applog(LOG_WARNING, "%d: no T3 chip-chain detected", cid);
	return 0;
}

static bool set_pll_config(struct T3_chain *t3, int idxpll)
{
	uint8_t pll_init[6][12] =
	{
		//pll_postdiv = 3(8), pll_fbdiv = 80, pll_prediv = 1, spi = 1(64)
		{0x02, 0x50, 0x40, 0xc2, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //120MHz, vco=960MHz

		//pll_postdiv = 2(4), pll_fbdiv = 56, pll_prediv = 1, spi = 1(64)
		{0x02, 0x38, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //168MHz, vco=672MHz

		//pll_postdiv = 1(2), pll_fbdiv = 54, pll_prediv = 1, spi = 1(128)
		{0x02, 0x36, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //324MHz, vco=648MHz

		//pll_postdiv = 0(1), pll_fbdiv = 53, pll_prediv = 1, spi = 0(128)
		{0x02, 0x36, 0x40, 0x02, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //636MHz, vco=636MHz
	};
	int index = 0;
	int clock = 0;
	int vco = 0;
	uint8_t temp_reg[REG_LENGTH];
	int i;

	for(i = 0; i < 1; i++)//max vco=1296, pll=162
	{
		vco = pll_init[0][1] * 12;
		clock = vco / 8;

		memcpy(temp_reg, pll_init[0], REG_LENGTH-2);
		applog(LOG_WARNING, "setting PLL:"
	       "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
	       temp_reg[0], temp_reg[1], temp_reg[2],
	       temp_reg[3], temp_reg[4], temp_reg[5],
	       temp_reg[6], temp_reg[7], temp_reg[8],
	       temp_reg[9], temp_reg[10], temp_reg[11]);
	    if (!cmd_WRITE_REG(t3, ADDR_BROADCAST, temp_reg))
		{
			applog(LOG_WARNING, "set PLL %d MHz fail vco %d MHz", clock, vco);
			return false;
		}
		applog(LOG_WARNING, "set PLL %d MHz success vco %d MHz", clock, vco);

		cgsleep_us(200000);
	}

	int from = 0;
	int to = t3->num_chips;

	for (i = from; i < to; i++) {
		int cid = i + 1;
		if (!check_chip_pll_lock(t3, cid, temp_reg)) {
			applog(LOG_ERR, "%d: chip %d failed PLL lock",
			       t3->chain_id, cid);
			//return false;
		}
	}
	return true;
}
static bool check_chip(struct T3_chain *t3, int i)
{
	int chip_id = i + 1;
	int cid = t3->chain_id;
	if (!cmd_READ_REG(t3, chip_id)) {
		applog(LOG_WARNING, "%d: Failed to read register for "
		       "chip %d -> disabling", cid, chip_id);
		t3->chips[i].num_cores = 0;
		t3->chips[i].disabled = 1;
		return false;;
	}
	t3->chips[i].num_cores = t3->spi_rx[13];
	t3->num_cores += t3->chips[i].num_cores;
	applog(LOG_WARNING, "%d: Found chip %d with %d active cores",
	       cid, chip_id, t3->chips[i].num_cores);
	return true;
}

/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct T3_chain *t3, uint8_t chip_id)
{
	struct T3_chip *chip = &t3->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}


int chain_t3_detect(struct T3_chain *t3, int idxpll)
{
	int i;
	int cid = t3->chain_id;
	int spi_clk_khz = 100000;
//	set_spi_speed(t3->spi_ctx, spi_clk_khz);

	if(!cmd_RESET_BCAST(t3, 0x00))
		applog(LOG_WARNING, "cmd_RESET_BCAST fail");
	applog(LOG_WARNING, "reset chip success");

	cgsleep_us(1000);
	t3->num_chips = chain_chips(t3);
	if (t3->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d %d: Found %d T3 chips",
   		t3->spi_ctx->config.bus, t3->spi_ctx->config.cs_line,
   		t3->chain_id, t3->num_chips);

	cgsleep_us(10000);
	if (!set_pll_config(t3, idxpll))
		goto failure;

	spi_clk_khz = 2000000;
//	set_spi_speed(t3->spi_ctx, spi_clk_khz);
	cgsleep_us(1000);

	t3->num_active_chips = t3->num_chips;
	t3->chips = calloc(t3->num_active_chips, sizeof(struct T3_chip));
	assert(t3->chips != NULL);

	cgsleep_us(10000);
	if(!cmd_BIST_COLLECT_BCAST(t3))
		goto failure;
	applog(LOG_WARNING, "collect core success");

	cgsleep_us(1000);
	if (!cmd_BIST_FIX_BCAST(t3))
		goto failure;
		
	applog(LOG_WARNING, "bist fix success");
		
	for (i = 0; i < t3->num_active_chips; i++)
	{
		check_chip(t3, i);
	}
	return t3->num_chips;
failure:
	return -1;
}

/* check and disable chip, remember time */
static void disable_chip(struct T3_chain *t3, uint8_t chip_id)
{
	flush_spi(t3);
	struct T3_chip *chip = &t3->chips[chip_id - 1];
	int cid = t3->chain_id;
	if (is_chip_disabled(t3, chip_id)) {
		applog(LOG_WARNING, "%d: chip %d already disabled",
		       cid, chip_id);
		return;
	}
	applog(LOG_WARNING, "%d: temporary disabling chip %d", cid, chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct T3_chain *t3, int pllnum)
{
	int i;
	int cid = t3->chain_id;
	uint8_t reg[REG_LENGTH];
	struct spi_ctx *ctx = t3->spi_ctx;

	for (i = 0; i < t3->num_active_chips; i++) 
	{
		int chip_id = i + 1;
		struct T3_chip *chip = &t3->chips[i];
		if (!is_chip_disabled(t3, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;

		//if the core in chain least than 432, reinit this chain
		if(t3->num_cores <= LEAST_CORE_ONE_CHAIN && chip->fail_reset < RESET_CHAIN_CNT)
		{

#if 0
			chip->fail_reset++;
			asic_gpio_write(ctx->reset, 0);
			cgsleep_us(500000);
			asic_gpio_write(ctx->reset, 1); 
#else
			chip->fail_reset++;
			system("echo 1 > /sys/class/gpio/gpio114/value");
			cgsleep_us(500000);
			system("echo 0 > /sys/class/gpio/gpio114/value");
#endif
			t3->num_chips = chain_t3_detect(t3, pllnum);
		
		}
		
		if (!cmd_READ_REG(t3, chip_id)) 
		{
			chip->fail_count++;
			applog(LOG_WARNING, "%d: chip %d not yet working - %d",
				   cid, chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) 
			{
				applog(LOG_WARNING, "%d: completely disabling chip %d at %d",
					   cid, chip_id, chip->fail_count);
				chip->disabled = true;
				t3->num_cores -= chip->num_cores;	
				
				continue;
			}
			/* restart cooldown period */
			chip->cooldown_begin = get_current_ms();
			continue;
		}
		applog(LOG_WARNING, "%d: chip %d is working again", cid, chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
		chip->fail_reset = 0;
	}
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	unsigned char *wdata = work->data;
	uint8_t data[128];
	double sdiff = work->sdiff;
	uint8_t tmp_buf[JOB_LENGTH];
	uint16_t crc;
	uint8_t i;

	memset(data, 0, 128);

    for(int j=0; j<20; j++)
    {
        data[j*4 + 3] = work->data[j*4 + 0];
        data[j*4 + 2] = work->data[j*4 + 1];
        data[j*4 + 1] = work->data[j*4 + 2];
        data[j*4 + 0] = work->data[j*4 + 3];
    }
    wdata = data;

	static uint8_t job[JOB_LENGTH] = {
		/* command */
		0x00, 0x00,
		/* wdata 75 to 0 */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* end nonce */
		0x00, 0x00, 0x00, 0x00,
//		/* crc data */
		0x00, 0x00
	};

	uint8_t diffIdx;
	uint8_t data75to0[76];	
	uint8_t startnonce[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t diff[8] = {0x1e, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff};
	uint8_t endnonce[4] = {0x00, 0x40, 0x00, 0x00}; // 10s

	memcpy(data75to0, wdata, 76);

	if(sdiff > 65535.0)
		memcpy(diff, difficult_Tbl[16], 8);
	else if(sdiff > 32767.0)
			memcpy(diff, difficult_Tbl[15], 8);
	else if(sdiff > 16383.0)
			memcpy(diff, difficult_Tbl[14], 8);
	else if(sdiff > 8191.0)
			memcpy(diff, difficult_Tbl[13], 8);
	else if(sdiff > 4095.0)
				memcpy(diff, difficult_Tbl[12], 8);
	else if(sdiff > 2047.0)
		memcpy(diff, difficult_Tbl[11], 8);
	else if(sdiff > 1023.0)
		memcpy(diff, difficult_Tbl[10], 8);
	else if(sdiff > 511.0)
		memcpy(diff, difficult_Tbl[9], 8);
	else if(sdiff > 255.0)
		memcpy(diff, difficult_Tbl[8], 8);
	else {
		if(opt_diff>=1&&opt_diff<=8)
		{
			diffIdx=opt_diff-1;
			memcpy(diff, difficult_Tbl[diffIdx], 8);
		}
		else
		{
			printf("dongfupang %s() %d\n", __func__, __LINE__);
			memcpy(diff, difficult_Tbl[7], 8);
		}
	}
	
	startnonce[0]=0x00;
	startnonce[1]=0x00;
	startnonce[2]=0x00;
	startnonce[3]=0x00;
	
	endnonce[0]=0xff;
	endnonce[1]=0xff;
	endnonce[2]=0xff;
	endnonce[3]=0xff;

	rev(data75to0, 76);
	rev(startnonce, 4);
	rev(diff, 8);
	rev(endnonce, 4);

	job[0] = (job_id << 4) | T3_WRITE_JOB;
	job[1] = chip_id;
	memcpy(job+2,			data75to0,	76);
	memcpy(job+2+76,		startnonce, 4);
	memcpy(job+2+76+4,	    diff, 8);
	memcpy(job+2+76+4+8, 	endnonce, 4);

	/* crc */
	memset(tmp_buf, 0, sizeof(tmp_buf));
	for(i = 0; i < 47; i++)
	{
		tmp_buf[(2 * i) + 1] = job[(2 * i) + 0];
		tmp_buf[(2 * i) + 0] = job[(2 * i) + 1];
	}
	crc = CRC16_2(tmp_buf, 94);
	job[94] = (uint8_t)((crc >> 8) & 0xff);
	job[95] = (uint8_t)((crc >> 0) & 0xff);


	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct T3_chain *t3, uint8_t chip_id, struct work *work,
		     uint8_t queue_states)
{
	int cid = t3->chain_id;
	struct T3_chip *chip = &t3->chips[chip_id - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;

	uint8_t *jobdata = create_job(chip_id, job_id, work);
	if (!cmd_WRITE_JOB(t3, chip_id, jobdata)) {
		/* give back work */
		work_completed(t3->cgpu, work);
		applog(LOG_ERR, "%d: failed to set work for chip %d.%d",
		       cid, chip_id, job_id);
		disable_chip(t3, chip_id);
	} else {
		chip->work[chip->last_queued_id] = work;
		chip->last_queued_id++;
		chip->last_queued_id &= 3;
	}
	return retval;
}

static bool get_nonce(struct T3_chain *t3, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_READ_RESULT_BCAST(t3);
	if (ret == NULL)
		return false;

	*job_id = ret[0] >> 4;
	*chip = ret[1];
	memcpy(nonce, ret + 2, 4);
	return true;
}

/* reset input work queues in chip chain */
static bool abort_work(struct T3_chain *t3)
{
	/* drop jobs already queued and result queue: reset strategy (0xeded & 0xf7f7) */
	//applog(LOG_INFO,"Start to reset ");
//return cmd_RESET_BCAST(t3, 0xe5);
	return true;
}

void exit_t3_chain(struct T3_chain *t3)
{
	if (t3 == NULL)
		return;
	free(t3->chips);
	t3->chips = NULL;
	t3->spi_ctx = NULL;
	free(t3);
}

void exit_T3_chain(struct T3_chain *t3)
{
	if (t3 == NULL)
		return;
	free(t3->chips);
	t3->chips = NULL;
	t3->spi_ctx = NULL;
	free(t3);
}

bool inno_cmd_resetjob(struct T3_chain *t3, uint8_t chip_id)
{
	//printf("send command job [reset] \n");

	if(!cmd_RESET_IDUAL(t3, chip_id))
	{
		applog(LOG_WARNING, "cmd_RESET_BCAST fail !");
	}

	if(cmd_CHECK_BUSY(t3, chip_id) != WORK_FREE)
	{
		return false;
	}

	return true;
	
}

static void chain_t3_hw_enable(int num_chain)
{
	/*
	 *chain 1
	 *GPIO_RST1(start pin) -> GPMC_A5(gpio1_21) -> gpio53
	 *CTL0_IO0 (reset pin) -> MCASP0_ACLKR(gpio3_18) -> gpio114
	 */
	applog(LOG_DEBUG, "T3 enable chain RST1");
	int ret;

	switch(num_chain){
		case 0:
			//GPIO3_19 host pin8
			ret = access("/sys/class/gpio/gpio115", F_OK); //power enable
			if(ret == -1)//file not exist
			{
				system("echo 115 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio115/direction");
				system("echo 0 > /sys/class/gpio/gpio115/value");
			}
			//GPMC_A5 gpio1_21 host pin14
			ret = access("/sys/class/gpio/gpio53", F_OK); //start
			if(ret == -1)//file not exist
			{
				system("echo 53 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio53/direction");
				system("echo 1 > /sys/class/gpio/gpio53/value");
			}
			//GPIO3_18 host pin6
			ret = access("/sys/class/gpio/gpio114", F_OK); //reset
			if(ret == -1)//file not exist
			{
				system("echo 114 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio114/direction");
				system("echo 1 > /sys/class/gpio/gpio114/value");
			}
			cgsleep_us(1000 * 500);
			system("echo 1 > /sys/class/gpio/gpio115/value");

			//start pin and reset pin set low level

			//after power on it assert reset pin and wait 500ms	
			system("echo 0 > /sys/class/gpio/gpio114/value");
			cgsleep_us(1000 * 500);

			//assert start pin and wait 500ms
			system("echo 0 > /sys/class/gpio/gpio53/value");

			cgsleep_us(1000 * 500);

			break;
		case 1:
			break;
		case 2:
			break;
		default:;
	}
}


static void chain_t2_hw_enable(int num_chain)
{
	/*
	 *chain 1
	 *GPIO_RST1(start pin) -> GPMC_A5(gpio1_21) -> gpio53
	 *CTL0_IO0 (reset pin) -> MCASP0_ACLKR(gpio3_18) -> gpio114
	 */
	int ret;
	applog(LOG_DEBUG, "T3 enable chain RST1");

	switch(num_chain){
		case 0:
			//GPIO3_19 host pin8
			ret = access("/sys/class/gpio/gpio115", F_OK); //power enable
			if(ret == -1)//file not exist
			{
				system("echo 115 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio115/direction");
				system("echo 0 > /sys/class/gpio/gpio115/value");
			}
			//GPMC_A5 gpio1_21 host pin14
			ret = access("/sys/class/gpio/gpio53", F_OK); //start
			if(ret == -1)//file not exist
			{
				system("echo 53 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio53/direction");
				system("echo 1 > /sys/class/gpio/gpio53/value");
			}
			//GPIO3_18 host pin6
			ret = access("/sys/class/gpio/gpio114", F_OK); //reset
			if(ret == -1)//file not exist
			{
				system("echo 114 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio114/direction");
				system("echo 1 > /sys/class/gpio/gpio114/value");
			}

			//start pin and reset pin set low level

			//after power on it assert reset pin and wait 500ms
			system("echo 1 > /sys/class/gpio/gpio115/value");
			system("echo 0 > /sys/class/gpio/gpio53/value");
			system("echo 0 > /sys/class/gpio/gpio114/value");
			cgsleep_us(1000 * 500);

			//assert start pin and wait 500ms
			system("echo 1 > /sys/class/gpio/gpio114/value");
			cgsleep_us(1000 * 500);
			system("echo 0 > /sys/class/gpio/gpio114/value");
			cgsleep_us(1000 * 500);
			break;
		case 1:
			//GPMC_AD11 gpio0_27  host pin8
			ret = access("/sys/class/gpio/gpio27", F_OK); //power enable
			if(ret == -1)//file not exist
			{
				system("echo 27 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio27/direction");
				system("echo 0 > /sys/class/gpio/gpio27/value");
			}
			//GPMC_A6 gpio1_22 host pin14
			ret = access("/sys/class/gpio/gpio54", F_OK); //start
			if(ret == -1)//file not exist
			{
				system("echo 54 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio54/direction");
				system("echo 1 > /sys/class/gpio/gpio54/value");
			}
			//GPMC_AD10 gpio0_26  host pin6
			ret = access("/sys/class/gpio/gpio45", F_OK); //reset
			if(ret == -1)//file not exist
			{
				system("echo 45 > /sys/class/gpio/export");
				system("echo out > /sys/class/gpio/gpio45/direction");
				system("echo 1 > /sys/class/gpio/gpio45/value");
			}

			//power on and reset pin and wait 500ms
			system("echo 1 > /sys/class/gpio/gpio27/value");
			system("echo 0 > /sys/class/gpio/gpio54/value");
			system("echo 0 > /sys/class/gpio/gpio45/value");
			cgsleep_us(1000 * 500);

			//assert start pin and wait 500ms
			system("echo 1 > /sys/class/gpio/gpio45/value");
			cgsleep_us(1000 * 500);
			system("echo 0 > /sys/class/gpio/gpio45/value");
			cgsleep_us(1000 * 500);
			break;
		case 2:
			break;
		default:;
	}
}


struct T3_chain *init_T3_chain(struct spi_ctx *ctx, int chain_id)
{
	int i;
	struct T3_chain *t3 = malloc(sizeof(*t3));
	assert(t3 != NULL);

	
	memset(t3, 0, sizeof(*t3));
	t3->spi_ctx = ctx;
	t3->chain_id = chain_id;

	applog(LOG_INFO,"chain_id:%d", chain_id);
	switch(chain_id){
		case 0:t3->num_chips = chain_t3_detect(t3, A1Pll1);break;
		case 1:t3->num_chips = chain_t3_detect(t3, A1Pll2);break;
		case 2:t3->num_chips = chain_t3_detect(t3, A1Pll3);break;
		case 3:t3->num_chips = chain_t3_detect(t3, A1Pll4);break;
		case 4:t3->num_chips = chain_t3_detect(t3, A1Pll5);break;
		case 5:t3->num_chips = chain_t3_detect(t3, A1Pll6);break;
		default:;
	}
	cgsleep_us(10000);
	
	if (t3->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d T3 chips",
	       t3->spi_ctx->config.bus, t3->spi_ctx->config.cs_line,
	       t3->chain_id, t3->num_chips);
	
	t3->num_active_chips = t3->num_chips;

	t3->chips = calloc(t3->num_active_chips, sizeof(struct T3_chip));
	assert (t3->chips != NULL);


	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       t3->chain_id, t3->num_active_chips, t3->num_cores);

	mutex_init(&t3->lock);
	INIT_LIST_HEAD(&t3->active_wq.head);

	return t3;

failure:
	exit_T3_chain(t3);
	return NULL;
}

static bool detect_T3_chain()
{
	int i;
	
	applog(LOG_WARNING, "T3: checking T3 chain");

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		cfg[i] = default_spi_config;
		cfg[i].bus     = i + 1;
		cfg[i].cs_line = 0;
		cfg[i].mode = SPI_MODE_1;

		cfg[i].speed = DEFAULT_SPI_SPEED;

		spi[i] = spi_init(&cfg[i]);
		if (spi == NULL)
		{
			applog(LOG_ERR, "spi init fail");
			return false;
		}
	}

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		// chains power on
		chain_t3_hw_enable( i );
	}

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		chain[i] = init_T3_chain(spi[i], i);
		if (chain[i] == NULL)
		{
			applog(LOG_ERR, "init t3 chain fail");
			return false;
		}

		struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
		assert(cgpu != NULL);
	
		memset(cgpu, 0, sizeof(*cgpu));
		cgpu->drv = &bitmineA1_drv;
		cgpu->name = "BitmineA1.SingleChainn";
		cgpu->threads = 1;

		cgpu->device_data = chain[i];

		chain[i]->cgpu = cgpu;
		add_cgpu(cgpu);

		applog(LOG_WARNING, "Detected the %d T3 chain with %d chips / %d cores",
		       i, chain[i]->num_active_chips, chain[i]->num_cores);
	}

	return true;
}

static void chain_hw_dis(void)
{
}

/* Probe SPI channel and register chip chain */
void T3_detect(bool hotplug)
{
	/* no hotplug support for SPI */
	if (hotplug)
		return;

	applog(LOG_DEBUG, "T3 detect");

	/* detect and register supported products */
	if (detect_T3_chain())
	{
		loop_main_test();
	}

    int i = 0;
	/* release SPI context if no T3 products found */
	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		spi_exit(spi[i]);
	}

}

#define TEMP_UPDATE_INT_MS	2000
static int64_t T3_scanwork(	struct thr_info *thr)
{
	int i;
	int32_t A1Pll = 1000;

	struct T3_chain *t3 = chain[0];

	if (t3->num_cores == 0) {
		return 0;
	}

	uint32_t nonce;
	//0x3429e96 0x9a9b041 0xb118d8f 0x1358109c
	uint32_t tnonce = 0x9a9b041;
	uint8_t chip_id;
	uint8_t job_id;
	bool work_updated = false;
	uint32_t k, *work_nonce=NULL;
	unsigned char pworkdata[128]= {0},  hash1[32]= {0};
    unsigned int endiandata[32]= {0};
    struct work work = {
    .data = {
		0x20, 0x00, 0x00, 0x00,
		0x0b, 0x77, 0xe5, 0x7f,
		0x5f, 0x70, 0x80, 0x36,
		0x3d, 0x13, 0xd1, 0x46,
		0xf0, 0x12, 0x2f, 0xe8,
		0x62, 0x1e, 0x7c, 0x7f,
		0x2c, 0x6e, 0x6a, 0x3a,
		0x00, 0x00, 0x00, 0x38,
		0x00, 0x00, 0x00, 0x00,
		0x34, 0x0e, 0x37, 0x0b,
		0x14, 0x53, 0x6c, 0xa3,
		0x34, 0xa3, 0x40, 0xc4,
		0x92, 0x3a, 0x11, 0x5b,
		0x50, 0x5d, 0xa4, 0x57,
		0x14, 0x61, 0xe1, 0x2b,
		0xfc, 0xde, 0x92, 0xb2,
		0x17, 0x10, 0x25, 0x8f,
		0x5a, 0x5a, 0x3f, 0xac,
		0x19, 0x43, 0x7e, 0x39,
		0x00, 0x00, 0x00, 0x00,
	},
	};

	mutex_lock(&t3->lock);
	int cid = t3->chain_id; 

	/* poll queued results */
	while (true)
	{

		if (!get_nonce(t3, (uint8_t*)&nonce, &chip_id, &job_id))
			break;

		nonce = bswap_32(nonce);   //modify for A4
		printf("dongfupang %s() %d chip %d nonce = %08x\n", __func__, __LINE__, chip_id, nonce);
		work_updated = true;
		if (chip_id < 1 || chip_id > t3->num_active_chips) 
		{
			applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 15) 
		{
			applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
			flush_spi(t3);
			continue;
		}

		struct T3_chip *chip = &t3->chips[chip_id - 1];

		work_nonce = (uint32_t *)(work.data + 64 + 12);
		*work_nonce = nonce;
		memcpy(pworkdata, work.data, 80);

        for (k=0; k < 20; k++)
        {
            endiandata[k] = ((uint32_t*)pworkdata)[k];
            endiandata[k] = Swap32(endiandata[k]);
            //applog(LOG_DEBUG,"%s: endiandata[%d] = 0x%08x", __FUNCTION__, k, endiandata[k]);
        }

        Xhash(hash1, endiandata);
        memcpy(work.hash, hash1, 32);
		printf("dongfupang %s() %d hash1[7] = %08x\n", __func__, __LINE__, hash1[7]);
		printf("dongfupang %s() %d hash1[6] = %08x\n", __func__, __LINE__, hash1[6]);
		if(*((uint32_t *)(&work.hash) + 7) <= 32)
		{
			printf("*******************************\n");
			//printf("dongfupang %s() %d tnonce = %08x\n", __func__, __LINE__, tnonce);
			printf("*******************************\n");
			if (fulltest(hash1, work.target))
			{
				printf("*******************************\n");
				//printf("dongfupang %s() %d tnonce = %08x\n", __func__, __LINE__, tnonce);
				printf("*******************************\n");
				//submit_nonce_direct(thr,work, nonce);
			}
		} else {
			applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x", cid, chip_id, nonce);
			chip->hw_errors++;
			/* add a penalty of a full nonce range on HW errors */
			continue;
		}
		applog(LOG_INFO, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x", cid, chip_id, job_id, nonce);
		chip->nonces_found++;
	}

	uint8_t reg[REG_LENGTH];

	for (i = t3->num_active_chips; i > 0; i--) 
	{

		uint8_t c = i;
		if (is_chip_disabled(t3, c))
			continue;
		if (!cmd_READ_REG(t3, c))
		{
			disable_chip(t3, c);
			continue;
		}
        else
        {
			//hexdump("send433: RX", t3->spi_rx, 18);
            /* update temp database */
            uint32_t temp = 0;
            float    temp_f = 0.0f;

            temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
            //inno_fan_temp_add(&s_fan_ctrl, cid, temp, false);
        }

		uint8_t qstate = t3->spi_rx[11] & 0x01;
		uint8_t qbuff = 0;
		struct T3_chip *chip = &t3->chips[i - 1];
		switch(qstate)
		{
		case 1:
			//applog(LOG_INFO, "chip %d busy now", i);
			break;
			/* fall through */
		case 0:

			if (set_work(t3, c, &work, qbuff)) 
			{
				chip->nonce_ranges_done++;
				applog(LOG_INFO, "set work success nonces processed %d", cid);
			}
			
			//applog(LOG_INFO, "%d: chip %d: job done: %d/%d/%d/%d",
			//       cid, c,
			//       chip->nonce_ranges_done, chip->nonces_found,
			//       chip->hw_errors, chip->stales);
			break;
		}
	} 

	mutex_unlock(&t3->lock);

	cgsleep_ms(40);

	return (int64_t)(2011173.18 * A1Pll / 1000 * (t3->num_cores/9.0) * (t3->tvScryptDiff.tv_usec / 1000000.0));

}


/* queue two work items per chip in chain */
static bool T3_queue_full(struct cgpu_info *cgpu)
{
	struct T3_chain *t3 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&t3->lock);
	applog(LOG_DEBUG, "%d, T3 running queue_full: %d/%d",
	       t3->chain_id, t3->active_wq.num_elems, t3->num_active_chips);

	if (t3->active_wq.num_elems >= t3->num_active_chips * 2)
		queue_full = true;
	else
	{
		printf("dongfupang %s() %d\n", __func__, __LINE__);
		wq_enqueue(&t3->active_wq, get_queued(cgpu));
	}
	mutex_unlock(&t3->lock);

	return queue_full;
}

static void T3_flush_work(struct cgpu_info *cgpu)
{
	struct T3_chain *t3 = cgpu->device_data;
	int cid = t3->chain_id;
	//board_selector->select(cid);
	int i;

	mutex_lock(&t3->lock);
	/* stop chips hashing current work */
	if (!abort_work(t3)) 
	{
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < t3->num_active_chips; i++) 
	{
		int j;
		struct T3_chip *chip = &t3->chips[i];
		for (j = 0; j < 4; j++)
		{
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			//applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
			//       cid, i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}

		chip->last_queued_id = 0;

		if(!inno_cmd_resetjob(t3, i + 1))
		{
			applog(LOG_WARNING, "chip %d clear work false", i + 1);
			continue;
		} else
		{
			cmd_BIST_FIX_NN(t3, i + 1);
			check_chip(t3, i);
		}

		//applog(LOG_INFO, "chip :%d flushing queued work success", i);
	}
	/* flush queued work */
	//applog(LOG_DEBUG, "%d: flushing queued work...", cid);
	while (t3->active_wq.num_elems > 0) 
	{
		struct work *work = wq_dequeue(&t3->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&t3->lock);

}

static bool bitmain_DASH_prepare(struct thr_info *thr)
{
	return true;
}

static void T3_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
	struct T3_chain *t3 = cgpu->device_data;
	char temp[10];
	if (t3->temp != 0)
		snprintf(temp, 9, "%2dC", t3->temp);
	tailsprintf(buf, len, " %2d:%2d/%3d %s",
		    t3->chain_id, t3->num_active_chips, t3->num_cores,
		    t3->temp == 0 ? "   " : temp);
}

void loop_main_test(void)
{
	while(1)
		T3_scanwork(NULL);
}
struct device_drv bitmineA1_drv = {
	.drv_id = DRIVER_bitmineA1,
	.dname = "BitmineA1",
	.name = "BA1",
	.drv_detect = T3_detect,
	.thread_prepare = bitmain_DASH_prepare,
	.hash_work = hash_queued_work,
	.scanwork = T3_scanwork,
	.queue_full = T3_queue_full,
	.flush_work = T3_flush_work,
	.get_statline_before = T3_get_statline_before,
};

