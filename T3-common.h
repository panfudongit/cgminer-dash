#ifndef T3_COMMON_H
#define T3_COMMON_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "elist.h"

/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

/********** chip and chain context structures */
/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */

#define ASIC_CHAIN_NUM                  1
#define ASIC_CHIP_NUM                   2

#define ADDR_BROADCAST		0x00

#define LEN_BIST_START		6
#define LEN_BIST_COLLECT	4
#define LEN_BIST_FIX		4
#define LEN_RESET			6
#define LEN_WRITE_JOB		94
#define LEN_READ_RESULT		4
#define LEN_WRITE_REG		18
#define LEN_READ_REG		4

#define SPI_REC_DATA_LOOP	10
#define SPI_REC_DATA_DELAY	1

//#define ASIC_REGISTER_NUM	12
#define ASIC_RESULT_LEN		6
#define READ_RESULT_LEN		(ASIC_RESULT_LEN + 2)

#define REG_LENGTH		14

#define JOB_LENGTH		96

#define MAX_CHAIN_LENGTH	64
#define MAX_CMD_LENGTH		(JOB_LENGTH + MAX_CHAIN_LENGTH * 2 * 2)

#define WORK_BUSY 0
#define WORK_FREE 1

#define T3_PLL_CLOCK_1400MHz 213 // (20 + 213) * 6 = 1398MHz
#define T3_REG_TO_CLOCK(x) ((20 + x) * 6)

/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
enum T3_command {
	T3_BIST_START       = 0x01,
	T3_BIST_FIX         = 0x03,
	T3_RESET            = 0x04,
	T3_WRITE_JOB        = 0x07,
	T3_READ_RESULT      = 0x08,
	T3_WRITE_REG        = 0x09,
	T3_READ_REG         = 0x0a,
	T3_READ_REG_RESP    = 0x1a,
	T3_BIST_COLLECT     = 0x0b,
	T3_RESET_RES        = 0xb4,
};

struct T3_chip {
	uint8_t reg[12];
	int num_cores;
	int last_queued_id;
	struct work *work[15];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	int fail_reset;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;

    /* temp */
	int temp;

};

struct T3_chain {
	int chain_id;
	struct cgpu_info *cgpu;
	struct mcp4x *trimpot;
	int num_chips;
	int num_cores;
	int num_active_chips;
	int chain_skew;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *spi_ctx;
	struct T3_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;

	/* mark chain disabled, do not try to re-enable it */
	bool disabled;
	uint8_t temp;
	int last_temp_time;

	struct timeval tvScryptLast;
	struct timeval tvScryptCurr;
	struct timeval tvScryptDiff;
	int work_start_delay;

};

#define MAX_CHAINS_PER_BOARD	2
struct T3_board {
	int board_id;
	int num_chains;
	struct T3_chain *chain[MAX_CHAINS_PER_BOARD];
};

void loop_main_test(void);
#endif /* T3_COMMON_H */
