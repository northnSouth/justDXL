/* jdxl - DYNAMIXEL Protocol 2.0
 * Copyright (c) 2026 Aftito Nur Faturohim
 * SPDX-License-Identifier: MIT
 *
 * Source: DYNAMIXEL Protocol 2.0 (https://docs.robotis.com/docs/dxl/protocol/protocol2)
 */
#ifndef JDXL_PH2_H
#define JDXL_PH2_H

#include "jdxl/jdxl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sizes. They dimension jdxl_ph2_ctx_t, so they are part of the ABI: the CMake build defines
 * them on the jdxl target (PUBLIC) so every consumer agrees with the library. */
#ifndef JDXL_PH2_PKT_MAX_LEN
#  error "JDXL_PH2_PKT_MAX_LEN must be defined (link against the jdxl CMake target)"
#endif
#ifndef JDXL_PH2_MAX_STATUS
#  error "JDXL_PH2_MAX_STATUS must be defined (link against the jdxl CMake target)"
#endif
/* 14 = a Ping status; 65542 = 7 header bytes + the 16-bit LENGTH's maximum */
#if JDXL_PH2_PKT_MAX_LEN < 14 || JDXL_PH2_PKT_MAX_LEN > 65542
#  error "JDXL_PH2_PKT_MAX_LEN must be in 14..65542"
#endif
/* 253 = number of unicast IDs (0-252) */
#if JDXL_PH2_MAX_STATUS < 1 || JDXL_PH2_MAX_STATUS > 253
#  error "JDXL_PH2_MAX_STATUS must be in 1..253"
#endif

/* Instructions ("Instruction") */
enum {
        JDXL_PH2_INST_PING              = 0x01,
        JDXL_PH2_INST_READ              = 0x02,
        JDXL_PH2_INST_WRITE             = 0x03,
        JDXL_PH2_INST_REG_WRITE         = 0x04,
        JDXL_PH2_INST_ACTION            = 0x05,
        JDXL_PH2_INST_FACTORY_RESET     = 0x06,
        JDXL_PH2_INST_REBOOT            = 0x08,
        JDXL_PH2_INST_CLEAR             = 0x10,
        JDXL_PH2_INST_CTRL_TABLE_BACKUP = 0x20,
        JDXL_PH2_INST_STATUS            = 0x55,
        JDXL_PH2_INST_SYNC_READ         = 0x82,
        JDXL_PH2_INST_SYNC_WRITE        = 0x83,
        JDXL_PH2_INST_FAST_SYNC_READ    = 0x8A,
        JDXL_PH2_INST_BULK_READ         = 0x92,
        JDXL_PH2_INST_BULK_WRITE        = 0x93,
        JDXL_PH2_INST_FAST_BULK_READ    = 0x9A
};

/* Status ERROR byte ("Error"): bit 7 is the Alert flag (hardware error, see the control table),
 * bits 6..0 the error number. */
#define JDXL_PH2_ERR_ALERT       0x80u
#define JDXL_PH2_ERR_NUMBER(e)   ((uint8_t)((e) & 0x7Fu))
enum {
        JDXL_PH2_ERR_NONE         = 0x00,
        JDXL_PH2_ERR_RESULT_FAIL  = 0x01,
        JDXL_PH2_ERR_INSTRUCTION  = 0x02,
        JDXL_PH2_ERR_CRC          = 0x03,
        JDXL_PH2_ERR_DATA_RANGE   = 0x04,
        JDXL_PH2_ERR_DATA_LENGTH  = 0x05,
        JDXL_PH2_ERR_DATA_LIMIT   = 0x06,
        JDXL_PH2_ERR_ACCESS       = 0x07
};

typedef enum {
        JDXL_PH2_RESET_ALL                 = 0xFF,
        JDXL_PH2_RESET_ALL_EXCEPT_ID       = 0x01,
        JDXL_PH2_RESET_ALL_EXCEPT_ID_BAUD  = 0x02
} jdxl_ph2_reset_t;

typedef enum {
        JDXL_PH2_CLEAR_MULTI_TURN = 0x01,
        JDXL_PH2_CLEAR_ERROR      = 0x02
} jdxl_ph2_clear_t;

typedef enum {
        JDXL_PH2_BACKUP_STORE   = 0x01,
        JDXL_PH2_BACKUP_RESTORE = 0x02
} jdxl_ph2_backup_t;

/* Sync Write entry: `data` points to the build call's len bytes for this servo. */
typedef struct {
        uint8_t        id;
        const uint8_t *data;
} jdxl_ph2_sync_write_t;

/* Bulk Read / Fast Bulk Read entry. */
typedef struct {
        uint8_t  id;
        uint16_t addr;
        uint16_t len;
} jdxl_ph2_bulk_read_t;

/* Bulk Write entry. */
typedef struct {
        uint8_t        id;
        uint16_t       addr;
        uint16_t       len;
        const uint8_t *data;
} jdxl_ph2_bulk_write_t;

/* One per bus. Initialize with jdxl_ph2_init(). No internal locking. */
typedef struct {
        jdxl_opts_t  opts;
        uint8_t      tx[JDXL_PH2_PKT_MAX_LEN];
        size_t       tx_len;

        struct {                                /* internal */
                uint8_t     rx[JDXL_PH2_PKT_MAX_LEN];
                size_t      rx_len;
                size_t      echo_left;
                uint8_t     fast;               /* pending reply is one Fast Sync/Bulk Read packet */
                uint8_t     count;              /* replies expected by this exchange */
                uint8_t     idx;                /* next reply expected */
                jdxl_slot_t slot[JDXL_PH2_MAX_STATUS];
        } priv;

        jdxl_debug_t debug;
} jdxl_ph2_ctx_t;

/* opts may be NULL: Status Return Level ALL, no echo. */
void jdxl_ph2_init(jdxl_ph2_ctx_t *ctx, const jdxl_opts_t *opts);

/* The instruction packet built last; valid until the next build. {NULL, 0} if none. */
jdxl_span_t jdxl_ph2_tx(const jdxl_ph2_ctx_t *ctx);

/* Status replies still expected. */
uint8_t jdxl_ph2_pending(const jdxl_ph2_ctx_t *ctx);

/* Gives up on the pending exchange (e.g. on timeout); the replies not received count as missed. */
void jdxl_ph2_abort(jdxl_ph2_ctx_t *ctx);

/* Feeds received bytes. Consumes the whole buffer, delivers every complete expected reply to
 * on_status (may be NULL), and skips (and counts) anything malformed or unexpected. Byte
 * stuffing is removed before delivery. A Fast Sync/Bulk Read reply is one packet, delivered as
 * one callback per servo. */
jdxl_feed_t jdxl_ph2_feed(jdxl_ph2_ctx_t *ctx, const uint8_t *buf, size_t len,
                          jdxl_on_status_t on_status, void *user);

/* Builders. Each starts a new exchange: a previous pending one is dropped (its replies count as
 * missed). Unicast IDs are 0-252; JDXL_BROADCAST_ID where noted (then only Ping and the read
 * instructions are answered, per spec). A JDXL_ERR_TOO_LONG caused by byte stuffing is only found
 * while encoding: the previous exchange is dropped by then, and nothing is pending. */

/* Answered regardless of the Status Return Level. Unicast only. */
jdxl_result_t jdxl_ph2_build_ping(jdxl_ph2_ctx_t *ctx, uint8_t id);
/* Every servo answers, in ID order. Completes after max_replies replies; with fewer servos on the
 * bus, call jdxl_ph2_abort() on timeout. */
jdxl_result_t jdxl_ph2_build_ping_broadcast(jdxl_ph2_ctx_t *ctx, uint8_t max_replies);
/* Unicast only; len >= 1. */
jdxl_result_t jdxl_ph2_build_read(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr, uint16_t len);
/* Unicast or broadcast (no reply); len >= 1. */
jdxl_result_t jdxl_ph2_build_write(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr,
                                   const uint8_t *data, size_t len);
jdxl_result_t jdxl_ph2_build_reg_write(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr,
                                       const uint8_t *data, size_t len);
jdxl_result_t jdxl_ph2_build_action(jdxl_ph2_ctx_t *ctx, uint8_t id);
/* Broadcast with JDXL_PH2_RESET_ALL is rejected: the spec says servos ignore it. */
jdxl_result_t jdxl_ph2_build_factory_reset(jdxl_ph2_ctx_t *ctx, uint8_t id, jdxl_ph2_reset_t mode);
jdxl_result_t jdxl_ph2_build_reboot(jdxl_ph2_ctx_t *ctx, uint8_t id);
jdxl_result_t jdxl_ph2_build_clear(jdxl_ph2_ctx_t *ctx, uint8_t id, jdxl_ph2_clear_t mode);
jdxl_result_t jdxl_ph2_build_ctrl_table_backup(jdxl_ph2_ctx_t *ctx, uint8_t id,
                                               jdxl_ph2_backup_t mode);
/* Always broadcast; one reply per ID, in ID-list order. */
jdxl_result_t jdxl_ph2_build_sync_read(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                       const uint8_t *ids, uint8_t count);
jdxl_result_t jdxl_ph2_build_fast_sync_read(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                            const uint8_t *ids, uint8_t count);
/* Always broadcast, never answered. Every entry writes `len` bytes at `addr`. */
jdxl_result_t jdxl_ph2_build_sync_write(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                        const jdxl_ph2_sync_write_t *entries, uint8_t count);
/* Always broadcast; one reply per entry, in entry order. */
jdxl_result_t jdxl_ph2_build_bulk_read(jdxl_ph2_ctx_t *ctx,
                                       const jdxl_ph2_bulk_read_t *entries, uint8_t count);
jdxl_result_t jdxl_ph2_build_fast_bulk_read(jdxl_ph2_ctx_t *ctx,
                                            const jdxl_ph2_bulk_read_t *entries, uint8_t count);
/* Always broadcast, never answered. */
jdxl_result_t jdxl_ph2_build_bulk_write(jdxl_ph2_ctx_t *ctx,
                                        const jdxl_ph2_bulk_write_t *entries, uint8_t count);

/* Pure encoder for any instruction: header, ID, LENGTH, INST, byte-stuffed params, CRC.
 * params and out must not overlap. */
jdxl_result_t jdxl_ph2_encode(uint8_t id, uint8_t inst, const uint8_t *params, size_t n,
                              uint8_t *out, size_t cap, size_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* JDXL_PH2_H */