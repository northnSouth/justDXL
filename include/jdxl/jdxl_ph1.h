/* jdxl - DYNAMIXEL Protocol 1.0
 * Copyright (c) 2026 Aftito Nur Faturohim
 * SPDX-License-Identifier: MIT
 *
 * Source: DYNAMIXEL Protocol 1.0 (https://emanual.robotis.com/docs/en/dxl/protocol1/)
 */
#ifndef JDXL_PH1_H
#define JDXL_PH1_H

#include "jdxl/jdxl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sizes. They dimension jdxl_ph1_ctx_t, so they are part of the ABI: the CMake build defines
 * them on the jdxl target (PUBLIC) so every consumer agrees with the library. */
#ifndef JDXL_PH1_PKT_MAX_LEN
#  error "JDXL_PH1_PKT_MAX_LEN must be defined (link against the jdxl CMake target)"
#endif
#ifndef JDXL_PH1_MAX_STATUS
#  error "JDXL_PH1_MAX_STATUS must be defined (link against the jdxl CMake target)"
#endif
/* 6 is the shortest packet; 259 = header(2) + ID + LENGTH + LENGTH's maximum of 255 */
#if JDXL_PH1_PKT_MAX_LEN < 6 || JDXL_PH1_PKT_MAX_LEN > 259
#  error "JDXL_PH1_PKT_MAX_LEN must be in 6..259"
#endif
/* 254 = number of unicast IDs (0x00-0xFD) */
#if JDXL_PH1_MAX_STATUS < 1 || JDXL_PH1_MAX_STATUS > 254
#  error "JDXL_PH1_MAX_STATUS must be in 1..254"
#endif

/* Instructions ("Instruction") */
enum {
        JDXL_PH1_INST_PING          = 0x01,
        JDXL_PH1_INST_READ          = 0x02,
        JDXL_PH1_INST_WRITE         = 0x03,
        JDXL_PH1_INST_REG_WRITE     = 0x04,
        JDXL_PH1_INST_ACTION        = 0x05,
        JDXL_PH1_INST_FACTORY_RESET = 0x06,
        JDXL_PH1_INST_REBOOT        = 0x08,
        JDXL_PH1_INST_SYNC_WRITE    = 0x83,
        JDXL_PH1_INST_BULK_READ     = 0x92
};

/* Status ERROR bits ("Error"); bit 7 is always 0 */
#define JDXL_PH1_ERR_INPUT_VOLTAGE (1u << 0)
#define JDXL_PH1_ERR_ANGLE_LIMIT   (1u << 1)
#define JDXL_PH1_ERR_OVERHEATING   (1u << 2)
#define JDXL_PH1_ERR_RANGE         (1u << 3)
#define JDXL_PH1_ERR_CHECKSUM      (1u << 4)
#define JDXL_PH1_ERR_OVERLOAD      (1u << 5)
#define JDXL_PH1_ERR_INSTRUCTION   (1u << 6)

/* Sync Write entry: `data` points to the build call's data_len bytes for this servo. */
typedef struct {
        uint8_t        id;
        const uint8_t *data;
} jdxl_ph1_sync_write_t;

/* Bulk Read entry. */
typedef struct {
        uint8_t id;
        uint8_t addr;
        uint8_t len;
} jdxl_ph1_bulk_read_t;

/* One per bus. Initialize with jdxl_ph1_init(). No internal locking. */
typedef struct {
        jdxl_opts_t  opts;
        uint8_t      tx[JDXL_PH1_PKT_MAX_LEN];
        size_t       tx_len;

        struct {                                /* internal */
                uint8_t     rx[JDXL_PH1_PKT_MAX_LEN];
                size_t      rx_len;
                size_t      echo_left;
                uint8_t     count;              /* replies expected by this exchange */
                uint8_t     idx;                /* next reply expected */
                jdxl_slot_t slot[JDXL_PH1_MAX_STATUS];
        } priv;

        jdxl_debug_t debug;
} jdxl_ph1_ctx_t;

/* opts may be NULL: Status Return Level ALL, no echo. */
void jdxl_ph1_init(jdxl_ph1_ctx_t *ctx, const jdxl_opts_t *opts);

/* The instruction packet built last; valid until the next build. {NULL, 0} if none. */
jdxl_span_t jdxl_ph1_tx(const jdxl_ph1_ctx_t *ctx);

/* Status replies still expected. */
uint8_t jdxl_ph1_pending(const jdxl_ph1_ctx_t *ctx);

/* Gives up on the pending exchange (e.g. on timeout); the replies not received count as missed. */
void jdxl_ph1_abort(jdxl_ph1_ctx_t *ctx);

/* Feeds received bytes. Consumes the whole buffer, delivers every complete expected reply to
 * on_status (may be NULL), and skips (and counts) anything malformed or unexpected. Bytes with
 * nothing pending are dropped (debug.ignored_bytes). */
jdxl_feed_t jdxl_ph1_feed(jdxl_ph1_ctx_t *ctx, const uint8_t *buf, size_t len,
                          jdxl_on_status_t on_status, void *user);

/* Builders. Each starts a new exchange: a previous pending one is dropped (its replies count as
 * missed). Unicast IDs are 0x00-0xFD; JDXL_BROADCAST_ID where noted. */

/* Answered regardless of the Status Return Level. Unicast only. */
jdxl_result_t jdxl_ph1_build_ping(jdxl_ph1_ctx_t *ctx, uint8_t id);
/* Unicast only; len >= 1. */
jdxl_result_t jdxl_ph1_build_read(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr, uint8_t len);
/* Unicast or broadcast (no reply); len >= 1. */
jdxl_result_t jdxl_ph1_build_write(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr,
                                   const uint8_t *data, size_t len);
jdxl_result_t jdxl_ph1_build_reg_write(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr,
                                       const uint8_t *data, size_t len);
jdxl_result_t jdxl_ph1_build_action(jdxl_ph1_ctx_t *ctx, uint8_t id);
/* Unicast only: the spec forbids broadcast Factory Reset. */
jdxl_result_t jdxl_ph1_build_factory_reset(jdxl_ph1_ctx_t *ctx, uint8_t id);
jdxl_result_t jdxl_ph1_build_reboot(jdxl_ph1_ctx_t *ctx, uint8_t id);
/* Always broadcast, never answered. Every entry writes `len` bytes at `addr`. */
jdxl_result_t jdxl_ph1_build_sync_write(jdxl_ph1_ctx_t *ctx, uint8_t addr, uint8_t len,
                                        const jdxl_ph1_sync_write_t *entries, uint8_t count);
/* Always broadcast; one reply per entry, in entry order. */
jdxl_result_t jdxl_ph1_build_bulk_read(jdxl_ph1_ctx_t *ctx,
                                       const jdxl_ph1_bulk_read_t *entries, uint8_t count);

/* Pure encoder for any instruction: writes FF FF ID LEN INST params CHECKSUM to out.
 * params and out must not overlap. */
jdxl_result_t jdxl_ph1_encode(uint8_t id, uint8_t inst, const uint8_t *params, size_t n,
                              uint8_t *out, size_t cap, size_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* JDXL_PH1_H */