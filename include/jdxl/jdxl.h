/* jdxl - DYNAMIXEL Protocol 1.0 / 2.0 packet handlers
 * Copyright (c) 2026 Aftito Nur Faturohim
 * SPDX-License-Identifier: MIT
 *
 * Types shared by jdxl_ph1.h and jdxl_ph2.h.
 */
#ifndef JDXL_H
#define JDXL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Broadcast ID, the same in both protocols. */
#define JDXL_BROADCAST_ID 0xFEu

/* Result of every build function. On any error the context is unchanged, except where a
 * function documents otherwise. */
typedef enum {
        JDXL_OK = 0,
        JDXL_ERR_NULL_ARG,          /* a required pointer is NULL */
        JDXL_ERR_INVALID_ID,        /* ID out of range, or broadcast where the spec forbids it */
        JDXL_ERR_INVALID_ARG,       /* unknown instruction or mode */
        JDXL_ERR_EMPTY,             /* zero servos, or zero data bytes */
        JDXL_ERR_DUPLICATE_ID,      /* the same ID twice in one Sync/Bulk instruction */
        JDXL_ERR_TOO_MANY_STATUS,   /* more status replies than JDXL_PHx_MAX_STATUS */
        JDXL_ERR_TOO_LONG           /* the instruction or its status reply does not fit JDXL_PHx_PKT_MAX_LEN */
} jdxl_result_t;

const char *jdxl_result_str(jdxl_result_t result);

/* Progress of the pending exchange, returned by jdxl_phX_feed(). */
typedef enum {
        JDXL_FEED_IDLE = 0,         /* nothing pending */
        JDXL_FEED_PENDING,          /* waiting for more status replies */
        JDXL_FEED_DONE              /* the last expected reply arrived during this call */
} jdxl_feed_t;

/* A read-only byte range. */
typedef struct {
        const uint8_t *data;
        size_t         len;
} jdxl_span_t;

/* One status reply, as delivered to jdxl_on_status_t. */
typedef struct {
        uint8_t        index;       /* position in the instruction's servo list
                                       (arrival order for a broadcast Ping) */
        uint8_t        id;          /* responding ID */
        uint8_t        error;       /* raw ERROR byte: JDXL_PH1_ERR_* bits / JDXL_PH2_ERR_* */
        uint16_t       params_len;  /* may be less than requested if the servo reports an error */
        const uint8_t *params;      /* points into the context: valid only during the callback */
} jdxl_status_t;

/* Called once per status reply, in arrival order. Must not call any jdxl function on the same
 * context. */
typedef void (*jdxl_on_status_t)(const jdxl_status_t *status, void *user);

/* Status Return Level configured on the servos (control table). Builders only wait for the
 * replies the servos will actually send. */
typedef enum {
        JDXL_SRL_PING_ONLY = 0,     /* only Ping is answered */
        JDXL_SRL_READ      = 1,     /* Ping and read instructions */
        JDXL_SRL_ALL       = 2      /* every unicast instruction (factory default) */
} jdxl_srl_t;

typedef struct {
        jdxl_srl_t status_return_level;
        bool       tx_echo;         /* the adapter echoes TX on RX: feed() skips the echo */
} jdxl_opts_t;

/* Diagnostics, never used for decisions. */
typedef struct {
        uint32_t status_ok;         /* status replies delivered */
        uint32_t exchanges_done;    /* exchanges whose last expected reply arrived */
        uint32_t missed;            /* expected replies that never arrived (skipped over,
                                       aborted, or superseded by a new build) */
        uint32_t bad_checksum;      /* 1.0 checksum / 2.0 CRC mismatch */
        uint32_t bad_length;        /* LENGTH impossible, or longer than the expected reply */
        uint32_t unexpected_id;     /* well-formed header from an ID nothing is waiting for */
        uint32_t not_status;        /* 2.0: instruction packet where a status was expected
                                       (typically an echo with tx_echo off) */
        uint32_t echo_mismatch;     /* tx_echo set, but the echo did not match what was sent */
        uint32_t ignored_bytes;     /* bytes fed with nothing pending */
} jdxl_debug_t;

/* Internal: one expected reply. */
#define JDXL_ANY_ID 0xFFFFu
typedef struct {
        uint16_t id;                /* expected responder, or JDXL_ANY_ID */
        uint16_t data_len;          /* requested data bytes */
} jdxl_slot_t;

#ifdef __cplusplus
}
#endif
#endif /* JDXL_H */