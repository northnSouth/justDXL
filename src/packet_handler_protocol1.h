/*
 * ================================================================================================
 * FILE    : packet_handler_protocol1.h
 * BRIEF   : //TODO
 * ================================================================================================
 * Author  : aftito.faturohim@gmail.com
 * Created : 2026-09-18
 * Version : 0.0.0
 * ================================================================================================
 * License
 * -------
 * MIT License
 * 
 * Copyright (c) 2026 Aftito Nur Faturohim
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ================================================================================================
 * Description
 * -----------
 * //TODO
 * ================================================================================================
 * Changelog
 * ---------
 * 0.0.0 | 2026-08-04 | Initial code, WARNING: written by Claude Sonnet 5, untested 
 * ================================================================================================
 */

#ifndef PACKET_HANDLER_PROTOCOL1_H
#define PACKET_HANDLER_PROTOCOL1_H

#include <stdint.h>
#include <stddef.h>

// ================================================================================================
// Protocol area
// ================================================================================================

/* Maximum length of a single packet this handler can process, including header/ID/length/checksum.
 * The protocol's hard ceiling is 4 + 255 = 259 bytes (header(2) + ID(1) + LENGTH(1) + LENGTH's own
 * max value of 255, since LENGTH is a single byte). Lower this for memory-constrained targets, but
 * never raise it past 259 - the wire format cannot represent anything larger.
 */
#define JDXL_PH1_PKT_MAX_LEN (259)

/* Maximum count of status packets expected from a single instruction (Bulk Read across multiple
 * servos, or a broadcast Ping bus scan). Cannot exceed 254 (the valid ID range), and should be
 * sized to your bus's actual servo population plus memory budget.
 */
#define JDXL_PH1_MAX_STATUS_PKT_COUNT (8)

/* Maximum parameter bytes retained per servo in a cleaned-up status entry, after checksum
 * validation. Tune to the largest single control-table read your application performs.
 */
#define JDXL_PH1_CLEAN_STATUS_PARAMS_MAX_LEN (20)

/* Maximum data bytes per servo entry for Sync Write / Bulk Read instructions. */
#define JDXL_PH1_SYNC_BULK_DATA_MAX_LEN (8)

/* Protocol 1.0 packet structure indices.
 * ERROR shares INSTRUCTION's offset: a status packet has no instruction byte of its own, the
 * error bitmask simply occupies that same wire position.
 */
#define JDXL_PH1_PKT_IDX_HEADER0     0
#define JDXL_PH1_PKT_IDX_HEADER1     1
#define JDXL_PH1_PKT_IDX_ID          2
#define JDXL_PH1_PKT_IDX_LENGTH      3
#define JDXL_PH1_PKT_IDX_INSTRUCTION 4
#define JDXL_PH1_PKT_IDX_ERROR       4
#define JDXL_PH1_PKT_IDX_PARAMETER0  5

#define JDXL_PH1_PKT_BYTE_HEADER_1 0xFF
#define JDXL_PH1_PKT_BYTE_HEADER_2 0xFF

static const uint8_t JDXL_PH1_PKT_HEADER_PATTERN[2] = {
        JDXL_PH1_PKT_BYTE_HEADER_1,
        JDXL_PH1_PKT_BYTE_HEADER_2
};

/* Protocol 1.0 instructions
 * Source: DYNAMIXEL Protocol 1.0 spec, "Instruction" section.
 * Only instructions documented in the spec are listed - no Clear, Fast Sync/Bulk Read, or Bulk
 * Write exist in Protocol 1.0.
 */
enum jdxl_ph1_dxl_inst {
        JDXL_PH1_DXL_INST_PING          = 0x01,
        JDXL_PH1_DXL_INST_READ          = 0x02,
        JDXL_PH1_DXL_INST_WRITE         = 0x03,
        JDXL_PH1_DXL_INST_REG_WRITE     = 0x04,
        JDXL_PH1_DXL_INST_ACTION        = 0x05,
        JDXL_PH1_DXL_INST_FACTORY_RESET = 0x06,
        JDXL_PH1_DXL_INST_REBOOT        = 0x08,
        JDXL_PH1_DXL_INST_SYNC_WRITE    = 0x83,
        JDXL_PH1_DXL_INST_BULK_READ     = 0x92
};

/* Protocol 1.0 broadcast ID */
#define JDXL_PH1_DXL_BROADCAST_ID 0xFE

/* Sentinel passed as `expected_id` to jdxl_ph1_parse_inbound() to accept a status packet from
 * any responding ID. Deliberately outside uint8_t's range (unlike a real ID, which is always
 * 0x00-0xFE) so it can never collide with a legitimate wire value.
 */
#define JDXL_PH1_ANY_ID (0xFFFFu)

/* Protocol 1.0 status error field - a genuine bitmask, unlike Protocol 2.0's single-value enum.
 * Bit 7 is reserved/always-0 on the wire per spec; this handler repurposes it internally as a
 * "clean status params array overflowed" sentinel (see jdxl_ph1_clean_status_data_t), which is
 * never something a real DYNAMIXEL sets.
 */
typedef enum {
        JDXL_PH1_DXL_ERR_NONE                        = 0x00,
        JDXL_PH1_DXL_ERR_INPUT_VOLTAGE                = (1u << 0),
        JDXL_PH1_DXL_ERR_ANGLE_LIMIT                  = (1u << 1),
        JDXL_PH1_DXL_ERR_OVERHEATING                  = (1u << 2),
        JDXL_PH1_DXL_ERR_RANGE                        = (1u << 3),
        JDXL_PH1_DXL_ERR_CHECKSUM                     = (1u << 4),
        JDXL_PH1_DXL_ERR_OVERLOAD                     = (1u << 5),
        JDXL_PH1_DXL_ERR_INSTRUCTION                  = (1u << 6),
        JDXL_PH1_DXL_ERR_JDXL_STATUS_PARAMS_OVERFLOW  = (1u << 7)
} jdxl_ph1_dxl_err_t;

// ================================================================================================
// Packet handler area
// ================================================================================================

/* Generic packet struct */
typedef struct {
        uint8_t dxl_buffer[JDXL_PH1_PKT_MAX_LEN];
        size_t  payload_len;
} jdxl_ph1_pkt_t;

/* Inbound (RX) packet parser states */
enum jdxl_ph1_inbound_parser_state {
        JDXL_PH1_INBOUND_PARSER_STATE_RESET,
        JDXL_PH1_INBOUND_PARSER_STATE_LOOK_HEADER,
        JDXL_PH1_INBOUND_PARSER_STATE_PKT_HEADER_FOUND,
        JDXL_PH1_INBOUND_PARSER_STATE_FEEDING
};

/* Inbound packet parser struct */
typedef struct {
        enum jdxl_ph1_inbound_parser_state state;
        uint8_t  pkt_header_seq_counter;
        uint8_t  pkt_start_counter;
        uint16_t pkt_body_counter;
} jdxl_ph1_inbound_parser_ctx_t;

/* Inbound packet parser return codes */
typedef enum {
        JDXL_PH1_INBOUND_PARSER_SUCCESS,
        JDXL_PH1_INBOUND_PARSER_NEED_MORE,
        JDXL_PH1_INBOUND_PARSER_ERROR_INVALID_ID,
        JDXL_PH1_INBOUND_PARSER_ERROR_UNEXPECTED_ID,
        JDXL_PH1_INBOUND_PARSER_ERROR_INVALID_LENGTH,
        JDXL_PH1_INBOUND_PARSER_ERROR_PARAM_TOO_LONG,
        JDXL_PH1_INBOUND_PARSER_ERROR_CHECKSUM_MISMATCH,
        JDXL_PH1_INBOUND_PARSER_ERROR_CTX_STILL_RESET
} jdxl_ph1_inbound_parser_return_t;

/* Outbound packet builder return codes */
typedef enum {
        JDXL_PH1_OUTBOUND_BUILDER_SUCCESS,
        JDXL_PH1_OUTBOUND_BUILDER_ERROR_INVALID_ID,
        JDXL_PH1_OUTBOUND_BUILDER_ERROR_INVALID_INST,
        JDXL_PH1_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG
} jdxl_ph1_outbound_builder_return_t;

/* Outbound (TX) packet builder */
jdxl_ph1_outbound_builder_return_t jdxl_ph1_build_outbound(
        const uint8_t   id,
        const uint8_t   inst,
        const uint8_t   param[],
        const size_t    param_len,
        jdxl_ph1_pkt_t* out_pkt
);

/* Inbound (RX) packet parser.
 * `expected_id`: the responding ID this call should accept, or JDXL_PH1_ANY_ID to accept any
 * (used for broadcast Ping bus scans, where the responder set is not known ahead of time).
 */
jdxl_ph1_inbound_parser_return_t jdxl_ph1_parse_inbound(
        jdxl_ph1_inbound_parser_ctx_t* parser_ctx,
        const uint8_t*                 inbound_buf,
        const size_t                   inbound_buf_len,
        const size_t                   pkt_len_estimate,
        const uint16_t                 expected_id,
        size_t*                        last_idx_fed,
        jdxl_ph1_pkt_t*                out_pkt
);

// ================================================================================================
// API area
// ================================================================================================

typedef struct {
        uint8_t id;
        uint8_t err;    /* bitmask - see jdxl_ph1_dxl_err_t */
        uint8_t params[JDXL_PH1_CLEAN_STATUS_PARAMS_MAX_LEN];
        size_t  params_len;
} jdxl_ph1_clean_status_data_t;

/* Protocol 1.0 packet handler Sync Write instruction parameters */
typedef struct {
        uint8_t id;
        uint8_t data[JDXL_PH1_SYNC_BULK_DATA_MAX_LEN];
} jdxl_ph1_sync_w_param_t;

/* Protocol 1.0 packet handler Bulk Read instruction parameters.
 * Note the wire parameter order for Bulk Read is {Length, ID, Starting Address} per entry -
 * different from this struct's field order, which follows jdxl_ph2's convention instead for
 * naming consistency. jdxl_ph1_build_bulk_read() handles the reordering internally.
 */
typedef struct {
        uint8_t id;
        uint8_t addr;
        uint8_t data_len;
} jdxl_ph1_bulk_r_param_t;

/* Protocol 1.0 packet handler context, use one per DYNAMIXEL bus */
typedef struct {
        jdxl_ph1_pkt_t outbound_pkt;
        jdxl_ph1_clean_status_data_t status_data[JDXL_PH1_MAX_STATUS_PKT_COUNT];

        struct {
                jdxl_ph1_pkt_t inbound_pkt;
                uint8_t        prev_inst;
                uint8_t        prev_target_id; /* ID targeted by the last built instruction packet */

                /* Wire LENGTH field value expected for each status packet (== params + 2, i.e.
                 * ERROR + CHECKSUM). Indexed in the order responses are expected to arrive. */
                uint16_t expected_packets_param_len[JDXL_PH1_MAX_STATUS_PKT_COUNT];
                /* Which responding ID each expected status packet should come from.
                 * JDXL_PH1_ANY_ID for "unknown ahead of time" (broadcast Ping scans). */
                uint16_t expected_packet_ids[JDXL_PH1_MAX_STATUS_PKT_COUNT];
                uint8_t  expected_packet_count;
                uint8_t  expected_packet_idx;

                jdxl_ph1_inbound_parser_ctx_t in_parser_ctx;
                size_t last_idx_fed;
        } internals;

        struct {
                size_t success_done;
                size_t success_need_more;
                size_t err_rx_parser;
                size_t invalid_id;
                size_t unexpected_id;
                size_t invalid_length;
                size_t param_too_long;
                size_t checksum_mismatch;
                size_t ctx_still_reset;
                size_t parser_error_unknown;
        } debug;
} jdxl_ph1_ctx_t;

/* Protocol 1.0 packet handler build instruction return codes */
typedef enum {
        JDXL_PH1_BUILD_INST_SUCCESS,
        JDXL_PH1_BUILD_INST_ERR_TX_BUILDER,
        JDXL_PH1_BUILD_INST_ERR_INVALID_ID,
        JDXL_PH1_BUILD_INST_ERR_PARAM_TOO_LONG,
        JDXL_PH1_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO,
        JDXL_PH1_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE,
        JDXL_PH1_BUILD_INST_ERR_TARGET_SERVO_CANNOT_BE_ZERO,
        JDXL_PH1_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY
} jdxl_ph1_build_inst_return_t;

/* Protocol 1.0 packet handler build return codes */
typedef struct {
        jdxl_ph1_build_inst_return_t       build_inst;
        jdxl_ph1_outbound_builder_return_t tx_builder;
} jdxl_ph1_build_return_t;

/* Protocol 1.0 packet handler build ping instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_ping(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
);

/* Protocol 1.0 packet handler build ping broadcast instruction packet.
 * NOTE: broadcast Ping for bus discovery/scanning is not shown in the Protocol 1.0 spec's
 * examples, but uses the exact same packet mechanics as a unicast Ping - only the number of
 * responders is unknown ahead of time. Included for API parity with jdxl_ph2's equivalent.
 */
jdxl_ph1_build_return_t jdxl_ph1_build_ping_broadcast(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   target_servo_count
);

/* Protocol 1.0 packet handler build read instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_read(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data_len
);

/* Protocol 1.0 packet handler build write instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_write(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data[],
        const size_t    data_len
);

/* Protocol 1.0 packet handler build reg write instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_reg_write(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data[],
        const size_t    data_len
);

/* Protocol 1.0 packet handler build action instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_action(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
);

/* Protocol 1.0 packet handler build factory reset instruction packet.
 * Unlike Protocol 2.0, Protocol 1.0's Factory Reset takes no selectivity parameter - it always
 * resets the full Control Table.
 */
jdxl_ph1_build_return_t jdxl_ph1_build_factory_reset(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
);

/* Protocol 1.0 packet handler build reboot instruction packet */
jdxl_ph1_build_return_t jdxl_ph1_build_reboot(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
);

/* Protocol 1.0 packet handler build sync write instruction packet.
 * Always targets the broadcast ID (per spec); no status packets are expected in response.
 */
jdxl_ph1_build_return_t jdxl_ph1_build_sync_write(
        jdxl_ph1_ctx_t*         ctx,
        const uint8_t           addr,
        const uint8_t           data_len,
        jdxl_ph1_sync_w_param_t write_param[],
        uint8_t                 write_param_len
);

/* Protocol 1.0 packet handler build bulk read instruction packet.
 * Always targets the broadcast ID (per spec); expects one status packet per entry in
 * read_param[], in the same order, from each entry's `id`.
 */
jdxl_ph1_build_return_t jdxl_ph1_build_bulk_read(
        jdxl_ph1_ctx_t*          ctx,
        jdxl_ph1_bulk_r_param_t  read_param[],
        uint8_t                  read_param_len
);

/* Protocol 1.0 packet handler feed buffer return codes */
typedef enum {
        JDXL_PH1_FEED_BUF_SUCCESS_DONE,
        JDXL_PH1_FEED_BUF_SUCCESS_NEED_MORE,
        JDXL_PH1_FEED_BUF_ERR_RX_PARSER
} jdxl_ph1_feed_buf_return_t;

/* Protocol 1.0 packet handler feed return codes */
typedef struct {
        jdxl_ph1_feed_buf_return_t        feed_buf;
        jdxl_ph1_inbound_parser_return_t  rx_parser;
} jdxl_ph1_feed_return_t;

/* Protocol 1.0 packet handler feed on inbound buffer */
jdxl_ph1_feed_return_t jdxl_ph1_feed(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t*  in_buf,
        const size_t    in_buf_len
);

#endif