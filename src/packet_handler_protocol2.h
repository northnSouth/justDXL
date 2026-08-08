/*
 * ================================================================================================
 * FILE    : packet_handler_protocol2.h
 * BRIEF   : //TODO
 * ================================================================================================
 * Author  : aftito.faturohim@gmail.com
 * Created : 2026-08-04
 * Version : 0.3.0
 * ================================================================================================
 * License
 * -------
 * MIT License
 * 
 * Copyright (c) 2026 North::ftr
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
 * 0.0.0 | 2026-08-04 | Initial code
 * 0.1.0 | 2026-08-04 | Simplification
 * 0.1.1 | 2026-08-06 | Refactor
 * 0.1.2 | 2026-08-06 | Comments
 * 0.2.0 | 2026-08-06 | API expansion, and ping wrapper function
 * 0.3.0 | 2026-08-07 | API redesign and ping
 * ================================================================================================
 */

#ifndef PACKET_HANDLER_PROTOCOL2_H
#define PACKET_HANDLER_PROTOCOL2_H

#include "string.h"
#include "stdint.h"

/* This defines the maximum length of a single packet that can be processed. It shall be adjusted
 * to the memory constraints of the target application or can be set to:
 *         UINT16_MAX + PKT_IDX_PARAMETER0
 * which is the maximum length of a packet including its headers that the DYNAMIXEL Protocol 2.0
 * can logically handle as of the time where this packet handler is written. 
 */
#define JDXL_PH2_PKT_MAX_LEN 256

/* Protocol 2.0 packet structure */
#define JDXL_PH2_PKT_IDX_HEADER0     0
#define JDXL_PH2_PKT_IDX_HEADER1     1
#define JDXL_PH2_PKT_IDX_HEADER2     2
#define JDXL_PH2_PKT_IDX_RESERVED    3
#define JDXL_PH2_PKT_IDX_ID          4
#define JDXL_PH2_PKT_IDX_LENGTH_L    5
#define JDXL_PH2_PKT_IDX_LENGTH_H    6
#define JDXL_PH2_PKT_IDX_INSTRUCTION 7
#define JDXL_PH2_PKT_IDX_ERROR       8
#define JDXL_PH2_PKT_IDX_PARAMETER0  8
#define JDXL_PH2_PKT_BYTE_HEADER_1   0xFF
#define JDXL_PH2_PKT_BYTE_HEADER_2   0xFF
#define JDXL_PH2_PKT_BYTE_HEADER_3   0xFD
#define JDXL_PH2_PKT_BYTE_RSRVD      0x00

static const uint8_t JDXL_PH2_PKT_HEADER_PATTERN[3] = {
        JDXL_PH2_PKT_BYTE_HEADER_1,
        JDXL_PH2_PKT_BYTE_HEADER_2,
        JDXL_PH2_PKT_BYTE_HEADER_3
};

/* Protocol 2.0 instructions */
/* CTRL_TABLE_BACKUP not implemented by the official SDK */
enum jdxl_ph2_dxl_inst {
        JDXL_PH2_DXL_INST_PING = 0x01,
        JDXL_PH2_DXL_INST_READ = 0x02,
        JDXL_PH2_DXL_INST_WRITE = 0x03,
        JDXL_PH2_DXL_INST_REG_WRITE = 0x04,
        JDXL_PH2_DXL_INST_ACTION = 0x05,
        JDXL_PH2_DXL_INST_FACTORY_RESET = 0x06,
        JDXL_PH2_DXL_INST_REBOOT = 0x08,
        JDXL_PH2_DXL_INST_CLEAR = 0x10,
        // JDXL_PH2_DXL_INST_CTRL_TABLE_BACKUP = 0x20,
        JDXL_PH2_DXL_INST_STATUS = 0x55,
        JDXL_PH2_DXL_INST_SYNC_READ = 0x82,
        JDXL_PH2_DXL_INST_SYNC_WRITE = 0x83,
        JDXL_PH2_DXL_INST_FAST_SYNC_READ = 0x8A,
        JDXL_PH2_DXL_INST_BULK_READ = 0x92,
        JDXL_PH2_DXL_INST_BULK_WRITE = 0x93,
        JDXL_PH2_DXL_INST_FAST_BULK_READ = 0x9A
};

typedef enum {
        JDXL_PH2_DXL_ERR_NONE,
        JDXL_PH2_DXL_ERR_RESULT_FAIL,
        JDXL_PH2_DXL_ERR_INSTRUCTION_ERROR,
        JDXL_PH2_DXL_ERR_CRC_ERROR,
        JDXL_PH2_DXL_ERR_DATA_RANGE_ERROR,
        JDXL_PH2_DXL_ERR_DATA_LENGTH_ERROR,
        JDXL_PH2_DXL_ERR_DATA_LIMIT_ERROR,
        JDXL_PH2_DXL_ERR_ACCESS_ERROR
} jdxl_ph2_dxl_err_t;

/* Protocol 2.0 special bytes */
typedef enum {
        JDXL_PH2_DXL_FACTORY_RESET_ALL = 0xFF,
        JDXL_PH2_DXL_FACTORY_RESET_ALL_BUT_ID = 0x01,
        JDXL_PH2_DXL_FACTORY_RESET_ALL_BUT_ID_AND_BAUDRATE = 0x02
} jdxl_ph2_dxl_factory_reset_t;

typedef enum {
        JDXL_PH2_DXL_CLEAR_POS = 0x01,
        JDXL_PH2_DXL_CLEAR_ERR
} jdxl_ph2_dxl_clear_t;

/* Generic packet struct */
typedef struct {
        uint8_t  dxl_buffer[JDXL_PH2_PKT_MAX_LEN];
        size_t   payload_len;
} jdxl_ph2_pkt_t;

/* Inbound (RX) packet parser states */
enum jdxl_ph2_inbound_parser_state {
        JDXL_PH2_INBOUND_PARSER_STATE_RESET,
        JDXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER,
        JDXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND,
        JDXL_PH2_INBOUND_PARSER_STATE_FEEDING
};

/* Inbound packet parser struct */
typedef struct {
        enum jdxl_ph2_inbound_parser_state state;
        uint8_t                            pkt_header_seq_counter;
        uint8_t                            pkt_start_counter;
        uint16_t                           pkt_body_counter;
} jdxl_ph2_inbound_parser_ctx_t;

typedef enum {
        JDXL_PH2_INBOUND_PARSER_SUCCESS,
        JDXL_PH2_INBOUND_PARSER_NEED_MORE,
        JDXL_PH2_INBOUND_PARSER_ERROR_INVALID_RSRVD,
        JDXL_PH2_INBOUND_PARSER_ERROR_INVALID_ID,
        JDXL_PH2_INBOUND_PARSER_ERROR_PARAM_TOO_LONG,
        JDXL_PH2_INBOUND_PARSER_ERROR_NOT_A_STATUS_PKT,
        JDXL_PH2_INBOUND_PARSER_ERROR_CRC_MISMATCH,
        JDXL_PH2_INBOUND_PARSER_ERROR_CTX_STILL_RESET
} jdxl_ph2_inbound_parser_return_t;

typedef enum {
        JDXL_PH2_OUTBOUND_BUILDER_SUCCESS,
        JDXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_ID,
        JDXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_INST,
        JDXL_PH2_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG,
        JDXL_PH2_OUTBOUND_BUILDER_ERROR_STUFFING_TOO_LONG
} jdxl_ph2_outbound_builder_return_t;

/* Outbound (TX) packet builder */
jdxl_ph2_outbound_builder_return_t jdxl_ph2_build_outbound(
        const uint8_t   id,
        const uint8_t   inst,
        const uint8_t   param[],
        const size_t    param_len,
        jdxl_ph2_pkt_t* out_pkt
);

/* Inbound (RX) packet parser */
jdxl_ph2_inbound_parser_return_t jdxl_ph2_parse_inbound(
        jdxl_ph2_inbound_parser_ctx_t* parser_ctx,
        const uint8_t*                 inbound_buf,
        const size_t                   inbound_buf_len,
        const size_t                   pkt_len_estimate,
        const uint8_t                  skip_stuffing,
        size_t*                        last_idx_fed,
        jdxl_ph2_pkt_t*                out_pkt
);

/* Worst-case packet body length estimator
 *
 * Given a body length (INST + params + CRC, i.e. param_len + 3, the same
 * value jdxl_ph2_build_tx computes internally as `packet_body`), returns an
 * upper bound on what that body length could grow to after byte stuffing,
 * without needing to actually run jdxl_ph2_add_stuffing on real data.
 *
 * Worst case is a repeating FF FF FD pattern: that 3-byte sequence has no
 * self-overlap, so occurrences can never be packed closer than 3 bytes apart,
 * giving a maximum of floor((body_len - 3) / 3) stuffed bytes. Below a body
 * length of 8, jdxl_ph2_add_stuffing's own minimum-length guard means nothing
 * can be stuffed at all, regardless of content.
 */
uint16_t jdxl_ph2_estimate_worst_case_body_len(uint16_t body_len);

typedef struct {
        jdxl_ph2_pkt_t outbound_pkt;
        jdxl_ph2_pkt_t inbound_pkt;

        struct {
                uint8_t prev_inst;
                jdxl_ph2_inbound_parser_ctx_t in_parser_ctx;
                size_t last_idx_fed;
        } internals;
} jdxl_ph2_ctx_t;

uint8_t jdxl_ph2_build_ping(jdxl_ph2_ctx_t* ctx, const uint8_t id);
// TODO: jdxl_ph2_build_ping_broadcast();
uint8_t jdxl_ph2_build_read(jdxl_ph2_ctx_t *ctx, const uint8_t id, const uint16_t addr, const uint16_t data_len);
uint8_t jdxl_ph2_build_write(jdxl_ph2_ctx_t *ctx, const uint8_t id, const uint16_t addr, const uint8_t data[], const size_t data_len);
uint8_t jdxl_ph2_build_reg_write(jdxl_ph2_ctx_t *ctx, const uint8_t id, const uint16_t addr, const uint8_t data[], const size_t data_len);
uint8_t jdxl_ph2_build_action(jdxl_ph2_ctx_t *ctx, const uint8_t id);
uint8_t jdxl_ph2_build_factory_reset(jdxl_ph2_ctx_t *ctx, const uint8_t id, jdxl_ph2_dxl_factory_reset_t byte);
uint8_t jdxl_ph2_build_reboot(jdxl_ph2_ctx_t *ctx, const uint8_t id);
uint8_t jdxl_ph2_build_clear(jdxl_ph2_ctx_t* ctx, const uint8_t id, const jdxl_ph2_dxl_clear_t clear_mode);
// TODO: jdxl_ph2_build_sync_read();
// TODO: jdxl_ph2_build_sync_write();
// TODO: jdxl_ph2_build_fast_sync_read();
// TODO: jdxl_ph2_build_bulk_read();
// TODO: jdxl_ph2_build_bulk_write();
// TODO: jdxl_ph2_build_fast_sync_write();

uint8_t jdxl_ph2_feed(jdxl_ph2_ctx_t* ctx, const uint8_t* in_buf, const size_t in_buf_len);

#endif