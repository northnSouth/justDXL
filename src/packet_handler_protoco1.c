/*
 * ================================================================================================
 * FILE    : packet_handler_protocol1.c
 * BRIEF   : Implementation for packet_handler_protocol1.h
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

#include "packet_handler_protocol1.h"
#include <string.h>

/* Lookup array for instruction validation */
static const uint8_t valid_insts[] = {
        JDXL_PH1_DXL_INST_PING,
        JDXL_PH1_DXL_INST_READ,
        JDXL_PH1_DXL_INST_WRITE,
        JDXL_PH1_DXL_INST_REG_WRITE,
        JDXL_PH1_DXL_INST_ACTION,
        JDXL_PH1_DXL_INST_FACTORY_RESET,
        JDXL_PH1_DXL_INST_REBOOT,
        JDXL_PH1_DXL_INST_SYNC_WRITE,
        JDXL_PH1_DXL_INST_BULK_READ
};
static const size_t valid_insts_len = sizeof(valid_insts) / sizeof(valid_insts[0]);

/* Checksum computation
 * Source: DYNAMIXEL Protocol 1.0 spec, "Checksum (Instruction Packet)" section.
 * Checksum = ~( ID + LENGTH + INSTRUCTION + Parameter1 + ... + Parameter N ), lower byte only.
 */
static uint8_t calc_checksum(
        const uint8_t* data_blk_ptr,
        uint16_t       data_blk_size
){
        uint16_t sum = 0;
        uint16_t i;

        for (i = 0; i < data_blk_size; i++)
        {
                sum += data_blk_ptr[i];
        }

        return (uint8_t)(~sum & 0xFFu);
}

/* Checksum TX helper - covers ID..last-param (offset JDXL_PH1_PKT_IDX_ID through the byte
 * immediately before the checksum byte itself). */
static void calc_checksum_to_txbuf(
        uint8_t* txbuf,
        uint16_t bufsize
){
        uint16_t chksum_idx = bufsize - 1;
        uint16_t cover_len  = chksum_idx - JDXL_PH1_PKT_IDX_ID;

        txbuf[chksum_idx] = calc_checksum(&txbuf[JDXL_PH1_PKT_IDX_ID], cover_len);
}

/* Checksum RX helper */
static uint8_t calc_checksum_from_rxbuf(
        const uint8_t* rxbuf,
        uint16_t       bufsize
){
        uint16_t cover_len = (bufsize - 1) - JDXL_PH1_PKT_IDX_ID;

        return calc_checksum(&rxbuf[JDXL_PH1_PKT_IDX_ID], cover_len);
}

/* TX packet builder */
jdxl_ph1_outbound_builder_return_t jdxl_ph1_build_outbound(
        const uint8_t   id,
        const uint8_t   inst,
        const uint8_t   param[],
        const size_t    param_len,
        jdxl_ph1_pkt_t* out_pkt
){
        /* ID must not be 0xFF - valid range is 0x00-0xFD (unicast) plus 0xFE (broadcast)
         * Source: DYNAMIXEL Protocol 1.0 spec, "Packet ID" section
         */
        if (id == 0xFF) return JDXL_PH1_OUTBOUND_BUILDER_ERROR_INVALID_ID;

        for (uint8_t i = 0; i < valid_insts_len; i++) {
                if (valid_insts[i] == inst) break;
                if (i == valid_insts_len - 1)
                        return JDXL_PH1_OUTBOUND_BUILDER_ERROR_INVALID_INST;
        }

        /* LENGTH is a single wire byte (max 255): LENGTH = param_len + 2 (INSTRUCTION + CHECKSUM).
         * This is a hard protocol ceiling, unlike Protocol 2.0's 16-bit LENGTH - there is no
         * wire-format room to widen this, so anything beyond 253 params is simply rejected. */
        if (param_len > 253) return JDXL_PH1_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG;

        uint16_t packet_body = (uint16_t)(param_len + 2);

        if ((size_t)packet_body + JDXL_PH1_PKT_IDX_INSTRUCTION > JDXL_PH1_PKT_MAX_LEN)
                return JDXL_PH1_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG;

        out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_ID]          = id;
        out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_LENGTH]      = (uint8_t)packet_body;
        out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_INSTRUCTION] = inst;
        if (param_len > 0)
                memcpy(&out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_PARAMETER0], param, param_len);

        out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_HEADER0] = JDXL_PH1_PKT_BYTE_HEADER_1;
        out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_HEADER1] = JDXL_PH1_PKT_BYTE_HEADER_2;

        out_pkt->payload_len = JDXL_PH1_PKT_IDX_INSTRUCTION + packet_body;
        calc_checksum_to_txbuf(out_pkt->dxl_buffer, (uint16_t)out_pkt->payload_len);

        return JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
}

/* Handle LOOK_HEADER state */
static uint8_t parser_handle_look_header(
        jdxl_ph1_inbound_parser_ctx_t* parser_ctx,
        const uint8_t*                 inbound_buf,
        const size_t                   inbound_buf_len,
        size_t*                        last_idx_fed,
        jdxl_ph1_pkt_t*                out_pkt
){
        for (; *last_idx_fed < inbound_buf_len; (*last_idx_fed)++) {
                uint8_t byte     = inbound_buf[*last_idx_fed];
                uint8_t expected = JDXL_PH1_PKT_HEADER_PATTERN[parser_ctx->pkt_header_seq_counter];

                if (byte == expected) {
                        parser_ctx->pkt_header_seq_counter++;

                        if (parser_ctx->pkt_header_seq_counter
                            == sizeof(JDXL_PH1_PKT_HEADER_PATTERN)
                        ) {
                                parser_ctx->pkt_header_seq_counter = 0;
                                parser_ctx->state = JDXL_PH1_INBOUND_PARSER_STATE_PKT_HEADER_FOUND;

                                memcpy(out_pkt->dxl_buffer,
                                       JDXL_PH1_PKT_HEADER_PATTERN,
                                       sizeof(JDXL_PH1_PKT_HEADER_PATTERN)
                                );
                                out_pkt->payload_len = sizeof(JDXL_PH1_PKT_HEADER_PATTERN);

                                (*last_idx_fed)++;
                                return 1;
                        }
                } else {
                        /* Both header bytes are 0xFF, so any run of 0xFF bytes keeps the
                         * sequence counter alive rather than resetting to 0. */
                        parser_ctx->pkt_header_seq_counter = (
                                byte == JDXL_PH1_PKT_HEADER_PATTERN[0]
                        ) ? 1 : 0;
                }
        }

        return 0;
}

/* Handle PKT_HEADER_FOUND state: ID and LENGTH bytes.
 * Unlike Protocol 2.0, there is no RSRVD byte and no INST validation here (status packets carry
 * no instruction marker), so this state only covers 2 bytes: ID, then LENGTH.
 */
static uint8_t parser_handle_pkt_header_found(
        jdxl_ph1_inbound_parser_ctx_t*     parser_ctx,
        const uint8_t*                     inbound_buf,
        const size_t                       inbound_buf_len,
        const size_t                       pkt_len_estimate,
        const uint16_t                     expected_id,
        size_t*                            last_idx_fed,
        jdxl_ph1_pkt_t*                    out_pkt,
        jdxl_ph1_inbound_parser_return_t*  parser_ret
){
        while (*last_idx_fed < inbound_buf_len) {
                uint8_t byte    = inbound_buf[*last_idx_fed];
                uint8_t counter = parser_ctx->pkt_start_counter;

                if (counter == 0) {  /* ID */
                        /* 0xFF is never a valid ID (range is 0x00-0xFE) - most likely this is
                         * still part of a longer run of header bytes, so bail out and let the
                         * caller resync rather than waiting for a checksum failure. */
                        if (byte == 0xFF) {
                                *parser_ret = JDXL_PH1_INBOUND_PARSER_ERROR_INVALID_ID;
                                return 1;
                        }
                        if (expected_id != JDXL_PH1_ANY_ID && byte != (uint8_t)expected_id) {
                                *parser_ret = JDXL_PH1_INBOUND_PARSER_ERROR_UNEXPECTED_ID;
                                return 1;
                        }
                }

                out_pkt->dxl_buffer[out_pkt->payload_len] = byte;
                out_pkt->payload_len++;
                parser_ctx->pkt_start_counter++;
                (*last_idx_fed)++;

                if (parser_ctx->pkt_start_counter >= 2) {  /* ID + LENGTH captured */
                        parser_ctx->pkt_start_counter = 0;

                        uint16_t body_len = out_pkt->dxl_buffer[JDXL_PH1_PKT_IDX_LENGTH];
                        uint32_t pkt_len  = (uint32_t)JDXL_PH1_PKT_IDX_INSTRUCTION + body_len;

                        /* Minimum valid LENGTH is 2 (INSTRUCTION/ERROR + CHECKSUM, zero params) */
                        if (body_len < 2) {
                                *parser_ret = JDXL_PH1_INBOUND_PARSER_ERROR_INVALID_LENGTH;
                                return 1;
                        }
                        if (pkt_len > pkt_len_estimate || pkt_len > JDXL_PH1_PKT_MAX_LEN) {
                                *parser_ret = JDXL_PH1_INBOUND_PARSER_ERROR_PARAM_TOO_LONG;
                                return 1;
                        }

                        parser_ctx->pkt_body_counter = body_len;
                        parser_ctx->state = JDXL_PH1_INBOUND_PARSER_STATE_FEEDING;

                        *parser_ret = JDXL_PH1_INBOUND_PARSER_NEED_MORE;
                        return 0;
                }
        }

        *parser_ret = JDXL_PH1_INBOUND_PARSER_NEED_MORE;
        return 1;
}

/* Handle FEEDING state: INSTRUCTION/ERROR byte, params, and CHECKSUM */
static void parser_handle_feeding(
        jdxl_ph1_inbound_parser_ctx_t*     parser_ctx,
        const uint8_t*                     inbound_buf,
        const size_t                       inbound_buf_len,
        size_t*                            last_idx_fed,
        jdxl_ph1_pkt_t*                    out_pkt,
        jdxl_ph1_inbound_parser_return_t*  parser_ret
){
        while (parser_ctx->pkt_body_counter > 0 && *last_idx_fed < inbound_buf_len) {
                out_pkt->dxl_buffer[out_pkt->payload_len] = inbound_buf[*last_idx_fed];
                out_pkt->payload_len++;
                parser_ctx->pkt_body_counter--;
                (*last_idx_fed)++;
        }

        if (parser_ctx->pkt_body_counter > 0) {
                *parser_ret = JDXL_PH1_INBOUND_PARSER_NEED_MORE;
                return;
        }

        uint8_t inbound_checksum    = out_pkt->dxl_buffer[out_pkt->payload_len - 1];
        uint8_t calculated_checksum = calc_checksum_from_rxbuf(
                out_pkt->dxl_buffer,
                (uint16_t)out_pkt->payload_len
        );

        if (inbound_checksum != calculated_checksum) {
                *parser_ret = JDXL_PH1_INBOUND_PARSER_ERROR_CHECKSUM_MISMATCH;
                return;
        }

        parser_ctx->state = JDXL_PH1_INBOUND_PARSER_STATE_RESET;
        *parser_ret = JDXL_PH1_INBOUND_PARSER_SUCCESS;
}

/* RX parser dispatcher */
jdxl_ph1_inbound_parser_return_t jdxl_ph1_parse_inbound(
        jdxl_ph1_inbound_parser_ctx_t* parser_ctx,
        const uint8_t*                 inbound_buf,
        const size_t                   inbound_buf_len,
        const size_t                   pkt_len_estimate,
        const uint16_t                 expected_id,
        size_t*                        last_idx_fed,
        jdxl_ph1_pkt_t*                out_pkt
){
        jdxl_ph1_inbound_parser_return_t ret = JDXL_PH1_INBOUND_PARSER_NEED_MORE;
        *last_idx_fed = 0;

        if (parser_ctx->state == JDXL_PH1_INBOUND_PARSER_STATE_RESET) {
                parser_ctx->state = JDXL_PH1_INBOUND_PARSER_STATE_LOOK_HEADER;
        }

        switch (parser_ctx->state) {
                case JDXL_PH1_INBOUND_PARSER_STATE_RESET:
                        ret = JDXL_PH1_INBOUND_PARSER_ERROR_CTX_STILL_RESET;
                        break;
                case JDXL_PH1_INBOUND_PARSER_STATE_LOOK_HEADER: {
                        uint8_t found = parser_handle_look_header(parser_ctx, inbound_buf,
                                                                  inbound_buf_len, last_idx_fed, out_pkt);
                        if (!found) break;
                        /* fall through to next state if header found */
                }
                case JDXL_PH1_INBOUND_PARSER_STATE_PKT_HEADER_FOUND: {
                        uint8_t done = parser_handle_pkt_header_found(parser_ctx, inbound_buf, inbound_buf_len,
                                                            pkt_len_estimate, expected_id, last_idx_fed, out_pkt, &ret);
                        if (done > 0) break;
                        /* fall through to next state if ID/LENGTH complete */
                }
                case JDXL_PH1_INBOUND_PARSER_STATE_FEEDING:
                        parser_handle_feeding(parser_ctx, inbound_buf, inbound_buf_len,
                                               last_idx_fed, out_pkt, &ret);
                        break;

                default:
                        ret = ret;
        }

        return ret;
}

// ================================================================================================
// Instruction builders
// ================================================================================================

jdxl_ph1_build_return_t jdxl_ph1_build_ping(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
){
        jdxl_ph1_build_return_t ret;

        if (id == JDXL_PH1_DXL_BROADCAST_ID) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_INVALID_ID;
                return ret;
        }

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, JDXL_PH1_DXL_INST_PING,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_PING;
        ctx->internals.prev_target_id = id;

        /* Ping status packet: H1 H2 ID LEN ERR CKSM, LEN = 0x02
         * Source: DYNAMIXEL Protocol 1.0 spec, "Ping" section
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_ping_broadcast(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   target_servo_count
){
        jdxl_ph1_build_return_t ret;
        uint8_t i;

        if (target_servo_count == 0) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TARGET_SERVO_CANNOT_BE_ZERO;
                return ret;
        }
        if (target_servo_count > JDXL_PH1_MAX_STATUS_PKT_COUNT) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY;
                return ret;
        }

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                JDXL_PH1_DXL_BROADCAST_ID, JDXL_PH1_DXL_INST_PING,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_PING;
        ctx->internals.prev_target_id = JDXL_PH1_DXL_BROADCAST_ID;

        ctx->internals.expected_packet_count = target_servo_count;
        ctx->internals.expected_packet_idx = 0;
        for (i = 0; i < target_servo_count; i++) {
                ctx->internals.expected_packets_param_len[i] = 2;
                /* responder set unknown ahead of a broadcast scan */
                ctx->internals.expected_packet_ids[i] = JDXL_PH1_ANY_ID;
        }

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_read(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data_len
){
        jdxl_ph1_build_return_t ret;

        if (id == JDXL_PH1_DXL_BROADCAST_ID) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_INVALID_ID;
                return ret;
        }

        uint8_t param[2] = { addr, data_len };

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, JDXL_PH1_DXL_INST_READ,
                param, sizeof(param), &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_READ;
        ctx->internals.prev_target_id = id;

        /* Read status packet: H1 H2 ID LEN ERR P1..PN CKSM, LEN = data_len + 2
         * Source: DYNAMIXEL Protocol 1.0 spec, "Read" section
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = (uint16_t)data_len + 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

/* Shared implementation for Write and Reg Write, which are wire-identical apart from the
 * instruction byte. */
static jdxl_ph1_build_return_t build_write_family(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data[],
        const size_t    data_len,
        const uint8_t   inst
){
        jdxl_ph1_build_return_t ret;

        /* minus CHECKSUM */
        uint8_t param[JDXL_PH1_PKT_MAX_LEN - JDXL_PH1_PKT_IDX_PARAMETER0 - 1];
        param[0] = addr;

        if (data_len > (sizeof(param) - 1)) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        if (data_len > 0)
                memcpy(param + 1, data, data_len);

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, inst,
                param, data_len + 1, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = inst;
        ctx->internals.prev_target_id = id;

        /* Write/Reg Write status packet: H1 H2 ID LEN ERR CKSM, LEN = 0x02
         * No status packet if ID is broadcast.
         * Source: DYNAMIXEL Protocol 1.0 spec, "Write" / "Reg Write" sections
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_write(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data[],
        const size_t    data_len
){
        return build_write_family(ctx, id, addr, data, data_len, JDXL_PH1_DXL_INST_WRITE);
}

jdxl_ph1_build_return_t jdxl_ph1_build_reg_write(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id,
        const uint8_t   addr,
        const uint8_t   data[],
        const size_t    data_len
){
        return build_write_family(ctx, id, addr, data, data_len, JDXL_PH1_DXL_INST_REG_WRITE);
}

jdxl_ph1_build_return_t jdxl_ph1_build_action(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
){
        jdxl_ph1_build_return_t ret;

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, JDXL_PH1_DXL_INST_ACTION,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_ACTION;
        ctx->internals.prev_target_id = id;

        /* No status packet if ID is broadcast.
         * Source: DYNAMIXEL Protocol 1.0 spec, "Action" section
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_factory_reset(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
){
        jdxl_ph1_build_return_t ret;

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, JDXL_PH1_DXL_INST_FACTORY_RESET,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_FACTORY_RESET;
        ctx->internals.prev_target_id = id;

        ctx->internals.expected_packet_count = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH1_DXL_BROADCAST_ID) ? 0 : 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_reboot(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t   id
){
        jdxl_ph1_build_return_t ret;

        if (id == JDXL_PH1_DXL_BROADCAST_ID) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_INVALID_ID;
                return ret;
        }

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                id, JDXL_PH1_DXL_INST_REBOOT,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_REBOOT;
        ctx->internals.prev_target_id = id;

        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packet_idx = 0;
        ctx->internals.expected_packets_param_len[0] = 2;
        ctx->internals.expected_packet_ids[0] = id;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_sync_write(
        jdxl_ph1_ctx_t*         ctx,
        const uint8_t           addr,
        const uint8_t           data_len,
        jdxl_ph1_sync_w_param_t write_param[],
        uint8_t                 write_param_len
){
        jdxl_ph1_build_return_t ret;
        uint8_t i, j;

        if (write_param_len == 0) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TARGET_SERVO_CANNOT_BE_ZERO;
                return ret;
        }
        if (data_len == 0) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        for (i = 0; i < write_param_len; i++) {
                for (j = (uint8_t)(i + 1); j < write_param_len; j++) {
                        if (write_param[i].id == write_param[j].id) {
                                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE;
                                return ret;
                        }
                }
        }

        /* minus CHECKSUM */
        uint8_t param[JDXL_PH1_PKT_MAX_LEN - JDXL_PH1_PKT_IDX_PARAMETER0 - 1];
        size_t  offset = 0;

        param[offset++] = addr;
        param[offset++] = data_len;

        for (i = 0; i < write_param_len; i++) {
                if (offset + 1 + data_len > sizeof(param)) {
                        ret.build_inst = JDXL_PH1_BUILD_INST_ERR_PARAM_TOO_LONG;
                        return ret;
                }
                param[offset++] = write_param[i].id;
                memcpy(&param[offset], write_param[i].data, data_len);
                offset += data_len;
        }

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                JDXL_PH1_DXL_BROADCAST_ID, JDXL_PH1_DXL_INST_SYNC_WRITE,
                param, offset, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_SYNC_WRITE;
        ctx->internals.prev_target_id = JDXL_PH1_DXL_BROADCAST_ID;

        /* Always broadcast - no status packets ever returned.
         * Source: DYNAMIXEL Protocol 1.0 spec, "Sync Write" section
         */
        ctx->internals.expected_packet_count = 0;
        ctx->internals.expected_packet_idx = 0;

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph1_build_return_t jdxl_ph1_build_bulk_read(
        jdxl_ph1_ctx_t*          ctx,
        jdxl_ph1_bulk_r_param_t  read_param[],
        uint8_t                  read_param_len
){
        jdxl_ph1_build_return_t ret;
        uint8_t i, j;

        if (read_param_len == 0) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TARGET_SERVO_CANNOT_BE_ZERO;
                return ret;
        }
        if (read_param_len > JDXL_PH1_MAX_STATUS_PKT_COUNT) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY;
                return ret;
        }

        for (i = 0; i < read_param_len; i++) {
                for (j = (uint8_t)(i + 1); j < read_param_len; j++) {
                        if (read_param[i].id == read_param[j].id) {
                                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE;
                                return ret;
                        }
                }
        }

        uint8_t param[1 + 3 * JDXL_PH1_MAX_STATUS_PKT_COUNT];
        uint8_t offset = 0;

        param[offset++] = 0x00;

        /* Wire order per entry is {Length of Data, ID, Starting Address} - note this differs
         * from this struct field order (id, addr, data_len), and from Protocol 2.0's Bulk Read
         * parameter layout.
         * Source: DYNAMIXEL Protocol 1.0 spec, "Bulk Read" section
         */
        for (i = 0; i < read_param_len; i++) {
                param[offset++] = read_param[i].data_len;
                param[offset++] = read_param[i].id;
                param[offset++] = read_param[i].addr;
        }

        jdxl_ph1_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph1_build_outbound(
                JDXL_PH1_DXL_BROADCAST_ID, JDXL_PH1_DXL_INST_BULK_READ,
                param, offset, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH1_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH1_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH1_DXL_INST_BULK_READ;
        ctx->internals.prev_target_id = JDXL_PH1_DXL_BROADCAST_ID;

        ctx->internals.expected_packet_count = read_param_len;
        ctx->internals.expected_packet_idx = 0;
        for (i = 0; i < read_param_len; i++) {
                ctx->internals.expected_packets_param_len[i] = (uint16_t)read_param[i].data_len + 2;
                ctx->internals.expected_packet_ids[i] = read_param[i].id;
        }

        ret.build_inst = JDXL_PH1_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH1_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

// ================================================================================================
// Feed / status extraction
// ================================================================================================

/* Extract the ID/ERR/params of the most recently parsed inbound packet into
 * ctx->status_data[ctx->internals.expected_packet_idx]. */
static void proc_single_stat_pkt(jdxl_ph1_ctx_t* ctx)
{
        jdxl_ph1_pkt_t* pkt = &ctx->internals.inbound_pkt;
        uint8_t idx = ctx->internals.expected_packet_idx;
        jdxl_ph1_clean_status_data_t* dst = &ctx->status_data[idx];

        uint8_t body_len   = pkt->dxl_buffer[JDXL_PH1_PKT_IDX_LENGTH];  /* ERR + params + CKSM */
        uint8_t params_len = (body_len >= 2) ? (uint8_t)(body_len - 2) : 0;

        dst->id  = pkt->dxl_buffer[JDXL_PH1_PKT_IDX_ID];
        dst->err = pkt->dxl_buffer[JDXL_PH1_PKT_IDX_ERROR];

        if (params_len > JDXL_PH1_CLEAN_STATUS_PARAMS_MAX_LEN) {
                dst->err |= JDXL_PH1_DXL_ERR_JDXL_STATUS_PARAMS_OVERFLOW;
                params_len = JDXL_PH1_CLEAN_STATUS_PARAMS_MAX_LEN;
        }

        memcpy(dst->params, &pkt->dxl_buffer[JDXL_PH1_PKT_IDX_PARAMETER0], params_len);
        dst->params_len = params_len;
}

jdxl_ph1_feed_return_t jdxl_ph1_feed(
        jdxl_ph1_ctx_t* ctx,
        const uint8_t*  in_buf,
        const size_t    in_buf_len
){
        jdxl_ph1_feed_return_t ret;

        if (ctx->internals.expected_packet_count == 0) {
                ret.feed_buf = JDXL_PH1_FEED_BUF_SUCCESS_DONE;
                return ret;
        }

        jdxl_ph1_inbound_parser_return_t parse_ret;

        if (ctx->internals.in_parser_ctx.state == JDXL_PH1_INBOUND_PARSER_STATE_RESET)
        {
                memset(&ctx->internals.inbound_pkt, 0, sizeof(ctx->internals.inbound_pkt));
        }

        size_t pkt_len_estimate = (size_t)ctx->internals.expected_packets_param_len[
                ctx->internals.expected_packet_idx
        ] + JDXL_PH1_PKT_IDX_INSTRUCTION;

        uint16_t expected_id = ctx->internals.expected_packet_ids[ctx->internals.expected_packet_idx];

        parse_ret = jdxl_ph1_parse_inbound(
                &ctx->internals.in_parser_ctx,
                in_buf, in_buf_len,
                pkt_len_estimate,
                expected_id,
                &ctx->internals.last_idx_fed,
                &ctx->internals.inbound_pkt
        );

        if (parse_ret == JDXL_PH1_INBOUND_PARSER_NEED_MORE) {
                ret.feed_buf = JDXL_PH1_FEED_BUF_SUCCESS_NEED_MORE;
                ret.rx_parser = parse_ret;
                return ret;
        };

        if (parse_ret != JDXL_PH1_INBOUND_PARSER_SUCCESS) {
                ctx->internals.in_parser_ctx.state = JDXL_PH1_INBOUND_PARSER_STATE_RESET;

                ret.feed_buf = JDXL_PH1_FEED_BUF_ERR_RX_PARSER;
                ret.rx_parser = parse_ret;
                return ret;
        }

        proc_single_stat_pkt(ctx);

        if (--ctx->internals.expected_packet_count > 0) {
                ctx->internals.expected_packet_idx++;

                ret.feed_buf = JDXL_PH1_FEED_BUF_SUCCESS_NEED_MORE;
                ret.rx_parser = parse_ret;
                return ret;
        };

        memset(
                ctx->internals.expected_packets_param_len, 0,
                sizeof(ctx->internals.expected_packets_param_len)
        );
        memset(
                ctx->internals.expected_packet_ids, 0,
                sizeof(ctx->internals.expected_packet_ids)
        );
        ctx->internals.expected_packet_count = 0;
        ctx->internals.expected_packet_idx = 0;

        ret.feed_buf = JDXL_PH1_FEED_BUF_SUCCESS_DONE;
        ret.rx_parser = parse_ret;
        return ret;
}