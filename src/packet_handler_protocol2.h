/*
 * ================================================================================================
 * FILE    : packet_handler_protocol2.h
 * BRIEF   : //TODO
 * ================================================================================================
 * Author  : aftito.faturohim@gmail.com
 * Created : 2026-08-04
 * Version : 0.1.2
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
 * ================================================================================================
 */

#include "string.h"
#include "stdint.h"

/* This defines the maximum length of a single packet that can be processed. It shall be adjusted
 * to the memory constraints of the target application or can be set to:
 *         UINT16_MAX + PKT_IDX_PARAMETER0
 * which is the maximum length of a packet including its headers that the DYNAMIXEL Protocol 2.0
 * can logically handle as of the time where this packet handler is written. 
 */
#define DXL_PH2_PKT_MAX_LEN 256

/* Protocol 2.0 packet structure */
#define DXL_PH2_PKT_IDX_HEADER0     0
#define DXL_PH2_PKT_IDX_HEADER1     1
#define DXL_PH2_PKT_IDX_HEADER2     2
#define DXL_PH2_PKT_IDX_RESERVED    3
#define DXL_PH2_PKT_IDX_ID          4
#define DXL_PH2_PKT_IDX_LENGTH_L    5
#define DXL_PH2_PKT_IDX_LENGTH_H    6
#define DXL_PH2_PKT_IDX_INSTRUCTION 7
#define DXL_PH2_PKT_IDX_ERROR       8
#define DXL_PH2_PKT_IDX_PARAMETER0  8
#define DXL_PH2_PKT_BYTE_HEADER_1   0xFF
#define DXL_PH2_PKT_BYTE_HEADER_2   0xFF
#define DXL_PH2_PKT_BYTE_HEADER_3   0xFD
#define DXL_PH2_PKT_BYTE_RSRVD      0x00

static const uint8_t DXL_PH2_PKT_HEADER_PATTERN[3] = {
        DXL_PH2_PKT_BYTE_HEADER_1,
        DXL_PH2_PKT_BYTE_HEADER_2,
        DXL_PH2_PKT_BYTE_HEADER_3
};

/* Protocol 2.0 instructions */
/* Enum for code clarity and type safety */
typedef enum {
    DXL_PH2_INST_PING = 0x01,
    DXL_PH2_INST_READ = 0x02,
    DXL_PH2_INST_WRITE = 0x03,
    DXL_PH2_INST_REG_WRITE = 0x04,
    DXL_PH2_INST_ACTION = 0x05,
    DXL_PH2_INST_FACTORY_RESET = 0x06,
    DXL_PH2_INST_REBOOT = 0x08,
    DXL_PH2_INST_CLEAR = 0x10,
    DXL_PH2_INST_CTRL_TABLE_BACKUP = 0x20,
    DXL_PH2_INST_STATUS = 0x55,
    DXL_PH2_INST_SYNC_READ = 0x82,
    DXL_PH2_INST_SYNC_WRITE = 0x83,
    DXL_PH2_INST_FAST_SYNC_READ = 0x8A,
    DXL_PH2_INST_BULK_READ = 0x92,
    DXL_PH2_INST_BULK_WRITE = 0x93,
    DXL_PH2_INST_FAST_BULK_READ = 0x9A
} dxl_ph2_inst_t;

/* Generic packet struct */
typedef struct {
        uint8_t  dxl_buffer[DXL_PH2_PKT_MAX_LEN];
        size_t   payload_len;
} dxl_ph2_pkt_t;

/* Inbound (RX) packet parser states */
enum dxl_ph2_inbound_parser_state {
        DXL_PH2_INBOUND_PARSER_STATE_RESET,
        DXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER,
        DXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND,
        DXL_PH2_INBOUND_PARSER_STATE_FEEDING
};

typedef enum {
        DXL_PH2_INBOUND_PARSER_SUCCESS,
        DXL_PH2_INBOUND_PARSER_NEED_MORE,
        DXL_PH2_INBOUND_PARSER_ERROR_INVALID_RSRVD,
        DXL_PH2_INBOUND_PARSER_ERROR_INVALID_ID,
        DXL_PH2_INBOUND_PARSER_ERROR_PARAM_TOO_LONG,
        DXL_PH2_INBOUND_PARSER_ERROR_NOT_A_STATUS_PKT,
        DXL_PH2_INBOUND_PARSER_ERROR_CRC_MISMATCH,
        DXL_PH2_INBOUND_PARSER_ERROR_CTX_STILL_RESET
} dxl_ph2_inbound_parser_return_t;

typedef enum {
        DXL_PH2_OUTBOUND_BUILDER_SUCCESS,
        DXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_ID,
        DXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_INST,
        DXL_PH2_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG,
        DXL_PH2_OUTBOUND_BUILDER_ERROR_STUFFING_TOO_LONG
} dxl_ph2_outbound_builder_return_t;

/* Inbound packet parser struct */
typedef struct {
        enum dxl_ph2_inbound_parser_state state;
        uint8_t                           pkt_header_seq_counter;
        uint8_t                           pkt_start_counter;
        uint16_t                          pkt_body_counter;
} dxl_ph2_inbound_parser_ctx_t;

/* Outbound (TX) packet builder */
dxl_ph2_outbound_builder_return_t dxl_ph2_build_tx(
        const uint8_t  id,
        const uint8_t  inst,
        const uint8_t  param[],
        const size_t   param_len,
        dxl_ph2_pkt_t* out_pkt
);

/* Inbound (RX) packet parser */
dxl_ph2_inbound_parser_return_t dxl_ph2_parse_rx(
        dxl_ph2_inbound_parser_ctx_t* parser_ctx,
        uint8_t*                      inbound_buf,
        size_t                        inbound_buf_len,
        size_t                        pkt_len_estimate,
        uint8_t                       skip_stuffing,
        size_t*                       last_idx_fed,
        dxl_ph2_pkt_t*                out_pkt
);
