/*
 * ================================================================================================
 * FILE    : packet_handler_protocol2.c
 * BRIEF   : Implementation for packet_handler_protocol2.h 
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

#include "packet_handler_protocol2.h"
#include <stdint.h>
#include <string.h>

#define BYTES_TO_U16(lo, hi) ((uint16_t)((uint16_t)(lo) | ((uint16_t)(hi) << 8)))
#define U16_TO_LOWBYTE(w) ((uint8_t)(w))
#define U16_TO_HIGHBYTE(w) ((uint8_t)((w) >> 8))

/* Lookup array for instruction validation */
static const uint8_t valid_insts[] = {
    DXL_PH2_INST_PING,
    DXL_PH2_INST_READ,
    DXL_PH2_INST_WRITE,
    DXL_PH2_INST_REG_WRITE,
    DXL_PH2_INST_ACTION,
    DXL_PH2_INST_FACTORY_RESET,
    DXL_PH2_INST_REBOOT,
    DXL_PH2_INST_CLEAR,
    DXL_PH2_INST_CTRL_TABLE_BACKUP,
    DXL_PH2_INST_STATUS,
    DXL_PH2_INST_SYNC_READ,
    DXL_PH2_INST_SYNC_WRITE,
    DXL_PH2_INST_FAST_SYNC_READ,
    DXL_PH2_INST_BULK_READ,
    DXL_PH2_INST_BULK_WRITE,
    DXL_PH2_INST_FAST_BULK_READ
};

static const size_t valid_insts_len = sizeof(valid_insts) / sizeof(valid_insts[0]);

/* CRC computation 
 * Source: https://docs.robotis.com/docs/dxl/protocol/crc
 */
unsigned short update_crc(
        unsigned short crc_accum,
        unsigned char *data_blk_ptr,
        unsigned short data_blk_size
){
        unsigned short i, j;
        static const unsigned short crc_table[256] = {
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
        };

        for(j = 0; j < data_blk_size; j++)
        {
                i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
                crc_accum = (crc_accum << 8) ^ crc_table[i];
        }

        return crc_accum;
}

/* CRC TX helper */
static void helper_calc_crc_to_txbuf(
        uint8_t* txbuf,
        uint16_t bufsize
){
        uint16_t crc_start = bufsize - 2;
        uint16_t crc_value = update_crc(0, txbuf, crc_start);
        uint8_t crc_l = crc_value & 0x00FF;
        uint8_t crc_h = (crc_value >> 8) & 0x00FF;

        txbuf[crc_start] = (uint8_t)crc_l;
        txbuf[crc_start + 1] = (uint8_t)crc_h;
}

/* CRC RX helper */
static uint16_t helper_calc_crc_from_rxbuf(
        uint8_t* rxbuf,
        uint16_t bufsize
){
        return update_crc(0, rxbuf, bufsize - 2);
}

/* Stuffing insertion 
 * Source: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp
 */
static uint8_t dxl_ph2_add_stuffing(uint8_t *packet)
{
        int packet_length_in = BYTES_TO_U16(
                packet[DXL_PH2_PKT_IDX_LENGTH_L],
                packet[DXL_PH2_PKT_IDX_LENGTH_H]
        );

        int packet_length_out = packet_length_in;
        
        // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        if (packet_length_in < 8) return 0;

        uint8_t *packet_ptr;
        uint16_t packet_length_before_crc = packet_length_in - 2;
        for (uint16_t i = 3; i < packet_length_before_crc; i++)
        {
                packet_ptr = &packet[i+DXL_PH2_PKT_IDX_INSTRUCTION-2];
                if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
                        packet_length_out++;
        }

        // buffer will overflow, this reports it
        if ((packet_length_out + DXL_PH2_PKT_IDX_INSTRUCTION) > DXL_PH2_PKT_MAX_LEN) return 1;

        // no stuffing required
        if (packet_length_in == packet_length_out) return 0;

        uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
        uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc
        while (out_index != in_index)
        {
                if (packet[in_index] == 0xFD 
                    && packet[in_index-1] == 0xFF 
                    && packet[in_index-2] == 0xFF
                ){
                        packet[out_index--] = 0xFD; // byte stuffing
                        if (out_index != in_index)
                        {
                                packet[out_index--] = packet[in_index--]; // FD
                                packet[out_index--] = packet[in_index--]; // FF
                                packet[out_index--] = packet[in_index--]; // FF
                        }
                }
                else
                {
                        packet[out_index--] = packet[in_index--];
                }
        }

        packet[DXL_PH2_PKT_IDX_LENGTH_L] = U16_TO_LOWBYTE(packet_length_out);
        packet[DXL_PH2_PKT_IDX_LENGTH_H] = U16_TO_HIGHBYTE(packet_length_out);
  
  return 0;
}

/* Stuffing removal
 * Source: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp
 */
static void dxl_ph2_rem_stuffing(uint8_t *packet)
{
        int i = 0, index = 0;
        int packet_length_in = BYTES_TO_U16(
                packet[DXL_PH2_PKT_IDX_LENGTH_L],
                packet[DXL_PH2_PKT_IDX_LENGTH_H]
        );

        int packet_length_out = packet_length_in;

        index = DXL_PH2_PKT_IDX_INSTRUCTION;
        for (i = 0; i < packet_length_in - 2; i++)  // except CRC
        {
                if (packet[i+DXL_PH2_PKT_IDX_INSTRUCTION] == 0xFD 
                    && packet[i+DXL_PH2_PKT_IDX_INSTRUCTION+1] == 0xFD 
                    && packet[i+DXL_PH2_PKT_IDX_INSTRUCTION-1] == 0xFF 
                    && packet[i+DXL_PH2_PKT_IDX_INSTRUCTION-2] == 0xFF)
                {   // FF FF FD FD
                        packet_length_out--;
                        i++;
                }
                packet[index++] = packet[i+DXL_PH2_PKT_IDX_INSTRUCTION];
        }
        packet[index++] = packet[DXL_PH2_PKT_IDX_INSTRUCTION+packet_length_in-2];
        packet[index++] = packet[DXL_PH2_PKT_IDX_INSTRUCTION+packet_length_in-1];

        packet[DXL_PH2_PKT_IDX_LENGTH_L] = U16_TO_LOWBYTE(packet_length_out);
        packet[DXL_PH2_PKT_IDX_LENGTH_H] = U16_TO_HIGHBYTE(packet_length_out);
}

/* TX packet builder */
dxl_ph2_outbound_builder_return_t dxl_ph2_build_tx(
        const uint8_t id,
        const uint8_t inst,
        const uint8_t param[],
        const size_t param_len,
        dxl_ph2_pkt_t* out_pkt
){
        /* ID must not be 0xFF or 0xFD 
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#packet-id
         */
        if (id == 0xFF || id == 0xFD) return DXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_ID;

        for (uint8_t i = 0; i < valid_insts_len; i++) {
                if (valid_insts[i] == inst) break;
                if (i == valid_insts_len - 1) 
                        return DXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_INST;
        }

        uint16_t packet_body = param_len + 3;

        if ((size_t)packet_body + DXL_PH2_PKT_IDX_INSTRUCTION > DXL_PH2_PKT_MAX_LEN) 
                return DXL_PH2_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG;

        uint8_t len_l = packet_body & 0xFF;
        uint8_t len_h = (packet_body >> 8) & 0xFF;
        
        // build packet without header and CRC
        out_pkt->dxl_buffer[4] = id;
        out_pkt->dxl_buffer[5] = len_l;
        out_pkt->dxl_buffer[6] = len_h;
        out_pkt->dxl_buffer[7] = inst;
        memcpy(&out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_INSTRUCTION + 1], param, param_len);

        if (dxl_ph2_add_stuffing(out_pkt->dxl_buffer)) 
                return DXL_PH2_OUTBOUND_BUILDER_ERROR_STUFFING_TOO_LONG;

        // update payload length after stuffing
        packet_body = BYTES_TO_U16(
                out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_L],
                out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_H]
        );

        // add header and CRC
        out_pkt->dxl_buffer[0] = DXL_PH2_PKT_BYTE_HEADER_1;
        out_pkt->dxl_buffer[1] = DXL_PH2_PKT_BYTE_HEADER_2;
        out_pkt->dxl_buffer[2] = DXL_PH2_PKT_BYTE_HEADER_3;
        out_pkt->dxl_buffer[3] = DXL_PH2_PKT_BYTE_RSRVD;
        
        out_pkt->payload_len = DXL_PH2_PKT_IDX_INSTRUCTION + packet_body;
        helper_calc_crc_to_txbuf(out_pkt->dxl_buffer, out_pkt->payload_len);
        
        return DXL_PH2_OUTBOUND_BUILDER_SUCCESS;
}

/* Handle LOOK_HEADER state */
static dxl_ph2_inbound_parser_return_t parser_handle_look_header(
        dxl_ph2_inbound_parser_ctx_t* parser_ctx,
        uint8_t* inbound_buf,
        size_t inbound_buf_len,
        size_t* last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
)
{
        for (; *last_idx_fed < inbound_buf_len; (*last_idx_fed)++) {
                uint8_t byte = inbound_buf[*last_idx_fed];
                uint8_t expected = DXL_PH2_PKT_HEADER_PATTERN[parser_ctx->pkt_header_seq_counter];

                if (byte == expected) {
                        parser_ctx->pkt_header_seq_counter++;
                        
                        if (parser_ctx->pkt_header_seq_counter == sizeof(DXL_PH2_PKT_HEADER_PATTERN)) {
                                /* header found, initialize packet buffer and move to next state */
                                parser_ctx->pkt_header_seq_counter = 0;
                                parser_ctx->state = DXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND;
                                
                                memcpy(out_pkt->dxl_buffer, 
                                       DXL_PH2_PKT_HEADER_PATTERN, 
                                       sizeof(DXL_PH2_PKT_HEADER_PATTERN)
                                );
                                out_pkt->payload_len = sizeof(DXL_PH2_PKT_HEADER_PATTERN);
                                
                                (*last_idx_fed)++;  /* consume this byte and exit */
                                return DXL_PH2_INBOUND_PARSER_NEED_MORE;
                        }
                } else {
                        /* reset counter, and check if current byte starts new sequence */
                        parser_ctx->pkt_header_seq_counter = (byte == DXL_PH2_PKT_HEADER_PATTERN[0]) ? 1 : 0;
                }
        }

        return DXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Validate bytes after header before params */
static dxl_ph2_inbound_parser_return_t validate_pkt_start_byte(
        uint8_t counter,
        uint8_t byte
)
{
        if (counter == 0) {
                /* RSRVD must be 0x00 */
                if (byte != DXL_PH2_PKT_BYTE_RSRVD) return DXL_PH2_INBOUND_PARSER_ERROR_INVALID_RSRVD;
        } else if (counter == 1) {
                /* ID must not be 0xFF or 0xFD 
                 * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#packet-id
                 */
                if (byte == 0xFF || byte == 0xFD) return DXL_PH2_INBOUND_PARSER_ERROR_INVALID_ID;
        }
        
        return DXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Handle PKT_HEADER_FOUND state, bytes after header before params */
static dxl_ph2_inbound_parser_return_t parser_handle_pkt_header_found(
        dxl_ph2_inbound_parser_ctx_t* parser_ctx,
        uint8_t* inbound_buf,
        size_t inbound_buf_len,
        size_t pkt_len_estimate,
        size_t* last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
)
{
        while (*last_idx_fed < inbound_buf_len) {
                uint8_t byte = inbound_buf[*last_idx_fed];
                uint8_t counter = parser_ctx->pkt_start_counter;

                /* validate starting bytes */
                if (counter < 5) {  /* RSRVD, ID, Length Low, Length High, INST */
                        dxl_ph2_inbound_parser_return_t val_ret = validate_pkt_start_byte(counter, byte);
                        if (val_ret != DXL_PH2_INBOUND_PARSER_NEED_MORE) return val_ret;
                }

                /* append byte to packet */
                out_pkt->dxl_buffer[out_pkt->payload_len] = byte;
                out_pkt->payload_len++;
                parser_ctx->pkt_start_counter++;
                (*last_idx_fed)++;

                if (parser_ctx->pkt_start_counter >= 5) {
                        uint16_t body_len = (uint16_t)out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_L] 
                                            | ((uint16_t)out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_H] << 8);
                        uint32_t pkt_len = body_len + DXL_PH2_PKT_IDX_INSTRUCTION;

                        if (pkt_len > pkt_len_estimate || pkt_len > DXL_PH2_PKT_MAX_LEN) {
                                return DXL_PH2_INBOUND_PARSER_ERROR_PARAM_TOO_LONG;
                        }
                        if (out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_INSTRUCTION] != 0x55) {
                                return DXL_PH2_INBOUND_PARSER_ERROR_NOT_A_STATUS_PKT;
                        }

                        parser_ctx->pkt_start_counter = 0;
                        parser_ctx->pkt_body_counter = body_len - 1;  /* minus INST */
                        parser_ctx->state = DXL_PH2_INBOUND_PARSER_STATE_FEEDING;
                        return DXL_PH2_INBOUND_PARSER_NEED_MORE;
                }
        }

        return DXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Handle FEEDING state, param bytes except INST since it is validated in HEADER_FOUND handler */
static dxl_ph2_inbound_parser_return_t parser_handle_feeding(
        dxl_ph2_inbound_parser_ctx_t* parser_ctx,
        uint8_t* inbound_buf,
        size_t inbound_buf_len,
        uint8_t skip_stuffing,
        size_t* last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
)
{
        while (parser_ctx->pkt_body_counter > 0 && *last_idx_fed < inbound_buf_len) {
                out_pkt->dxl_buffer[out_pkt->payload_len] = inbound_buf[*last_idx_fed];
                out_pkt->payload_len++;
                parser_ctx->pkt_body_counter--;
                (*last_idx_fed)++;
        }

        if (parser_ctx->pkt_body_counter > 0) {
                return DXL_PH2_INBOUND_PARSER_NEED_MORE;
        }

        uint16_t inbound_crc = ((uint16_t)out_pkt->dxl_buffer[out_pkt->payload_len - 1] << 8)
                | out_pkt->dxl_buffer[out_pkt->payload_len - 2];
        uint16_t calculated_crc = helper_calc_crc_from_rxbuf(
                out_pkt->dxl_buffer,
                out_pkt->payload_len
        );

        if (inbound_crc != calculated_crc) {
                return DXL_PH2_INBOUND_PARSER_ERROR_CRC_MISMATCH;
        }

        if (!skip_stuffing) {
                dxl_ph2_rem_stuffing(out_pkt->dxl_buffer);
        }

        parser_ctx->state = DXL_PH2_INBOUND_PARSER_STATE_RESET;
        return DXL_PH2_INBOUND_PARSER_SUCCESS;
}

/* RX parser dispatcher */
dxl_ph2_inbound_parser_return_t dxl_ph2_parse_rx(
        dxl_ph2_inbound_parser_ctx_t* parser_ctx,
        uint8_t* inbound_buf,
        size_t inbound_buf_len,
        size_t pkt_len_estimate,
        uint8_t skip_stuffing,
        size_t* last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
)
{
        dxl_ph2_inbound_parser_return_t ret = DXL_PH2_INBOUND_PARSER_NEED_MORE;
        *last_idx_fed = 0;

        if (parser_ctx->state == DXL_PH2_INBOUND_PARSER_STATE_RESET) {
                parser_ctx->state = DXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER;
        }

        switch (parser_ctx->state) {
                case DXL_PH2_INBOUND_PARSER_STATE_RESET: 
                        ret = DXL_PH2_INBOUND_PARSER_ERROR_CTX_STILL_RESET;
                        break;
                case DXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER:
                        ret = parser_handle_look_header(parser_ctx, inbound_buf, inbound_buf_len, 
                                                       last_idx_fed, out_pkt);
                        if (ret != DXL_PH2_INBOUND_PARSER_NEED_MORE) break;
                        /* fall through to next state if header found */

                case DXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND:
                        ret = parser_handle_pkt_header_found(parser_ctx, inbound_buf, inbound_buf_len, 
                                                            pkt_len_estimate, last_idx_fed, out_pkt);
                        if (ret != DXL_PH2_INBOUND_PARSER_NEED_MORE) break;
                        /* fall through to next state if header and starting bytes complete */

                case DXL_PH2_INBOUND_PARSER_STATE_FEEDING:
                        ret = parser_handle_feeding(parser_ctx, inbound_buf, inbound_buf_len, 
                                                   skip_stuffing, last_idx_fed, out_pkt);
                        break;

                default:
                        ret = ret;
        }

        return ret;
}

/* Worst-case packet body length estimator */
uint16_t dxl_ph2_estimate_worst_case_body_len(uint16_t body_len)
{
        if (body_len < 8) {
                return body_len;
        }

        uint16_t extra = (body_len - 3) / 3;
        uint32_t estimate = (uint32_t)body_len + extra;

        return (estimate > UINT16_MAX) ? UINT16_MAX : (uint16_t)estimate;
}