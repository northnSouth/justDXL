/*
 * ================================================================================================
 * FILE    : packet_handler_protocol2.c
 * BRIEF   : Implementation for packet_handler_protocol2.h 
 * ================================================================================================
 * Author  : aftito.faturohim@gmail.com
 * Created : 2026-08-04
 * Version : 0.4.3
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
 * 0.0.0 | 2026-08-04 | Initial code
 * 0.1.0 | 2026-08-04 | Simplification
 * 0.1.1 | 2026-08-06 | Refactor
 * 0.1.2 | 2026-08-06 | Comments
 * 0.2.0 | 2026-08-06 | API expansion, and ping wrapper function
 * 0.3.0 | 2026-08-07 | API redesign and ping
 * 0.4.0 | 2026-08-09 | API completion #1, untested. License fix
 * 0.4.1 | 2026-08-09 | API completion #2, tested virtually. Implemented packet len estimation
 * 0.4.2 | 2026-08-10 | Hot packet mechanism
 * 0.4.3 | 2026-08-10 | Added debug stuff, fixed stupid bug
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
    JDXL_PH2_DXL_INST_PING,
    JDXL_PH2_DXL_INST_READ,
    JDXL_PH2_DXL_INST_WRITE,
    JDXL_PH2_DXL_INST_REG_WRITE,
    JDXL_PH2_DXL_INST_ACTION,
    JDXL_PH2_DXL_INST_FACTORY_RESET,
    JDXL_PH2_DXL_INST_REBOOT,
    JDXL_PH2_DXL_INST_CLEAR,
    JDXL_PH2_DXL_INST_STATUS,
    JDXL_PH2_DXL_INST_SYNC_READ,
    JDXL_PH2_DXL_INST_SYNC_WRITE,
    JDXL_PH2_DXL_INST_FAST_SYNC_READ,
    JDXL_PH2_DXL_INST_BULK_READ,
    JDXL_PH2_DXL_INST_BULK_WRITE,
    JDXL_PH2_DXL_INST_FAST_BULK_READ
};
static const size_t valid_insts_len = sizeof(valid_insts) / sizeof(valid_insts[0]);

/* CRC computation 
 * Source: https://docs.robotis.com/docs/dxl/protocol/crc
 */
static unsigned short update_crc(
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
static void calc_crc_to_txbuf(
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
static uint16_t calc_crc_from_rxbuf(
        uint8_t* rxbuf,
        uint16_t bufsize
){
        return update_crc(0, rxbuf, bufsize - 2);
}

/* Stuffing insertion 
 * Source: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp
 */
static uint8_t add_stuffing(uint8_t *packet)
{
        int packet_length_in = BYTES_TO_U16(
                packet[JDXL_PH2_PKT_IDX_LENGTH_L],
                packet[JDXL_PH2_PKT_IDX_LENGTH_H]
        );

        int packet_length_out = packet_length_in;
        
        // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        if (packet_length_in < 8) return 0;

        uint8_t *packet_ptr;
        uint16_t packet_length_before_crc = packet_length_in - 2;
        for (uint16_t i = 3; i < packet_length_before_crc; i++)
        {
                packet_ptr = &packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION-2];
                if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
                        packet_length_out++;
        }

        // buffer will overflow, this reports it
        if ((packet_length_out + JDXL_PH2_PKT_IDX_INSTRUCTION) > JDXL_PH2_PKT_MAX_LEN) return 1;

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

        packet[JDXL_PH2_PKT_IDX_LENGTH_L] = U16_TO_LOWBYTE(packet_length_out);
        packet[JDXL_PH2_PKT_IDX_LENGTH_H] = U16_TO_HIGHBYTE(packet_length_out);
  
  return 0;
}

/* Stuffing removal
 * Source: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp
 */
static void rem_stuffing(uint8_t *packet)
{
        int i = 0, index = 0;
        int packet_length_in = BYTES_TO_U16(
                packet[JDXL_PH2_PKT_IDX_LENGTH_L],
                packet[JDXL_PH2_PKT_IDX_LENGTH_H]
        );

        int packet_length_out = packet_length_in;

        index = JDXL_PH2_PKT_IDX_INSTRUCTION;
        for (i = 0; i < packet_length_in - 2; i++)  // except CRC
        {
                if (packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION] == 0xFD 
                    && packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION+1] == 0xFD 
                    && packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION-1] == 0xFF 
                    && packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION-2] == 0xFF)
                {   // FF FF FD FD
                        packet_length_out--;
                        i++;
                }
                packet[index++] = packet[i+JDXL_PH2_PKT_IDX_INSTRUCTION];
        }
        packet[index++] = packet[JDXL_PH2_PKT_IDX_INSTRUCTION+packet_length_in-2];
        packet[index++] = packet[JDXL_PH2_PKT_IDX_INSTRUCTION+packet_length_in-1];

        packet[JDXL_PH2_PKT_IDX_LENGTH_L] = U16_TO_LOWBYTE(packet_length_out);
        packet[JDXL_PH2_PKT_IDX_LENGTH_H] = U16_TO_HIGHBYTE(packet_length_out);
}

/* TX packet builder */
jdxl_ph2_outbound_builder_return_t jdxl_ph2_build_outbound(
        const uint8_t   id,
        const uint8_t   inst,
        const uint8_t   param[],
        const size_t    param_len,
        jdxl_ph2_pkt_t* out_pkt
){
        /* ID must not be 0xFF or 0xFD 
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#packet-id
         */
        if (id == 0xFF || id == 0xFD) return JDXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_ID;

        for (uint8_t i = 0; i < valid_insts_len; i++) {
                if (valid_insts[i] == inst) break;
                if (i == valid_insts_len - 1) 
                        return JDXL_PH2_OUTBOUND_BUILDER_ERROR_INVALID_INST;
        }

        uint16_t packet_body = param_len + 3;

        if ((size_t)packet_body + JDXL_PH2_PKT_IDX_INSTRUCTION > JDXL_PH2_PKT_MAX_LEN) 
                return JDXL_PH2_OUTBOUND_BUILDER_ERROR_PARAM_TOO_LONG;

        uint8_t len_l = packet_body & 0xFF;
        uint8_t len_h = (packet_body >> 8) & 0xFF;
        
        // build packet without header and CRC
        out_pkt->dxl_buffer[4] = id;
        out_pkt->dxl_buffer[5] = len_l;
        out_pkt->dxl_buffer[6] = len_h;
        out_pkt->dxl_buffer[7] = inst;
        if (param_len > 0)
                memcpy(&out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_INSTRUCTION + 1], param, param_len);

        if (add_stuffing(out_pkt->dxl_buffer)) 
                return JDXL_PH2_OUTBOUND_BUILDER_ERROR_STUFFING_TOO_LONG;

        // update payload length after stuffing
        packet_body = BYTES_TO_U16(
                out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_LENGTH_L],
                out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_LENGTH_H]
        );

        // add header and CRC
        out_pkt->dxl_buffer[0] = JDXL_PH2_PKT_BYTE_HEADER_1;
        out_pkt->dxl_buffer[1] = JDXL_PH2_PKT_BYTE_HEADER_2;
        out_pkt->dxl_buffer[2] = JDXL_PH2_PKT_BYTE_HEADER_3;
        out_pkt->dxl_buffer[3] = JDXL_PH2_PKT_BYTE_RSRVD;
        
        out_pkt->payload_len = JDXL_PH2_PKT_IDX_INSTRUCTION + packet_body;
        calc_crc_to_txbuf(out_pkt->dxl_buffer, out_pkt->payload_len);
        
        return JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
}

/* Handle LOOK_HEADER state */
static jdxl_ph2_inbound_parser_return_t parser_handle_look_header(
        jdxl_ph2_inbound_parser_ctx_t* parser_ctx,
        const uint8_t* inbound_buf,
        const size_t inbound_buf_len,
        size_t* last_idx_fed,
        jdxl_ph2_pkt_t* out_pkt
){
        for (; *last_idx_fed < inbound_buf_len; (*last_idx_fed)++) {
                uint8_t byte = inbound_buf[*last_idx_fed];
                uint8_t expected = JDXL_PH2_PKT_HEADER_PATTERN[parser_ctx->pkt_header_seq_counter];

                if (byte == expected) {
                        parser_ctx->pkt_header_seq_counter++;
                        
                        if (parser_ctx->pkt_header_seq_counter 
                            == sizeof(JDXL_PH2_PKT_HEADER_PATTERN)
                        ) {
                                /* header found, initialize packet buffer and move to next state */
                                parser_ctx->pkt_header_seq_counter = 0;
                                parser_ctx->state = JDXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND;
                                
                                memcpy(out_pkt->dxl_buffer, 
                                       JDXL_PH2_PKT_HEADER_PATTERN, 
                                       sizeof(JDXL_PH2_PKT_HEADER_PATTERN)
                                );
                                out_pkt->payload_len = sizeof(JDXL_PH2_PKT_HEADER_PATTERN);
                                
                                (*last_idx_fed)++;  /* consume this byte and exit */
                                return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
                        }
                } else {
                        /* reset counter, and check if current byte starts new sequence */
                        parser_ctx->pkt_header_seq_counter = (
                                byte == JDXL_PH2_PKT_HEADER_PATTERN[0]
                        ) ? 1 : 0;
                }
        }

        return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Validate bytes after header before params */
static jdxl_ph2_inbound_parser_return_t validate_pkt_start_byte(
        uint8_t counter,
        uint8_t byte
){
        if (counter == 0) {
                /* RSRVD must be 0x00 */
                if (byte != JDXL_PH2_PKT_BYTE_RSRVD) 
                        return JDXL_PH2_INBOUND_PARSER_ERROR_INVALID_RSRVD;
        } else if (counter == 1) {
                /* ID must not be 0xFF or 0xFD 
                 * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#packet-id
                 */
                if (byte == 0xFF || byte == 0xFD) 
                        return JDXL_PH2_INBOUND_PARSER_ERROR_INVALID_ID;
        }
        
        return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Handle PKT_HEADER_FOUND state, bytes after header before params */
static jdxl_ph2_inbound_parser_return_t parser_handle_pkt_header_found(
        jdxl_ph2_inbound_parser_ctx_t* parser_ctx,
        const uint8_t* inbound_buf,
        const size_t inbound_buf_len,
        const size_t pkt_len_estimate,
        size_t* last_idx_fed,
        jdxl_ph2_pkt_t* out_pkt
){
        while (*last_idx_fed < inbound_buf_len) {
                uint8_t byte = inbound_buf[*last_idx_fed];
                uint8_t counter = parser_ctx->pkt_start_counter;

                /* validate starting bytes */
                if (counter < 5) {  /* RSRVD, ID, Length Low, Length High, INST */
                        jdxl_ph2_inbound_parser_return_t val_ret;
                        val_ret = validate_pkt_start_byte(counter, byte);
                        if (val_ret != JDXL_PH2_INBOUND_PARSER_NEED_MORE) return val_ret;
                }

                /* append byte to packet */
                out_pkt->dxl_buffer[out_pkt->payload_len] = byte;
                out_pkt->payload_len++;
                parser_ctx->pkt_start_counter++;
                (*last_idx_fed)++;

                if (parser_ctx->pkt_start_counter >= 5) {
                        parser_ctx->pkt_start_counter = 0;
                        
                        uint16_t body_len = (uint16_t)out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_LENGTH_L]
                                            | ((uint16_t)out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_LENGTH_H] << 8);
                        uint32_t pkt_len = body_len + JDXL_PH2_PKT_IDX_INSTRUCTION;

                        if (pkt_len > pkt_len_estimate || pkt_len > JDXL_PH2_PKT_MAX_LEN) {
                                return JDXL_PH2_INBOUND_PARSER_ERROR_PARAM_TOO_LONG;
                        }
                        if (out_pkt->dxl_buffer[JDXL_PH2_PKT_IDX_INSTRUCTION] != 0x55) {
                                return JDXL_PH2_INBOUND_PARSER_ERROR_NOT_A_STATUS_PKT;
                        }

                        parser_ctx->pkt_body_counter = body_len - 1;  /* minus INST */
                        parser_ctx->state = JDXL_PH2_INBOUND_PARSER_STATE_FEEDING;
                        return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
                }
        }

        return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
}

/* Handle FEEDING state, param bytes except INST since it is validated in HEADER_FOUND handler */
static jdxl_ph2_inbound_parser_return_t parser_handle_feeding(
        jdxl_ph2_inbound_parser_ctx_t* parser_ctx,
        const uint8_t* inbound_buf,
        const size_t inbound_buf_len,
        const uint8_t skip_stuffing,
        size_t* last_idx_fed,
        jdxl_ph2_pkt_t* out_pkt
){
        while (parser_ctx->pkt_body_counter > 0 && *last_idx_fed < inbound_buf_len) {
                out_pkt->dxl_buffer[out_pkt->payload_len] = inbound_buf[*last_idx_fed];
                out_pkt->payload_len++;
                parser_ctx->pkt_body_counter--;
                (*last_idx_fed)++;
        }

        if (parser_ctx->pkt_body_counter > 0) {
                return JDXL_PH2_INBOUND_PARSER_NEED_MORE;
        }

        uint16_t inbound_crc = ((uint16_t)out_pkt->dxl_buffer[out_pkt->payload_len - 1] << 8)
                | out_pkt->dxl_buffer[out_pkt->payload_len - 2];
        uint16_t calculated_crc = calc_crc_from_rxbuf(
                out_pkt->dxl_buffer,
                out_pkt->payload_len
        );

        if (inbound_crc != calculated_crc) {
                return JDXL_PH2_INBOUND_PARSER_ERROR_CRC_MISMATCH;
        }

        if (!skip_stuffing) {
                rem_stuffing(out_pkt->dxl_buffer);
        }

        parser_ctx->state = JDXL_PH2_INBOUND_PARSER_STATE_RESET;
        return JDXL_PH2_INBOUND_PARSER_SUCCESS;
}

/* RX parser dispatcher */
jdxl_ph2_inbound_parser_return_t jdxl_ph2_parse_inbound(
        jdxl_ph2_inbound_parser_ctx_t* parser_ctx,
        const uint8_t* inbound_buf,
        const size_t inbound_buf_len,
        const size_t pkt_len_estimate,
        const uint8_t skip_stuffing,
        size_t* last_idx_fed,
        jdxl_ph2_pkt_t* out_pkt
)
{
        jdxl_ph2_inbound_parser_return_t ret = JDXL_PH2_INBOUND_PARSER_NEED_MORE;
        *last_idx_fed = 0;

        if (parser_ctx->state == JDXL_PH2_INBOUND_PARSER_STATE_RESET) {
                parser_ctx->state = JDXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER;
        }

        switch (parser_ctx->state) {
                case JDXL_PH2_INBOUND_PARSER_STATE_RESET: 
                        ret = JDXL_PH2_INBOUND_PARSER_ERROR_CTX_STILL_RESET;
                        break;
                case JDXL_PH2_INBOUND_PARSER_STATE_LOOK_HEADER:
                        ret = parser_handle_look_header(parser_ctx, inbound_buf, inbound_buf_len, 
                                                       last_idx_fed, out_pkt);
                        if (ret != JDXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND) break;
                        /* fall through to next state if header found */

                case JDXL_PH2_INBOUND_PARSER_STATE_PKT_HEADER_FOUND:
                        ret = parser_handle_pkt_header_found(parser_ctx, inbound_buf, inbound_buf_len, 
                                                            pkt_len_estimate, last_idx_fed, out_pkt);
                        if (ret != JDXL_PH2_INBOUND_PARSER_STATE_FEEDING) break;
                        /* fall through to next state if header and starting bytes complete */

                case JDXL_PH2_INBOUND_PARSER_STATE_FEEDING:
                        ret = parser_handle_feeding(parser_ctx, inbound_buf, inbound_buf_len, 
                                                   skip_stuffing, last_idx_fed, out_pkt);
                        break;

                default:
                        ret = ret;
        }

        return ret;
}

/* Worst-case packet body length estimator */
uint16_t jdxl_ph2_estimate_worst_case_body_len(uint16_t body_len)
{
        if (body_len < 8) {
                return body_len;
        }

        uint16_t extra = (body_len - 3) / 3;
        uint32_t estimate = (uint32_t)body_len + extra;

        return (estimate > UINT16_MAX) ? UINT16_MAX : (uint16_t)estimate;
}

// ================================================================================================
// Instruction builders
// ================================================================================================

jdxl_ph2_build_return_t jdxl_ph2_build_ping(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t id
){
        jdxl_ph2_build_return_t ret;

        if (id == JDXL_PH2_DXL_BROADCAST_ID) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                return ret;
        } 

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_PING, 
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_PING;

        /* Ping status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | P1 | P2 | P3 | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#ping-0x01
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packets_param_len[0] = 7;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_ping_broadcast(
        jdxl_ph2_ctx_t *ctx, 
        const uint8_t target_servo_count
){
        jdxl_ph2_build_return_t ret;

        if (target_servo_count > 253) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_IMPOSSIBLE_SERVO_COUNT;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_PING, 
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_PING;

        /* Ping status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | P1 | P2 | P3 | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#ping-0x01
         */
        ctx->internals.expected_packet_count = target_servo_count;

        if (ctx->internals.expected_packet_count > JDXL_PH2_MAX_STATUS_PKT_COUNT) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY;
                ret.tx_builder = build_ret;
                return ret;
        }

        for (uint8_t i = 0; i < ctx->internals.expected_packet_count; i++)
                ctx->internals.expected_packets_param_len[i] = 7;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_read(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t id,
        const uint16_t addr,
        const uint16_t data_len
){
        jdxl_ph2_build_return_t ret;

        if (id == JDXL_PH2_DXL_BROADCAST_ID) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                return ret;
        } 

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        uint8_t param[4] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr),
                U16_TO_LOWBYTE(data_len),
                U16_TO_HIGHBYTE(data_len)
        };
        
        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_READ, 
                param, sizeof(param), &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_READ;

        /* Read status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | Pn... | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#read-0x02
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packets_param_len[0] = 4 + data_len;
        
        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
};

jdxl_ph2_build_return_t jdxl_ph2_build_write(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t id,
        const uint16_t addr,
        const uint8_t data[],
        const size_t data_len
){
        jdxl_ph2_build_return_t ret;

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr)
        };
        
        if (data_len > (sizeof(param) - 2)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        memcpy(param + 2, data, data_len);
        
        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_WRITE, 
                param, data_len + 2, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_WRITE;
        
        /* Write status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#write-0x03
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_reg_write(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t id,
        const uint16_t addr,
        const uint8_t data[],
        const size_t data_len
){
        jdxl_ph2_build_return_t ret;

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr)
        };

        if (data_len > (sizeof(param) - 2)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        memcpy(param + 2, data, data_len);

        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_REG_WRITE,
                param, data_len + 2, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_REG_WRITE;
        
        /* Reg Write status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#reg-write-0x04
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_action(jdxl_ph2_ctx_t *ctx, const uint8_t id)
{
        jdxl_ph2_build_return_t ret;

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_ACTION,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_ACTION;

        /* Action status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#action-0x05
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_factory_reset(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t id,
        jdxl_ph2_dxl_factory_reset_t byte
){
        jdxl_ph2_build_return_t ret;

        if (id == JDXL_PH2_DXL_BROADCAST_ID && byte == 0xFF) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                return ret;
        }

        if (byte != 0xFF && byte != 0x01 && byte != 0x02) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_MODE;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        uint8_t param[1] = {byte};

        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_FACTORY_RESET,
                param, sizeof(param), &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_FACTORY_RESET;

         /* Factory reset status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#factory-reset-0x06
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_reboot(jdxl_ph2_ctx_t *ctx, const uint8_t id)
{
        jdxl_ph2_build_return_t ret;

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_REBOOT,
                NULL, 0, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_REBOOT;

        /* Reboot status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#reboot-0x08
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_clear(
        jdxl_ph2_ctx_t* ctx,
        const uint8_t id,
        const jdxl_ph2_dxl_clear_t clear_mode
){
        jdxl_ph2_build_return_t ret;
        uint8_t param[5] = {0};

        if (clear_mode == JDXL_PH2_DXL_CLEAR_POS) {
                param[0] = 0x01;
                param[1] = 0x44;
                param[2] = 0x58;
                param[3] = 0x4C;
                param[4] = 0x22;
        } else if (clear_mode == JDXL_PH2_DXL_CLEAR_ERR) {
                param[0] = 0x02;
                param[1] = 0x45;
                param[2] = 0x52;
                param[3] = 0x43;
                param[4] = 0x4C;
        } else {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_MODE;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        build_ret = jdxl_ph2_build_outbound(
                id, JDXL_PH2_DXL_INST_CLEAR,
                param, sizeof(param), &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_CLEAR;

        /* Clear status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | CRC 1 | CRC 2
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#clear-0x10
         */
        ctx->internals.expected_packet_count = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 1;
        ctx->internals.expected_packets_param_len[0] = (id == JDXL_PH2_DXL_BROADCAST_ID) ? 0 : 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_sync_read(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t ids[],
        const uint8_t ids_len,
        const uint16_t addr,
        const uint16_t data_len
){
        jdxl_ph2_build_return_t ret;

        if (ids_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr),
                U16_TO_LOWBYTE(data_len),
                U16_TO_HIGHBYTE(data_len)
        };

        if (ids_len > (sizeof(param) - 4)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        memcpy(param, ids, ids_len);
        
        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_SYNC_READ, 
                param, 4 + ids_len, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_SYNC_READ;
        
        /* Sync read status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | Pn... | CRC 1 | CRC 2
         * ids_len times with different IDs
         * status packets come in instruction order, guaranteed by protocol.
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#sync-read-0x82
         */
        ctx->internals.expected_packet_count = ids_len;
        
        if (ctx->internals.expected_packet_count > JDXL_PH2_MAX_STATUS_PKT_COUNT) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY;
                ret.tx_builder = build_ret;
                return ret;
        }

        for (uint8_t i = 0; i < ctx->internals.expected_packet_count; i++)
                ctx->internals.expected_packets_param_len[i] = data_len + 4;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_sync_write(
        jdxl_ph2_ctx_t *ctx,
        uint16_t addr,
        const uint16_t data_len,
        jdxl_ph2_sync_w_param_t write_param[],
        uint8_t write_param_len
){
        jdxl_ph2_build_return_t ret;

        if (write_param_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr),
                U16_TO_LOWBYTE(data_len),
                U16_TO_HIGHBYTE(data_len)
        };

        uint32_t params_chunk = data_len + 1; // id + data bytes

        if (data_len > JDXL_PH2_SYNC_BULK_DATA_WRITE_MAX_LEN) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_WRITE_DATA_LEN_TOO_LONG;
                return ret;
        }

        if ((params_chunk * write_param_len) > (sizeof(param) - 4)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        for (uint8_t i = 0; i < write_param_len; i++) {
                uint8_t offset = 4 + i * params_chunk;
                const uint8_t id = write_param[i].id;

                if (id > 252) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                        return ret;
                }

                param[offset] = id;
                memcpy(&param[offset + 1], write_param[i].data, data_len);
        }
        
        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_SYNC_WRITE, 
                param, 4 + (params_chunk * write_param_len), &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_SYNC_WRITE;
        ctx->internals.expected_packet_count = 0;

        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_fast_sync_read(
        jdxl_ph2_ctx_t *ctx,
        const uint8_t ids[],
        const uint8_t ids_len,
        const uint16_t addr,
        const uint16_t data_len
){
        jdxl_ph2_build_return_t ret;

        if (ids_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {
                U16_TO_LOWBYTE(addr),
                U16_TO_HIGHBYTE(addr),
                U16_TO_LOWBYTE(data_len),
                U16_TO_HIGHBYTE(data_len)
        };

        if (ids_len > (sizeof(param) - 4)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));
        
        memcpy(param, ids, ids_len);
        
        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_FAST_SYNC_READ, 
                param, 4 + ids_len, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_FAST_SYNC_READ;

        /* Fast sync read status packet:
         * H1 | H2 | H3 | RSRV | BROADCAST ID | LEN1 | LEN2 | INST | ERR | ID1 | Dn... | CRC 1 | CRC 2
         * then,
         * ERR | IDx | Dn... | CRC 1 | CRC 2
         * ids_len - 1 times with different IDs
         * CRC of the whole sequence is at the last 2 bytes of the last status packet, 
         * guaranteed by protocol.
         * status packets come in instruction order, guaranteed by protocol.
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#fast-sync-read-0x8a
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packets_param_len[0] = 1 + ((2 + data_len + 2) * ids_len);
        
        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

jdxl_ph2_build_return_t jdxl_ph2_build_bulk_read(
        jdxl_ph2_ctx_t *ctx,
        jdxl_ph2_bulk_r_param_t read_param[],
        uint8_t read_param_len
){
        jdxl_ph2_build_return_t ret;

        if (read_param_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {0};

        // ID, L_ADDR, H_ADDR, L_DATA_LEN, H_DATA_LEN
        if ((5 * read_param_len) > sizeof(param)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        // 256-bit presence bitmap for duplicate ID check
        uint32_t seen[8] = {0};
        for (uint8_t i = 0; i < read_param_len; i++) {
                const uint8_t id = read_param[i].id;
                
                if (id > 252) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                        return ret;
                }
                
                // duplicate found
                if (seen[id >> 5] & (1u << (id & 31))) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE;
                        return ret;
                }

                seen[id >> 5] |= (1u << (id & 31));
        }

        size_t off = 0;
        for (uint8_t i = 0; i < read_param_len; i++) {
                param[off++] = read_param[i].id;
                param[off++] = U16_TO_LOWBYTE(read_param[i].addr);
                param[off++] = U16_TO_HIGHBYTE(read_param[i].addr);
                param[off++] = U16_TO_LOWBYTE(read_param[i].data_len);
                param[off++] = U16_TO_HIGHBYTE(read_param[i].data_len);
        }

        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_BULK_READ, 
                param, off, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_BULK_READ;

        /* Bulk read status packet:
         * H1 | H2 | H3 | RSRV | ID | LEN1 | LEN2 | INST | ERR | Pn... | CRC 1 | CRC 2
         * read_param_len times with different IDs and data lengths
         * status packets come in instruction order, guaranteed by protocol.
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#bulk-read-0x92
         */
        ctx->internals.expected_packet_count = read_param_len;
        
        if (ctx->internals.expected_packet_count > JDXL_PH2_MAX_STATUS_PKT_COUNT) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_EXPECTED_STATUS_PKT_TOO_MANY;
                ret.tx_builder = build_ret;
                return ret;
        }

        for (uint8_t i = 0; i < ctx->internals.expected_packet_count; i++)
                ctx->internals.expected_packets_param_len[i] = read_param[i].data_len + 4;
        
        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
};

jdxl_ph2_build_return_t jdxl_ph2_build_bulk_write(
        jdxl_ph2_ctx_t* ctx,
        jdxl_ph2_bulk_w_param_t write_param[],
        uint8_t write_param_len
){
        jdxl_ph2_build_return_t ret;

        if (write_param_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {0};

        // data_len counter for bounds check
        size_t data_len_count = 0;

        // 256-bit presence bitmap for duplicate ID check
        uint32_t seen[8] = {0};
        for (uint8_t i = 0; i < write_param_len; i++) {
                const uint8_t id = write_param[i].id;
                
                if (id > 252) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                        return ret;
                }

                if (write_param[i].data_len > JDXL_PH2_SYNC_BULK_DATA_WRITE_MAX_LEN) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_WRITE_DATA_LEN_TOO_LONG;
                        return ret;
                }
                
                // duplicate found
                if (seen[id >> 5] & (1u << (id & 31))) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE;
                        return ret;
                }

                seen[id >> 5] |= (1u << (id & 31));

                data_len_count += write_param[i].data_len;
        }

        // ID, L_ADDR, H_ADDR, L_DATA_LEN, H_DATA_LEN + DATA_BYTES
        if ((5 * write_param_len) + data_len_count > sizeof(param)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        size_t off = 0;
        for (uint8_t i = 0; i < write_param_len; i++) {
                param[off++] = write_param[i].id;
                param[off++] = U16_TO_LOWBYTE(write_param[i].addr);
                param[off++] = U16_TO_HIGHBYTE(write_param[i].addr);
                param[off++] = U16_TO_LOWBYTE(write_param[i].data_len);
                param[off++] = U16_TO_HIGHBYTE(write_param[i].data_len);

                if (write_param[i].data_len == 0) continue;

                memcpy(&param[off], write_param[i].data, write_param[i].data_len);
                off += write_param[i].data_len;
        }

        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_BULK_WRITE, 
                param, off, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_BULK_WRITE;
        ctx->internals.expected_packet_count = 0;
        
        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
};

jdxl_ph2_build_return_t jdxl_ph2_build_fast_bulk_read(
        jdxl_ph2_ctx_t *ctx,
        jdxl_ph2_bulk_r_param_t read_param[],
        uint8_t read_param_len
){
        jdxl_ph2_build_return_t ret;

        if (read_param_len == 0) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_CANNOT_BE_ZERO;
                return ret;
        }

        // minus CRC
        uint8_t param[JDXL_PH2_PKT_MAX_LEN - JDXL_PH2_PKT_IDX_PARAMETER0 - 2] = {0};

        // ID, L_ADDR, H_ADDR, L_DATA_LEN, H_DATA_LEN
        if ((5 * read_param_len) > sizeof(param)) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_TOO_LONG;
                return ret;
        }

        jdxl_ph2_outbound_builder_return_t build_ret;
        memset(&ctx->outbound_pkt, 0, sizeof(ctx->outbound_pkt));

        // 256-bit presence bitmap for duplicate ID check
        uint32_t seen[8] = {0};
        for (uint8_t i = 0; i < read_param_len; i++) {
                const uint8_t id = read_param[i].id;
                
                if (id > 252) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_INVALID_ID;
                        return ret;
                }
                
                // duplicate found
                if (seen[id >> 5] & (1u << (id & 31))) {
                        ret.build_inst = JDXL_PH2_BUILD_INST_ERR_PARAM_ID_CANNOT_BE_DUPLICATE;
                        return ret;
                }

                seen[id >> 5] |= (1u << (id & 31));
        }

        size_t off = 0;
        for (uint8_t i = 0; i < read_param_len; i++) {
                param[off++] = read_param[i].id;
                param[off++] = U16_TO_LOWBYTE(read_param[i].addr);
                param[off++] = U16_TO_HIGHBYTE(read_param[i].addr);
                param[off++] = U16_TO_LOWBYTE(read_param[i].data_len);
                param[off++] = U16_TO_HIGHBYTE(read_param[i].data_len);
        }

        build_ret = jdxl_ph2_build_outbound(
                JDXL_PH2_DXL_BROADCAST_ID, JDXL_PH2_DXL_INST_FAST_BULK_READ, 
                param, off, &ctx->outbound_pkt
        );

        if (build_ret != JDXL_PH2_OUTBOUND_BUILDER_SUCCESS) {
                ret.build_inst = JDXL_PH2_BUILD_INST_ERR_TX_BUILDER;
                ret.tx_builder = build_ret;
                return ret;
        }

        ctx->internals.prev_inst = JDXL_PH2_DXL_INST_FAST_BULK_READ;

        /* Fast Bulk Read status packet:
         * H1 | H2 | H3 | RSRV | BROADCAST ID | LEN1 | LEN2 | INST | ERR | ID1 | Pn... | CRC1 | CRC2
         * then,
         * ERR | IDx | Pn... | CRC1 | CRC2
         * read_param_len - 1 times, each with different IDs data lengths
         * CRC of the whole sequence is at the last 2 bytes of the last status packet, guaranteed by protocol.
         * status packets come in instruction order, guaranteed by protocol.
         * Source: https://docs.robotis.com/docs/dxl/protocol/protocol2/#fast-bulk-read-0x9a
         */
        ctx->internals.expected_packet_count = 1;
        ctx->internals.expected_packets_param_len[0] = 1;
        for (uint8_t i = 0; i < read_param_len; i++) {
                ctx->internals.expected_packets_param_len[0] += (2 + read_param[i].data_len + 2);
        }
        
        ret.build_inst = JDXL_PH2_BUILD_INST_SUCCESS;
        ret.tx_builder = JDXL_PH2_OUTBOUND_BUILDER_SUCCESS;
        return ret;
}

// ================================================================================================
// Status packet feeder
// ================================================================================================

jdxl_ph2_feed_return_t jdxl_ph2_feed(
        jdxl_ph2_ctx_t *ctx, 
        const uint8_t *in_buf, 
        const size_t in_buf_len,
        uint8_t* is_packet_available
){
        jdxl_ph2_feed_return_t ret;
        *is_packet_available = 0;

        if (ctx->internals.expected_packet_count == 0 || ctx->internals.prev_inst == 0) {
                ret.feed_buf = JDXL_PH2_FEED_BUF_SUCCESS_DONE;
                return ret;
        };

        jdxl_ph2_inbound_parser_return_t parse_ret;

        if (ctx->internals.in_parser_ctx.state == JDXL_PH2_INBOUND_PARSER_STATE_RESET)
        {
                memset(&ctx->inbound_pkt, 0, sizeof(ctx->inbound_pkt));
        }

        uint8_t skip_stuffing = (ctx->internals.prev_inst == JDXL_PH2_DXL_INST_FAST_SYNC_READ
                                 || ctx->internals.prev_inst == JDXL_PH2_DXL_INST_FAST_BULK_READ
                                ) ? 1 : 0;
        
        size_t pkt_len_estimate = jdxl_ph2_estimate_worst_case_body_len(
                ctx->internals.expected_packets_param_len[ctx->internals.expected_packet_idx]
        ) + JDXL_PH2_PKT_IDX_INSTRUCTION;

        parse_ret = jdxl_ph2_parse_inbound(
                &ctx->internals.in_parser_ctx,
                in_buf, in_buf_len, 
                pkt_len_estimate,
                skip_stuffing, &ctx->internals.last_idx_fed, 
                &ctx->inbound_pkt
        );

        if (parse_ret == JDXL_PH2_INBOUND_PARSER_NEED_MORE) {
                ret.feed_buf = JDXL_PH2_FEED_BUF_SUCCESS_NEED_MORE;
                ret.rx_parser = parse_ret;
                return ret;
        };

        if (parse_ret != JDXL_PH2_INBOUND_PARSER_SUCCESS) {
                ctx->internals.in_parser_ctx.state = JDXL_PH2_INBOUND_PARSER_STATE_RESET;
                
                ret.feed_buf = JDXL_PH2_FEED_BUF_ERR_RX_PARSER;
                ret.rx_parser = parse_ret;
                return ret;
        }

        /* Due to the single-packet limit nature of the context packet buffer, the higher-level
         * code must process one packet at a time, and later on feed for more packets.
         * //TODO: someone please design a better mechanism
         */
        *is_packet_available = 1;

        if (--ctx->internals.expected_packet_count > 0) {
                ctx->internals.expected_packet_idx++;

                ret.feed_buf = JDXL_PH2_FEED_BUF_SUCCESS_PACKET_HOT;
                ret.rx_parser = parse_ret;
                return ret;
        };

        ctx->internals.prev_inst = 0;
        memset(
                ctx->internals.expected_packets_param_len, 0, 
                sizeof(ctx->internals.expected_packets_param_len)
        );
        ctx->internals.expected_packet_count = 0;
        ctx->internals.expected_packet_idx = 0;

        ret.feed_buf = JDXL_PH2_FEED_BUF_SUCCESS_DONE;
        ret.rx_parser = parse_ret;
        return ret;
}