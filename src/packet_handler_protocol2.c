/*
 * ================================================================================================
 TODO: File name
 TODO: Brief
 * ================================================================================================
 *  Author  : aftito.faturohim@gmail.com
 TODO: Creation date
 TODO: Version
 * ================================================================================================
 *  License
 *  -------
 TODO: License
 * ================================================================================================
 *  Description
 *  -----------
 TODO: Description
 * ================================================================================================
 *  Changelog
 *  ---------
 TODO: Changelog
 * ================================================================================================
 */

#include "packet_handler_protocol2.h"

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

/* Stuffing insertion */
// TODO: Notes on the difference between my stuffing and official ROBOTIS DynamixelSDK stuffing
void dxl_ph2_add_stuffing(uint8_t *packet)
{
    uint8_t *packet_ptr;
    uint16_t idx;
    uint16_t out_index, in_index;
    uint16_t packet_length_before_crc;
    uint16_t body_end;
    uint16_t packet_length_in = (uint16_t)packet[DXL_PH2_PKT_IDX_LENGTH_L] | ((uint16_t)packet[DXL_PH2_PKT_IDX_LENGTH_H] << 8);
    uint16_t packet_length_out = packet_length_in;

    if (packet_length_in < 5) return; // INST, P1, P2, CRC_L, CRC_H

    packet_length_before_crc = packet_length_in - 2;            // INST, P1-Pn
    body_end = DXL_PH2_PKT_IDX_INSTRUCTION + packet_length_before_crc;  // CRC_L

    // scan window: [LENGTH_L, CRC_L)
    for (idx = DXL_PH2_PKT_IDX_LENGTH_L; idx + 2 < body_end; idx++)
    {
        packet_ptr = &packet[idx];
        if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) packet_length_out++;
    }

    if ((uint32_t)packet_length_out + DXL_PH2_PKT_IDX_INSTRUCTION > DXL_PH2_PKT_MAX_LEN) return; // TODO: proper error propagation
    if (packet_length_in == packet_length_out) return; // no stuffing required

    out_index  = packet_length_out + 6 - 2; // last index before crc
    in_index   = packet_length_in + 6 - 2;  // last index before crc
    while (out_index != in_index)
    {
        if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF)
        {
            packet[out_index--] = 0xFD; // byte stuffing
            if (out_index != in_index)
            {
                packet[out_index--] = packet[in_index--]; // FD
                packet[out_index--] = packet[in_index--]; // FF
                packet[out_index--] = packet[in_index--]; // FF
            }
        }
        else packet[out_index--] = packet[in_index--];
    }

    packet[DXL_PH2_PKT_IDX_LENGTH_L] = packet_length_out & 0xFF;
    packet[DXL_PH2_PKT_IDX_LENGTH_H] = (packet_length_out >> 8) & 0xFF;
}

/* Stuffing removal */
// TODO: Notes on the difference between my stuffing and official ROBOTIS DynamixelSDK stuffing
void dxl_ph2_rem_stuffing(uint8_t *packet)
{
    uint16_t stuff_idx = 0;
    uint16_t pkt_idx = 0;
    uint16_t packet_length_in = (uint16_t)packet[DXL_PH2_PKT_IDX_LENGTH_L] | ((uint16_t)packet[DXL_PH2_PKT_IDX_LENGTH_H] << 8);
    uint16_t packet_length_out = packet_length_in;

    pkt_idx = DXL_PH2_PKT_IDX_LENGTH_L;

    // LENGTH_L..LENGTH_H..INST..Pn
    for (stuff_idx = 0; stuff_idx < packet_length_in; stuff_idx++)
    {
        if ( // FF FF FD FD
            packet[stuff_idx + DXL_PH2_PKT_IDX_LENGTH_L] == 0xFD &&
            packet[stuff_idx + DXL_PH2_PKT_IDX_LENGTH_L + 1] == 0xFD &&
            packet[stuff_idx + DXL_PH2_PKT_IDX_LENGTH_L - 1] == 0xFF &&
            packet[stuff_idx + DXL_PH2_PKT_IDX_LENGTH_L - 2] == 0xFF)
        {
            packet_length_out--;
            stuff_idx++;
        }
        packet[pkt_idx++] = packet[stuff_idx + DXL_PH2_PKT_IDX_LENGTH_L];
    }

    packet[pkt_idx++] = packet[DXL_PH2_PKT_IDX_LENGTH_L + packet_length_in];     // CRC_L
    packet[pkt_idx++] = packet[DXL_PH2_PKT_IDX_LENGTH_L + packet_length_in + 1]; // CRC_H

    packet[DXL_PH2_PKT_IDX_LENGTH_L] = packet_length_out & 0xFF;
    packet[DXL_PH2_PKT_IDX_LENGTH_H] = (packet_length_out >> 8) & 0xFF;
}

void dxl_ph2_build_tx(
        dxl_ph2_ctx_t* ctx,
        const uint8_t id,
        const uint8_t inst,
        const uint8_t param[],
        const size_t param_len,
        dxl_ph2_pkt_t* out_pkt
){
        if (ctx->pkt_state != DXL_PH2_PKT_STATE_OUTBOUND) return; // TODO: proper error propagation

        uint16_t packet_body = param_len + 3;

        if ((uint32_t)packet_body + DXL_PH2_PKT_IDX_INSTRUCTION > DXL_PH2_PKT_MAX_LEN) return; // TODO: proper error propagation

        uint8_t len_l = packet_body & 0xFF;
        uint8_t len_h = (packet_body >> 8) & 0xFF;

        uint8_t tx_buf[DXL_PH2_PKT_MAX_LEN] = {0};
        
        tx_buf[4] = id;
        tx_buf[5] = len_l;
        tx_buf[6] = len_h;
        tx_buf[7] = inst;
        memcpy(&tx_buf[DXL_PH2_PKT_IDX_INSTRUCTION + 1], param, param_len);
        dxl_ph2_add_stuffing(tx_buf);
        
        // update payload length after stuffing
        packet_body = (uint16_t)tx_buf[DXL_PH2_PKT_IDX_LENGTH_L] | ((uint16_t)tx_buf[DXL_PH2_PKT_IDX_LENGTH_H] << 8);

        tx_buf[0] = DXL_PH2_PKT_BYTE_HEADER_1;
        tx_buf[1] = DXL_PH2_PKT_BYTE_HEADER_2;
        tx_buf[2] = DXL_PH2_PKT_BYTE_HEADER_3;
        tx_buf[3] = DXL_PH2_PKT_BYTE_RSRVD;
        
        out_pkt->payload_len = DXL_PH2_PKT_IDX_INSTRUCTION + packet_body;
        helper_calc_crc_to_txbuf(tx_buf, out_pkt->payload_len);
        
        memcpy(out_pkt->dxl_buffer, tx_buf, out_pkt->payload_len);
        ctx->pkt_state = DXL_PH2_PKT_STATE_BUILT;
}

void dxl_ph2_parse_rx(
        dxl_ph2_ctx_t* ctx,
        uint8_t* inbound_buf,
        size_t inbound_buf_len,
        size_t pkt_len_estimate,
        uint8_t skip_stuffing,
        size_t* last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
){
        if (ctx->pkt_state == DXL_PH2_PKT_STATE_INBOUND || ctx->pkt_state == DXL_PH2_PKT_STATE_NEED_MORE)
                ctx->pkt_state = DXL_PH2_PKT_STATE_NEED_MORE;
        else return; // TODO: proper error prop

        *last_idx_fed = 0;
        switch (ctx->inbound_ctx.state) {
                case DXL_PH2_INBOUND_PARSER_RESET:
                        ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_LOOK_HEADER;
                case DXL_PH2_INBOUND_PARSER_LOOK_HEADER:
                        for (; *last_idx_fed < inbound_buf_len && ctx->inbound_ctx.state != DXL_PH2_INBOUND_PARSER_PKT_HEADER_FOUND; (*last_idx_fed)++) {
                                if (inbound_buf[*last_idx_fed] == DXL_PH2_PKT_HEADER_PATTERN[ctx->inbound_ctx.pkt_header_seq_counter]) {
                                        ctx->inbound_ctx.pkt_header_seq_counter++;
                                        
                                        if (ctx->inbound_ctx.pkt_header_seq_counter == sizeof(DXL_PH2_PKT_HEADER_PATTERN)) {
                                                ctx->inbound_ctx.pkt_header_seq_counter = 0;
                                                ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_PKT_HEADER_FOUND;

                                                memcpy(out_pkt->dxl_buffer, DXL_PH2_PKT_HEADER_PATTERN, sizeof(DXL_PH2_PKT_HEADER_PATTERN));
                                                out_pkt->payload_len = sizeof(DXL_PH2_PKT_HEADER_PATTERN);       
                                        }
                                } else {
                                        ctx->inbound_ctx.pkt_header_seq_counter = (inbound_buf[*last_idx_fed] == DXL_PH2_PKT_HEADER_PATTERN[0]) ? 1 : 0;
                                }
                        }
                case DXL_PH2_INBOUND_PARSER_PKT_HEADER_FOUND: // start at RSRVD
                        while (*last_idx_fed < inbound_buf_len) {
                                if (ctx->inbound_ctx.pkt_start_counter < 5) // RSRVD, ID, Length Low, Length High, INST
                                {
                                        switch (ctx->inbound_ctx.pkt_start_counter) {
                                                case 0: if (inbound_buf[*last_idx_fed] != DXL_PH2_PKT_BYTE_RSRVD) return; break; // TODO: proper error prop, if 0xFD then out of sync
                                                case 1: if (inbound_buf[*last_idx_fed] == 0xFF || inbound_buf[*last_idx_fed] == 0xFD) return; break; // TODO: proper error prop
                                        }

                                        out_pkt->dxl_buffer[out_pkt->payload_len] = inbound_buf[*last_idx_fed];
                                        out_pkt->payload_len++;
                                        ctx->inbound_ctx.pkt_start_counter++;
                                }
                                else 
                                {
                                        uint16_t body_len = (uint16_t)out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_L] | ((uint16_t)out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_LENGTH_H] << 8);
                                        uint32_t pkt_len = body_len + DXL_PH2_PKT_IDX_PARAMETER0;
                                        if (pkt_len > pkt_len_estimate || pkt_len > DXL_PH2_PKT_MAX_LEN) return; // TODO: proper error prop
                                        if (out_pkt->dxl_buffer[DXL_PH2_PKT_IDX_INSTRUCTION] != 0x55) return; // TODO: proper error prop

                                        ctx->inbound_ctx.pkt_start_counter = 0;
                                        ctx->inbound_ctx.pkt_body_counter = body_len - 1; // minus INST
                                        ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_FEEDING;
                                        break;
                                }

                                (*last_idx_fed)++;
                        }
                case DXL_PH2_INBOUND_PARSER_FEEDING: // start at PARAMETER0 or ERROR byte
                        while (*last_idx_fed < inbound_buf_len) {
                                if (ctx->inbound_ctx.pkt_body_counter > 0) {
                                        out_pkt->dxl_buffer[out_pkt->payload_len] = inbound_buf[*last_idx_fed];
                                        out_pkt->payload_len++;
                                        ctx->inbound_ctx.pkt_body_counter--;
                                        (*last_idx_fed)++;
                                
                                        if (ctx->inbound_ctx.pkt_body_counter == 0) {
                                                ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_FED;
                                                break;
                                        }
                                }
                                else
                                {
                                        ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_FED;
                                        ctx->pkt_state = DXL_PH2_PKT_STATE_FILLED;
                                        break;
                                }
                        }
                case DXL_PH2_INBOUND_PARSER_FED: // all bytes stored
                        if (ctx->inbound_ctx.state == DXL_PH2_INBOUND_PARSER_FED) {
                                uint16_t inbound_crc = ((uint16_t)out_pkt->dxl_buffer[out_pkt->payload_len - 1] << 8) 
                                        | out_pkt->dxl_buffer[out_pkt->payload_len - 2];
                                uint16_t calculated_crc = helper_calc_crc_from_rxbuf(
                                                out_pkt->dxl_buffer,
                                                out_pkt->payload_len
                                        );

                                if(inbound_crc != calculated_crc) return; // TODO: proper error prop
                                if (!skip_stuffing) dxl_ph2_rem_stuffing(out_pkt->dxl_buffer);
                                ctx->inbound_ctx.state = DXL_PH2_INBOUND_PARSER_DONE;
                        }
                case DXL_PH2_INBOUND_PARSER_DONE: break;
        }
}