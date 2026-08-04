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

/* Packet handler states */
enum dxl_ph2_state {
        DXL_PH2_PKT_STATE_RESET,
        DXL_PH2_PKT_STATE_ERR,

        DXL_PH2_PKT_STATE_OUTBOUND,
        DXL_PH2_PKT_STATE_BUILT,
        
        DXL_PH2_PKT_STATE_INBOUND,
        DXL_PH2_PKT_STATE_FILLED,
        DXL_PH2_PKT_STATE_NEED_MORE
};

/* Inbound (RX) packet parser states */
enum dxl_ph2_inbound_parser_state {
        DXL_PH2_INBOUND_PARSER_RESET,
        DXL_PH2_INBOUND_PARSER_LOOK_HEADER,
        DXL_PH2_INBOUND_PARSER_PKT_HEADER_FOUND,
        DXL_PH2_INBOUND_PARSER_FEEDING,
        DXL_PH2_INBOUND_PARSER_FED,
        DXL_PH2_INBOUND_PARSER_DONE
};

/* Generic packet struct */
typedef struct {
        uint8_t  dxl_buffer[DXL_PH2_PKT_MAX_LEN];
        size_t   payload_len;
} dxl_ph2_pkt_t;

/* Inbound packet parser struct */
typedef struct {
        enum dxl_ph2_inbound_parser_state state;
        uint8_t                           pkt_header_seq_counter;
        uint8_t                           pkt_start_counter;
        uint16_t                          pkt_body_counter;
} dxl_ph2_inbound_parser_ctx_t;

/* Packet handler context struct */
typedef struct {
        enum dxl_ph2_state           pkt_state;
        dxl_ph2_inbound_parser_ctx_t inbound_ctx;
} dxl_ph2_ctx_t;

/* Outbound (TX) packet builder */
void dxl_ph2_build_tx(
        dxl_ph2_ctx_t* ctx,
        const uint8_t  id,
        const uint8_t  inst,
        const uint8_t  param[],
        const size_t   param_len,
        dxl_ph2_pkt_t* out_pkt
);

/* Inbound (RX) packet parser */
void dxl_ph2_parse_rx(
        dxl_ph2_ctx_t* ctx,
        uint8_t*       inbound_buf,
        size_t         inbound_buf_len,
        size_t         pkt_len_estimate,
        uint8_t        skip_stuffing,
        size_t*        last_idx_fed,
        dxl_ph2_pkt_t* out_pkt
);
