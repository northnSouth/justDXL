#include "jdxl/jdxl_ph2.h"
#include "jdxl_internal.h"

#include <string.h>

#define IDX_ID          4u
#define IDX_LEN_L       5u
#define IDX_LEN_H       6u
#define IDX_INST        7u
#define IDX_ERR         8u      /* status: ERROR follows INST */
#define HDR_LEN         7u      /* FF FF FD 00 ID LEN_L LEN_H; total = HDR_LEN + LENGTH */
#define OVERHEAD        10u     /* HDR_LEN + INST + CRC(2) */
#define MAX_UNICAST     252u    /* "Packet ID": 0-252 */

static const uint8_t k_header[4] = { 0xFF, 0xFF, 0xFD, 0x00 };

static const uint8_t k_valid_inst[] = {
        JDXL_PH2_INST_PING, JDXL_PH2_INST_READ, JDXL_PH2_INST_WRITE, JDXL_PH2_INST_REG_WRITE,
        JDXL_PH2_INST_ACTION, JDXL_PH2_INST_FACTORY_RESET, JDXL_PH2_INST_REBOOT,
        JDXL_PH2_INST_CLEAR, JDXL_PH2_INST_CTRL_TABLE_BACKUP, JDXL_PH2_INST_STATUS,
        JDXL_PH2_INST_SYNC_READ, JDXL_PH2_INST_SYNC_WRITE, JDXL_PH2_INST_FAST_SYNC_READ,
        JDXL_PH2_INST_BULK_READ, JDXL_PH2_INST_BULK_WRITE, JDXL_PH2_INST_FAST_BULK_READ
};

static int is_valid_inst(uint8_t inst)
{
        size_t i;

        for (i = 0; i < sizeof(k_valid_inst); i++) {
                if (k_valid_inst[i] == inst) return 1;
        }
        return 0;
}

static int is_unicast(uint8_t id)       { return id <= MAX_UNICAST; }
static int is_target(uint8_t id)        { return id <= MAX_UNICAST || id == JDXL_BROADCAST_ID; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* CRC-16, poly 0x8005, init 0, MSB first.
 * Source: https://docs.robotis.com/docs/dxl/protocol/crc */
static uint16_t crc16(const uint8_t *data, size_t n)
{
        static const uint16_t table[256] = {
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
        uint16_t crc = 0;
        size_t   i;

        for (i = 0; i < n; i++)
                crc = (uint16_t)((crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFFu]);
        return crc;
}

/* Whether an instruction with n params fits the TX buffer (before byte stuffing). */
static int fits(size_t n)
{
        return n + OVERHEAD <= JDXL_PH2_PKT_MAX_LEN;
}

/* Whether a status packet with this LENGTH field fits the RX buffer and the 16-bit LENGTH. */
static int reply_len_fits(uint32_t length_field)
{
        return length_field <= 0xFFFFu && HDR_LEN + length_field <= JDXL_PH2_PKT_MAX_LEN;
}

/* Status for a single servo: INST + ERR + data + CRC */
static int reply_fits(uint32_t data_len) { return reply_len_fits(4u + data_len); }

jdxl_result_t jdxl_ph2_encode(uint8_t id, uint8_t inst, const uint8_t *params, size_t n,
                              uint8_t *out, size_t cap, size_t *out_len)
{
        const size_t first = IDX_INST + 1u;
        size_t       w     = first;
        size_t       i, length_field;
        uint16_t     crc;

        if (out_len != NULL) *out_len = 0;
        if (out == NULL || out_len == NULL || (params == NULL && n > 0)) return JDXL_ERR_NULL_ARG;
        if (id == 0xFD || id == 0xFF) return JDXL_ERR_INVALID_ID;
        if (!is_valid_inst(inst)) return JDXL_ERR_INVALID_ARG;

        for (i = 0; i < n; i++) {
                if (w + 1u + 2u > cap) return JDXL_ERR_TOO_LONG;
                out[w++] = params[i];
                /* Byte stuffing: FF FF FD in the payload becomes FF FF FD FD, so it is never taken
                 * for a header ("Packet Process", Transmission Process 2). Applied to any length;
                 * ROBOTIS's SDK skips it below LENGTH 8, which the spec does not allow for. */
                if (w - first >= 3u && out[w - 3] == 0xFF && out[w - 2] == 0xFF && out[w - 1] == 0xFD) {
                        if (w + 1u + 2u > cap) return JDXL_ERR_TOO_LONG;
                        out[w++] = 0xFD;
                }
        }
        if (w + 2u > cap) return JDXL_ERR_TOO_LONG;

        length_field = (w - IDX_INST) + 2u;     /* INST + stuffed params + CRC */
        if (length_field > 0xFFFFu) return JDXL_ERR_TOO_LONG;

        memcpy(out, k_header, sizeof(k_header));
        out[IDX_ID]   = id;
        put16(&out[IDX_LEN_L], (uint16_t)length_field);
        out[IDX_INST] = inst;

        crc = crc16(out, w);                    /* header through the last param */
        put16(&out[w], crc);
        *out_len = w + 2u;
        return JDXL_OK;
}

// ------------------------------------------------------------------------------------------------
// Context
// ------------------------------------------------------------------------------------------------

void jdxl_ph2_init(jdxl_ph2_ctx_t *ctx, const jdxl_opts_t *opts)
{
        if (ctx == NULL) return;
        memset(ctx, 0, sizeof(*ctx));
        ctx->opts = jdxl_opts_or_default(opts);
}

jdxl_span_t jdxl_ph2_tx(const jdxl_ph2_ctx_t *ctx)
{
        jdxl_span_t s = { NULL, 0 };

        if (ctx != NULL && ctx->tx_len > 0) {
                s.data = ctx->tx;
                s.len  = ctx->tx_len;
        }
        return s;
}

uint8_t jdxl_ph2_pending(const jdxl_ph2_ctx_t *ctx)
{
        return (ctx != NULL) ? (uint8_t)(ctx->priv.count - ctx->priv.idx) : 0;
}

static void drop_pending(jdxl_ph2_ctx_t *ctx)
{
        ctx->debug.missed   += (uint32_t)(ctx->priv.count - ctx->priv.idx);
        ctx->priv.count      = 0;
        ctx->priv.idx        = 0;
        ctx->priv.fast       = 0;
        ctx->priv.rx_len     = 0;
        ctx->priv.echo_left  = 0;
}

void jdxl_ph2_abort(jdxl_ph2_ctx_t *ctx)
{
        if (ctx != NULL) drop_pending(ctx);
}

// ------------------------------------------------------------------------------------------------
// Builders. Pattern: validate everything (context untouched on error), begin(), assemble params
// in priv.rx (free once the previous exchange is dropped), emit(), expect() the replies.
// ------------------------------------------------------------------------------------------------

static void begin(jdxl_ph2_ctx_t *ctx)
{
        drop_pending(ctx);
        ctx->tx_len = 0;
}

static jdxl_result_t emit(jdxl_ph2_ctx_t *ctx, uint8_t id, uint8_t inst, size_t n)
{
        const jdxl_result_t r = jdxl_ph2_encode(id, inst, ctx->priv.rx, n,
                                                ctx->tx, sizeof(ctx->tx), &ctx->tx_len);
        if (r != JDXL_OK) return r;
        ctx->priv.echo_left = ctx->opts.tx_echo ? ctx->tx_len : 0;
        return JDXL_OK;
}

#define EMIT(ctx, id, inst, n) \
        do { const jdxl_result_t r_ = emit((ctx), (id), (inst), (n)); if (r_ != JDXL_OK) return r_; } while (0)

static void expect(jdxl_ph2_ctx_t *ctx, uint16_t id, uint16_t len)
{
        ctx->priv.slot[ctx->priv.count].id       = id;
        ctx->priv.slot[ctx->priv.count].data_len = len;
        ctx->priv.count++;
}

/* Broadcast is only answered for Ping and the read instructions ("Response Policy"); unicast
 * non-read instructions only at Status Return Level ALL. */
static void expect_ack(jdxl_ph2_ctx_t *ctx, uint8_t id)
{
        if (id != JDXL_BROADCAST_ID && ctx->opts.status_return_level >= JDXL_SRL_ALL)
                expect(ctx, id, 0);
}

static int reads_answered(const jdxl_ph2_ctx_t *ctx)
{
        return ctx->opts.status_return_level >= JDXL_SRL_READ;
}

jdxl_result_t jdxl_ph2_build_ping(jdxl_ph2_ctx_t *ctx, uint8_t id)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_unicast(id)) return JDXL_ERR_INVALID_ID;

        begin(ctx);
        EMIT(ctx, id, JDXL_PH2_INST_PING, 0);
        expect(ctx, id, 3);             /* model number (2) + firmware version */
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_ping_broadcast(jdxl_ph2_ctx_t *ctx, uint8_t max_replies)
{
        uint8_t i;

        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (max_replies == 0) return JDXL_ERR_EMPTY;
        if (max_replies > JDXL_PH2_MAX_STATUS) return JDXL_ERR_TOO_MANY_STATUS;

        begin(ctx);
        EMIT(ctx, JDXL_BROADCAST_ID, JDXL_PH2_INST_PING, 0);
        for (i = 0; i < max_replies; i++) expect(ctx, JDXL_ANY_ID, 3);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_read(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr, uint16_t len)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_unicast(id)) return JDXL_ERR_INVALID_ID;   /* not executed when broadcast */
        if (len == 0) return JDXL_ERR_EMPTY;
        if (!reply_fits(len)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        put16(&ctx->priv.rx[0], addr);
        put16(&ctx->priv.rx[2], len);
        EMIT(ctx, id, JDXL_PH2_INST_READ, 4);
        if (reads_answered(ctx)) expect(ctx, id, len);
        return JDXL_OK;
}

static jdxl_result_t write_family(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr,
                                  const uint8_t *data, size_t len, uint8_t inst)
{
        if (ctx == NULL || (data == NULL && len > 0)) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        if (len == 0) return JDXL_ERR_EMPTY;
        if (!fits(2u + len)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        put16(&ctx->priv.rx[0], addr);
        memcpy(&ctx->priv.rx[2], data, len);
        EMIT(ctx, id, inst, 2u + len);
        expect_ack(ctx, id);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_write(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr,
                                   const uint8_t *data, size_t len)
{
        return write_family(ctx, id, addr, data, len, JDXL_PH2_INST_WRITE);
}

jdxl_result_t jdxl_ph2_build_reg_write(jdxl_ph2_ctx_t *ctx, uint8_t id, uint16_t addr,
                                       const uint8_t *data, size_t len)
{
        return write_family(ctx, id, addr, data, len, JDXL_PH2_INST_REG_WRITE);
}

/* Instructions with fixed params already in priv.rx[0..n) once validated. */
static jdxl_result_t simple(jdxl_ph2_ctx_t *ctx, uint8_t id, uint8_t inst,
                            const uint8_t *params, size_t n)
{
        begin(ctx);
        if (n > 0) memcpy(ctx->priv.rx, params, n);
        EMIT(ctx, id, inst, n);
        expect_ack(ctx, id);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_action(jdxl_ph2_ctx_t *ctx, uint8_t id)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        return simple(ctx, id, JDXL_PH2_INST_ACTION, NULL, 0);
}

jdxl_result_t jdxl_ph2_build_reboot(jdxl_ph2_ctx_t *ctx, uint8_t id)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        return simple(ctx, id, JDXL_PH2_INST_REBOOT, NULL, 0);
}

jdxl_result_t jdxl_ph2_build_factory_reset(jdxl_ph2_ctx_t *ctx, uint8_t id, jdxl_ph2_reset_t mode)
{
        const uint8_t p = (uint8_t)mode;

        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        if (mode != JDXL_PH2_RESET_ALL && mode != JDXL_PH2_RESET_ALL_EXCEPT_ID
            && mode != JDXL_PH2_RESET_ALL_EXCEPT_ID_BAUD)
                return JDXL_ERR_INVALID_ARG;
        /* Broadcast + Reset All is not executed by the servos ("Factory Reset" warning) */
        if (id == JDXL_BROADCAST_ID && mode == JDXL_PH2_RESET_ALL) return JDXL_ERR_INVALID_ID;
        return simple(ctx, id, JDXL_PH2_INST_FACTORY_RESET, &p, 1);
}

jdxl_result_t jdxl_ph2_build_clear(jdxl_ph2_ctx_t *ctx, uint8_t id, jdxl_ph2_clear_t mode)
{
        static const uint8_t k_multi_turn[5] = { 0x01, 0x44, 0x58, 0x4C, 0x22 };
        static const uint8_t k_error[5]      = { 0x02, 0x45, 0x52, 0x43, 0x4C };

        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        if (mode == JDXL_PH2_CLEAR_MULTI_TURN) return simple(ctx, id, JDXL_PH2_INST_CLEAR, k_multi_turn, 5);
        if (mode == JDXL_PH2_CLEAR_ERROR)      return simple(ctx, id, JDXL_PH2_INST_CLEAR, k_error, 5);
        return JDXL_ERR_INVALID_ARG;
}

jdxl_result_t jdxl_ph2_build_ctrl_table_backup(jdxl_ph2_ctx_t *ctx, uint8_t id,
                                               jdxl_ph2_backup_t mode)
{
        uint8_t p[5] = { 0x00, 0x43, 0x54, 0x52, 0x4C };   /* "CTRL" */

        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_target(id)) return JDXL_ERR_INVALID_ID;
        if (mode != JDXL_PH2_BACKUP_STORE && mode != JDXL_PH2_BACKUP_RESTORE)
                return JDXL_ERR_INVALID_ARG;
        p[0] = (uint8_t)mode;
        return simple(ctx, id, JDXL_PH2_INST_CTRL_TABLE_BACKUP, p, sizeof(p));
}

static jdxl_result_t sync_read_family(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                      const uint8_t *ids, uint8_t count, uint8_t inst, uint8_t fast)
{
        uint32_t seen[8] = { 0 };
        uint8_t  i;

        if (ctx == NULL || (ids == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0 || len == 0) return JDXL_ERR_EMPTY;
        if (count > JDXL_PH2_MAX_STATUS) return JDXL_ERR_TOO_MANY_STATUS;
        for (i = 0; i < count; i++) {
                if (!is_unicast(ids[i])) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, ids[i])) return JDXL_ERR_DUPLICATE_ID;
        }
        if (!fits(4u + count)) return JDXL_ERR_TOO_LONG;
        /* Fast: one packet, INST + per servo (ERR + ID + data + CRC); otherwise one per servo */
        if (fast ? !reply_len_fits(1u + (uint32_t)count * (4u + len)) : !reply_fits(len))
                return JDXL_ERR_TOO_LONG;

        begin(ctx);
        put16(&ctx->priv.rx[0], addr);
        put16(&ctx->priv.rx[2], len);
        memcpy(&ctx->priv.rx[4], ids, count);
        EMIT(ctx, JDXL_BROADCAST_ID, inst, 4u + count);
        if (reads_answered(ctx)) {
                ctx->priv.fast = fast;
                for (i = 0; i < count; i++) expect(ctx, ids[i], len);
        }
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_sync_read(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                       const uint8_t *ids, uint8_t count)
{
        return sync_read_family(ctx, addr, len, ids, count, JDXL_PH2_INST_SYNC_READ, 0);
}

jdxl_result_t jdxl_ph2_build_fast_sync_read(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                            const uint8_t *ids, uint8_t count)
{
        return sync_read_family(ctx, addr, len, ids, count, JDXL_PH2_INST_FAST_SYNC_READ, 1);
}

jdxl_result_t jdxl_ph2_build_sync_write(jdxl_ph2_ctx_t *ctx, uint16_t addr, uint16_t len,
                                        const jdxl_ph2_sync_write_t *entries, uint8_t count)
{
        uint32_t seen[8] = { 0 };
        size_t   need, w = 4;
        uint8_t  i;

        if (ctx == NULL || (entries == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0 || len == 0) return JDXL_ERR_EMPTY;
        for (i = 0; i < count; i++) {
                if (entries[i].data == NULL) return JDXL_ERR_NULL_ARG;
                if (!is_unicast(entries[i].id)) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, entries[i].id)) return JDXL_ERR_DUPLICATE_ID;
        }
        need = 4u + (size_t)count * (1u + len);
        if (!fits(need)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        put16(&ctx->priv.rx[0], addr);
        put16(&ctx->priv.rx[2], len);
        for (i = 0; i < count; i++) {
                ctx->priv.rx[w++] = entries[i].id;
                memcpy(&ctx->priv.rx[w], entries[i].data, len);
                w += len;
        }
        EMIT(ctx, JDXL_BROADCAST_ID, JDXL_PH2_INST_SYNC_WRITE, w);
        return JDXL_OK;
}

static jdxl_result_t bulk_read_family(jdxl_ph2_ctx_t *ctx, const jdxl_ph2_bulk_read_t *entries,
                                      uint8_t count, uint8_t inst, uint8_t fast)
{
        uint32_t seen[8] = { 0 };
        uint32_t fast_len = 1;          /* INST */
        size_t   w = 0;
        uint8_t  i;

        if (ctx == NULL || (entries == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0) return JDXL_ERR_EMPTY;
        if (count > JDXL_PH2_MAX_STATUS) return JDXL_ERR_TOO_MANY_STATUS;
        for (i = 0; i < count; i++) {
                if (entries[i].len == 0) return JDXL_ERR_EMPTY;
                if (!is_unicast(entries[i].id)) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, entries[i].id)) return JDXL_ERR_DUPLICATE_ID;
                if (!fast && !reply_fits(entries[i].len)) return JDXL_ERR_TOO_LONG;
                fast_len += 4u + entries[i].len;        /* ERR + ID + data + CRC */
        }
        if (!fits(5u * (size_t)count)) return JDXL_ERR_TOO_LONG;
        if (fast && !reply_len_fits(fast_len)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        for (i = 0; i < count; i++) {
                ctx->priv.rx[w++] = entries[i].id;
                put16(&ctx->priv.rx[w], entries[i].addr); w += 2;
                put16(&ctx->priv.rx[w], entries[i].len);  w += 2;
        }
        EMIT(ctx, JDXL_BROADCAST_ID, inst, w);
        if (reads_answered(ctx)) {
                ctx->priv.fast = fast;
                for (i = 0; i < count; i++) expect(ctx, entries[i].id, entries[i].len);
        }
        return JDXL_OK;
}

jdxl_result_t jdxl_ph2_build_bulk_read(jdxl_ph2_ctx_t *ctx,
                                       const jdxl_ph2_bulk_read_t *entries, uint8_t count)
{
        return bulk_read_family(ctx, entries, count, JDXL_PH2_INST_BULK_READ, 0);
}

jdxl_result_t jdxl_ph2_build_fast_bulk_read(jdxl_ph2_ctx_t *ctx,
                                            const jdxl_ph2_bulk_read_t *entries, uint8_t count)
{
        return bulk_read_family(ctx, entries, count, JDXL_PH2_INST_FAST_BULK_READ, 1);
}

jdxl_result_t jdxl_ph2_build_bulk_write(jdxl_ph2_ctx_t *ctx,
                                        const jdxl_ph2_bulk_write_t *entries, uint8_t count)
{
        uint32_t seen[8] = { 0 };
        size_t   need = 0, w = 0;
        uint8_t  i;

        if (ctx == NULL || (entries == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0) return JDXL_ERR_EMPTY;
        for (i = 0; i < count; i++) {
                if (entries[i].data == NULL) return JDXL_ERR_NULL_ARG;
                if (entries[i].len == 0) return JDXL_ERR_EMPTY;
                if (!is_unicast(entries[i].id)) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, entries[i].id)) return JDXL_ERR_DUPLICATE_ID;
                need += 5u + entries[i].len;
        }
        if (!fits(need)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        for (i = 0; i < count; i++) {
                ctx->priv.rx[w++] = entries[i].id;
                put16(&ctx->priv.rx[w], entries[i].addr); w += 2;
                put16(&ctx->priv.rx[w], entries[i].len);  w += 2;
                memcpy(&ctx->priv.rx[w], entries[i].data, entries[i].len);
                w += entries[i].len;
        }
        EMIT(ctx, JDXL_BROADCAST_ID, JDXL_PH2_INST_BULK_WRITE, w);
        return JDXL_OK;
}

// ------------------------------------------------------------------------------------------------
// Feed. priv.rx holds one candidate packet starting at rx[0]. After every byte the candidate is
// checked; a rejected candidate slides by one byte, so the parser resynchronizes by itself.
// FF FF FD FD (stuffing) never matches the 4-byte header FF FF FD 00, so it is ignored for free.
// ------------------------------------------------------------------------------------------------

static void drop_front(jdxl_ph2_ctx_t *ctx, size_t k)
{
        memmove(ctx->priv.rx, ctx->priv.rx + k, ctx->priv.rx_len - k);
        ctx->priv.rx_len -= k;
}

/* Upper bound on HDR_LEN + LENGTH for the reply to `slot`. */
static size_t reply_bound(const jdxl_ph2_ctx_t *ctx, int slot)
{
        size_t len_field;
        uint8_t k;

        if (ctx->priv.fast) {                   /* never byte-stuffed: exact */
                len_field = 1;
                for (k = 0; k < ctx->priv.count; k++) len_field += 4u + ctx->priv.slot[k].data_len;
                return HDR_LEN + len_field;
        }
        len_field = 4u + ctx->priv.slot[slot].data_len;
        /* + worst-case stuffing: FF FF FD repeated over ERR + data (LENGTH - 3 bytes) */
        return HDR_LEN + len_field + (len_field - 3u) / 3u;
}

static int rx_check(jdxl_ph2_ctx_t *ctx, int *slot)
{
        const uint8_t *p = ctx->priv.rx;
        const size_t   n = ctx->priv.rx_len;
        size_t         k, need;
        uint16_t       len_field;

        if (n == 0) return JDXL_RX_MORE;
        for (k = 0; k < n && k < sizeof(k_header); k++) {
                if (p[k] != k_header[k]) return JDXL_RX_SKIP;
        }
        if (n <= IDX_ID) return JDXL_RX_MORE;
        if (p[IDX_ID] == 0xFD || p[IDX_ID] == 0xFF) return JDXL_RX_SKIP;
        if (n <= IDX_INST) return JDXL_RX_MORE;

        if (p[IDX_INST] != JDXL_PH2_INST_STATUS) {
                ctx->debug.not_status++;
                return JDXL_RX_DROP;
        }
        if (ctx->priv.fast) {                   /* one reply from the broadcast ID */
                *slot = ctx->priv.idx;
                if (p[IDX_ID] != JDXL_BROADCAST_ID) {
                        ctx->debug.unexpected_id++;
                        return JDXL_RX_DROP;
                }
        } else {
                *slot = jdxl_find_slot(ctx->priv.slot, ctx->priv.idx, ctx->priv.count, p[IDX_ID]);
                if (*slot < 0) {
                        ctx->debug.unexpected_id++;
                        return JDXL_RX_DROP;
                }
        }

        /* INST + ERR + CRC is the shortest status; longer than requested cannot be ours */
        len_field = get16(&p[IDX_LEN_L]);
        need      = HDR_LEN + len_field;
        if (len_field < 4u || need > reply_bound(ctx, *slot) || need > sizeof(ctx->priv.rx)) {
                ctx->debug.bad_length++;
                return JDXL_RX_DROP;
        }
        if (n < need) return JDXL_RX_MORE;

        if (crc16(p, need - 2u) != get16(&p[need - 2u])) {
                ctx->debug.bad_checksum++;
                return JDXL_RX_DROP;
        }
        return JDXL_RX_DONE;
}

/* Removes byte stuffing in place (FF FF FD FD -> FF FF FD); returns the new length. */
static size_t unstuff(uint8_t *p, size_t n)
{
        uint8_t a = 0, b = 0, c = 0;    /* the last three input bytes */
        size_t  r, w = 0;

        for (r = 0; r < n; r++) {
                const uint8_t x = p[r];
                if (!(a == 0xFF && b == 0xFF && c == 0xFD && x == 0xFD)) p[w++] = x;
                a = b; b = c; c = x;
        }
        return w;
}

static void finish_if_done(jdxl_ph2_ctx_t *ctx)
{
        if (ctx->priv.idx == ctx->priv.count) {
                ctx->priv.count = 0;
                ctx->priv.idx   = 0;
                ctx->priv.fast  = 0;
                ctx->debug.exchanges_done++;
        }
}

static void deliver_fast(jdxl_ph2_ctx_t *ctx, size_t total, jdxl_on_status_t cb, void *user)
{
        const uint8_t *p   = ctx->priv.rx;
        size_t         off = IDX_ERR;
        uint8_t        k;
        jdxl_status_t  s;

        /* INST, then per servo ERR | ID | data | CRC; the last CRC is the packet CRC */
        for (k = 0; k < ctx->priv.count; k++) {
                const size_t chunk = 4u + ctx->priv.slot[k].data_len;
                if (off + chunk > total) break;

                ctx->debug.status_ok++;
                ctx->priv.idx = (uint8_t)(k + 1);

                s.index      = k;
                s.id         = p[off + 1];
                s.error      = p[off];
                s.params     = &p[off + 2];
                s.params_len = ctx->priv.slot[k].data_len;
                if (cb != NULL) cb(&s, user);
                off += chunk;
        }
        ctx->debug.missed += (uint32_t)(ctx->priv.count - k);
        ctx->priv.idx = ctx->priv.count;
        finish_if_done(ctx);
}

static void deliver(jdxl_ph2_ctx_t *ctx, int slot, jdxl_on_status_t cb, void *user)
{
        uint8_t      *p     = ctx->priv.rx;
        const size_t  total = HDR_LEN + get16(&p[IDX_LEN_L]);
        size_t        region, params_len;
        jdxl_status_t s;

        if (ctx->priv.fast) {
                deliver_fast(ctx, total, cb, user);
                return;
        }

        /* ERR + params, between INST and CRC; the CRC was checked over the stuffed bytes */
        region     = unstuff(&p[IDX_ERR], total - 2u - IDX_ERR);
        params_len = region - 1u;
        if (params_len > ctx->priv.slot[slot].data_len) {
                ctx->debug.bad_length++;
                return;
        }

        ctx->debug.missed += (uint32_t)(slot - ctx->priv.idx);
        ctx->debug.status_ok++;
        ctx->priv.idx = (uint8_t)(slot + 1);

        s.index      = (uint8_t)slot;
        s.id         = p[IDX_ID];
        s.error      = p[IDX_ERR];
        s.params     = &p[IDX_ERR + 1];
        s.params_len = (uint16_t)params_len;
        if (cb != NULL) cb(&s, user);
        finish_if_done(ctx);
}

static void rx_byte(jdxl_ph2_ctx_t *ctx, uint8_t b, jdxl_on_status_t cb, void *user)
{
        if (ctx->priv.count == 0) {
                ctx->debug.ignored_bytes++;
                return;
        }
        if (ctx->priv.rx_len == sizeof(ctx->priv.rx)) drop_front(ctx, 1);  /* unreachable: lengths are bounded */
        ctx->priv.rx[ctx->priv.rx_len++] = b;

        for (;;) {
                int       slot = -1;
                const int r    = rx_check(ctx, &slot);

                if (r == JDXL_RX_MORE) return;
                if (r != JDXL_RX_DONE) {
                        drop_front(ctx, 1);
                        continue;
                }
                deliver(ctx, slot, cb, user);
                ctx->priv.rx_len = 0;
                return;
        }
}

jdxl_feed_t jdxl_ph2_feed(jdxl_ph2_ctx_t *ctx, const uint8_t *buf, size_t len,
                          jdxl_on_status_t on_status, void *user)
{
        bool   done = false;
        size_t i;

        if (ctx == NULL) return JDXL_FEED_IDLE;
        if (buf == NULL) len = 0;

        for (i = 0; i < len; i++) {
                const uint8_t before = ctx->priv.count;

                if (ctx->priv.echo_left > 0) {
                        const size_t pos = ctx->tx_len - ctx->priv.echo_left;

                        if (buf[i] == ctx->tx[pos]) {
                                ctx->priv.echo_left--;
                                continue;
                        }
                        /* No echo after all (or a corrupted one): replay what was taken for it. */
                        ctx->debug.echo_mismatch++;
                        ctx->priv.echo_left = 0;
                        {
                                size_t k;
                                for (k = 0; k < pos; k++) rx_byte(ctx, ctx->tx[k], on_status, user);
                        }
                }
                rx_byte(ctx, buf[i], on_status, user);
                if (before > 0 && ctx->priv.count == 0) done = true;
        }

        if (done) return JDXL_FEED_DONE;
        return (ctx->priv.count > 0) ? JDXL_FEED_PENDING : JDXL_FEED_IDLE;
}