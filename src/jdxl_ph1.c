#include "jdxl/jdxl_ph1.h"
#include "jdxl_internal.h"

#include <string.h>

#define IDX_ID      2u
#define IDX_LEN     3u
#define IDX_INST    4u
#define IDX_ERR     4u          /* a status has ERROR where an instruction has INSTRUCTION */
#define IDX_PARAM0  5u
#define OVERHEAD    6u          /* FF FF ID LEN INST CHECKSUM */
#define MAX_PARAMS  253u        /* LENGTH is one byte: params + 2 <= 255 */
#define MAX_UNICAST 0xFDu       /* "Packet ID": 0-253 */

static const uint8_t k_valid_inst[] = {
        JDXL_PH1_INST_PING, JDXL_PH1_INST_READ, JDXL_PH1_INST_WRITE, JDXL_PH1_INST_REG_WRITE,
        JDXL_PH1_INST_ACTION, JDXL_PH1_INST_FACTORY_RESET, JDXL_PH1_INST_REBOOT,
        JDXL_PH1_INST_SYNC_WRITE, JDXL_PH1_INST_BULK_READ
};

static int is_valid_inst(uint8_t inst)
{
        size_t i;

        for (i = 0; i < sizeof(k_valid_inst); i++) {
                if (k_valid_inst[i] == inst) return 1;
        }
        return 0;
}

static int is_unicast(uint8_t id) { return id <= MAX_UNICAST; }

/* ~(ID + LENGTH + INSTRUCTION/ERROR + params), lower byte ("Checksum") */
static uint8_t checksum(const uint8_t *p, size_t n)
{
        unsigned sum = 0;
        size_t   i;

        for (i = 0; i < n; i++) sum += p[i];
        return (uint8_t)~sum;
}

/* Whether an instruction with n params fits the TX buffer and the one-byte LENGTH. */
static int fits(size_t n)
{
        return n <= MAX_PARAMS && n + OVERHEAD <= JDXL_PH1_PKT_MAX_LEN;
}

/* Same for a status reply carrying `len` data bytes. */
static int reply_fits(size_t len)
{
        return fits(len);
}

jdxl_result_t jdxl_ph1_encode(uint8_t id, uint8_t inst, const uint8_t *params, size_t n,
                              uint8_t *out, size_t cap, size_t *out_len)
{
        if (out_len != NULL) *out_len = 0;
        if (out == NULL || out_len == NULL || (params == NULL && n > 0)) return JDXL_ERR_NULL_ARG;
        if (id == 0xFF) return JDXL_ERR_INVALID_ID;
        if (!is_valid_inst(inst)) return JDXL_ERR_INVALID_ARG;
        if (n > MAX_PARAMS || n + OVERHEAD > cap) return JDXL_ERR_TOO_LONG;

        out[0]        = 0xFF;
        out[1]        = 0xFF;
        out[IDX_ID]   = id;
        out[IDX_LEN]  = (uint8_t)(n + 2);       /* "Length = number of Parameters + 2" */
        out[IDX_INST] = inst;
        if (n > 0) memcpy(&out[IDX_PARAM0], params, n);
        out[IDX_PARAM0 + n] = checksum(&out[IDX_ID], n + 3);

        *out_len = n + OVERHEAD;
        return JDXL_OK;
}

// ------------------------------------------------------------------------------------------------
// Context
// ------------------------------------------------------------------------------------------------

void jdxl_ph1_init(jdxl_ph1_ctx_t *ctx, const jdxl_opts_t *opts)
{
        if (ctx == NULL) return;
        memset(ctx, 0, sizeof(*ctx));
        ctx->opts = jdxl_opts_or_default(opts);
}

jdxl_span_t jdxl_ph1_tx(const jdxl_ph1_ctx_t *ctx)
{
        jdxl_span_t s = { NULL, 0 };

        if (ctx != NULL && ctx->tx_len > 0) {
                s.data = ctx->tx;
                s.len  = ctx->tx_len;
        }
        return s;
}

uint8_t jdxl_ph1_pending(const jdxl_ph1_ctx_t *ctx)
{
        return (ctx != NULL) ? (uint8_t)(ctx->priv.count - ctx->priv.idx) : 0;
}

static void drop_pending(jdxl_ph1_ctx_t *ctx)
{
        ctx->debug.missed   += (uint32_t)(ctx->priv.count - ctx->priv.idx);
        ctx->priv.count      = 0;
        ctx->priv.idx        = 0;
        ctx->priv.rx_len     = 0;
        ctx->priv.echo_left  = 0;
}

void jdxl_ph1_abort(jdxl_ph1_ctx_t *ctx)
{
        if (ctx != NULL) drop_pending(ctx);
}

// ------------------------------------------------------------------------------------------------
// Builders. Pattern: validate everything (context untouched on error), begin(), assemble params
// in priv.rx (free once the previous exchange is dropped), emit(), expect() the replies.
// ------------------------------------------------------------------------------------------------

static void begin(jdxl_ph1_ctx_t *ctx)
{
        drop_pending(ctx);
        ctx->tx_len = 0;
}

static jdxl_result_t emit(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t inst, size_t n)
{
        const jdxl_result_t r = jdxl_ph1_encode(id, inst, ctx->priv.rx, n,
                                                ctx->tx, sizeof(ctx->tx), &ctx->tx_len);
        if (r != JDXL_OK) return r;
        ctx->priv.echo_left = ctx->opts.tx_echo ? ctx->tx_len : 0;
        return JDXL_OK;
}

#define EMIT(ctx, id, inst, n) \
        do { const jdxl_result_t r_ = emit((ctx), (id), (inst), (n)); if (r_ != JDXL_OK) return r_; } while (0)

static void expect(jdxl_ph1_ctx_t *ctx, uint8_t id, uint16_t len)
{
        ctx->priv.slot[ctx->priv.count].id       = id;
        ctx->priv.slot[ctx->priv.count].data_len = len;
        ctx->priv.count++;
}

/* Unicast non-read instructions are answered only at Status Return Level ALL; broadcast never. */
static void expect_ack(jdxl_ph1_ctx_t *ctx, uint8_t id)
{
        if (id != JDXL_BROADCAST_ID && ctx->opts.status_return_level >= JDXL_SRL_ALL)
                expect(ctx, id, 0);
}

static int reads_answered(const jdxl_ph1_ctx_t *ctx)
{
        return ctx->opts.status_return_level >= JDXL_SRL_READ;
}

jdxl_result_t jdxl_ph1_build_ping(jdxl_ph1_ctx_t *ctx, uint8_t id)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_unicast(id)) return JDXL_ERR_INVALID_ID;

        begin(ctx);
        EMIT(ctx, id, JDXL_PH1_INST_PING, 0);
        expect(ctx, id, 0);     /* "Regardless of the Status Return Level, the Ping Instruction returns the Status Packet" */
        return JDXL_OK;
}

jdxl_result_t jdxl_ph1_build_read(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr, uint8_t len)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (!is_unicast(id)) return JDXL_ERR_INVALID_ID;
        if (len == 0) return JDXL_ERR_EMPTY;
        if (!fits(2) || !reply_fits(len)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        ctx->priv.rx[0] = addr;
        ctx->priv.rx[1] = len;
        EMIT(ctx, id, JDXL_PH1_INST_READ, 2);
        if (reads_answered(ctx)) expect(ctx, id, len);
        return JDXL_OK;
}

static jdxl_result_t write_family(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr,
                                  const uint8_t *data, size_t len, uint8_t inst)
{
        if (ctx == NULL || (data == NULL && len > 0)) return JDXL_ERR_NULL_ARG;
        if (id == 0xFF) return JDXL_ERR_INVALID_ID;
        if (len == 0) return JDXL_ERR_EMPTY;
        if (!fits(1 + len)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        ctx->priv.rx[0] = addr;
        memcpy(&ctx->priv.rx[1], data, len);
        EMIT(ctx, id, inst, 1 + len);
        expect_ack(ctx, id);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph1_build_write(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr,
                                   const uint8_t *data, size_t len)
{
        return write_family(ctx, id, addr, data, len, JDXL_PH1_INST_WRITE);
}

jdxl_result_t jdxl_ph1_build_reg_write(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t addr,
                                       const uint8_t *data, size_t len)
{
        return write_family(ctx, id, addr, data, len, JDXL_PH1_INST_REG_WRITE);
}

static jdxl_result_t no_param(jdxl_ph1_ctx_t *ctx, uint8_t id, uint8_t inst)
{
        if (ctx == NULL) return JDXL_ERR_NULL_ARG;
        if (id == 0xFF) return JDXL_ERR_INVALID_ID;

        begin(ctx);
        EMIT(ctx, id, inst, 0);
        expect_ack(ctx, id);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph1_build_action(jdxl_ph1_ctx_t *ctx, uint8_t id)
{
        return no_param(ctx, id, JDXL_PH1_INST_ACTION);
}

jdxl_result_t jdxl_ph1_build_factory_reset(jdxl_ph1_ctx_t *ctx, uint8_t id)
{
        /* "Broadcast ID(0xFE) cannot be used for Factory Reset instruction." */
        if (ctx != NULL && !is_unicast(id)) return JDXL_ERR_INVALID_ID;
        return no_param(ctx, id, JDXL_PH1_INST_FACTORY_RESET);
}

jdxl_result_t jdxl_ph1_build_reboot(jdxl_ph1_ctx_t *ctx, uint8_t id)
{
        return no_param(ctx, id, JDXL_PH1_INST_REBOOT);
}

jdxl_result_t jdxl_ph1_build_sync_write(jdxl_ph1_ctx_t *ctx, uint8_t addr, uint8_t len,
                                        const jdxl_ph1_sync_write_t *entries, uint8_t count)
{
        uint32_t seen[8] = { 0 };
        size_t   need, w = 0;
        uint8_t  i;

        if (ctx == NULL || (entries == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0 || len == 0) return JDXL_ERR_EMPTY;
        for (i = 0; i < count; i++) {
                if (entries[i].data == NULL) return JDXL_ERR_NULL_ARG;
                if (!is_unicast(entries[i].id)) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, entries[i].id)) return JDXL_ERR_DUPLICATE_ID;
        }
        need = 2u + (size_t)count * (1u + len);         /* Length = ((L + 1) * N) + 4 */
        if (!fits(need)) return JDXL_ERR_TOO_LONG;

        begin(ctx);
        ctx->priv.rx[w++] = addr;
        ctx->priv.rx[w++] = len;
        for (i = 0; i < count; i++) {
                ctx->priv.rx[w++] = entries[i].id;
                memcpy(&ctx->priv.rx[w], entries[i].data, len);
                w += len;
        }
        EMIT(ctx, JDXL_BROADCAST_ID, JDXL_PH1_INST_SYNC_WRITE, w);
        return JDXL_OK;
}

jdxl_result_t jdxl_ph1_build_bulk_read(jdxl_ph1_ctx_t *ctx,
                                       const jdxl_ph1_bulk_read_t *entries, uint8_t count)
{
        uint32_t seen[8] = { 0 };
        size_t   w = 0;
        uint8_t  i;

        if (ctx == NULL || (entries == NULL && count > 0)) return JDXL_ERR_NULL_ARG;
        if (count == 0) return JDXL_ERR_EMPTY;
        if (count > JDXL_PH1_MAX_STATUS) return JDXL_ERR_TOO_MANY_STATUS;
        for (i = 0; i < count; i++) {
                if (entries[i].len == 0) return JDXL_ERR_EMPTY;
                if (!is_unicast(entries[i].id)) return JDXL_ERR_INVALID_ID;
                if (jdxl_seen(seen, entries[i].id)) return JDXL_ERR_DUPLICATE_ID;
                if (!reply_fits(entries[i].len)) return JDXL_ERR_TOO_LONG;
        }
        if (!fits(1u + 3u * (size_t)count)) return JDXL_ERR_TOO_LONG;   /* Length = 3N + 3 */

        begin(ctx);
        ctx->priv.rx[w++] = 0x00;
        for (i = 0; i < count; i++) {           /* wire order: length, ID, address */
                ctx->priv.rx[w++] = entries[i].len;
                ctx->priv.rx[w++] = entries[i].id;
                ctx->priv.rx[w++] = entries[i].addr;
        }
        EMIT(ctx, JDXL_BROADCAST_ID, JDXL_PH1_INST_BULK_READ, w);
        if (reads_answered(ctx)) {
                for (i = 0; i < count; i++) expect(ctx, entries[i].id, entries[i].len);
        }
        return JDXL_OK;
}

// ------------------------------------------------------------------------------------------------
// Feed. priv.rx holds one candidate packet starting at rx[0]. After every byte the candidate is
// checked; a rejected candidate slides by one byte, so the parser resynchronizes by itself and a
// header inside garbage (or after stray 0xFF bytes) is still found.
// ------------------------------------------------------------------------------------------------

static void drop_front(jdxl_ph1_ctx_t *ctx, size_t k)
{
        memmove(ctx->priv.rx, ctx->priv.rx + k, ctx->priv.rx_len - k);
        ctx->priv.rx_len -= k;
}

static int rx_check(jdxl_ph1_ctx_t *ctx, int *slot)
{
        const uint8_t *p = ctx->priv.rx;
        const size_t   n = ctx->priv.rx_len;
        size_t         need;

        if (n == 0) return JDXL_RX_MORE;
        if (p[0] != 0xFF || (n > 1 && p[1] != 0xFF)) return JDXL_RX_SKIP;
        if (n <= IDX_ID) return JDXL_RX_MORE;
        if (p[IDX_ID] == 0xFF) return JDXL_RX_SKIP;     /* FF FF FF: the header is one byte later */
        if (n <= IDX_LEN) return JDXL_RX_MORE;

        *slot = jdxl_find_slot(ctx->priv.slot, ctx->priv.idx, ctx->priv.count, p[IDX_ID]);
        if (*slot < 0) {
                ctx->debug.unexpected_id++;
                return JDXL_RX_DROP;
        }
        /* LENGTH = ERROR + params + CHECKSUM: at least 2, at most what was requested */
        if (p[IDX_LEN] < 2u || p[IDX_LEN] > ctx->priv.slot[*slot].data_len + 2u) {
                ctx->debug.bad_length++;
                return JDXL_RX_DROP;
        }

        need = 4u + p[IDX_LEN];
        if (n < need) return JDXL_RX_MORE;
        if (checksum(&p[IDX_ID], need - 3u) != p[need - 1u]) {
                ctx->debug.bad_checksum++;
                return JDXL_RX_DROP;
        }
        return JDXL_RX_DONE;
}

static void deliver(jdxl_ph1_ctx_t *ctx, int slot, jdxl_on_status_t cb, void *user)
{
        const uint8_t *p = ctx->priv.rx;
        jdxl_status_t  s;

        ctx->debug.missed += (uint32_t)(slot - ctx->priv.idx);
        ctx->debug.status_ok++;
        ctx->priv.idx = (uint8_t)(slot + 1);

        s.index      = (uint8_t)slot;
        s.id         = p[IDX_ID];
        s.error      = p[IDX_ERR];
        s.params     = &p[IDX_PARAM0];
        s.params_len = (uint16_t)(p[IDX_LEN] - 2u);
        if (cb != NULL) cb(&s, user);

        if (ctx->priv.idx == ctx->priv.count) {
                ctx->priv.count = 0;
                ctx->priv.idx   = 0;
                ctx->debug.exchanges_done++;
        }
}

static void rx_byte(jdxl_ph1_ctx_t *ctx, uint8_t b, jdxl_on_status_t cb, void *user)
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

jdxl_feed_t jdxl_ph1_feed(jdxl_ph1_ctx_t *ctx, const uint8_t *buf, size_t len,
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
                        /* No echo after all (or a corrupted one): the bytes taken for it may be
                         * the start of a reply, so replay them as received data. */
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