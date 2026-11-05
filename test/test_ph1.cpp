// Protocol 1.0: every packet example from the spec (protocol1.md), then regression and
// robustness tests for the issues found in the audit.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "jdxl/jdxl_ph1.h"

namespace {

using Bytes = std::vector<uint8_t>;

struct Got {
    uint8_t index, id, error;
    Bytes   params;
};

void Collect(const jdxl_status_t *s, void *user)
{
    static_cast<std::vector<Got> *>(user)->push_back(
        { s->index, s->id, s->error, Bytes(s->params, s->params + s->params_len) });
}

Bytes Cat(std::initializer_list<Bytes> parts)
{
    Bytes out;
    for (const Bytes &p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

// Status packets from the spec
const Bytes kPingStatus1  = { 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC };
const Bytes kReadStatus1  = { 0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB };
const Bytes kErrorStatus1 = { 0xFF, 0xFF, 0x01, 0x02, 0x24, 0xD8 };
const Bytes kResetStatus0 = { 0xFF, 0xFF, 0x00, 0x02, 0x00, 0xFD };
const Bytes kBulkStatus1  = { 0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x80, 0x7A };
const Bytes kBulkStatus2  = { 0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x80, 0x79 };

class Ph1 : public ::testing::Test {
protected:
    void SetUp() override { Init(nullptr); }
    void Init(const jdxl_opts_t *o)
    {
        jdxl_ph1_init(ctx, o);
        got.clear();
    }
    Bytes Tx() const
    {
        const jdxl_span_t s = jdxl_ph1_tx(ctx);
        return Bytes(s.data, s.data + s.len);
    }
    jdxl_feed_t Feed(const Bytes &b) { return jdxl_ph1_feed(ctx, b.data(), b.size(), Collect, &got); }

    std::unique_ptr<jdxl_ph1_ctx_t> holder = std::make_unique<jdxl_ph1_ctx_t>();
    jdxl_ph1_ctx_t                 *ctx    = holder.get();
    std::vector<Got>                got;
};

using Ph1Spec   = Ph1;
using Ph1Robust = Ph1;

// ---------------------------------------------------------------------------------------------
// Instruction packets, byte-exact against the spec
// ---------------------------------------------------------------------------------------------

TEST_F(Ph1Spec, Ping)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB }));
}

TEST_F(Ph1Spec, Read)
{
    ASSERT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x2B, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC }));
}

TEST_F(Ph1Spec, WriteBroadcast)
{
    const uint8_t d[] = { 0x01 };
    ASSERT_EQ(jdxl_ph1_build_write(ctx, JDXL_BROADCAST_ID, 0x03, d, 1), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, 0x01, 0xF6 }));
    EXPECT_EQ(jdxl_ph1_pending(ctx), 0);   // broadcast: no status
}

TEST_F(Ph1Spec, RegWrite)
{
    const uint8_t d[] = { 0xF4, 0x01 };
    ASSERT_EQ(jdxl_ph1_build_reg_write(ctx, 0x01, 0x1E, d, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0x01, 0x05, 0x04, 0x1E, 0xF4, 0x01, 0xE2 }));
}

TEST_F(Ph1Spec, ActionBroadcast)
{
    ASSERT_EQ(jdxl_ph1_build_action(ctx, JDXL_BROADCAST_ID), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA }));
}

TEST_F(Ph1Spec, FactoryReset)
{
    ASSERT_EQ(jdxl_ph1_build_factory_reset(ctx, 0x00), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0x00, 0x02, 0x06, 0xF7 }));
}

TEST_F(Ph1Spec, Reboot)
{
    ASSERT_EQ(jdxl_ph1_build_reboot(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0x01, 0x02, 0x08, 0xF4 }));
}

TEST_F(Ph1Spec, SyncWrite)
{
    const uint8_t d0[] = { 0x10, 0x00, 0x50, 0x01 }, d1[] = { 0x20, 0x02, 0x60, 0x03 };
    const jdxl_ph1_sync_write_t e[] = { { 0x00, d0 }, { 0x01, d1 } };
    ASSERT_EQ(jdxl_ph1_build_sync_write(ctx, 0x1E, 4, e, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFE, 0x0E, 0x83, 0x1E, 0x04, 0x00, 0x10, 0x00, 0x50,
                            0x01, 0x01, 0x20, 0x02, 0x60, 0x03, 0x67 }));
}

TEST_F(Ph1Spec, BulkRead)
{
    const jdxl_ph1_bulk_read_t e[] = { { 0x01, 0x1E, 2 }, { 0x02, 0x24, 2 } };
    ASSERT_EQ(jdxl_ph1_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFE, 0x09, 0x92, 0x00, 0x02, 0x01, 0x1E, 0x02, 0x02,
                            0x24, 0x1D }));
}

TEST_F(Ph1Spec, EncodeMatchesBuilder)
{
    const uint8_t p[] = { 0x1E, 0xF4, 0x01 };
    uint8_t       out[16];
    size_t        n = 0;
    ASSERT_EQ(jdxl_ph1_encode(0x01, JDXL_PH1_INST_REG_WRITE, p, 3, out, sizeof out, &n), JDXL_OK);
    EXPECT_EQ(Bytes(out, out + n), (Bytes{ 0xFF, 0xFF, 0x01, 0x05, 0x04, 0x1E, 0xF4, 0x01, 0xE2 }));
}

// ---------------------------------------------------------------------------------------------
// Status packets from the spec
// ---------------------------------------------------------------------------------------------

TEST_F(Ph1Spec, PingStatus)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kPingStatus1), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].id, 0x01);
    EXPECT_EQ(got[0].error, 0x00);
    EXPECT_TRUE(got[0].params.empty());
    EXPECT_EQ(jdxl_ph1_pending(ctx), 0);
}

TEST_F(Ph1Spec, ReadStatus)
{
    ASSERT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x2B, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kReadStatus1), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].params, (Bytes{ 0x20 }));
}

TEST_F(Ph1Spec, ErrorBitsAreReported)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kErrorStatus1), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].error, JDXL_PH1_ERR_OVERLOAD | JDXL_PH1_ERR_OVERHEATING);
}

TEST_F(Ph1Spec, FactoryResetStatus)
{
    ASSERT_EQ(jdxl_ph1_build_factory_reset(ctx, 0x00), JDXL_OK);
    EXPECT_EQ(Feed(kResetStatus0), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].id, 0x00);
}

TEST_F(Ph1Spec, BulkReadStatusesInOneBuffer)
{
    const jdxl_ph1_bulk_read_t e[] = { { 0x01, 0x1E, 2 }, { 0x02, 0x24, 2 } };
    ASSERT_EQ(jdxl_ph1_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kBulkStatus1, kBulkStatus2 })), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].index, 0);
    EXPECT_EQ(got[0].id, 0x01);
    EXPECT_EQ(got[0].params, (Bytes{ 0x00, 0x80 }));
    EXPECT_EQ(got[1].index, 1);
    EXPECT_EQ(got[1].id, 0x02);
    EXPECT_EQ(ctx->debug.exchanges_done, 1u);
}

// ---------------------------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------------------------

TEST_F(Ph1Robust, FactoryResetBroadcastIsRejected)
{
    EXPECT_EQ(jdxl_ph1_build_factory_reset(ctx, JDXL_BROADCAST_ID), JDXL_ERR_INVALID_ID);
}

TEST_F(Ph1Robust, ValidationErrorLeavesPendingExchangeUntouched)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    const Bytes tx = Tx();
    EXPECT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x00, 0), JDXL_ERR_EMPTY);
    EXPECT_EQ(jdxl_ph1_pending(ctx), 1);
    EXPECT_EQ(Tx(), tx);
    EXPECT_EQ(Feed(kPingStatus1), JDXL_FEED_DONE);
}

TEST_F(Ph1Robust, ReadWhoseStatusCannotFitIsRejected)
{
    EXPECT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x00, 254), JDXL_ERR_TOO_LONG);
}

TEST_F(Ph1Robust, SyncWriteRejectsBadEntries)
{
    const uint8_t d[] = { 0 };
    const jdxl_ph1_sync_write_t bcast[] = { { JDXL_BROADCAST_ID, d } };
    const jdxl_ph1_sync_write_t dup[]   = { { 0x01, d }, { 0x01, d } };
    const jdxl_ph1_sync_write_t null[]  = { { 0x01, nullptr } };
    EXPECT_EQ(jdxl_ph1_build_sync_write(ctx, 0, 1, bcast, 1), JDXL_ERR_INVALID_ID);
    EXPECT_EQ(jdxl_ph1_build_sync_write(ctx, 0, 1, dup, 2), JDXL_ERR_DUPLICATE_ID);
    EXPECT_EQ(jdxl_ph1_build_sync_write(ctx, 0, 1, null, 1), JDXL_ERR_NULL_ARG);
}

TEST_F(Ph1Robust, StrayFFBeforeHeader)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ { 0xFF }, kPingStatus1 })), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
    EXPECT_EQ(ctx->debug.bad_length + ctx->debug.unexpected_id + ctx->debug.bad_checksum, 0u);
}

TEST_F(Ph1Robust, ChecksumMismatchThenRecovery)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed({ 0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00 }), JDXL_FEED_PENDING);
    EXPECT_EQ(ctx->debug.bad_checksum, 1u);
    EXPECT_EQ(Feed(kPingStatus1), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
}

TEST_F(Ph1Robust, UnexpectedIdIsSkippedAndCounted)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ { 0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB }, kPingStatus1 })), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].id, 0x01);
    EXPECT_EQ(ctx->debug.unexpected_id, 1u);
}

TEST_F(Ph1Robust, MissingReplyIsSkippedInsteadOfBlocking)
{
    const jdxl_ph1_bulk_read_t e[] = { { 0x01, 0x1E, 2 }, { 0x02, 0x24, 2 } };
    ASSERT_EQ(jdxl_ph1_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Feed(kBulkStatus2), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].index, 1);
    EXPECT_EQ(ctx->debug.missed, 1u);
}

TEST_F(Ph1Robust, AbortedExchangeDoesNotLeakIntoTheNextOne)
{
    const jdxl_ph1_bulk_read_t e[] = { { 0x01, 0x1E, 2 }, { 0x02, 0x24, 2 } };
    ASSERT_EQ(jdxl_ph1_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kBulkStatus1, Bytes(kBulkStatus2.begin(), kBulkStatus2.begin() + 3) })),
              JDXL_FEED_PENDING);

    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x03), JDXL_OK);   // supersedes, half packet dropped
    EXPECT_EQ(ctx->debug.missed, 1u);
    EXPECT_EQ(Feed({ 0xFF, 0xFF, 0x03, 0x02, 0x00, 0xFA }), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[1].id, 0x03);
}

TEST_F(Ph1Robust, AbortCountsMissedAndIgnoresLateBytes)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    jdxl_ph1_abort(ctx);
    EXPECT_EQ(jdxl_ph1_pending(ctx), 0);
    EXPECT_EQ(ctx->debug.missed, 1u);
    EXPECT_EQ(Feed(kPingStatus1), JDXL_FEED_IDLE);
    EXPECT_EQ(ctx->debug.ignored_bytes, kPingStatus1.size());
}

TEST_F(Ph1Robust, TrailingBytesAfterDoneAreIgnored)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kPingStatus1, { 0xAA, 0xBB } })), JDXL_FEED_DONE);
    EXPECT_EQ(ctx->debug.ignored_bytes, 2u);
}

TEST_F(Ph1Robust, ByteByByte)
{
    ASSERT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x2B, 0x01), JDXL_OK);
    for (size_t i = 0; i < kReadStatus1.size(); i++) {
        const jdxl_feed_t r = Feed(Bytes(1, kReadStatus1[i]));
        EXPECT_EQ(r, i + 1 < kReadStatus1.size() ? JDXL_FEED_PENDING : JDXL_FEED_DONE);
    }
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].params, (Bytes{ 0x20 }));
}

// Why tx_echo exists: in Protocol 1.0 a Ping echo is a valid "status" with ERR = 0x01.
TEST_F(Ph1Robust, WithoutEchoOptionAnEchoParsesAsStatus)
{
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Tx()), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].error, JDXL_PH1_INST_PING);
}

TEST_F(Ph1Robust, EchoIsSkippedWhenConfigured)
{
    const jdxl_opts_t o = { JDXL_SRL_ALL, true };
    Init(&o);
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ Tx(), kPingStatus1 })), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].error, 0x00);
    EXPECT_EQ(ctx->debug.echo_mismatch, 0u);
}

TEST_F(Ph1Robust, MissingEchoDoesNotLoseTheReply)
{
    const jdxl_opts_t o = { JDXL_SRL_ALL, true };
    Init(&o);
    ASSERT_EQ(jdxl_ph1_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kPingStatus1), JDXL_FEED_DONE);   // shares FF FF 01 02 with the instruction
    EXPECT_EQ(got.size(), 1u);
    EXPECT_EQ(ctx->debug.echo_mismatch, 1u);
}

TEST_F(Ph1Robust, StatusReturnLevelSuppressesWriteReply)
{
    const jdxl_opts_t o = { JDXL_SRL_READ, false };
    Init(&o);
    const uint8_t d[] = { 0x01 };
    ASSERT_EQ(jdxl_ph1_build_write(ctx, 0x01, 0x03, d, 1), JDXL_OK);
    EXPECT_EQ(jdxl_ph1_pending(ctx), 0);
    ASSERT_EQ(jdxl_ph1_build_read(ctx, 0x01, 0x2B, 1), JDXL_OK);
    EXPECT_EQ(jdxl_ph1_pending(ctx), 1);
}

TEST_F(Ph1Robust, NullArguments)
{
    jdxl_ph1_init(nullptr, nullptr);
    EXPECT_EQ(jdxl_ph1_build_ping(nullptr, 1), JDXL_ERR_NULL_ARG);
    EXPECT_EQ(jdxl_ph1_build_write(ctx, 1, 0, nullptr, 1), JDXL_ERR_NULL_ARG);
    EXPECT_EQ(jdxl_ph1_feed(nullptr, nullptr, 10, nullptr, nullptr), JDXL_FEED_IDLE);
    EXPECT_EQ(jdxl_ph1_feed(ctx, nullptr, 10, nullptr, nullptr), JDXL_FEED_IDLE);
    EXPECT_EQ(jdxl_ph1_tx(nullptr).len, 0u);
    EXPECT_EQ(jdxl_ph1_pending(nullptr), 0);
    jdxl_ph1_abort(nullptr);
}

} // namespace