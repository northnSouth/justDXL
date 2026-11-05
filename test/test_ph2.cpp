// Protocol 2.0: every packet example from the spec (protocol2.md), an independent reference
// encoder (bitwise CRC + byte stuffing), and regression/robustness tests.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "jdxl/jdxl_ph2.h"

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

// Independent CRC: bitwise, poly 0x8005, init 0, MSB first. Deliberately not table driven.
uint16_t Crc16Ref(const uint8_t *d, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(d[i]) << 8));
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x8005u)
                                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// Independent packet builder. body = INST + params (or 0x55 + ERR + params); FF FF FD after INST
// is stuffed to FF FF FD FD when `stuff` is set.
Bytes RefPacket(uint8_t id, const Bytes &body, bool stuff)
{
    Bytes b;
    for (size_t i = 0; i < body.size(); i++) {
        b.push_back(body[i]);
        const size_t n = b.size();
        if (stuff && i >= 3 && b[n - 3] == 0xFF && b[n - 2] == 0xFF && b[n - 1] == 0xFD)
            b.push_back(0xFD);
    }
    const size_t len = b.size() + 2;
    Bytes p = { 0xFF, 0xFF, 0xFD, 0x00, id, static_cast<uint8_t>(len), static_cast<uint8_t>(len >> 8) };
    p.insert(p.end(), b.begin(), b.end());
    const uint16_t crc = Crc16Ref(p.data(), p.size());
    p.push_back(static_cast<uint8_t>(crc));
    p.push_back(static_cast<uint8_t>(crc >> 8));
    return p;
}

Bytes Status(uint8_t id, uint8_t err, const Bytes &params, bool stuff = true)
{
    Bytes body = { 0x55, err };
    body.insert(body.end(), params.begin(), params.end());
    return RefPacket(id, body, stuff);
}

// Status packets from the spec
const Bytes kPing1 = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D };
const Bytes kPing2 = { 0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x6F, 0x6D };
const Bytes kRead1 = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0 };
const Bytes kRead2 = { 0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x08, 0x00, 0x55, 0x00, 0x1F, 0x08, 0x00, 0x00, 0xBA, 0xBE };
const Bytes kAck1  = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C };
const Bytes kBulk1 = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, 0x55, 0x00, 0x77, 0x00, 0xC3, 0x69 };
const Bytes kBulk2 = { 0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x05, 0x00, 0x55, 0x00, 0x24, 0x8B, 0xA9 };
const Bytes kFastSync = {
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, 0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08,
    0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA,
    0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E };
const Bytes kFastBulk = {
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x14, 0x00, 0x55, 0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x67, 0xA4,
    0x00, 0x07, 0xA5, 0x01, 0x24, 0x74,
    0x00, 0x04, 0x1F, 0xD9, 0xC1 };

class Ph2 : public ::testing::Test {
protected:
    void SetUp() override { Init(nullptr); }
    void Init(const jdxl_opts_t *o)
    {
        jdxl_ph2_init(ctx, o);
        got.clear();
    }
    Bytes Tx() const
    {
        const jdxl_span_t s = jdxl_ph2_tx(ctx);
        return Bytes(s.data, s.data + s.len);
    }
    jdxl_feed_t Feed(const Bytes &b) { return jdxl_ph2_feed(ctx, b.data(), b.size(), Collect, &got); }
    void ExpectGot(size_t i, uint8_t index, uint8_t id, const Bytes &params) const
    {
        SCOPED_TRACE("got[" + std::to_string(i) + "]");
        ASSERT_LT(i, got.size());
        EXPECT_EQ(got[i].index, index);
        EXPECT_EQ(got[i].id, id);
        EXPECT_EQ(got[i].error, 0x00);
        EXPECT_EQ(got[i].params, params);
    }

    std::unique_ptr<jdxl_ph2_ctx_t> holder = std::make_unique<jdxl_ph2_ctx_t>();
    jdxl_ph2_ctx_t                 *ctx    = holder.get();
    std::vector<Got>                got;
};

using Ph2Spec   = Ph2;
using Ph2Robust = Ph2;

TEST(Ph2Ref, ReferenceEncoderMatchesSpec)
{
    EXPECT_EQ(Status(0x01, 0x00, { 0x06, 0x04, 0x26 }), kPing1);
    EXPECT_EQ(Status(0x01, 0x00, { 0x77, 0x00 }), kBulk1);
    EXPECT_EQ(RefPacket(0xFE, { 0x01 }, true),
              (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x03, 0x00, 0x01, 0x31, 0x42 }));
}

// ---------------------------------------------------------------------------------------------
// Instruction packets, byte-exact against the spec
// ---------------------------------------------------------------------------------------------

TEST_F(Ph2Spec, Ping)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E }));
}

TEST_F(Ph2Spec, PingBroadcast)
{
    ASSERT_EQ(jdxl_ph2_build_ping_broadcast(ctx, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x03, 0x00, 0x01, 0x31, 0x42 }));
}

TEST_F(Ph2Spec, Read)
{
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00,
                            0x1D, 0x15 }));
}

TEST_F(Ph2Spec, Write)
{
    const uint8_t d[] = { 0x00, 0x02, 0x00, 0x00 };
    ASSERT_EQ(jdxl_ph2_build_write(ctx, 0x01, 116, d, 4), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02,
                            0x00, 0x00, 0xCA, 0x89 }));
}

TEST_F(Ph2Spec, RegWrite)
{
    const uint8_t d[] = { 0xC8, 0x00, 0x00, 0x00 };
    ASSERT_EQ(jdxl_ph2_build_reg_write(ctx, 0x01, 104, d, 4), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x04, 0x68, 0x00, 0xC8, 0x00,
                            0x00, 0x00, 0xAE, 0x8E }));
}

TEST_F(Ph2Spec, Action)
{
    ASSERT_EQ(jdxl_ph2_build_action(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x05, 0x02, 0xCE }));
}

TEST_F(Ph2Spec, FactoryReset)
{
    ASSERT_EQ(jdxl_ph2_build_factory_reset(ctx, 0x01, JDXL_PH2_RESET_ALL_EXCEPT_ID), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x06, 0x01, 0xA1, 0xE6 }));
}

TEST_F(Ph2Spec, Reboot)
{
    ASSERT_EQ(jdxl_ph2_build_reboot(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x08, 0x2F, 0x4E }));
}

TEST_F(Ph2Spec, Clear)
{
    ASSERT_EQ(jdxl_ph2_build_clear(ctx, 0x01, JDXL_PH2_CLEAR_MULTI_TURN), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x10, 0x01, 0x44, 0x58, 0x4C,
                            0x22, 0xB1, 0xDC }));
}

TEST_F(Ph2Spec, CtrlTableBackup)
{
    ASSERT_EQ(jdxl_ph2_build_ctrl_table_backup(ctx, 0x01, JDXL_PH2_BACKUP_STORE), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x20, 0x01, 0x43, 0x54, 0x52,
                            0x4C, 0x16, 0xF5 }));
}

TEST_F(Ph2Spec, SyncRead)
{
    const uint8_t ids[] = { 0x01, 0x02 };
    ASSERT_EQ(jdxl_ph2_build_sync_read(ctx, 132, 4, ids, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x82, 0x84, 0x00, 0x04, 0x00,
                            0x01, 0x02, 0xCE, 0xFA }));
}

TEST_F(Ph2Spec, SyncWrite)
{
    const uint8_t d1[] = { 0x96, 0x00, 0x00, 0x00 }, d2[] = { 0xAA, 0x00, 0x00, 0x00 };
    const jdxl_ph2_sync_write_t e[] = { { 0x01, d1 }, { 0x02, d2 } };
    ASSERT_EQ(jdxl_ph2_build_sync_write(ctx, 116, 4, e, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00,
                            0x01, 0x96, 0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87 }));
}

TEST_F(Ph2Spec, FastSyncRead)
{
    const uint8_t ids[] = { 0x03, 0x07, 0x04 };
    ASSERT_EQ(jdxl_ph2_build_fast_sync_read(ctx, 132, 4, ids, 3), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0A, 0x00, 0x8A, 0x84, 0x00, 0x04, 0x00,
                            0x03, 0x07, 0x04, 0x20, 0xF2 }));
}

TEST_F(Ph2Spec, BulkRead)
{
    const jdxl_ph2_bulk_read_t e[] = { { 0x01, 0x90, 2 }, { 0x02, 0x92, 1 } };
    ASSERT_EQ(jdxl_ph2_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0D, 0x00, 0x92, 0x01, 0x90, 0x00, 0x02,
                            0x00, 0x02, 0x92, 0x00, 0x01, 0x00, 0x1A, 0x05 }));
}

TEST_F(Ph2Spec, BulkWrite)
{
    const uint8_t d1[] = { 0xA0, 0x00 }, d2[] = { 0x50 };
    const jdxl_ph2_bulk_write_t e[] = { { 0x01, 0x20, 2, d1 }, { 0x02, 0x1F, 1, d2 } };
    ASSERT_EQ(jdxl_ph2_build_bulk_write(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x10, 0x00, 0x93, 0x01, 0x20, 0x00, 0x02,
                            0x00, 0xA0, 0x00, 0x02, 0x1F, 0x00, 0x01, 0x00, 0x50, 0xB7, 0x68 }));
}

TEST_F(Ph2Spec, FastBulkRead)
{
    const jdxl_ph2_bulk_read_t e[] = { { 0x03, 132, 4 }, { 0x07, 124, 2 }, { 0x04, 146, 1 } };
    ASSERT_EQ(jdxl_ph2_build_fast_bulk_read(ctx, e, 3), JDXL_OK);
    EXPECT_EQ(Tx(), (Bytes{ 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x12, 0x00, 0x9A, 0x03, 0x84, 0x00, 0x04,
                            0x00, 0x07, 0x7C, 0x00, 0x02, 0x00, 0x04, 0x92, 0x00, 0x01, 0x00, 0xDA,
                            0x2D }));
}

// ---------------------------------------------------------------------------------------------
// Status packets from the spec
// ---------------------------------------------------------------------------------------------

TEST_F(Ph2Spec, PingStatus)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kPing1), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0x06, 0x04, 0x26 });
}

TEST_F(Ph2Spec, BroadcastPingStatuses)
{
    ASSERT_EQ(jdxl_ph2_build_ping_broadcast(ctx, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kPing1, kPing2 })), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0x06, 0x04, 0x26 });
    ExpectGot(1, 1, 0x02, { 0x06, 0x04, 0x26 });
}

TEST_F(Ph2Spec, ReadStatus)
{
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(Feed(kRead1), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0xA6, 0x00, 0x00, 0x00 });
}

TEST_F(Ph2Spec, WriteStatus)
{
    const uint8_t d[] = { 0x00, 0x02, 0x00, 0x00 };
    ASSERT_EQ(jdxl_ph2_build_write(ctx, 0x01, 116, d, 4), JDXL_OK);
    EXPECT_EQ(Feed(kAck1), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, {});
}

TEST_F(Ph2Spec, SyncReadStatuses)
{
    const uint8_t ids[] = { 0x01, 0x02 };
    ASSERT_EQ(jdxl_ph2_build_sync_read(ctx, 132, 4, ids, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kRead1, kRead2 })), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0xA6, 0x00, 0x00, 0x00 });
    ExpectGot(1, 1, 0x02, { 0x1F, 0x08, 0x00, 0x00 });
}

TEST_F(Ph2Spec, BulkReadStatuses)
{
    const jdxl_ph2_bulk_read_t e[] = { { 0x01, 0x90, 2 }, { 0x02, 0x92, 1 } };
    ASSERT_EQ(jdxl_ph2_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kBulk1, kBulk2 })), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0x77, 0x00 });
    ExpectGot(1, 1, 0x02, { 0x24 });
}

TEST_F(Ph2Spec, FastSyncReadStatus)
{
    const uint8_t ids[] = { 0x03, 0x07, 0x04 };
    ASSERT_EQ(jdxl_ph2_build_fast_sync_read(ctx, 132, 4, ids, 3), JDXL_OK);
    EXPECT_EQ(Feed(kFastSync), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x03, { 0xA6, 0x00, 0x00, 0x00 });
    ExpectGot(1, 1, 0x07, { 0x1F, 0x08, 0x00, 0x00 });
    ExpectGot(2, 2, 0x04, { 0xFF, 0x03, 0x00, 0x00 });
}

TEST_F(Ph2Spec, FastBulkReadStatus)
{
    const jdxl_ph2_bulk_read_t e[] = { { 0x03, 132, 4 }, { 0x07, 124, 2 }, { 0x04, 146, 1 } };
    ASSERT_EQ(jdxl_ph2_build_fast_bulk_read(ctx, e, 3), JDXL_OK);
    EXPECT_EQ(Feed(kFastBulk), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x03, { 0xA6, 0x00, 0x00, 0x00 });
    ExpectGot(1, 1, 0x07, { 0xA5, 0x01 });
    ExpectGot(2, 2, 0x04, { 0x1F });
}

// ---------------------------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------------------------

TEST_F(Ph2Robust, ZeroLengthStatusIsRejectedWithoutOverflow)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    Bytes b = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x00, 0x00, 0x55 };
    b.resize(b.size() + 400, 0x00);
    EXPECT_EQ(Feed(b), JDXL_FEED_PENDING);
    EXPECT_EQ(ctx->debug.bad_length, 1u);
    EXPECT_TRUE(got.empty());
}

TEST_F(Ph2Robust, StrayFFBeforeHeader)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ { 0xFF }, kPing1 })), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
}

TEST_F(Ph2Robust, StuffingIsNotAHeader)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ { 0xFF, 0xFF, 0xFD, 0xFD }, kPing1 })), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
    EXPECT_EQ(ctx->debug.bad_length + ctx->debug.not_status + ctx->debug.bad_checksum, 0u);
}

TEST_F(Ph2Robust, StuffedStatusParamsAreExact)
{
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(Feed(Status(0x01, 0x00, { 0xFF, 0xFF, 0xFD, 0x07 })), JDXL_FEED_DONE);
    ExpectGot(0, 0, 0x01, { 0xFF, 0xFF, 0xFD, 0x07 });
}

TEST_F(Ph2Robust, WriteIsStuffedLikeTheReference)
{
    const uint8_t d[] = { 0xFF, 0xFF, 0xFD, 0x00 };
    ASSERT_EQ(jdxl_ph2_build_write(ctx, 0x01, 116, d, 4), JDXL_OK);
    EXPECT_EQ(Tx(), RefPacket(0x01, { 0x03, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0x00 }, true));
}

TEST_F(Ph2Robust, ShortPacketIsStuffedToo)
{
    // LENGTH 6 before stuffing: the spec stuffs, ROBOTIS's SDK does not
    const uint8_t d[] = { 0xFD };
    ASSERT_EQ(jdxl_ph2_build_write(ctx, 0x01, 0xFFFF, d, 1), JDXL_OK);
    EXPECT_EQ(Tx(), RefPacket(0x01, { 0x03, 0xFF, 0xFF, 0xFD }, true));
}

TEST_F(Ph2Robust, LongerFastReplyIsRejected)
{
    const uint8_t ids[] = { 0x03 };
    ASSERT_EQ(jdxl_ph2_build_fast_sync_read(ctx, 132, 4, ids, 1), JDXL_OK);
    EXPECT_EQ(Feed(kFastSync), JDXL_FEED_PENDING);
    EXPECT_EQ(ctx->debug.bad_length, 1u);
    EXPECT_TRUE(got.empty());
}

TEST_F(Ph2Robust, TooManyStatusRepliesLeavesContextUntouched)
{
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    std::vector<uint8_t> ids(JDXL_PH2_MAX_STATUS + 1);
    for (size_t i = 0; i < ids.size(); i++) ids[i] = static_cast<uint8_t>(i);
    EXPECT_EQ(jdxl_ph2_build_sync_read(ctx, 132, 4, ids.data(), static_cast<uint8_t>(ids.size())),
              JDXL_ERR_TOO_MANY_STATUS);
    EXPECT_EQ(jdxl_ph2_pending(ctx), 1);
    EXPECT_EQ(Feed(kPing1), JDXL_FEED_DONE);
}

TEST_F(Ph2Robust, AbortedExchangeDoesNotLeakIntoTheNextOne)
{
    const jdxl_ph2_bulk_read_t e[] = { { 0x01, 0x90, 2 }, { 0x02, 0x92, 1 } };
    ASSERT_EQ(jdxl_ph2_build_bulk_read(ctx, e, 2), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ kBulk1, Bytes(kBulk2.begin(), kBulk2.begin() + 5) })), JDXL_FEED_PENDING);

    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(Feed(kPing1), JDXL_FEED_DONE);
    ASSERT_EQ(got.size(), 2u);
    ExpectGot(1, 0, 0x01, { 0x06, 0x04, 0x26 });
    EXPECT_EQ(ctx->debug.missed, 1u);
}

TEST_F(Ph2Robust, MissingReplyIsSkippedInsteadOfBlocking)
{
    const uint8_t ids[] = { 0x01, 0x02 };
    ASSERT_EQ(jdxl_ph2_build_sync_read(ctx, 132, 4, ids, 2), JDXL_OK);
    EXPECT_EQ(Feed(kRead2), JDXL_FEED_DONE);
    ExpectGot(0, 1, 0x02, { 0x1F, 0x08, 0x00, 0x00 });
    EXPECT_EQ(ctx->debug.missed, 1u);
}

TEST_F(Ph2Robust, EchoWithoutOptionIsCountedAsNotStatus)
{
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ Tx(), kRead1 })), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
    EXPECT_EQ(ctx->debug.not_status, 1u);
}

TEST_F(Ph2Robust, EchoIsSkippedWhenConfigured)
{
    const jdxl_opts_t o = { JDXL_SRL_ALL, true };
    Init(&o);
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(Feed(Cat({ Tx(), kRead1 })), JDXL_FEED_DONE);
    EXPECT_EQ(got.size(), 1u);
    EXPECT_EQ(ctx->debug.not_status, 0u);
}

TEST_F(Ph2Robust, BuildRejections)
{
    EXPECT_EQ(jdxl_ph2_build_read(ctx, 0x01, 0, JDXL_PH2_PKT_MAX_LEN), JDXL_ERR_TOO_LONG);
    EXPECT_EQ(jdxl_ph2_build_read(ctx, JDXL_BROADCAST_ID, 0, 1), JDXL_ERR_INVALID_ID);
    EXPECT_EQ(jdxl_ph2_build_factory_reset(ctx, JDXL_BROADCAST_ID, JDXL_PH2_RESET_ALL), JDXL_ERR_INVALID_ID);
    EXPECT_EQ(jdxl_ph2_build_factory_reset(ctx, 0x01, static_cast<jdxl_ph2_reset_t>(0x05)), JDXL_ERR_INVALID_ARG);
    EXPECT_EQ(jdxl_ph2_build_ping(ctx, 0xFD), JDXL_ERR_INVALID_ID);

    const jdxl_ph2_bulk_read_t dup[] = { { 0x01, 0, 1 }, { 0x01, 2, 1 } };
    EXPECT_EQ(jdxl_ph2_build_bulk_read(ctx, dup, 2), JDXL_ERR_DUPLICATE_ID);
}

TEST_F(Ph2Robust, EncodeRejectsHugeParamCountWithoutOverflow)
{
    uint8_t p[16] = {}, out[16];
    size_t  n = 0;
    EXPECT_EQ(jdxl_ph2_encode(0x01, JDXL_PH2_INST_WRITE, p, 65534, out, sizeof out, &n), JDXL_ERR_TOO_LONG);
    EXPECT_EQ(n, 0u);
}

TEST_F(Ph2Robust, ByteByByteFastSyncRead)
{
    const uint8_t ids[] = { 0x03, 0x07, 0x04 };
    ASSERT_EQ(jdxl_ph2_build_fast_sync_read(ctx, 132, 4, ids, 3), JDXL_OK);
    for (size_t i = 0; i < kFastSync.size(); i++) {
        const jdxl_feed_t r = Feed(Bytes(1, kFastSync[i]));
        EXPECT_EQ(r, i + 1 < kFastSync.size() ? JDXL_FEED_PENDING : JDXL_FEED_DONE);
    }
    EXPECT_EQ(got.size(), 3u);
}

TEST_F(Ph2Robust, StatusReturnLevelGatesReplies)
{
    const jdxl_opts_t o = { JDXL_SRL_PING_ONLY, false };
    Init(&o);
    ASSERT_EQ(jdxl_ph2_build_read(ctx, 0x01, 132, 4), JDXL_OK);
    EXPECT_EQ(jdxl_ph2_pending(ctx), 0);
    ASSERT_EQ(jdxl_ph2_build_ping(ctx, 0x01), JDXL_OK);
    EXPECT_EQ(jdxl_ph2_pending(ctx), 1);
}

TEST_F(Ph2Robust, NullArguments)
{
    jdxl_ph2_init(nullptr, nullptr);
    EXPECT_EQ(jdxl_ph2_build_ping(nullptr, 1), JDXL_ERR_NULL_ARG);
    EXPECT_EQ(jdxl_ph2_build_sync_read(ctx, 0, 1, nullptr, 2), JDXL_ERR_NULL_ARG);
    EXPECT_EQ(jdxl_ph2_feed(nullptr, nullptr, 10, nullptr, nullptr), JDXL_FEED_IDLE);
    EXPECT_EQ(jdxl_ph2_tx(nullptr).len, 0u);
    jdxl_ph2_abort(nullptr);
}

} // namespace