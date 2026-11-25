// control_board_source_gtest.cpp
// The decoder table and dispatch. The Smart Port is faked: openSmartPort()
// below stands in for the PROS one.

#include "locagatr/sources/control_board_source.h"

#include <gtest/gtest.h>

#include "locagatr/smart_port.h"

namespace locagatr
{

int g_open_failures = 0; // opens that fail before one works
int g_opens         = 0; // calls

// A port where nothing ever arrives.
std::optional<Io> openSmartPort(uint8_t port, int32_t) {
    ++g_opens;
    if (port == 0 || g_open_failures > 0) {
        --g_open_failures;
        return std::nullopt;
    }
    Io io;
    io.read   = [](uint8_t*, std::size_t) -> std::size_t { return 0; };
    io.now_us = [] { return uint64_t{0}; };
    return io;
}

namespace
{

constexpr uint8_t kOtherType = 0x42;

using Decoders = std::vector<const RecordDecoder*>;

// A READINGS frame as the Link hands it over.
Poll answer(std::initializer_list<brainlink::Record> records) {
    brainlink::ReadingsWriter writer(0x1234);
    for (const brainlink::Record& r : records) {
        writer.add(r.sensor_type, r.port, r.health, r.age_us, r.data.data(), r.size);
    }
    const brainlink::Frame f = writer.frame(1);
    brainlink::Scanner     scanner;
    Poll                   poll;
    scanner.feed(f.bytes.data(), f.size, [&](const brainlink::Message& m) {
        poll.readings = *brainlink::decodeReadings(m);
    });
    poll.updated = true;
    poll.fresh   = true;
    return poll;
}

brainlink::Record encoder(uint8_t port, int32_t count) {
    brainlink::Record r;
    r.sensor_type = records::encoder::kSensorType;
    r.port        = port;
    r.health      = brainlink::Health::kOk;
    r.size        = records::encoder::kSize;
    records::encoder::write(count, r.data.data());
    return r;
}

brainlink::Record other(uint8_t port) {
    brainlink::Record r;
    r.sensor_type = kOtherType;
    r.port        = port;
    r.size        = 2;
    return r;
}

Config onPort(uint8_t port) {
    Config c;
    c.robot.board_port = port;
    return c;
}

TEST(ControlBoardSource, DefaultDecodesEncoderCountsByConnector) {
    ControlBoardSource board;
    ASSERT_TRUE(board.begin(onPort(16)));

    SensorMap m;
    board.decode(answer({encoder(0, 1520), encoder(2, -7)}), LinkStats{}, m);
    EXPECT_EQ(m.encoder_counts[0], 1520);
    EXPECT_FALSE(m.encoder_counts[1].has_value());
    EXPECT_EQ(m.encoder_counts[2], -7);
    EXPECT_TRUE(m.board.fresh);
    EXPECT_EQ(m.board.unhandled_records, 0u);
}

TEST(ControlBoardSource, RecordWithNoDecoderIsSkippedAndCounted) {
    ControlBoardSource board;
    ASSERT_TRUE(board.buildTable());

    SensorMap m;
    board.decode(answer({other(3), encoder(1, 9)}), LinkStats{}, m);
    EXPECT_EQ(m.encoder_counts[1], 9); // the rest still decode
    EXPECT_EQ(m.board.unhandled_records, 1u);
    EXPECT_EQ(m.board.last_unhandled_type, kOtherType);
    EXPECT_EQ(m.board.last_unhandled_port, 3);
}

TEST(ControlBoardSource, NoDecodersMeansNothingIsRead) {
    ControlBoardSource board(Decoders{});
    ASSERT_TRUE(board.buildTable());
    SensorMap m;
    board.decode(answer({encoder(0, 5)}), LinkStats{}, m);
    EXPECT_FALSE(m.encoder_counts[0].has_value());
    EXPECT_EQ(m.board.unhandled_records, 1u);
}

TEST(ControlBoardSource, TwoDecodersForOneTypeFailBegin) {
    EncoderCountDecoder a;
    EncoderCountDecoder b;
    ControlBoardSource  board(Decoders{&a, &b});
    EXPECT_FALSE(board.begin(onPort(16)));
}

TEST(ControlBoardSource, NoPortFailsBegin) {
    ControlBoardSource board;
    EXPECT_FALSE(board.begin(onPort(0)));
    EXPECT_FALSE(board.begin(onPort(22)));
}

TEST(ControlBoardSource, PortThatFailsToOpenIsRetried) {
    g_open_failures = 2;
    g_opens         = 0;
    ControlBoardSource board;
    ASSERT_TRUE(board.begin(onPort(16))); // runs; the port is retried from read()
    EXPECT_EQ(g_opens, 1);

    SensorMap m;
    board.read(m); // second try, fails
    EXPECT_FALSE(m.board.port_open);
    EXPECT_FALSE(m.board.fresh);
    EXPECT_EQ(g_opens, 2);

    for (uint32_t i = 0; i < ControlBoardSource::kRetryReads; ++i) {
        m = SensorMap{};
        board.read(m); // waiting
        EXPECT_FALSE(m.board.port_open);
    }
    EXPECT_EQ(g_opens, 2);

    m = SensorMap{};
    board.read(m); // third try opens and reads
    EXPECT_EQ(g_opens, 3);
    EXPECT_TRUE(m.board.port_open);
    EXPECT_FALSE(m.board.fresh); // nothing has arrived
}

TEST(ControlBoardSource, NoNewFrameLeavesCountsEmpty) {
    ControlBoardSource board;
    ASSERT_TRUE(board.buildTable());
    SensorMap m;
    LinkStats stats;
    stats.frames  = 9;
    stats.dropped = 4;
    board.decode(Poll{}, stats, m);
    EXPECT_FALSE(m.encoder_counts[0].has_value());
    EXPECT_FALSE(m.board.fresh);
    EXPECT_EQ(m.board.frames, 9u);
    EXPECT_EQ(m.board.dropped, 4u);
}

TEST(ControlBoardSource, EmptyRecordGivesNoCount) {
    ControlBoardSource board;
    ASSERT_TRUE(board.buildTable());
    brainlink::Record empty = encoder(0, 0);
    empty.size              = 0;
    SensorMap m;
    board.decode(answer({empty}), LinkStats{}, m);
    EXPECT_FALSE(m.encoder_counts[0].has_value());
}

} // namespace
} // namespace locagatr
