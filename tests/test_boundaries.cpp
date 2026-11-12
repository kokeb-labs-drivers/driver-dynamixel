/**
 * @file test_boundaries.cpp
 * @brief The edges of every check: the exact limits of arguments and buffers, which timeout each
 * read gets, when the driver waits, which request goes on the line, and which per-device outcome a
 * damaged fast reply gives.
 *
 * These tests were written from the survivors of a mutation run over the driver (docs/testing.md,
 * "Mutation testing"): each pins a limit one operator away from the right one.
 */
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "boundary_support.hpp"

using namespace dynamixel;
using namespace rig;
using namespace bsupport;
namespace p = dynamixel::protocol;

// ------------------------------------------------------------------------------------ frame writer

TEST_CASE("FrameWriter 2.0: a buffer that holds the CRC exactly is enough, one byte less is not") {
  std::array<uint8_t, 10> exact{};
  p::FrameWriter fits{std::span<uint8_t>{exact}, Protocol::V2, 1, p::Instruction::Ping};
  CHECK_EQ(fits.Finish(), 10U);
  std::array<uint8_t, 9> small{};
  p::FrameWriter tight{std::span<uint8_t>{small}, Protocol::V2, 1, p::Instruction::Ping};
  CHECK_EQ(tight.Finish(), 0U);
}

TEST_CASE("FrameWriter 2.0: the length field holds both bytes of a long packet") {
  std::array<uint8_t, 400> buf{};
  p::FrameWriter w{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Write};
  std::array<uint8_t, 300> data{};
  w.PutBytes(S(data));
  const std::size_t n = w.Finish();
  REQUIRE(n == 300U + 10U);
  CHECK_EQ(buf[5], static_cast<uint8_t>((300U + 3U) & 0xFFU));
  CHECK_EQ(buf[6], static_cast<uint8_t>((300U + 3U) >> 8U));
}

TEST_CASE("FrameWriter 1.0: 253 parameters fit, 254 do not") {
  std::array<uint8_t, 300> buf{};
  std::array<uint8_t, 254> data{};
  p::FrameWriter a{std::span<uint8_t>{buf}, Protocol::V1, 1, p::Instruction::Write};
  a.PutBytes(std::span<const uint8_t>{data.data(), 253});
  CHECK_EQ(a.Finish(), 253U + 6U);
  p::FrameWriter b{std::span<uint8_t>{buf}, Protocol::V1, 1, p::Instruction::Write};
  b.PutBytes(S(data));
  CHECK_EQ(b.Finish(), 0U);
}

TEST_CASE("FrameWriter 2.0 stuffs FD only after two FF and counts the stuffed byte in the length") {
  std::array<uint8_t, 40> buf{};
  p::FrameWriter w{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Write};
  const std::array<uint8_t, 7> data{0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0xFF, 0xFD};
  w.PutBytes(S(data));
  const std::size_t n = w.Finish();
  // FF FD (no stuffing), FF FF FD FD (one stuffed), FF FD (none): one byte added.
  CHECK_EQ(n, 7U + 1U + 10U);
  CHECK_EQ(buf[5], static_cast<uint8_t>(8U + 3U));
}

// ------------------------------------------------------------------------------------- the scanner

TEST_CASE("Scan 2.0: a header byte that is wrong in the second place is garbage, not a candidate") {
  std::vector<uint8_t> f = StatusV2(1, 0, {0x01});
  f[1] = 0x00; // FF 00 FD 00: not a header; the CRC is recomputed so only the header tells
  const uint16_t crc = p::Crc16(0, std::span<const uint8_t>{f.data(), f.size() - 2U});
  f[f.size() - 2U] = static_cast<uint8_t>(crc & 0xFFU);
  f[f.size() - 1U] = static_cast<uint8_t>(crc >> 8U);
  const p::ScanResult r = p::Scan(Protocol::V2, std::span<const uint8_t>{f}, 64);
  CHECK(r.kind != p::ScanResult::Kind::Frame);
}

TEST_CASE(
    "Scan 2.0: consumed garbage asks for one more byte, a bare header for the three that follow") {
  const std::array<uint8_t, 2> junk{0x01, 0x02};
  const p::ScanResult a = p::Scan(Protocol::V2, S(junk), 64);
  CHECK_EQ(a.discard, 2U);
  CHECK_EQ(a.need, 1U);
  CHECK_EQ(a.garbage, 2U);
  const std::array<uint8_t, 4> header{0xFF, 0xFF, 0xFD, 0x00};
  const p::ScanResult b = p::Scan(Protocol::V2, S(header), 64);
  CHECK_EQ(b.discard, 0U);
  CHECK_EQ(b.need, 3U);
  const std::array<uint8_t, 3> partial{0xFF, 0xFF, 0xFD};
  const p::ScanResult c = p::Scan(Protocol::V2, S(partial), 64);
  CHECK_EQ(c.need, 1U);
  CHECK_EQ(c.garbage, 0U);
}

TEST_CASE("Scan 1.0: FF FF FF is a longer run of header bytes, FF FF alone may still become one") {
  const std::array<uint8_t, 3> run{0xFF, 0xFF, 0xFF};
  const p::ScanResult a = p::Scan(Protocol::V1, S(run), 64);
  CHECK_EQ(a.garbage, 1U);
  CHECK_EQ(a.need, 2U);
  const std::array<uint8_t, 2> two{0xFF, 0xFF};
  const p::ScanResult b = p::Scan(Protocol::V1, S(two), 64);
  CHECK_EQ(b.garbage, 0U);
  CHECK_EQ(b.need, 2U);
}

TEST_CASE("Scan 2.0: a packet exactly as long as the buffer is accepted, a longer one is not") {
  const std::vector<uint8_t> f = StatusV2(1, 0, {0x11, 0x22, 0x33});
  const p::ScanResult fits = p::Scan(Protocol::V2, std::span<const uint8_t>{f}, f.size());
  CHECK(fits.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(fits.frame_size, f.size());
  const p::ScanResult over = p::Scan(Protocol::V2, std::span<const uint8_t>{f}, f.size() - 1U);
  CHECK(over.kind != p::ScanResult::Kind::Frame);
}

TEST_CASE("Scan 2.0: a length below 4 is not a status packet even with a valid CRC") {
  // Length 3 holds the instruction and the CRC only: no error byte, so it cannot be a status
  // packet.
  std::vector<uint8_t> f{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x55};
  const uint16_t crc = p::Crc16(0, std::span<const uint8_t>{f});
  f.push_back(static_cast<uint8_t>(crc & 0xFFU));
  f.push_back(static_cast<uint8_t>(crc >> 8U));
  REQUIRE(f.size() == 10U); // 7 + length: the whole packet is there and its CRC is right
  const p::ScanResult r = p::Scan(Protocol::V2, std::span<const uint8_t>{f}, 64);
  CHECK(r.kind != p::ScanResult::Kind::Frame);
  CHECK(r.garbage >= 1U);
}

TEST_CASE("Scan 2.0: eight bytes with a wrong instruction are refused at once, not waited for") {
  const std::array<uint8_t, 8> inst{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x20, 0x00, 0x02};
  const p::ScanResult r = p::Scan(Protocol::V2, S(inst), 64);
  CHECK(r.garbage >= 1U);
  CHECK(r.kind != p::ScanResult::Kind::Frame);
}

// ---------------------------------------------------------------------------------- codecs, tables

TEST_CASE("LoadLe accepts 1 to 4 bytes and returns 0 for none or more") {
  const std::array<uint8_t, 5> five{1, 2, 3, 4, 5};
  CHECK_EQ(p::LoadLe(std::span<const uint8_t>{five.data(), 4}), 0x04030201U);
  CHECK_EQ(p::LoadLe(S(five)), 0U);
  CHECK_EQ(p::LoadLe(std::span<const uint8_t>{}), 0U);
  CHECK_EQ(p::LoadLe(std::span<const uint8_t>{five.data(), 1}), 1U);
}

TEST_CASE("EncodeValue: the exact edges of every kind") {
  std::array<uint8_t, 4> out{};
  using dynamixel::Kind;
  CHECK(p::EncodeValue(Kind::Signed, 1, 127, out));
  CHECK(!p::EncodeValue(Kind::Signed, 1, 128, out));
  CHECK(p::EncodeValue(Kind::Signed, 1, -128, out));
  CHECK(!p::EncodeValue(Kind::Signed, 1, -129, out));
  CHECK(p::EncodeValue(Kind::Unsigned, 1, 255, out));
  CHECK(!p::EncodeValue(Kind::Unsigned, 1, 256, out));
  CHECK(!p::EncodeValue(Kind::Unsigned, 1, -1, out));
  out = {0xAA, 0xAA, 0xAA, 0xAA};
  CHECK(p::EncodeValue(Kind::SignMag10, 2, 0, out)); // zero is plain zero, never "negative zero"
  CHECK_EQ(out[0], 0U);
  CHECK_EQ(out[1], 0U);
  CHECK(p::EncodeValue(Kind::SignMag10, 2, -1, out));
  CHECK_EQ(out[0], 1U);
  CHECK_EQ(out[1], 4U);
  CHECK(p::EncodeValue(Kind::SignMag10, 2, 1023, out));
  CHECK(!p::EncodeValue(Kind::SignMag10, 2, 1024, out));
  CHECK(!p::EncodeValue(Kind::SignMag10, 2, -1024, out));
  CHECK(p::EncodeValue(Kind::Offset2048, 2, -2048, out)); // raw 0
  CHECK_EQ(out[0], 0U);
  CHECK(!p::EncodeValue(Kind::Offset2048, 2, -2049, out));
  CHECK(p::EncodeValue(Kind::Offset2048, 2, 2047, out));
  CHECK(!p::EncodeValue(Kind::Offset2048, 2, 2048 + 0xF000, out));
  CHECK(!p::EncodeValue(Kind::Signed, 3, 0, out)); // a register is 1, 2 or 4 bytes
}

TEST_CASE("Families that speak both protocols say so, and a value that is no family is Generic") {
  for (const Family f : {Family::X330, Family::X430Basic, Family::X430, Family::X540, Family::Xw}) {
    CHECK_EQ(Info(f).protocols, static_cast<uint8_t>(kProtocolBit1 | kProtocolBit2));
  }
  CHECK_EQ(Info(Family::AxRxDx).protocols, kProtocolBit1);
  CHECK_EQ(Info(Family::Xl320).protocols, kProtocolBit2);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(Info(static_cast<Family>(13)).position_counts, Info(Family::Generic).position_counts);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(Info(static_cast<Family>(200)).protocols, Info(Family::Generic).protocols);
}

TEST_CASE("Indirect pairs: first and last of each block, none outside") {
  CHECK_EQ(Indirect(Family::X430, 0).address_register, 0U);
  CHECK_EQ(Indirect(Family::X430, 1).address_register, 168U);
  CHECK_EQ(Indirect(Family::X430, 1).data_register, 224U);
  CHECK_EQ(Indirect(Family::X430, 28).address_register, 168U + 54U);
  CHECK_EQ(Indirect(Family::X430, 28).data_register, 224U + 27U);
  CHECK_EQ(Indirect(Family::X430, 29).address_register, 578U);
  CHECK_EQ(Indirect(Family::X430, 29).data_register, 634U);
  CHECK_EQ(Indirect(Family::X430, 56).data_register, 634U + 27U);
  CHECK_EQ(Indirect(Family::X430, 57).address_register, 0U);
  CHECK_EQ(Indirect(Family::X330, 28).address_register, 168U + 54U);
  CHECK_EQ(Indirect(Family::X330, 29).address_register, 0U);
  CHECK_EQ(Indirect(Family::AxRxDx, 1).address_register, 0U);
}

TEST_CASE("Baud register values: the last valid value of each table, none past it") {
  uint8_t value = 0;
  CHECK(BaudValueFor(Family::Xl320, 1'000'000U, value));
  CHECK_EQ(value, 3U);
  CHECK(!BaudValueFor(Family::Xl320, 2'000'000U, value));
  CHECK(BaudValueFor(Family::X430, 4'500'000U, value));
  CHECK_EQ(value, 7U);
  CHECK(BaudValueFor(Family::X330, 4'000'000U, value));
  CHECK_EQ(value, 6U);
  CHECK(!BaudValueFor(Family::X330, 4'500'000U, value));
  CHECK(BaudValueFor(Family::AxRxDx, 1'000'000U, value));
  CHECK_EQ(value, 1U);
}

TEST_CASE(
    "Protocol 2.0 error numbers: 7 is the last one defined, 8 and above are a failed result") {
  CHECK(ErrorNumber(0x07) == Protocol2Error::AccessError);
  CHECK(ErrorNumber(0x87) == Protocol2Error::AccessError);
  CHECK(ErrorNumber(0x08) == Protocol2Error::ResultFail);
  CHECK(ErrorNumber(0x00) == Protocol2Error::None);
}

TEST_CASE("Validate accepts the largest timeouts and rejects the next microsecond") {
  Config c{};
  c.reply_timeout_us = kMaxTimeoutUs;
  CHECK_EQ(Validate(c), Error::Ok);
  c.reply_timeout_us = kMaxTimeoutUs + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.broadcast_timeout_us = kMaxTimeoutUs;
  CHECK_EQ(Validate(c), Error::Ok);
  c.broadcast_timeout_us = kMaxTimeoutUs + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.reply_timeout_us = kMinTimeoutUs;
  CHECK_EQ(Validate(c), Error::Ok);
  c.reply_timeout_us = kMinTimeoutUs - 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
}

TEST_CASE("A DeviceReply starts as a timeout") {
  const DeviceReply d{};
  CHECK(d.result == Error::Timeout);
  CHECK(!d.alert);
}

// -------------------------------------------------------------------------------- argument limits

TEST_CASE("Protocol 1.0: address 255 is valid and 256 is refused, in every call that takes one") {
  V1Rig r;
  std::array<uint8_t, 1> one{};
  int before = r.bus.writes;
  CHECK(r.drv.Read(1, 255, std::span<uint8_t>{one}).error != Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, before + 1);
  CHECK_EQ(r.drv.Read(1, 256, std::span<uint8_t>{one}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Write(1, 256, S(one)).error, Error::InvalidArgument);
  CHECK(r.drv.Write(1, 255, S(one)).error != Error::InvalidArgument);
  const std::array<uint8_t, 1> ids{1};
  CHECK(r.drv.SyncWrite(255, 1, S(ids), S(one)).error != Error::InvalidArgument);
  CHECK_EQ(r.drv.SyncWrite(256, 1, S(ids), S(one)).error, Error::InvalidArgument);
  std::array<uint8_t, 251> big{};
  CHECK_EQ(r.drv.SyncWrite(24, 251, S(ids), S(big)).error, Error::InvalidArgument);
  const std::array<BulkReadItem, 1> at255{{{1, 255, 1}}};
  std::array<DeviceReply, 1> rep{};
  CHECK(r.drv
            .BulkRead(std::span<const BulkReadItem>{at255}, std::span<uint8_t>{one},
                      std::span<DeviceReply>{rep})
            .error != Error::InvalidArgument);
  const std::array<BulkReadItem, 1> at256{{{1, 256, 1}}};
  CHECK_EQ(r.drv
               .BulkRead(std::span<const BulkReadItem>{at256}, std::span<uint8_t>{one},
                         std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("A reply that exactly fills the receive buffer is allowed, one byte more is refused") {
  mock::Bus bus{};
  mock::Port port{bus};
  bus.v2 = true;
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  static_cast<void>(bus.Add(mock::Kind::Xm430, 2));
  DYNAMIXEL<mock::Port, 64> drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  std::array<uint8_t, 40> out{};
  // 39 bytes: stuffing bound 39 + 13 + 1 plus 11 bytes of packet = 64.
  CHECK(drv.Read(1, 0, std::span<uint8_t>{out.data(), 39}).error != Error::InvalidArgument);
  CHECK_EQ(drv.Read(1, 0, std::span<uint8_t>{out.data(), 40}).error, Error::InvalidArgument);
  const std::array<uint8_t, 2> ids{1, 2};
  std::array<uint8_t, 80> data{};
  std::array<DeviceReply, 2> rep{};
  CHECK(
      drv.SyncRead(0, 39, S(ids), std::span<uint8_t>{data.data(), 78}, std::span<DeviceReply>{rep})
          .error != Error::InvalidArgument);
  CHECK_EQ(
      drv.SyncRead(0, 40, S(ids), std::span<uint8_t>{data.data(), 80}, std::span<DeviceReply>{rep})
          .error,
      Error::InvalidArgument);
  // Fast: 8 + total + 4 per device = 64 at 24 bytes each, 66 at 25.
  CHECK(drv.FastSyncRead(0, 24, S(ids), std::span<uint8_t>{data.data(), 48},
                         std::span<DeviceReply>{rep})
            .error != Error::InvalidArgument);
  CHECK_EQ(drv.FastSyncRead(0, 25, S(ids), std::span<uint8_t>{data.data(), 50},
                            std::span<DeviceReply>{rep})
               .error,
           Error::InvalidArgument);
}

TEST_CASE("ChangeBusBaud accepts the two limits and refuses the values just outside") {
  V2Rig r;
  CHECK(r.drv.ChangeBusBaud(kMinBaud).ok());
  CHECK_EQ(r.bus.baud, kMinBaud);
  CHECK(r.drv.ChangeBusBaud(kMaxBaud).ok());
  CHECK_EQ(r.bus.baud, kMaxBaud);
  CHECK_EQ(r.drv.ChangeBusBaud(kMinBaud - 1U).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.ChangeBusBaud(kMaxBaud + 1U).error, Error::InvalidArgument);
}

TEST_CASE("ScanIds: one ID is a valid range, the last ID is included, a full span stops the scan") {
  V2Rig r;
  std::array<PingInfo, 8> found{};
  const auto one = r.drv.ScanIds(2, 2, std::span<PingInfo>{found});
  REQUIRE(one.ok());
  CHECK_EQ(one.value, 1U);
  CHECK_EQ(found[0].id, 2U);
  const auto all = r.drv.ScanIds(1, 3, std::span<PingInfo>{found});
  REQUIRE(all.ok());
  CHECK_EQ(all.value, 3U);
  CHECK_EQ(found[2].id, 3U);
  std::array<PingInfo, 2> two{};
  const int before = r.bus.writes;
  const auto cut = r.drv.ScanIds(1, 3, std::span<PingInfo>{two});
  REQUIRE(cut.ok());
  CHECK_EQ(cut.value, 2U);
  CHECK_EQ(r.bus.writes, before + 2); // the third ID is never pinged
  CHECK_EQ(r.drv.ScanIds(3, 2, std::span<PingInfo>{found}).error, Error::InvalidArgument);
}

TEST_CASE("Broadcast ping counts the highest valid ID and each ID once") {
  V2Rig r;
  static_cast<void>(r.bus.Add(mock::Kind::Xm430, 252));
  std::array<PingInfo, 8> found{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 4U);
  CHECK_EQ(found[3].id, 252U);
  V2Rig d; // two servos with the same ID answer in turn: the second is not another device, and ends
           // the list
  static_cast<void>(d.bus.Add(mock::Kind::Xm430, 1));
  std::array<PingInfo, 8> again{};
  const auto m = d.drv.BroadcastPing(std::span<PingInfo>{again});
  REQUIRE(m.ok());
  CHECK_EQ(m.value, 1U);
}

TEST_CASE("Identify on protocol 2.0 is one transfer; a failed ping is its own result") {
  V2Rig r;
  const int before = r.bus.writes;
  CHECK(r.drv.Identify(1).ok());
  CHECK_EQ(r.bus.writes, before + 1);
  r.bus.Find(2)->silent = true;
  CHECK_EQ(r.drv.Identify(2).error, Error::Timeout);
}

// ---------------------------------------------------------------------------- what is waited, sent

TEST_CASE("The first read of a reply waits the reply timeout, the next ones the gap timeout") {
  mock::Bus bus{};
  bus.v2 = true;
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  ProbePort port{bus};
  Config c{};
  c.reply_timeout_us = 7'000;
  c.gap_timeout_us = 900;
  DYNAMIXEL<ProbePort> drv{port, c};
  REQUIRE(drv.EnsureInitialized().ok());
  bus.knob.split_chunk = 3;
  port.reads.clear();
  CHECK(drv.Ping(1).ok());
  REQUIRE(port.reads.size() >= 3U);
  CHECK_EQ(port.reads[0].timeout_us, 7'000U);
  CHECK_EQ(port.reads[1].timeout_us, 900U);
  CHECK_EQ(port.reads[2].timeout_us, 900U);
}

TEST_CASE("The echo is read into the buffer one piece after the other, with the gap timeout") {
  mock::Bus bus{};
  bus.v2 = true;
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  ProbePort port{bus};
  Config c{};
  c.echo = EchoMode::Loopback;
  c.gap_timeout_us = 900;
  DYNAMIXEL<ProbePort> drv{port, c};
  REQUIRE(drv.EnsureInitialized().ok());
  bus.knob.echo = true;
  bus.knob.split_chunk = 4;
  port.reads.clear();
  CHECK(drv.Ping(1).ok());
  REQUIRE(port.reads.size() >= 3U);
  CHECK_EQ(port.reads[0].timeout_us, 900U);
  CHECK(port.reads[1].data == port.reads[0].data + 4);
  CHECK(port.reads[2].data == port.reads[0].data + 8);
  CHECK_EQ(port.reads[0].max, 10U);
  CHECK_EQ(port.reads[1].max, 6U);
}

TEST_CASE("Noise that fills the receive buffer is dropped and the real reply is still found") {
  mock::Bus bus{};
  bus.v2 = true;
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  mock::Port port{bus};
  DYNAMIXEL<mock::Port, 64> drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  bus.knob.garbage_prefix = 64; // the buffer is full of junk before the reply starts
  const auto r = drv.Ping(1);
  CHECK(r.ok());
  CHECK(drv.GetStats().garbage_bytes >= 1U);
}

TEST_CASE("A line stuck low or high is a line fault from four bytes on, a timeout below") {
  V2Rig r;
  r.bus.knob.stuck = 1;
  r.bus.knob.stuck_len = 4;
  CHECK_EQ(r.drv.Ping(1).error, Error::LineFault);
  r.bus.knob.stuck = 2;
  CHECK_EQ(r.drv.Ping(1).error, Error::LineFault);
  r.bus.knob.stuck_len = 3;
  CHECK_EQ(r.drv.Ping(1).error, Error::Timeout);
}

TEST_CASE("Resync gives up after 16 rounds, and a full read is data, not a broken port") {
  mock::Bus bus{};
  bus.v2 = true;
  ProbePort port{bus};
  DYNAMIXEL<ProbePort, 64> drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  port.endless = true;
  const int before = bus.flushes;
  const auto r = drv.Resync();
  CHECK_EQ(r.error, Error::LineFault); // not Bus: the port returned exactly the room it was given
  CHECK_EQ(bus.flushes - before, 16);
  CHECK_EQ(drv.GetStats().line_faults, 1U);
}

TEST_CASE("A reply that never ends is refused after a bounded number of reads") {
  mock::Bus bus{};
  bus.v2 = true;
  ProbePort port{bus};
  DYNAMIXEL<ProbePort, 64> drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  port.endless = true;
  port.reads.clear();
  const auto r = drv.Ping(1);
  CHECK(r.error == Error::InvalidReply || r.error == Error::LineFault);
  CHECK(port.reads.size() <= 64U);
}

TEST_CASE("Only reboot, factory reset and a restore wait for the servo; nothing else does") {
  V2Rig r;
  Config c = r.drv.GetConfig();
  c.allow_destructive = true;
  c.reboot_wait_us = 123'456;
  REQUIRE(r.drv.ApplyConfig(c).ok());
  const std::array<uint8_t, 1> one{1};
  auto delay_of = [&](auto&& call) {
    const uint32_t before = r.bus.delay_us;
    static_cast<void>(call());
    return r.bus.delay_us - before;
  };
  CHECK_EQ(delay_of([&] { return r.drv.Reboot(1); }), 123'456U);
  CHECK_EQ(delay_of([&] { return r.drv.FactoryReset(1, ResetOption::KeepId); }), 123'456U);
  CHECK_EQ(delay_of([&] { return r.drv.Backup(1, BackupOption::Restore); }), 123'456U);
  CHECK_EQ(delay_of([&] { return r.drv.RegWrite(1, 65, S(one)); }), 0U);
  CHECK_EQ(delay_of([&] { return r.drv.Action(1); }), 0U);
  CHECK_EQ(delay_of([&] { return r.drv.Clear(1, ClearOption::MultiTurn); }), 0U);
  CHECK_EQ(delay_of([&] { return r.drv.Write(1, 65, S(one)); }), 0U);
  CHECK_EQ(delay_of([&] { return r.drv.Backup(1, BackupOption::Store); }), 0U);
}

TEST_CASE("Writes are sent once whatever happens; reads are repeated") {
  V2Rig r;
  Config c = r.drv.GetConfig();
  c.retries = 3;
  c.allow_destructive = true;
  REQUIRE(r.drv.ApplyConfig(c).ok());
  const std::array<uint8_t, 1> one{1};
  auto sent = [&](auto&& call) {
    r.bus.knob.drop_replies = 100;
    const int before = r.bus.writes;
    static_cast<void>(call());
    r.bus.knob.drop_replies = 0;
    return r.bus.writes - before;
  };
  CHECK_EQ(sent([&] { return r.drv.Write(1, 65, S(one)); }), 1);
  CHECK_EQ(sent([&] { return r.drv.RegWrite(1, 65, S(one)); }), 1);
  CHECK_EQ(sent([&] { return r.drv.Action(1); }), 1);
  CHECK_EQ(sent([&] { return r.drv.Reboot(1); }), 1);
  CHECK_EQ(sent([&] { return r.drv.Clear(1, ClearOption::Errors); }), 1);
  CHECK_EQ(sent([&] { return r.drv.FactoryReset(1, ResetOption::KeepId); }), 1);
  std::array<uint8_t, 1> in{};
  CHECK_EQ(sent([&] { return r.drv.Read(1, 64, std::span<uint8_t>{in}); }), 4);
  CHECK_EQ(sent([&] { return r.drv.Ping(1); }), 4);
}

TEST_CASE(
    "A status packet with a parameter too many is not a valid answer to a write or an action") {
  V2Rig r;
  const std::array<uint8_t, 1> one{1};
  r.bus.knob.extra_param = 1;
  CHECK_EQ(r.drv.Write(1, 65, S(one)).error, Error::InvalidReply);
  r.bus.knob.extra_param = 1;
  CHECK_EQ(r.drv.RegWrite(1, 65, S(one)).error, Error::InvalidReply);
  r.bus.knob.extra_param = 1;
  CHECK_EQ(r.drv.Reboot(1).error, Error::InvalidReply);
  r.bus.knob.extra_param = 1;
  std::array<PingInfo, 4> found{};
  const auto b = r.drv.BroadcastPing(std::span<PingInfo>{found});
  CHECK_EQ(b.error, Error::InvalidReply); // the first answer is refused and no other was collected
}

TEST_CASE("Torque off: a damaged confirmation is asked for again; an extra parameter is damage") {
  V2Rig r;
  r.bus.knob.extra_param = 1;
  const int before = r.bus.writes;
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.bus.writes - before, 2);
}

TEST_CASE("Torque off for a list reports the first failure, after trying all of them") {
  V2Rig r;
  r.bus.Find(1)->silent = true;
  const std::array<ServoRef, 3> list{{kX1, ServoRef{2, Family::Generic}, kX3}};
  const auto done = r.drv.DisableTorqueEach(std::span<const ServoRef>{list});
  CHECK_EQ(done.error, Error::Timeout);
  CHECK(!r.bus.Find(3)->TorqueOn());
}

TEST_CASE("The request on the line carries the instruction the call is named for") {
  V2Rig r;
  const std::array<uint8_t, 2> ids{1, 2};
  std::array<uint8_t, 2> out{};
  std::array<DeviceReply, 2> rep{};
  const std::array<BulkReadItem, 2> items{{{1, 64, 1}, {2, 64, 1}}};
  auto instruction = [&] { return r.bus.last_tx[7]; };
  static_cast<void>(
      r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep}));
  CHECK_EQ(instruction(), 0x82U);
  static_cast<void>(
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep}));
  CHECK_EQ(instruction(), 0x8AU);
  static_cast<void>(r.drv.BulkRead(std::span<const BulkReadItem>{items}, std::span<uint8_t>{out},
                                   std::span<DeviceReply>{rep}));
  CHECK_EQ(instruction(), 0x92U);
  static_cast<void>(r.drv.FastBulkRead(std::span<const BulkReadItem>{items},
                                       std::span<uint8_t>{out}, std::span<DeviceReply>{rep}));
  CHECK_EQ(instruction(), 0x9AU);
}

TEST_CASE("Sync write and bulk write return the port's failure") {
  V2Rig r;
  const std::array<uint8_t, 1> ids{1};
  const std::array<uint8_t, 1> data{1};
  const std::array<BulkWriteItem, 1> items{{{1, 65, std::span<const uint8_t>{data}}}};
  r.bus.fail_write = true;
  CHECK_EQ(r.drv.SyncWrite(65, 1, S(ids), S(data)).error, Error::Bus);
  CHECK_EQ(r.drv.BulkWrite(std::span<const BulkWriteItem>{items}).error, Error::Bus);
}

// -------------------------------------------------------------- per-device outcomes of the reads

TEST_CASE(
    "Sync read: a reply that belongs to no listed device is a mismatch, not a silent device") {
  V2Rig r;
  const std::array<uint8_t, 2> ids{1, 3};
  std::array<uint8_t, 2> out{};
  std::array<DeviceReply, 2> rep{};
  r.bus.knob.wrong_id = 1; // servo 1 answers as ID 2, which is not in the list
  const auto done =
      r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::DeviceIdMismatch);
  CHECK_EQ(rep[0].result, Error::DeviceIdMismatch);
  CHECK_EQ(rep[1].result, Error::Ok);
}

TEST_CASE(
    "Sync read: a part with a parameter too many is invalid; the first failure is the result") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  r.bus.knob.extra_param = 1;
  auto done = r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::InvalidReply);
  CHECK_EQ(rep[0].result, Error::InvalidReply);
  CHECK_EQ(rep[1].result, Error::Ok);
  r.bus.Find(1)->silent = true;
  r.bus.Find(2)->force_error = 0x07;
  done = r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Timeout); // servo 1 is first, whatever servo 2 did
  CHECK_EQ(rep[0].result, Error::Timeout);
  CHECK_EQ(rep[1].result, Error::DeviceError);
  CHECK_EQ(rep[1].error_byte, 0x07U);
  CHECK_EQ(rep[2].result, Error::Ok);
}

TEST_CASE("Sync read: when the port fails, every device after it carries the same failure") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  r.bus.knob.oversize_read = 1;
  const auto done =
      r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Bus);
  for (std::size_t k = 0; k < rep.size(); ++k) {
    CHECK_EQ(rep[k].result, Error::Bus);
    CHECK_EQ(rep[k].id, ids[k]);
  }
}

TEST_CASE(
    "Fast sync read: a packet cut inside the second part keeps the first and times out the rest") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  r.bus.Find(1)->mem[64] = 0x31;
  r.bus.Find(2)->alert = true;
  // header 8, part 1 = 1 + 1 + 1 + 2 = 5 bytes, part 2 starts at 13: cut after its ID.
  r.bus.knob.truncate_to = 8 + 5 + 2;
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Timeout);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(out[0], 0x31U);
  CHECK_EQ(rep[1].result, Error::Timeout);
  CHECK_EQ(rep[2].result, Error::Timeout);
  CHECK(!rep[1].alert);
  CHECK(!rep[2].alert);
  CHECK_EQ(rep[1].error_byte, 0U);
  // Cut before the ID of the first part: nothing is delivered, nothing is claimed.
  r.bus.knob.truncate_to = 8 + 1;
  const auto none =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(none.error, Error::Timeout);
  CHECK_EQ(rep[0].result, Error::Timeout);
  CHECK(!rep[0].alert);
  // Cut inside the last part's check field.
  r.bus.knob.truncate_to = 8 + 5 + 5 + 4;
  const auto last =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(last.error, Error::Timeout);
  CHECK_EQ(rep[1].result, Error::Ok);
  CHECK_EQ(rep[2].result, Error::Timeout);
}

TEST_CASE(
    "Fast sync read: a damaged check field at the end of the packet condemns the whole packet") {
  V2Rig r;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  r.bus.knob.corrupt_crc = 1; // the last byte of the packet: the third part's check field
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Crc); // the whole packet is damaged: no part of it is trusted
  for (const DeviceReply& d : rep) {
    CHECK_EQ(d.result, Error::Crc);
  }
  CHECK_EQ(r.drv.GetStats().crc_errors >= 1U, true);
}

TEST_CASE("Fast sync read: a silent device in the middle is a timeout, the others are read") {
  V2Rig r;
  r.bus.Find(2)->silent = true;
  r.bus.Find(1)->mem[64] = 0x11;
  r.bus.Find(3)->mem[64] = 0x33;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Timeout);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(rep[1].result, Error::Timeout);
  CHECK_EQ(rep[2].result, Error::Ok);
  CHECK_EQ(out[0], 0x11U);
  CHECK_EQ(out[2], 0x33U);
}

TEST_CASE(
    "Fast sync read: a part from a device that is not listed is a mismatch and stops the packet") {
  V2Rig r;
  r.bus.Find(2)->id = 9; // the model answers as ID 9 while the host asked for ID 2
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  // Ask the model for the ID it has, so that it is in the packet, but list another one.
  const std::array<uint8_t, 3> asked{1, 9, 3};
  static_cast<void>(ids);
  r.bus.Find(9)->mem[7] = 9;
  const auto done =
      r.drv.FastSyncRead(64, 1, S(asked), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(done.error == Error::Ok || done.error == Error::DeviceIdMismatch);
  r.bus.Find(1)->id = 8; // now the first part carries 8 for a requested 1
  const std::array<uint8_t, 3> wrong{1, 9, 3};
  const auto bad =
      r.drv.FastSyncRead(64, 1, S(wrong), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(bad.error != Error::Ok);
}

TEST_CASE("Fast sync read: a rejected part and an alert part are reported per device") {
  V2Rig r;
  r.bus.Find(1)->alert = true;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Ok);
  CHECK(rep[0].alert);
  CHECK_EQ(rep[0].error_byte, 0x80U);
  CHECK(!rep[1].alert);
  // An address the servo does not have: the part carries error 7 and no data.
  done = r.drv.FastSyncRead(1100, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::DeviceError);
  CHECK_EQ(rep[0].result, Error::DeviceError);
  CHECK_EQ(rep[1].result, Error::DeviceError);
  // With the alert policy that fails, an alert part is a device fault, and carries no data.
  Config c = r.drv.GetConfig();
  c.alert_policy = AlertPolicy::Fail;
  REQUIRE(r.drv.ApplyConfig(c).ok());
  r.bus.Find(1)->mem[64] = 0x77;
  out = {0xEE, 0xEE, 0xEE};
  done = r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::DeviceFault);
  CHECK_EQ(rep[0].result, Error::DeviceFault);
  CHECK_EQ(out[0], 0xEEU);
  CHECK_EQ(rep[1].result, Error::Ok);
}

// Replies built by hand, so that a single part can be wrong while the packet as a whole is valid.

TEST_CASE("Fast sync read by hand: the check field of one part is that part's alone") {
  HandRig r;
  r.port.inject =
      FastPacket({{0x00, 1, {0x11}, false}, {0x00, 2, {0x22}, true}, {0x00, 3, {0x33}, false}});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Crc);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(out[0], 0x11U);
  CHECK_EQ(rep[1].result, Error::Crc);
  CHECK_EQ(rep[2].result, Error::Crc); // after a damaged part the rest of the packet is not trusted
  CHECK_EQ(r.drv.GetStats().crc_errors, 1U);
}

TEST_CASE("Fast sync read by hand: a part with another ID is a mismatch and stops the packet") {
  HandRig r;
  r.port.inject =
      FastPacket({{0x00, 1, {0x11}, false}, {0x00, 9, {0x22}, false}, {0x00, 3, {0x33}, false}});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::DeviceIdMismatch);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(rep[1].result, Error::DeviceIdMismatch);
  CHECK_EQ(rep[2].result, Error::DeviceIdMismatch);
}

TEST_CASE("Fast sync read by hand: a device that is skipped keeps its timeout, the next part is "
          "its owner's") {
  HandRig r;
  r.port.inject = FastPacket({{0x00, 1, {0x11}, false}, {0x00, 3, {0x33}, false}});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Timeout);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(rep[1].result, Error::Timeout);
  CHECK_EQ(rep[2].result, Error::Ok);
  CHECK_EQ(out[2], 0x33U);
}

TEST_CASE(
    "Fast sync read by hand: an error number in a part is that device's and the others go on") {
  HandRig r;
  r.port.inject =
      FastPacket({{0x00, 1, {0x11}, false}, {0x04, 2, {0x00}, false}, {0x80, 3, {0x33}, false}});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{0xEE, 0xEE, 0xEE};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::DeviceError);
  CHECK_EQ(rep[0].result, Error::Ok);
  CHECK_EQ(rep[1].result, Error::DeviceError);
  CHECK_EQ(rep[1].error_byte, 0x04U);
  CHECK_EQ(out[1], 0xEEU); // a rejected part delivers no data
  CHECK_EQ(rep[2].result, Error::Ok);
  CHECK(rep[2].alert);
  CHECK_EQ(out[2], 0x33U);
}

TEST_CASE("Fast bulk read by hand: parts of different lengths land at their own offsets") {
  HandRig r;
  r.port.inject =
      FastPacket({{0x00, 1, {0x11, 0x12}, false}, {0x00, 2, {0x21, 0x22, 0x23}, false}});
  const std::array<BulkReadItem, 2> items{{{1, 64, 2}, {2, 64, 3}}};
  std::array<uint8_t, 5> out{};
  std::array<DeviceReply, 2> rep{};
  const auto done = r.drv.FastBulkRead(std::span<const BulkReadItem>{items},
                                       std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK(done.ok());
  CHECK_EQ(out[0], 0x11U);
  CHECK_EQ(out[1], 0x12U);
  CHECK_EQ(out[2], 0x21U);
  CHECK_EQ(out[4], 0x23U);
}

TEST_CASE("Sync read by hand: a reply from a device that is listed later is kept for that device") {
  HandRig r;
  // Only servo 2 answers; the host asked 1, 2, 3. Servo 1's turn finds servo 2's packet and must
  // not take it.
  r.port.inject = StatusV2(2, 0x00, {0x22});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::Timeout);
  CHECK_EQ(rep[0].result, Error::Timeout);
  CHECK_EQ(rep[1].result, Error::Ok);
  CHECK_EQ(out[1], 0x22U);
  CHECK_EQ(rep[2].result, Error::Timeout);
}

// ---------------------------------------------------------------------------------------------
// Second pass over the survivors of the mutation run: limits of Validate(), the baud tables, the
// fast-read firmware rule, the sign-magnitude edge and long packets.
// ---------------------------------------------------------------------------------------------

namespace {
/// True when `Validate()` accepts the default configuration with one change.
template <typename Change>
bool Accepts(Change change) {
  Config c{};
  change(c);
  return Validate(c) == Error::Ok;
}
} // namespace

TEST_CASE("Validate: every limit is inclusive at both ends") {
  CHECK(Accepts([](Config& c) { c.baud = kMinBaud; }));
  CHECK(!Accepts([](Config& c) { c.baud = kMinBaud - 1U; }));
  CHECK(Accepts([](Config& c) { c.baud = kMaxBaud; }));
  CHECK(!Accepts([](Config& c) { c.baud = kMaxBaud + 1U; }));

  CHECK(Accepts([](Config& c) { c.reply_timeout_us = kMinTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.reply_timeout_us = kMinTimeoutUs - 1U; }));
  CHECK(Accepts([](Config& c) { c.reply_timeout_us = kMaxTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.reply_timeout_us = kMaxTimeoutUs + 1U; }));

  CHECK(Accepts([](Config& c) { c.gap_timeout_us = kMinTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.gap_timeout_us = kMinTimeoutUs - 1U; }));
  CHECK(Accepts([](Config& c) { c.gap_timeout_us = kMaxTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.gap_timeout_us = kMaxTimeoutUs + 1U; }));

  CHECK(Accepts([](Config& c) { c.broadcast_timeout_us = kMinTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.broadcast_timeout_us = kMinTimeoutUs - 1U; }));
  CHECK(Accepts([](Config& c) { c.broadcast_timeout_us = kMaxTimeoutUs; }));
  CHECK(!Accepts([](Config& c) { c.broadcast_timeout_us = kMaxTimeoutUs + 1U; }));

  CHECK(Accepts([](Config& c) { c.reboot_wait_us = kMaxSettleUs; }));
  CHECK(!Accepts([](Config& c) { c.reboot_wait_us = kMaxSettleUs + 1U; }));
  CHECK(Accepts([](Config& c) { c.baud_change_wait_us = kMaxSettleUs; }));
  CHECK(!Accepts([](Config& c) { c.baud_change_wait_us = kMaxSettleUs + 1U; }));

  CHECK(Accepts([](Config& c) { c.retries = kMaxRetries; }));
  CHECK(!Accepts([](Config& c) { c.retries = kMaxRetries + 1U; }));
}

TEST_CASE("Baud tables: the divider scheme reaches value 254 and the tables stop at their count") {
  uint8_t value = 0;
  CHECK(BaudValueFor(Family::AxRxDx, 2'000'000U / 255U, value)); // 7843 bit/s is value 254
  CHECK_EQ(value, 254U);
  CHECK_EQ(BaudRateOf(Family::AxRxDx, 254), 2'000'000U / 255U);
  CHECK_EQ(BaudRateOf(Family::AxRxDx, 255), 0U);
  CHECK_EQ(BaudRateOf(Family::X430, 7), 4'500'000U); // the last of the table of 8
  CHECK_EQ(BaudRateOf(Family::X430, 8), 0U);
  CHECK_EQ(BaudRateOf(Family::Xl320, 3), 1'000'000U); // the last of the table of 4
  CHECK_EQ(BaudRateOf(Family::Xl320, 4), 0U);
}

TEST_CASE("SupportsFastRead: the first documented firmware counts") {
  CHECK(SupportsFastRead(1020, 45)); // XM430-W350: Fast Sync Read from firmware 45
  CHECK(!SupportsFastRead(1020, 44));
  CHECK(!SupportsFastRead(9999, 255)); // an unlisted model
}

TEST_CASE("EncodeValue: the sign-magnitude extremes") {
  std::array<uint8_t, 4> out{};
  CHECK(p::EncodeValue(Kind::SignMag10, 2, -1023, out));
  CHECK_EQ(out[0], 0xFFU);
  CHECK_EQ(out[1], 0x07U);
  CHECK_EQ(p::DecodeValue(Kind::SignMag10, std::span<const uint8_t>{out.data(), 2}), -1023);
}

TEST_CASE("FrameWriter: a protocol 2.0 length above 127 and above 255 keeps all its bits") {
  std::array<uint8_t, 600> buf{};
  p::FrameWriter w{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Write};
  for (int i = 0; i < 300; ++i) {
    w.Put(0x11);
  }
  const std::size_t len = w.Finish();
  CHECK(len > 0U);
  // length = instruction + parameters + 2 CRC bytes: 303 = 0x012F
  CHECK_EQ(buf[5], 0x2FU);
  CHECK_EQ(buf[6], 0x01U);
  std::array<uint8_t, 600> small{};
  p::FrameWriter w2{std::span<uint8_t>{small}, Protocol::V2, 1, p::Instruction::Write};
  for (int i = 0; i < 130; ++i) {
    w2.Put(0x22);
  }
  CHECK(w2.Finish() > 0U);
  CHECK_EQ(small[5], 133U); // 0x85: bit 7 set
  CHECK_EQ(small[6], 0U);
}

TEST_CASE("Fast sync read by hand: the call reports the first failing device, not the last") {
  HandRig r;
  r.port.inject =
      FastPacket({{0x04, 1, {0x00}, false}, {0x00, 9, {0x22}, false}, {0x00, 3, {0x33}, false}});
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.FastSyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(rep[0].result, Error::DeviceError);
  CHECK_EQ(rep[1].result, Error::DeviceIdMismatch);
  CHECK_EQ(done.error, Error::DeviceError);
}

TEST_CASE("Instructions without parameters refuse an ID the protocol does not have") {
  V2Rig r;
  CHECK_EQ(r.drv.Reboot(253).error,
           Error::InvalidArgument); // 252 is the highest ID of protocol 2.0
  CHECK_EQ(r.drv.Action(253).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, 0);
  V1Rig v1;
  CHECK_EQ(v1.drv.Reboot(255).error,
           Error::InvalidArgument); // 253 is the highest ID of protocol 1.0
  CHECK_EQ(v1.bus.writes, 0);
}
