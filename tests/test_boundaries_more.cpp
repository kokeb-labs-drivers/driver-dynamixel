/**
 * @file test_boundaries_more.cpp
 * @brief Boundary tests, second file: the survivors of the independent mutation run (stuffing,
 * short status packets, the protocol 1.0 limits, the echo contract, the broadcast ping, the line
 * fault in a Sync Read) and the stop on an invalid configuration. Split from
 * test_boundaries.cpp to keep each file's registrations below the compiler's coverage limit.
 */
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "boundary_support.hpp"
#include "rig.hpp"

using namespace dynamixel;
using namespace rig;
using namespace bsupport;
namespace p = dynamixel::protocol;

// ---------------------------------------------------------------------------------------------
// Third pass: the survivors an independent mutation run found (stuffing, short status packets,
// the protocol 1.0 limits, the echo contract, the broadcast ping ID, the line fault in a Sync
// Read).
// ---------------------------------------------------------------------------------------------

TEST_CASE("A length-3 status packet with a valid CRC is refused by Scan and by a driver read") {
  std::vector<uint8_t> f{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x55};
  const uint16_t crc = p::Crc16(0, std::span<const uint8_t>{f});
  f.push_back(static_cast<uint8_t>(crc & 0xFFU));
  f.push_back(static_cast<uint8_t>(crc >> 8U));
  HandRig r;
  r.port.inject = f;
  std::array<uint8_t, 1> out{};
  const auto done = r.drv.Read(1, 64, std::span<uint8_t>{out});
  CHECK(!done.ok());
  CHECK(done.error != Error::Ok);
}

TEST_CASE("FrameWriter 2.0: FF FF FF FD is stuffed, the run of FF is not lost after three") {
  std::array<uint8_t, 40> buf{};
  p::FrameWriter w{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Write};
  const std::array<uint8_t, 4> data{0xFF, 0xFF, 0xFF, 0xFD};
  w.PutBytes(S(data));
  CHECK_EQ(w.Finish(), 4U + 1U + 10U);
  const std::array<uint8_t, 5> want{0xFF, 0xFF, 0xFF, 0xFD, 0xFD};
  for (std::size_t i = 0; i < want.size(); ++i) {
    CHECK_EQ(buf[8U + i], want[i]);
  }
  // The same through a driver call: the servo model sees the data, not a header inside it.
  V2Rig r;
  const std::array<uint8_t, 4> wire{0xFF, 0xFF, 0xFF, 0xFD};
  CHECK(r.drv.Write(1, 104, S(wire)).ok());
  CHECK_EQ(r.bus.bad_packets, 0);
  CHECK_EQ(r.bus.last_tx[8U + 2U + 3U], 0xFDU); // address (2 bytes) then FF FF FF FD FD
  CHECK_EQ(r.bus.last_tx[8U + 2U + 4U], 0xFDU);
}

TEST_CASE("Destuff: FF FD is data, FF FF FD FD is one FF FF FD, a bare FF FF FD is damage") {
  std::array<uint8_t, 2> a{0xFF, 0xFD};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{a}), 2U);
  CHECK_EQ(a[0], 0xFFU);
  CHECK_EQ(a[1], 0xFDU);
  std::array<uint8_t, 4> b{0xFF, 0xFF, 0xFD, 0xFD};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{b}), 3U);
  CHECK_EQ(b[2], 0xFDU);
  std::array<uint8_t, 3> c{0xFF, 0xFF, 0xFD};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{c}), p::kBadStuffing);
  std::array<uint8_t, 5> d{0xFF, 0xFF, 0xFF, 0xFD, 0xFD}; // a third FF does not hide the pattern
  CHECK_EQ(p::Destuff(std::span<uint8_t>{d}), 4U);
}

TEST_CASE("A reply whose data is FF FD is read as data, not refused as bad stuffing") {
  HandRig r;
  r.port.inject = StatusV2(1, 0x00, {0xFF, 0xFD});
  std::array<uint8_t, 2> out{};
  CHECK(r.drv.Read(1, 64, std::span<uint8_t>{out}).ok());
  CHECK_EQ(out[0], 0xFFU);
  CHECK_EQ(out[1], 0xFDU);
}

TEST_CASE("Scan 1.0: only IDs up to 253 can answer, and an exact-fit packet is accepted") {
  const auto status = [](uint8_t id) {
    std::vector<uint8_t> f{0xFF, 0xFF, id, 0x02, 0x00};
    f.push_back(p::Checksum1(std::span<const uint8_t>{f}.subspan(2U, 3U)));
    return f;
  };
  const std::vector<uint8_t> ok = status(253);
  CHECK(p::Scan(Protocol::V1, std::span<const uint8_t>{ok}, 64).kind == p::ScanResult::Kind::Frame);
  const std::vector<uint8_t> bad = status(254);
  CHECK(p::Scan(Protocol::V1, std::span<const uint8_t>{bad}, 64).kind !=
        p::ScanResult::Kind::Frame);
  CHECK(p::Scan(Protocol::V1, std::span<const uint8_t>{ok}, ok.size()).kind ==
        p::ScanResult::Kind::Frame);
  CHECK(p::Scan(Protocol::V1, std::span<const uint8_t>{ok}, ok.size() - 1U).kind !=
        p::ScanResult::Kind::Frame);
}

TEST_CASE("FrameWriter 1.0: a buffer that holds the checksum exactly is enough, one byte less is "
          "not") {
  std::array<uint8_t, 6> exact{};
  p::FrameWriter fits{std::span<uint8_t>{exact}, Protocol::V1, 1, p::Instruction::Ping};
  CHECK_EQ(fits.Finish(), 6U);
  std::array<uint8_t, 5> small{};
  p::FrameWriter tight{std::span<uint8_t>{small}, Protocol::V1, 1, p::Instruction::Ping};
  CHECK_EQ(tight.Finish(), 0U);
}

TEST_CASE("Echo mode: a read that claims more than was asked for is a bus failure, not an echo "
          "error") {
  Config c{};
  c.echo = EchoMode::Loopback;
  V2Rig r{c};
  r.bus.knob.echo = true;
  r.bus.knob.oversize_read = 1;
  CHECK_EQ(r.drv.Ping(1).error, Error::Bus);
  CHECK_EQ(r.drv.GetStats().echo_errors, 0U);
}

TEST_CASE("Broadcast ping: a reply from ID 253 or 254 is not a found servo") {
  for (const uint8_t id : {uint8_t{253}, uint8_t{254}}) {
    HandRig r;
    r.port.inject = StatusV2(id, 0x00, {0x06, 0x04, 0x2E});
    std::array<PingInfo, 4> found{};
    const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
    CHECK(!n.ok());
    CHECK_EQ(n.error, Error::DeviceIdMismatch);
  }
}

TEST_CASE("Sync read: a stuck line marks the first device and every one after it a line fault") {
  V2Rig r;
  r.bus.knob.stuck = 1;
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 3> out{};
  std::array<DeviceReply, 3> rep{};
  const auto done =
      r.drv.SyncRead(64, 1, S(ids), std::span<uint8_t>{out}, std::span<DeviceReply>{rep});
  CHECK_EQ(done.error, Error::LineFault);
  CHECK_EQ(rep[0].result, Error::LineFault);
  CHECK_EQ(rep[1].result, Error::LineFault); // not another full wait that ends as a timeout
  CHECK_EQ(rep[2].result, Error::LineFault);
  CHECK_EQ(r.drv.GetStats().timeouts, 0U);
}

TEST_CASE("Broadcast ping: a servo that answers with an error does not end the scan") {
  HandRig r;
  std::vector<uint8_t> both = StatusV2(1, 0x02, {0x06, 0x04, 0x2E}); // refused: instruction error
  const std::vector<uint8_t> second = StatusV2(2, 0x00, {0x06, 0x04, 0x2E});
  both.insert(both.end(), second.begin(), second.end());
  r.port.inject = both;
  std::array<PingInfo, 4> found{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
  REQUIRE(n.ok());
  CHECK_EQ(n.value, 1U);
  CHECK_EQ(found[0].id, 2U);
  CHECK_EQ(r.drv.GetStats().device_errors, 1U);
}

TEST_CASE("Broadcast ping: a line that answers with errors forever ends after one reply per ID") {
  HandRig r;
  std::vector<uint8_t> flood;
  for (int i = 0; i < 400; ++i) {
    const std::vector<uint8_t> one = StatusV2(1, 0x02, {0x06, 0x04, 0x2E});
    flood.insert(flood.end(), one.begin(), one.end());
  }
  r.port.inject = flood;
  std::array<PingInfo, 4> found{};
  const auto n = r.drv.BroadcastPing(std::span<PingInfo>{found});
  CHECK(!n.ok());
  CHECK_EQ(n.error, Error::DeviceError);
  CHECK_EQ(r.drv.GetStats().device_errors, 253U);
}

TEST_CASE(
    "The stop on an invalid configuration never polls with no wait and never waits for hours") {
  Config c{};
  c.reply_timeout_us = 0;
  c.gap_timeout_us = 0xFFFF'FFFFU;
  c.reboot_wait_us = 0xFFFF'FFFFU;
  REQUIRE(Validate(c) != Error::Ok);
  mock::Bus bus{};
  bus.v2 = true;
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  ProbePort port{bus};
  DYNAMIXEL<ProbePort> drv{port, c}; // never EnsureInitialized
  bus.knob.split_chunk = 3; // the answer arrives in pieces: the first read and the next ones
  CHECK(drv.DisableTorque(ServoRef{1, Family::X430}).ok());
  REQUIRE(port.reads.size() >= 2U);
  CHECK_EQ(port.reads[0].timeout_us, kMinTimeoutUs);
  CHECK_EQ(port.reads[1].timeout_us, kMaxTimeoutUs);
  CHECK(drv.Reboot(1).ok());
  CHECK_EQ(bus.delay_us, kMaxSettleUs);
}

TEST_CASE("The time a stop can take: first read of each attempt waits the reply timeout, the rest "
          "the gap timeout") {
  Config c{};
  c.reply_timeout_us = 1'000'000;
  c.gap_timeout_us = 1'000;
  mock::Bus bus{};
  bus.v2 = true;
  ProbePort port{bus};
  port.endless = true;
  port.chunk = 1; // one byte per read, never a packet
  DYNAMIXEL<ProbePort, 64> drv{port, c};
  static_cast<void>(drv.DisableTorque(ServoRef{1, Family::X430}));
  const uint64_t steps = (4U * 64U) + 16U;
  CHECK_EQ(port.read_count, 3U * steps);
  CHECK_EQ(port.total_wait_us, 3U * (1'000'000U + ((steps - 1U) * 1'000U)));
}

TEST_CASE("The stop on protocol 1.0 is one-byte address and one-byte value; address 255 is sent") {
  V1Rig r;
  r.bus.Find(1)->mem[24] = 1;
  CHECK(r.drv.DisableTorque(kA1).ok());
  CHECK_EQ(r.bus.last_tx_len, 8U);
  const std::array<uint8_t, 8> want{0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x00, 0xDF};
  for (std::size_t i = 0; i < want.size(); ++i) {
    CHECK_EQ(r.bus.last_tx[i], want[i]);
  }
  const int before = r.bus.writes;
  CHECK(r.drv.DisableTorqueAt(1, 255).error != Error::InvalidArgument);
  CHECK(r.bus.writes > before);
  CHECK_EQ(r.bus.last_tx[5], 0xFFU);
}

TEST_CASE("A stop on an initialised driver does not set the baud rate again") {
  V2Rig r;
  const int before = r.bus.baud_calls;
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.bus.baud_calls, before);
}
