/**
 * @file test_protocol.cpp
 * @brief The packet layer: CRC and checksum against the e-Manual's examples and an independent
 * implementation, framing, byte stuffing, the receive scanner, register decodings.
 */
#include <array>
#include <cstdint>
#include <span>

#include "dynamixel.hpp"
#include "mock_servo_bus.hpp"
#include "test_harness.hpp"

namespace p = dynamixel::protocol;
using dynamixel::Protocol;

namespace {
template <std::size_t N>
std::span<const uint8_t> S(const std::array<uint8_t, N>& a) {
  return std::span<const uint8_t>{a};
}
} // namespace

TEST_CASE("CRC-16 of the e-Manual ping example is 4E19") {
  const std::array<uint8_t, 8> ping{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01};
  CHECK_EQ(p::Crc16(0, S(ping)), 0x4E19U);
  CHECK_EQ(mock::Crc(ping.data(), ping.size()), 0x4E19U);
}

TEST_CASE("CRC-16 agrees with the bitwise reference on varied data, also chained") {
  std::array<uint8_t, 64> data{};
  for (std::size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>((i * 37U) ^ 0xA5U);
  }
  for (std::size_t n = 0; n <= data.size(); n += 7U) {
    CHECK_EQ(p::Crc16(0, std::span<const uint8_t>{data.data(), n}), mock::Crc(data.data(), n));
  }
  const uint16_t first = p::Crc16(0, std::span<const uint8_t>{data.data(), 20});
  CHECK_EQ(p::Crc16(first, std::span<const uint8_t>{data.data() + 20, 30}),
           mock::Crc(data.data(), 50));
}

TEST_CASE("Protocol 1.0 checksum is the complement of the sum") {
  const std::array<uint8_t, 6> write{0x01, 0x05, 0x03, 0x0C, 0x64, 0xAA};
  CHECK_EQ(p::Checksum1(S(write)), 0xDCU);
  const std::array<uint8_t, 3> ping{0x01, 0x02, 0x01};
  CHECK_EQ(p::Checksum1(S(ping)), 0xFBU);
}

TEST_CASE("FrameWriter builds the e-Manual ping packet for both protocols") {
  std::array<uint8_t, 32> buf{};
  p::FrameWriter v2{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Ping};
  const std::size_t n2 = v2.Finish();
  const std::array<uint8_t, 10> want2{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};
  REQUIRE(n2 == want2.size());
  CHECK(std::equal(want2.begin(), want2.end(), buf.begin()));
  p::FrameWriter v1{std::span<uint8_t>{buf}, Protocol::V1, 1, p::Instruction::Ping};
  const std::size_t n1 = v1.Finish();
  const std::array<uint8_t, 6> want1{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  REQUIRE(n1 == want1.size());
  CHECK(std::equal(want1.begin(), want1.end(), buf.begin()));
}

TEST_CASE("FrameWriter: the e-Manual read example, and stuffing of FF FF FD in the parameters") {
  std::array<uint8_t, 32> buf{};
  p::FrameWriter r{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Read};
  r.PutLe16(132);
  r.PutLe16(4);
  const std::size_t n = r.Finish();
  const std::array<uint8_t, 14> want{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                     0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15};
  REQUIRE(n == want.size());
  CHECK(std::equal(want.begin(), want.end(), buf.begin()));

  p::FrameWriter w{std::span<uint8_t>{buf}, Protocol::V2, 1, p::Instruction::Write};
  w.PutLe16(0x0040);
  w.Put(0xFF);
  w.Put(0xFF);
  w.Put(0xFD);
  const std::size_t m = w.Finish();
  // Length counts the inserted FD: instruction + 2 address + 4 data + 2 CRC = 9.
  REQUIRE(m == 16U);
  CHECK_EQ(buf[5], 9U);
  CHECK_EQ(buf[12], 0xFDU);
  CHECK_EQ(buf[13], 0xFDU);
  CHECK_EQ(mock::Crc(buf.data(), m - 2U),
           static_cast<uint16_t>(static_cast<uint32_t>(buf[m - 2U]) |
                                 (static_cast<uint32_t>(buf[m - 1U]) << 8U)));
}

TEST_CASE("FrameWriter refuses what does not fit and protocol 1.0 over 253 parameters") {
  std::array<uint8_t, 12> small{};
  p::FrameWriter a{std::span<uint8_t>{small}, Protocol::V2, 1, p::Instruction::Write};
  a.PutLe16(0);
  a.Put(1);
  a.Put(2);
  a.Put(3);
  CHECK_EQ(a.Finish(), 0U);
  std::array<uint8_t, 400> big{};
  p::FrameWriter b{std::span<uint8_t>{big}, Protocol::V1, 1, p::Instruction::Write};
  for (int i = 0; i < 254; ++i) {
    b.Put(0);
  }
  CHECK_EQ(b.Finish(), 0U);
}

TEST_CASE("Destuff removes the FD after FF FF FD and rejects a bare FF FF FD") {
  std::array<uint8_t, 6> ok{0x01, 0xFF, 0xFF, 0xFD, 0xFD, 0x02};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{ok}), 5U);
  CHECK_EQ(ok[4], 0x02U);
  std::array<uint8_t, 5> bad{0x01, 0xFF, 0xFF, 0xFD, 0x02};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{bad}), p::kBadStuffing);
  std::array<uint8_t, 3> tail{0xFF, 0xFF, 0xFD};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{tail}), p::kBadStuffing);
  std::array<uint8_t, 3> plain{1, 2, 3};
  CHECK_EQ(p::Destuff(std::span<uint8_t>{plain}), 3U);
}

namespace {
// The e-Manual's protocol 2.0 status packet example: XM430-W210, model 1030, firmware 38.
const std::array<uint8_t, 14> kStatus{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                      0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D};
} // namespace

TEST_CASE("Scan finds the e-Manual status packet and splits it") {
  std::array<uint8_t, 14> frame = kStatus;
  const p::ScanResult r = p::Scan(Protocol::V2, S(frame), 64);
  CHECK(r.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(r.frame_size, 14U);
  CHECK_EQ(r.discard, 0U);
  const p::StatusView v = p::Split(Protocol::V2, std::span<uint8_t>{frame});
  CHECK_EQ(v.id, 1U);
  CHECK_EQ(v.error, 0U);
  CHECK_EQ(v.params.size(), 3U);
  CHECK_EQ(p::LoadLe(v.params.first(2)), 1030U);
}

TEST_CASE("Scan resynchronises after garbage, a false header, and a corrupt packet") {
  std::array<uint8_t, 30> data{};
  // junk, a false header FF FF FD 00 with an absurd length, then the real packet
  const std::array<uint8_t, 5> junk{0x12, 0xFF, 0xFF, 0xFD, 0x07};
  std::size_t n = 0;
  for (const uint8_t b : junk) {
    data[n++] = b;
  }
  for (const uint8_t b : kStatus) {
    data[n++] = b;
  }
  const p::ScanResult r = p::Scan(Protocol::V2, std::span<const uint8_t>{data.data(), n}, 64);
  CHECK(r.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(r.discard, 5U);
  CHECK_EQ(r.frame_size, 14U);
  CHECK(r.garbage >= 4U);

  std::array<uint8_t, 28> two{};
  std::array<uint8_t, 14> bad = kStatus;
  bad[13] = 0x00;
  std::copy(bad.begin(), bad.end(), two.begin());
  std::copy(kStatus.begin(), kStatus.end(), two.begin() + 14);
  const p::ScanResult r2 = p::Scan(Protocol::V2, S(two), 64);
  CHECK(r2.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(r2.crc_drops, 1U);
  CHECK_EQ(r2.discard, 14U);
}

TEST_CASE("Scan keeps a partial packet and asks for the rest") {
  const p::ScanResult a = p::Scan(Protocol::V2, std::span<const uint8_t>{kStatus.data(), 9}, 64);
  CHECK(a.kind == p::ScanResult::Kind::NeedMore);
  CHECK_EQ(a.discard, 0U);
  CHECK_EQ(a.need, 5U);
  const p::ScanResult b = p::Scan(Protocol::V2, std::span<const uint8_t>{kStatus.data(), 2}, 64);
  CHECK(b.kind == p::ScanResult::Kind::NeedMore);
  CHECK_EQ(b.discard, 0U);
  const std::array<uint8_t, 3> noise{0x10, 0x20, 0xFF};
  const p::ScanResult c = p::Scan(Protocol::V2, S(noise), 64);
  CHECK(c.kind == p::ScanResult::Kind::NeedMore);
  CHECK_EQ(c.discard, 2U);
}

TEST_CASE("Scan refuses a packet larger than the buffer and a wrong instruction byte") {
  std::array<uint8_t, 14> big = kStatus;
  big[5] = 0xF0;
  const p::ScanResult a = p::Scan(Protocol::V2, S(big), 64);
  CHECK(a.kind == p::ScanResult::Kind::NeedMore);
  std::array<uint8_t, 14> notstatus = kStatus;
  notstatus[7] = 0x03;
  const p::ScanResult b = p::Scan(Protocol::V2, S(notstatus), 64);
  CHECK(b.kind == p::ScanResult::Kind::NeedMore);
}

TEST_CASE("Scan for protocol 1.0 finds a status packet and rejects a bad checksum") {
  std::array<uint8_t, 6> st{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  const p::ScanResult a = p::Scan(Protocol::V1, S(st), 32);
  CHECK(a.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(a.frame_size, 6U);
  st[5] = 0xFD;
  const p::ScanResult b = p::Scan(Protocol::V1, S(st), 32);
  CHECK(b.kind == p::ScanResult::Kind::NeedMore);
  CHECK_EQ(b.crc_drops, 1U);
  const p::StatusView v = p::Split(Protocol::V1, std::span<uint8_t>{st});
  CHECK_EQ(v.id, 1U);
  CHECK_EQ(v.params.size(), 0U);
}

TEST_CASE("Scan: protocol 1.0 with a run of FF before the header") {
  const std::array<uint8_t, 7> d{0xFF, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  const p::ScanResult r = p::Scan(Protocol::V1, S(d), 32);
  CHECK(r.kind == p::ScanResult::Kind::Frame);
  CHECK_EQ(r.discard, 1U);
}

TEST_CASE("Register decodings round-trip and refuse values that do not fit") {
  std::array<uint8_t, 4> raw{};
  const std::span<uint8_t, 4> out{raw};
  CHECK(p::EncodeValue(dynamixel::Kind::Signed, 4, -1000, out));
  CHECK_EQ(p::DecodeValue(dynamixel::Kind::Signed, std::span<const uint8_t>{raw.data(), 4}), -1000);
  CHECK(p::EncodeValue(dynamixel::Kind::Signed, 2, -2, out));
  CHECK_EQ(p::DecodeValue(dynamixel::Kind::Signed, std::span<const uint8_t>{raw.data(), 2}), -2);
  CHECK(!p::EncodeValue(dynamixel::Kind::Signed, 2, 40000, out));
  CHECK(!p::EncodeValue(dynamixel::Kind::Unsigned, 1, 256, out));
  CHECK(!p::EncodeValue(dynamixel::Kind::Unsigned, 1, -1, out));
  CHECK(p::EncodeValue(dynamixel::Kind::SignMag10, 2, -100, out));
  CHECK_EQ(raw[1], 0x04U);
  CHECK_EQ(p::DecodeValue(dynamixel::Kind::SignMag10, std::span<const uint8_t>{raw.data(), 2}),
           -100);
  CHECK(!p::EncodeValue(dynamixel::Kind::SignMag10, 2, 1024, out));
  CHECK(p::EncodeValue(dynamixel::Kind::Offset2048, 2, -5, out));
  CHECK_EQ(p::DecodeValue(dynamixel::Kind::Offset2048, std::span<const uint8_t>{raw.data(), 2}),
           -5);
  CHECK(!p::EncodeValue(dynamixel::Kind::Unsigned, 3, 1, out));
}

TEST_CASE("Error byte decoding: protocol 2.0 and 1.0") {
  using dynamixel::Classify;
  const auto a = Classify(Protocol::V2, 5, 0x80);
  CHECK(a.alert);
  CHECK(!a.rejected);
  const auto b = Classify(Protocol::V2, 5, 0x07);
  CHECK(!b.alert);
  CHECK(b.rejected);
  const auto c = Classify(Protocol::V1, 5, 0x24);
  CHECK(c.alert);
  CHECK(!c.rejected);
  const auto d = Classify(Protocol::V1, 5, 0x08);
  CHECK(d.rejected);
  CHECK(dynamixel::ErrorNumber(0x84) == dynamixel::Protocol2Error::DataRangeError);
  CHECK(dynamixel::ErrorNumber(0x7F) == dynamixel::Protocol2Error::ResultFail);
  CHECK(dynamixel::ErrorNumber(0x00) == dynamixel::Protocol2Error::None);
}
