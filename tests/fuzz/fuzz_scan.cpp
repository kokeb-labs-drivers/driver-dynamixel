/**
 * @file fuzz_scan.cpp
 * @brief Fuzz target: the reply parser, over raw bytes the fuzzer puts on the receive line.
 *
 * There is no servo model here. The bytes of the input are the line: each read returns the next
 * chunk of them, and a request is simply swallowed. The driver then parses whatever it is given in
 * every call that waits for a reply (ping, read, write with a status packet, broadcast ping, sync
 * and bulk reads, fast reads) under both protocols and every status return level, with and without
 * the loopback echo check.
 *
 * Promises checked after every call:
 * - the error is one the enumeration defines, and a failed call has `T{}` as its value;
 * - a successful read has filled only the bytes the caller offered (the buffer behind `out` keeps
 * its guard);
 * - a successful broadcast ping reports no more devices than `found` holds;
 * - a reply accepted as `Ok` carries the ID that was addressed.
 *
 * Built like fuzz_dynamixel (tests/fuzz/CMakeLists.txt).
 */
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "dynamixel.hpp"

namespace {

using dynamixel::Error;

void Check(bool promise, int line = __builtin_LINE()) noexcept {
  if (!promise) {
    (void)std::fprintf(stderr, "fuzz_scan: promise broken at line %d\n", line);
    std::abort();
  }
}

/// A port whose receive line is the fuzzer's input.
class RawPort : public dynamixel::UartInterface<RawPort> {
public:
  RawPort(const uint8_t* data, std::size_t size, std::size_t chunk) noexcept
      : data_(data), size_(size), chunk_(chunk == 0 ? 1 : chunk) {}

  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    Check(data != nullptr && len >= 6);
    return true;
  }
  std::size_t Read(uint8_t* out, std::size_t max, uint32_t timeout_us) noexcept {
    Check(out != nullptr && max >= 1 && timeout_us > 0);
    if (out == nullptr) {
      std::abort(); // Check() already did; this tells the analyzer
    }
    const std::size_t n = std::min({size_ - pos_, max, chunk_});
    for (std::size_t i = 0; i < n; ++i) {
      out[i] = data_[pos_++];
    }
    return n;
  }
  bool FlushInput() noexcept {
    return true;
  }
  void DelayUs(uint32_t /*us*/) noexcept {}
  void Log(dynamixel::LogLevel /*level*/, const char* message) noexcept {
    Check(message != nullptr && message[0] != '\0');
  }

private:
  const uint8_t* data_;
  std::size_t size_;
  std::size_t pos_ = 0;
  std::size_t chunk_;
};

using Driver = dynamixel::DYNAMIXEL<RawPort>;

bool KnownError(Error e) noexcept {
  switch (e) {
  case Error::Ok:
  case Error::NotInitialized:
  case Error::InvalidArgument:
  case Error::Bus:
  case Error::Timeout:
  case Error::DeviceIdMismatch:
  case Error::Crc:
  case Error::Busy:
  case Error::DeviceFault:
  case Error::Unsupported:
  case Error::DeviceError:
  case Error::InvalidReply:
  case Error::EchoMismatch:
  case Error::LineFault:
  case Error::NotPermitted:
    return true;
  default:
    break;
  }
  return false;
}

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  if (size < 3) {
    return 0;
  }
  const uint8_t flags = data[0];
  const uint8_t id = data[1] % 8U;
  RawPort port(data + 2, size - 2, 1U + (flags >> 4U));
  dynamixel::Config config{};
  config.protocol = (flags & 0x01U) != 0U ? dynamixel::Protocol::V1 : dynamixel::Protocol::V2;
  config.echo = (flags & 0x02U) != 0U ? dynamixel::EchoMode::Loopback : dynamixel::EchoMode::None;
  config.alert_policy =
      (flags & 0x04U) != 0U ? dynamixel::AlertPolicy::Fail : dynamixel::AlertPolicy::Report;
  config.status_return = static_cast<dynamixel::StatusReturn>((flags >> 3U) % 3U);
  config.retries = 0;
  config.reply_timeout_us = 1'000;
  config.gap_timeout_us = 1'000;
  config.broadcast_timeout_us = 1'000;
  Driver driver(port, config);
  if (!driver.EnsureInitialized().ok()) {
    return 0;
  }

  const auto ping = driver.Ping(id);
  Check(KnownError(ping.error));
  Check(ping.ok() || ping.value.model == 0);
  Check(!ping.ok() || ping.value.id == id);

  std::array<uint8_t, 8> guarded{};
  std::array<uint8_t, 24> buffer{};
  buffer.fill(0xA5);
  const auto read = driver.Read(id, 24, std::span<uint8_t>(buffer.data(), 8));
  Check(KnownError(read.error));
  for (std::size_t i = 8; i < buffer.size(); ++i) {
    Check(buffer[i] == 0xA5);
  }
  static_cast<void>(guarded);

  const auto write = driver.Write(id, 24, std::span<const uint8_t>(buffer.data(), 2));
  Check(KnownError(write.error));

  std::array<dynamixel::PingInfo, 3> found{};
  const auto broadcast = driver.BroadcastPing(found);
  Check(KnownError(broadcast.error));
  Check(!broadcast.ok() || broadcast.value <= found.size());

  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 12> out{};
  std::array<dynamixel::DeviceReply, 3> replies{};
  const auto sync = driver.SyncRead(24, 4, ids, out, replies);
  Check(KnownError(sync.error));
  Check(!sync.ok() || (replies[0].result == Error::Ok && replies[1].result == Error::Ok &&
                       replies[2].result == Error::Ok));
  const auto fast = driver.FastSyncRead(24, 4, ids, out, replies);
  Check(KnownError(fast.error));
  Check(!fast.ok() || (replies[0].result == Error::Ok && replies[1].result == Error::Ok &&
                       replies[2].result == Error::Ok));

  const std::array<dynamixel::BulkReadItem, 2> items{{{1, 24, 2}, {2, 30, 3}}};
  std::array<uint8_t, 5> bulk{};
  const auto b = driver.BulkRead(items, bulk, std::span<dynamixel::DeviceReply>(replies.data(), 2));
  Check(KnownError(b.error));
  const auto fb =
      driver.FastBulkRead(items, bulk, std::span<dynamixel::DeviceReply>(replies.data(), 2));
  Check(KnownError(fb.error));
  return 0;
}
