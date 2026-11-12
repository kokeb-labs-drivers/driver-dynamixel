/**
 * @file boundary_support.hpp
 * @brief Helpers shared by the boundary test files: a probing port, replies built by hand and a
 * rig that uses them.
 */
#ifndef DYNAMIXEL_TEST_BOUNDARY_SUPPORT_HPP
#define DYNAMIXEL_TEST_BOUNDARY_SUPPORT_HPP

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "rig.hpp"

namespace bsupport {

using namespace dynamixel;
namespace p = dynamixel::protocol;

/// The mock port with a record of every read: the pointer, size and timeout the driver passed.
class ProbePort : public UartInterface<ProbePort> {
public:
  struct ReadCall {
    const uint8_t* data;
    std::size_t max;
    uint32_t timeout_us;
  };
  explicit ProbePort(mock::Bus& bus) noexcept : bus_(bus) {}
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    ++bus_.writes;
    bus_.Transmit(data, len);
    bus_.PushBytes(inject.data(), inject.size());
    return true;
  }
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    if (reads.size() < 64U) {
      reads.push_back({data, max, timeout_us});
    }
    ++read_count;
    total_wait_us += timeout_us;
    if (endless) {
      const std::size_t n = (chunk > 0U) ? std::min(max, chunk) : max;
      for (std::size_t i = 0; i < n; ++i) {
        data[i] = 0x37;
      }
      return n;
    }
    std::size_t n = std::min(max, bus_.Queued());
    if (bus_.knob.split_chunk > 0) {
      n = std::min(n, static_cast<std::size_t>(bus_.knob.split_chunk));
    }
    for (std::size_t i = 0; i < n; ++i) {
      data[i] = bus_.rx[bus_.rx_head++];
    }
    return n;
  }
  bool FlushInput() noexcept {
    ++bus_.flushes;
    bus_.Clear();
    return true;
  }
  void DelayUs(uint32_t us) noexcept {
    bus_.delay_us += us;
  }
  bool SetBaudRate(uint32_t baud) noexcept {
    bus_.baud = baud;
    return true;
  }
  std::vector<ReadCall> reads;
  bool endless{false};
  std::size_t chunk{0};      ///< With `endless`: bytes per read (0: as many as asked).
  uint64_t read_count{0};    ///< Every read, however many `reads` keeps.
  uint64_t total_wait_us{0}; ///< The sum of the timeouts of every read.
  std::vector<uint8_t>
      inject; ///< Bytes the line carries after the next request (a reply built by hand).

private:
  mock::Bus& bus_;
};

template <std::size_t N>
std::span<const uint8_t> S(const std::array<uint8_t, N>& a) {
  return std::span<const uint8_t>{a};
}

/// A status packet of protocol 2.0 with a valid CRC.
inline std::vector<uint8_t> StatusV2(uint8_t id, uint8_t error, std::vector<uint8_t> params) {
  std::vector<uint8_t> f{0xFF, 0xFF, 0xFD, 0x00, id, 0, 0, 0x55, error};
  f.insert(f.end(), params.begin(), params.end());
  const std::size_t length = params.size() + 4U;
  f[5] = static_cast<uint8_t>(length & 0xFFU);
  f[6] = static_cast<uint8_t>(length >> 8U);
  const uint16_t crc = p::Crc16(0, std::span<const uint8_t>{f});
  f.push_back(static_cast<uint8_t>(crc & 0xFFU));
  f.push_back(static_cast<uint8_t>(crc >> 8U));
  return f;
}

/// One part of a fast read reply built by hand.
struct Part {
  uint8_t error;
  uint8_t id;
  std::vector<uint8_t> data;
  bool bad_crc;
};

/// A fast read reply of protocol 2.0: each part ends with a CRC over the packet so far. A part
/// flagged `bad_crc` has a wrong CRC, and the parts after it are computed over the damaged bytes,
/// so that the whole packet's final CRC is right: only the part's own check field is wrong.
inline std::vector<uint8_t> FastPacket(const std::vector<Part>& parts) {
  std::size_t total = 1U;
  for (const Part& part : parts) {
    total += part.data.size() + 4U;
  }
  std::vector<uint8_t> f{0xFF,
                         0xFF,
                         0xFD,
                         0x00,
                         parts.front().id,
                         static_cast<uint8_t>(total & 0xFFU),
                         static_cast<uint8_t>(total >> 8U),
                         0x55};
  for (const Part& part : parts) {
    f.push_back(part.error);
    f.push_back(part.id);
    f.insert(f.end(), part.data.begin(), part.data.end());
    const uint16_t crc = p::Crc16(0, std::span<const uint8_t>{f});
    f.push_back(static_cast<uint8_t>((crc & 0xFFU) ^ (part.bad_crc ? 0x5AU : 0U)));
    f.push_back(static_cast<uint8_t>(crc >> 8U));
  }
  return f;
}

struct HandRig {
  mock::Bus bus{};
  ProbePort port{bus};
  DYNAMIXEL<ProbePort> drv{port};
  HandRig() {
    bus.v2 = true;
    static_cast<void>(drv.EnsureInitialized());
  }
};

} // namespace bsupport

#endif // DYNAMIXEL_TEST_BOUNDARY_SUPPORT_HPP
