/**
 * @file dynamixel_protocol.hpp
 * @brief The packet layer of DYNAMIXEL protocols 1.0 and 2.0: CRC-16, checksum, byte stuffing, the
 * frame writer, the receive scanner and register value coding. Pure functions, no I/O.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * Protocol 2.0 packet: `FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAM... | CRC_L CRC_H`; `LEN`
 * counts the instruction, the parameters and the CRC; a status packet carries the error byte as its
 * first parameter. Byte stuffing inserts `FD` after every `FF FF FD` of the parameters (e-Manual,
 * protocol 2.0, "Packet Process"). CRC-16 is polynomial 0x8005, initial value 0, no reflection
 * (e-Manual, "CRC Calculation"), computed over the stuffed packet from the header to the last
 * parameter. Protocol 1.0 packet: `FF FF | ID | LEN | INST | PARAM... | CHK`, `LEN` = parameters +
 * 2, `CHK = ~(ID + LEN + INST + PARAM...)` (e-Manual, protocol 1.0).
 */
#ifndef DYNAMIXEL_PROTOCOL_HPP
#define DYNAMIXEL_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "dynamixel_detail.hpp"
#include "dynamixel_tables.hpp"
#include "dynamixel_types.hpp"

namespace dynamixel::protocol {

/** @brief Instruction codes of both protocols (the e-Manual instruction tables). */
enum class Instruction : uint8_t {
  Ping = 0x01,         ///< Ping (both).
  Read = 0x02,         ///< Read (both).
  Write = 0x03,        ///< Write (both).
  RegWrite = 0x04,     ///< Reg Write (both).
  Action = 0x05,       ///< Action (both).
  FactoryReset = 0x06, ///< Factory Reset (both; protocol 2.0 has an option byte).
  Reboot = 0x08,       ///< Reboot (both).
  Clear = 0x10,        ///< Clear (protocol 2.0).
  Backup = 0x20,       ///< Control Table Backup (protocol 2.0).
  Status = 0x55,       ///< Status (return) packet (protocol 2.0).
  SyncRead = 0x82,     ///< Sync Read (protocol 2.0).
  SyncWrite = 0x83,    ///< Sync Write (both).
  FastSyncRead = 0x8A, ///< Fast Sync Read (protocol 2.0).
  BulkRead = 0x92,     ///< Bulk Read (both, different parameter layouts).
  BulkWrite = 0x93,    ///< Bulk Write (protocol 2.0).
  FastBulkRead = 0x9A, ///< Fast Bulk Read (protocol 2.0).
};

/// First three bytes of a protocol 2.0 header (the fourth is the reserved 0x00).
inline constexpr std::array<uint8_t, 4> kHeaderV2{0xFF, 0xFF, 0xFD, 0x00};
/// Bytes before the parameters of a protocol 2.0 instruction packet: header, ID, length,
/// instruction.
inline constexpr std::size_t kPrefixV2 = 8;
/// Bytes before the parameters of a protocol 1.0 instruction packet: header, ID, length,
/// instruction.
inline constexpr std::size_t kPrefixV1 = 5;
/// Fixed bytes of a protocol 2.0 packet around its parameters (prefix and CRC).
inline constexpr std::size_t kOverheadV2 = kPrefixV2 + 2U;
/// Fixed bytes of a protocol 1.0 packet around its parameters (prefix and checksum).
inline constexpr std::size_t kOverheadV1 = kPrefixV1 + 1U;
/// The most parameters a protocol 1.0 packet holds: the length byte (255) minus instruction and
/// checksum.
inline constexpr std::size_t kMaxParamsV1 = 253;

namespace detail_crc {
/**
 * @brief The CRC-16 lookup table, generated at build time from the polynomial 0x8005.
 * @return The 256 entries of the table.
 */
constexpr std::array<uint16_t, 256> MakeTable() noexcept {
  std::array<uint16_t, 256> table{};
  for (uint32_t i = 0U; i < 256U; ++i) {
    uint32_t crc = i << 8U;
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
      crc = ((crc & 0x8000U) != 0U) ? (((crc << 1U) ^ 0x8005U) & 0xFFFFU) : ((crc << 1U) & 0xFFFFU);
    }
    table[i] = static_cast<uint16_t>(crc);
  }
  return table;
}
/// The table `MakeTable()` built.
inline constexpr std::array<uint16_t, 256> kTable = MakeTable();
} // namespace detail_crc

/**
 * @brief Continue a protocol 2.0 CRC-16 over more bytes.
 * @param crc The CRC so far (0 at the start).
 * @param data The bytes.
 * @return The new CRC.
 */
[[nodiscard]] constexpr uint16_t Crc16(uint16_t crc, std::span<const uint8_t> data) noexcept {
  uint32_t acc = crc;
  for (const uint8_t byte : data) {
    const uint32_t index = ((acc >> 8U) ^ static_cast<uint32_t>(byte)) & 0xFFU;
    acc = ((acc << 8U) & 0xFFFFU) ^ static_cast<uint32_t>(detail_crc::kTable[index]);
  }
  return static_cast<uint16_t>(acc);
}

/**
 * @brief The protocol 1.0 checksum of the bytes from the ID to the last parameter.
 * @param data ID, length, instruction or error byte, parameters.
 * @return The bitwise complement of the low byte of their sum.
 */
[[nodiscard]] constexpr uint8_t Checksum1(std::span<const uint8_t> data) noexcept {
  uint32_t sum = 0U;
  for (const uint8_t byte : data) {
    sum += static_cast<uint32_t>(byte);
  }
  return static_cast<uint8_t>(~sum & 0xFFU);
}

/**
 * @brief Builds one instruction packet in a caller buffer, with byte stuffing for protocol 2.0.
 *
 * Put the parameters, then `Finish()`. Any write that does not fit, or a protocol 1.0 packet over
 * 253 parameters, makes `Finish()` return 0: nothing is sent then.
 */
class FrameWriter {
public:
  /**
   * @brief Start a packet.
   * @param buffer Where the packet goes; the driver's transmit buffer.
   * @param protocol The packet format.
   * @param id The target ID (0 to 254).
   * @param instruction The instruction code.
   */
  FrameWriter(std::span<uint8_t> buffer, Protocol protocol, uint8_t id,
              Instruction instruction) noexcept
      : buf_(buffer), protocol_(protocol) {
    if (protocol == Protocol::V2) {
      for (const uint8_t byte : kHeaderV2) {
        raw(byte);
      }
      raw(id);
      raw(0_u8); // length low, set by Finish()
      raw(0_u8); // length high
    } else {
      raw(0xFF_u8);
      raw(0xFF_u8);
      raw(id);
      raw(0_u8); // length, set by Finish()
    }
    raw(static_cast<uint8_t>(instruction));
    params_start_ = len_;
  }

  /**
   * @brief Append one parameter byte (stuffed in protocol 2.0).
   * @param byte The byte.
   */
  void Put(uint8_t byte) noexcept {
    raw(byte);
    if (protocol_ != Protocol::V2) {
      return;
    }
    // Stuffing: after FF FF FD of the parameters one more FD follows (e-Manual, protocol 2.0).
    if (byte == 0xFF_u8) {
      run_ = (run_ < 2U) ? (run_ + 1U) : 2U;
    } else if ((byte == 0xFD_u8) && (run_ == 2U)) {
      raw(0xFD_u8);
      run_ = 0U;
    } else {
      run_ = 0U;
    }
  }

  /**
   * @brief Append a 16-bit value, low byte first.
   * @param value The value.
   */
  void PutLe16(uint16_t value) noexcept {
    Put(static_cast<uint8_t>(static_cast<uint32_t>(value) & 0xFFU));
    Put(static_cast<uint8_t>((static_cast<uint32_t>(value) >> 8U) & 0xFFU));
  }

  /**
   * @brief Append bytes.
   * @param data The bytes.
   */
  void PutBytes(std::span<const uint8_t> data) noexcept {
    for (const uint8_t byte : data) {
      Put(byte);
    }
  }

  /**
   * @brief Complete the packet: length and CRC or checksum.
   * @return The packet size in bytes, or 0 when it did not fit or is too long for the protocol.
   */
  [[nodiscard]] std::size_t Finish() noexcept {
    if (protocol_ == Protocol::V2) {
      const std::size_t length = (len_ - params_start_) + 3U;
      if ((length > 0xFFFFU) || ((len_ + 2U) > buf_.size()) || overflow_) {
        return 0U;
      }
      buf_[5] = static_cast<uint8_t>(length & 0xFFU);
      buf_[6] = static_cast<uint8_t>((length >> 8U) & 0xFFU);
      const uint16_t crc = Crc16(0_u16, std::span<const uint8_t>{buf_.data(), len_});
      buf_[len_] = static_cast<uint8_t>(static_cast<uint32_t>(crc) & 0xFFU);
      buf_[len_ + 1U] = static_cast<uint8_t>((static_cast<uint32_t>(crc) >> 8U) & 0xFFU);
      return len_ + 2U;
    }
    const std::size_t params = len_ - params_start_;
    if ((params > kMaxParamsV1) || ((len_ + 1U) > buf_.size()) || overflow_) {
      return 0U;
    }
    buf_[3] = static_cast<uint8_t>(params + 2U);
    buf_[len_] = Checksum1(std::span<const uint8_t>{buf_.data() + 2, len_ - 2U});
    return len_ + 1U;
  }

private:
  void raw(uint8_t byte) noexcept {
    if (len_ < buf_.size()) {
      buf_[len_] = byte;
      ++len_;
    } else {
      overflow_ = true;
    }
  }

  std::span<uint8_t> buf_;
  Protocol protocol_;
  std::size_t len_{0};
  std::size_t params_start_{0};
  uint32_t run_{0};
  bool overflow_{false};
};

/// Returned by `Destuff()` for parameters that are not validly stuffed.
inline constexpr std::size_t kBadStuffing = static_cast<std::size_t>(-1);

/**
 * @brief Remove the stuffed bytes of received protocol 2.0 parameters, in place.
 * @param params The parameters as received (CRC already checked).
 * @return The unstuffed length, or `kBadStuffing` when an `FF FF FD` is not followed by `FD`.
 */
[[nodiscard]] constexpr std::size_t Destuff(std::span<uint8_t> params) noexcept {
  std::size_t out = 0;
  uint32_t run = 0U;
  for (std::size_t i = 0; i < params.size(); ++i) {
    const uint8_t byte = params[i];
    params[out] = byte;
    ++out;
    if (byte == 0xFF_u8) {
      run = (run < 2U) ? (run + 1U) : 2U;
    } else if ((byte == 0xFD_u8) && (run == 2U)) {
      // FF FF FD must be followed by the stuffed FD, which is dropped.
      if (((i + 1U) >= params.size()) || (params[i + 1U] != 0xFD_u8)) {
        return kBadStuffing;
      }
      ++i;
      run = 0U;
    } else {
      run = 0U;
    }
  }
  return out;
}

/** @brief What `Scan()` found in the receive buffer. */
struct ScanResult {
  /** @brief The outcome. */
  enum class Kind : uint8_t {
    NeedMore = 0, ///< No complete valid status packet yet; read `need` more bytes.
    Frame = 1,    ///< A valid status packet starts at `discard`, `frame_size` bytes long.
  };
  Kind kind{Kind::NeedMore}; ///< The outcome.
  std::size_t discard{
      0}; ///< Bytes before the packet (or all useless bytes) to drop from the front.
  std::size_t frame_size{0}; ///< Size of the valid packet when `kind` is `Frame`.
  std::size_t need{1};       ///< Smallest number of further bytes that can make progress.
  std::size_t crc_drops{0};  ///< Candidates dropped for a wrong CRC or checksum.
  std::size_t garbage{0};    ///< Bytes skipped that did not start a candidate.
};

namespace detail_scan {
/**
 * @brief Whether the four bytes at `i` are the protocol 2.0 header FF FF FD 00.
 * @param b The receive bytes; `i + 3` must be inside.
 * @param i The offset to test.
 * @return true on a header.
 */
constexpr bool MatchesV2(std::span<const uint8_t> b, std::size_t i) noexcept {
  return (b[i] == 0xFF_u8) && (b[i + 1U] == 0xFF_u8) && (b[i + 2U] == 0xFD_u8) &&
         (b[i + 3U] == 0x00_u8);
}

/**
 * @brief Whether the `left` bytes at `pos` (fewer than a header) are the start of one.
 * @param protocol The packet format.
 * @param bytes The receive bytes.
 * @param pos The offset of the bytes.
 * @param left How many bytes there are from `pos`.
 * @return true when every byte matches the header's beginning.
 */
constexpr bool IsPartialHeader(Protocol protocol, std::span<const uint8_t> bytes, std::size_t pos,
                               std::size_t left) noexcept {
  bool partial = true;
  for (std::size_t k = 0; k < left; ++k) {
    const uint8_t expect = (protocol == Protocol::V2) ? kHeaderV2[k] : 0xFF_u8;
    if (bytes[pos + k] != expect) {
      partial = false;
    }
  }
  return partial;
}

/**
 * @brief Whether a full header starts at `pos`.
 * @param protocol The packet format.
 * @param bytes The receive bytes.
 * @param pos The offset.
 * @param left How many bytes there are from `pos`, at least a header.
 * @return true on a header.
 */
constexpr bool IsCandidate(Protocol protocol, std::span<const uint8_t> bytes, std::size_t pos,
                           std::size_t left) noexcept {
  if (protocol == Protocol::V2) {
    return MatchesV2(bytes, pos);
  }
  // Protocol 1.0: FF FF then an ID that is not 0xFF (0xFF FF FF would be a longer run of header
  // bytes).
  return (bytes[pos] == 0xFF_u8) && (bytes[pos + 1U] == 0xFF_u8) &&
         ((left < 3U) || (bytes[pos + 2U] != 0xFF_u8));
}

/**
 * @brief The size a candidate packet claims, and whether that claim is plausible.
 * @param protocol The packet format.
 * @param bytes The receive bytes.
 * @param pos The offset of the candidate; its length field is inside `bytes`.
 * @param left How many bytes there are from `pos`.
 * @param max_frame The largest packet the receive buffer holds.
 * @param total Set to the claimed packet size.
 * @return true when the length, the size and (where visible) the instruction or ID are acceptable.
 */
constexpr bool IsPlausible(Protocol protocol, std::span<const uint8_t> bytes, std::size_t pos,
                           std::size_t left, std::size_t max_frame, std::size_t& total) noexcept {
  if (protocol == Protocol::V2) {
    const std::size_t length = static_cast<std::size_t>(bytes[pos + 5U]) |
                               (static_cast<std::size_t>(bytes[pos + 6U]) << 8U);
    total = 7U + length;
    const bool wrong_instruction =
        (left >= 8U) && (bytes[pos + 7U] != static_cast<uint8_t>(Instruction::Status));
    return (length >= 4U) && (total <= max_frame) && !wrong_instruction;
  }
  const std::size_t length = bytes[pos + 3U];
  total = 4U + length;
  return (length >= 2U) && (total <= max_frame) && (bytes[pos + 2U] <= kMaxIdV1);
}

/**
 * @brief Whether the CRC-16 or checksum of a complete packet matches.
 * @param protocol The packet format.
 * @param frame The whole packet.
 * @return true when it does.
 */
constexpr bool IsChecked(Protocol protocol, std::span<const uint8_t> frame) noexcept {
  const std::size_t total = frame.size();
  if (protocol == Protocol::V2) {
    const uint16_t crc = Crc16(0_u16, frame.first(total - 2U));
    return (frame[total - 2U] == static_cast<uint8_t>(static_cast<uint32_t>(crc) & 0xFFU)) &&
           (frame[total - 1U] == static_cast<uint8_t>((static_cast<uint32_t>(crc) >> 8U) & 0xFFU));
  }
  return frame[total - 1U] == Checksum1(frame.subspan(2U, total - 3U));
}
} // namespace detail_scan

/**
 * @brief Find the first valid status packet in received bytes, skipping garbage and resynchronising
 * after a damaged packet.
 *
 * A candidate is a header followed by an in-range length; it is valid when it fits `max_frame`, its
 * instruction byte is the status instruction (protocol 2.0) and its CRC or checksum matches. A
 * damaged candidate is dropped by one byte and the search continues inside it, so a real packet
 * that starts within the damaged bytes is still found.
 *
 * @param protocol The packet format.
 * @param bytes What the port returned so far.
 * @param max_frame The largest packet the receive buffer holds.
 * @return The result; see `ScanResult`.
 */
[[nodiscard]] constexpr ScanResult Scan(Protocol protocol, std::span<const uint8_t> bytes,
                                        std::size_t max_frame) noexcept {
  ScanResult result{};
  const std::size_t header = (protocol == Protocol::V2) ? 4U : 2U;
  const std::size_t min_for_len = (protocol == Protocol::V2) ? 7U : 4U;
  std::size_t pos = 0;
  while (pos < bytes.size()) {
    const std::size_t left = bytes.size() - pos;
    // A partial header at the end of the data may still become one: keep it.
    if ((left < header) && detail_scan::IsPartialHeader(protocol, bytes, pos, left)) {
      result.discard = pos;
      result.need = header - left;
      return result;
    }
    if ((left < header) || !detail_scan::IsCandidate(protocol, bytes, pos, left)) {
      result.garbage += 1U;
      ++pos;
      continue;
    }
    if (left < min_for_len) {
      result.discard = pos;
      result.need = min_for_len - left;
      return result;
    }
    std::size_t total = 0;
    if (!detail_scan::IsPlausible(protocol, bytes, pos, left, max_frame, total)) {
      result.garbage += 1U;
      ++pos;
      continue;
    }
    if (left < total) {
      result.discard = pos;
      result.need = total - left;
      return result;
    }
    if (detail_scan::IsChecked(protocol, bytes.subspan(pos, total))) {
      result.kind = ScanResult::Kind::Frame;
      result.discard = pos;
      result.frame_size = total;
      return result;
    }
    result.crc_drops += 1U;
    ++pos;
  }
  result.discard = pos;
  return result;
}

/** @brief The fields of a located status packet; the spans point into the receive buffer. */
struct StatusView {
  uint8_t id{0};             ///< The ID field.
  uint8_t error{0};          ///< The error byte.
  std::span<uint8_t> params; ///< The parameters, still stuffed for protocol 2.0.
};

/**
 * @brief Split a valid status packet found by `Scan()`.
 * @param protocol The packet format.
 * @param frame The whole packet.
 * @return The fields.
 */
[[nodiscard]] constexpr StatusView Split(Protocol protocol, std::span<uint8_t> frame) noexcept {
  StatusView view{};
  if (protocol == Protocol::V2) {
    view.id = frame[4];
    view.error = frame[8];
    view.params = frame.subspan(9U, frame.size() - 11U);
  } else {
    view.id = frame[2];
    view.error = frame[4];
    view.params = frame.subspan(5U, frame.size() - 6U);
  }
  return view;
}

/**
 * @brief Read a little-endian unsigned value of 1 to 4 bytes.
 * @param data The bytes.
 * @return The value; 0 for an empty or over-long span.
 */
[[nodiscard]] constexpr uint32_t LoadLe(std::span<const uint8_t> data) noexcept {
  if (data.empty() || (data.size() > 4U)) {
    return 0U;
  }
  uint32_t value = 0U;
  for (std::size_t i = data.size(); i > 0U; --i) {
    value = (value << 8U) | static_cast<uint32_t>(data[i - 1U]);
  }
  return value;
}

/**
 * @brief Decode a register's raw bytes.
 * @param kind The decoding of the register.
 * @param data The raw bytes, 1, 2 or 4.
 * @return The value as a signed 32-bit number.
 */
[[nodiscard]] constexpr int32_t DecodeValue(Kind kind, std::span<const uint8_t> data) noexcept {
  const uint32_t raw = LoadLe(data);
  const uint32_t bits = static_cast<uint32_t>(data.size()) * 8U;
  switch (kind) {
  case Kind::Signed:
    if ((bits < 32U) && ((raw & (1U << (bits - 1U))) != 0U)) {
      return static_cast<int32_t>(raw) - static_cast<int32_t>(1U << bits);
    }
    return static_cast<int32_t>(raw);
  case Kind::SignMag10: {
    const auto magnitude = static_cast<int32_t>(raw & 0x3FFU);
    return ((raw & 0x400U) != 0U) ? -magnitude : magnitude;
  }
  case Kind::Offset2048:
    return static_cast<int32_t>(raw) - 2048;
  case Kind::Unsigned:
    return static_cast<int32_t>(raw);
  default:
    break;
  }
  return static_cast<int32_t>(raw);
}

/**
 * @brief Encode a value into a register's raw bytes.
 * @param kind The decoding of the register.
 * @param size The register size, 1, 2 or 4.
 * @param value The value to write.
 * @param out Receives `size` bytes, low byte first.
 * @return false when the value does not fit the register (nothing is written).
 */
[[nodiscard]] constexpr bool EncodeValue(Kind kind, std::size_t size, int32_t value,
                                         std::span<uint8_t, 4> out) noexcept {
  if ((size != 1U) && (size != 2U) && (size != 4U)) {
    return false;
  }
  const int64_t v = value;
  const uint32_t bits = static_cast<uint32_t>(size) * 8U;
  const auto range = static_cast<int64_t>(uint64_t{1} << bits); // 2^bits
  const int64_t half = range / 2;
  uint32_t raw = 0U;
  switch (kind) {
  case Kind::Unsigned:
    if ((v < 0) || (v >= range)) {
      return false;
    }
    raw = static_cast<uint32_t>(v);
    break;
  case Kind::Signed:
    if ((v < -half) || (v >= half)) {
      return false;
    }
    raw = static_cast<uint32_t>(static_cast<uint64_t>(v) & 0xFFFFFFFFULL);
    break;
  case Kind::SignMag10:
    if ((v < -1023) || (v > 1023)) {
      return false;
    }
    raw = (v < 0) ? (static_cast<uint32_t>(-v) | 0x400U) : static_cast<uint32_t>(v);
    break;
  case Kind::Offset2048:
    if (((v + 2048) < 0) || ((v + 2048) >= range)) {
      return false;
    }
    raw = static_cast<uint32_t>(v + 2048);
    break;
  default:
    return false;
  }
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<uint8_t>((raw >> (8U * static_cast<uint32_t>(i))) & 0xFFU);
  }
  return true;
}

// The CRC of the e-Manual's ping example (protocol 2.0, Ping, ID 1): 19 4E.
static_assert(Crc16(0_u16, std::array<uint8_t, 8>{0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00,
                                                  0x01}) == 0x4E19U);
// The e-Manual's protocol 1.0 checksum example: ID 1, length 5, write, 0C 64 AA gives DC.
static_assert(Checksum1(std::array<uint8_t, 6>{0x01, 0x05, 0x03, 0x0C, 0x64, 0xAA}) == 0xDCU);

} // namespace dynamixel::protocol

namespace dynamixel::detail {
/// Last enumerator of `Kind`.
template <>
struct EnumLast<Kind> {
  static constexpr Kind kValue = Kind::Offset2048; ///< The last enumerator.
};
} // namespace dynamixel::detail

#endif // DYNAMIXEL_PROTOCOL_HPP
