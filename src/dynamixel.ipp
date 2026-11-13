/**
 * @file dynamixel.ipp
 * @brief Implementation of `dynamixel::DYNAMIXEL`; included at the end of `inc/dynamixel.hpp`.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 */
#ifndef DYNAMIXEL_IPP
#define DYNAMIXEL_IPP

namespace dynamixel {

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::validId(uint8_t id, bool allow_broadcast) const noexcept {
  const uint8_t highest = (config_.protocol == Protocol::V2) ? kMaxIdV2 : kMaxIdV1;
  return (id <= highest) || (allow_broadcast && (id == kBroadcastId));
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::replyExpected(ReplyRule rule, uint8_t id) const noexcept {
  if (id == kBroadcastId) {
    return false;
  }
  switch (rule) {
  case ReplyRule::ReadLevel:
    return config_.status_return != StatusReturn::PingOnly;
  case ReplyRule::WriteLevel:
    return config_.status_return == StatusReturn::All;
  default:
    break;
  }
  return false;
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::retryable(Error error) noexcept {
  return (error == Error::Timeout) || (error == Error::Crc) || (error == Error::InvalidReply) ||
         (error == Error::LineFault) || (error == Error::EchoMismatch);
}

template <UartPort Port, std::size_t FrameBytes>
uint32_t DYNAMIXEL<Port, FrameBytes>::saneTimeout(uint32_t us) noexcept {
  // A valid configuration is inside these limits already; the stop path also runs on one that is
  // not, and must neither poll with no wait nor wait for hours.
  return std::clamp(us, kMinTimeoutUs, kMaxTimeoutUs);
}

template <UartPort Port, std::size_t FrameBytes>
std::size_t DYNAMIXEL<Port, FrameBytes>::stuffedBound(std::size_t bytes) noexcept {
  // Stuffing adds one byte per FF FF FD of the parameters: at most one per three bytes.
  return bytes + (bytes / 3U) + 1U;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::ready() const noexcept {
  return initialized_ ? Error::Ok : Error::NotInitialized;
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::fail(Error error, const char* message) noexcept {
  detail::Log(port_, LogLevel::Error, message);
  return Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
const RegInfo* DYNAMIXEL<Port, FrameBytes>::lookup(Family family, Reg reg) const noexcept {
  // A family that does not speak the bus protocol has other registers at other addresses: a typed
  // call would write or read the wrong one, so it is refused before anything is sent.
  const uint8_t bit = (config_.protocol == Protocol::V2) ? kProtocolBit2 : kProtocolBit1;
  if ((Info(family).protocols & bit) == 0U) {
    return nullptr;
  }
  return Lookup(family, reg);
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::startForStop() noexcept {
  // A stop is never refused for the driver's state: no initialisation check, no consent, no earlier
  // failure counts. Only a port that cannot start at all stops it.
  if (!port_.EnsureInitialized()) {
    return false;
  }
  if (!initialized_ && (config_.baud >= kMinBaud) && (config_.baud <= kMaxBaud)) {
    static_cast<void>(applyRate()); // best effort: the frame goes out at the bus rate if it can
  }
  return true;
}

template <UartPort Port, std::size_t FrameBytes>
void DYNAMIXEL<Port, FrameBytes>::noteReply(const ReplyInfo& info) noexcept {
  last_reply_ = info;
  if (info.rejected) {
    ++stats_.device_errors;
  }
  if (info.alert) {
    ++stats_.alerts;
    if (latched_alert_.error_byte == 0U) {
      latched_alert_ = info;
    }
  }
}

template <UartPort Port, std::size_t FrameBytes>
void DYNAMIXEL<Port, FrameBytes>::dropFront(std::size_t count) noexcept {
  if (count >= rx_have_) {
    rx_have_ = 0;
    return;
  }
  if (count > 0U) {
    static_cast<void>(std::copy(rx_.begin() + static_cast<std::ptrdiff_t>(count),
                                rx_.begin() + static_cast<std::ptrdiff_t>(rx_have_), rx_.begin()));
    rx_have_ -= count;
  }
}

// ---------------------------------------------------------------------------------------------
// Transfer engine
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::readEcho(std::size_t len) noexcept {
  std::size_t got = 0;
  // Each pass reads at least one byte or returns, so at most `len` passes.
  for (std::size_t pass = 0; (pass < len) && (got < len); ++pass) {
    const std::size_t want = len - got;
    const std::size_t n = port_.Read(rx_.data() + got, want, saneTimeout(config_.gap_timeout_us));
    if (n > want) {
      return Error::Bus;
    }
    if (n == 0U) {
      ++stats_.echo_errors;
      return Error::EchoMismatch;
    }
    got += n;
  }
  const bool same =
      (got == len) &&
      std::equal(tx_.begin(), tx_.begin() + static_cast<std::ptrdiff_t>(len), rx_.begin());
  rx_have_ = 0;
  if (!same) {
    ++stats_.echo_errors;
    static_cast<void>(port_.FlushInput());
    return Error::EchoMismatch;
  }
  return Error::Ok;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::sendFrame(std::size_t len, bool stop) noexcept {
  tx_len_ = len;
  rx_have_ = 0;
  rx_drop_ = 0;
  // Anything still on the line is old: it must not be taken for this instruction's answer.
  flush_fault_ = !port_.FlushInput();
  if (flush_fault_ && !stop) {
    return Error::Bus;
  }
  // A stop is written even when the flush failed: the caller reports the fault after the write.
  if (!port_.Write(tx_.data(), len)) {
    return Error::Bus;
  }
  ++stats_.transactions;
  if (config_.echo == EchoMode::Loopback) {
    return readEcho(len);
  }
  return Error::Ok;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::receiveFrame(uint32_t first_timeout_us,
                                                std::size_t& frame_size) noexcept {
  dropFront(rx_drop_); // the reply accepted by the previous call is no longer needed
  rx_drop_ = 0;
  bool all_zero = true;
  bool all_ones = true;
  std::size_t total = 0;
  std::size_t crc_seen = 0;
  const std::size_t max_steps =
      (4U * FrameBytes) + 16U; // every pass reads a byte, drops one or returns
  for (std::size_t step = 0; step < max_steps; ++step) {
    const protocol::ScanResult scan = protocol::Scan(
        config_.protocol, std::span<const uint8_t>{rx_.data(), rx_have_}, FrameBytes);
    stats_.crc_errors += static_cast<uint32_t>(scan.crc_drops);
    stats_.garbage_bytes += static_cast<uint32_t>(scan.garbage);
    crc_seen += scan.crc_drops;
    dropFront(scan.discard);
    if (scan.kind == protocol::ScanResult::Kind::Frame) {
      frame_size = scan.frame_size;
      return Error::Ok;
    }
    if (rx_have_ >= FrameBytes) {
      rx_have_ = 0; // a full buffer without a frame is noise
    }
    const std::size_t room = FrameBytes - rx_have_;
    const uint32_t timeout = saneTimeout((total == 0U) ? first_timeout_us : config_.gap_timeout_us);
    const std::size_t n = port_.Read(rx_.data() + rx_have_, room, timeout);
    if (n > room) {
      return Error::Bus;
    }
    if (n == 0U) {
      if (crc_seen > 0U) {
        return Error::Crc;
      }
      if ((total >= 4U) && (all_zero || all_ones)) {
        ++stats_.line_faults;
        return Error::LineFault;
      }
      ++stats_.timeouts;
      return Error::Timeout;
    }
    for (std::size_t i = 0; i < n; ++i) {
      const uint8_t byte = rx_[rx_have_ + i];
      all_zero = all_zero && (byte == 0x00U);
      all_ones = all_ones && (byte == 0xFFU);
    }
    total += n;
    rx_have_ += n;
  }
  ++stats_.line_faults;
  return Error::InvalidReply; // the line never stopped talking
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::acceptStatus(std::size_t frame_size, uint8_t expect_id,
                                                std::size_t expect_params,
                                                std::span<uint8_t>& data) noexcept {
  const protocol::StatusView view =
      protocol::Split(config_.protocol, std::span<uint8_t>{rx_.data(), frame_size});
  if ((view.id != expect_id) && (static_cast<uint16_t>(view.id) != alt_id_)) {
    return Error::DeviceIdMismatch;
  }
  std::span<uint8_t> params = view.params;
  if (config_.protocol == Protocol::V2) {
    const std::size_t unstuffed = protocol::Destuff(params);
    if (unstuffed == protocol::kBadStuffing) {
      return Error::InvalidReply;
    }
    params = params.first(unstuffed);
  }
  const ReplyInfo info = Classify(config_.protocol, view.id, view.error);
  noteReply(info);
  if (info.rejected) {
    return Error::DeviceError;
  }
  if (params.size() != expect_params) {
    return Error::InvalidReply;
  }
  data = params;
  if (info.alert && (config_.alert_policy == AlertPolicy::Fail)) {
    return Error::DeviceFault;
  }
  return Error::Ok;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::transactOnce(std::size_t len, uint8_t id, bool expect_reply,
                                                std::size_t expect_params, bool stop,
                                                std::span<uint8_t>& data) noexcept {
  const Error sent = sendFrame(len, stop);
  if (sent != Error::Ok) {
    return sent;
  }
  if (!expect_reply) {
    return Error::Ok;
  }
  std::size_t frame_size = 0;
  const Error got = receiveFrame(config_.reply_timeout_us, frame_size);
  if (got != Error::Ok) {
    return got;
  }
  const Error accepted = acceptStatus(frame_size, id, expect_params, data);
  rx_drop_ = frame_size; // `data` points into `rx_`: it stays valid until the next receive
  return accepted;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::transact(std::size_t len, uint8_t id, bool expect_reply,
                                            std::size_t expect_params, bool idempotent,
                                            std::span<uint8_t>& data) noexcept {
  const uint32_t attempts = idempotent ? (1U + config_.retries) : 1U;
  Error error = Error::Ok;
  for (uint32_t attempt = 0; attempt < attempts; ++attempt) {
    if (attempt > 0U) {
      ++stats_.retries;
    }
    error = transactOnce(len, id, expect_reply, expect_params, false, data);
    if (!retryable(error)) {
      break;
    }
  }
  return error;
}

// ---------------------------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::applyRate() noexcept {
  if constexpr (detail::HasSetBaudRate<Port>) {
    if (!port_.SetBaudRate(config_.baud)) {
      return fail(Error::Bus, "dynamixel: SetBaudRate failed");
    }
  }
  return Result<void>::Success();
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::EnsureInitialized() noexcept {
  if (initialized_) {
    return Result<void>::Success();
  }
  if (Validate(config_) != Error::Ok) {
    return fail(Error::InvalidArgument, "dynamixel: invalid configuration");
  }
  if (!port_.EnsureInitialized()) {
    return fail(Error::Bus, "dynamixel: port start-up failed");
  }
  const Result<void> rate = applyRate();
  if (!rate.ok()) {
    return rate;
  }
  rx_have_ = 0;
  initialized_ = true;
  detail::Log(port_, LogLevel::Info, "dynamixel: initialised");
  return Result<void>::Success();
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Deinitialize() noexcept {
  initialized_ = false;
  rx_have_ = 0;
  return Result<void>::Success();
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Resync() noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  rx_have_ = 0;
  for (uint32_t round = 0; round < 16U; ++round) {
    if (!port_.FlushInput()) {
      return fail(Error::Bus, "dynamixel: flush failed");
    }
    const std::size_t n = port_.Read(rx_.data(), FrameBytes, config_.gap_timeout_us);
    if (n > FrameBytes) {
      return fail(Error::Bus, "dynamixel: Read returned too much");
    }
    if (n == 0U) {
      return Result<void>::Success(); // quiet for one gap timeout
    }
  }
  ++stats_.line_faults;
  return fail(Error::LineFault, "dynamixel: the line does not go quiet");
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::ApplyConfig(const Config& config) noexcept {
  if (Validate(config) != Error::Ok) {
    return fail(Error::InvalidArgument, "dynamixel: invalid configuration");
  }
  const Config previous = config_;
  config_ = config;
  if (initialized_) {
    const Result<void> rate = applyRate();
    if (!rate.ok()) {
      config_ = previous;
      return rate;
    }
  }
  return Result<void>::Success();
}

// ---------------------------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Result<PingInfo> DYNAMIXEL<Port, FrameBytes>::Ping(uint8_t id) noexcept {
  if (ready() != Error::Ok) {
    return Result<PingInfo>::Failure(Error::NotInitialized);
  }
  if (!validId(id, false)) {
    return Result<PingInfo>::Failure(Error::InvalidArgument);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id,
                               protocol::Instruction::Ping};
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<PingInfo>::Failure(Error::InvalidArgument);
  }
  const std::size_t expect = (config_.protocol == Protocol::V2) ? 3U : 0U;
  std::span<uint8_t> data{};
  const Error error = transact(len, id, true, expect, true, data);
  if (error != Error::Ok) {
    return Result<PingInfo>::Failure(error);
  }
  PingInfo info{};
  info.id = id;
  if (config_.protocol == Protocol::V2) {
    info.model = static_cast<uint16_t>(protocol::LoadLe(data.first(2U)));
    info.firmware = data[2];
    info.family = FamilyOf(info.model);
  }
  return Result<PingInfo>::Success(info);
}

template <UartPort Port, std::size_t FrameBytes>
Result<PingInfo> DYNAMIXEL<Port, FrameBytes>::Identify(uint8_t id) noexcept {
  const Result<PingInfo> pinged = Ping(id);
  if (!pinged.ok() || (config_.protocol == Protocol::V2)) {
    return pinged;
  }
  if (config_.status_return == StatusReturn::PingOnly) {
    return Result<PingInfo>::Failure(Error::Unsupported);
  }
  std::array<uint8_t, 3> block{};
  const Result<void> read = Read(id, 0U, std::span<uint8_t>{block});
  if (!read.ok()) {
    return Result<PingInfo>::Failure(read.error);
  }
  PingInfo info = pinged.value;
  info.model = static_cast<uint16_t>(protocol::LoadLe(std::span<const uint8_t>{block}.first(2U)));
  info.firmware = block[2];
  info.family = FamilyOf(info.model);
  return Result<PingInfo>::Success(info);
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::pingReply(std::size_t frame_size,
                                             std::span<const PingInfo> known,
                                             PingInfo& info) noexcept {
  const protocol::StatusView view =
      protocol::Split(config_.protocol, std::span<uint8_t>{rx_.data(), frame_size});
  bool fresh = (view.id <= kMaxIdV2);
  for (const PingInfo& seen : known) {
    fresh = fresh && (seen.id != view.id);
  }
  if (!fresh) {
    return Error::DeviceIdMismatch;
  }
  std::span<uint8_t> data{};
  const Error accepted = acceptStatus(frame_size, view.id, 3U, data);
  if ((accepted == Error::Ok) || (accepted == Error::DeviceFault)) {
    info.id = view.id;
    info.model = static_cast<uint16_t>(protocol::LoadLe(data.first(2U)));
    info.firmware = data[2];
    info.family = FamilyOf(info.model);
  }
  return accepted;
}

template <UartPort Port, std::size_t FrameBytes>
Result<uint32_t> DYNAMIXEL<Port, FrameBytes>::BroadcastPing(std::span<PingInfo> found) noexcept {
  if (ready() != Error::Ok) {
    return Result<uint32_t>::Failure(Error::NotInitialized);
  }
  if (found.empty()) {
    return Result<uint32_t>::Failure(Error::InvalidArgument);
  }
  if (config_.protocol != Protocol::V2) {
    return Result<uint32_t>::Failure(Error::Unsupported);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, kBroadcastId,
                               protocol::Instruction::Ping};
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<uint32_t>::Failure(Error::InvalidArgument);
  }
  const Error sent = sendFrame(len, false);
  if (sent != Error::Ok) {
    return Result<uint32_t>::Failure(sent);
  }
  uint32_t count = 0;
  Error last = Error::Timeout;
  // At most one reply per ID, so a line that never stops answering ends the scan too.
  for (uint32_t replies = 0; (count < found.size()) && (replies <= kMaxIdV2); ++replies) {
    std::size_t frame_size = 0;
    last = receiveFrame(config_.broadcast_timeout_us, frame_size);
    if (last != Error::Ok) {
      break;
    }
    PingInfo info{};
    last = pingReply(frame_size, found.first(count), info);
    rx_drop_ = frame_size;
    if (last == Error::DeviceError) {
      continue; // this servo answered with an error: it is not counted, the others still are
    }
    if ((last != Error::Ok) && (last != Error::DeviceFault)) {
      break;
    }
    found[count] = info;
    ++count;
  }
  if ((count == 0U) && (last != Error::Timeout)) {
    return Result<uint32_t>::Failure(last);
  }
  return Result<uint32_t>::Success(count);
}

template <UartPort Port, std::size_t FrameBytes>
Result<uint32_t> DYNAMIXEL<Port, FrameBytes>::ScanIds(uint8_t first, uint8_t last,
                                                      std::span<PingInfo> found) noexcept {
  if (ready() != Error::Ok) {
    return Result<uint32_t>::Failure(Error::NotInitialized);
  }
  if (found.empty() || (first > last) || !validId(last, false)) {
    return Result<uint32_t>::Failure(Error::InvalidArgument);
  }
  uint32_t count = 0;
  for (uint32_t id = first; (id <= last) && (count < found.size()); ++id) {
    const Result<PingInfo> pinged = Ping(static_cast<uint8_t>(id));
    if (pinged.ok()) {
      found[count] = pinged.value;
      ++count;
    } else if (pinged.error == Error::Bus) {
      return Result<uint32_t>::Failure(Error::Bus);
    } else {
      continue; // silent or damaged: this ID is not counted
    }
  }
  return Result<uint32_t>::Success(count);
}

// ---------------------------------------------------------------------------------------------
// Raw access
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Read(uint8_t id, uint16_t address,
                                               std::span<uint8_t> out) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  const bool v2 = (config_.protocol == Protocol::V2);
  const std::size_t reply_room = v2 ? (stuffedBound(out.size()) + 11U) : (out.size() + 6U);
  if (!validId(id, false) || out.empty() || (reply_room > FrameBytes) ||
      (!v2 && ((address > 255U) || (out.size() > protocol::kMaxParamsV1))) ||
      (out.size() > 0xFFFFU)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!replyExpected(ReplyRule::ReadLevel, id)) {
    return Result<void>::Failure(Error::Unsupported);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id,
                               protocol::Instruction::Read};
  if (v2) {
    writer.PutLe16(address);
    writer.PutLe16(static_cast<uint16_t>(out.size()));
  } else {
    writer.Put(static_cast<uint8_t>(address));
    writer.Put(static_cast<uint8_t>(out.size()));
  }
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> data{};
  const Error error = transact(len, id, true, out.size(), true, data);
  if (error != Error::Ok) {
    return Result<void>::Failure(error);
  }
  static_cast<void>(std::copy(data.begin(), data.end(), out.begin()));
  return Result<void>::Success();
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::sendWrite(protocol::Instruction instruction, uint8_t id,
                                                    uint16_t address, std::span<const uint8_t> data,
                                                    bool settle) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  const bool v2 = (config_.protocol == Protocol::V2);
  if (!validId(id, true) || data.empty() || (!v2 && ((address > 255U) || (data.size() > 250U)))) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id, instruction};
  if (v2) {
    writer.PutLe16(address);
  } else {
    writer.Put(static_cast<uint8_t>(address));
  }
  writer.PutBytes(data);
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> reply{};
  const Error error = transact(len, id, replyExpected(ReplyRule::WriteLevel, id), 0U, false, reply);
  if (settle) {
    port_.DelayUs(config_.reboot_wait_us);
  }
  return (error == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Write(uint8_t id, uint16_t address,
                                                std::span<const uint8_t> data) noexcept {
  return sendWrite(protocol::Instruction::Write, id, address, data, false);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::RegWrite(uint8_t id, uint16_t address,
                                                   std::span<const uint8_t> data) noexcept {
  return sendWrite(protocol::Instruction::RegWrite, id, address, data, false);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::sendSimple(protocol::Instruction instruction, uint8_t id,
                                                     std::span<const uint8_t> params,
                                                     bool settle) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  if (!validId(id, true)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id, instruction};
  writer.PutBytes(params);
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> reply{};
  const Error error = transact(len, id, replyExpected(ReplyRule::WriteLevel, id), 0U, false, reply);
  if (settle) {
    port_.DelayUs(config_.reboot_wait_us);
  }
  return (error == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Action(uint8_t id) noexcept {
  return sendSimple(protocol::Instruction::Action, id, std::span<const uint8_t>{}, false);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Reboot(uint8_t id) noexcept {
  // A reboot also ends a torque hold: like the stop it is sent whatever the driver's state and
  // whatever the input flush did; a flush fault is reported after the write.
  if (!validId(id, true)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!startForStop()) {
    return fail(Error::Bus, "dynamixel: port start-up failed during a reboot");
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id,
                               protocol::Instruction::Reboot};
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> reply{};
  Error error = transactOnce(len, id, replyExpected(ReplyRule::WriteLevel, id), 0U, true, reply);
  if (flush_fault_ && (error == Error::Ok)) {
    error = Error::Bus; // sent, but old bytes could not be cleared: the answer is not trusted
  }
  port_.DelayUs(
      std::min(config_.reboot_wait_us, kMaxSettleUs)); // the stop path may run on an invalid config
  return (error == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::FactoryReset(uint8_t id, ResetOption option) noexcept {
  const bool v2 = (config_.protocol == Protocol::V2);
  bool known = false;
  switch (option) {
  case ResetOption::KeepIdAndBaud:
  case ResetOption::KeepId:
  case ResetOption::All:
    known = true;
    break;
  default:
    break;
  }
  if (!known || (!v2 && (option != ResetOption::All)) ||
      ((id == kBroadcastId) && (!v2 || (option == ResetOption::All))) || !validId(id, true)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!config_.allow_destructive) {
    return fail(Error::NotPermitted, "dynamixel: factory reset needs allow_destructive");
  }
  const std::array<uint8_t, 1> params{static_cast<uint8_t>(option)};
  return sendSimple(protocol::Instruction::FactoryReset, id,
                    v2 ? std::span<const uint8_t>{params} : std::span<const uint8_t>{}, true);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Clear(uint8_t id, ClearOption option) noexcept {
  if (config_.protocol != Protocol::V2) {
    return Result<void>::Failure(Error::Unsupported);
  }
  std::array<uint8_t, 5> params{};
  switch (option) {
  case ClearOption::MultiTurn:
    params = {0x01, 0x44, 0x58, 0x4C, 0x22};
    break;
  case ClearOption::Errors:
    params = {0x02, 0x45, 0x52, 0x43, 0x4C};
    break;
  default:
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!config_.allow_destructive) {
    return fail(Error::NotPermitted, "dynamixel: clear needs allow_destructive");
  }
  return sendSimple(protocol::Instruction::Clear, id, std::span<const uint8_t>{params}, false);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::Backup(uint8_t id, BackupOption option) noexcept {
  if (config_.protocol != Protocol::V2) {
    return Result<void>::Failure(Error::Unsupported);
  }
  const std::array<uint8_t, 4> magic{0x43, 0x54, 0x52, 0x4C};
  std::array<uint8_t, 5> params{};
  bool restore = false;
  switch (option) {
  case BackupOption::Store:
    params[0] = 0x01;
    break;
  case BackupOption::Restore:
    params[0] = 0x02;
    restore = true;
    break;
  default:
    return Result<void>::Failure(Error::InvalidArgument);
  }
  for (std::size_t i = 0; i < magic.size(); ++i) {
    params[i + 1U] = magic[i];
  }
  if (restore && !config_.allow_destructive) {
    return fail(Error::NotPermitted, "dynamixel: restore needs allow_destructive");
  }
  return sendSimple(protocol::Instruction::Backup, id, std::span<const uint8_t>{params}, restore);
}

// ---------------------------------------------------------------------------------------------
// Sync and bulk
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SyncWrite(uint16_t address, uint16_t length,
                                                    std::span<const uint8_t> ids,
                                                    std::span<const uint8_t> data) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  const bool v2 = (config_.protocol == Protocol::V2);
  if (ids.empty() || (length == 0U) ||
      (data.size() != (ids.size() * static_cast<std::size_t>(length))) ||
      (!v2 && ((address > 255U) || (length > 250U)))) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!std::all_of(ids.begin(), ids.end(), [this](uint8_t id) { return validId(id, false); })) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, kBroadcastId,
                               protocol::Instruction::SyncWrite};
  if (v2) {
    writer.PutLe16(address);
    writer.PutLe16(length);
  } else {
    writer.Put(static_cast<uint8_t>(address));
    writer.Put(static_cast<uint8_t>(length));
  }
  for (std::size_t k = 0; k < ids.size(); ++k) {
    writer.Put(ids[k]);
    writer.PutBytes(data.subspan(k * length, length));
  }
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> reply{};
  const Error error = transactOnce(len, kBroadcastId, false, 0U, false, reply);
  return (error == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::BulkWrite(std::span<const BulkWriteItem> items) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  if (config_.protocol != Protocol::V2) {
    return Result<void>::Failure(Error::Unsupported);
  }
  if (items.empty()) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  for (std::size_t i = 0; i < items.size(); ++i) {
    bool bad =
        !validId(items[i].id, false) || items[i].data.empty() || (items[i].data.size() > 0xFFFFU);
    for (std::size_t j = 0; j < i; ++j) {
      bad = bad || (items[j].id == items[i].id);
    }
    if (bad) {
      return Result<void>::Failure(Error::InvalidArgument);
    }
  }
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, kBroadcastId,
                               protocol::Instruction::BulkWrite};
  for (const BulkWriteItem& item : items) {
    writer.Put(item.id);
    writer.PutLe16(item.address);
    writer.PutLe16(static_cast<uint16_t>(item.data.size()));
    writer.PutBytes(item.data);
  }
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::span<uint8_t> reply{};
  const Error error = transactOnce(len, kBroadcastId, false, 0U, false, reply);
  return (error == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(error);
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::separateReceive(const detail::MultiRequest& rq, std::size_t k,
                                                   std::span<uint8_t>& data) noexcept {
  const uint8_t id = rq.IdAt(k);
  std::size_t frame_size = 0;
  const Error received = receiveFrame(config_.reply_timeout_us, frame_size);
  if (received != Error::Ok) {
    return received;
  }
  const protocol::StatusView view =
      protocol::Split(config_.protocol, std::span<uint8_t>{rx_.data(), frame_size});
  // The frame belongs to a device listed after this one: this one stayed silent. Keep the frame for
  // its device.
  if ((view.id != id) && rq.ListedAfter(k, view.id)) {
    return Error::Timeout;
  }
  const Error accepted = acceptStatus(frame_size, id, rq.LengthAt(k), data);
  rx_drop_ = frame_size;
  return accepted;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::collectSeparate(const detail::MultiRequest& rq,
                                                   std::span<uint8_t> out,
                                                   std::span<DeviceReply> replies) noexcept {
  const std::size_t count = rq.Count();
  std::size_t offset = 0;
  Error first_error = Error::Ok;
  for (std::size_t k = 0; k < count; ++k) {
    std::span<uint8_t> data{};
    const Error error = separateReceive(rq, k, data);
    DeviceReply& reply = replies[k];
    reply.id = rq.IdAt(k);
    reply.result = error;
    const bool answered =
        (error == Error::Ok) || (error == Error::DeviceError) || (error == Error::DeviceFault);
    reply.error_byte = answered ? last_reply_.error_byte : 0U;
    reply.alert = answered && last_reply_.alert;
    if (error == Error::Ok) {
      static_cast<void>(
          std::copy(data.begin(), data.end(), out.begin() + static_cast<std::ptrdiff_t>(offset)));
    }
    offset += rq.LengthAt(k);
    if ((error != Error::Ok) && (first_error == Error::Ok)) {
      first_error = error;
    }
    if ((error == Error::Bus) || (error == Error::LineFault)) {
      for (std::size_t rest = k + 1U; rest < count; ++rest) {
        replies[rest].id = rq.IdAt(rest);
        replies[rest].result = error;
      }
      break;
    }
  }
  return first_error;
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::fastPartCrcOk(std::size_t pos, std::size_t part) const noexcept {
  const uint16_t crc = protocol::Crc16(0, std::span<const uint8_t>{rx_.data(), pos + part - 2U});
  return (rx_[pos + part - 2U] == static_cast<uint8_t>(static_cast<uint32_t>(crc) & 0xFFU)) &&
         (rx_[pos + part - 1U] == static_cast<uint8_t>((static_cast<uint32_t>(crc) >> 8U) & 0xFFU));
}

template <UartPort Port, std::size_t FrameBytes>
void DYNAMIXEL<Port, FrameBytes>::fastAccept(uint8_t id, std::size_t want, std::size_t pos,
                                             std::span<uint8_t> out, std::size_t offset,
                                             DeviceReply& reply) noexcept {
  const ReplyInfo info = Classify(config_.protocol, id, rx_[pos]);
  noteReply(info);
  reply.error_byte = info.error_byte;
  reply.alert = info.alert;
  if (info.rejected) {
    reply.result = Error::DeviceError;
  } else if (info.alert && (config_.alert_policy == AlertPolicy::Fail)) {
    reply.result = Error::DeviceFault;
  } else {
    reply.result = Error::Ok;
    static_cast<void>(std::copy_n(rx_.begin() + static_cast<std::ptrdiff_t>(pos + 2U), want,
                                  out.begin() + static_cast<std::ptrdiff_t>(offset)));
  }
}

template <UartPort Port, std::size_t FrameBytes>
void DYNAMIXEL<Port, FrameBytes>::fastPart(const detail::MultiRequest& rq, std::size_t k,
                                           detail::FastCursor& cur, std::span<uint8_t> out,
                                           std::size_t offset, DeviceReply& reply) noexcept {
  if (!cur.open) {
    return; // the packet is exhausted or damaged: the device keeps the packet-level result
  }
  if ((cur.pos + 2U) > cur.have) {
    cur.open = false; // the packet stopped before this part's ID
    return;
  }
  const uint8_t id = rq.IdAt(k);
  const std::size_t want = rq.LengthAt(k);
  const uint8_t part_id = rx_[cur.pos + 1U];
  const std::size_t part = want + 4U; // error, ID, data, CRC
  if ((part_id != id) && rq.ListedAfter(k, part_id)) {
    return; // skipped: the part belongs to a later device, so this one did not answer (Timeout)
  }
  if ((cur.pos + part) > cur.have) {
    cur.open = false; // the packet stopped before the end of this part
    return;
  }
  if (!fastPartCrcOk(cur.pos, part)) {
    reply.result = Error::Crc;
    ++stats_.crc_errors;
    cur.open = false;
    cur.stop = Error::Crc;
    return;
  }
  if (part_id != id) {
    reply.result = Error::DeviceIdMismatch;
    cur.open = false;
    cur.stop = Error::DeviceIdMismatch;
    return;
  }
  fastAccept(id, want, cur.pos, out, offset, reply);
  cur.pos += part;
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::collectFast(const detail::MultiRequest& rq,
                                               std::span<uint8_t> out,
                                               std::span<DeviceReply> replies) noexcept {
  std::size_t frame_size = 0;
  const Error received = receiveFrame(config_.reply_timeout_us, frame_size);
  // A whole packet, or the start of one that stopped early: only those carry parts to read.
  const bool parts_possible = (received == Error::Ok) || (received == Error::Timeout);
  detail::FastCursor cur{};
  cur.pos = 8; // header (7) and the status instruction
  cur.have = (received == Error::Ok) ? frame_size : rx_have_;
  cur.open = parts_possible && (cur.have >= cur.pos);
  cur.stop = parts_possible ? Error::Timeout : received;
  std::size_t offset = 0;
  Error first_error = Error::Ok;
  for (std::size_t k = 0; k < rq.Count(); ++k) {
    DeviceReply& reply = replies[k];
    reply.id = rq.IdAt(k);
    reply.result = cur.stop;
    reply.error_byte = 0;
    reply.alert = false;
    fastPart(rq, k, cur, out, offset, reply);
    offset += rq.LengthAt(k);
    if ((reply.result != Error::Ok) && (first_error == Error::Ok)) {
      first_error = reply.result;
    }
  }
  if (received == Error::Ok) {
    dropFront(frame_size);
  }
  return first_error;
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::multiItemBad(const detail::MultiRequest& rq,
                                               std::size_t k) const noexcept {
  const bool v2 = (config_.protocol == Protocol::V2);
  const std::size_t want = rq.LengthAt(k);
  const std::size_t addr = rq.AddressAt(k);
  const bool v1_limits = !v2 && ((addr > 255U) || (want > protocol::kMaxParamsV1));
  const bool too_big = v2 ? ((stuffedBound(want) + 11U) > FrameBytes) : ((want + 6U) > FrameBytes);
  return !validId(rq.IdAt(k), false) || (want == 0U) || v1_limits || too_big;
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::multiBad(const detail::MultiRequest& rq, std::span<uint8_t> out,
                                           std::span<DeviceReply> replies) const noexcept {
  const std::size_t count = rq.Count();
  if ((count == 0U) || (replies.size() != count)) {
    return true;
  }
  std::size_t total = 0;
  for (std::size_t k = 0; k < count; ++k) {
    if (multiItemBad(rq, k)) {
      return true;
    }
    for (std::size_t j = 0; rq.bulk && (j < k); ++j) {
      if (rq.IdAt(j) == rq.IdAt(k)) {
        return true; // each ID appears once in a Bulk Read
      }
    }
    total += rq.LengthAt(k);
  }
  return (out.size() != total) || (rq.fast && ((8U + total + (4U * count)) > FrameBytes));
}

template <UartPort Port, std::size_t FrameBytes>
protocol::Instruction DYNAMIXEL<Port, FrameBytes>::multiInstruction(bool fast, bool bulk) noexcept {
  if (bulk) {
    return fast ? protocol::Instruction::FastBulkRead : protocol::Instruction::BulkRead;
  }
  return fast ? protocol::Instruction::FastSyncRead : protocol::Instruction::SyncRead;
}

template <UartPort Port, std::size_t FrameBytes>
std::size_t DYNAMIXEL<Port, FrameBytes>::buildMultiRead(const detail::MultiRequest& rq) noexcept {
  const protocol::Instruction instruction = multiInstruction(rq.fast, rq.bulk);
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, kBroadcastId,
                               instruction};
  if (!rq.bulk) {
    writer.PutLe16(rq.address);
    writer.PutLe16(rq.length);
    writer.PutBytes(rq.ids);
  } else if (config_.protocol == Protocol::V2) {
    for (const BulkReadItem& item : rq.items) {
      writer.Put(item.id);
      writer.PutLe16(item.address);
      writer.PutLe16(item.length);
    }
  } else {
    writer.Put(0x00_u8);
    for (const BulkReadItem& item : rq.items) {
      writer.Put(static_cast<uint8_t>(item.length));
      writer.Put(item.id);
      writer.Put(static_cast<uint8_t>(item.address));
    }
  }
  return writer.Finish();
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::multiRead(const detail::MultiRequest& rq,
                                                    std::span<uint8_t> out,
                                                    std::span<DeviceReply> replies) noexcept {
  if (ready() != Error::Ok) {
    return Result<void>::Failure(Error::NotInitialized);
  }
  if (((config_.protocol != Protocol::V2) && (rq.fast || !rq.bulk)) ||
      (config_.status_return == StatusReturn::PingOnly)) {
    return Result<void>::Failure(Error::Unsupported);
  }
  if (multiBad(rq, out, replies)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  const std::size_t len = buildMultiRead(rq);
  if (len == 0U) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  for (std::size_t k = 0; k < rq.Count(); ++k) {
    replies[k] = DeviceReply{};
    replies[k].id = rq.IdAt(k);
  }
  const Error sent = sendFrame(len, false);
  if (sent != Error::Ok) {
    return Result<void>::Failure(sent);
  }
  const Error result = rq.fast ? collectFast(rq, out, replies) : collectSeparate(rq, out, replies);
  return (result == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(result);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SyncRead(uint16_t address, uint16_t length,
                                                   std::span<const uint8_t> ids,
                                                   std::span<uint8_t> out,
                                                   std::span<DeviceReply> replies) noexcept {
  return multiRead({false, false, address, length, ids, std::span<const BulkReadItem>{}}, out,
                   replies);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::FastSyncRead(uint16_t address, uint16_t length,
                                                       std::span<const uint8_t> ids,
                                                       std::span<uint8_t> out,
                                                       std::span<DeviceReply> replies) noexcept {
  return multiRead({true, false, address, length, ids, std::span<const BulkReadItem>{}}, out,
                   replies);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::BulkRead(std::span<const BulkReadItem> items,
                                                   std::span<uint8_t> out,
                                                   std::span<DeviceReply> replies) noexcept {
  return multiRead({false, true, 0U, 0U, std::span<const uint8_t>{}, items}, out, replies);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::FastBulkRead(std::span<const BulkReadItem> items,
                                                       std::span<uint8_t> out,
                                                       std::span<DeviceReply> replies) noexcept {
  return multiRead({true, true, 0U, 0U, std::span<const uint8_t>{}, items}, out, replies);
}

// ---------------------------------------------------------------------------------------------
// Typed access
// ---------------------------------------------------------------------------------------------

template <UartPort Port, std::size_t FrameBytes>
Result<int32_t> DYNAMIXEL<Port, FrameBytes>::ReadRegister(ServoRef servo, Reg reg) noexcept {
  const RegInfo* info = lookup(servo.family, reg);
  if (info == nullptr) {
    return Result<int32_t>::Failure(Error::Unsupported);
  }
  const std::span<uint8_t> raw{scratch_.data(), info->size};
  const Result<void> read = Read(servo.id, info->address, raw);
  if (!read.ok()) {
    return Result<int32_t>::Failure(read.error);
  }
  return Result<int32_t>::Success(protocol::DecodeValue(info->kind, raw));
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::WriteRegister(ServoRef servo, Reg reg,
                                                        int32_t value) noexcept {
  const RegInfo* info = lookup(servo.family, reg);
  if (info == nullptr) {
    return Result<void>::Failure(Error::Unsupported);
  }
  if (info->access == Access::ReadOnly) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  std::array<uint8_t, 4> raw{};
  if (!protocol::EncodeValue(info->kind, info->size, value, std::span<uint8_t, 4>{raw})) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  return Write(servo.id, info->address, std::span<const uint8_t>{raw.data(), info->size});
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SetTorque(ServoRef servo, bool enable) noexcept {
  return WriteRegister(servo, Reg::TorqueEnable, enable ? 1 : 0);
}

template <UartPort Port, std::size_t FrameBytes>
Error DYNAMIXEL<Port, FrameBytes>::stopOnce(uint16_t address, uint8_t id, bool answer) noexcept {
  protocol::FrameWriter writer{std::span<uint8_t>{tx_}, config_.protocol, id,
                               protocol::Instruction::Write};
  if (config_.protocol == Protocol::V2) {
    writer.PutLe16(address);
  } else {
    writer.Put(static_cast<uint8_t>(address));
  }
  writer.Put(0x00_u8);
  const std::size_t len = writer.Finish();
  if (len == 0U) {
    return Error::InvalidArgument; // the only way this helper reports it: nothing was sent
  }
  std::span<uint8_t> reply{};
  return transactOnce(len, id, answer, 0U, true, reply);
}

template <UartPort Port, std::size_t FrameBytes>
bool DYNAMIXEL<Port, FrameBytes>::noteStop(detail::StopTally& tally, Error attempt,
                                           bool answer) const noexcept {
  tally.last = attempt;
  tally.flush_fault = tally.flush_fault || flush_fault_;
  const bool carried_out = (attempt == Error::Ok) || (attempt == Error::DeviceFault);
  tally.sent = tally.sent || carried_out;
  // An alert in the answer: the write was carried out. With no answer expected, or one that came
  // after a failed flush, nothing confirms it: send again, the write is idempotent.
  tally.confirmed = carried_out && answer && !tally.flush_fault;
  // A refusal ends the attempts (repeating changes nothing), unless the flush failed: the answer
  // may be an old one.
  return (attempt != Error::DeviceError) || tally.flush_fault;
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::stopOutcome(const detail::StopTally& tally,
                                                      bool answer) noexcept {
  if (tally.confirmed) {
    return Result<void>::Success();
  }
  if (tally.flush_fault) {
    return fail(Error::Bus, "dynamixel: input flush failed during a stop; torque off was sent");
  }
  if (!answer && tally.sent) {
    return Result<void>::Success(); // no answer exists at this level: sent kStopAttempts times
  }
  return fail(tally.last, "dynamixel: torque off not confirmed");
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::DisableTorque(ServoRef servo) noexcept {
  const RegInfo* info = lookup(servo.family, Reg::TorqueEnable);
  if (info == nullptr) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  return DisableTorqueAt(servo.id, info->address);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::DisableTorqueAt(uint8_t id,
                                                          uint16_t torque_address) noexcept {
  if (!validId(id, true) || ((config_.protocol != Protocol::V2) && (torque_address > 255U))) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if (!startForStop()) {
    return fail(Error::Bus, "dynamixel: port start-up failed during a stop");
  }
  const bool answer = replyExpected(ReplyRule::WriteLevel, id);
  detail::StopTally tally{};
  for (uint32_t attempt = 0; (attempt < kStopAttempts) && !tally.confirmed; ++attempt) {
    const Error once = stopOnce(torque_address, id, answer);
    if (once == Error::InvalidArgument) {
      return Result<void>::Failure(Error::InvalidArgument);
    }
    if (!noteStop(tally, once, answer)) {
      break;
    }
  }
  return stopOutcome(tally, answer);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::DisableTorqueAll(Family family) noexcept {
  return DisableTorque(ServoRef{.id = kBroadcastId, .family = family});
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::DisableTorqueEach(
    std::span<const ServoRef> servos) noexcept {
  Error first = Error::Ok;
  for (const ServoRef& servo : servos) {
    const Result<void> done = DisableTorque(servo);
    if ((first == Error::Ok) && !done.ok()) {
      first = done.error;
    }
  }
  return (first == Error::Ok) ? Result<void>::Success() : Result<void>::Failure(first);
}

template <UartPort Port, std::size_t FrameBytes>
Result<Feedback> DYNAMIXEL<Port, FrameBytes>::ReadFeedback(ServoRef servo) noexcept {
  const RegInfo* position = lookup(servo.family, Reg::PresentPosition);
  const RegInfo* velocity = lookup(servo.family, Reg::PresentVelocity);
  const RegInfo* voltage = lookup(servo.family, Reg::PresentVoltage);
  const RegInfo* temperature = lookup(servo.family, Reg::PresentTemperature);
  const RegInfo* current = lookup(servo.family, Reg::PresentCurrent);
  const RegInfo* load = lookup(servo.family, Reg::PresentLoad);
  const RegInfo* effort = (current != nullptr) ? current : load;
  if ((position == nullptr) || (velocity == nullptr) || (voltage == nullptr) ||
      (temperature == nullptr)) {
    return Result<Feedback>::Failure(Error::Unsupported);
  }
  const std::array<const RegInfo*, 5> parts{position, velocity, voltage, temperature, effort};
  uint32_t first = 0xFFFFU;
  uint32_t last = 0U;
  for (const RegInfo* part : parts) {
    if (part != nullptr) {
      first = std::min(first, static_cast<uint32_t>(part->address));
      last = std::max(last, static_cast<uint32_t>(part->address) + part->size);
    }
  }
  if ((last - first) > kScratchBytes) {
    return Result<Feedback>::Failure(Error::Unsupported);
  }
  const std::span<uint8_t> block{scratch_.data(), last - first};
  const Result<void> read = Read(servo.id, static_cast<uint16_t>(first), block);
  if (!read.ok()) {
    return Result<Feedback>::Failure(read.error);
  }
  const auto decode = [&block, first](const RegInfo& reg) noexcept {
    return protocol::DecodeValue(reg.kind,
                                 std::span<const uint8_t>{block}.subspan(
                                     static_cast<std::size_t>(reg.address) - first, reg.size));
  };
  Feedback feedback{};
  feedback.position = decode(*position);
  feedback.velocity = decode(*velocity);
  feedback.voltage_dv = static_cast<uint16_t>(decode(*voltage));
  feedback.temperature_c = static_cast<uint8_t>(decode(*temperature));
  if (effort != nullptr) {
    feedback.effort = decode(*effort);
    feedback.effort_kind = (current != nullptr) ? Feedback::EffortKind::CurrentCounts
                                                : Feedback::EffortKind::LoadPermille;
  }
  return Result<Feedback>::Success(feedback);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SetGoalPosition(ServoRef servo, int32_t counts) noexcept {
  return WriteRegister(servo, Reg::GoalPosition, counts);
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SetId(ServoRef servo, uint8_t new_id) noexcept {
  if (!validId(new_id, false)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  if ((servo.id == kBroadcastId) && !config_.allow_destructive) {
    return fail(Error::NotPermitted,
                "dynamixel: an ID write to every servo needs allow_destructive");
  }
  // The e-Manual does not say which ID the status packet of this write carries: accept both.
  alt_id_ = new_id;
  const Result<void> written = WriteRegister(servo, Reg::Id, static_cast<int32_t>(new_id));
  alt_id_ = 0x100U;
  return written;
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SetServoBaudRate(ServoRef servo,
                                                           uint32_t bits_per_second) noexcept {
  uint8_t value = 0;
  if (!BaudValueFor(servo.family, bits_per_second, value)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  const Result<void> written = WriteRegister(servo, Reg::BaudRate, static_cast<int32_t>(value));
  if (written.ok()) {
    port_.DelayUs(config_.baud_change_wait_us);
  }
  return written;
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::ChangeBusBaud(uint32_t bits_per_second) noexcept {
  if constexpr (detail::HasSetBaudRate<Port>) {
    if (ready() != Error::Ok) {
      return Result<void>::Failure(Error::NotInitialized);
    }
    if ((bits_per_second < kMinBaud) || (bits_per_second > kMaxBaud)) {
      return Result<void>::Failure(Error::InvalidArgument);
    }
    if (!port_.SetBaudRate(bits_per_second)) {
      return fail(Error::Bus, "dynamixel: SetBaudRate failed");
    }
    config_.baud = bits_per_second;
    port_.DelayUs(config_.baud_change_wait_us);
    return Resync();
  } else {
    static_cast<void>(bits_per_second);
    return Result<void>::Failure(Error::Unsupported);
  }
}

template <UartPort Port, std::size_t FrameBytes>
Result<void> DYNAMIXEL<Port, FrameBytes>::SetStatusReturnLevel(ServoRef servo,
                                                               StatusReturn level) noexcept {
  if (!detail::IsEnumerator(level)) {
    return Result<void>::Failure(Error::InvalidArgument);
  }
  const StatusReturn before = config_.status_return;
  Result<void> written = WriteRegister(servo, Reg::StatusReturnLevel,
                                       static_cast<int32_t>(detail::ToUnderlying(level)));
  // Lowering the level may silence the answer to this very write: accept the silence, then adopt
  // the level.
  const bool silent_ok = (written.error == Error::Timeout) && (before == StatusReturn::All) &&
                         (level != StatusReturn::All);
  if (written.ok() || silent_ok) {
    if (silent_ok) {
      // A lost write and a lost answer look the same: the servo may still answer at the old level.
      detail::Log(port_, LogLevel::Warning,
                  "dynamixel: no answer to the level write; adopted, verify with a read");
    }
    config_.status_return = level;
    return Result<void>::Success();
  }
  return written;
}

} // namespace dynamixel

#endif // DYNAMIXEL_IPP
