/**
 * @file fuzz_dynamixel.cpp
 * @brief Fuzz target: the driver over a bus of simulated servos whose behaviour
 * the fuzzer controls.
 *
 * What the driver reads comes from the servos over the bus, so the reply bytes
 * are the untrusted input. The fuzzer's bytes choose a configuration (sometimes
 * invalid) and a protocol, a set of servos (X-series and AX-series), then a
 * sequence of calls covering the whole API. On every transfer they also choose
 * what the line does: pass it, fail it, cut the reply short, flip bits, replace
 * it with noise, or lose a request. They can also change the servos: silence
 * one, make it reject the next instruction, raise its alert, stick the line low
 * or high, or put the echo of a single-wire loopback on the receive side.
 *
 * After every call the target checks the driver's promises (docs/contract.md),
 * and aborts, naming the line, when one breaks:
 *
 * - a failed call returns an `Error` the enumeration defines, and `T{}` as its
 * value;
 * - before `EnsureInitialized()` succeeds, no call makes a transfer: a call
 * returns `NotInitialized`, or the argument or permission error it would return
 * anyway;
 * - `InvalidArgument`, `Unsupported` and `NotPermitted` mean nothing was sent;
 * - the safety rule: `DisableTorque`, `DisableTorqueAll`, `DisableTorqueEach`
 * and `Reboot` are never refused for the driver's state, a failing flush or
 * earlier failures: a call that is not an argument error writes at least once,
 * unless the port itself failed to start;
 * - no frame sent is longer than the transmit buffer, and every one is a
 * well-formed packet;
 * - a bulk read that returns `Ok` has `Ok` in every per-device entry;
 * - the statistics only grow, and every log line is a non-empty plain-text
 * message.
 *
 * Built two ways (tests/fuzz/CMakeLists.txt): with libFuzzer (Clang,
 * `-DDYNAMIXEL_BUILD_FUZZERS=ON`), and as a plain executable that replays the
 * corpus in tests/fuzz/corpus/ as a CTest on any compiler.
 */
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "dynamixel.hpp"
#include "mock_servo_bus.hpp" // the independent bus model of the host tests

namespace {

using dynamixel::Error;

/// The fuzzer's bytes, read front to back; zeros once they run out.
class Input {
public:
  Input(const uint8_t* data, std::size_t size) noexcept : data_(data), size_(size) {}

  [[nodiscard]] bool Empty() const noexcept {
    return pos_ >= size_;
  }
  uint8_t Byte() noexcept {
    return pos_ < size_ ? data_[pos_++] : uint8_t{0};
  }
  uint16_t Word() noexcept {
    const auto high = Byte();
    return static_cast<uint16_t>((static_cast<uint32_t>(high) << 8U) | Byte());
  }

private:
  const uint8_t* data_;
  std::size_t size_;
  std::size_t pos_ = 0;
};

/// A broken promise: name the line and abort, so libFuzzer saves the input as a
/// crash.
void Check(bool promise, int line = __builtin_LINE()) noexcept {
  if (!promise) {
    (void)std::fprintf(stderr, "fuzz_dynamixel: promise broken at line %d\n", line);
    std::abort();
  }
}

/// What the line does with one transfer, chosen by the fuzzer; any other value
/// passes it.
constexpr uint8_t kFail = 0xFF;    ///< The transfer fails (a hook returns `false`).
constexpr uint8_t kCut = 0xFE;     ///< Data read: only the first byte arrives. A request: lost.
constexpr uint8_t kFlip = 0xFD;    ///< Data read: bits flipped (noise).
constexpr uint8_t kReplace = 0xFC; ///< Data read: replaced (another device spoke).

/// The port: the servo bus of the host tests behind a line the fuzzer controls.
class FuzzPort : public dynamixel::UartInterface<FuzzPort> {
public:
  FuzzPort(Input& in, mock::Bus& bus) noexcept : in_(in), bus_(bus) {}

  bool EnsureInitialized() noexcept {
    const bool up = nextAction() != kFail;
    ensure_failures_ += up ? 0 : 1;
    return up;
  }

  bool Write(const uint8_t* data, std::size_t len) noexcept {
    Check(data != nullptr && len >= 6 && len <= kTxMax);
    ++writes_;
    const uint8_t action = nextAction();
    if (action == kFail) {
      return false;
    }
    if (action == kCut) {
      return true; // lost on the line: no servo sees it
    }
    bus_.Transmit(data, len);
    return true;
  }

  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    Check(data != nullptr && max >= 1 && timeout_us > 0);
    const uint8_t action = nextAction();
    if (action == kFail) {
      return 0;
    }
    std::size_t n = bus_.Queued() < max ? bus_.Queued() : max;
    if (action == kCut && n > 1) {
      n = 1;
    }
    for (std::size_t i = 0; i < n; ++i) {
      data[i] = bus_.rx[bus_.rx_head++];
      if (action == kFlip) {
        data[i] = static_cast<uint8_t>(data[i] ^ in_.Byte());
      } else if (action == kReplace) {
        data[i] = in_.Byte();
      } else {
        // The byte passes unchanged.
      }
    }
    return n;
  }

  bool FlushInput() noexcept {
    bus_.Clear();
    return nextAction() != kFail;
  }

  void DelayUs(uint32_t /*us*/) noexcept {}

  bool SetBaudRate(uint32_t baud) noexcept {
    Check(baud > 0);
    return nextAction() != kFail;
  }

  void Log(dynamixel::LogLevel /*level*/, const char* message) noexcept {
    Check(message != nullptr && message[0] != '\0');
  }

  [[nodiscard]] int Writes() const noexcept {
    return writes_;
  }

  /// How many times the port reported that it could not start.
  [[nodiscard]] int EnsureFailures() const noexcept {
    return ensure_failures_;
  }

  /// The largest frame the driver may send: its transmit buffer.
  static constexpr std::size_t kTxMax = 512;

private:
  uint8_t nextAction() noexcept {
    return in_.Byte();
  }

  Input& in_;
  mock::Bus& bus_;
  int writes_ = 0;
  int ensure_failures_ = 0;
};

using Driver = dynamixel::DYNAMIXEL<FuzzPort>;

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

/// A configuration from the fuzzer's bytes; sometimes invalid on purpose.
dynamixel::Config FuzzConfig(Input& in) noexcept {
  dynamixel::Config c{};
  const uint8_t flags = in.Byte();
  c.protocol = (flags & 0x01U) != 0U ? dynamixel::Protocol::V1 : dynamixel::Protocol::V2;
  c.echo = (flags & 0x02U) != 0U ? dynamixel::EchoMode::Loopback : dynamixel::EchoMode::None;
  c.alert_policy =
      (flags & 0x04U) != 0U ? dynamixel::AlertPolicy::Fail : dynamixel::AlertPolicy::Report;
  c.allow_destructive = (flags & 0x08U) != 0U;
  c.status_return = static_cast<dynamixel::StatusReturn>(in.Byte() % 3U);
  c.retries = static_cast<uint8_t>(in.Byte() % 5U); // 4 is past the largest accepted value
  c.baud = (flags & 0x10U) != 0U ? 0U : 1'000'000U;
  // Short timeouts and waits: the model answers at once, so a time-out only
  // counts calls.
  c.reply_timeout_us =
      (flags & 0x20U) != 0U ? uint32_t{in.Byte()} : 10'000U; // 0 to 255: sometimes invalid
  c.gap_timeout_us = 2'000;
  c.reboot_wait_us = 1'000;
  return c;
}

/// The servo set: up to four servos of either kind at fuzzer chosen IDs.
void FuzzServos(mock::Bus& bus, Input& in, bool v2) noexcept {
  bus.v2 = v2;
  const auto count = static_cast<uint8_t>(in.Byte() % 5U);
  for (uint8_t i = 0; i < count; ++i) {
    const auto id = static_cast<uint8_t>(1U + (in.Byte() % 6U));
    if (bus.Find(id) == nullptr) {
      static_cast<void>(bus.Add(v2 ? mock::Kind::Xm430 : mock::Kind::Ax12, id));
    }
  }
}

/// Change the servos or the line between two calls.
void FuzzBus(mock::Bus& bus, Input& in) noexcept {
  mock::Servo& s = bus.servos[in.Byte() % bus.servos.size()];
  switch (in.Byte() % 12U) {
  case 0:
    s.silent = !s.silent;
    break;
  case 1:
    s.force_error = in.Byte();
    break;
  case 2:
    s.alert = !s.alert;
    break;
  case 3:
    bus.knob.drop_replies = static_cast<int>(in.Byte() % 3U);
    break;
  case 4:
    bus.knob.corrupt_crc = static_cast<int>(in.Byte() % 3U);
    break;
  case 5:
    bus.knob.truncate_to = static_cast<int>(in.Byte() % 14U) - 1;
    break;
  case 6:
    bus.knob.garbage_prefix = static_cast<int>(in.Byte() % 8U);
    bus.knob.trailing_junk = static_cast<int>(in.Byte() % 8U);
    break;
  case 7:
    bus.knob.stuck = static_cast<int>(in.Byte() % 3U);
    break;
  case 8:
    bus.knob.wrong_id = static_cast<int>(in.Byte() % 3U);
    break;
  case 9:
    bus.knob.split_chunk = static_cast<int>(in.Byte() % 5U);
    break;
  case 10:
    bus.knob.echo = !bus.knob.echo;
    bus.knob.echo_corrupt = static_cast<int>(in.Byte() % 2U);
    bus.knob.echo_drop = static_cast<int>(in.Byte() % 2U);
    break;
  default:
    bus.knob.bad_stuffing = static_cast<int>(in.Byte() % 2U);
    bus.knob.extra_param = static_cast<int>(in.Byte() % 2U);
    break;
  }
}

/// IDs and data buffers for the multi-device calls, drawn from the fuzzer's
/// bytes.
struct Batch {
  std::array<uint8_t, 6> ids{};
  std::array<uint8_t, 48> data{}; // 6 devices of 8 bytes
  std::array<uint8_t, 48> out{};
  std::array<dynamixel::DeviceReply, 6> replies{};
  std::array<dynamixel::BulkReadItem, 6> reads{};
  std::array<dynamixel::BulkWriteItem, 6> writes{};
  std::size_t count = 0;
  uint16_t address = 0;
  uint16_t length = 0;
};

Batch FuzzBatch(Input& in) noexcept {
  Batch b{};
  b.count = in.Byte() % 7U;
  b.address = static_cast<uint16_t>(((in.Byte() % 3U) == 0U) ? (in.Word() % 2000U)
                                                             : (24U + (in.Byte() % 8U)));
  b.length = static_cast<uint16_t>(in.Byte() % 9U);
  for (std::size_t i = 0; i < b.count; ++i) {
    b.ids[i] = static_cast<uint8_t>(in.Byte() % 8U);
    b.reads[i] = {b.ids[i], b.address, static_cast<uint16_t>(in.Byte() % 9U)};
    b.writes[i] = {b.ids[i], b.address, std::span<const uint8_t>(b.data.data(), in.Byte() % 5U)};
  }
  for (auto& d : b.data) {
    d = in.Byte();
  }
  return b;
}

/// Total length of the reads of a bulk request.
std::size_t BulkBytes(const Batch& b) noexcept {
  std::size_t total = 0;
  for (std::size_t i = 0; i < b.count; ++i) {
    total += b.reads[i].length;
  }
  return total;
}

/// The promises of any call.
void CheckCall(const Driver& d, Error error, bool was_up, int sent) noexcept {
  Check(KnownError(error));
  if (!was_up) {
    // An argument or permission the call refuses before it needs the bus is
    // refused first; nothing is sent.
    Check((error == Error::NotInitialized || error == Error::InvalidArgument ||
           error == Error::Unsupported || error == Error::NotPermitted) &&
          sent == 0 && !d.IsInitialized());
  }
  if (error == Error::InvalidArgument || error == Error::Unsupported ||
      error == Error::NotPermitted || error == Error::NotInitialized) {
    Check(sent == 0);
  }
}

/// The safety rule of a stop or a reboot: refused only for an argument the protocol does not have
/// (nothing sent) or a port that cannot start; otherwise at least one packet goes out, whatever
/// the initialisation, flush, read and earlier failures were. `list` is a call for several servos,
/// which reports the first failure although a later servo was served.
void CheckStop(Error error, int sent, int ensure_failures, bool list = false) noexcept {
  Check(KnownError(error));
  Check(error != Error::NotInitialized && error != Error::NotPermitted &&
        error != Error::Unsupported);
  if (error == Error::InvalidArgument) {
    Check(list || sent == 0);
  } else if (ensure_failures == 0) {
    Check(sent >= 1);
  } else {
    // The port itself could not start for at least one servo: that is the one allowed refusal.
  }
}

/// A failed call returns `T{}`.
void CheckValue(const dynamixel::Result<int32_t>& r) noexcept {
  Check(r.ok() || r.value == 0);
}

void CheckValue(const dynamixel::Result<dynamixel::PingInfo>& r) noexcept {
  Check(r.ok() || (r.value.id == 0 && r.value.model == 0 && r.value.firmware == 0));
}

/// A multi-device read: an overall `Ok` means every device answered.
void CheckMulti(const Driver& d, const dynamixel::Result<void>& r, const Batch& b, bool was_up,
                int sent) noexcept {
  CheckCall(d, r.error, was_up, sent);
  if (r.ok()) {
    for (std::size_t i = 0; i < b.count; ++i) {
      Check(b.replies[i].result == Error::Ok);
    }
  }
}

void RunOne(Driver& d, FuzzPort& port, mock::Bus& bus, Input& in) noexcept {
  const uint8_t op = in.Byte() % 34U;
  const bool was_up = d.IsInitialized();
  const int before = port.Writes();
  const int ensure_before = port.EnsureFailures();
  const auto sent = [&] { return port.Writes() - before; };
  const auto ensure_failed = [&] { return port.EnsureFailures() - ensure_before; };
  const uint8_t id = in.Byte() % 8U;
  const dynamixel::Family family =
      in.Byte() % 2U == 0U ? dynamixel::Family::X430 : dynamixel::Family::AxRxDx;
  const dynamixel::ServoRef servo{id, family};
  const uint16_t word = in.Word();
  uint32_t before_tx = d.GetStats().transactions;

  switch (op) {
  case 0:
    Check(KnownError(d.EnsureInitialized().error));
    break;
  case 1:
    Check(d.Deinitialize().ok() && !d.IsInitialized());
    break;
  case 2:
    CheckCall(d, d.Resync().error, was_up, sent());
    break;
  case 3: {
    const auto r = d.Ping(id);
    CheckCall(d, r.error, was_up, sent());
    CheckValue(r);
    break;
  }
  case 4: {
    const auto r = d.Identify(id);
    // Protocol 1.0 pings first and may answer `Unsupported` after it (status
    // return level ping only).
    CheckCall(d, r.error == Error::Unsupported && sent() > 0 ? Error::Ok : r.error, was_up, sent());
    CheckValue(r);
    break;
  }
  case 5: {
    std::array<dynamixel::PingInfo, 4> found{};
    const auto r = d.BroadcastPing(found);
    CheckCall(d, r.error, was_up, sent());
    Check(!r || r.value <= found.size());
    break;
  }
  case 6: {
    std::array<dynamixel::PingInfo, 4> found{};
    const auto r = d.ScanIds(id, static_cast<uint8_t>(id + (in.Byte() % 4U)), found);
    CheckCall(d, r.error, was_up, sent());
    Check(!r || r.value <= found.size());
    break;
  }
  case 7: {
    std::array<uint8_t, 16> out{};
    const std::size_t n = in.Byte() % 17U;
    CheckCall(d, d.Read(id, word % 1100U, std::span<uint8_t>(out.data(), n)).error, was_up, sent());
    break;
  }
  case 8: {
    std::array<uint8_t, 16> data{};
    for (auto& x : data) {
      x = in.Byte();
    }
    CheckCall(
        d, d.Write(id, word % 1100U, std::span<const uint8_t>(data.data(), in.Byte() % 17U)).error,
        was_up, sent());
    break;
  }
  case 9: {
    std::array<uint8_t, 4> data{in.Byte(), in.Byte(), in.Byte(), in.Byte()};
    CheckCall(
        d,
        d.RegWrite(id, word % 1100U, std::span<const uint8_t>(data.data(), 1U + (in.Byte() % 4U)))
            .error,
        was_up, sent());
    break;
  }
  case 10:
    CheckCall(d, d.Action(id).error, was_up, sent());
    break;
  case 11:
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    CheckCall(d, d.FactoryReset(id, static_cast<dynamixel::ResetOption>(in.Byte() % 4U)).error,
              was_up, sent());
    break;
  case 12:
    CheckStop(d.Reboot(id).error, sent(), ensure_failed());
    break;
  case 13:
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    CheckCall(d, d.Clear(id, static_cast<dynamixel::ClearOption>(in.Byte() % 4U)).error, was_up,
              sent());
    break;
  case 14:
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    CheckCall(d, d.Backup(id, static_cast<dynamixel::BackupOption>(in.Byte() % 4U)).error, was_up,
              sent());
    break;
  case 15: {
    const Batch b = FuzzBatch(in);
    CheckCall(
        d,
        d.SyncWrite(b.address, b.length, std::span<const uint8_t>(b.ids.data(), b.count), b.data)
            .error,
        was_up, sent());
    break;
  }
  case 16: {
    const Batch b = FuzzBatch(in);
    CheckCall(
        d, d.BulkWrite(std::span<const dynamixel::BulkWriteItem>(b.writes.data(), b.count)).error,
        was_up, sent());
    break;
  }
  case 17: {
    Batch b = FuzzBatch(in);
    const auto r = d.SyncRead(b.address, b.length, std::span<const uint8_t>(b.ids.data(), b.count),
                              b.out, std::span<dynamixel::DeviceReply>(b.replies.data(), b.count));
    CheckMulti(d, r, b, was_up, sent());
    break;
  }
  case 18: {
    Batch b = FuzzBatch(in);
    const auto r =
        d.FastSyncRead(b.address, b.length, std::span<const uint8_t>(b.ids.data(), b.count), b.out,
                       std::span<dynamixel::DeviceReply>(b.replies.data(), b.count));
    CheckMulti(d, r, b, was_up, sent());
    break;
  }
  case 19: {
    Batch b = FuzzBatch(in);
    const auto r = d.BulkRead(
        std::span<const dynamixel::BulkReadItem>(b.reads.data(), b.count),
        std::span<uint8_t>(b.out.data(), BulkBytes(b) <= b.out.size() ? BulkBytes(b) : 0U),
        std::span<dynamixel::DeviceReply>(b.replies.data(), b.count));
    CheckMulti(d, r, b, was_up, sent());
    break;
  }
  case 20: {
    Batch b = FuzzBatch(in);
    const auto r = d.FastBulkRead(
        std::span<const dynamixel::BulkReadItem>(b.reads.data(), b.count),
        std::span<uint8_t>(b.out.data(), BulkBytes(b) <= b.out.size() ? BulkBytes(b) : 0U),
        std::span<dynamixel::DeviceReply>(b.replies.data(), b.count));
    CheckMulti(d, r, b, was_up, sent());
    break;
  }
  case 21: {
    const auto r = d.ReadRegister(servo, static_cast<dynamixel::Reg>(word % 90U));
    CheckCall(d, r.error, was_up, sent());
    CheckValue(r);
    break;
  }
  case 22:
    CheckCall(d,
              d.WriteRegister(servo, static_cast<dynamixel::Reg>(word % 90U),
                              static_cast<int32_t>(static_cast<int16_t>(in.Word())) * 3)
                  .error,
              was_up, sent());
    break;
  case 23:
    CheckCall(d, d.SetTorque(servo, (word & 1U) != 0U).error, was_up, sent());
    break;
  case 24: {
    const auto r = d.ReadFeedback(servo);
    CheckCall(d, r.error, was_up, sent());
    break;
  }
  case 25:
    CheckCall(d,
              d.SetGoalPosition(servo, static_cast<int32_t>(static_cast<int16_t>(word)) * 2).error,
              was_up, sent());
    break;
  case 26:
    CheckCall(d, d.SetId(servo, static_cast<uint8_t>(in.Byte() % 8U)).error, was_up, sent());
    break;
  case 27:
    CheckCall(d, d.SetServoBaudRate(servo, (word & 1U) != 0U ? 1'000'000U : word * 100U).error,
              was_up, sent());
    break;
  case 28:
    CheckCall(d,
              d.SetStatusReturnLevel(servo, static_cast<dynamixel::StatusReturn>(word % 4U)).error,
              was_up, sent());
    break;
  case 29:
    CheckStop(d.DisableTorque(servo).error, sent(), ensure_failed());
    break;
  case 30:
    CheckStop(d.DisableTorqueAll(family).error, sent(), ensure_failed());
    break;
  case 31: {
    const std::array<dynamixel::ServoRef, 2> two{
        servo, dynamixel::ServoRef{static_cast<uint8_t>(id + 1U), family}};
    CheckStop(d.DisableTorqueEach(two).error, sent(), ensure_failed(), true);
    break;
  }
  case 32:
    CheckStop(d.DisableTorqueAt(id, static_cast<uint16_t>(word % 300U)).error, sent(),
              ensure_failed());
    break;
  default:
    FuzzBus(bus, in);
    break;
  }
  Check(d.GetStats().transactions >= before_tx);
  Check(d.GetLastRequest().size() <= FuzzPort::kTxMax);
}

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  Input in(data, size);
  static mock::Bus bus;
  bus = mock::Bus{};
  const dynamixel::Config config = FuzzConfig(in);
  FuzzServos(bus, in, config.protocol == dynamixel::Protocol::V2);
  FuzzPort port(in, bus);
  Driver driver(port, config);
  for (int calls = 0; calls < 96 && !in.Empty(); ++calls) {
    RunOne(driver, port, bus, in);
  }
  return 0;
}
