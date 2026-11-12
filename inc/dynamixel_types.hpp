/**
 * @file dynamixel_types.hpp
 * @brief Public types of the DYNAMIXEL driver: errors, `Result<T>`, protocol enumerations, replies,
 * statistics and the configuration.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * Everything here is plain data with no platform dependency, so handlers, tests and tools can
 * include it without the driver template. The configuration defaults follow the ROBOTIS e-Manual
 * (protocol 2.0 at 57 600 bit/s, status packets for every instruction) and the timing values that
 * the manual does not give are documented knobs (docs/configuration.md).
 */
#ifndef DYNAMIXEL_TYPES_HPP
#define DYNAMIXEL_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <span>

#include "dynamixel_detail.hpp"
#include "dynamixel_tables.hpp"
#include "dynamixel_version.h"

/** @brief The DYNAMIXEL driver: everything it declares lives here; internals in `detail`. */
namespace dynamixel {

/**
 * @defgroup dynamixel_types Driver types
 * @brief Errors, results, replies, statistics and configuration.
 * @{
 */

/**
 * @brief Why a call failed. Every call of the driver returns one of these in its `Result`.
 *
 * Values 0 to 9 are the shared base set; bus-specific values start at 16. `[[nodiscard]]` on
 * the type: a function that returns a bare `Error` cannot have it ignored either (KB-CPP20-037).
 */
enum class [[nodiscard]] Error : uint8_t {
  // Base set: the same names and values in every driver of the org (standards/driver.md).
  Ok = 0,              ///< The call succeeded.
  NotInitialized = 1,  ///< `EnsureInitialized()` has not succeeded, or `Deinitialize()` ran since.
  InvalidArgument = 2, ///< An argument or a `Config` field is out of range; nothing was sent.
  Bus = 3,             ///< A port hook returned `false`.
  Timeout = 4,         ///< No complete reply within the deadline (nothing, or a truncated packet).
  DeviceIdMismatch =
      5,    ///< The reply came from another ID than the one addressed, or the model differs.
  Crc = 6,  ///< The CRC-16 (protocol 2.0) or checksum (protocol 1.0) of a reply did not match.
  Busy = 7, ///< Reserved by the shared set; this driver has no pending state.
  DeviceFault = 8, ///< The servo reported a hardware alert and `AlertPolicy::Fail` is set.
  Unsupported = 9, ///< The hook, protocol or family does not offer this call.
  // Bus-specific values start at 16; each one is returned by some call (tests/test_faults.cpp).
  DeviceError =
      16, ///< The servo rejected the instruction: see `GetLastReply()` for the error number.
  InvalidReply =
      17, ///< A reply the protocol refuses: header, length, instruction or parameter count.
  EchoMismatch = 18, ///< The transmitted bytes were not read back as sent (collision or wiring).
  LineFault = 19, ///< The line carried only 0x00 or only 0xFF: stuck low, stuck high or floating.
  NotPermitted = 22, ///< A destructive instruction without `Config::allow_destructive`.
};

/**
 * @brief The name of an error, for logs.
 * @param error The error.
 * @return A static string such as `"Timeout"`; `"Unknown"` for a value that is not an enumerator.
 */
constexpr const char* ToString(Error error) noexcept {
  switch (error) {
  case Error::Ok:
    return "Ok";
  case Error::NotInitialized:
    return "NotInitialized";
  case Error::InvalidArgument:
    return "InvalidArgument";
  case Error::Bus:
    return "Bus";
  case Error::Timeout:
    return "Timeout";
  case Error::DeviceIdMismatch:
    return "DeviceIdMismatch";
  case Error::Crc:
    return "Crc";
  case Error::Busy:
    return "Busy";
  case Error::DeviceFault:
    return "DeviceFault";
  case Error::Unsupported:
    return "Unsupported";
  case Error::DeviceError:
    return "DeviceError";
  case Error::InvalidReply:
    return "InvalidReply";
  case Error::EchoMismatch:
    return "EchoMismatch";
  case Error::LineFault:
    return "LineFault";
  case Error::NotPermitted:
    return "NotPermitted";
  default:
    break;
  }
  return "Unknown";
}

/**
 * @brief The value of a call, or why there is none.
 *
 * Check `ok()` (or the `bool` conversion) before reading `value`; on failure `value` is `T{}`.
 *
 * @tparam T The value type. `Result<void>` carries only the error.
 */
template <typename T>
struct [[nodiscard]] Result {
  T value{};              ///< The value; `T{}` when the call failed.
  Error error{Error::Ok}; ///< `Error::Ok` on success.

  /** @return true when the call succeeded. */
  [[nodiscard]] constexpr bool ok() const noexcept {
    return error == Error::Ok;
  }
  /** @return true when the call succeeded. */
  constexpr explicit operator bool() const noexcept {
    return ok();
  }

  /**
   * @brief A successful result.
   * @param v The value.
   * @return `{v, Error::Ok}`.
   */
  static constexpr Result Success(T v) noexcept {
    return {v, Error::Ok};
  }
  /**
   * @brief A failed result.
   * @param e Why it failed; never `Error::Ok`.
   * @return `{T{}, e}`.
   */
  static constexpr Result Failure(Error e) noexcept {
    return {T{}, e};
  }
};

/** @brief The result of a call that returns no value. */
template <>
struct [[nodiscard]] Result<void> {
  Error error{Error::Ok}; ///< `Error::Ok` on success.

  /** @return true when the call succeeded. */
  [[nodiscard]] constexpr bool ok() const noexcept {
    return error == Error::Ok;
  }
  /** @return true when the call succeeded. */
  constexpr explicit operator bool() const noexcept {
    return ok();
  }

  /** @return A successful result. */
  static constexpr Result Success() noexcept {
    return {Error::Ok};
  }
  /**
   * @brief A failed result.
   * @param e Why it failed; never `Error::Ok`.
   * @return `{e}`.
   */
  static constexpr Result Failure(Error e) noexcept {
    return {e};
  }
};

/** @brief Severity passed to the optional `Log()` hook. */
enum class LogLevel : uint8_t {
  Error = 0, ///< A call failed.
  Warning,   ///< Something unexpected that the driver recovered from.
  Info,      ///< Lifecycle events: initialised, resynchronised.
  Debug,     ///< Detail for bring-up.
};

/** @brief The packet format of the bus: one per `DYNAMIXEL` object, because every servo on a bus
 * shares it. */
enum class Protocol : uint8_t {
  V1 = 0, ///< Protocol 1.0: header `FF FF`, one-byte length, additive checksum, error flags.
  V2 = 1, ///< Protocol 2.0: header `FF FF FD 00`, two-byte length, CRC-16, byte stuffing, error
          ///< number.
};

/** @brief Whether the transmitted bytes come back on the receive line (a single-wire loopback). */
enum class EchoMode : uint8_t {
  None = 0,     ///< Separate lines or a direction-controlled transceiver: nothing is echoed.
  Loopback = 1, ///< TX and RX are tied: every sent byte is read back and compared.
};

/** @brief What a reply with a hardware alert means for the call. */
enum class AlertPolicy : uint8_t {
  Report = 0, ///< The call succeeds, the data is returned and the alert is latched
              ///< (`GetLatchedAlert()`).
  Fail = 1,   ///< The call returns `DeviceFault` and discards the data; the alert is latched too.
};

/** @brief How much a servo answers (the Status Return Level register, values 0 to 2). */
enum class StatusReturn : uint8_t {
  PingOnly = 0, ///< Only ping instructions are answered.
  ReadOnly = 1, ///< Ping and read instructions are answered.
  All = 2,      ///< Every instruction addressed to one servo is answered. Factory default.
};

/** @brief The option byte of a Factory Reset instruction (protocol 2.0). */
enum class ResetOption : uint8_t {
  KeepIdAndBaud = 0x02, ///< Reset all except ID and baud rate.
  KeepId = 0x01,        ///< Reset all except ID.
  All = 0xFF,           ///< Reset all; the servo returns to ID 1 and 57 600 bit/s.
};

/** @brief The option of a Clear instruction (protocol 2.0). */
enum class ClearOption : uint8_t {
  MultiTurn = 0x01, ///< Reset the present position to one turn (0 to 4095); only while stopped.
  Errors = 0x02,    ///< Clear registered errors (Y series only in the e-Manual).
};

/** @brief The option of a Control Table Backup instruction (protocol 2.0). */
enum class BackupOption : uint8_t {
  Store = 0x01,   ///< Store the control table in the backup area (torque must be off).
  Restore = 0x02, ///< Restore the EEPROM from the backup; the servo reboots.
};

/** @brief Protocol 2.0 error numbers (bits 6..0 of the error byte of a status packet). */
enum class [[nodiscard]] Protocol2Error : uint8_t {
  None = 0,             ///< No error.
  ResultFail = 1,       ///< The instruction could not be processed.
  InstructionError = 2, ///< Undefined instruction, or Action without a Reg Write.
  CrcError = 3,         ///< The servo's CRC check of the instruction failed.
  DataRangeError = 4,   ///< The value is outside the register's range.
  DataLengthError = 5,  ///< The data is shorter than the register.
  DataLimitError = 6,   ///< The value is outside a configured limit.
  AccessError =
      7, ///< Read-only, write-only or undefined address, or an EEPROM write with torque on.
};

/** @brief Protocol 1.0 error flags (the error byte of a status packet). */
namespace p1_error {
inline constexpr uint8_t kInputVoltage =
    0x01; ///< Input voltage out of the configured range (condition).
inline constexpr uint8_t kAngleLimit =
    0x02; ///< Goal position outside the angle limits (command fault).
inline constexpr uint8_t kOverheating = 0x04; ///< Internal temperature above the limit (condition).
inline constexpr uint8_t kRange = 0x08; ///< An instruction value out of range (command fault).
inline constexpr uint8_t kChecksum =
    0x10; ///< The instruction's checksum was wrong (command fault).
inline constexpr uint8_t kOverload =
    0x20; ///< Load beyond what the torque limit controls (condition).
inline constexpr uint8_t kInstruction =
    0x40; ///< Undefined instruction, or Action without Reg Write (command fault).
/// Flags that mean the instruction was not carried out.
inline constexpr uint8_t kCommandFaults = static_cast<uint8_t>(
    uint32_t{kAngleLimit} | uint32_t{kRange} | uint32_t{kChecksum} | uint32_t{kInstruction});
/// Flags that report the servo's condition, not the instruction.
inline constexpr uint8_t kConditions =
    static_cast<uint8_t>(uint32_t{kInputVoltage} | uint32_t{kOverheating} | uint32_t{kOverload});
} // namespace p1_error

/** @brief Protocol 2.0 alert bit of the error byte. */
inline constexpr uint8_t kAlertBit = 0x80;

/** @brief Bits of the X-series Hardware Error Status register (address 70) and the Shutdown
 * register. */
namespace hw_error {
inline constexpr uint8_t kInputVoltage = 0x01; ///< Input voltage out of range.
inline constexpr uint8_t kOverheating = 0x04;  ///< Temperature above the limit.
inline constexpr uint8_t kMotorEncoder = 0x08; ///< Motor encoder malfunction (X430, X540, MX(2.0)).
inline constexpr uint8_t kElectricalShock = 0x10; ///< Electrical shock or too little power.
inline constexpr uint8_t kOverload = 0x20;        ///< Persistent load above the maximum output.
} // namespace hw_error

/**
 * @brief An error byte, decoded.
 *
 * Protocol 2.0: bit 7 is the alert, bits 6..0 the error number. Protocol 1.0: every bit is a flag;
 * the command faults (`p1_error::kCommandFaults`) mean the instruction was not carried out, the
 * conditions (`p1_error::kConditions`) report the servo's state next to valid data.
 */
struct ReplyInfo {
  uint8_t id{0};         ///< The ID the reply came from; the first device for a broadcast ping.
  uint8_t error_byte{0}; ///< The raw error byte of the status packet.
  bool alert{false};     ///< Protocol 2.0 alert bit, or a protocol 1.0 condition flag.
  bool rejected{false};  ///< The servo did not carry out the instruction (an error number, or a
                         ///< command fault).
};

/** @brief The identity a ping returns. */
struct PingInfo {
  uint8_t id{0};       ///< The ID that answered.
  uint16_t model{0};   ///< Model Number register.
  uint8_t firmware{0}; ///< Firmware Version register.
  Family family{
      Family::Generic}; ///< The control table family of `model` (`Generic` when unlisted).
};

/** @brief A servo on the bus: its ID and its control-table family. */
struct ServoRef {
  uint8_t id{1}; ///< 0 to 252 (protocol 2.0) or 253 (protocol 1.0); 254 is the broadcast ID.
  Family family{
      Family::Generic}; ///< Selects the typed registers; `Generic` allows raw addresses only.
};

/** @brief The per-device outcome of a Sync Read, Bulk Read or Fast read. */
struct DeviceReply {
  uint8_t id{0};                ///< The requested ID.
  Error result{Error::Timeout}; ///< `Ok`, or why this device's data is not valid.
  uint8_t error_byte{0};        ///< The raw error byte when a packet arrived.
  bool alert{false};            ///< The alert or condition flag of this device's error byte.
};

/** @brief One device of a Bulk Read request. */
struct BulkReadItem {
  uint8_t id{0};       ///< Device ID; each ID appears at most once.
  uint16_t address{0}; ///< First control-table address.
  uint16_t length{0};  ///< Bytes to read.
};

/** @brief One device of a Bulk Write request. */
struct BulkWriteItem {
  uint8_t id{0};                 ///< Device ID; each ID appears at most once.
  uint16_t address{0};           ///< First control-table address.
  std::span<const uint8_t> data; ///< The bytes to write there (at least one).
};

/** @brief Position, velocity, effort, voltage and temperature of a servo, read in one transfer. */
struct Feedback {
  /** @brief What `effort` holds. */
  enum class EffortKind : uint8_t {
    None = 0, ///< The family has neither load nor current.
    LoadPermille =
        1, ///< Present Load in 0.1 percent of the maximum torque (signed: negative is clockwise).
    CurrentCounts = 2, ///< Present Current in the model's unit (`ModelInfo::current_ua`).
  };
  int32_t position{0}; ///< Present Position in counts.
  int32_t velocity{0}; ///< Present Velocity (or Speed) in counts (`Info(family).mrpm_per_count`).
  int32_t effort{0};   ///< Present Load or Present Current, see `effort_kind`.
  EffortKind effort_kind{EffortKind::None}; ///< Meaning of `effort`.
  uint16_t voltage_dv{0};                   ///< Input voltage in 0.1 V.
  uint8_t temperature_c{0};                 ///< Internal temperature in degrees Celsius.
};

/** @brief What the driver counted since `ResetStats()`. */
struct Stats {
  uint32_t transactions{0};  ///< Instruction packets sent.
  uint32_t retries{0};       ///< Repeats of an idempotent instruction after a bad or missing reply.
  uint32_t timeouts{0};      ///< Replies that did not arrive in time.
  uint32_t crc_errors{0};    ///< Packets dropped for a wrong CRC or checksum.
  uint32_t garbage_bytes{0}; ///< Received bytes skipped while looking for a header.
  uint32_t line_faults{0};   ///< Stuck or floating line detections.
  uint32_t echo_errors{0};   ///< Loopback echoes that differed.
  uint32_t device_errors{0}; ///< Status packets with a rejecting error byte.
  uint32_t alerts{0};        ///< Status packets with an alert or condition flag.
};

/** @brief The broadcast ID: an instruction to it reaches every servo and none answers. */
inline constexpr uint8_t kBroadcastId = 0xFE;
/** @brief The largest valid servo ID of protocol 2.0. */
inline constexpr uint8_t kMaxIdV2 = 252;
/** @brief The largest valid servo ID of protocol 1.0. */
inline constexpr uint8_t kMaxIdV1 = 253;

/** @brief Shortest and longest reply wait the driver accepts, microseconds (KB-CPP20-058). */
inline constexpr uint32_t kMinTimeoutUs = 100;
/** @brief Longest timeout the driver accepts: one second. */
inline constexpr uint32_t kMaxTimeoutUs = 1'000'000;
/** @brief Longest settle time after a reboot or factory reset the driver accepts: 10 seconds. */
inline constexpr uint32_t kMaxSettleUs = 10'000'000;
/** @brief Lowest and highest bit rate the driver accepts for `Config::baud`. */
inline constexpr uint32_t kMinBaud = 9'600;
/** @brief Highest bit rate the driver accepts (the e-Manual lists up to 4.5 Mbit/s, the P
 * series 10.5). */
inline constexpr uint32_t kMaxBaud = 10'500'000;
/** @brief The most repeats of an idempotent instruction `Config::retries` allows. */
inline constexpr uint8_t kMaxRetries = 5;

/**
 * @brief Everything the driver needs to know about the bus.
 *
 * The defaults suit a protocol 2.0 bus at the factory baud rate; the timing fields are driver
 * side (the e-Manual gives no reply deadline: see docs/configuration.md, ambiguities).
 */
struct Config {
  Protocol protocol{Protocol::V2}; ///< Packet format of every servo on the bus.
  uint32_t baud{
      57'600}; ///< Bit rate of the UART; applied through `SetBaudRate()` when the port has it.
  uint32_t reply_timeout_us{
      20'000}; ///< Longest wait for the first byte of a reply (`kMinTimeoutUs` to `kMaxTimeoutUs`).
  uint32_t gap_timeout_us{5'000};   ///< Longest wait for the next byte inside a packet.
  uint32_t reboot_wait_us{500'000}; ///< Wait after a Reboot, Restore or Factory Reset reply before
                                    ///< the next instruction.
  uint32_t baud_change_wait_us{10'000}; ///< Wait after a servo's baud rate changed.
  uint8_t retries{
      1}; ///< Repeats of a ping or read after a bad or missing reply, 0 to `kMaxRetries`.
  EchoMode echo{EchoMode::None}; ///< Single-wire loopback verification.
  AlertPolicy alert_policy{
      AlertPolicy::Report}; ///< What a hardware alert in a reply does to the call.
  StatusReturn status_return{
      StatusReturn::All}; ///< How much the servos answer (applies to the whole bus).
  uint32_t broadcast_timeout_us{
      100'000}; ///< Wait for each reply of a broadcast ping (the servos answer in ID order).
  bool allow_destructive{
      false}; ///< Permit Factory Reset, Backup Restore and writes to the ID with broadcast.
};

/**
 * @brief Calls on a bus of servos that all answer: the shortest sensible setup for a first test.
 * @return A configuration with a 10 ms reply timeout.
 */
constexpr Config FastBusPreset() noexcept {
  Config c{};
  c.reply_timeout_us = 10'000;
  c.gap_timeout_us = 2'000;
  return c;
}

/**
 * @brief A bus on a USB adapter whose receive buffer is flushed in 16 ms steps.
 * @return A configuration with long timeouts that tolerate the adapter's latency timer.
 */
constexpr Config UsbAdapterPreset() noexcept {
  Config c{};
  c.reply_timeout_us = 60'000;
  c.gap_timeout_us = 40'000;
  return c;
}

/**
 * @brief The legacy bus: protocol 1.0 at 1 Mbit/s with the single-wire loopback of a half-duplex
 * adapter.
 * @return A protocol 1.0 configuration.
 */
constexpr Config LegacyBusPreset() noexcept {
  Config c{};
  c.protocol = Protocol::V1;
  c.baud = 1'000'000;
  return c;
}

namespace detail {
/// Last enumerator of `Protocol` (detail::IsEnumerator).
template <>
struct EnumLast<Protocol> {
  static constexpr Protocol kValue = Protocol::V2; ///< The last enumerator.
};
/// Last enumerator of `EchoMode`.
template <>
struct EnumLast<EchoMode> {
  static constexpr EchoMode kValue = EchoMode::Loopback; ///< The last enumerator.
};
/// Last enumerator of `AlertPolicy`.
template <>
struct EnumLast<AlertPolicy> {
  static constexpr AlertPolicy kValue = AlertPolicy::Fail; ///< The last enumerator.
};
/// Last enumerator of `StatusReturn`.
template <>
struct EnumLast<StatusReturn> {
  static constexpr StatusReturn kValue = StatusReturn::All; ///< The last enumerator.
};
} // namespace detail

/**
 * @brief Check a configuration without touching the bus.
 *
 * Fails when an enum holds a value that is not an enumerator, the baud rate is outside
 * `kMinBaud` to `kMaxBaud`, a timeout is outside `kMinTimeoutUs` to `kMaxTimeoutUs`, a wait is
 * above `kMaxSettleUs`, or `retries` exceeds `kMaxRetries`.
 *
 * @param config The configuration.
 * @return `Error::Ok` or `Error::InvalidArgument`.
 */
constexpr Error Validate(const Config& config) noexcept {
  const bool enums_ok =
      detail::IsEnumerator(config.protocol) && detail::IsEnumerator(config.echo) &&
      detail::IsEnumerator(config.alert_policy) && detail::IsEnumerator(config.status_return);
  const bool baud_ok = (config.baud >= kMinBaud) && (config.baud <= kMaxBaud);
  const bool timeouts_ok =
      detail::InRange(config.reply_timeout_us, kMinTimeoutUs, kMaxTimeoutUs) &&
      detail::InRange(config.gap_timeout_us, kMinTimeoutUs, kMaxTimeoutUs) &&
      detail::InRange(config.broadcast_timeout_us, kMinTimeoutUs, kMaxTimeoutUs);
  const bool waits_ok =
      (config.reboot_wait_us <= kMaxSettleUs) && (config.baud_change_wait_us <= kMaxSettleUs);
  const bool retries_ok = config.retries <= kMaxRetries;
  return (enums_ok && baud_ok && timeouts_ok && waits_ok && retries_ok) ? Error::Ok
                                                                        : Error::InvalidArgument;
}

// Every preset is valid, proved at build time (KB-CPP20-012).
static_assert(Validate(Config{}) == Error::Ok);
static_assert(Validate(FastBusPreset()) == Error::Ok);
static_assert(Validate(UsbAdapterPreset()) == Error::Ok);
static_assert(Validate(LegacyBusPreset()) == Error::Ok);

/**
 * @brief Decode the error byte of a status packet.
 * @param protocol The protocol the packet used.
 * @param id The ID the packet came from.
 * @param error_byte The raw error byte.
 * @return The decoded `ReplyInfo`.
 */
constexpr ReplyInfo Classify(Protocol protocol, uint8_t id, uint8_t error_byte) noexcept {
  ReplyInfo info{};
  info.id = id;
  info.error_byte = error_byte;
  const auto bits = static_cast<uint32_t>(error_byte);
  if (protocol == Protocol::V2) {
    info.alert = (bits & kAlertBit) != 0U;
    info.rejected = (bits & 0x7FU) != 0U;
  } else {
    info.alert = (bits & p1_error::kConditions) != 0U;
    info.rejected = (bits & p1_error::kCommandFaults) != 0U;
  }
  return info;
}

/**
 * @brief The protocol 2.0 error number of an error byte.
 * @param error_byte The raw error byte.
 * @return The number, or `Protocol2Error::None`; a number the e-Manual does not define maps to
 * `ResultFail`.
 */
constexpr Protocol2Error ErrorNumber(uint8_t error_byte) noexcept {
  const uint32_t number = static_cast<uint32_t>(error_byte) & 0x7FU;
  return (number <= 7U) ? static_cast<Protocol2Error>(number) : Protocol2Error::ResultFail;
}

static_assert(!Classify(Protocol::V2, 1_u8, 0x80_u8).rejected &&
              Classify(Protocol::V2, 1_u8, 0x80_u8).alert);
static_assert(Classify(Protocol::V2, 1_u8, 0x04_u8).rejected);
static_assert(Classify(Protocol::V1, 1_u8, 0x24_u8).alert &&
              !Classify(Protocol::V1, 1_u8, 0x24_u8).rejected);
static_assert(Classify(Protocol::V1, 1_u8, 0x40_u8).rejected);

/**
 * @brief Reached only when a `CheckedConfig` is built from an invalid configuration: it is never
 * defined, so the build fails and names it (KB-CPP20-013).
 */
void ConfigIsInvalid() noexcept;

/**
 * @brief A configuration proven valid at build time.
 *
 * `constexpr CheckedConfig kBus{Config{.baud = 1'000'000}};` fails to compile, naming
 * `ConfigIsInvalid`, when `Validate()` rejects the configuration. Pass `kBus.Get()` to the
 * driver; the driver still validates at start-up, for configurations built at run time.
 */
class CheckedConfig {
public:
  /**
   * @brief Check `config` at build time.
   * @param config The configuration; it must pass `Validate()`.
   */
  consteval explicit CheckedConfig(const Config& config) noexcept : config_(config) {
    if (Validate(config) != Error::Ok) {
      ConfigIsInvalid();
    }
  }
  /** @return The checked configuration. */
  [[nodiscard]] constexpr const Config& Get() const& noexcept {
    return config_;
  }
  /** @brief Not on a temporary: the reference would dangle (KB-CPP20-044). */
  void Get() const&& = delete;

private:
  Config config_;
};

/** @return The driver version, e.g. `"0.1.0"`, from the generated `dynamixel_version.h`. No I/O. */
constexpr const char* GetDriverVersion() noexcept {
  return DYNAMIXEL_VERSION_STRING;
}
/** @return The major version. No I/O. */
constexpr uint32_t GetDriverVersionMajor() noexcept {
  return DYNAMIXEL_VERSION_MAJOR;
}
/** @return The minor version. No I/O. */
constexpr uint32_t GetDriverVersionMinor() noexcept {
  return DYNAMIXEL_VERSION_MINOR;
}
/** @return The patch version. No I/O. */
constexpr uint32_t GetDriverVersionPatch() noexcept {
  return DYNAMIXEL_VERSION_PATCH;
}

/** @} */

} // namespace dynamixel

#endif // DYNAMIXEL_TYPES_HPP
