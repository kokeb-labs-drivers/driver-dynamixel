// clang-format off: the tables below must keep one row per line.
/**
 * @file dynamixel.hpp
 * @brief DYNAMIXEL driver: the one header to include.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * `dynamixel::DYNAMIXEL<Port>` is the bus master of a DYNAMIXEL servo bus: protocol 1.0 or 2.0 over a
 * half-duplex UART, through a platform port you write (see `dynamixel_uart_interface.hpp`). The driver
 * has no platform code, allocates nothing, throws nothing, takes no lock and is not thread-safe: give
 * each object one owner, or guard it with one mutex. Every call blocks until its reply is complete or
 * the configured timeouts have passed (docs/architecture.md explains why there is no polling API).
 *
 * Instruction coverage: Ping, Read, Write, Reg Write, Action, Factory Reset, Reboot, Clear, Control Table
 * Backup, Sync Read, Sync Write, Fast Sync Read, Bulk Read, Bulk Write and Fast Bulk Read (docs/protocol.md
 * has the table with the protocol each instruction needs).
 */
// clang-format on
#ifndef DYNAMIXEL_HPP
#define DYNAMIXEL_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "dynamixel_detail.hpp"
#include "dynamixel_protocol.hpp"
#include "dynamixel_tables.hpp"
#include "dynamixel_types.hpp"
#include "dynamixel_uart_interface.hpp"
#include "dynamixel_version.h"

namespace dynamixel {

namespace detail {

// Optional hooks: a `Declares*` concept only tests the name, the `Has*` concept the exact
// signature, `noexcept` and the result type; the driver rejects a hook that is declared but does
// not match (KB-CPP20-007).

/// The port declares something called `SetBaudRate`.
template <typename Port>
concept DeclaresSetBaudRate = requires(Port& port) { port.SetBaudRate(1_u32); };

/// The port reconfigures its UART: `bool SetBaudRate(uint32_t baud) noexcept`.
template <typename Port>
concept HasSetBaudRate =
    requires { ExactHook<Port, bool(uint32_t) noexcept>::Take(&Port::SetBaudRate); };

/// The port declares something called `Log`.
template <typename Port>
concept DeclaresLog = requires(Port& port) { port.Log(LogLevel::Info, ""); };

/// The port takes log lines: `void Log(LogLevel, const char* message) noexcept`.
template <typename Port>
concept HasLog =
    requires { ExactHook<Port, void(LogLevel, const char*) noexcept>::Take(&Port::Log); };

/// Log through the port, or nothing when the port has no `Log()`. The driver never formats text.
template <typename Port>
void Log(Port& port, [[maybe_unused]] LogLevel level,
         [[maybe_unused]] const char* message) noexcept {
  if constexpr (HasLog<Port>) {
    port.Log(level, message);
  } else {
    static_cast<void>(port);
  }
}

/// One multi-servo read request: a Sync Read (one address and length for the listed IDs) or a Bulk
/// Read (one item each).
struct MultiRequest {
  bool fast;        ///< Fast Sync Read or Fast Bulk Read.
  bool bulk;        ///< Bulk Read: `items` is used; otherwise `ids`, `address` and `length`.
  uint16_t address; ///< Sync Read: the first address.
  uint16_t length;  ///< Sync Read: the bytes per servo.
  std::span<const uint8_t> ids;        ///< Sync Read: the IDs.
  std::span<const BulkReadItem> items; ///< Bulk Read: the servos with their addresses and lengths.

  /** @return How many servos are asked. */
  [[nodiscard]] std::size_t Count() const noexcept {
    return bulk ? items.size() : ids.size();
  }
  /** @return The ID of servo `k`. */
  [[nodiscard]] uint8_t IdAt(std::size_t k) const noexcept {
    return bulk ? items[k].id : ids[k];
  }
  /** @return The bytes asked of servo `k`. */
  [[nodiscard]] std::size_t LengthAt(std::size_t k) const noexcept {
    return bulk ? static_cast<std::size_t>(items[k].length) : static_cast<std::size_t>(length);
  }
  /** @return The first address read from servo `k`. */
  [[nodiscard]] std::size_t AddressAt(std::size_t k) const noexcept {
    return bulk ? static_cast<std::size_t>(items[k].address) : static_cast<std::size_t>(address);
  }
  /** @return true when a servo listed after `k` has the ID `id`. */
  [[nodiscard]] bool ListedAfter(std::size_t k, uint8_t id) const noexcept {
    bool later = false;
    for (std::size_t j = k + 1U; j < Count(); ++j) {
      later = later || (IdAt(j) == id);
    }
    return later;
  }
};

/// Where the walk through a Fast read packet is.
struct FastCursor {
  std::size_t pos;  ///< Offset of the next part in the receive buffer.
  std::size_t have; ///< Bytes of the packet that arrived.
  bool open;        ///< More parts can still be read from the packet.
  Error stop;       ///< The result of the devices after the packet ended or broke.
};

/// What the attempts of a torque-off have shown so far. Outside the class so that Doxygen's class
/// page has no nested-class section (its link check fails on the anchor that section would need).
struct StopTally {
  Error last{Error::Bus};  ///< The outcome of the latest attempt.
  bool sent{false};        ///< At least one attempt was carried out on the line.
  bool confirmed{false};   ///< A trusted answer says the write was carried out.
  bool flush_fault{false}; ///< The flush before an attempt failed: answers are not trusted.
};

} // namespace detail

/**
 * @defgroup dynamixel_driver DYNAMIXEL driver
 * @brief The driver class.
 * @{
 */

// clang-format off
/**
 * @brief Master of one DYNAMIXEL bus on a platform port.
 *
 * Lifecycle: construct with the configuration (no I/O), then `EnsureInitialized()`, use, optionally
 * `Deinitialize()`. Every call that touches the bus returns `Error::NotInitialized` until
 * `EnsureInitialized()` has succeeded, with one exception: the torque-off calls (`DisableTorque()`,
 * `DisableTorqueAll()`, `DisableTorqueEach()`, `DisableTorqueAt()`) and `Reboot()` are never refused for the driver's
 * state; they bring the port up themselves (and set the bus baud rate) and write even when the
 * input flush fails.
 *
 * Optional port hooks, found at compile time:
 *
 * | Hook | Used for | Without it |
 * | --- | --- | --- |
 * | `bool SetBaudRate(uint32_t baud) noexcept` | `EnsureInitialized()`, `ApplyConfig()` and `ChangeBusBaud()` set the UART rate | The port's own rate stays; `ChangeBusBaud()` returns `Unsupported` |
 * | `void Log(LogLevel, const char* message) noexcept` | One line per lifecycle event or failure | Nothing is logged |
 *
 * A hook declared with the wrong signature is a compile error.
 *
 * @tparam Port Your class derived from `UartInterface<Port>`.
 * @tparam FrameBytes Size of the transmit and of the receive buffer, 64 to 4096 bytes (default 512). A
 *         transfer that does not fit returns `InvalidArgument` before anything is sent.
 */
// clang-format on
template <UartPort Port, std::size_t FrameBytes = 512>
class DYNAMIXEL {
  static_assert((FrameBytes >= 64U) && (FrameBytes <= 4096U),
                "dynamixel: FrameBytes must be 64 to 4096");
  static_assert(
      !detail::DeclaresSetBaudRate<Port> || detail::HasSetBaudRate<Port>,
      "dynamixel: the optional SetBaudRate hook must be `bool SetBaudRate(uint32_t) noexcept`");
  static_assert(!detail::DeclaresLog<Port> || detail::HasLog<Port>,
                "dynamixel: the optional Log hook must be `void Log(LogLevel, const char* message) "
                "noexcept`");

public:
  /**
   * @brief Bind the driver to a port and store the configuration. No I/O, cannot fail.
   * @param port The port; it must outlive the driver.
   * @param config What `EnsureInitialized()` will apply; checked there, before any I/O.
   */
  explicit DYNAMIXEL(Port& port, const Config& config = {}) noexcept
      : port_(port), config_(config) {}

  /** @brief One driver object per bus. */
  DYNAMIXEL(const DYNAMIXEL&) = delete;
  /** @brief One driver object per bus. */
  DYNAMIXEL& operator=(const DYNAMIXEL&) = delete;

  /// @name Lifecycle
  /// @{

  /**
   * @brief Start the driver, once: validate the configuration, bring the port up and set its rate.
   *
   * After it succeeds, later calls return `Ok` at once with no I/O. After it fails, the driver
   * stays uninitialised and the next call starts again. No servo is addressed.
   *
   * @return `Ok`, `InvalidArgument` (the stored configuration fails `Validate()`; no I/O) or `Bus`.
   */
  Result<void> EnsureInitialized() noexcept;

  /** @return true after a successful `EnsureInitialized()` and before `Deinitialize()`. No I/O. */
  [[nodiscard]] bool IsInitialized() const noexcept {
    return initialized_;
  }

  /**
   * @brief Mark the driver uninitialised. No bus traffic: the servos keep their state (torque stays
   * on; call `DisableTorqueAll()` first when the servos must stop).
   * @return `Ok`.
   */
  Result<void> Deinitialize() noexcept;

  /**
   * @brief Resynchronise: drop everything received and wait until the line has been quiet for one
   * reply timeout, so that a late status packet cannot be taken for the answer to the next
   * instruction.
   * @return `Ok`, `NotInitialized`, `Bus`, or `LineFault` when the line never went quiet.
   */
  Result<void> Resync() noexcept;

  /**
   * @brief Change the whole configuration; the UART rate follows through `SetBaudRate()` when the
   * port has it.
   * @param config Checked with `Validate()` before any I/O.
   * @return `Ok`, `InvalidArgument` or `Bus`.
   */
  Result<void> ApplyConfig(const Config& config) noexcept;

  /** @return The configuration last applied or stored. No I/O. */
  [[nodiscard]] const Config& GetConfig() const& noexcept {
    return config_;
  }
  /** @brief Not on a temporary driver: the reference would dangle (KB-CPP20-044). */
  void GetConfig() const&& = delete;

  /// @}
  /// @name Discovery
  /// @{

  /**
   * @brief Ping one servo (always answered, whatever the Status Return Level).
   * @param id 0 to 252 (protocol 2.0) or 253 (protocol 1.0); not the broadcast ID.
   * @return Protocol 2.0: ID, model number, firmware version and family. Protocol 1.0: the ping
   * carries no data, so only `id` is set (`Identify()` reads the rest). Errors: `NotInitialized`,
   *         `InvalidArgument`, `Bus`, `Timeout`, `Crc`, `InvalidReply`, `LineFault`,
   * `EchoMismatch`, `DeviceIdMismatch`, `DeviceError`, `DeviceFault`.
   */
  Result<PingInfo> Ping(uint8_t id) noexcept;

  /**
   * @brief Ping and learn the model: protocol 2.0 is one ping, protocol 1.0 a ping and a read of
   * the Model Number and Firmware Version registers.
   * @param id The servo ID.
   * @return The identity, or the errors of `Ping()` and `Read()` (`Unsupported` when the servos
   * answer pings only on protocol 1.0).
   */
  Result<PingInfo> Identify(uint8_t id) noexcept;

  /**
   * @brief Ping the broadcast ID and collect the answers (protocol 2.0): every servo answers, in ID
   * order.
   *
   * Waits `Config::broadcast_timeout_us` for each answer and stops at the first silence, at a
   * damaged answer or when `found` is full (the remaining answers are dropped by the next call's
   * flush).
   *
   * @param found Receives one entry per distinct ID that answered validly.
   * @return The number of entries, or `NotInitialized`, `InvalidArgument` (empty `found`),
   * `Unsupported` (protocol 1.0 has no broadcast ping answers), `Bus`, or the error of the first
   * answer when none was valid.
   */
  Result<uint32_t> BroadcastPing(std::span<PingInfo> found) noexcept;

  /**
   * @brief Ping every ID from `first` to `last` one after another.
   * @param first First ID.
   * @param last Last ID, not below `first`, within the protocol's ID range.
   * @param found Receives the IDs that answered; `InvalidArgument` when it is empty.
   * @return The number of entries (silent IDs are skipped, so is a damaged answer), or
   * `NotInitialized`, `InvalidArgument` or `Bus`. A full `found` stops the scan.
   */
  Result<uint32_t> ScanIds(uint8_t first, uint8_t last, std::span<PingInfo> found) noexcept;

  /// @}
  /// @name Raw control-table access
  /// @{

  /**
   * @brief Read bytes of one servo's control table.
   * @param id The servo ID (not the broadcast ID: reads are not executed on it).
   * @param address First address (0 to 255 in protocol 1.0).
   * @param out Receives `out.size()` bytes, 1 or more; the reply must fit the receive buffer even
   * with byte stuffing.
   * @return `Ok`, `NotInitialized`, `InvalidArgument` (nothing sent), `Unsupported` (the status
   * return level is ping only), `Bus`, `Timeout`, `Crc`, `InvalidReply`, `LineFault`,
   * `EchoMismatch`, `DeviceIdMismatch`, `DeviceError` (the servo's error number is in
   * `GetLastReply()`) or `DeviceFault`. Retried `Config::retries` times.
   */
  Result<void> Read(uint8_t id, uint16_t address, std::span<uint8_t> out) noexcept;

  /**
   * @brief Write bytes to the control table.
   *
   * Not retried: a lost reply does not tell whether the servo applied the write. With the broadcast
   * ID nothing is answered and `Ok` only means "sent".
   *
   * @param id The servo ID or 254 (every servo).
   * @param address First address.
   * @param data The bytes, 1 or more.
   * @return As `Read()`, plus `Ok` for a sent broadcast.
   */
  Result<void> Write(uint8_t id, uint16_t address, std::span<const uint8_t> data) noexcept;

  /**
   * @brief Store a write to be executed by the next Action instruction (Reg Write).
   * @param id The servo ID or 254.
   * @param address First address.
   * @param data The bytes, 1 or more.
   * @return As `Write()`.
   */
  Result<void> RegWrite(uint8_t id, uint16_t address, std::span<const uint8_t> data) noexcept;

  /**
   * @brief Execute the stored Reg Write (all servos at once with the broadcast ID).
   * @param id The servo ID or 254.
   * @return As `Write()`.
   */
  Result<void> Action(uint8_t id) noexcept;

  /// @}
  /// @name Reset, reboot, clear, backup
  /// @{

  /**
   * @brief Reset a servo's control table to the factory values.
   *
   * Needs `Config::allow_destructive`. Reset all (`ResetOption::All`) also resets the ID to 1 and
   * the baud rate to 57 600 bit/s: the servo then answers at another ID and rate than the bus uses.
   * Protocol 1.0 has no option byte (it resets everything: only `ResetOption::All` is accepted) and
   * cannot use the broadcast ID; protocol 2.0 refuses reset all on the broadcast ID (the firmware
   * does too). Waits `Config::reboot_wait_us` afterwards.
   *
   * @param id The servo ID (broadcast only protocol 2.0 with a keep option).
   * @param option What to keep.
   * @return `NotPermitted`, `InvalidArgument` or the errors of `Write()`.
   */
  Result<void> FactoryReset(uint8_t id, ResetOption option) noexcept;

  /**
   * @brief Reboot a servo; waits `Config::reboot_wait_us` after the reply. Sent like the stop: it
   * needs no `EnsureInitialized()` and is written even when the input flush fails (the call then
   * returns `Bus` after the write).
   * @param id The servo ID or 254.
   * @return `InvalidArgument`, `Bus`, or as `Write()`.
   */
  Result<void> Reboot(uint8_t id) noexcept;

  /**
   * @brief Clear the multi-turn count or registered errors (protocol 2.0). Needs
   * `Config::allow_destructive`.
   * @param id The servo ID or 254.
   * @param option What to clear.
   * @return `NotPermitted`, `Unsupported` (protocol 1.0), `InvalidArgument` or the errors of
   * `Write()`.
   */
  Result<void> Clear(uint8_t id, ClearOption option) noexcept;

  /**
   * @brief Store the control table in the backup area, or restore the EEPROM from it
   * (protocol 2.0).
   *
   * Restore needs `Config::allow_destructive`, makes the servo reboot and waits
   * `Config::reboot_wait_us`. Torque must be off or the servo answers with a result fail error.
   * Store is not behind `allow_destructive`, although it overwrites the previous backup in the
   * servo's backup area: it changes nothing the servo runs on, and a restore still needs consent.
   *
   * @param id The servo ID or 254.
   * @param option Store or restore.
   * @return `NotPermitted`, `Unsupported` (protocol 1.0), `InvalidArgument` or the errors of
   * `Write()`.
   */
  Result<void> Backup(uint8_t id, BackupOption option) noexcept;

  /// @}
  /// @name Sync and bulk instructions
  /// @{

  /**
   * @brief Write the same address and length to several servos with one packet (both protocols). No
   * answer.
   * @param address First address (0 to 255 in protocol 1.0).
   * @param length Bytes per servo, 1 or more.
   * @param ids The servo IDs (1 to many, not 254).
   * @param data `ids.size() * length` bytes, the bytes of the first servo first.
   * @return `Ok` (sent), `NotInitialized`, `InvalidArgument` (nothing sent, also when the packet
   * does not fit the transmit buffer) or `Bus`.
   */
  Result<void> SyncWrite(uint16_t address, uint16_t length, std::span<const uint8_t> ids,
                         std::span<const uint8_t> data) noexcept;

  /**
   * @brief Write different addresses and lengths to several servos with one packet (protocol 2.0).
   * No answer.
   * @param items One entry per servo, each ID once, each with at least one byte.
   * @return `Ok`, `NotInitialized`, `Unsupported` (protocol 1.0), `InvalidArgument` or `Bus`.
   */
  Result<void> BulkWrite(std::span<const BulkWriteItem> items) noexcept;

  /**
   * @brief Read the same address and length from several servos (protocol 2.0): each one answers
   * with its own packet, in the order of `ids`.
   * @param address First address.
   * @param length Bytes per servo, 1 or more.
   * @param ids The servo IDs, 1 to many, none the broadcast ID.
   * @param out `ids.size() * length` bytes; entry k holds servo k's data when `replies[k].result`
   * is `Ok`.
   * @param replies One entry per ID: its outcome.
   * @return `Ok` when every servo answered validly, else the first failing outcome;
   * `NotInitialized`, `Unsupported` (protocol 1.0, or the status return level is ping only),
   * `InvalidArgument` or `Bus`. Not retried as a whole: re-read the failed servos.
   */
  Result<void> SyncRead(uint16_t address, uint16_t length, std::span<const uint8_t> ids,
                        std::span<uint8_t> out, std::span<DeviceReply> replies) noexcept;

  /**
   * @brief Sync Read with one combined answer packet (Fast Sync Read, protocol 2.0): the servos
   * answer back to back inside one packet, every part with its own error byte, ID and CRC.
   * @param address First address.
   * @param length Bytes per servo.
   * @param ids The servo IDs.
   * @param out `ids.size() * length` bytes.
   * @param replies One entry per ID. A packet that ends early still delivers the parts that arrived
   * whole and checked; the rest is `Timeout`.
   * @return As `SyncRead()`. The combined packet must fit the receive buffer (`InvalidArgument`
   * otherwise).
   */
  Result<void> FastSyncRead(uint16_t address, uint16_t length, std::span<const uint8_t> ids,
                            std::span<uint8_t> out, std::span<DeviceReply> replies) noexcept;

  /**
   * @brief Read different addresses and lengths from several servos (protocol 1.0 for MX and X
   * servos, and 2.0).
   * @param items One entry per servo, each ID once.
   * @param out The sum of all lengths in bytes; the data of servo k follows that of servo k - 1.
   * @param replies One entry per item.
   * @return As `SyncRead()`.
   */
  Result<void> BulkRead(std::span<const BulkReadItem> items, std::span<uint8_t> out,
                        std::span<DeviceReply> replies) noexcept;

  /**
   * @brief Bulk Read with one combined answer packet (Fast Bulk Read, protocol 2.0).
   * @param items One entry per servo, each ID once.
   * @param out The sum of all lengths in bytes.
   * @param replies One entry per item.
   * @return As `FastSyncRead()`.
   */
  Result<void> FastBulkRead(std::span<const BulkReadItem> items, std::span<uint8_t> out,
                            std::span<DeviceReply> replies) noexcept;

  /// @}
  /// @name Typed register access
  /// @{

  /**
   * @brief Read one register by its family-neutral name and decode it.
   * @param servo The servo: ID and family (`FamilyOf()` of a ping's model number).
   * @param reg The register.
   * @return The decoded value, `Unsupported` (the family has no such register, or does not speak
   * the bus protocol: nothing is sent), or the errors of `Read()`.
   */
  Result<int32_t> ReadRegister(ServoRef servo, Reg reg) noexcept;

  /**
   * @brief Encode and write one register.
   * @param servo The servo (or `{254, family}` for every servo of a family).
   * @param reg The register.
   * @param value The value in the register's unit.
   * @return `InvalidArgument` (the register is read-only, or the value does not fit its size;
   * nothing sent), `Unsupported` (the family has no such register, or does not speak the bus
   * protocol) or the errors of `Write()`.
   */
  Result<void> WriteRegister(ServoRef servo, Reg reg, int32_t value) noexcept;

  /**
   * @brief Turn the torque on or off. Use `DisableTorque()` to stop: it is never refused.
   * @param servo The servo.
   * @param enable true to enable.
   * @return As `WriteRegister()`.
   */
  Result<void> SetTorque(ServoRef servo, bool enable) noexcept;

  /**
   * @brief Turn one servo's torque off. Never refused for the driver's state: it needs no
   * `EnsureInitialized()` (it starts the port and sets the bus baud rate), ignores any earlier
   * failure and asks for no consent, and writes even when the input flush fails. The write is
   * idempotent: it is repeated (up to three times) until a trusted answer confirms it, and sent all
   * three times when no answer can (status return level below 2, or a broadcast).
   * @param servo The servo; its family must have a torque register and speak the bus protocol
   * (not `Generic`: use `DisableTorqueAt()`).
   * @return `Ok` when confirmed (or sent, without answers), `InvalidArgument` (the ID, or a family
   * with no torque register on this protocol; nothing is sent), `Bus` (the port did not start, or
   * the flush failed: the packets were sent all the same), or the last error of the attempts.
   */
  Result<void> DisableTorque(ServoRef servo) noexcept;

  /**
   * @brief Turn the torque of every servo of a family off with one broadcast. Never refused for the
   * driver's state. Nothing is answered, so the broadcast is sent three times and `Ok` means sent.
   * @param family The family whose torque register is written.
   * @return `Ok` (sent), `InvalidArgument` (`Generic`, or a family that does not speak the bus
   * protocol) or `Bus`.
   */
  Result<void> DisableTorqueAll(Family family) noexcept;

  /**
   * @brief Turn the torque off at an explicit address: the stop for a servo whose family is
   * `Generic` (a model the catalogue does not list) or one that has no `Reg::TorqueEnable`. It
   * shares the stop path of `DisableTorque()`: never refused for the driver's state, written even
   * when the input flush fails, up to three attempts. The caller supplies the address from the
   * servo's own control table (64 on the X series in protocol 2.0, 24 on AX/RX/DX).
   * @param id The servo ID, or 254 for every servo.
   * @param torque_address The Torque Enable address (at most 255 in protocol 1.0).
   * @return As `DisableTorque()`; `InvalidArgument` for an ID or address the protocol does not
   * have (nothing is sent).
   *
   * Warning: this writes one byte 0 at the address given, with ID 254 to every servo. A wrong
   * address can set IDs to 0, change the baud rate or silence the servos. Take the address from
   * the e-Manual's control table for the exact model and firmware (64 on the X series, 24 on
   * AX/RX/DX, MX 1.0 and XL-320; the Pro series differ).
   */
  Result<void> DisableTorqueAt(uint8_t id, uint16_t torque_address) noexcept;

  /**
   * @brief `DisableTorque()` for each servo of a list, whatever one of them does.
   * @param servos The servos.
   * @return `Ok`, or only the first error after all of them were tried: to know which servo
   * failed, call `DisableTorque()` for each one.
   */
  Result<void> DisableTorqueEach(std::span<const ServoRef> servos) noexcept;

  /**
   * @brief Read position, velocity, load or current, voltage and temperature in one transfer.
   * @param servo The servo; its family needs these registers (not `Generic`).
   * @return The feedback, or the errors of `Read()` and `Unsupported`.
   */
  Result<Feedback> ReadFeedback(ServoRef servo) noexcept;

  /**
   * @brief Write the Goal Position.
   * @param servo The servo.
   * @param counts Position in counts.
   * @return As `WriteRegister()`.
   */
  Result<void> SetGoalPosition(ServoRef servo, int32_t counts) noexcept;

  /**
   * @brief Change a servo's ID. Needs `Config::allow_destructive` when `servo.id` is the broadcast
   * ID.
   * @param servo The servo (current ID).
   * @param new_id 0 to 252 (253 in protocol 1.0).
   * @return `NotPermitted`, `InvalidArgument` or the errors of `WriteRegister()`.
   */
  Result<void> SetId(ServoRef servo, uint8_t new_id) noexcept;

  /**
   * @brief Write a servo's Baud Rate register; the new rate applies after the reply. The port keeps
   * its rate: follow with `ChangeBusBaud()`.
   * @param servo The servo.
   * @param bits_per_second A rate the family's baud scheme offers.
   * @return `InvalidArgument` (not offered) or the errors of `WriteRegister()`.
   */
  Result<void> SetServoBaudRate(ServoRef servo, uint32_t bits_per_second) noexcept;

  /**
   * @brief Change the UART rate of the port (after `SetServoBaudRate()`), wait
   * `Config::baud_change_wait_us` and resynchronise.
   * @param bits_per_second The new rate, `kMinBaud` to `kMaxBaud`.
   * @return `NotInitialized`, `InvalidArgument`, `Unsupported` (the port has no `SetBaudRate()`),
   * `Bus`.
   */
  Result<void> ChangeBusBaud(uint32_t bits_per_second) noexcept;

  /**
   * @brief Write the Status Return Level and adopt it as the bus's expectation. A write that lowers
   * the level may go unanswered: silence is then accepted as success and the level is adopted, with
   * a warning in the log. A lost write and a lost answer look the same: when it matters, read the
   * register back (at the old level, with `Read()`) to verify.
   * @param servo The servo (or `{254, family}`).
   * @param level The new level.
   * @return `InvalidArgument` or the errors of `WriteRegister()`.
   */
  Result<void> SetStatusReturnLevel(ServoRef servo, StatusReturn level) noexcept;

  /// @}
  /// @name Diagnostics
  /// @{

  /** @return The decoded error byte of the last status packet received. No I/O. */
  [[nodiscard]] const ReplyInfo& GetLastReply() const& noexcept {
    return last_reply_;
  }
  /** @brief Not on a temporary driver (KB-CPP20-044). */
  void GetLastReply() const&& = delete;

  /** @return The first alert or condition flag seen since `ClearLatchedAlert()`; all zero when
   * none. No I/O. */
  [[nodiscard]] const ReplyInfo& GetLatchedAlert() const& noexcept {
    return latched_alert_;
  }
  /** @brief Not on a temporary driver (KB-CPP20-044). */
  void GetLatchedAlert() const&& = delete;

  /** @brief Forget the latched alert. No I/O. */
  void ClearLatchedAlert() noexcept {
    latched_alert_ = ReplyInfo{};
  }

  /** @return The counters since `ResetStats()`. No I/O. */
  [[nodiscard]] const Stats& GetStats() const& noexcept {
    return stats_;
  }
  /** @brief Not on a temporary driver (KB-CPP20-044). */
  void GetStats() const&& = delete;

  /** @brief Zero the counters. No I/O. */
  void ResetStats() noexcept {
    stats_ = Stats{};
  }

  /**
   * @return The bytes of the last instruction packet sent, as put on the line (stuffed, with CRC);
   * empty before the first. No I/O (handbook KB-CPP-129).
   */
  [[nodiscard]] std::span<const uint8_t> GetLastRequest() const& noexcept {
    return std::span<const uint8_t>{tx_.data(), tx_len_};
  }
  /** @brief Not on a temporary driver (KB-CPP20-044). */
  void GetLastRequest() const&& = delete;

  /// @}

private:
  /// Which instructions the servo answers (the e-Manual's Response Policy).
  enum class ReplyRule : uint8_t {
    ReadLevel,  ///< Read: answered at status return level 1 and 2.
    WriteLevel, ///< Write, Reg Write, Action, reset, reboot, clear, backup: level 2 only.
  };

  [[nodiscard]] bool validId(uint8_t id, bool allow_broadcast) const noexcept;
  [[nodiscard]] bool replyExpected(ReplyRule rule, uint8_t id) const noexcept;
  [[nodiscard]] static bool retryable(Error error) noexcept;
  [[nodiscard]] static std::size_t stuffedBound(std::size_t bytes) noexcept;
  [[nodiscard]] static uint32_t saneTimeout(uint32_t us) noexcept;
  Error ready() const noexcept;
  Error sendFrame(std::size_t len, bool stop) noexcept;
  Error readEcho(std::size_t len) noexcept;
  void dropFront(std::size_t count) noexcept;
  Error receiveFrame(uint32_t first_timeout_us, std::size_t& frame_size) noexcept;
  Error acceptStatus(std::size_t frame_size, uint8_t expect_id, std::size_t expect_params,
                     std::span<uint8_t>& data) noexcept;
  Error transactOnce(std::size_t len, uint8_t id, bool expect_reply, std::size_t expect_params,
                     bool stop, std::span<uint8_t>& data) noexcept;
  [[nodiscard]] const RegInfo* lookup(Family family, Reg reg) const noexcept;
  bool startForStop() noexcept;
  Error stopOnce(uint16_t address, uint8_t id, bool answer) noexcept;
  bool noteStop(detail::StopTally& tally, Error attempt, bool answer) const noexcept;
  Result<void> stopOutcome(const detail::StopTally& tally, bool answer) noexcept;
  Error pingReply(std::size_t frame_size, std::span<const PingInfo> known, PingInfo& info) noexcept;
  Error transact(std::size_t len, uint8_t id, bool expect_reply, std::size_t expect_params,
                 bool idempotent, std::span<uint8_t>& data) noexcept;
  Result<void> sendWrite(protocol::Instruction instruction, uint8_t id, uint16_t address,
                         std::span<const uint8_t> data, bool settle) noexcept;
  Result<void> sendSimple(protocol::Instruction instruction, uint8_t id,
                          std::span<const uint8_t> params, bool settle) noexcept;
  Result<void> multiRead(const detail::MultiRequest& rq, std::span<uint8_t> out,
                         std::span<DeviceReply> replies) noexcept;
  [[nodiscard]] bool multiItemBad(const detail::MultiRequest& rq, std::size_t k) const noexcept;
  [[nodiscard]] bool multiBad(const detail::MultiRequest& rq, std::span<uint8_t> out,
                              std::span<DeviceReply> replies) const noexcept;
  [[nodiscard]] static protocol::Instruction multiInstruction(bool fast, bool bulk) noexcept;
  std::size_t buildMultiRead(const detail::MultiRequest& rq) noexcept;
  Error separateReceive(const detail::MultiRequest& rq, std::size_t k,
                        std::span<uint8_t>& data) noexcept;
  Error collectSeparate(const detail::MultiRequest& rq, std::span<uint8_t> out,
                        std::span<DeviceReply> replies) noexcept;
  [[nodiscard]] bool fastPartCrcOk(std::size_t pos, std::size_t part) const noexcept;
  void fastAccept(uint8_t id, std::size_t want, std::size_t pos, std::span<uint8_t> out,
                  std::size_t offset, DeviceReply& reply) noexcept;
  void fastPart(const detail::MultiRequest& rq, std::size_t k, detail::FastCursor& cur,
                std::span<uint8_t> out, std::size_t offset, DeviceReply& reply) noexcept;
  Error collectFast(const detail::MultiRequest& rq, std::span<uint8_t> out,
                    std::span<DeviceReply> replies) noexcept;
  Result<void> applyRate() noexcept;
  Result<void> fail(Error error, const char* message) noexcept;
  void noteReply(const ReplyInfo& info) noexcept;

  /// Longest typed read: the feedback block of any family fits.
  static constexpr std::size_t kScratchBytes = 64;
  /// Attempts of a torque-off write: the first and two repeats.
  static constexpr uint32_t kStopAttempts = 3;

  Port& port_;
  Config config_{};
  std::array<uint8_t, FrameBytes> tx_{};         ///< The instruction packet being sent.
  std::array<uint8_t, FrameBytes> rx_{};         ///< Received bytes not yet consumed.
  std::array<uint8_t, kScratchBytes> scratch_{}; ///< Typed reads and writes.
  std::size_t tx_len_{0};
  bool flush_fault_{false}; ///< The flush before the last frame failed (a stop is sent anyway).
  std::size_t rx_have_{0};
  ReplyInfo last_reply_{};
  ReplyInfo latched_alert_{};
  uint16_t alt_id_{
      0x100U}; ///< A second ID accepted in the reply to the call in progress (`SetId`), or 0x100.
  std::size_t rx_drop_{
      0}; ///< Bytes of the accepted reply still at the front of `rx_`, dropped lazily.
  Stats stats_{};
  bool initialized_{false};
};

/** @} */

} // namespace dynamixel

// codeql::misra_deviation_next_line(MD-CPP-009)
#include "../src/dynamixel.ipp" // IWYU pragma: export

#endif // DYNAMIXEL_HPP
