// clang-format off: the table below must keep one row per line.
/**
 * @file dynamixel_uart_interface.hpp
 * @brief The UART port of the DYNAMIXEL driver: the CRTP base, the `UartPort` concept and the line
 * requirements.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * The driver never calls an SDK. It calls hooks on a class you derive from
 * `dynamixel::UartInterface<Self>`:
 *
 * | Hook | Contract |
 * | --- | --- |
 * | `bool EnsureInitialized() noexcept` | UART up at 8 data bits, no parity, 1 stop bit; `true` at once afterwards |
 * | `bool Write(const uint8_t* data, std::size_t len) noexcept` | Switch the line to transmit, send all `len` bytes without gaps, wait until the last stop bit has left the shift register, switch back to receive; `false` on an error |
 * | `std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept` | Return 1 to `max` bytes as soon as one is there, waiting at most `timeout_us` for the first; 0 when none arrived |
 * | `bool FlushInput() noexcept` | Drop every received byte not yet returned by `Read()` |
 * | `void DelayUs(uint32_t us) noexcept` | Wait at least `us` microseconds (reboot and baud change settle times) |
 * | `bool SetBaudRate(uint32_t baud) noexcept` (optional) | Reconfigure the UART; without it the baud is the port's own |
 * | `void Log(LogLevel, const char*) noexcept` (optional) | One plain text line per event |
 *
 * Half duplex (e-Manual, protocol 2.0, "Tx, Rx Direction"): the direction switch back to receive must
 * wait for the transmit shift register to be empty, not only the buffer, and must not be delayed by an
 * interrupt past the servo's Return Delay Time. A transmit pause longer than 1.5 ms inside a protocol 2.0
 * packet (100 ms in protocol 1.0) makes the servo drop the packet (`kInstructionByteGapMaxUs...`), so `Write()`
 * sends the packet in one go. With a single-wire loopback the port leaves the echo in the receive FIFO and
 * `Config::echo = EchoMode::Loopback` makes the driver verify it.
 * The port moves raw bytes; the driver builds, frames, checks and times everything.
 */
// clang-format on
#ifndef DYNAMIXEL_UART_INTERFACE_HPP
#define DYNAMIXEL_UART_INTERFACE_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>

#include "dynamixel_detail.hpp"
#include "dynamixel_types.hpp"

namespace dynamixel {

/**
 * @name Bus requirements
 * What the servos need from the UART (e-Manual, protocol 1.0 and 2.0, "DYNAMIXEL Protocol"). A port
 * and a board configuration check their settings against these with `static_assert` (handbook
 * KB-CPP-123).
 * @{
 */
/// Highest rate of the families in the catalogue (X series: 4.5 Mbit/s); the P series goes to 10.5
/// Mbit/s.
inline constexpr uint32_t kMaxBaudRate = 4'500'000;
/// Factory rate of the X series (Baud Rate register value 1).
inline constexpr uint32_t kFactoryBaudRate = 57'600;
/// Return Delay Time factory value 250 counts of 2 microseconds: the servo answers after 500
/// microseconds.
inline constexpr uint32_t kTurnaroundUs = 500;
/// Largest Return Delay Time (254 counts of 2 microseconds).
inline constexpr uint32_t kMaxTurnaroundUs = 508;
/// Longest pause between two bytes of a protocol 2.0 instruction packet that the servo accepts.
inline constexpr uint32_t kInstructionByteGapMaxUsV2 = 1500;
/// Longest pause between two bytes of a protocol 1.0 instruction packet that the servo accepts.
inline constexpr uint32_t kInstructionByteGapMaxUsV1 = 100'000;
/** @} */

/**
 * @brief CRTP base of the platform UART port.
 *
 * Derive `class MyPort : public dynamixel::UartInterface<MyPort>` and implement the hooks in this
 * file's description. The base declares no hook itself (a forwarding hook would turn a missing
 * override into infinite recursion) and no data; the driver checks your class with the `UartPort`
 * concept, so a missing or misdeclared hook is a compile error that names it.
 *
 * @tparam Derived Your port class.
 */
template <typename Derived>
class UartInterface {
public:
  /** @brief Ports are not copied: the driver holds a reference. */
  UartInterface(const UartInterface&) = delete;
  /** @brief Ports are not copied: the driver holds a reference. */
  UartInterface& operator=(const UartInterface&) = delete;

protected:
  /** @brief Only the derived port is constructed. */
  UartInterface() noexcept = default;
  /** @brief Destroyed through the derived port only. */
  ~UartInterface() noexcept = default;
};

/**
 * @brief Satisfied by a class that derives from `UartInterface<Port>` and implements the required
 * hooks with exactly the types above and `noexcept`.
 *
 * Each hook's address is matched with `detail::ExactHook`, which takes a member function of exactly
 * the signature and converts no value: a hook declared with a narrower length or timeout, or
 * without `noexcept`, does not satisfy the concept (handbook KB-CPP20-006).
 *
 * @tparam Port The port class.
 */
template <typename Port>
concept UartPort = std::derived_from<Port, UartInterface<Port>> && requires {
  detail::ExactHook<Port, bool() noexcept>::Take(&Port::EnsureInitialized);
  detail::ExactHook<Port, bool(const uint8_t*, std::size_t) noexcept>::Take(&Port::Write);
  detail::ExactHook<Port, std::size_t(uint8_t*, std::size_t, uint32_t) noexcept>::Take(&Port::Read);
  detail::ExactHook<Port, bool() noexcept>::Take(&Port::FlushInput);
  detail::ExactHook<Port, void(uint32_t) noexcept>::Take(&Port::DelayUs);
};

} // namespace dynamixel

#endif // DYNAMIXEL_UART_INTERFACE_HPP
