/**
 * @file host_dynamixel_port.hpp
 * @brief The host port: the driver's UART hooks over a simulated servo bus, so the example runs on
 * any PC.
 *
 * `mock::Bus` (tests/mock_servo_bus.hpp) is the bus model the host tests use: servos with control
 * tables behind a byte queue. This class adds what a real port adds: the five hooks,
 * `SetBaudRate()`, and `Log()` to the console. A port for a real bus on a PC (a USB adapter through
 * termios) implements the same hooks: see docs/porting.md.
 */
#ifndef DYNAMIXEL_HOST_DYNAMIXEL_PORT_HPP
#define DYNAMIXEL_HOST_DYNAMIXEL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "dynamixel_uart_interface.hpp"
#include "mock_servo_bus.hpp"

/// The host port: simulated servos, real console.
class HostDynamixelPort : public dynamixel::UartInterface<HostDynamixelPort> {
public:
  /// The simulated bus: the example adds its servos here.
  mock::Bus bus{};

  /// Nothing to start on a simulated bus.
  bool EnsureInitialized() noexcept {
    return true;
  }
  /// The simulated servos see the packet at once.
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    bus.Transmit(data, len);
    return true;
  }
  /// Hand over what the servos answered.
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t /*timeout_us*/) noexcept {
    std::size_t n = 0;
    while ((n < max) && (bus.Queued() > 0U)) {
      data[n] = bus.rx[bus.rx_head];
      ++bus.rx_head;
      ++n;
    }
    return n;
  }
  /// Drop what has not been read.
  bool FlushInput() noexcept {
    bus.Clear();
    return true;
  }
  /// Time passes in the simulation; nothing sleeps.
  void DelayUs(uint32_t /*us*/) noexcept {}
  /// A real port reconfigures the UART here; the simulation accepts any rate.
  bool SetBaudRate(uint32_t /*baud*/) noexcept {
    return true;
  }
  /// Driver messages (they carry their own `dynamixel:` prefix) go to the console, one line each.
  void Log(dynamixel::LogLevel /*level*/, const char* message) noexcept {
    static_cast<void>(std::puts(message)); // plain text: never pass it as a format string
  }
};

#endif // DYNAMIXEL_HOST_DYNAMIXEL_PORT_HPP
