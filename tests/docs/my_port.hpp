/**
 * @file my_port.hpp
 * @brief The UART port shown on the Quickstart and Porting pages, compiled by the host build.
 *
 * Every platform call goes through the `platform_*` functions declared below; a real port replaces
 * them with its SDK (ESP-IDF, STM32 HAL, Zephyr, Linux termios).
 */
#ifndef DYNAMIXEL_MY_PORT_HPP
#define DYNAMIXEL_MY_PORT_HPP

#include <cstddef>
#include <cstdint>

#include "dynamixel_uart_interface.hpp"

// Your SDK's calls. Declared only: the docs build checks that the port compiles.
bool platform_uart_init() noexcept; // 57600 8N1, half duplex
bool platform_uart_send(const uint8_t* data,
                        std::size_t len) noexcept; // returns after the last stop bit
std::size_t platform_uart_receive(uint8_t* data, std::size_t max,
                                  uint32_t timeout_us) noexcept; // 0 when nothing arrived
bool platform_uart_discard_input() noexcept;
void platform_delay_us(uint32_t us) noexcept;

// --8<-- [start:port]
class MyPort : public dynamixel::UartInterface<MyPort> {
public:
  // Bring the UART up once; later calls return true at once.
  bool EnsureInitialized() noexcept {
    if (!ready_) {
      ready_ = platform_uart_init();
    }
    return ready_;
  }

  // Transmit the whole packet in one go, wait for the last stop bit, then listen again.
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    return platform_uart_send(data, len);
  }

  // One to `max` bytes as soon as one has arrived, or 0 after `timeout_us` without a byte.
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    return platform_uart_receive(data, max, timeout_us);
  }

  // Drop whatever has been received and not yet read.
  bool FlushInput() noexcept {
    return platform_uart_discard_input();
  }

  // Wait at least `us` microseconds (the settle times after a reboot or a baud rate change).
  void DelayUs(uint32_t us) noexcept {
    platform_delay_us(us);
  }

private:
  bool ready_ = false;
};
// --8<-- [end:port]

#endif // DYNAMIXEL_MY_PORT_HPP
