/**
 * @file ports.cpp
 * @brief The port with every optional hook, as shown on the Porting page. Compiled, never run.
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

bool platform_uart_init() noexcept;
bool platform_uart_send(const uint8_t* data, std::size_t len) noexcept;
std::size_t platform_uart_receive(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept;
bool platform_uart_discard_input() noexcept;
bool platform_uart_set_baud(uint32_t baud) noexcept;
bool platform_direction_transmit(bool transmit) noexcept;
void platform_delay_us(uint32_t us) noexcept;
void platform_log(const char* line) noexcept;

// --8<-- [start:full-port]
class FullPort : public dynamixel::UartInterface<FullPort> {
public:
  bool EnsureInitialized() noexcept {
    return platform_uart_init();
  }

  // Half duplex through a direction pin: transmit, wait for the last stop bit, listen again.
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    if (!platform_direction_transmit(true)) {
      return false;
    }
    const bool sent = platform_uart_send(data, len); // returns when the shift register is empty
    const bool listening = platform_direction_transmit(false);
    return sent && listening;
  }

  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    return platform_uart_receive(data, max, timeout_us);
  }

  bool FlushInput() noexcept {
    return platform_uart_discard_input();
  }

  void DelayUs(uint32_t us) noexcept {
    platform_delay_us(us);
  }

  // Optional: lets ApplyConfig() and ChangeBusBaud() move the UART to a new rate.
  bool SetBaudRate(uint32_t baud) noexcept {
    return platform_uart_set_baud(baud);
  }

  // Optional: one line per lifecycle event or failure. The driver never formats; the port does.
  void Log(dynamixel::LogLevel level, const char* message) noexcept {
    if (level == dynamixel::LogLevel::Error) {
      platform_log(message); // plain text: never use it as a format string
    } else {
      return;
    }
  }
};
// --8<-- [end:full-port]

// --8<-- [start:change-baud]
dynamixel::Error MoveBusTo1Mbit(dynamixel::DYNAMIXEL<FullPort>& servos, dynamixel::ServoRef servo) {
  // First the servo, at the old rate; then the port, followed by a resynchronisation.
  if (auto r = servos.SetServoBaudRate(servo, 1'000'000); !r) {
    return r.error;
  }
  return servos.ChangeBusBaud(1'000'000).error;
}
// --8<-- [end:change-baud]
