/**
 * @file port_without_delay.cpp
 * @brief Must NOT compile: the reboot and baud change waits need the port's DelayUs().
 *
 * EXPECT: DelayUs
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

class NoDelayPort : public dynamixel::UartInterface<NoDelayPort> {
public:
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    static_cast<void>(data);
    return len > 0U;
  }
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    static_cast<void>(data);
    static_cast<void>(timeout_us);
    return max;
  }
  bool FlushInput() noexcept {
    return true;
  }
};

int main() {
  NoDelayPort port;
  dynamixel::DYNAMIXEL<NoDelayPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
