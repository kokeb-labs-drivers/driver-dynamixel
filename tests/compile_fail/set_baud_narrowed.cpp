/**
 * @file set_baud_narrowed.cpp
 * @brief Must NOT compile: a SetBaudRate hook with a narrower parameter would truncate the bit
 * rate, so the driver rejects it instead of using it (handbook KB-CPP20-007).
 *
 * EXPECT: SetBaudRate hook must be
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

class ShortBaudPort : public dynamixel::UartInterface<ShortBaudPort> {
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
  void DelayUs(uint32_t us) noexcept {
    static_cast<void>(us);
  }
  bool SetBaudRate(uint16_t baud) noexcept {
    static_cast<void>(baud);
    return true;
  }
};

int main() {
  ShortBaudPort port;
  dynamixel::DYNAMIXEL<ShortBaudPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
