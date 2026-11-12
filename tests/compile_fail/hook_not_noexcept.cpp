/**
 * @file hook_not_noexcept.cpp
 * @brief Must NOT compile: a Read hook that is not noexcept breaks the port contract, so the port
 * concept rejects it (handbook KB-CPP20-006).
 *
 * EXPECT: UartPort
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

class ThrowingPort : public dynamixel::UartInterface<ThrowingPort> {
public:
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    static_cast<void>(data);
    return len > 0U;
  }
  std::size_t Read(uint8_t* data, std::size_t len, uint32_t timeout_ms) {
    static_cast<void>(data);
    static_cast<void>(timeout_ms);
    return len;
  }
  bool FlushInput() noexcept {
    return true;
  }
  void DelayUs(uint32_t us) noexcept {
    static_cast<void>(us);
  }
};

int main() {
  ThrowingPort port;
  dynamixel::DYNAMIXEL<ThrowingPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
