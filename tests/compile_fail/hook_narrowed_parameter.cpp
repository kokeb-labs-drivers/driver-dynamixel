/**
 * @file hook_narrowed_parameter.cpp
 * @brief Must NOT compile: a Read hook with a narrower length would truncate replies, so the port
 * concept rejects it (handbook KB-CPP20-006).
 *
 * EXPECT: UartPort
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

class NarrowPort : public dynamixel::UartInterface<NarrowPort> {
public:
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    static_cast<void>(data);
    return len > 0U;
  }
  std::size_t Read(uint8_t* data, uint8_t len, uint32_t timeout_ms) noexcept {
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
  NarrowPort port;
  dynamixel::DYNAMIXEL<NarrowPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
