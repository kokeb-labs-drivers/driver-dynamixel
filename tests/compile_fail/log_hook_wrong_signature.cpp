/**
 * @file log_hook_wrong_signature.cpp
 * @brief Must NOT compile: an optional hook with the wrong signature is rejected, not ignored.
 *
 * EXPECT: Log hook must be
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

class WrongLogPort : public dynamixel::UartInterface<WrongLogPort> {
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
  bool Log(dynamixel::LogLevel level, const char* message) noexcept {
    static_cast<void>(level);
    static_cast<void>(message);
    return true;
  }
};

int main() {
  WrongLogPort port;
  dynamixel::DYNAMIXEL<WrongLogPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
