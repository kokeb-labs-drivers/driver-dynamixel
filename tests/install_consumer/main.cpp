/**
 * @file main.cpp
 * @brief Builds the driver from the installed package only: headers, version header and .ipp.
 */
#include <cstddef>
#include <cstdint>

#include "dynamixel.hpp"

namespace {

// A port that never receives a byte: enough to instantiate the whole driver template.
class SilentPort : public dynamixel::UartInterface<SilentPort> {
public:
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    (void)data;
    (void)len;
    return true;
  }
  std::size_t Read(uint8_t* /*data*/, std::size_t /*max*/, uint32_t /*timeout_us*/) noexcept {
    return 0;
  }
  bool FlushInput() noexcept {
    return true;
  }
  void DelayUs(uint32_t us) noexcept {
    (void)us;
  }
};

} // namespace

int main() {
  SilentPort port;
  dynamixel::DYNAMIXEL<SilentPort> bus(port);
  // No reply ever arrives, so a ping must time out: proof the template compiled and ran.
  static_cast<void>(bus.EnsureInitialized());
  const auto r = bus.Ping(1);
  return (r.error == dynamixel::Error::Timeout && dynamixel::GetDriverVersion()[0] != '\0') ? 0 : 1;
}
