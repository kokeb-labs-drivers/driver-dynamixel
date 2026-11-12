/**
 * @file not_a_port.cpp
 * @brief Must NOT compile: the driver is constrained by the port concept.
 *
 * EXPECT: Port
 */
#include "dynamixel.hpp"

struct NotAPort {
  bool EnsureInitialized() noexcept {
    return true;
  }
};

int main() {
  NotAPort port;
  dynamixel::DYNAMIXEL<NotAPort> servos(port);
  return static_cast<int>(servos.EnsureInitialized().error);
}
