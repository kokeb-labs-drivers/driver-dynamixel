/**
 * @file checked_config_invalid.cpp
 * @brief Must NOT compile: a CheckedConfig built from a configuration that Validate() rejects fails
 * the build, naming the error (handbook KB-CPP20-013).
 *
 * EXPECT: ConfigIsInvalid
 */
#include "dynamixel_types.hpp"

int main() {
  // 100 bit/s is below the lowest rate a servo supports: Validate() rejects it.
  constexpr dynamixel::CheckedConfig kBoard{dynamixel::Config{.baud = 100}};
  return static_cast<int>(kBoard.Get().baud);
}
