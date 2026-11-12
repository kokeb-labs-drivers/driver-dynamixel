/**
 * @file configuration.cpp
 * @brief The Configuration page's examples, compiled by the host build (never run).
 */
#include "dynamixel.hpp"
#include "dynamixel_kconfig.hpp"
#include "my_port.hpp"

// --8<-- [start:custom]
dynamixel::Config LegacyConfig() {
  dynamixel::Config c = dynamixel::LegacyBusPreset(); // protocol 1.0 at 1 Mbit/s
  c.retries = 2;                                      // repeat a lost ping or read twice
  c.reply_timeout_us = 5'000;                         // a short bus answers quickly
  return c; // dynamixel::Validate(c) checks it without the bus
}

dynamixel::Error StartLegacy(MyPort& port) {
  static dynamixel::DYNAMIXEL<MyPort> servos(port, LegacyConfig()); // stored, no I/O
  return servos.EnsureInitialized().error; // InvalidArgument if the config is out of range
}
// --8<-- [end:custom]

// --8<-- [start:kconfig]
dynamixel::Error StartFromMenuconfig(MyPort& port) {
  static dynamixel::DYNAMIXEL<MyPort> servos(port, dynamixel::KconfigDefaults());
  return servos.EnsureInitialized().error;
}
// --8<-- [end:kconfig]

// --8<-- [start:checked]
// A configuration fixed at build time: an invalid value stops the build instead of the first call.
inline constexpr dynamixel::CheckedConfig kBoard{
    dynamixel::Config{.baud = 1'000'000, .retries = 2}};
// --8<-- [end:checked]
