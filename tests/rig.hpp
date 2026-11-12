/**
 * @file rig.hpp
 * @brief Test fixture: a mock bus, its port and a driver on it.
 */
#ifndef DYNAMIXEL_TEST_RIG_HPP
#define DYNAMIXEL_TEST_RIG_HPP

#include <array>
#include <cstdint>

#include "dynamixel.hpp"
#include "mock_servo_bus.hpp"
#include "test_harness.hpp"

namespace rig {

using Driver = dynamixel::DYNAMIXEL<mock::Port>;

/// A bus of protocol 2.0 servos (IDs 1 to 3, XM430-style) with a driver, initialised.
struct V2Rig {
  mock::Bus bus{};
  mock::Port port{bus};
  Driver drv;
  explicit V2Rig(dynamixel::Config config = {}) : drv(port, config) {
    bus.v2 = true;
    static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
    static_cast<void>(bus.Add(mock::Kind::Xm430, 2));
    static_cast<void>(bus.Add(mock::Kind::Xm430, 3));
    static_cast<void>(drv.EnsureInitialized());
  }
};

/// A bus of protocol 1.0 servos (IDs 1 and 2, AX-12A-style) with a driver, initialised.
struct V1Rig {
  mock::Bus bus{};
  mock::Port port{bus};
  Driver drv;
  static dynamixel::Config Cfg() {
    dynamixel::Config c = dynamixel::LegacyBusPreset();
    return c;
  }
  explicit V1Rig(dynamixel::Config config = Cfg()) : drv(port, config) {
    bus.v2 = false;
    static_cast<void>(bus.Add(mock::Kind::Ax12, 1));
    static_cast<void>(bus.Add(mock::Kind::Ax12, 2));
    static_cast<void>(drv.EnsureInitialized());
  }
};

inline constexpr dynamixel::ServoRef kX1{1, dynamixel::Family::X430};
inline constexpr dynamixel::ServoRef kX2{2, dynamixel::Family::X430};
inline constexpr dynamixel::ServoRef kX3{3, dynamixel::Family::X430};
inline constexpr dynamixel::ServoRef kA1{1, dynamixel::Family::AxRxDx};

} // namespace rig

#endif // DYNAMIXEL_TEST_RIG_HPP
