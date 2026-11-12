/**
 * @file main.cpp
 * @brief The DYNAMIXEL driver on your PC: find the servos on a simulated bus, move one and read the
 * feedback of both with a single Sync Read.
 *
 * No hardware and no SDK: the host port (host_dynamixel_port.hpp) connects the real driver to the
 * bus model the host tests use. The same calls run on an MCU with that MCU's port (examples/esp32/
 * for ESP-IDF). Exit code 0 when every call succeeded; the output is in expected_output.txt.
 */
#include <array>
#include <cstdint>
#include <cstdio>

#include "dynamixel.hpp"
#include "host_dynamixel_port.hpp"

namespace {
int Fail(const char* what, dynamixel::Error error) {
  std::printf("%s failed: %s\n", what, dynamixel::ToString(error));
  return 1;
}
} // namespace

int main() {
  HostDynamixelPort port;
  static_cast<void>(port.bus.Add(mock::Kind::Xm430, 1));
  static_cast<void>(port.bus.Add(mock::Kind::Xm430, 2));
  port.bus.Find(1)->mem[146] = 31; // temperature of servo 1 in degrees Celsius

  dynamixel::DYNAMIXEL<HostDynamixelPort> servos(port); // no I/O yet
  std::printf("DYNAMIXEL driver %s on the host\n", dynamixel::GetDriverVersion());
  if (const auto r = servos.EnsureInitialized(); !r) {
    return Fail("EnsureInitialized", r.error);
  }

  std::array<dynamixel::PingInfo, 4> found{};
  const auto count = servos.BroadcastPing(std::span<dynamixel::PingInfo>{found});
  if (!count) {
    return Fail("BroadcastPing", count.error);
  }
  for (uint32_t i = 0; i < count.value; ++i) {
    std::printf("servo %u: model %u, firmware %u\n", static_cast<unsigned>(found[i].id),
                static_cast<unsigned>(found[i].model), static_cast<unsigned>(found[i].firmware));
  }

  const dynamixel::ServoRef first{.id = found[0].id, .family = found[0].family};
  if (const auto r = servos.SetTorque(first, true); !r) {
    return Fail("SetTorque", r.error);
  }
  if (const auto r = servos.SetGoalPosition(first, 2048); !r) {
    return Fail("SetGoalPosition", r.error);
  }

  const std::array<uint8_t, 2> ids{found[0].id, found[1].id};
  std::array<uint8_t, 2> temperatures{};
  std::array<dynamixel::DeviceReply, 2> replies{};
  const auto sync =
      servos.SyncRead(146, 1, std::span<const uint8_t>{ids}, std::span<uint8_t>{temperatures},
                      std::span<dynamixel::DeviceReply>{replies});
  if (!sync) {
    return Fail("SyncRead", sync.error);
  }
  for (std::size_t i = 0; i < ids.size(); ++i) {
    std::printf("servo %u: %u C\n", static_cast<unsigned>(ids[i]),
                static_cast<unsigned>(temperatures[i]));
  }

  if (const auto r = servos.DisableTorqueAll(dynamixel::Family::X430); !r) {
    return Fail("DisableTorqueAll", r.error);
  }
  std::printf("torque off\n");
  return 0;
}
