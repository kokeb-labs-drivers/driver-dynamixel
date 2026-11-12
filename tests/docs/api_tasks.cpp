/**
 * @file api_tasks.cpp
 * @brief The API guide's examples, compiled by the host build (never run).
 */
#include <array>
#include <cstdio>

#include "dynamixel.hpp"
#include "my_port.hpp"

using Servos = dynamixel::DYNAMIXEL<MyPort>;

// --8<-- [start:lifecycle]
dynamixel::Error BringUp(Servos& servos) {
  // The one entry point: starts the UART on the first call, then returns Ok with no I/O. After a
  // failure the next call starts again from the beginning.
  const auto r = servos.EnsureInitialized();
  if (!r) {
    std::printf("DYNAMIXEL init: %s\n", dynamixel::ToString(r.error));
  }
  return r.error;
}
// --8<-- [end:lifecycle]

// --8<-- [start:discover]
dynamixel::Result<uint32_t> FindServos(Servos& servos, std::span<dynamixel::PingInfo> found) {
  // Protocol 2.0: one broadcast ping, every servo answers in ID order.
  const auto all = servos.BroadcastPing(found);
  if (all.ok() && (all.value > 0U)) {
    return all;
  }
  // Protocol 1.0 has no broadcast answers: ping the IDs one by one.
  return servos.ScanIds(1, 20, found);
}
// --8<-- [end:discover]

// --8<-- [start:typed]
dynamixel::Error GoTo(Servos& servos, dynamixel::ServoRef servo, int32_t counts) {
  if (auto r = servos.SetTorque(servo, true); !r) {
    return r.error;
  }
  if (auto r = servos.SetGoalPosition(servo, counts); !r) {
    return r.error;
  }
  const auto feedback =
      servos.ReadFeedback(servo); // position, velocity, load, voltage, temperature
  return feedback.error;
}
// --8<-- [end:typed]

// --8<-- [start:sync-read]
dynamixel::Error ReadAllPositions(Servos& servos, std::span<int32_t> positions) {
  const std::array<uint8_t, 3> ids{1, 2, 3};
  std::array<uint8_t, 12> raw{};
  std::array<dynamixel::DeviceReply, 3> replies{};
  // Present Position of the X430 family: address 132, four bytes. Fast Sync Read answers in one
  // packet.
  const auto r = servos.FastSyncRead(132, 4, ids, raw, replies);
  for (std::size_t i = 0; (i < replies.size()) && (i < positions.size()); ++i) {
    if (replies[i].result == dynamixel::Error::Ok) { // each servo has its own outcome
      positions[i] = static_cast<int32_t>(static_cast<uint32_t>(raw[4U * i]) |
                                          (static_cast<uint32_t>(raw[(4U * i) + 1U]) << 8U) |
                                          (static_cast<uint32_t>(raw[(4U * i) + 2U]) << 16U) |
                                          (static_cast<uint32_t>(raw[(4U * i) + 3U]) << 24U));
    }
  }
  return r.error;
}
// --8<-- [end:sync-read]

// --8<-- [start:stop]
void EmergencyStop(Servos& servos) {
  // Never refused: no initialisation needed, no consent flag, three attempts per servo.
  // One broadcast packet; the servos do not answer a broadcast, so confirm with a read afterwards.
  static_cast<void>(servos.DisableTorqueAll(dynamixel::Family::X430));
}
// --8<-- [end:stop]

// --8<-- [start:alerts]
void CheckAlerts(Servos& servos) {
  const auto temperature =
      servos.ReadRegister({1, dynamixel::Family::X430}, dynamixel::Reg::PresentTemperature);
  if (temperature.ok() && servos.GetLastReply().alert) {
    // Bit 7 of the error byte: the servo reports a hardware error. The data is valid.
    std::printf("servo 1 alert, error byte 0x%02X\n",
                static_cast<unsigned>(servos.GetLatchedAlert().error_byte));
    servos.ClearLatchedAlert();
  }
}
// --8<-- [end:alerts]

// --8<-- [start:errors]
const char* Describe(dynamixel::Error e) {
  switch (e) {
  case dynamixel::Error::Timeout:
    return "no answer: ID, baud rate, protocol or wiring";
  case dynamixel::Error::LineFault:
    return "the line is stuck or floating: check power and the direction control";
  case dynamixel::Error::EchoMismatch:
    return "the echo differs: another talker on the bus, or the loopback setting is wrong";
  case dynamixel::Error::DeviceError:
    return "the servo refused the instruction: see GetLastReply().error_byte";
  case dynamixel::Error::NotPermitted:
    return "destructive instruction: set Config::allow_destructive on purpose";
  default:
    return dynamixel::ToString(e);
  }
}
// --8<-- [end:errors]

// --8<-- [start:reset]
dynamixel::Error ResetToFactory(MyPort& port, dynamixel::Config config) {
  config.allow_destructive =
      true; // the one switch that unlocks reset, restore and broadcast ID writes
  static Servos servos(port, config);
  return servos.FactoryReset(1, dynamixel::ResetOption::KeepIdAndBaud).error;
}
// --8<-- [end:reset]

// --8<-- [start:deinit]
void PowerDown(Servos& servos) {
  // No bus traffic: the servos keep their state. Stop them first when they must not move.
  static_cast<void>(servos.DisableTorqueAll(dynamixel::Family::X430));
  static_cast<void>(servos.Deinitialize());
}
// --8<-- [end:deinit]
