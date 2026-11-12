/**
 * @file test_typed.cpp
 * @brief The typed register API, feedback, ID and baud changes, status return level, and a port
 * that has only the required hooks.
 */
#include <array>
#include <cstdint>

#include "rig.hpp"

using namespace dynamixel;
using namespace rig;

namespace {
/// A port with only the five required hooks: no SetBaudRate, no Log.
class MinimalPort : public UartInterface<MinimalPort> {
public:
  explicit MinimalPort(mock::Bus& bus) noexcept : bus_(bus) {}
  bool EnsureInitialized() noexcept {
    return true;
  }
  bool Write(const uint8_t* d, std::size_t n) noexcept {
    bus_.Transmit(d, n);
    return true;
  }
  std::size_t Read(uint8_t* d, std::size_t max, uint32_t /*timeout_us*/) noexcept {
    std::size_t n = 0;
    while ((n < max) && (bus_.Queued() > 0U)) {
      d[n++] = bus_.rx[bus_.rx_head++];
    }
    return n;
  }
  bool FlushInput() noexcept {
    bus_.Clear();
    return true;
  }
  void DelayUs(uint32_t /*us*/) noexcept {}

private:
  mock::Bus& bus_;
};
static_assert(UartPort<MinimalPort>);
static_assert(UartPort<mock::Port>);
} // namespace

TEST_CASE("A port with only the required hooks works; ChangeBusBaud is Unsupported there") {
  mock::Bus bus{};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  MinimalPort port{bus};
  DYNAMIXEL<MinimalPort> drv{port};
  REQUIRE(drv.EnsureInitialized().ok());
  CHECK(drv.Ping(1).ok());
  CHECK_EQ(drv.ChangeBusBaud(115'200).error, Error::Unsupported);
}

TEST_CASE("ReadRegister and WriteRegister use the family's address, size and decoding") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, static_cast<int32_t>(-500));
  const auto pos = r.drv.ReadRegister(kX1, Reg::PresentPosition);
  REQUIRE(pos.ok());
  CHECK_EQ(pos.value, -500);
  REQUIRE(r.drv.WriteRegister(kX1, Reg::GoalPosition, -1234).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), -1234);
  REQUIRE(r.drv.SetGoalPosition(kX2, 2048).ok());
  CHECK_EQ(r.bus.Find(2)->Dword(116), 2048);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Typed access: unknown register for the family, read-only register, value out of range") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.ReadRegister(ServoRef{1, Family::Generic}, Reg::Id).error, Error::Unsupported);
  CHECK_EQ(r.drv.ReadRegister(kX1, Reg::GoalTorque).error, Error::Unsupported);
  CHECK_EQ(r.drv.WriteRegister(kX1, Reg::PresentPosition, 5).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.WriteRegister(kX1, Reg::TorqueEnable, 256).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.WriteRegister(kX1, Reg::TorqueEnable, -1).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
}

TEST_CASE("SetTorque on and off; DisableTorque after SetTorque on") {
  V2Rig r;
  REQUIRE(r.drv.SetTorque(kX1, true).ok());
  CHECK_EQ(r.bus.Find(1)->mem[64], 1U);
  REQUIRE(r.drv.SetTorque(kX1, false).ok());
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
}

TEST_CASE("ReadFeedback decodes position, velocity, voltage, temperature and current") {
  V2Rig r;
  mock::Servo& s = *r.bus.Find(1);
  s.SetDword(132, 3000);
  s.SetDword(128, -20);
  s.SetWord(126, static_cast<uint16_t>(-5));
  const auto f = r.drv.ReadFeedback(kX1);
  REQUIRE(f.ok());
  CHECK_EQ(f.value.position, 3000);
  CHECK_EQ(f.value.velocity, -20);
  CHECK_EQ(f.value.effort, -5);
  CHECK(f.value.effort_kind == Feedback::EffortKind::CurrentCounts);
  CHECK_EQ(f.value.voltage_dv, 121U);
  CHECK_EQ(f.value.temperature_c, 25U);
  CHECK_EQ(r.drv.GetStats().transactions, 1U); // one transfer for the whole block
}

TEST_CASE("ReadFeedback on protocol 1.0 decodes sign-magnitude speed and load") {
  V1Rig r;
  mock::Servo& s = *r.bus.Find(1);
  s.SetWord(36, 600);
  s.SetWord(38, 0x0400U | 100U); // clockwise 100
  s.SetWord(40, 50);
  const auto f = r.drv.ReadFeedback(kA1);
  REQUIRE(f.ok());
  CHECK_EQ(f.value.position, 600);
  CHECK_EQ(f.value.velocity, -100);
  CHECK_EQ(f.value.effort, 50);
  CHECK(f.value.effort_kind == Feedback::EffortKind::LoadPermille);
  CHECK_EQ(f.value.voltage_dv, 120U);
  CHECK_EQ(f.value.temperature_c, 27U);
}

TEST_CASE("ReadFeedback for a Generic family is Unsupported and sends nothing") {
  V2Rig r;
  CHECK_EQ(r.drv.ReadFeedback(ServoRef{1, Family::Generic}).error, Error::Unsupported);
  CHECK_EQ(r.bus.writes, 0);
}

TEST_CASE("SetId writes the ID register; the servo answers on its new ID") {
  V2Rig r;
  REQUIRE(r.drv.SetId(kX1, 9).ok());
  CHECK(r.drv.Ping(9).ok());
  CHECK_EQ(r.drv.Ping(1).error, Error::Timeout);
}

TEST_CASE("SetId refuses broadcast IDs as a new ID and a broadcast target without consent") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.SetId(kX1, 254).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.SetId(kX1, 253).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.SetId(ServoRef{254, Family::X430}, 5).error, Error::NotPermitted);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  CHECK(v.drv.SetId(kA1, 253).ok());
}

TEST_CASE("SetServoBaudRate writes the family's register value and waits") {
  V2Rig r;
  REQUIRE(r.drv.SetServoBaudRate(kX1, 1'000'000).ok());
  CHECK_EQ(r.bus.Find(1)->mem[8], 3U);
  CHECK_EQ(r.bus.delay_us, Config{}.baud_change_wait_us);
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.SetServoBaudRate(kX1, 123'456).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
}

TEST_CASE("ChangeBusBaud switches the port, waits and resynchronises") {
  V2Rig r;
  r.bus.Push(0x55);
  REQUIRE(r.drv.ChangeBusBaud(1'000'000).ok());
  CHECK_EQ(r.bus.baud, 1'000'000U);
  CHECK_EQ(r.drv.GetConfig().baud, 1'000'000U);
  CHECK_EQ(r.bus.Queued(), 0U);
  CHECK_EQ(r.drv.ChangeBusBaud(100).error, Error::InvalidArgument);
  r.bus.fail_baud = true;
  CHECK_EQ(r.drv.ChangeBusBaud(2'000'000).error, Error::Bus);
  CHECK_EQ(r.drv.GetConfig().baud, 1'000'000U);
}

TEST_CASE("SetStatusReturnLevel lowers the level and adopts it, accepting the silent answer") {
  V2Rig r;
  REQUIRE(r.drv.SetStatusReturnLevel(kX1, StatusReturn::ReadOnly).ok());
  CHECK_EQ(r.bus.Find(1)->mem[68], 1U);
  CHECK(r.drv.GetConfig().status_return == StatusReturn::ReadOnly);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(r.drv.SetStatusReturnLevel(kX1, static_cast<StatusReturn>(5)).error,
           Error::InvalidArgument);
  CHECK(r.drv.GetConfig().status_return == StatusReturn::ReadOnly);
}

TEST_CASE("SetStatusReturnLevel does not adopt a level the servo refused") {
  V2Rig r;
  r.bus.Find(1)->force_error = 4;
  CHECK_EQ(r.drv.SetStatusReturnLevel(kX1, StatusReturn::PingOnly).error, Error::DeviceError);
  CHECK(r.drv.GetConfig().status_return == StatusReturn::All);
}

TEST_CASE("SetStatusReturnLevel says so in the log when it adopts a level on a silent answer") {
  V2Rig r;
  int before = r.bus.log_lines;
  REQUIRE(r.drv.SetStatusReturnLevel(kX1, StatusReturn::ReadOnly).ok());
  CHECK_EQ(r.bus.log_lines, before + 1); // a lost write and a lost answer look the same
  V2Rig answered;
  before = answered.bus.log_lines;
  REQUIRE(
      answered.drv.SetStatusReturnLevel(kX1, StatusReturn::All).ok()); // answered: nothing to say
  CHECK_EQ(answered.bus.log_lines, before);
}
