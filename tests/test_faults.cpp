/**
 * @file test_faults.cpp
 * @brief Reply validity, resynchronisation, retries, line faults, echo, port failures, alert
 * policy, and the guarantee that a stop is never refused.
 */
#include <array>
#include <cstdint>

#include "rig.hpp"

using namespace dynamixel;
using namespace rig;

namespace {
Error ReadTemp(V2Rig& r, uint8_t& out) {
  std::array<uint8_t, 1> b{};
  const auto res = r.drv.Read(1, 146, std::span<uint8_t>{b});
  out = b[0];
  return res.error;
}
} // namespace

TEST_CASE("A wrong CRC is retried once and counted; the retry's good answer is returned") {
  V2Rig r;
  r.bus.knob.corrupt_crc = 1;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(t, 25U);
  CHECK_EQ(r.drv.GetStats().crc_errors, 1U);
  CHECK_EQ(r.drv.GetStats().retries, 1U);
}

TEST_CASE("Persistent CRC errors end as Crc, not as a timeout") {
  V2Rig r;
  r.bus.knob.corrupt_crc = 5;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Crc);
  CHECK_EQ(r.drv.GetStats().transactions, 2U);
}

TEST_CASE("Garbage before the reply and junk after it are skipped; the next call is not confused") {
  V2Rig r;
  r.bus.knob.garbage_prefix = 7;
  r.bus.knob.trailing_junk = 5;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK(r.drv.GetStats().garbage_bytes >= 7U);
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(r.drv.GetStats().retries, 0U);
}

TEST_CASE("A reply that arrives a few bytes at a time is assembled") {
  V2Rig r;
  r.bus.knob.split_chunk = 1;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(t, 25U);
}

TEST_CASE("A truncated reply is a timeout, retried; a second truncation fails") {
  V2Rig r;
  r.bus.knob.truncate_to = 9;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(r.drv.GetStats().retries, 1U);
  r.bus.knob.drop_replies = 2;
  CHECK_EQ(ReadTemp(r, t), Error::Timeout);
}

TEST_CASE("A reply from another ID is DeviceIdMismatch and is not retried into the wrong data") {
  V2Rig r;
  r.bus.knob.wrong_id = 1;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::DeviceIdMismatch);
  CHECK_EQ(r.drv.GetStats().retries, 0U);
}

TEST_CASE("Wrong status instruction byte, bad stuffing and an extra parameter byte are "
          "InvalidReply or ignored safely") {
  V2Rig r;
  r.bus.knob.status_instruction = 1;
  uint8_t t = 0;
  // The scanner refuses a packet whose instruction byte is not 0x55: it times out, then the retry
  // works.
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  r.bus.knob.bad_stuffing = 2;
  CHECK_EQ(ReadTemp(r, t), Error::InvalidReply);
  r.bus.knob.extra_param = 2;
  CHECK_EQ(ReadTemp(r, t), Error::InvalidReply);
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
}

TEST_CASE("A line stuck low or high is LineFault, not a timeout, in both directions") {
  V2Rig r;
  r.bus.knob.stuck = 1;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::LineFault);
  CHECK(r.drv.GetStats().line_faults >= 1U);
  r.bus.knob.stuck = 2;
  CHECK_EQ(ReadTemp(r, t), Error::LineFault);
  r.bus.knob.stuck = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  V1Rig v;
  v.bus.knob.stuck = 2;
  std::array<uint8_t, 1> b{};
  CHECK_EQ(v.drv.Read(1, 43, std::span<uint8_t>{b}).error, Error::LineFault);
}

TEST_CASE("Three stray bytes of 0x00 are a timeout, not yet a line fault") {
  V2Rig r;
  r.bus.knob.stuck = 1;
  r.bus.knob.stuck_len = 3;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Timeout);
}

TEST_CASE("A Read that claims more bytes than asked is a Bus error, never trusted") {
  V2Rig r;
  r.bus.knob.oversize_read = 1;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Bus);
}

TEST_CASE("Port failures propagate: write, flush") {
  V2Rig r;
  uint8_t t = 0;
  r.bus.fail_write = true;
  CHECK_EQ(ReadTemp(r, t), Error::Bus);
  r.bus.fail_write = false;
  r.bus.fail_flush = true;
  CHECK_EQ(ReadTemp(r, t), Error::Bus);
  CHECK_EQ(r.drv.Ping(1).error, Error::Bus);
  const std::array<uint8_t, 1> on{1};
  CHECK_EQ(r.drv.Write(1, 64, std::span<const uint8_t>{on}).error, Error::Bus);
  r.bus.fail_flush = false;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
}

TEST_CASE("Writes are not retried: a lost answer is reported once") {
  V2Rig r;
  r.bus.knob.drop_replies = 1;
  const std::array<uint8_t, 1> on{1};
  CHECK_EQ(r.drv.Write(1, 64, std::span<const uint8_t>{on}).error, Error::Timeout);
  CHECK_EQ(r.drv.GetStats().transactions, 1U);
  CHECK_EQ(r.drv.GetStats().retries, 0U);
}

TEST_CASE("Retries follow the configuration: zero means one attempt") {
  Config c{};
  c.retries = 0;
  V2Rig r{c};
  r.bus.knob.drop_replies = 1;
  CHECK_EQ(r.drv.Ping(1).error, Error::Timeout);
  CHECK_EQ(r.drv.GetStats().transactions, 1U);
  Config d{};
  d.retries = 3;
  V2Rig q{d};
  q.bus.knob.drop_replies = 3;
  CHECK(q.drv.Ping(1).ok());
  CHECK_EQ(q.drv.GetStats().retries, 3U);
}

TEST_CASE("Single-wire loopback: the echo is verified and consumed before the reply") {
  Config c{};
  c.echo = EchoMode::Loopback;
  V2Rig r{c};
  r.bus.knob.echo = true;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(t, 25U);
  r.bus.knob.echo_corrupt = 2;
  CHECK_EQ(ReadTemp(r, t), Error::EchoMismatch);
  CHECK(r.drv.GetStats().echo_errors >= 1U);
  r.bus.knob.echo_drop = 2;
  CHECK_EQ(ReadTemp(r, t), Error::EchoMismatch);
  r.bus.knob.echo_drop = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  const std::array<uint8_t, 1> on{1};
  CHECK(r.drv.Write(1, 64, std::span<const uint8_t>{on}).ok());
}

TEST_CASE("Without loopback configured, an echoing bus is not mistaken for a reply") {
  V2Rig r;
  r.bus.knob.echo = true;
  uint8_t t = 0;
  // The echo is skipped by the scanner (it is an instruction packet) and the reply behind it is
  // found.
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
}

TEST_CASE("Hardware alert: reported with the data by default, latched until cleared") {
  V2Rig r;
  r.bus.Find(1)->alert = true;
  uint8_t t = 0;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK_EQ(t, 25U);
  CHECK(r.drv.GetLastReply().alert);
  CHECK_EQ(r.drv.GetLatchedAlert().error_byte, 0x80U);
  CHECK_EQ(r.drv.GetStats().alerts, 1U);
  r.bus.Find(1)->alert = false;
  CHECK_EQ(ReadTemp(r, t), Error::Ok);
  CHECK(!r.drv.GetLastReply().alert);
  CHECK_EQ(r.drv.GetLatchedAlert().error_byte, 0x80U); // still latched
  r.drv.ClearLatchedAlert();
  CHECK_EQ(r.drv.GetLatchedAlert().error_byte, 0U);
}

TEST_CASE("Alert policy Fail: DeviceFault, data discarded, and the alert is still latched") {
  Config c{};
  c.alert_policy = AlertPolicy::Fail;
  V2Rig r{c};
  r.bus.Find(1)->alert = true;
  uint8_t t = 77;
  CHECK_EQ(ReadTemp(r, t), Error::DeviceFault);
  CHECK_EQ(t, 0U);
  CHECK_EQ(r.drv.GetLatchedAlert().error_byte, 0x80U);
}

TEST_CASE(
    "Protocol 1.0 flags: a condition (overheating) is an alert, a command fault is a rejection") {
  V1Rig r;
  r.bus.Find(1)->alert = true; // 0x04, overheating
  std::array<uint8_t, 1> b{};
  CHECK(r.drv.Read(1, 43, std::span<uint8_t>{b}).ok());
  CHECK(r.drv.GetLastReply().alert);
  r.bus.Find(1)->alert = false;
  r.bus.Find(1)->force_error = 0x08; // range
  CHECK_EQ(r.drv.Read(1, 43, std::span<uint8_t>{b}).error, Error::DeviceError);
  CHECK(r.drv.GetLastReply().rejected);
}

TEST_CASE("Stats count and reset") {
  V2Rig r;
  CHECK(r.drv.Ping(1).ok());
  CHECK_EQ(r.drv.GetStats().transactions, 1U);
  r.drv.ResetStats();
  CHECK_EQ(r.drv.GetStats().transactions, 0U);
}

// ---------------------------------------------------------------------------------------------
// Stop is never refused
// ---------------------------------------------------------------------------------------------

TEST_CASE("DisableTorque works on an uninitialised driver and without allow_destructive") {
  mock::Bus bus{};
  mock::Port port{bus};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  bus.Find(1)->mem[64] = 1;
  Driver drv{port}; // never EnsureInitialized
  CHECK(drv.DisableTorque(kX1).ok());
  CHECK_EQ(bus.Find(1)->mem[64], 0U);
}

TEST_CASE("DisableTorque still works after a sequence of failures and after Deinitialize") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  r.bus.fail_write = true;
  CHECK(!r.drv.SetGoalPosition(kX1, 5).ok());
  r.bus.fail_write = false;
  CHECK(r.drv.Deinitialize().ok());
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
}

TEST_CASE("DisableTorque retries a lost answer up to three times") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  r.bus.knob.drop_replies = 2;
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.drv.GetStats().transactions, 3U);
  r.bus.knob.drop_replies = 3;
  CHECK_EQ(r.drv.DisableTorque(kX1).error, Error::Timeout);
}

TEST_CASE("DisableTorque to a servo that reports an alert succeeds; a servo that refuses is "
          "DeviceError") {
  Config c{};
  c.alert_policy = AlertPolicy::Fail;
  V2Rig r{c};
  r.bus.Find(1)->alert = true;
  CHECK(r.drv.DisableTorque(kX1).ok()); // the write was carried out; the alert is latched
  CHECK_EQ(r.drv.GetLatchedAlert().error_byte, 0x80U);
  r.bus.Find(2)->force_error = 4;
  CHECK_EQ(r.drv.DisableTorque(kX2).error, Error::DeviceError);
  CHECK_EQ(r.drv.GetStats().transactions, 2U); // the refusal was not repeated
}

TEST_CASE("DisableTorqueAll broadcasts one packet; DisableTorqueEach tries every servo") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  r.bus.Find(2)->mem[64] = 1;
  r.bus.Find(3)->mem[64] = 1;
  CHECK(r.drv.DisableTorqueAll(Family::X430).ok());
  CHECK_EQ(r.bus.Find(2)->mem[64], 0U);
  CHECK_EQ(r.bus.Find(3)->mem[64], 0U);
  r.bus.Find(1)->mem[64] = 1;
  r.bus.Find(2)->mem[64] = 1;
  r.bus.Find(3)->mem[64] = 1;
  r.bus.Find(1)->silent = true;
  const std::array<ServoRef, 3> all{kX1, kX2, kX3};
  CHECK_EQ(r.drv.DisableTorqueEach(std::span<const ServoRef>{all}).error, Error::Timeout);
  CHECK_EQ(r.bus.Find(2)->mem[64], 0U); // the failure of the first did not stop the others
  CHECK_EQ(r.bus.Find(3)->mem[64], 0U);
}

TEST_CASE("DisableTorque on protocol 1.0, with an invalid family or ID") {
  V1Rig r;
  r.bus.Find(1)->mem[24] = 1;
  CHECK(r.drv.DisableTorque(kA1).ok());
  CHECK_EQ(r.bus.Find(1)->mem[24], 0U);
  CHECK_EQ(r.drv.DisableTorque(ServoRef{1, Family::Generic}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.DisableTorque(ServoRef{255, Family::AxRxDx}).error, Error::InvalidArgument);
}

TEST_CASE("DisableTorque fails with Bus when the port cannot start; it does not hide that") {
  mock::Bus bus{};
  mock::Port port{bus};
  bus.fail_ensure = true;
  Driver drv{port};
  CHECK_EQ(drv.DisableTorque(kX1).error, Error::Bus);
}

TEST_CASE("The stop path is not affected by an invalid configuration of the driver") {
  Config c{};
  c.retries = 99;
  mock::Bus bus{};
  mock::Port port{bus};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  bus.Find(1)->mem[64] = 1;
  Driver drv{port, c};
  CHECK(drv.DisableTorque(kX1).ok());
  CHECK_EQ(bus.Find(1)->mem[64], 0U);
}

// ---------------------------------------------------------------------------------------------
// The stop path after an independent review: a failing flush, no reply to confirm, start-up order,
// and a family that does not speak the bus protocol.
// ---------------------------------------------------------------------------------------------

TEST_CASE("A stop is sent even when the input flush fails; the bus fault is reported after") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  r.bus.Find(2)->mem[64] = 1;
  r.bus.fail_flush = true;
  int before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorque(kX1).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 3); // the answer cannot be trusted, so every attempt goes out
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
  before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorqueAll(Family::X430).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 3);
  CHECK_EQ(r.bus.Find(2)->mem[64], 0U);
  r.bus.Find(2)->mem[64] = 1;
  const std::array<ServoRef, 2> both{kX1, kX2};
  before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorqueEach(std::span<const ServoRef>{both}).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 6);
  CHECK_EQ(r.bus.Find(2)->mem[64], 0U);
}

TEST_CASE("A reboot is sent even when the input flush fails") {
  V2Rig r;
  r.bus.fail_flush = true;
  const int before = r.bus.writes;
  CHECK_EQ(r.drv.Reboot(1).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 1);
  CHECK_EQ(r.bus.Find(1)->reboots, 1U);
}

TEST_CASE("A stop that no answer can confirm is sent three times, to one servo and to all") {
  for (const StatusReturn level : {StatusReturn::ReadOnly, StatusReturn::PingOnly}) {
    Config c{};
    c.status_return = level;
    V2Rig r{c};
    r.bus.Find(1)->mem[64] = 1;
    int before = r.bus.writes;
    CHECK(r.drv.DisableTorque(kX1).ok());
    CHECK_EQ(r.bus.writes - before, 3);
    CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
    before = r.bus.writes;
    CHECK(r.drv.DisableTorqueAll(Family::X430).ok());
    CHECK_EQ(r.bus.writes - before, 3);
  }
}

TEST_CASE("A stop with a confirmed answer is sent once") {
  V2Rig r;
  const int before = r.bus.writes;
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.bus.writes - before, 1);
}

TEST_CASE("A stop before EnsureInitialized sets the bus baud rate; so does a reboot") {
  Config c{};
  c.baud = 1'000'000;
  mock::Bus bus{};
  mock::Port port{bus};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  bus.Find(1)->mem[64] = 1;
  Driver drv{port, c};
  CHECK(drv.DisableTorque(kX1).ok());
  CHECK_EQ(bus.baud, 1'000'000U);
  CHECK_EQ(bus.Find(1)->mem[64], 0U);
  mock::Bus bus2{};
  mock::Port port2{bus2};
  static_cast<void>(bus2.Add(mock::Kind::Xm430, 1));
  Driver drv2{port2, c};
  CHECK(drv2.Reboot(1).ok()); // before EnsureInitialized: a stop path, never refused for that
  CHECK_EQ(bus2.baud, 1'000'000U);
  CHECK_EQ(bus2.Find(1)->reboots, 1U);
}

TEST_CASE("A family that does not speak the bus protocol is refused before anything is sent") {
  struct Row {
    Family family;
    bool v1;
    bool v2;
  };
  // Written from the e-Manual's model pages, not from the driver's table.
  const std::array<Row, 13> rows{{{Family::Generic, true, true},
                                  {Family::AxRxDx, true, false},
                                  {Family::Mx28, true, false},
                                  {Family::Mx64, true, false},
                                  {Family::Mx106, true, false},
                                  {Family::Mx28v2, false, true},
                                  {Family::Mx64v2, false, true},
                                  {Family::Xl320, false, true},
                                  {Family::X330, true, true},
                                  {Family::X430Basic, true, true},
                                  {Family::X430, true, true},
                                  {Family::X540, true, true},
                                  {Family::Xw, true, true}}};
  for (const Row& row : rows) {
    const ServoRef ref{1, row.family};
    for (const bool v2 : {true, false}) {
      const bool speaks = v2 ? row.v2 : row.v1;
      if (speaks) {
        continue;
      }
      V1Rig v1;
      V2Rig v2rig;
      mock::Bus& bus = v2 ? v2rig.bus : v1.bus;
      Driver& drv = v2 ? v2rig.drv : v1.drv;
      const int before = bus.writes;
      CHECK_EQ(drv.DisableTorque(ref).error, Error::InvalidArgument);
      CHECK_EQ(drv.DisableTorqueAll(row.family).error, Error::InvalidArgument);
      CHECK_EQ(drv.SetTorque(ref, true).error, Error::Unsupported);
      CHECK_EQ(drv.WriteRegister(ref, Reg::TorqueEnable, 1).error, Error::Unsupported);
      CHECK_EQ(drv.ReadRegister(ref, Reg::TorqueEnable).error, Error::Unsupported);
      CHECK_EQ(drv.ReadFeedback(ref).error, Error::Unsupported);
      CHECK_EQ(bus.writes, before);
    }
  }
}

TEST_CASE("A family that speaks the bus protocol is still served: stop and typed access") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  CHECK(r.drv.DisableTorque(ServoRef{1, Family::X430}).ok());
  CHECK(r.drv.ReadRegister(ServoRef{1, Family::X430}, Reg::TorqueEnable).ok());
  V1Rig v;
  const int before = v.bus.writes;
  CHECK(v.drv.DisableTorque(kA1).ok());
  CHECK(v.bus.writes > before);
}

TEST_CASE("A refusal in the answer does not end the attempts when the flush failed") {
  V2Rig r;
  r.bus.fail_flush = true;
  r.bus.Find(1)->force_error = 4; // the answer may be an old one: keep sending
  const int before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorque(kX1).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 3);
}

TEST_CASE("A flush that failed once taints the stop: the answer after it is not trusted either") {
  V2Rig r;
  r.bus.Find(1)->mem[64] = 1;
  r.bus.fail_flush_first = 1; // attempt 1 only
  const int before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorque(kX1).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 3);
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
}

TEST_CASE("A stop that no answer can confirm is Ok when one attempt reached the line") {
  Config c{};
  c.status_return = StatusReturn::ReadOnly;
  V2Rig r{c};
  r.bus.Find(1)->mem[64] = 1;
  r.bus.fail_write_after = 1; // attempt 1 is written, the next two are not
  CHECK(r.drv.DisableTorque(kX1).ok());
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
  V2Rig none{c};
  none.bus.fail_write = true; // nothing reached the line
  CHECK_EQ(none.drv.DisableTorque(kX1).error, Error::Bus);
}

TEST_CASE("DisableTorqueAt: the stop for a servo with no family, never refused for the flush") {
  const ServoRef generic{1, Family::Generic};
  V2Rig r;
  CHECK_EQ(r.drv.DisableTorque(generic).error, Error::InvalidArgument);
  r.bus.Find(1)->mem[64] = 1;
  r.bus.fail_flush = true;
  int before = r.bus.writes;
  CHECK_EQ(r.drv.DisableTorqueAt(1, 64).error, Error::Bus);
  CHECK_EQ(r.bus.writes - before, 3);
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
  r.bus.fail_flush = false;
  before = r.bus.writes;
  CHECK(r.drv.DisableTorqueAt(kBroadcastId, 64).ok());
  CHECK_EQ(r.bus.writes - before, 3); // no servo answers a broadcast: all three attempts go out
}

TEST_CASE("DisableTorqueAt before EnsureInitialized, and the arguments it refuses") {
  mock::Bus bus{};
  mock::Port port{bus};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  bus.Find(1)->mem[64] = 1;
  Driver drv{port};
  CHECK(drv.DisableTorqueAt(1, 64).ok());
  CHECK_EQ(bus.Find(1)->mem[64], 0U);
  const int before = bus.writes;
  CHECK_EQ(drv.DisableTorqueAt(253, 64).error, Error::InvalidArgument);
  V1Rig v1;
  const int v1_before = v1.bus.writes;
  CHECK_EQ(v1.drv.DisableTorqueAt(1, 256).error, Error::InvalidArgument);
  CHECK_EQ(bus.writes, before);
  CHECK_EQ(v1.bus.writes, v1_before);
  v1.bus.Find(1)->mem[24] = 1;
  CHECK(v1.drv.DisableTorqueAt(1, 24).ok());
  CHECK_EQ(v1.bus.Find(1)->mem[24], 0U);
}
