/**
 * @file test_driver.cpp
 * @brief Lifecycle, discovery, raw read and write, reply rules by Status Return Level, reset and
 * reboot.
 */
#include <array>
#include <cstdint>

#include "rig.hpp"

using namespace dynamixel;
using namespace rig;

TEST_CASE("Constructor does no I/O; every call fails NotInitialized before EnsureInitialized") {
  mock::Bus bus{};
  mock::Port port{bus};
  static_cast<void>(bus.Add(mock::Kind::Xm430, 1));
  Driver drv{port};
  CHECK_EQ(bus.ensure_calls, 0);
  CHECK(!drv.IsInitialized());
  std::array<uint8_t, 2> buf{};
  const std::array<uint8_t, 1> one{1};
  const std::array<uint8_t, 1> ids{1};
  std::array<DeviceReply, 1> replies{};
  std::array<PingInfo, 2> found{};
  CHECK_EQ(drv.Ping(1).error, Error::NotInitialized);
  CHECK_EQ(drv.Read(1, 0, std::span<uint8_t>{buf}).error, Error::NotInitialized);
  CHECK_EQ(drv.Write(1, 64, std::span<const uint8_t>{one}).error, Error::NotInitialized);
  CHECK_EQ(drv.RegWrite(1, 64, std::span<const uint8_t>{one}).error, Error::NotInitialized);
  CHECK_EQ(drv.Action(1).error, Error::NotInitialized);
  CHECK_EQ(drv.SyncWrite(64, 1, std::span<const uint8_t>{ids}, std::span<const uint8_t>{one}).error,
           Error::NotInitialized);
  CHECK_EQ(drv.SyncRead(64, 1, std::span<const uint8_t>{ids}, std::span<uint8_t>{buf}.first(1),
                        std::span<DeviceReply>{replies})
               .error,
           Error::NotInitialized);
  CHECK_EQ(drv.BroadcastPing(std::span<PingInfo>{found}).error, Error::NotInitialized);
  CHECK_EQ(drv.ScanIds(1, 2, std::span<PingInfo>{found}).error, Error::NotInitialized);
  CHECK_EQ(drv.ReadRegister(kX1, Reg::PresentPosition).error, Error::NotInitialized);
  CHECK_EQ(drv.Resync().error, Error::NotInitialized);
  CHECK_EQ(bus.writes, 0);
  CHECK_EQ(bus.flushes, 0);
}

TEST_CASE(
    "EnsureInitialized applies the rate once and is idempotent; a failure leaves it retryable") {
  mock::Bus bus{};
  mock::Port port{bus};
  Config c{};
  c.baud = 1'000'000;
  Driver drv{port, c};
  bus.fail_ensure = true;
  CHECK_EQ(drv.EnsureInitialized().error, Error::Bus);
  CHECK(!drv.IsInitialized());
  bus.fail_ensure = false;
  bus.fail_baud = true;
  CHECK_EQ(drv.EnsureInitialized().error, Error::Bus);
  CHECK(!drv.IsInitialized());
  bus.fail_baud = false;
  CHECK(drv.EnsureInitialized().ok());
  CHECK(drv.IsInitialized());
  CHECK_EQ(bus.baud, 1'000'000U);
  const int calls = bus.ensure_calls;
  CHECK(drv.EnsureInitialized().ok());
  CHECK_EQ(bus.ensure_calls, calls);
  CHECK(drv.Deinitialize().ok());
  CHECK(!drv.IsInitialized());
  CHECK_EQ(drv.Ping(1).error, Error::NotInitialized);
}

TEST_CASE("An invalid stored configuration stops EnsureInitialized before any I/O") {
  mock::Bus bus{};
  mock::Port port{bus};
  Config c{};
  c.retries = 99;
  Driver drv{port, c};
  CHECK_EQ(drv.EnsureInitialized().error, Error::InvalidArgument);
  CHECK_EQ(bus.ensure_calls, 0);
  CHECK_EQ(drv.ApplyConfig(c).error, Error::InvalidArgument);
}

TEST_CASE("ApplyConfig changes the rate and keeps the old configuration when the port refuses") {
  V2Rig r;
  Config c{};
  c.baud = 115'200;
  CHECK(r.drv.ApplyConfig(c).ok());
  CHECK_EQ(r.bus.baud, 115'200U);
  CHECK_EQ(r.drv.GetConfig().baud, 115'200U);
  Config d{};
  d.baud = 3'000'000;
  r.bus.fail_baud = true;
  CHECK_EQ(r.drv.ApplyConfig(d).error, Error::Bus);
  CHECK_EQ(r.drv.GetConfig().baud, 115'200U);
}

TEST_CASE(
    "Ping returns model, firmware and family on protocol 2.0, and sends the e-Manual packet") {
  V2Rig r;
  const auto p = r.drv.Ping(2);
  REQUIRE(p.ok());
  CHECK_EQ(p.value.id, 2U);
  CHECK_EQ(p.value.model, 1020U);
  CHECK_EQ(p.value.firmware, 46U);
  CHECK(p.value.family == Family::X430);
  const std::array<uint8_t, 10> want{0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x01, 0, 0};
  const auto sent = r.drv.GetLastRequest();
  REQUIRE(sent.size() == 10U);
  CHECK(std::equal(want.begin(), want.begin() + 8, sent.begin()));
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Ping on protocol 1.0 carries only the ID; Identify reads the model") {
  V1Rig r;
  const auto p = r.drv.Ping(1);
  REQUIRE(p.ok());
  CHECK_EQ(p.value.id, 1U);
  CHECK_EQ(p.value.model, 0U);
  const auto i = r.drv.Identify(2);
  REQUIRE(i.ok());
  CHECK_EQ(i.value.model, 12U);
  CHECK_EQ(i.value.firmware, 25U);
  CHECK(i.value.family == Family::AxRxDx);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Ping rejects the broadcast ID and out-of-range IDs without sending") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.Ping(254).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Ping(253).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Ping(255).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  const int w1 = v.bus.writes;
  CHECK_EQ(v.drv.Ping(254).error, Error::InvalidArgument);
  CHECK(v.drv.Ping(253).error == Error::Timeout);
  CHECK_EQ(v.bus.writes, w1 + 2); // one retry for the idempotent ping
}

TEST_CASE("Ping of an absent ID times out, retries once, and counts") {
  V2Rig r;
  const auto p = r.drv.Ping(9);
  CHECK_EQ(p.error, Error::Timeout);
  CHECK_EQ(r.drv.GetStats().transactions, 2U);
  CHECK_EQ(r.drv.GetStats().retries, 1U);
  CHECK_EQ(r.drv.GetStats().timeouts, 2U);
}

TEST_CASE("Read returns the register bytes (protocol 2.0)") {
  V2Rig r;
  r.bus.Find(1)->SetDword(132, 2048);
  std::array<uint8_t, 4> out{};
  REQUIRE(r.drv.Read(1, 132, std::span<uint8_t>{out}).ok());
  CHECK_EQ(out[0], 0x00U);
  CHECK_EQ(out[1], 0x08U);
  const auto sent = r.drv.GetLastRequest();
  REQUIRE(sent.size() == 14U);
  CHECK_EQ(sent[7], 0x02U);
  CHECK_EQ(sent[8], 132U);
  CHECK_EQ(sent[10], 4U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Read returns the register bytes (protocol 1.0)") {
  V1Rig r;
  r.bus.Find(1)->SetWord(36, 512);
  std::array<uint8_t, 2> out{};
  REQUIRE(r.drv.Read(1, 36, std::span<uint8_t>{out}).ok());
  CHECK_EQ(out[0], 0x00U);
  CHECK_EQ(out[1], 0x02U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Read: invalid arguments send nothing") {
  V2Rig r;
  std::array<uint8_t, 4> out{};
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.Read(254, 0, std::span<uint8_t>{out}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Read(1, 0, std::span<uint8_t>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Read(253, 0, std::span<uint8_t>{out}).error, Error::InvalidArgument);
  std::array<uint8_t, 600> huge{};
  CHECK_EQ(r.drv.Read(1, 0, std::span<uint8_t>{huge}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  const int w1 = v.bus.writes;
  CHECK_EQ(v.drv.Read(1, 256, std::span<uint8_t>{out}).error, Error::InvalidArgument);
  CHECK_EQ(v.bus.writes, w1);
}

TEST_CASE(
    "Read of an address the servo does not have is DeviceError with the error number latched") {
  V2Rig r;
  std::array<uint8_t, 4> out{};
  const auto e = r.drv.Read(1, 1022, std::span<uint8_t>{out});
  CHECK_EQ(e.error, Error::DeviceError);
  CHECK_EQ(r.drv.GetLastReply().error_byte, 7U);
  CHECK(r.drv.GetLastReply().rejected);
  CHECK_EQ(r.drv.GetStats().device_errors, 1U);
  CHECK_EQ(r.drv.GetStats().retries, 0U); // a servo's refusal is not repeated
}

TEST_CASE("Write, then read it back; the packet is the e-Manual's layout") {
  V2Rig r;
  const std::array<uint8_t, 4> pos{0x00, 0x08, 0x00, 0x00};
  REQUIRE(r.drv.Write(1, 116, std::span<const uint8_t>{pos}).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), 2048);
  CHECK_EQ(r.bus.bad_packets, 0);
  const auto sent = r.drv.GetLastRequest();
  REQUIRE(sent.size() == 16U);
  CHECK_EQ(sent[7], 0x03U);
  CHECK_EQ(sent[8], 116U);
}

TEST_CASE("Write to a read-only register and to EEPROM with torque on are refused by the servo") {
  V2Rig r;
  const std::array<uint8_t, 2> v{1, 0};
  CHECK_EQ(r.drv.Write(1, 132, std::span<const uint8_t>{v}).error, Error::DeviceError);
  CHECK_EQ(r.drv.GetLastReply().error_byte, 7U);
  const std::array<uint8_t, 1> on{1};
  REQUIRE(r.drv.Write(1, 64, std::span<const uint8_t>{on}).ok());
  const std::array<uint8_t, 1> id{5};
  CHECK_EQ(r.drv.Write(1, 7, std::span<const uint8_t>{id}).error, Error::DeviceError);
  CHECK_EQ(r.bus.Find(1)->id, 1U);
}

TEST_CASE("Writes at status return level 1 and 0 are not waited for, reads follow the level") {
  V2Rig r;
  r.bus.Find(1)->mem[68] = 1;
  Config c{};
  c.status_return = StatusReturn::ReadOnly;
  REQUIRE(r.drv.ApplyConfig(c).ok());
  const std::array<uint8_t, 1> on{1};
  const auto tx = r.drv.GetStats().transactions;
  CHECK(r.drv.Write(1, 64, std::span<const uint8_t>{on}).ok());
  CHECK_EQ(r.drv.GetStats().timeouts, 0U);
  CHECK_EQ(r.bus.Find(1)->mem[64], 1U);
  CHECK_EQ(r.drv.GetStats().transactions, tx + 1U);
  std::array<uint8_t, 1> out{};
  CHECK(r.drv.Read(1, 64, std::span<uint8_t>{out}).ok());
  c.status_return = StatusReturn::PingOnly;
  r.bus.Find(1)->mem[68] = 0;
  REQUIRE(r.drv.ApplyConfig(c).ok());
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.Read(1, 64, std::span<uint8_t>{out}).error, Error::Unsupported);
  CHECK_EQ(r.bus.writes, w); // nothing sent: the servo would not answer
  CHECK(r.drv.Ping(1).ok());
}

TEST_CASE("A write to the broadcast ID sends and does not wait") {
  V2Rig r;
  const std::array<uint8_t, 1> on{1};
  REQUIRE(r.drv.Write(254, 64, std::span<const uint8_t>{on}).ok());
  CHECK_EQ(r.bus.Find(1)->mem[64], 1U);
  CHECK_EQ(r.bus.Find(2)->mem[64], 1U);
  CHECK_EQ(r.bus.Find(3)->mem[64], 1U);
  CHECK_EQ(r.drv.GetStats().timeouts, 0U);
  CHECK_EQ(r.bus.Queued(), 0U);
}

TEST_CASE("Reading with the broadcast ID is refused without sending") {
  V2Rig r;
  std::array<uint8_t, 1> out{};
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.Read(254, 64, std::span<uint8_t>{out}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
}

TEST_CASE(
    "Reg Write holds the value until Action; Action without Reg Write is an instruction error") {
  V2Rig r;
  const std::array<uint8_t, 4> pos{0x00, 0x04, 0x00, 0x00};
  REQUIRE(r.drv.RegWrite(1, 116, std::span<const uint8_t>{pos}).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), 0);
  REQUIRE(r.drv.Action(1).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), 1024);
  CHECK_EQ(r.drv.Action(1).error, Error::DeviceError);
  CHECK_EQ(r.drv.GetLastReply().error_byte, 2U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Broadcast Reg Write and Action move every servo together") {
  V2Rig r;
  const std::array<uint8_t, 4> pos{0x00, 0x02, 0x00, 0x00};
  REQUIRE(r.drv.RegWrite(254, 116, std::span<const uint8_t>{pos}).ok());
  CHECK_EQ(r.bus.Find(2)->Dword(116), 0);
  REQUIRE(r.drv.Action(254).ok());
  CHECK_EQ(r.bus.Find(1)->Dword(116), 512);
  CHECK_EQ(r.bus.Find(3)->Dword(116), 512);
}

TEST_CASE("Protocol 1.0 write and Reg Write") {
  V1Rig r;
  const std::array<uint8_t, 2> pos{0x00, 0x02};
  REQUIRE(r.drv.Write(1, 30, std::span<const uint8_t>{pos}).ok());
  CHECK_EQ(r.bus.Find(1)->Word(30), 512);
  REQUIRE(r.drv.RegWrite(2, 30, std::span<const uint8_t>{pos}).ok());
  CHECK_EQ(r.bus.Find(2)->Word(30), 0);
  REQUIRE(r.drv.Action(254).ok());
  CHECK_EQ(r.bus.Find(2)->Word(30), 512);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Writes with an invalid address, length or ID send nothing") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.Write(1, 64, std::span<const uint8_t>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Write(253, 64, std::span<const uint8_t>{}).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.Write(255, 64, std::span<const uint8_t>{}).error, Error::InvalidArgument);
  std::array<uint8_t, 600> huge{};
  CHECK_EQ(r.drv.Write(1, 64, std::span<const uint8_t>{huge}).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  V1Rig v;
  const std::array<uint8_t, 1> one{1};
  CHECK_EQ(v.drv.Write(1, 300, std::span<const uint8_t>{one}).error, Error::InvalidArgument);
  CHECK_EQ(v.bus.writes, 0);
}

TEST_CASE("Writing FF FF FD in the data is stuffed and the servo sees the original bytes") {
  V2Rig r;
  const std::array<uint8_t, 4> data{0xFF, 0xFF, 0xFD, 0x01};
  REQUIRE(r.drv.Write(1, 104, std::span<const uint8_t>{data}).ok());
  CHECK_EQ(r.bus.Find(1)->mem[104], 0xFFU);
  CHECK_EQ(r.bus.Find(1)->mem[105], 0xFFU);
  CHECK_EQ(r.bus.Find(1)->mem[106], 0xFDU);
  CHECK_EQ(r.bus.Find(1)->mem[107], 0x01U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("A reply whose data contains FF FF FD is destuffed") {
  V2Rig r;
  r.bus.Find(1)->mem[104] = 0xFF;
  r.bus.Find(1)->mem[105] = 0xFF;
  r.bus.Find(1)->mem[106] = 0xFD;
  r.bus.Find(1)->mem[107] = 0x09;
  std::array<uint8_t, 4> out{};
  REQUIRE(r.drv.Read(1, 104, std::span<uint8_t>{out}).ok());
  CHECK_EQ(out[2], 0xFDU);
  CHECK_EQ(out[3], 0x09U);
}

TEST_CASE("Reboot, factory reset: reply, settle wait, and what the servo did") {
  Config c{};
  c.allow_destructive = true;
  V2Rig r{c};
  const std::array<uint8_t, 1> on{1};
  REQUIRE(r.drv.Write(1, 64, std::span<const uint8_t>{on}).ok());
  REQUIRE(r.drv.Reboot(1).ok());
  CHECK_EQ(r.bus.Find(1)->reboots, 1U);
  CHECK_EQ(r.bus.Find(1)->mem[64], 0U);
  CHECK_EQ(r.bus.delay_us, c.reboot_wait_us);
  REQUIRE(r.drv.FactoryReset(2, ResetOption::KeepIdAndBaud).ok());
  CHECK_EQ(r.bus.Find(2)->last_reset_option, 0x02U);
  REQUIRE(r.drv.FactoryReset(3, ResetOption::All).ok());
  CHECK_EQ(r.bus.Find(1)->id, 1U);
  CHECK_EQ(r.bus.bad_packets, 0);
}

TEST_CASE("Factory reset and restore need allow_destructive and send nothing without it") {
  V2Rig r;
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.FactoryReset(1, ResetOption::All).error, Error::NotPermitted);
  CHECK_EQ(r.drv.FactoryReset(1, ResetOption::KeepId).error, Error::NotPermitted);
  CHECK_EQ(r.drv.Backup(1, BackupOption::Restore).error, Error::NotPermitted);
  CHECK_EQ(r.drv.Clear(1, ClearOption::MultiTurn).error, Error::NotPermitted);
  CHECK_EQ(r.bus.writes, w);
  // Store is not destructive.
  CHECK(r.drv.Backup(1, BackupOption::Store).ok());
  CHECK_EQ(r.bus.Find(1)->backups, 1U);
}

TEST_CASE("Factory reset refusals: broadcast with All, unknown option, protocol 1.0 option") {
  Config c{};
  c.allow_destructive = true;
  V2Rig r{c};
  const int w = r.bus.writes;
  CHECK_EQ(r.drv.FactoryReset(254, ResetOption::All).error, Error::InvalidArgument);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(r.drv.FactoryReset(1, static_cast<ResetOption>(0x55)).error, Error::InvalidArgument);
  CHECK_EQ(r.drv.FactoryReset(200 + 53, ResetOption::KeepId).error, Error::InvalidArgument);
  CHECK_EQ(r.bus.writes, w);
  CHECK(r.drv.FactoryReset(254, ResetOption::KeepIdAndBaud).ok());
  auto d = Config{};
  d = LegacyBusPreset();
  d.allow_destructive = true;
  V1Rig v{d};
  CHECK_EQ(v.drv.FactoryReset(1, ResetOption::KeepId).error, Error::InvalidArgument);
  CHECK_EQ(v.drv.FactoryReset(254, ResetOption::All).error, Error::InvalidArgument);
  CHECK(v.drv.FactoryReset(1, ResetOption::All).ok());
  CHECK_EQ(v.bus.Find(1)->factory_resets, 1U);
  CHECK_EQ(v.bus.bad_packets, 0);
}

TEST_CASE("Clear and Backup carry the e-Manual's confirmation bytes") {
  Config c{};
  c.allow_destructive = true;
  V2Rig r{c};
  REQUIRE(r.drv.Clear(1, ClearOption::MultiTurn).ok());
  REQUIRE(r.drv.Clear(1, ClearOption::Errors).ok());
  CHECK_EQ(r.bus.Find(1)->clears, 2U);
  REQUIRE(r.drv.Backup(1, BackupOption::Restore).ok());
  CHECK_EQ(r.bus.Find(1)->restores, 1U);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(r.drv.Clear(1, static_cast<ClearOption>(9)).error, Error::InvalidArgument);
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK_EQ(r.drv.Backup(1, static_cast<BackupOption>(9)).error, Error::InvalidArgument);
  V1Rig v;
  CHECK_EQ(v.drv.Clear(1, ClearOption::MultiTurn).error, Error::Unsupported);
  CHECK_EQ(v.drv.Backup(1, BackupOption::Store).error, Error::Unsupported);
}

TEST_CASE("The line is flushed before every instruction, and the buffers are driver members") {
  V2Rig r;
  const int f = r.bus.flushes;
  CHECK(r.drv.Ping(1).ok());
  CHECK_EQ(r.bus.flushes, f + 1);
  static_assert(sizeof(Driver) > 1024U, "transmit and receive buffers live inside the driver");
}

TEST_CASE("Resync drains a line that is talking and succeeds when it goes quiet") {
  V2Rig r;
  for (int i = 0; i < 10; ++i) {
    r.bus.Push(0x42);
  }
  CHECK(r.drv.Resync().ok());
  CHECK_EQ(r.bus.Queued(), 0U);
  r.bus.fail_flush = true;
  CHECK_EQ(r.drv.Resync().error, Error::Bus);
}
