/**
 * @file test_model.cpp
 * @brief Configuration checks, the control-table catalogue and the error names.
 */
#include <array>
#include <cstdint>
#include <string_view>

#include "dynamixel.hpp"
#include "test_harness.hpp"

using namespace dynamixel;

TEST_CASE("Config defaults and presets validate; each bad field is refused") {
  CHECK_EQ(Validate(Config{}), Error::Ok);
  CHECK_EQ(Validate(FastBusPreset()), Error::Ok);
  CHECK_EQ(Validate(UsbAdapterPreset()), Error::Ok);
  CHECK_EQ(Validate(LegacyBusPreset()), Error::Ok);
  CHECK(LegacyBusPreset().protocol == Protocol::V1);
  Config c{};
  c.baud = 9'599;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.baud = kMaxBaud + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.reply_timeout_us = kMinTimeoutUs - 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.gap_timeout_us = kMaxTimeoutUs + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.broadcast_timeout_us = 0;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.reboot_wait_us = kMaxSettleUs + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.baud_change_wait_us = kMaxSettleUs + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  c.retries = kMaxRetries + 1U;
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  c.protocol = static_cast<Protocol>(7);
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  c.echo = static_cast<EchoMode>(7);
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  c.alert_policy = static_cast<AlertPolicy>(7);
  CHECK_EQ(Validate(c), Error::InvalidArgument);
  c = Config{};
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  c.status_return = static_cast<StatusReturn>(7);
  CHECK_EQ(Validate(c), Error::InvalidArgument);
}

TEST_CASE("Every error has a name and a value that is not an enumerator is Unknown") {
  const std::array<Error, 15> all{
      Error::Ok,           Error::NotInitialized, Error::InvalidArgument,
      Error::Bus,          Error::Timeout,        Error::DeviceIdMismatch,
      Error::Crc,          Error::Busy,           Error::DeviceFault,
      Error::Unsupported,  Error::DeviceError,    Error::InvalidReply,
      Error::EchoMismatch, Error::LineFault,      Error::NotPermitted};
  for (const Error e : all) {
    CHECK(std::string_view{ToString(e)} != std::string_view{"Unknown"});
  }
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  CHECK(std::string_view{ToString(static_cast<Error>(99))} == std::string_view{"Unknown"});
  CHECK_EQ(static_cast<unsigned>(Error::Ok), 0U);
  CHECK_EQ(static_cast<unsigned>(Error::Unsupported), 9U);
  CHECK_EQ(static_cast<unsigned>(Error::DeviceError), 16U);
}

TEST_CASE("Result carries a value or an error") {
  const auto ok = Result<int>::Success(5);
  CHECK(ok.ok());
  CHECK(static_cast<bool>(ok));
  CHECK_EQ(ok.value, 5);
  const auto bad = Result<int>::Failure(Error::Timeout);
  CHECK(!bad.ok());
  CHECK_EQ(bad.value, 0);
  CHECK(Result<void>::Success().ok());
  CHECK(!Result<void>::Failure(Error::Bus).ok());
}

TEST_CASE("Models map to families; unknown numbers are Generic") {
  CHECK(FamilyOf(1020) == Family::X430);
  CHECK(FamilyOf(1060) == Family::X430Basic);
  CHECK(FamilyOf(12) == Family::AxRxDx);
  CHECK(FamilyOf(1190) == Family::X330);
  CHECK(FamilyOf(350) == Family::Xl320);
  CHECK(FamilyOf(65535) == Family::Generic);
  CHECK(FindModel(1020) != nullptr);
  CHECK(FindModel(60000) == nullptr);
}

TEST_CASE("Every family table is sorted, non-overlapping and has the common registers") {
  for (uint8_t f = 1; f <= static_cast<uint8_t>(Family::Xw); ++f) {
    const auto fam = static_cast<Family>(f);
    const auto table = Table(fam);
    CHECK(!table.empty());
    uint32_t end = 0;
    for (const RegInfo& r : table) {
      CHECK(r.address >= end);
      CHECK((r.size == 1U) || (r.size == 2U) || (r.size == 4U));
      end = static_cast<uint32_t>(r.address) + r.size;
    }
    CHECK(Lookup(fam, Reg::ModelNumber) != nullptr);
    CHECK(Lookup(fam, Reg::TorqueEnable) != nullptr);
    CHECK(Lookup(fam, Reg::GoalPosition) != nullptr);
    CHECK(Lookup(fam, Reg::PresentPosition) != nullptr);
    CHECK(Lookup(fam, Reg::Id) != nullptr);
  }
  CHECK(Table(Family::Generic).empty());
  CHECK(Lookup(Family::Generic, Reg::Id) == nullptr);
}

TEST_CASE("Register addresses match the e-Manual for the X430 and AX tables") {
  CHECK_EQ(Lookup(Family::X430, Reg::Id)->address, 7U);
  CHECK_EQ(Lookup(Family::X430, Reg::TorqueEnable)->address, 64U);
  CHECK_EQ(Lookup(Family::X430, Reg::StatusReturnLevel)->address, 68U);
  CHECK_EQ(Lookup(Family::X430, Reg::GoalPosition)->address, 116U);
  CHECK_EQ(Lookup(Family::X430, Reg::GoalPosition)->size, 4U);
  CHECK_EQ(Lookup(Family::X430, Reg::PresentPosition)->address, 132U);
  CHECK_EQ(Lookup(Family::X430, Reg::PresentTemperature)->address, 146U);
  CHECK_EQ(Lookup(Family::AxRxDx, Reg::Id)->address, 3U);
  CHECK_EQ(Lookup(Family::AxRxDx, Reg::TorqueEnable)->address, 24U);
  CHECK_EQ(Lookup(Family::AxRxDx, Reg::GoalPosition)->address, 30U);
  CHECK_EQ(Lookup(Family::AxRxDx, Reg::PresentPosition)->address, 36U);
  CHECK_EQ(Lookup(Family::AxRxDx, Reg::StatusReturnLevel)->address, 16U);
  CHECK(Lookup(Family::AxRxDx, Reg::PresentLoad)->kind == Kind::SignMag10);
}

TEST_CASE("Baud schemes: divider, XL-320 table, X330 and X430 tables") {
  uint8_t v = 99;
  CHECK(BaudValueFor(Family::AxRxDx, 1'000'000, v));
  CHECK_EQ(v, 1U);
  CHECK(!BaudValueFor(Family::AxRxDx, 57'600, v));
  CHECK_EQ(BaudRateOf(Family::AxRxDx, 207), 9'615U);
  CHECK(BaudValueFor(Family::X430, 57'600, v));
  CHECK_EQ(v, 1U);
  CHECK(BaudValueFor(Family::X430, 4'500'000, v));
  CHECK_EQ(v, 7U);
  CHECK(!BaudValueFor(Family::X430, 123'456, v));
  CHECK(BaudValueFor(Family::Xl320, 1'000'000, v));
  CHECK_EQ(v, 3U);
  CHECK(!BaudValueFor(Family::Xl320, 4'500'000, v));
  CHECK(BaudValueFor(Family::X330, 4'000'000, v));
  CHECK_EQ(v, 6U);
  CHECK(!BaudValueFor(Family::X330, 4'500'000, v));
  CHECK(!BaudValueFor(Family::Generic, 57'600, v));
  CHECK_EQ(BaudRateOf(Family::X430, 1), 57'600U);
  CHECK_EQ(BaudRateOf(Family::X430, 9), 0U);
  CHECK_EQ(BaudRateOf(Family::Generic, 1), 0U);
}

TEST_CASE("Fast read support depends on the model and firmware") {
  CHECK(SupportsFastRead(1020, 46));
  CHECK(!SupportsFastRead(12, 25));
}
