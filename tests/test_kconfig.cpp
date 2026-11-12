/**
 * @file test_kconfig.cpp
 * @brief KconfigDefaults(): the defaults without Kconfig, and menuconfig values when they are set.
 *
 * Built twice: without CONFIG_ macros (KCONFIG_SET undefined) and with the values an sdkconfig.h
 * would define (KCONFIG_SET=1, see tests/CMakeLists.txt).
 */
#include "dynamixel_kconfig.hpp"
#include "test_harness.hpp"

#ifdef KCONFIG_SET
TEST_CASE("Kconfig values reach the Config") {
  constexpr dynamixel::Config kC = dynamixel::KconfigDefaults();
  CHECK_EQ(kC.protocol, dynamixel::Protocol::V1);
  CHECK_EQ(kC.baud, 1'000'000U);
  CHECK_EQ(kC.reply_timeout_us, 5'000U);
  CHECK_EQ(kC.retries, 3U);
  CHECK_EQ(kC.echo, dynamixel::EchoMode::Loopback);
  CHECK(!kC.allow_destructive); // never a Kconfig option
  CHECK_EQ(dynamixel::Validate(kC), dynamixel::Error::Ok);
}
#else
TEST_CASE("Without Kconfig, KconfigDefaults() equals Config{}") {
  constexpr dynamixel::Config kC = dynamixel::KconfigDefaults();
  constexpr dynamixel::Config kD{};
  CHECK_EQ(kC.protocol, kD.protocol);
  CHECK_EQ(kC.baud, kD.baud);
  CHECK_EQ(kC.reply_timeout_us, kD.reply_timeout_us);
  CHECK_EQ(kC.retries, kD.retries);
  CHECK_EQ(kC.echo, kD.echo);
  CHECK_EQ(kC.allow_destructive, kD.allow_destructive);
}
#endif
