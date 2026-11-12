/**
 * @file dynamixel_kconfig.hpp
 * @brief Optional ESP-IDF layer: a `Config` built from the `CONFIG_DYNAMIXEL_*` Kconfig options.
 * @copyright Copyright (c) 2026 The driver-dynamixel authors. SPDX-License-Identifier: MIT
 *
 * @details
 * Include this header only where you want menuconfig to choose the defaults; the driver never
 * includes it and no driver signature depends on it. Under ESP-IDF the values come from
 * `sdkconfig.h`. Without Kconfig every option falls back to the datasheet default, so
 * `KconfigDefaults()` equals `Config{}`.
 *
 * Kconfig writes a bool option set to "n" as an absent macro, so an absent
 * `CONFIG_DYNAMIXEL_ECHO_LOOPBACK` means off in both cases.
 *
 * One configuration per program: the inline function below depends on the `CONFIG_` macros, so
 * every translation unit of one program must see the same values (ESP-IDF has one `sdkconfig.h`
 * per build; each host test is its own program). No driver or library header includes this one
 * (handbook KB-CPP20-018).
 */
#ifndef DYNAMIXEL_KCONFIG_HPP
#define DYNAMIXEL_KCONFIG_HPP

#ifdef __has_include
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#include "dynamixel_types.hpp"

#ifndef CONFIG_DYNAMIXEL_BAUD
/** Bit rate when Kconfig does not set it: 57 600, the factory rate of the X series. */
#define CONFIG_DYNAMIXEL_BAUD 57600
#endif
#ifndef CONFIG_DYNAMIXEL_REPLY_TIMEOUT_US
/** Reply timeout in microseconds when Kconfig does not set it: `Config{}.reply_timeout_us`. */
#define CONFIG_DYNAMIXEL_REPLY_TIMEOUT_US 20000
#endif
#ifndef CONFIG_DYNAMIXEL_RETRIES
/** Repeats of an idempotent instruction when Kconfig does not set it: `Config{}.retries`. */
#define CONFIG_DYNAMIXEL_RETRIES 1
#endif

namespace dynamixel {

/**
 * @brief The configuration menuconfig selected, on top of the defaults.
 * @return `Config{}` with the protocol, bit rate, reply timeout, retries and echo mode from
 * Kconfig. `allow_destructive` is never a Kconfig option: it is decided in code, call by call.
 */
constexpr Config KconfigDefaults() noexcept {
  Config c{};
#ifdef CONFIG_DYNAMIXEL_PROTOCOL_1
  c.protocol = Protocol::V1;
#else
  c.protocol = Protocol::V2;
#endif
  c.baud = static_cast<uint32_t>(CONFIG_DYNAMIXEL_BAUD);
  c.reply_timeout_us = static_cast<uint32_t>(CONFIG_DYNAMIXEL_REPLY_TIMEOUT_US);
  c.retries = static_cast<uint8_t>(CONFIG_DYNAMIXEL_RETRIES);
#if defined(CONFIG_DYNAMIXEL_ECHO_LOOPBACK) && CONFIG_DYNAMIXEL_ECHO_LOOPBACK
  c.echo = EchoMode::Loopback;
#endif
  return c;
}

} // namespace dynamixel

#endif // DYNAMIXEL_KCONFIG_HPP
