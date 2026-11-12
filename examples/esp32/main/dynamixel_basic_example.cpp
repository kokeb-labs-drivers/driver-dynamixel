/**
 * @file dynamixel_basic_example.cpp
 * @brief Find the servos on the bus and log the feedback of each twice a second. The torque stays
 * off: the example never moves a servo.
 *
 * Wiring: examples/esp32/README.md. Expected output: one "initialised" line, one line per servo
 * found, then a feedback line per servo every 500 ms. Every Result is checked and logged by name.
 */
#include <array>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dynamixel.hpp"
#include "dynamixel_kconfig.hpp"
#include "esp32_dynamixel_port.hpp"

namespace {
constexpr const char* kTag = "dynamixel_example";
constexpr std::size_t kMaxServos = 8;
} // namespace

extern "C" void app_main(void) {
  static Esp32DynamixelPort port; // static: the driver keeps a reference
  static dynamixel::DYNAMIXEL<Esp32DynamixelPort> servos(
      port, dynamixel::KconfigDefaults()); // no I/O yet

  ESP_LOGI(kTag, "DYNAMIXEL driver %s", dynamixel::GetDriverVersion());
  const auto init = servos.EnsureInitialized();
  if (!init) {
    ESP_LOGE(kTag, "EnsureInitialized failed: %s", dynamixel::ToString(init.error));
    return;
  }
  ESP_LOGI(kTag, "initialised");

  // Protocol 2.0 servos answer the broadcast ping in ID order; protocol 1.0 buses scan the ID
  // range.
  static std::array<dynamixel::PingInfo, kMaxServos> found{};
  const bool v2 = servos.GetConfig().protocol == dynamixel::Protocol::V2;
  const auto scan = v2 ? servos.BroadcastPing(std::span<dynamixel::PingInfo>{found})
                       : servos.ScanIds(1, 32, std::span<dynamixel::PingInfo>{found});
  if (!scan) {
    ESP_LOGE(kTag, "scan failed: %s", dynamixel::ToString(scan.error));
    return;
  }
  for (uint32_t i = 0; i < scan.value; ++i) {
    const auto identity = servos.Identify(found[i].id); // protocol 1.0 needs the read for the model
    if (!identity) {
      ESP_LOGW(kTag, "servo %u: identify failed: %s", static_cast<unsigned>(found[i].id),
               dynamixel::ToString(identity.error));
      continue;
    }
    found[i] = identity.value;
    const dynamixel::ModelInfo* model = dynamixel::FindModel(found[i].model);
    ESP_LOGI(kTag, "servo %u: %s (model %u)", static_cast<unsigned>(found[i].id),
             (model != nullptr) ? model->name : "unlisted", static_cast<unsigned>(found[i].model));
  }

  while (true) {
    for (uint32_t i = 0; i < scan.value; ++i) {
      const dynamixel::ServoRef servo{found[i].id, found[i].family};
      const auto fb = servos.ReadFeedback(servo);
      if (fb) {
        ESP_LOGI(kTag, "servo %u: position %d, %u.%u V, %u C", static_cast<unsigned>(servo.id),
                 static_cast<int>(fb.value.position),
                 static_cast<unsigned>(fb.value.voltage_dv / 10U),
                 static_cast<unsigned>(fb.value.voltage_dv % 10U),
                 static_cast<unsigned>(fb.value.temperature_c));
      } else {
        ESP_LOGW(kTag, "servo %u: read failed: %s", static_cast<unsigned>(servo.id),
                 dynamixel::ToString(fb.error));
        if (fb.error == dynamixel::Error::LineFault) {
          static_cast<void>(
              servos.Resync()); // the line was stuck: wait until it is quiet before the next call
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
