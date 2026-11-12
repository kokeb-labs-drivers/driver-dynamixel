/**
 * @file esp32_dynamixel_port.hpp
 * @brief ESP-IDF port for the DYNAMIXEL driver: the UART driver in RS-485 half-duplex mode (the RTS
 * pin is the direction pin of the line stage), esp_rom delays and ESP_LOG for the log hook.
 *
 * Start from this file for any ESP-IDF target. The pins are in Esp32DynamixelPins; the defaults
 * match the wiring table in examples/esp32/README.md (ESP32-C6 DevKitC-1). Not verified on
 * hardware.
 */
#ifndef DYNAMIXEL_ESP32_DYNAMIXEL_PORT_HPP
#define DYNAMIXEL_ESP32_DYNAMIXEL_PORT_HPP

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dynamixel.hpp"

/// The receive buffer of the UART driver: the largest packet the driver reads (512 bytes) and some
/// slack.
inline constexpr int kEsp32DynamixelRxBytes = 1024;
/// The bus rate of the port: the factory rate of the X series.
inline constexpr uint32_t kEsp32DynamixelBaud = dynamixel::kFactoryBaudRate;
static_assert(kEsp32DynamixelBaud <= dynamixel::kMaxBaudRate,
              "the baud rate is above what the servos run at");
static_assert(kEsp32DynamixelRxBytes >= 512,
              "the receive buffer must hold the driver's largest packet");

/// Pins and bus settings of the ESP-IDF port.
struct Esp32DynamixelPins {
  uart_port_t port = UART_NUM_1;
  int tx = 4;        ///< To the line stage's transmit input.
  int rx = 5;        ///< From the line stage's receive output.
  int direction = 6; ///< RTS: high while transmitting. -1 for a single wire with the echo checked.
  uint32_t baud = kEsp32DynamixelBaud;
};

/// The DYNAMIXEL UART port on ESP-IDF, with the optional baud and log hooks.
class Esp32DynamixelPort : public dynamixel::UartInterface<Esp32DynamixelPort> {
public:
  explicit Esp32DynamixelPort(const Esp32DynamixelPins& pins = {}) noexcept : pins_(pins) {}

  bool EnsureInitialized() noexcept {
    if (ready_) {
      return true;
    }
    uart_config_t cfg{};
    cfg.baud_rate = static_cast<int>(pins_.baud);
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    const int rts = (pins_.direction >= 0) ? pins_.direction : UART_PIN_NO_CHANGE;
    if ((uart_driver_install(pins_.port, kEsp32DynamixelRxBytes, 0, 0, nullptr, 0) != ESP_OK) ||
        (uart_param_config(pins_.port, &cfg) != ESP_OK) ||
        (uart_set_pin(pins_.port, pins_.tx, pins_.rx, rts, UART_PIN_NO_CHANGE) != ESP_OK)) {
      return false;
    }
    // The hardware raises RTS with the first start bit and drops it after the last stop bit, so the
    // line is turned round without a gap and without the CPU (e-Manual, Tx, Rx Direction).
    if ((pins_.direction >= 0) &&
        (uart_set_mode(pins_.port, UART_MODE_RS485_HALF_DUPLEX) != ESP_OK)) {
      return false;
    }
    ready_ = true;
    return true;
  }

  // The whole packet in one call, then until the last stop bit has left the shift register.
  bool Write(const uint8_t* data, std::size_t len) noexcept {
    if (uart_write_bytes(pins_.port, data, len) != static_cast<int>(len)) {
      return false;
    }
    const uint32_t bits = static_cast<uint32_t>(len) * 10U;
    const uint32_t ms = (bits * 1000U) / pins_.baud + 5U;
    return uart_wait_tx_done(pins_.port, pdMS_TO_TICKS(ms)) == ESP_OK;
  }

  // 1 to `max` bytes as soon as the first has arrived: wait for one, then take what is already
  // buffered.
  std::size_t Read(uint8_t* data, std::size_t max, uint32_t timeout_us) noexcept {
    const uint32_t wait_ms = (timeout_us + 999U) / 1000U;
    TickType_t ticks = pdMS_TO_TICKS(wait_ms);
    if (ticks == 0U) {
      ticks = 1U;
    }
    if (uart_read_bytes(pins_.port, data, 1U, ticks) != 1) {
      return 0U;
    }
    std::size_t got = 1U;
    size_t buffered = 0;
    if ((max > 1U) && (uart_get_buffered_data_len(pins_.port, &buffered) == ESP_OK) &&
        (buffered > 0U)) {
      const std::size_t more = (buffered < (max - 1U)) ? buffered : (max - 1U);
      const int n = uart_read_bytes(pins_.port, data + 1, static_cast<uint32_t>(more), 0);
      if (n > 0) {
        got += static_cast<std::size_t>(n);
      }
    }
    return got;
  }

  bool FlushInput() noexcept {
    return uart_flush_input(pins_.port) == ESP_OK;
  }

  // Waits of a millisecond or more yield to the other tasks (at least one tick); only shorter ones
  // spin.
  void DelayUs(uint32_t us) noexcept {
    if (us < 1000U) {
      esp_rom_delay_us(us);
      return;
    }
    const TickType_t ticks = pdMS_TO_TICKS(us / 1000U);
    vTaskDelay((ticks > 0U) ? ticks : 1U);
  }

  bool SetBaudRate(uint32_t baud) noexcept {
    if (uart_set_baudrate(pins_.port, baud) != ESP_OK) {
      return false;
    }
    pins_.baud = baud;
    return true;
  }

  void Log(dynamixel::LogLevel level, const char* message) noexcept {
    esp_log_level_t esp_level = ESP_LOG_DEBUG;
    switch (level) {
    case dynamixel::LogLevel::Error:
      esp_level = ESP_LOG_ERROR;
      break;
    case dynamixel::LogLevel::Warning:
      esp_level = ESP_LOG_WARN;
      break;
    case dynamixel::LogLevel::Info:
      esp_level = ESP_LOG_INFO;
      break;
    case dynamixel::LogLevel::Debug:
    default:
      break;
    }
    esp_log_write(esp_level, kTag, "%s\n", message); // plain text, never a format
  }

private:
  static constexpr const char* kTag = "dynamixel";
  Esp32DynamixelPins pins_;
  bool ready_ = false;
};

#endif // DYNAMIXEL_ESP32_DYNAMIXEL_PORT_HPP
