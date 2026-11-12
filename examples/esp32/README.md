# DYNAMIXEL ESP32 examples

ESP-IDF applications that run the DYNAMIXEL driver on an ESP32-C6, built by CI on every pull request. They
have not been run against a servo yet.

## Wiring (ESP32-C6 DevKitC-1)

The servo bus is one half-duplex data line. Use a line stage (a tri-state buffer for TTL servos, an RS-485
transceiver for RS-485 servos) and let the UART drive its direction pin, see
[hardware](../../docs/hardware.md).

| ESP32-C6 | Line stage | Notes |
| --- | --- | --- |
| GPIO4 (TX) | Transmit input | UART1, 57600 8N1 (the factory rate) |
| GPIO5 (RX) | Receive output | |
| GPIO6 (RTS) | Direction | High while transmitting; set `direction = -1` in `Esp32DynamixelPins` for a single wire with the echo checked |
| GND | GND | Common with the servo supply |

The servos run from their own supply (10.0 to 14.8 V for the XM430-W350), never from the 3V3 pin.

## Build and flash

```bash
git submodule update --init scripts      # the ESP-IDF build scripts
. $IDF_PATH/export.sh                     # ESP-IDF v5.5
./scripts/build_app.sh list
./scripts/build_app.sh dynamixel_basic_example Debug
./scripts/flash_app.sh flash_monitor dynamixel_basic_example Debug
```

`idf.py menuconfig` shows "DYNAMIXEL servo bus" under Component config: the packet protocol, the bus rate, the reply
timeout, the retries and the echo setting that `dynamixel::KconfigDefaults()` returns.

## Expected output

```text
I (312) dynamixel_example: DYNAMIXEL driver 0.1.0
I (318) dynamixel_example: initialised
I (340) dynamixel_example: servo 1: XM430-W350 (model 1020)
I (845) dynamixel_example: servo 1: position 2048, 12.0 V, 31 C
```

The values are an illustration. The torque stays off and the example never moves a servo.

A `scan failed: Timeout` line means nothing answered: check the servo supply, the bus rate (the servo's Baud
Rate register must match `DYNAMIXEL_BAUD`), the protocol and the direction pin first.

## Files

| File | What it is |
| --- | --- |
| [`main/dynamixel_basic_example.cpp`](main/dynamixel_basic_example.cpp) | The example app |
| [`main/esp32_dynamixel_port.hpp`](main/esp32_dynamixel_port.hpp) | The ESP-IDF port: UART in RS-485 half-duplex mode, delay, baud, log |
| [`CMakeLists.txt`](CMakeLists.txt) | The ESP-IDF project; `EXTRA_COMPONENT_DIRS` points at the repository root, which is the component |
| [`app_config.yml`](app_config.yml) | The app matrix for the scripts, CI and the docs |
