# Quickstart

Run the driver on your PC first, with no hardware; then move a servo from your MCU: wire the bus, write a small
port class, call four driver functions.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    - Byte to byte time and the packet gap: ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark, 1.6 Byte to Byte Time, p. 1
    - Control table, X series (addresses, sizes, units): ROBOTIS e-Manual, XM430-W350 web page, no revision mark, 2 Control Table; 2.1 Control Table of EEPROM Area; 2.2 Control Table of RAM Area, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## Try it on your PC

[`examples/host/`](../examples/host/README.md) runs the real driver against a simulated bus of two XM430 servos,
through a host port. From the repository root, with CMake and a C++20 compiler:

```bash
cmake -S examples/host -B build-example
cmake --build build-example
./build-example/dynamixel_host_example
```

```text
DYNAMIXEL driver 0.1.0 on the host
dynamixel: initialised
servo 1: model 1020, firmware 46
servo 2: model 1020, firmware 46
servo 1: 31 C
servo 2: 25 C
torque off
```

The calls in [`main.cpp`](../examples/host/main.cpp) are the ones below; only the port differs on an MCU. CI builds
and runs this example on every pull request.

## 1. Wire it

| MCU | Servo bus | Notes |
| --- | --- | --- |
| UART TX and RX | DATA, through a half-duplex stage | 57 600 8N1 for an X-series servo at the factory settings |
| A GPIO or the UART's RTS | The stage's direction input | High while the MCU sends |
| GND | GND | Common with the servo supply |
| | VDD | 10.0 V to 14.8 V from its own supply (XM430-W350), never from the MCU board |

The [Hardware](hardware.md) page has the pins, the circuit, the bus settings and the timing. Check the servo's ID,
bit rate and protocol first (the Dynamixel Wizard of ROBOTIS shows them); the driver defaults are protocol 2.0,
57 600 bit/s and ID 1.

## 2. Add the driver

Add the repository as a submodule and link the `dynamixel::dynamixel` CMake target. [Installation](installation.md)
covers CMake on any MCU, the installed package and ESP-IDF.

```bash
git submodule add https://github.com/kokeb-labs-drivers/driver-dynamixel.git external/driver-dynamixel
```

## 3. Write the port

The port is the only platform code: it brings the UART up and moves bytes. Replace the `platform_*` calls with your
SDK's.

```cpp title="my_port.hpp"
--8<-- "tests/docs/my_port.hpp:port"
```

[Porting](porting.md) explains each method, the optional hooks and the half-duplex rules.

## 4. Move the servo

```cpp title="main.cpp"
--8<-- "tests/docs/quickstart.cpp:quickstart"
```

What each step does:

1. `EnsureInitialized()` brings the UART up through your port and applies `Config::baud`. It sends nothing on the
   bus.
2. `Ping(1)` asks servo 1 for its model number and firmware. The model number selects the control table
   (`PingInfo::family`) that the typed calls below use.
3. `SetTorque()` writes the Torque Enable register of the servo's family.
4. `SetGoalPosition()` writes the Goal Position register. The torque must be on first; position 2048 is the
   middle of the 4096 counts of an X-series servo.

This example is built by the host CI, so it compiles against the current headers.

## If it does not work

| Symptom | First thing to check |
| --- | --- |
| `EnsureInitialized()` returns `Bus` | Your port's `EnsureInitialized()`: the UART did not come up |
| `Ping()` returns `Timeout` | The servo's supply, its ID, bit rate and protocol; the direction pin |
| `Ping()` returns `LineFault` | The line is stuck low, high or floating: power, ground, the half-duplex stage |
| `Ping()` returns `EchoMismatch` | Single-wire loopback is on and the echo differs: another talker, or `Config::echo` is wrong |
| A call returns `DeviceError` | The servo refused the instruction; `GetLastReply().error_byte` says why |

[Troubleshooting](troubleshooting.md) goes through each.

## Next

- [API guide](api-guide.md): discover servos, typed registers, Sync and Fast reads, the safe stop, errors.
- [Configuration](configuration.md): presets, every `Config` field, Kconfig.
- [Examples](examples.md): the host example and the ESP32 example.
- [Platforms](platforms.md): which MCUs have a port and an example.
