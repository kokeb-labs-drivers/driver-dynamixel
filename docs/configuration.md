# Configuration

Every field of `dynamixel::Config`, the presets, how a configuration is checked, and the optional Kconfig layer.

<!-- generated: datasheet-refs -->
??? abstract "Datasheet sections this page relies on"

    From ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0 web page, no revision mark:

    - Byte to byte time and the packet gap: 1.6 Byte to Byte Time, p. 1
    - Response policy and the broadcast ID: 3.4 Response Policy, p. 1

    All of them, with the driver API for each: [Datasheet map](datasheet.md).
<!-- /generated -->

## Fields

`Config{}` suits a protocol 2.0 bus of X-series servos at their factory settings. The timing fields are driver side:
the e-Manual gives no reply deadline, so these are the driver's own choices (see the
[Contract](contract.md#open-points)). Fields with a unit carry it in the name.

| Field | Default | Range | What it does |
| --- | --- | --- | --- |
| `protocol` | `Protocol::V2` | `V1`, `V2` | Packet format of every servo on the bus |
| `baud` | 57 600 | 9 600 to 10 500 000 | UART rate; applied through the port's `SetBaudRate()` when it has one |
| `reply_timeout_us` | 20 000 | 100 to 1 000 000 | Longest wait for the first byte of a reply |
| `gap_timeout_us` | 5 000 | 100 to 1 000 000 | Longest wait for the next bytes of a packet that has started |
| `reboot_wait_us` | 500 000 | 0 to 10 000 000 | Wait after a Reboot, Restore or Factory Reset reply |
| `baud_change_wait_us` | 10 000 | 0 to 10 000 000 | Wait after a servo's baud rate changed |
| `retries` | 1 | 0 to 5 | Repeats of a ping or read after a missing, corrupted or invalid reply; writes are never repeated |
| `echo` | `EchoMode::None` | `None`, `Loopback` | Verify the echo of every sent byte on a single-wire bus |
| `alert_policy` | `AlertPolicy::Report` | `Report`, `Fail` | A hardware alert in a reply: succeed and latch it, or return `DeviceFault` |
| `status_return` | `StatusReturn::All` | `PingOnly`, `ReadOnly`, `All` | How much the servos answer; calls that need an answer the level does not give return `Unsupported` |
| `broadcast_timeout_us` | 100 000 | 100 to 1 000 000 | Wait for each reply of a broadcast ping |
| `allow_destructive` | `false` | | Permit Factory Reset, Control Table Restore, Clear and ID writes to every servo |

Choose `reply_timeout_us` longer than the servos' Return Delay Time (500 microseconds at the factory value) plus the
time the longest reply takes on the line. At 57 600 bit/s a byte takes 174 microseconds, so a 100-byte reply needs
about 17 ms; the default 20 ms covers the reads of a few registers, and a Fast Sync Read of many servos at a low
rate needs more. `gap_timeout_us` has to cover a pause inside a packet, such as the latency timer of a USB adapter.

`allow_destructive` is never set from Kconfig: it is a decision made in code, where the consequence is visible.

## Presets

| Preset | What it sets | Use it for |
| --- | --- | --- |
| `FastBusPreset()` | Reply wait 10 ms, gap 2 ms | A short bus of servos that all answer, at a high rate |
| `UsbAdapterPreset()` | Reply wait 60 ms, gap 40 ms | A bus behind a USB adapter with a 16 ms latency timer |
| `LegacyBusPreset()` | Protocol 1.0 at 1 Mbit/s | AX, RX, DX and MX (1.0) servos at their factory rate |

Start from a preset or from `Config{}` and change fields:

```cpp
--8<-- "tests/docs/configuration.cpp:custom"
```

## Validation

`dynamixel::Validate(config)` checks a configuration without touching the bus, and `EnsureInitialized()` and
`ApplyConfig()` call it before any I/O. It returns `Error::InvalidArgument` when an enum holds a value that is not
an enumerator, the baud rate or a timeout is outside its range, a wait is above ten seconds, or `retries` is above 5.

A configuration fixed at build time can be checked by the compiler instead:

```cpp
--8<-- "tests/docs/configuration.cpp:checked"
```

`CheckedConfig` does not compile for an invalid value, so a wrong number stops the build and not the first call.

## Kconfig

Under ESP-IDF, `idf.py menuconfig` shows **DYNAMIXEL servo bus** when the driver is a component of the project
([Installation](installation.md)); the menu is [`_config/Kconfig`](../_config/Kconfig). The options reach the code
only through [`inc/dynamixel_kconfig.hpp`](../inc/dynamixel_kconfig.hpp), never through a driver signature:

```cpp
--8<-- "tests/docs/configuration.cpp:kconfig"
```

| Option | Default | `Config` field |
| --- | --- | --- |
| `CONFIG_DYNAMIXEL_PROTOCOL_1` or `_2` | protocol 2.0 | `protocol` |
| `CONFIG_DYNAMIXEL_BAUD` | 57 600 | `baud` |
| `CONFIG_DYNAMIXEL_REPLY_TIMEOUT_US` | 20 000 | `reply_timeout_us` |
| `CONFIG_DYNAMIXEL_RETRIES` | 1 | `retries` |
| `CONFIG_DYNAMIXEL_ECHO_LOOPBACK` | n | `echo` |

Without Kconfig (plain CMake, other platforms) `KconfigDefaults()` returns `Config{}`. Both cases are host tests.

## Changing the configuration later

`ApplyConfig()` stores a new, valid configuration and applies the baud rate at once when the driver is up. To move
a running bus to a new rate, set the servo first and then the port: `SetServoBaudRate()` writes the servo's Baud Rate
register, `ChangeBusBaud()` moves the UART, waits `baud_change_wait_us` and resynchronises:

```cpp
--8<-- "tests/docs/ports.cpp:change-baud"
```

Next: [Porting](porting.md).
