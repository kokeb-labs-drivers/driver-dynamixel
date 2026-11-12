---
hide:
  - navigation
  - toc
---

<div class="kb-hero" markdown>
<span class="kb-kicker">Driver</span>

# DYNAMIXEL Driver

<p>A C++20 bus-master driver for ROBOTIS DYNAMIXEL servos. It speaks protocol 1.0 and 2.0 over a half-duplex UART,
finds the servos, reads and writes their control tables by name, and runs Sync, Bulk and Fast reads, on any MCU.</p>

[Quickstart](quickstart.md){ .md-button .md-button--primary }
[API guide](api-guide.md){ .md-button }
[Source on GitHub](https://github.com/kokeb-labs-drivers/driver-dynamixel){ .md-button }
</div>

The driver has no platform code. You give it a small class that moves bytes on your UART; it does the rest:
packet framing and byte stuffing, CRC-16 and checksums, finding the reply in the received bytes, retries, the
per-register decoding of 9 families of servos and the per-servo outcome of a Sync or Fast read.

<div class="kb-chip" markdown>
<div class="kb-chip__head">
<span class="kb-chip__kicker">Smart servo</span>
<span class="kb-chip__name">DYNAMIXEL</span>
<span class="kb-chip__vendor">ROBOTIS</span>
</div>

Function
:   Servo with a control table of registers; position, velocity, current and PWM control

Interface
:   Half-duplex TTL or RS-485 UART, 8N1, 9 600 to 4.5 Mbit/s; one line shared by every servo

Protocols
:   2.0 (CRC-16, byte stuffing) and 1.0 (checksum); one per bus

Instructions
:   Ping, Read, Write, Reg Write, Action, Reboot, Factory Reset, Clear, Backup, Sync, Bulk and the Fast reads

Catalogue
:   47 model numbers in 9 control-table families: AX, RX, DX, MX (1.0 and 2.0), XL-320, XL330, XL430, XM, XH, XD, XW
</div>


## What you get

- **One object per bus.** `DYNAMIXEL<YourPort>` takes a reference to your port and does no I/O until
  `EnsureInitialized()`. See [API guide](api-guide.md).
- **Errors you can act on.** Every call returns `Result<T>` with an `Error` such as `Timeout`, `LineFault`,
  `EchoMismatch` or `DeviceError`, and the raw reply is kept for inspection. See [API guide](api-guide.md#handle-errors).
- **Typed registers.** `ReadRegister()`, `WriteRegister()` and `ReadFeedback()` use one name per register for every
  family, with decoding, range checks and units. See [Control tables](control-tables.md).
- **One outcome per servo.** A Sync, Bulk or Fast read fills a `DeviceReply` per ID: a truncated or corrupted
  packet still delivers the servos whose part arrived whole.
- **A safe stop.** `DisableTorque()` is never refused: not before the bus is initialised, not when the input flush fails.
- **Zero-overhead portability.** The port is a CRTP class checked by a concept: five hooks, plus optional baud and
  logging hooks found at compile time. See [Porting](porting.md).
- **No heap, no exceptions, no RTTI.** The host tests build with `-fno-exceptions -fno-rtti`. See
  [Contract](contract.md#memory).

## Where to start

| You want to | Read |
| --- | --- |
| Move a servo from your PC or MCU in a few minutes | [Quickstart](quickstart.md) |
| Wire the bus and set the direction control | [Hardware](hardware.md) |
| Write the UART port for your platform | [Porting](porting.md) |
| Know the packets and the reply search | [Protocol](protocol.md) |
| Know the registers of your servo | [Control tables](control-tables.md) |
| Read, write and read many servos at once | [API guide](api-guide.md) |
| Find out why nothing answers | [Troubleshooting](troubleshooting.md) |
| See what is tested, fuzzed and mutated | [Testing and CI](testing.md) |

## Status

Version 0.1.0. The behaviour is checked against a model of the bus that is written separately from the driver, by
unit tests, fuzzing and mutation testing ([Testing and CI](testing.md)). It has not been run against a servo yet.
Four points the e-Manual does not specify are listed in the [Contract](contract.md#open-points).

<!-- generated: datasheet-card -->
| Document | Revision | Get it |
| --- | --- | --- |
| Protocol specification: DYNAMIXEL Protocol 2.0 (e-Manual) (ROBOTIS e-Manual, DYNAMIXEL Protocol 2.0) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/protocol2/) |
| Protocol specification: DYNAMIXEL Protocol 1.0 (e-Manual) (ROBOTIS e-Manual, DYNAMIXEL Protocol 1.0) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/protocol1/) |
| Protocol specification: CRC Calculation (e-Manual) (ROBOTIS e-Manual, CRC Calculation) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/crc/) |
| Servo manual: XM430-W350 (e-Manual): the X series reference (ROBOTIS e-Manual, XM430-W350) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) |
| Servo manual: AX-12A (e-Manual): the protocol 1.0 reference (ROBOTIS e-Manual, AX-12A) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) |
| Servo manual: XL-320 (e-Manual) (ROBOTIS e-Manual, XL-320) | web page, no revision mark, 2026-11-12 | [vendor page](https://emanual.robotis.com/docs/en/dxl/x/xl320/) |

Written against these revisions; the SHA-256 of each file is in [`docs/datasheets.yml`](datasheets.yml). Where each part
of the driver is in the datasheet: [Datasheet map](datasheet.md).
<!-- /generated -->
